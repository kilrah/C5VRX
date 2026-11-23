"""C5VRX by Twotoz and the C5VRX contributors: picture grain on a strong
carrier against the total IQ radius, for the LEVEL quality table.

The measured table (level_quality_grid.py) scores luma SINAD on a textured
picture and saturates at SINAD_FULL (12 dB): from ~15 dB C/N up every radius
from 2.5 to 7 cells reads 96-100, so LEVEL may park a strong carrier anywhere
in that band. The picture noise does depend on it there. On a flat grey field
the bit-exact decoders on the 4-bit ultrafine lane are best at 5.5-6.0 cells:
U50P64-R1 dips at 3.5-4.0 (deviation 0.68, C/N 35 dB: 25.0 dB at 3.5, 28.1
at 5.5), D32-R2 has no loop to average its 32 sectors and falls steeply
under ~5 cells (22.1 dB at 2.5, 24.3 at 4.0, 28.9 at 5.5), while the table
without grain rated 3.0 cells best. This study measures it with the same physical-ADC
model and C/N scale as the grid, so level_quality.py can count it.

Grain SNR = 100 IRE over the rms of the goggle-filtered (5 MHz) output inside
each flat active line around that line's own mean: the visible noise, not the
level error of the decoder's transfer (that is in the grid's SINAD already).
The default decoder D32-R2 and U50P64-R1 (the range choice), NTSC and PAL,
deviation 0.68 and 1.0, five grey levels, two seeds. Host simulation only.

    python research/strong_grain.py research/data/strong_grain.json
"""
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
from scipy import signal as sg

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim  # noqa: E402
import p64_model  # noqa: E402
import u50p64  # noqa: E402
import nl_tracker  # noqa: E402

CNS = [15, 20, 30]                       # C/N per 20 MHz (sim.make_case), the grid's strong rows
RMS = [0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0]
DEVIATIONS = [0.68, 1.0]
STANDARDS = ['NTSC', 'PAL']
LEVELS = [10, 30, 50, 70, 90]           # flat grey, IRE
SEEDS = [4101, 4102]
_tables = None


def line_noise_ire(codes, c, deviation):
    """rms (IRE) of the goggle-filtered output around each flat line's mean."""
    hz = codes.astype(float) / 63.0 * 15e6 - 6e6                  # decoder window -6..+9 MHz
    ire = (hz - sim.PORCH_HZ) / (deviation * sim.DEV_PP / 140)
    y = sg.sosfilt(sim.GOGGLE, ire)
    m = c['region'] == 1
    idx = np.flatnonzero(m)
    starts = idx[np.r_[True, np.diff(idx) > 1]]
    ends = idx[np.r_[np.diff(idx) > 1, True]]
    parts = [y[s + 200:e - 100] for s, e in zip(starts, ends) if e - s > 600]
    return float(np.sqrt(np.mean(np.concatenate([p - p.mean() for p in parts]) ** 2)))


def one(job):
    global _tables
    if _tables is None:
        _tables = {'D32-R2': (nl_tracker.table(), 5), 'U50P64-R1': (u50p64.table(), 4)}
    cn, rms, dev, std, seed = job
    out = []
    for level in LEVELS:
        c = sim.make_case(seed + level, cn, std, dev, filter_hz=12e6, fields=1, physical_adc=True,
                          adc_gain=rms, flat_ire=level)
        for name, (t, bits) in _tables.items():
            out.append((name, line_noise_ire(sim._pair_codes(c['raw'], t, sim.PAIR_BITS, bits), c, dev)))
    res = {}
    for name in _tables:
        v = np.array([r for n, r in out if n == name])
        res[name] = float(np.sqrt(np.mean(v ** 2)))
    return dict(cn=cn, rms=rms, deviation=dev, std=std, seed=seed, noise_ire=res)


def main():
    jobs = [(cn, r, d, st, s) for cn in CNS for r in RMS for d in DEVIATIONS for st in STANDARDS for s in SEEDS]
    t0 = time.time()
    rows = []
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for i, r in enumerate(pool.imap_unordered(one, jobs, chunksize=2)):
            rows.append(r)
            if i % 40 == 0:
                print(i, len(jobs), round(time.time() - t0), flush=True)
    rows.sort(key=lambda r: (r['cn'], r['rms'], r['deviation'], r['std'], r['seed']))
    Path(sys.argv[1]).write_text(json.dumps({
        'evidence': 'host simulation only (sim.py physical-ADC model, 12 MHz filter, flat grey field, bit-exact '
                    'D32-R2 and U50P64-R1); no board or goggle result',
        'metric': 'rms IRE of the 5 MHz goggle-filtered output around each flat line mean',
        'rows': rows}, indent=0) + '\n')
    print('done', len(rows), round(time.time() - t0))


if __name__ == '__main__':
    main()
