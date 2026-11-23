"""C5VRX by Twotoz and the C5VRX contributors: the fold-free window at one analog gain.

Extends the lane work of C5VRX-3/4 (legacy/c5vrx4/LANE_HANDOVER.md,
ULTRAFINE_TEST.md), IQLaneFusion (research/iq_lane_fusion.py, its exact ADC
bit model is used here) and the LEVEL walks of docs/FINDINGS.md ("Video
follows the gain, not the walking").

Question: with the analog gain left alone on the top row (receiver noise
4.8 cells^2 on the ultrafine lane, 35 ADC codes rms), which carrier levels does
each sign-preserving lane decode? The decoder's token plane is the angle of the
signed nibble pair, so the same table decodes every lane; only the lane's step
and fold point change. `ideal` places every level at radius 4.2 cells on the
ultrafine lane, what a perfect gain control without write cost would do.

Second table: residual receiver DC at radius 4.2 cells on the ultrafine lane,
and the same DC with the token plane computed around the DC point.

Bit-exact U80P64-R1, NTSC deviation 0.7, 12 MHz filter, goggle model of
research/sim.py. C/N is the carrier over the noise after that filter (the
noise the receiver's own meter reads). Synthetic: no lane was switched on a
board, a live lane change and its mixed samples are not modelled.

    python research/edge_window.py            print the tables
    python research/edge_window.py --write    rewrite research/data/edge_window.json
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'research'))
import iq_lane_fusion as F  # noqa: E402
import p64_model  # noqa: E402
import sim  # noqa: E402

OUT = ROOT / 'research/data/edge_window.json'
LUT = p64_model.checked_in_table()
PLANS = (('ultrafine', F.ULTRA), ('fine', F.FINE), ('coarse', F.COARSE))
STANDARD, DEVIATION, FILTER_HZ = 'NTSC', 0.7, 12e6
NOISE_TOP_CELLS2 = 4.8      # receiver.c AGC_NOISE_TOP_DEFAULT: G83 on the ultrafine lane
RADIUS = 4.2
LANE_CNS = (0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 24, 28)
LANE_SEEDS = (11, 12)
DC_CNS = (4, 6, 9, 12, 20)
DC_CELLS = (0.0, 0.3, 0.6, 1.0, 1.5, 2.0, 3.0)
DC_SEEDS = (21, 22)


def decode(raw, lut=LUT):
    return sim._pair_codes(raw, lut, sim.PAIR_BITS, 4)


def parts(seed, cn_db):
    """The case with its carrier and its noise separately, each at unit power."""
    c = sim.make_case(seed, cn_db, STANDARD, deviation=DEVIATION, filter_hz=FILTER_HZ, physical_adc=True, adc_gain=1.0)
    clean = c['clean_adc'] / 16.0
    noise = (c['adc'] - c['clean_adc']) / 16.0
    return c, clean / math.sqrt(np.mean(abs(clean) ** 2)), noise / math.sqrt(np.mean(abs(noise) ** 2))


def score(c, codes, ref):
    m = sim.measure(c, codes, ref)
    return m['picture'], m['sinad']


def token_plane_around(dc_i, dc_q):
    """The U80P64-R1 table with its 16-sector token plane centred on a DC point (cells)."""
    lut = LUT.astype(np.int64) & 0x0fff
    for a in range(1024):
        word = 0
        for j in range(10):
            word |= ((a >> j) & 1) << int(sim.PAIR_BITS[j])
        i = p64_model._nibble(word, (4, 5, 6, 7)) - dc_i
        q = p64_model._nibble(word, (0, 1, 2, 3)) - dc_q
        lut[a] |= (int(math.floor((math.atan2(q, i) + math.pi) * 16 / (2 * math.pi))) % 16) << 12
    return lut.astype(np.uint16)


def lanes():
    noise_codes = math.sqrt(NOISE_TOP_CELLS2) * 16.0
    rows = []
    for cn in LANE_CNS:
        acc = {}
        for seed in LANE_SEEDS:
            c, clean, noise = parts(seed, cn)
            carrier = math.sqrt(10 ** (cn / 10)) * noise_codes
            ref = decode(F.capture(clean * RADIUS * 16.0, F.ULTRA))
            z = clean * carrier + noise * noise_codes
            acc.setdefault('ideal', []).append(score(c, decode(F.capture(z * RADIUS * 16.0 / carrier, F.ULTRA)), ref))
            for name, plan in PLANS:
                acc.setdefault(name, []).append(score(c, decode(F.capture(z, plan)), ref))
        row = dict(cn_db=cn)
        for name, v in acc.items():
            p, s = np.mean(v, axis=0)
            row[name] = dict(picture=round(float(p), 3), sinad_db=round(float(s), 2))
        rows.append(row)
        print(f"lanes C/N {cn:>2}  " + '  '.join(f"{n} {row[n]['picture']:.2f}/{row[n]['sinad_db']:5.1f}"
                                                 for n in ('ideal', 'ultrafine', 'fine', 'coarse')), flush=True)
    return rows


def dc():
    rows = []
    for cn in DC_CNS:
        row = dict(cn_db=cn, dc={})
        fixed = []
        for d in DC_CELLS:
            v = []
            for seed in DC_SEEDS:
                c, clean, noise = parts(seed, cn)
                ref = decode(F.capture(clean * RADIUS * 16.0, F.ULTRA))
                k = d / math.sqrt(2)
                raw = F.capture((clean * RADIUS + noise * RADIUS / math.sqrt(10 ** (cn / 10)) + k * (1 + 1j)) * 16.0, F.ULTRA)
                v.append(score(c, decode(raw), ref))
                if d == 2.0:
                    fixed.append(score(c, decode(raw, token_plane_around(k, k)), ref))
            p, s = np.mean(v, axis=0)
            row['dc'][str(d)] = dict(picture=round(float(p), 3), sinad_db=round(float(s), 2))
        p, s = np.mean(fixed, axis=0)
        row['dc_2.0_token_plane_centred'] = dict(picture=round(float(p), 3), sinad_db=round(float(s), 2))
        rows.append(row)
        print(f"dc C/N {cn:>2}  " + '  '.join(f"{d}:{row['dc'][str(d)]['picture']:.2f}/{row['dc'][str(d)]['sinad_db']:5.1f}"
                                              for d in DC_CELLS) + f"  centred@2.0 {p:.2f}/{s:5.1f}", flush=True)
    return rows


if __name__ == '__main__':
    result = dict(source='research/edge_window.py',
                  evidence='host model only; C/N after the 12 MHz filter; fixed analog gain, no gain write modelled',
                  decoder='U80P64-R1', standard=STANDARD, deviation=DEVIATION, noise_top_cells2=NOISE_TOP_CELLS2,
                  radius_cells=RADIUS, lanes=lanes(), dc=dc())
    if '--write' in sys.argv:
        OUT.write_text(json.dumps(result, indent=1) + '\n')
        print('wrote', OUT)
