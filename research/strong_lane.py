"""C5VRX by Twotoz and the C5VRX contributors: a strong carrier on the gain floor, per lane.

Extends research/edge_window.py (same exact ADC bit model of
research/iq_lane_fusion.py, same decoder and goggle model) and the STRONG LANE
policy of main/strong_lane.h (2026-11-13, an external LNA close in).

Question: with the analog gain on its lowest row and nothing left to lower,
what does each sign-preserving lane decode as the carrier grows past the
ultrafine window? Radius in ultrafine cells (16 ADC codes): the ultrafine lane
folds above 8 cells, the fine lane above 16, the coarse lane spans the whole
ADC (32 cells, +-512 codes). `policy` is the lane STRONG LANE settles on:
the finest lane whose window holds the radius.

Bit-exact U80P64-R1, NTSC deviation 0.7, 12 MHz filter. C/N 30 dB (a strong
carrier; with an LNA the noise on the floor row is small). Synthetic: an ideal
constant-envelope carrier; no compression of the LNA or the C5 input before
the ADC, no route-write transient, nothing switched on a board.

    python research/strong_lane.py            print the table
    python research/strong_lane.py --write    rewrite research/data/strong_lane.json
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'research'))
import edge_window as E  # noqa: E402
import iq_lane_fusion as F  # noqa: E402

OUT = ROOT / 'research/data/strong_lane.json'
CN_DB = 30
RADII = (4, 6, 8, 9, 10, 12, 14, 16, 18, 20, 24, 28, 31, 34, 40)
SEEDS = (31, 32)
WINDOW = dict(ultrafine=8.0, fine=16.0, coarse=32.0)


def policy_lane(radius):
    for name in ('ultrafine', 'fine', 'coarse'):
        if radius < WINDOW[name]:   # on the edge the rails are reached: the firmware reads a fold
            return name
    return 'coarse'


def table():
    rows = []
    for r in RADII:
        acc = {}
        for seed in SEEDS:
            c, clean, noise = E.parts(seed, CN_DB)
            ref = E.decode(F.capture(clean * E.RADIUS * 16.0, F.ULTRA))
            z = (clean + noise / math.sqrt(10 ** (CN_DB / 10))) * r * 16.0
            for name, plan in E.PLANS:
                acc.setdefault(name, []).append(E.score(c, E.decode(F.capture(z, plan)), ref))
        row = dict(radius_cells=r, db_over_ultrafine_window=round(20 * math.log10(r / 8.0), 1), policy=policy_lane(r))
        for name, v in acc.items():
            p, s = np.mean(v, axis=0)
            row[name] = dict(picture=round(float(p), 3), sinad_db=round(float(s), 2))
        rows.append(row)
        print(f"radius {r:>2} cells ({row['db_over_ultrafine_window']:+5.1f} dB)  " +
              '  '.join(f"{n} {row[n]['picture']:.2f}/{row[n]['sinad_db']:5.1f}" for n in ('ultrafine', 'fine', 'coarse')) +
              f"  policy={row['policy']} {row[row['policy']]['picture']:.2f}/{row[row['policy']]['sinad_db']:5.1f}", flush=True)
    return rows


if __name__ == '__main__':
    result = dict(source='research/strong_lane.py',
                  evidence='host model only; ideal carrier, no analog compression, no route-write transient, no board',
                  decoder='U80P64-R1', standard=E.STANDARD, deviation=E.DEVIATION, cn_db=CN_DB, rows=table())
    if '--write' in sys.argv:
        OUT.write_text(json.dumps(result, indent=1) + '\n')
        print('wrote', OUT)
