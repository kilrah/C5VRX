"""C5VRX by Twotoz and the C5VRX contributors: where AUTO OUTPUT puts the sync tip.

Extends AUTO OUTPUT (main/output_auto.h, PR #301) and research/output_level.py
(issue #200). The affine output word puts the measured sync tip on a fixed DAC
code and saturates everything under code 0. With the tip on code 3 the receiver
noise and the FM clicks under the tip are cut one-sidedly at the range edge:
the low-passed sync tip rises towards the porch and a goggle's sync slicer
loses lines the 1.0x word still shows. This sweep puts the tip on several codes
(peak white stays on code 60) and counts the lines the goggle model keeps.

Bit-exact decoders (U80P64-R1, U50P64-R1), the firmware's integer output word
(output_level.h output_entry), the resistor DAC and the goggle model of
research/sim.py (worse of the ideal and the peak-clamp slicer). The word is the
one a strong signal would have set (clean tip and peak white), held at the
edge, as the policy does. Synthetic: no goggle, voltage or range is measured.

    python research/output_tip_headroom.py            print the table
    python research/output_tip_headroom.py --write    rewrite research/data/output_tip_headroom.json
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
import u50p64  # noqa: E402

OUT = ROOT / 'research/data/output_tip_headroom.json'
TIPS = (3, 6, 8, 10, 12, 14, 16, 20)
WHITE_CODE = 60
SYNC_CODES = 18
CNS = (0, 1, 2, 3, 4, 6)
SEEDS = (4101, 4102, 4103, 4104)
CASES = (('NTSC', 0.7), ('PAL', 1.0))
RADIUS = 4.2


def affine(codes, gain_q3, src_tip, out_tip):
    """output_level.h output_entry for an affine word, with the tip on out_tip."""
    x8 = 8 * out_tip + gain_q3 * (codes.astype(np.int64) - src_tip)
    return np.clip(np.floor_divide(x8 + 4, 8), 0, 63).astype(np.uint8)


def word(tip, depth, white, out_tip):
    """output_auto.h output_auto_word: gain (Q3) for the span above out_tip."""
    span = max(white - tip, depth * 3.5)
    g = int(8 * (WHITE_CODE - out_tip) / span)
    return max(8, min(32, g, int(8 * SYNC_CODES / depth)))


def run():
    luts = {'U80P64-R1': p64_model.checked_in_table(), 'U50P64-R1': u50p64.table()}
    rows = []
    for standard, deviation in CASES:
        for cn in CNS:
            acc = {}
            for seed in SEEDS:
                c = sim.make_case(seed, cn, standard, deviation=deviation, filter_hz=12e6, physical_adc=True, adc_gain=1.0)
                clean = c['clean_adc'] / 16.0
                noise = (c['adc'] - c['clean_adc']) / 16.0
                clean /= math.sqrt(np.mean(abs(clean) ** 2))
                noise /= math.sqrt(np.mean(abs(noise) ** 2))
                raw = F.capture((clean * RADIUS + noise * RADIUS / math.sqrt(10 ** (cn / 10))) * 16.0, F.ULTRA)
                ref_raw = F.capture(clean * RADIUS * 16.0, F.ULTRA)
                for name, lut in luts.items():
                    codes = sim._pair_codes(raw, lut, sim.PAIR_BITS, 4)
                    ref = sim._pair_codes(ref_raw, lut, sim.PAIR_BITS, 4)
                    sync = ref[np.flatnonzero(c['region'] == 2)[200:-200]]
                    tip = int(round(float(np.percentile(sync, 50))))
                    porch = float(np.median(ref[np.flatnonzero(c['region'] == 0)[2000:-2000]]))
                    white = float(np.percentile(ref[5000:-5000], 99.5))
                    acc.setdefault((name, '1.0x'), []).append(sim.measure(c, codes, ref)['picture'])
                    for out_tip in TIPS:
                        g = word(tip, max(porch - tip, 1.0), white, out_tip)
                        m = sim.measure(c, affine(codes, g, tip, out_tip), affine(ref, g, tip, out_tip))
                        acc.setdefault((name, out_tip), []).append(m['picture'])
                        acc.setdefault((name, f'gain{out_tip}'), []).append(g / 8.0)
            for name in luts:
                rows.append(dict(standard=standard, deviation=deviation, cn_db=cn, decoder=name,
                                 picture_1x=round(float(np.mean(acc[(name, '1.0x')])), 3),
                                 picture={str(t): round(float(np.mean(acc[(name, t)])), 3) for t in TIPS},
                                 gain={str(t): round(float(np.mean(acc[(name, f'gain{t}')])), 2) for t in TIPS}))
                r = rows[-1]
                print(f"{standard} dev {deviation} {name:10s} C/N {cn:>2}  1.0x {r['picture_1x']:.2f}  " +
                      '  '.join(f"tip{t}:{r['picture'][str(t)]:.2f}" for t in TIPS), flush=True)
    return dict(source='research/output_tip_headroom.py', evidence='host model only; C/N after the 12 MHz filter',
                tips=list(TIPS), white_code=WHITE_CODE, seeds=list(SEEDS), radius_cells=RADIUS, rows=rows)


if __name__ == '__main__':
    result = run()
    if '--write' in sys.argv:
        OUT.write_text(json.dumps(result, indent=1) + '\n')
        print('wrote', OUT)
