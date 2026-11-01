"""C5VRX by Twotoz and the C5VRX contributors: U80P64-R1 dropouts on real C5
IQ snapshots (console I, tools/capture_iq_snapshot.py; many of them stacked
in one .npy), for the P4 display's black streaks in highlights.

A dropout: a 20 MS/s code at least DROP below the median of the 13 codes
around it, in picture (the line's codes between sync and sync). Reports
their lengths, the picture level around them, the IQ radius and ADC rails
(I or Q at -8/+7) in their 2 us, against the same over all picture samples.

usage: python p4_board_clicks.py snaps.npy [gains.npy]
"""
import sys
import numpy as np
from scipy.ndimage import median_filter

import p64_model
import sim

P64 = p64_model.checked_in_table()
DROP = 10


def codes(raw):
    return sim._pair_codes(raw, P64, sim.PAIR_BITS, 4)[::2].astype(np.int16)


def iq(raw):
    i = (raw >> 4).astype(np.int16); q = (raw & 15).astype(np.int16)
    i[i > 7] -= 16; q[q > 7] -= 16
    return i, q


def main():
    snaps = np.load(sys.argv[1])
    gains = np.load(sys.argv[2]) if len(sys.argv) > 2 else np.zeros(len(snaps), int)
    lens, levels, rad_ev, rail_ev, rad_all, rail_all, lvl_all = [], [], [], [], [], [], []
    per_gain = {}
    examples = []
    for n, raw in enumerate(snaps):
        y = codes(raw)[2:]
        i, q = iq(raw)
        r = np.hypot(i + .5, q + .5)
        rail = ((i == -8) | (i == 7) | (q == -8) | (q == 7))
        base = median_filter(y, 13, mode='nearest')
        sync = base < np.percentile(base, 8) + 3                 # sync tips and broad pulses
        pic = ~np.convolve(sync, np.ones(240), 'same').astype(bool) # 6 us clear of any sync
        dip = pic & (base - y >= DROP)
        d = np.diff(np.r_[0, dip.astype(np.int8), 0])
        st, en = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
        g = per_gain.setdefault(int(gains[n]), [0, 0])
        g[0] += len(st); g[1] += pic.sum()
        for a, b in zip(st, en):
            lens.append(b - a)
            levels.append(base[a])
            w = slice(2 * max(a - 20, 0), 2 * min(b + 20, len(y)))   # 40 MS/s, +-1 us
            rad_ev.append(r[w].mean()); rail_ev.append(rail[w].mean())
            if len(examples) < 6 and base[a] >= np.percentile(base[pic], 85):
                examples.append((n, int(gains[n]), y[a - 6:b + 6].tolist(), np.round(r[2 * a - 12:2 * b + 12:2], 1).tolist()))
        p40 = np.zeros(len(r), bool); p40[:2 * len(pic)] = np.repeat(pic, 2)
        rad_all.append(r[p40].mean()); rail_all.append(rail[p40].mean())
        lvl_all.append(base[pic])
    lens, levels = np.array(lens), np.array(levels)
    lvl_all = np.concatenate(lvl_all)
    print(f'{len(snaps)} snapshots, {len(lens)} dropouts >= {DROP} codes in picture')
    print('lengths 1..7,8+ (20 MS/s codes):', np.bincount(np.minimum(lens, 8), minlength=9)[1:].tolist())
    edges = np.percentile(lvl_all, [0, 25, 50, 75, 90, 100])
    for lo, hi in zip(edges[:-1], edges[1:]):
        m_all = (lvl_all >= lo) & (lvl_all < hi); m = (levels >= lo) & (levels < hi)
        print(f'  picture level {lo:4.0f}-{hi:4.0f} codes: {m.sum() / max(m_all.sum(), 1) * 1e4:7.1f} dropouts / 10k samples')
    print(f'IQ radius: around dropouts {np.mean(rad_ev):.2f}, all picture {np.mean(rad_all):.2f}; '
          f'ADC at a rail: around dropouts {np.mean(rail_ev) * 100:.1f} %, all picture {np.mean(rail_all) * 100:.1f} %')
    for gn, (k, m) in sorted(per_gain.items()):
        print(f'  gain {gn}: {k} dropouts in {m} picture samples ({k / max(m, 1) * 1e4:.1f} / 10k)')
    for e in examples:
        print('example snap', e[0], 'gain', e[1], 'codes', e[2], 'radius', e[3])


if __name__ == '__main__':
    main()
