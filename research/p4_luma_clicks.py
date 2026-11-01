"""C5VRX by Twotoz and the C5VRX contributors: what U80P64-R1 clicks leave
in the P4 display's luma (p4/main/cvbs.c row_out, luma.S), and candidate
filters. Host simulation only (research/sim.py chain).

P4 path: 20 MS/s codes -> sums of pairs (10 MS/s) -> 3-tap kernel -> clamp
at black/white (blanking .. blanking + 2.33 x sync depth) -> 0..255.
Counts pixels at least 40/255 darker than the noise-free picture, in bright
(>= 160) and darker areas, and the luma error (dB below the picture) against
the same chain's noise-free output (blur not counted; the sharp-reference
comparison is in docs/P4LINK.md). "pre [1 2 1]" then pair sums is the
[1 3 3 1] / 4 history of p4/main/fill.S (2026-11-01).

usage: python p4_luma_clicks.py [C/N dB ...]
"""
import sys
import numpy as np

import sim
from p4_clicks import codes

KERNELS = {
    'pairs [0 4 0] (mono lines)': ('pair', (0, 4, 0)),
    'pairs [1 2 1] (colour lines)': ('pair', (1, 2, 1)),
    'chosen, colour lines: pre [1 2 1] + [1 2 1]': ('pre121', (1, 2, 1)),
    '20M [1 1 1 1] /2 at 10M (box 4)': ('box4', (0, 4, 0)),
    '20M [1 2 2 2 1] /4 at 10M': ('k5', (0, 4, 0)),
    'chosen, other lines: pre [1 2 1] + [1 6 1]': ('pre121', (1, 6, 1)),
}


def p4_luma(y20, mode, k, blank, depth, phase=0):
    y20 = y20.astype(np.float64)
    if mode == 'pre121':
        y20 = np.convolve(y20, [.25, .5, .25], 'same') * 1.0
    if mode == 'box4':
        y20 = np.convolve(y20, [.25, .25, .25, .25], 'same') * 2 / 2
    if mode == 'k5':
        y20 = np.convolve(y20, np.array([1, 2, 2, 2, 1]) / 8, 'same')
    s = y20[phase::2][:len(y20) // 2 - 1]
    s = s + y20[phase + 1::2][:len(s)]                   # pair sums, 10 MS/s (two codes)
    k = np.array(k, float) / sum(k)
    s = np.convolve(s, k[::-1], 'same')
    black, white = 2 * blank, 2 * (blank + 2.333 * depth)
    return np.clip((s - black) / (white - black) * 255, 0, 255)


if __name__ == '__main__':
    for cnr in map(float, sys.argv[1:] or ['8', '10', '12']):
        c = sim.make_case(3, cnr, 'PAL', deviation=1.24)
        y, yc = codes(c['raw']), codes(c['clean'])
        pic = (c['region'][::2] == 1)[0:len(y) - 2:2]
        sync, blank = np.percentile(yc[c['region'][::2] == 2], 50), np.percentile(yc[c['region'][::2] == 0], 50)
        print(f'C/N {cnr:4.1f} dB (sync {sync:.0f}, blank {blank:.0f} codes)')
        for name, (mode, k) in KERNELS.items():
            res = []
            for ph in (0, 1):
                a = p4_luma(y, mode, k, blank, blank - sync, ph)
                b = p4_luma(yc, mode, k, blank, blank - sync, ph)
                m = pic[:len(a)]
                bright = m & (b >= 160)
                dark = (b - a >= 40)
                err = 10 * np.log10(np.var(b[m]) / np.mean((a - b)[m] ** 2))
                res.append(((dark & bright).sum() / bright.sum() * 1e4, (dark & m & ~bright).sum() / (m & ~bright).sum() * 1e4, err))
            r = np.mean(res, 0)
            print(f'   {name:34s} dark clicks/10k px: bright {r[0]:6.1f}  other {r[1]:6.1f}   luma SNR {r[2]:5.2f} dB')
