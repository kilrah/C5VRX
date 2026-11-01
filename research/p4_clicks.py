"""C5VRX by Twotoz and the C5VRX contributors: U80P64-R1 clicks (dropouts)
by brightness and length, and a single-sample declicker for the P4 display's
20 MS/s codes (black streaks in highlights, docs/P4LINK.md). Host simulation
only (research/sim.py chain); no picture or range claim on hardware.

usage: python p4_clicks.py [C/N dB ...]
"""
import sys
import numpy as np

import p64_model
import sim

P64 = p64_model.checked_in_table()


def codes(raw):
    """U80P64-R1 bit-exact (two LUT16 lookups a pair), unique 20 MS/s codes."""
    return sim._pair_codes(raw, P64, sim.PAIR_BITS, 4)[::2].astype(np.int16)


def declick(y, thr):
    """A code thr or more below (or above) the mean of its two neighbours
    takes that mean: one 50 ns sample, which 5 MHz video cannot produce."""
    y = y.copy()
    m = (y[:-2] + y[2:] + 1) >> 1
    d = y[1:-1] - m
    hit = (d <= -thr) | (d >= thr)
    y[1:-1][hit] = m[hit]
    return y


def runs(mask):
    d = np.diff(np.r_[0, mask.astype(np.int8), 0])
    return np.flatnonzero(d == -1) - np.flatnonzero(d == 1)


def sinad(c, y20):
    """sim.measure's luma SINAD; codes back to [D,D] at 40 MS/s."""
    y = np.repeat(np.clip(y20, 0, 63).astype(np.uint8), 2)
    yc = np.repeat(np.clip(codes(c['clean']), 0, 63).astype(np.uint8), 2)
    return sim.measure(c, y[:len(c['raw'])], yc[:len(c['raw'])])['sinad']


def clicks(y, yc, pic, drop=12):
    bad = pic & ((yc - y >= drop) | (y - yc >= drop))
    hi = pic & (yc >= 38)
    return (bad & hi).sum() / max(hi.sum(), 1) * 1e4, (bad & pic & ~hi).sum() / max((pic & ~hi).sum(), 1) * 1e4


if __name__ == '__main__':
    for cnr in map(float, sys.argv[1:] or ['8', '10', '12', '14']):
        c = sim.make_case(3, cnr, 'PAL', deviation=1.24)
        y, yc = codes(c['raw']), codes(c['clean'])
        pic = c['region'][::2] == 1
        neg = pic & (yc - y >= 12)
        pos = pic & (y - yc >= 12)
        print(f'C/N {cnr:4.1f}: down {neg.sum()} up {pos.sum()} run lengths down 1..6: '
              f'{np.bincount(np.minimum(runs(neg), 6), minlength=7)[1:].tolist()}')
        for thr in (0, 16, 12, 10, 8):
            z = y if thr == 0 else declick(y, thr)
            zc = yc if thr == 0 else declick(yc, thr)
            ch, cl = clicks(z, yc, pic)
            changed_clean = int((zc != yc)[pic].sum())
            print(f'   declick thr={thr:2d}: clicks/10k bright {ch:6.1f} dark {cl:6.1f}  '
                  f'SINAD {sinad(c, z):5.2f} dB  clean samples changed {changed_clean}')
