"""C5VRX by Twotoz and the C5VRX contributors: the strongest offline (PC)
FM demodulator for 4-bit ultrafine IQ snapshots, as an upper bound for what
the C5's IQ can give. Not a firmware design: it needs floating point, the
whole snapshot and a backward pass.

Steps, each its own decoder for comparison:
  PC0  float discriminator on the cell centres (DC removed): the "IDEAL" of
       tools/frame_puzzle.py.
  PC1  + DC and I/Q gain/skew removed by an ellipse fit, and the phase taken
       as the middle of the arc of the carrier circle inside each 4-bit cell
       (a constant-envelope carrier lies on that circle), not the cell
       centre's angle.
  PC2  + a phase/frequency Kalman filter with a Rauch-Tung-Striebel backward
       pass, each sample weighted by the angular width of its arc (wide arcs
       near the axes count less). Process noise sized for composite video.
Output: DAC codes on the receiver's -6..+9 MHz scale (float).
"""
import math

import numpy as np
from numba import njit

FS = 40e6


def unpack(raw):
    b = raw.astype(np.int64)
    i = ((b >> 4) ^ 8) - 8
    q = ((b & 15) ^ 8) - 8
    return i, q


def to_codes(hz):
    return (hz + 6e6) / 15e6 * 63.0


def pc0(raw):
    i, q = unpack(raw)
    z = (i + .5) + 1j * (q + .5)
    z = z - z.mean()
    hz = np.angle(z[1:] * np.conj(z[:-1])) * FS / (2 * np.pi)
    return to_codes(np.r_[hz[0], hz])


def ellipse(i, q):
    """Centre, I/Q gain ratio, skew and radius of the cloud of cell centres."""
    x, y = i + .5, q + .5
    A = np.c_[x * x, x * y, y * y, x, y]
    c = np.linalg.lstsq(A, np.ones(len(x)), rcond=None)[0]
    a, b, cc, d, e = c
    M = np.array([[2 * a, b], [b, 2 * cc]])
    x0, y0 = np.linalg.solve(M, [-d, -e])
    xs, ys = x - x0, y - y0
    gi, gq = np.sqrt(np.mean(xs * xs)), np.sqrt(np.mean(ys * ys))
    rho = np.mean(xs * ys) / (gi * gq)
    return x0, y0, gi, gq, rho


def arc_phase(raw, steps=4096):
    """Per sample: the middle angle and half-width of the piece of the
    carrier circle that falls in its raw 4-bit cell. The circle is the fitted
    ellipse (DC, I/Q gain and skew) traced in fine steps and mapped back to
    raw coordinates; a cell the circle does not cross takes the angle of its
    centre with a wide uncertainty."""
    i, q = unpack(raw)
    x0, y0, gi, gq, rho = ellipse(i.astype(float), q.astype(float))
    s = math.sqrt(max(1e-9, 1.0 - rho * rho))
    th = (np.arange(steps) + .5) * 2 * np.pi / steps - np.pi
    # corrected circle radius sqrt(2) (unit rms per axis) -> raw coordinates
    xc = np.sqrt(2.0) * np.cos(th)
    yc = np.sqrt(2.0) * np.sin(th)
    xr = x0 + gi * xc
    yr = y0 + gq * (rho * xc + s * yc)
    ci = np.floor(xr).astype(int)
    cq = np.floor(yr).astype(int)
    lut_mid = {}
    lut_half = {}
    for key in set(zip(ci.tolist(), cq.tolist())):
        m = (ci == key[0]) & (cq == key[1])
        a = th[m]
        ref = a[0]
        d = (a - ref + np.pi) % (2 * np.pi) - np.pi
        lut_mid[key] = ref + 0.5 * (d.min() + d.max())
        lut_half[key] = max(0.5 * (d.max() - d.min()), np.pi / steps)
    mid = np.empty(len(i))
    half = np.empty(len(i))
    for k, key in enumerate(zip(i.tolist(), q.tolist())):
        if key in lut_mid:
            mid[k], half[k] = lut_mid[key], lut_half[key]
        else:
            x = (key[0] + .5 - x0) / gi
            y = (key[1] + .5 - y0) / gq
            mid[k] = math.atan2((y - rho * x) / s, x)
            half[k] = 0.6
    return mid, half


def pc1(raw):
    mid, _ = arc_phase(raw)
    d = (np.diff(mid) + np.pi) % (2 * np.pi) - np.pi
    return to_codes(np.r_[d[0], d] * FS / (2 * np.pi))


@njit(cache=True)
def _kalman(z, rvar, q_f):
    """State (phase, frequency per sample). Wrapped innovations; RTS smoother."""
    n = len(z)
    xs = np.zeros((n, 2))
    Ps = np.zeros((n, 2, 2))
    xp = np.zeros((n, 2))
    Pp = np.zeros((n, 2, 2))
    x0 = z[0]
    x1 = 0.0
    P00, P01, P11 = 1.0, 0.0, 1.0
    for k in range(n):
        # predict
        a0 = x0 + x1
        a1 = x1
        p00 = P00 + 2 * P01 + P11 + 1e-6
        p01 = P01 + P11
        p11 = P11 + q_f
        xp[k, 0], xp[k, 1] = a0, a1
        Pp[k, 0, 0], Pp[k, 0, 1], Pp[k, 1, 0], Pp[k, 1, 1] = p00, p01, p01, p11
        # update with the wrapped innovation
        inn = (z[k] - a0 + math.pi) % (2 * math.pi) - math.pi
        s = p00 + rvar[k]
        k0 = p00 / s
        k1 = p01 / s
        x0 = a0 + k0 * inn
        x1 = a1 + k1 * inn
        P00 = (1 - k0) * p00
        P01 = (1 - k0) * p01
        P11 = p11 - k1 * p01
        xs[k, 0], xs[k, 1] = x0, x1
        Ps[k, 0, 0], Ps[k, 0, 1], Ps[k, 1, 0], Ps[k, 1, 1] = P00, P01, P01, P11
    # RTS backward pass
    for k in range(n - 2, -1, -1):
        # C = P_k F^T Pp_{k+1}^-1, F = [[1,1],[0,1]]
        a00 = Ps[k, 0, 0] + Ps[k, 0, 1]
        a01 = Ps[k, 0, 1]
        a10 = Ps[k, 1, 0] + Ps[k, 1, 1]
        a11 = Ps[k, 1, 1]
        b00, b01, b10, b11 = Pp[k + 1, 0, 0], Pp[k + 1, 0, 1], Pp[k + 1, 1, 0], Pp[k + 1, 1, 1]
        det = b00 * b11 - b01 * b10
        i00, i01, i10, i11 = b11 / det, -b01 / det, -b10 / det, b00 / det
        c00 = a00 * i00 + a01 * i10
        c01 = a00 * i01 + a01 * i11
        c10 = a10 * i00 + a11 * i10
        c11 = a10 * i01 + a11 * i11
        d0 = xs[k + 1, 0] - xp[k + 1, 0]
        d1 = xs[k + 1, 1] - xp[k + 1, 1]
        xs[k, 0] += c00 * d0 + c01 * d1
        xs[k, 1] += c10 * d0 + c11 * d1
    return xs[:, 1]


def pc2(raw, q_mhz=1.2):
    """q_mhz: rms frequency change per sample the model allows (composite
    video moves up to ~7 MHz within ~100 ns at an edge)."""
    mid, half = arc_phase(raw)
    rvar = (half ** 2) / 3.0 + 1e-4          # uniform over the arc
    q_f = (2 * np.pi * q_mhz * 1e6 / FS) ** 2
    f = _kalman(np.unwrap(mid), rvar, q_f)
    return to_codes(f * FS / (2 * np.pi))
