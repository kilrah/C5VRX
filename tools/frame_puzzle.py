#!/usr/bin/env python3
"""Puzzle several IQ frame sweeps (tools/frame_sweep.py) into one aligned frame.

C5VRX by Twotoz and the C5VRX contributors. Bench tool; extends
tools/frame_sweep.py. Each sweep is first placed on its own (boundary times,
fitted line period, per-snapshot sync correction). The sweeps are then
aligned to each other on the picture itself (masked 2-D cross-correlation of
the low-passed frames: whole lines and a fraction of a line), and every
snapshot is refined against the mean of all of them (+-1 line,
+-3 us); a snapshot that fits nowhere (damaged, folded) is left out.

Outputs in OUT:
  frame_single.png  every pixel from ONE snapshot (the best fitting): the
                    real picture noise, as the receiver decodes it;
  frame_mean.png    the mean over the snapshots covering a pixel: less
                    noise, shows how well the pieces line up;
  puzzle.json       fill, per-sweep offsets and the refinement statistics.

    python tools/frame_puzzle.py OUT sweep_dir1 sweep_dir2 [...] [--decoder U50P64-R1]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import signal as sg

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'research'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import frame_sweep as fs  # noqa: E402

FS = fs.FS
SNAP = fs.SNAP
COLS = 2542            # samples per NTSC line at 40 MS/s (63.556 us)
LP = sg.butter(2, 1.0e6, fs=FS, output='sos')


_NL = {}


def decode(raw, decoder):
    import sim
    if decoder == 'IDEAL':
        b = raw.astype(int)
        z = ((((b >> 4) ^ 8) - 8) + .5) + 1j * ((((b & 15) ^ 8) - 8) + .5)
        z = z - z.mean()
        hz = np.angle(z[1:] * np.conj(z[:-1])) * FS / (2 * np.pi)
        return np.r_[hz[0], hz] / 15e6 * 63 + 6e6 / 15e6 * 63
    if decoder.startswith('PC'):     # research/pc_demod.py: offline upper bound (PC0, PC1, PC2[:q_mhz])
        import pc_demod
        name, _, arg = decoder.partition(':')
        if name == 'PC2':
            return pc_demod.pc2(raw, float(arg) if arg else 1.2)
        return getattr(pc_demod, name.lower())(raw)
    if decoder.startswith('NL:'):    # research/nl_tracker.py: NL:kp_hi,kp_lo,knee_deg[,pred]
        import nl_tracker
        p = decoder[3:].split(',')
        key = tuple(p)
        if key not in _NL:
            _NL[key] = nl_tracker.build_lut(float(p[0]), float(p[1]), float(p[2]), out=p[3] if len(p) > 3 else 'committed')
        return sim._pair_codes(raw, _NL[key], sim.PAIR_BITS, 5).astype(float)
    if decoder == 'T32K70':      # research candidate: 32-token closed-form tracker, loop gain 0.70
        import kp_morph
        return sim._pair_codes(raw, kp_morph.build_lut(0.70), sim.PAIR_BITS, 5).astype(float)
    if decoder == 'U50P64-R1':
        import u50p64
        t, bits = u50p64.table(), 4
    elif decoder == 'U80P64-R1':
        import p64_model
        t, bits = p64_model.repaired_table(), 4
    else:
        t, bits = sim.u85q_lut(), 5
    return sim._pair_codes(raw, t, sim.PAIR_BITS, bits).astype(float)


def place_sweep(src, decoder, lines):
    """Per snapshot: the video, its low-pass and the frame position of sample 0 in lines."""
    import sim
    meta = json.loads((src / 'sweep.json').read_text())
    snaps = meta['snapshots']
    out = []
    edges = {}
    for r in snaps:
        raw = np.frombuffer((src / f'k{r["k"]:03d}.bin').read_bytes(), np.uint8)
        b = raw.astype(int)
        i, q = ((b >> 4) ^ 8) - 8, ((b & 15) ^ 8) - 8
        folds = float(np.mean((i == 7) | (i == -8) | (q == 7) | (q == -8)))
        codes = decode(raw, decoder)
        v = sg.sosfilt(sim.GOGGLE, codes)
        e = fs.hsync_edges(codes)[0]
        # keep only edges that sit a whole number of lines (1-3) from another one
        if len(e) > 1:
            d = np.abs(e[:, None] - e[None, :]) / FS * 1e6
            near = np.zeros(len(e), bool)
            for m in (1, 2, 3):
                near |= np.any(np.abs(d - m * 63.5556) < 0.3, axis=1)
            e = e[near]
        else:
            e = e[:0]
        edges[r['k']] = e
        out.append(dict(k=r['k'], t=r['t_us'] - snaps[0]['t_us'], v=v, lp=sg.sosfiltfilt(LP, v), folds=folds, codes=codes))
    snap_us = SNAP / FS * 1e6
    te = np.concatenate([s['t'] - snap_us + edges[s['k']] / FS * 1e6 for s in out if len(edges[s['k']])])
    span = max(float(np.ptp(te)), 1.0)
    h, step = 63.5556, 63.5556 * 300e-6
    grid = np.arange(-1.0, 1.0001, 1.0 / 600)
    while True:
        cand = h + grid * step
        h = float(cand[int(np.argmax([np.abs(np.mean(np.exp(2j * np.pi * te / c))) for c in cand]))])
        if step * len(grid) / 2 < 63.5556 * 0.02 / span:
            break
        step /= 100
    phase = float(np.angle(np.mean(np.exp(2j * np.pi * te / h)))) / (2 * np.pi) * h
    coh = float(np.abs(np.mean(np.exp(2j * np.pi * te / h))))
    for s in out:
        e = edges[s['k']]
        s['own_sync'] = len(e) > 0
        s['e'] = e
        s['Y'], s['U'], s['V'] = color_decode(s['codes'], e)
        d = 0.0
        if len(e):
            tt = s['t'] - snap_us + e / FS * 1e6 - phase
            d = float(np.median(np.mod(tt + h / 2, h) - h / 2))
        s['x0'] = np.mod((s['t'] - snap_us - phase - d) / h, lines)   # frame position of sample 0, lines
        s['h'] = h
    return out, dict(src=str(src), line_us=h, sync_coherence=round(coh, 3), snapshots=len(out))


FSC = 315e6 / 88          # NTSC colour subcarrier, Hz
LUMA = sg.butter(4, 2.8e6, fs=FS, output='sos')
CHROMA = sg.butter(2, 1.0e6, fs=FS, output='sos')


def color_decode(codes, edges):
    """NTSC Y/U/V (IRE) of a snapshot, line by line from each line's own
    sync and colour burst, like a TV: levels from the sync tip and the
    blanking under the burst, hue from the burst phase (burst on -U),
    saturation scaled by the burst (40 IRE p-p). Samples before the first
    sync have no burst in the snapshot: luma only, U = V = NaN."""
    n = len(codes)
    Y = np.full(n, np.nan)
    U = np.full(n, np.nan)
    V = np.full(n, np.nan)
    bounds = list(edges) + [n]
    for m, e in enumerate(edges):
        a, b = int(np.ceil(e)), int(min(bounds[m + 1], e + COLS, n))
        b0, b1 = a + 232, a + 292          # 5.8-7.3 us: inside the burst
        if b1 >= b or b - a < 400:
            continue
        seg = codes[a:b]
        sync = float(np.mean(codes[a + 40:a + 150]))
        blank = float(np.mean(codes[b0:b1]))
        if blank - sync < 2:
            continue
        ire = (seg - blank) / (blank - sync) * 40.0
        j = np.arange(len(seg)) + (a - e)
        z = sg.sosfiltfilt(CHROMA, ire * np.exp(-2j * np.pi * FSC * j / FS))
        zb = np.mean(z[b0 - a:b1 - a])
        amp = abs(zb)
        Y[a:b] = sg.sosfiltfilt(LUMA, ire)
        if amp < 2.0:                      # no burst: black and white
            U[a:b] = 0.0
            V[a:b] = 0.0
            continue
        w = z * (-np.conj(zb) / amp)
        U[a:b] = w.real / amp * 20.0
        V[a:b] = -w.imag / amp * 20.0
    return Y, U, V


def to_rgb(Y, U, V):
    """IRE (NTSC, 7.5 setup) to RGB 0..1."""
    y = (Y - 7.5) / 92.5
    u = np.nan_to_num(U) / 92.5
    v = np.nan_to_num(V) / 92.5
    b = y + u / 0.492
    r = y + v / 0.877
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    return np.clip(np.stack([r, g, b], -1), 0, 1)


def coords(s, lines):
    """Row and column of every sample. Like a goggle's line lock: a line
    starts at its own H-sync edge (column 0); the snapshot's frame position
    only says which line that is. Without a sync of its own the fitted
    lattice places it."""
    j = np.arange(SNAP)
    x = s['x0'] + j / FS * 1e6 / s['h']
    row = np.floor(x).astype(int)
    col = np.floor((x - row) * COLS).astype(int)
    e = s.get('e', np.array([]))
    if s.get('linelock', True) and len(e):
        per = s['h'] * FS / 1e6
        bounds = np.r_[e[0] - per, e, np.inf]
        seg = np.searchsorted(e, j, side='right')           # 0: before the first edge
        start = bounds[seg]
        xe = s['x0'] + e[0] / FS * 1e6 / s['h']             # the first edge's place, ~whole line
        r0 = int(np.floor(xe + 0.5))
        row = r0 + seg - 1
        col = np.floor(j - start).astype(int)
        far = col >= COLS                                    # past the next (missing) sync
        row[far] += col[far] // COLS
        col[far] %= COLS
        col = np.clip(col, 0, COLS - 1)
    return row % lines, col


def paint(snaps, lines, which='lp', mode='first', exclude=None):
    """Frame (lines x COLS) from the snapshots at their x0."""
    acc = [[] for _ in range(lines)] if mode == 'median' else None
    img = np.full((lines, COLS), np.nan)
    stack = {}
    for idx, s in enumerate(snaps):
        if exclude is not None and idx == exclude or s.get('drop'):
            continue
        row, col = coords(s, lines)
        keep = np.arange(SNAP) > 60
        r, c, v = row[keep], col[keep], s[which][keep]
        if mode == 'first':
            empty = np.isnan(img[r, c])
            img[r[empty], c[empty]] = v[empty]
        else:
            key = r * COLS + c
            for kk, vv in zip(key, v):
                stack.setdefault(kk, []).append(vv)
    if mode != 'first':
        for kk, vals in stack.items():
            img[kk // COLS, kk % COLS] = np.median(vals)
    return img


def paint_best(snaps, lines, ref_lp):
    """Every pixel from ONE snapshot: per line piece the snapshot whose
    low-pass agrees best with the mean of all (ref_lp), so a misplaced or
    damaged piece loses to a good one covering the same line."""
    pieces = []
    for s in snaps:
        if s.get('drop'):
            continue
        row, col = coords(s, lines)
        j = np.arange(SNAP)
        keep = j > 60
        e = s.get('e', np.array([]))
        for r in np.unique(row[keep]):
            m = keep & (row == r)
            rv = ref_lp[r, col[m]]
            ok = ~np.isnan(rv)
            if ok.sum() < 50:
                continue
            err = float(np.mean(np.abs(s['lp'][m][ok] - rv[ok])))
            # anchored: the line's own sync edge is inside this snapshot
            jm = j[m]
            anchored = len(e) and np.any((e >= jm.min() - 2) & (e <= jm.min() + 2))
            if not anchored:
                err += 2.0
            pieces.append((err, r, col[m], np.stack([s['v'][m], s['Y'][m], s['U'][m], s['V'][m]])))
    pieces.sort(key=lambda p: p[0])
    img = np.full((4, lines, COLS), np.nan)
    for err, r, c, v in pieces:
        empty = np.isnan(img[0, r, c])
        img[:, r, c[empty]] = v[:, empty]
    return img


def paint_fast(snaps, lines, which='lp', exclude=None, reducer='mean'):
    """Mean (fast) frame over all snapshots covering each pixel."""
    s_sum = np.zeros(lines * COLS)
    s_n = np.zeros(lines * COLS)
    for idx, s in enumerate(snaps):
        if (exclude is not None and idx == exclude) or s.get('drop'):
            continue
        row, col = coords(s, lines)
        key = row * COLS + col
        keep = np.arange(SNAP) > 60
        np.add.at(s_sum, key[keep], s[which][keep])
        np.add.at(s_n, key[keep], 1)
    out = np.where(s_n > 0, s_sum / np.maximum(s_n, 1), np.nan)
    return out.reshape(lines, COLS), s_n.reshape(lines, COLS)


def xcorr_shift(a, b):
    """Shift (rows, cols) of b onto a by masked circular cross-correlation."""
    ma, mb = ~np.isnan(a), ~np.isnan(b)
    a0 = np.where(ma, a - np.nanmean(a), 0)
    b0 = np.where(mb, b - np.nanmean(b), 0)
    fa, fb = np.fft.rfft2(a0), np.fft.rfft2(b0)
    num = np.fft.irfft2(fa * np.conj(fb), a.shape)
    cnt = np.fft.irfft2(np.fft.rfft2(ma.astype(float)) * np.conj(np.fft.rfft2(mb.astype(float))), a.shape)
    score = np.where(cnt > 0.15 * min(ma.sum(), mb.sum()), num / np.maximum(cnt, 1), -np.inf)
    r, c = np.unravel_index(int(np.argmax(score)), score.shape)
    return int(r), int(c)


def snapshot_score(s, ref, lines, dx_lines):
    """Correlation of a snapshot's low-pass with the reference frame at an offset."""
    row, col = coords(dict(s, x0=s['x0'] + dx_lines), lines)
    rv = ref[row[60::4], col[60::4]]
    v = s['lp'][60::4]
    m = ~np.isnan(rv)
    if m.sum() < 400:
        return -np.inf, int(m.sum())
    a, b = v[m] - v[m].mean(), rv[m] - rv[m].mean()
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)), int(m.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', type=Path)
    ap.add_argument('sweeps', type=Path, nargs='+')
    ap.add_argument('--decoder', default='U50P64-R1', help='U50P64-R1, U80P64-R1, U85Q, IDEAL, T32K70 or NL:kp_hi,kp_lo,knee[,pred]')
    ap.add_argument('--vb', type=int, help='frame start (line of the vertical blanking) from another run, puzzle.json vblank_line')
    ap.add_argument('--synced-only', action='store_true', help='leave out snapshots without an H-sync of their own')
    a = ap.parse_args()
    lines = 525
    sweeps, info = [], []
    for src in a.sweeps:
        s, i = place_sweep(src, a.decoder, lines)
        sweeps.append(s)
        info.append(i)
        print('placed', i, flush=True)

    # Sweeps onto the first one: whole lines and a fraction of a line, on the picture.
    base, _ = paint_fast(sweeps[0], lines)
    small = lambda img: np.nanmean(img[:, :COLS - COLS % 2].reshape(lines, -1, 2), axis=2)
    for j, sw in enumerate(sweeps[1:], 1):
        img, _ = paint_fast(sw, lines)
        r, c = xcorr_shift(small(base), small(img))
        dx = r + c * 2 / COLS
        dx = dx - lines if dx > lines / 2 else dx
        for s in sw:
            s['x0'] = np.mod(s['x0'] + dx, lines)
        info[j]['offset_lines'] = round(dx, 4)
        print(f'sweep {j}: offset {dx:.4f} lines', flush=True)
    snaps = [s for sw in sweeps for s in sw]
    if a.synced_only:
        for s in snaps:
            if not len(s.get('e', [])):
                s['drop'] = True

    # Every snapshot against the mean of all the OTHERS (leave-one-out, so a
    # misplaced piece is not held in place by itself): +-1 line, and +-3 us
    # for one without a sync of its own.
    for rnd in range(3):
        S = np.zeros(lines * COLS)
        N = np.zeros(lines * COLS)
        placed = []
        for s in snaps:
            if s.get('drop'):
                placed.append(None)
                continue
            row, col = coords(s, lines)
            key = (row * COLS + col)[60:]
            np.add.at(S, key, s['lp'][60:])
            np.add.at(N, key, 1)
            placed.append(key)
        moved = dropped = 0
        scores = []
        for idx, s in enumerate(snaps):
            if s.get('drop') or len(s.get('e', [])):
                continue   # line-locked: its row comes from its own syncs and time, not from content
            own = placed[idx]
            np.add.at(S, own, -s['lp'][60:])
            np.add.at(N, own, -1)
            best, best_dx = -np.inf, 0.0
            for dl in (-1, 0, 1):
                for dus in (np.zeros(1) if len(s.get('e', [])) else np.arange(-3.0, 3.01, 0.25)):
                    dx = dl + dus / s['h']
                    row, col = coords(dict(s, x0=s['x0'] + dx), lines)
                    k = (row * COLS + col)[60::4]
                    n = N[k]
                    m = n > 0.5
                    if m.sum() < 400:
                        continue
                    rv = S[k][m] / n[m]
                    v = s['lp'][60::4][m]
                    a_, b_ = v - v.mean(), rv - rv.mean()
                    sc = float(np.dot(a_, b_) / (np.linalg.norm(a_) * np.linalg.norm(b_) + 1e-9))
                    if sc > best:
                        best, best_dx = sc, dx
            if best < 0.6:
                s['drop'] = True
                dropped += 1
                continue
            if best_dx != 0.0:
                moved += 1
            s['x0'] = np.mod(s['x0'] + best_dx, lines)
            row, col = coords(s, lines)
            key = (row * COLS + col)[60:]
            np.add.at(S, key, s['lp'][60:])
            np.add.at(N, key, 1)
            placed[idx] = key
            scores.append(best)
        print(f'round {rnd}: moved {moved}, dropped {dropped}, kept {sum(not s.get("drop") for s in snaps)}/{len(snaps)}, '
              f'median score {np.median(scores):.3f}', flush=True)

    # Images: single snapshot per pixel (best = least folding first), and median-ish mean.
    ref_lp, _ = paint_fast(snaps, lines, which='lp')
    single_all = paint_best(snaps, lines, ref_lp)
    single = single_all[0]
    mean, cover = paint_fast(snaps, lines, which='v')
    # vertical blanking on top, fields interleaved, active picture only
    sync = np.nanpercentile(mean, 3)
    blank = np.nanpercentile(mean, 40)
    low = np.nanmean(mean < (sync + blank) / 2, axis=1)
    vb = int(np.argmax(np.convolve(np.r_[low, low[:9]], np.ones(9), 'valid')[:lines]))
    if a.vb is not None:
        vb = a.vb
    a.out.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def save(img, name):
        f = np.roll(img, -vb, axis=0)
        half = lines // 2
        inter = np.full((lines, COLS), np.nan)
        inter[0::2][:half + 1] = f[:half + 1]
        inter[1::2][:lines - half - 1] = f[half + 1:]
        pic = inter[22:-2, int(10.6 * 40):int(62.0 * 40)]
        lo, hi = np.nanpercentile(pic, 0.5), np.nanpercentile(pic, 99.5)
        im = np.where(np.isnan(pic), .5, np.clip((pic - lo) / (hi - lo), 0, 1))
        x = np.linspace(0, im.shape[1] - 1, 720)
        im = np.stack([np.interp(x, np.arange(im.shape[1]), row) for row in im])
        plt.imsave(a.out / name, im, cmap='gray', vmin=0, vmax=1)
        return float(np.mean(~np.isnan(pic)))
    fill = save(single, 'frame_single.png')

    def interleave(img):
        f = np.roll(img, -vb, axis=0)
        half = lines // 2
        inter = np.full((lines, COLS), np.nan)
        inter[0::2][:half + 1] = f[:half + 1]
        inter[1::2][:lines - half - 1] = f[half + 1:]
        return inter[22:-2, int(10.6 * 40):int(62.0 * 40)]
    Yi, Ui, Vi = (interleave(single_all[c]) for c in (1, 2, 3))
    rgb = to_rgb(np.nan_to_num(Yi, nan=50.0), Ui, Vi)
    x = np.linspace(0, rgb.shape[1] - 1, 720).astype(int)
    plt.imsave(a.out / 'frame_color.png', rgb[:, x])
    plt.imsave(a.out / 'frame_color_vflip.png', to_rgb(np.nan_to_num(Yi, nan=50.0), Ui, -Vi)[:, x])
    save(mean, 'frame_mean.png')
    res = dict(decoder=a.decoder, sweeps=info, snapshots=len(snaps), kept=sum(not s.get('drop') for s in snaps),
               vblank_line=vb, active_fill=round(fill, 3), mean_cover=round(float(np.mean(cover)), 2))
    (a.out / 'puzzle.json').write_text(json.dumps(res, indent=1))
    print(json.dumps(res))


if __name__ == '__main__':
    main()
