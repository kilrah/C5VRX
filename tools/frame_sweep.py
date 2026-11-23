#!/usr/bin/env python3
"""Rebuild one video frame from a C5VRX-5 IQ frame sweep (console key J).

C5VRX by Twotoz and the C5VRX contributors. Bench tool; it extends the IQ
snapshot (`I`, tools/capture_iq_snapshot.py). One snapshot is 8190 bytes
(204.75 us, ~3 lines) and lands at a random place in the frame. J takes them
spread over many frames, each FRAME_SWEEP_STEP_NS (150 us) later in the frame
than the one before (`frames` whole frames on from the first), and stamps each with the time of the RX descriptor boundary it ends
on (~1 us). This tool:

1. sends J (DTR/RTS low: a line reset hangs a running board) and reads the
   IQSWEEP records, checking every snapshot's FNV-1a;
2. demodulates each with the bit-exact U50P64-R1 (or another production
   table, --decoder), the same table the firmware runs;
3. places the snapshots in the frame: nominal position from the boundary
   times, corrected onto the transmitter's own line lattice with the H-syncs
   (line period measured from the snapshots), so crystal drift between VTX
   and C5 does not smear the picture;
4. rotates the frame so the vertical blanking is at the top, interleaves the
   two fields and writes a grey image of the active picture (5 MHz goggle
   filter on, as a goggle sees it; --raw for the bare DAC codes).

The camera picture must stand still for the capture (~30-40 s). Missing pieces
(a refused snapshot) stay mid grey. It is a picture of what the receiver
decodes, not a measurement of range or of the goggle.

    python tools/frame_sweep.py COM10 out_dir            # capture, rebuild, frame.png
    python tools/frame_sweep.py --from out_dir out_dir   # rebuild a saved capture
"""
import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'research'))
FS = 40e6
SNAP = 8190


def fnv1a(data):
    h = 2166136261
    for b in data:
        h = ((h ^ b) * 16777619) & 0xffffffff
    return h


def capture(port, out, timeout=600.0):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.2, 1
    s.dtr = False
    s.rts = False
    s.open()
    try:
        # An earlier sweep may still be printing: wait for 3 s of quiet.
        quiet = time.monotonic()
        stop = quiet + 120
        while time.monotonic() - quiet < 3.0 and time.monotonic() < stop:
            if b'IQSWEEP_' in s.read(65536):
                quiet = time.monotonic()
        s.reset_input_buffer()
        s.write(b'J')
        end = time.monotonic() + timeout
        buf, meta, snaps, data = b'', None, {}, {}
        while time.monotonic() < end:
            buf += s.read(65536)
            *lines, buf = buf.split(b'\n')
            for raw in lines:
                line = raw.decode('ascii', 'replace').strip()
                at = line.find('IQSWEEP_')
                if at < 0:
                    continue
                line = line[at:]
                if line.startswith('IQSWEEP_CAPTURE'):
                    print(line, flush=True)
                    continue
                if meta is None and not line.startswith(('IQSWEEP_BEGIN', 'IQSWEEP_REFUSED')):
                    continue
                if line.startswith('IQSWEEP_REFUSED'):
                    raise SystemExit(line)
                if line.startswith('IQSWEEP_BEGIN'):
                    meta = dict(re.findall(r'(\w+)=(\S+)', line))
                    print(line, flush=True)
                    print('>>> CAPTURE DONE: the VTX may go off now <<<', flush=True)
                elif line.startswith('IQSWEEP_SNAP'):
                    d = dict(re.findall(r'(\w+)=(\S+)', line))
                    k = int(d['k'])
                    if k % 25 == 0:
                        print(f'snapshot {k}/{meta["n"] if meta else "?"}', flush=True)
                    if d['ok'] == '1':
                        snaps[k] = d
                        data[k] = bytearray()
                elif line.startswith('IQSWEEP_DATA'):
                    # Another task's print can cut into a line: such a snapshot is
                    # dropped (and its hash would fail anyway), the rest go on.
                    m = re.match(r'IQSWEEP_DATA k=(\d+) offset=(\d+) ([0-9a-f]+)$', line)
                    if not m:
                        continue
                    k, off = int(m.group(1)), int(m.group(2))
                    if k not in data or data[k] is None:
                        continue
                    if len(data[k]) != off or len(m.group(3)) % 2:
                        print(f'snapshot {k}: damaged line at offset {off}, dropped', flush=True)
                        data[k] = None
                        continue
                    data[k] += bytes.fromhex(m.group(3))
                elif line.startswith('IQSWEEP_END'):
                    return save(out, meta, snaps, data)
        raise SystemExit('timeout before IQSWEEP_END')
    finally:
        s.close()


def save(out, meta, snaps, data):
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for k in sorted(snaps):
        if data.get(k) is None:
            continue
        b = bytes(data[k])
        if len(b) != SNAP or f'{fnv1a(b):08x}' != snaps[k]['fnv1a']:
            print(f'snapshot {k}: length/hash mismatch, dropped')
            continue
        (out / f'k{k:03d}.bin').write_bytes(b)
        rows.append({'k': k, 'frames': int(snaps[k]['frames']), 't_us': int(snaps[k]['t_us']),
                     'gain': int(snaps[k]['gain'])})
    meta = dict(meta, snapshots=rows)
    (out / 'sweep.json').write_text(json.dumps(meta, indent=1))
    print(f'saved {len(rows)} snapshots to {out}')
    return meta


def lp(x, hz, order=2):
    from scipy import signal as sg
    return sg.sosfiltfilt(sg.butter(order, hz, fs=FS, output='sos'), x)


def hsync_edges(v):
    """Falling sync edges (sub-sample) of normal 4.7 us H-syncs in a decoded
    snapshot, plus the sync and blanking levels."""
    s = lp(v, 1e6)
    lo, mid = np.percentile(s, 1), np.percentile(s, 50)
    # Sync tip to blanking is ~5-10 codes; a threshold from the picture's
    # median missed every sync under a bright picture.
    th = lo + min(5.0, (mid - lo) / 2)
    under = s < th
    e = np.flatnonzero(np.diff(np.r_[0, under.astype(int), 0]))
    edges = []
    for a, b in zip(e[::2], e[1::2]):
        if 150 <= b - a <= 230 and a > 2:
            x0, x1 = s[a - 1], s[a]
            edges.append(a - 1 + (x0 - th) / (x0 - x1) if x0 != x1 else float(a))
    return np.array(edges), lo, mid


def rebuild(src, out, decoder, raw_codes, debug=False):
    import sim
    from scipy import signal as sg
    meta = json.loads((src / 'sweep.json').read_text())
    if decoder == 'U50P64-R1':
        import u50p64
        table, bits = u50p64.table(), 4
    elif decoder == 'U80P64-R1':
        import p64_model
        table, bits = p64_model.repaired_table(), 4
    elif decoder == 'U85Q':
        table, bits = sim.u85q_lut(), 5
    else:
        table = None   # IDEAL: float discriminator on the same 4-bit IQ (DC removed), no LUT

    def ideal_codes(raw):
        b = raw.astype(int)
        z = ((((b >> 4) ^ 8) - 8) + .5) + 1j * ((((b & 15) ^ 8) - 8) + .5)
        z = z - z.mean()
        hz = np.angle(z[1:] * np.conj(z[:-1])) * FS / (2 * np.pi)
        return np.r_[hz[0], hz] / 15e6 * 63 + 6e6 / 15e6 * 63
    frame_us = int(meta['frame_ns']) / 1000.0
    snaps = meta['snapshots']
    video, edges = {}, {}
    for r in snaps:
        raw = np.frombuffer((src / f'k{r["k"]:03d}.bin').read_bytes(), np.uint8)
        codes = ideal_codes(raw) if table is None else sim._pair_codes(raw, table, sim.PAIR_BITS, bits).astype(float)
        video[r['k']] = codes if raw_codes else sg.sosfilt(sim.GOGGLE, codes)
        edges[r['k']] = hsync_edges(codes)[0]

    # Every snapshot sits at its own boundary time (C5 clock, ~1 us). The
    # transmitter's line period H in that clock is fitted over ALL H-sync
    # edges of the capture at once (the edges must fall on one lattice
    # t = phase + n * H); a frame is exactly 525 (625) lines, so the place of
    # any sample in the frame is (t - phase) mod (lines * H). Snapshots
    # without a sync of their own (folds, blanking) are placed the same way.
    lines = 525 if meta['std'] != 'PAL' else 625
    nominal_h = 64.0 if meta['std'] == 'PAL' else 63.5556
    snap_us = SNAP / FS * 1e6
    t_edges = np.concatenate([r['t_us'] - snap_us + edges[r['k']] / FS * 1e6 for r in snaps if len(edges[r['k']])])
    t_edges -= snaps[0]['t_us']
    span = max(float(np.ptp(t_edges)), 1.0)

    def coherence(h):
        return np.abs(np.mean(np.exp(2j * np.pi * t_edges / h)))
    h_us, step = nominal_h, nominal_h * 300e-6
    grid = np.arange(-1.0, 1.0001, 1.0 / 600)
    while True:
        cand = h_us + grid * step * 1.0
        c = np.array([coherence(h) for h in cand])
        h_us = float(cand[int(np.argmax(c))])
        if step * len(grid) / 2 < nominal_h * 0.02 / span:   # finer than 2 % of a line over the span
            break
        step = step / 100
    coh = coherence(h_us)
    phase = float(np.angle(np.mean(np.exp(2j * np.pi * t_edges / h_us)))) / (2 * np.pi) * h_us
    frame_len = lines * h_us
    line_n = int(round(h_us * FS / 1e6))
    # Per snapshot: its own syncs pull it onto the lattice (a few us of
    # boundary-time error); one without syncs takes its time neighbours'.
    resid = {}
    for r in snaps:
        e = edges[r['k']]
        if len(e):
            te = (r['t_us'] - snaps[0]['t_us']) - snap_us + e / FS * 1e6 - phase
            d = np.mod(te + h_us / 2, h_us) - h_us / 2
            resid[r['k']] = float(np.median(d))
    known = np.array(sorted(resid))
    shift = {}
    for r in snaps:
        k = r['k']
        if k in resid:
            shift[k] = resid[k]
        elif len(known):
            near = known[np.argsort(np.abs(known - k))[:2]]
            shift[k] = float(np.mean([resid[j] for j in near]))
        else:
            shift[k] = 0.0
    frame = np.full((lines, line_n), np.nan)
    for r in snaps:
        k = r['k']
        t = (r['t_us'] - snaps[0]['t_us']) - snap_us + np.arange(SNAP) / FS * 1e6 - phase - shift[k]
        p = np.mod(t, frame_len)
        n = np.floor(p / h_us).astype(int)
        col = np.floor((p - n * h_us) * FS / 1e6).astype(int)
        keep = (col >= 0) & (col < line_n) & (np.arange(SNAP) > 60)   # the decoder's first samples settle
        rows = n[keep] % lines
        cols = col[keep]
        empty = np.isnan(frame[rows, cols])
        frame[rows[empty], cols[empty]] = video[k][keep][empty]

    # Vertical blanking: the run of lines with the most time at sync level.
    sync_level = np.nanpercentile(frame, 3)
    blank = np.nanpercentile(frame, 40)
    low = np.nanmean(frame < (sync_level + blank) / 2, axis=1)
    score = np.convolve(np.r_[low, low[:9]], np.ones(9), 'valid')[:lines]
    vb = int(np.argmax(score))
    frame = np.roll(frame, -vb, axis=0)
    if debug:   # the frame in transmission order (field 1 then field 2), whole lines incl. sync
        out.mkdir(parents=True, exist_ok=True)
        lo_d, hi_d = np.nanpercentile(frame, 1), np.nanpercentile(frame, 99)
        seq = np.where(np.isnan(frame), .5, np.clip((frame - lo_d) / (hi_d - lo_d), 0, 1))[:, ::3]
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.imsave(out / 'sequential.png', seq, cmap='gray', vmin=0, vmax=1)
    half = lines // 2
    f1, f2 = frame[:half + 1], frame[half + 1:]
    inter = np.full((len(f1) + len(f2), line_n), np.nan)
    inter[0::2][:len(f1)] = f1
    inter[1::2][:len(f2)] = f2
    act_a, act_b = int(10.6 * FS / 1e6), int(62.0 * FS / 1e6)  # active line after the sync edge
    pic = inter[22:-2, act_a:act_b]
    lo, hi = np.nanpercentile(pic, 0.5), np.nanpercentile(pic, 99.5)
    img = np.clip((pic - lo) / (hi - lo), 0, 1)
    img = np.where(np.isnan(img), 0.5, img)
    w = 720
    x = np.linspace(0, img.shape[1] - 1, w)
    img = np.stack([np.interp(x, np.arange(img.shape[1]), row) for row in img])
    filled = float(np.mean(~np.isnan(frame)))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out.mkdir(parents=True, exist_ok=True)
    png = out / (('frame_raw' if raw_codes else 'frame') + ('' if decoder == 'U50P64-R1' else '_' + decoder) + '.png')
    try:
        plt.imsave(png, img, cmap='gray', vmin=0, vmax=1)
    except PermissionError:   # open in a viewer
        png = png.with_name(png.stem + f'_{int(time.time())}.png')
        plt.imsave(png, img, cmap='gray', vmin=0, vmax=1)
    info = dict(decoder=decoder, std=meta['std'], snapshots=len(snaps), line_us=round(h_us, 6), sync_coherence=round(float(coh), 3),
                frame_filled=round(filled, 3), shift_us_p95=round(float(np.percentile(np.abs(list(shift.values())), 95)), 2), vblank_line=vb, gains=sorted({r['gain'] for r in snaps}))
    (out / 'frame.json').write_text(json.dumps(info, indent=1))
    print(json.dumps(info))
    print(png)
    return png


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', nargs='?')
    ap.add_argument('out', type=Path)
    ap.add_argument('--from', dest='src', type=Path, help='rebuild a saved capture directory')
    ap.add_argument('--decoder', default='U50P64-R1', choices=['U50P64-R1', 'U80P64-R1', 'U85Q', 'IDEAL'])
    ap.add_argument('--raw', action='store_true', help='bare DAC codes, no 5 MHz goggle filter')
    ap.add_argument('--debug', action='store_true', help='also write sequential.png (transmission order, whole lines)')
    a = ap.parse_args()
    src = a.src
    if src is None:
        if not a.port:
            ap.error('give a port, or --from a saved capture')
        capture(a.port, a.out)
        src = a.out
    rebuild(src, a.out, a.decoder, a.raw, a.debug)


if __name__ == '__main__':
    main()
