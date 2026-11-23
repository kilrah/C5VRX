"""PC receiver for the P4's Quest stream (docs/P4QUEST.md, wire format in p4/main/stream.h).

Says hello to the P4's UDP port 30210, receives the decoded lines, prints rates once a second
(rows, datagrams lost, fields, the P4's status line) and saves the newest picture as a PNG
(4:3, both fields woven) every --snap seconds.

Usage: python stream_view.py P4_ADDRESS [--secs 10] [--snap 2] [--out snap.png]
Needs numpy and Pillow.
"""
import argparse
import socket
import struct
import time

import numpy as np
from PIL import Image

PORT = 30210
W, CW = 512, 64
ROW = 4 + W + 2 * CW


def to_rgb(y, cb, cr):
    """Full-range BT.601 YCbCr (chroma at 1/8 horizontal resolution) -> RGB, 4:3."""
    x = (np.arange(W) + 0.5) / 8 - 0.5          # window i centred on pixel 8i + 4
    i0 = np.clip(np.floor(x).astype(int), 0, CW - 1)
    i1 = np.clip(i0 + 1, 0, CW - 1)
    f = np.clip(x - np.floor(x), 0, 1)
    up = lambda c: c[:, i0] * (1 - f) + c[:, i1] * f
    Y = y.astype(np.float32)
    Cb = up(cb.astype(np.float32)) - 128
    Cr = up(cr.astype(np.float32)) - 128
    rgb = np.stack([Y + 1.402 * Cr, Y - 0.344136 * Cb - 0.714136 * Cr, Y + 1.772 * Cb], axis=-1)
    img = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8))
    return img.resize((img.height * 4 // 3, img.height), Image.BILINEAR)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("host")
    ap.add_argument("--secs", type=float, default=10)
    ap.add_argument("--snap", type=float, default=2)
    ap.add_argument("--out", default="snap.png")
    a = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    s.settimeout(0.2)
    dst = (a.host, PORT)

    H = 576
    y = np.zeros((H, W), np.uint8)
    cb = np.full((H, CW), 128, np.uint8)
    cr = np.full((H, CW), 128, np.uint8)
    t0 = last = last_hello = last_snap = time.monotonic()
    expect = None
    n_rows = n_dg = lost = 0
    fields = set()
    status = ""
    prog = False
    s.sendto(b"H", dst)
    while time.monotonic() - t0 < a.secs:
        now = time.monotonic()
        if now - last_hello >= 1:
            s.sendto(b"H", dst)
            last_hello = now
        try:
            d, _ = s.recvfrom(65536)
        except socket.timeout:
            d = None
        if d and d[:2] == b"C5":
            typ = d[3]
            seq, field, height = struct.unpack_from("<IHH", d, 4)
            if expect is not None and seq > expect:
                lost += seq - expect
            expect = seq + 1
            n_dg += 1
            if typ == 2:
                status = d[12:].decode(errors="replace")
            elif typ == 1:
                if height != H:
                    H = height
                    y = np.zeros((H, W), np.uint8)
                    cb = np.full((H, CW), 128, np.uint8)
                    cr = np.full((H, CW), 128, np.uint8)
                fields.add(field)
                for off in range(12, len(d) - ROW + 1, ROW):
                    row, flags = struct.unpack_from("<HB", d, off)
                    prog = bool(flags & 2)
                    rows = (2 * row, 2 * row + 1) if prog else (row,)
                    for r in rows:
                        if r < H:
                            y[r] = np.frombuffer(d, np.uint8, W, off + 4)
                            cb[r] = np.frombuffer(d, np.uint8, CW, off + 4 + W)
                            cr[r] = np.frombuffer(d, np.uint8, CW, off + 4 + W + CW)
                    n_rows += 1
        if now - last >= 1:
            print(f"{now - t0:5.1f}s rows {n_rows:6d}/s  dgrams {n_dg:5d}  lost {lost:4d}  fields {len(fields):3d}"
                  f"  {'prog' if prog else 'interl'} {H}  | {status}")
            n_rows = n_dg = lost = 0
            fields.clear()
            last = now
        if now - last_snap >= a.snap:
            to_rgb(y, cb, cr).save(a.out)
            last_snap = now
    to_rgb(y, cb, cr).save(a.out)
    print("saved", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
