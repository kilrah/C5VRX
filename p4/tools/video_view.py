"""Live PC video viewer for the P4's line stream (docs/P4QUEST.md, wire format in p4/main/stream.h).

C5VRX by Twotoz and the C5VRX contributors.

Says hello to the P4's UDP port 30210 and draws the decoded picture as the rows arrive (4:3), with
the P4's status underneath: the rows/s reaching the PC (14,400 at full rate), missing datagram
sequence numbers (lost on the network, or a failed send on the P4, which numbers a datagram before
sending it: send_err / raw_err in its STREAM console line) and rows the P4 itself dropped (its
sender fell behind). A row that did not arrive keeps its previous picture.

A receive thread only queues datagrams; the main loop parses them in bulk (numpy) and converts and
scales with OpenCV, so the GUI does not hold the socket off.

Keys
  Left/Right      channel within the band       Up/Down   band
  a               AFC on/off                    t         latency test (LED on P4 GPIO7)
  o e v f n x c b s g G m                       the C5 console key (o menu; n/x/e inside it)
  F / B (shift)   calibrate quick / full
  F2              deinterlace (below)           F5        save a PNG
  F11             full screen                   Esc       quit

Deinterlacing, as the Quest app's (F2 cycles; a progressive source is always drawn as received):
  bob, whole fields  (default) the last complete field, interpolated to full height: no tear, the
                     picture changes once a field (~10 ms more latency on average, up to a field)
  bob, newest lines  the field being received above its newest line, the previous one below it,
                     each interpolated to full height: lines on screen as soon as they arrive, but
                     the picture tears where the two meet
  weave              both fields' lines as received (full detail when nothing moves)
Each field's line k of parity p sits at frame row 2k + p; the rows in between are the mean of the
field lines above and below.

Usage: python video_view.py P4_ADDRESS [--height 720] [--deint whole|newest|weave]
Needs numpy and opencv-python.
"""
import argparse
import socket
import struct
import threading
import time

import cv2
import numpy as np

PORT = 30210
W, CW = 512, 64
ROW_DT = np.dtype([("row", "<u2"), ("flags", "u1"), ("pad", "u1"),
                   ("y", "u1", W), ("cb", "u1", CW), ("cr", "u1", CW)])
ROW = ROW_DT.itemsize      # 644
C5_KEYS = set("oevfnxcbsgGmFB")
LINK_CHANNELS = 48
BAR = 48                   # status bar height (pixels at the display size)
WIN = "C5VRX P4 video"
WEAVE, NEWEST, WHOLE = 0, 1, 2
DEINT_NAME = {WEAVE: "weave", NEWEST: "bob, newest lines", WHOLE: "bob, whole fields"}
DEINT_CYCLE = (WHOLE, NEWEST, WEAVE)

# cv2.waitKeyEx codes (Windows)
K_LEFT, K_UP, K_RIGHT, K_DOWN = 0x250000, 0x260000, 0x270000, 0x280000
K_F2, K_F5, K_F11 = 0x710000, 0x740000, 0x7A0000


class Receiver(threading.Thread):
    """Socket -> queue of row datagrams; status and loss counters; hello every second."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.dst = (host, PORT)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 << 20)
        self.s.settimeout(0.2)
        self.lock = threading.Lock()
        self.pending = []
        self.status = {}
        self.latency = ""
        self.lost = 0
        self.last_rx = 0.0
        self.running = True

    def send(self, msg: bytes):
        try:
            self.s.sendto(msg, self.dst)
        except OSError:
            pass

    def take(self):
        with self.lock:
            p, self.pending = self.pending, []
        return p

    def run(self):
        expect = None
        last_hello = 0.0
        recv = self.s.recv
        while self.running:
            now = time.monotonic()
            if now - last_hello >= 1:
                self.send(b"H")
                last_hello = now
            try:
                d = recv(65536)
            except socket.timeout:
                continue
            except OSError:          # Windows: ICMP port unreachable while the P4 boots
                time.sleep(0.2)
                continue
            if len(d) < 12 or d[0] != 0x43 or d[1] != 0x35:
                continue
            self.last_rx = now
            seq = int.from_bytes(d[4:8], "little")
            if expect is not None and seq > expect:
                self.lost += seq - expect
            expect = seq + 1
            typ = d[3]
            if typ == 1:
                with self.lock:
                    self.pending.append(d)
            elif typ == 2:
                txt = d[12:].decode(errors="replace")
                self.status = dict(kv.split("=", 1) for kv in txt.split() if "=" in kv)
            elif typ == 3:
                self.latency = d[12:].decode(errors="replace")
                print(self.latency)


class Frame:
    """The picture: Y at full width, Cb/Cr at 64 windows a line."""

    def __init__(self):
        self.alloc(576)
        self.prog = False
        self.parity = 0          # field parity of the newest row (interlaced)
        self.row = 0             # newest frame row
        self.fields = 0          # bumps when the parity changes: the previous field is complete

    def alloc(self, h):
        self.H = h
        self.y = np.zeros((h, W), np.uint8)
        self.cb = np.full((h, CW), 128, np.uint8)
        self.cr = np.full((h, CW), 128, np.uint8)

    def add(self, dgrams):
        """Parse a batch of row datagrams; returns the number of rows."""
        n = 0
        parts = []
        for d in dgrams:
            height = struct.unpack_from("<H", d, 10)[0]
            if height and height != self.H:
                self.alloc(height)
                parts = []
            k = (len(d) - 12) // ROW
            if k:
                parts.append(np.frombuffer(d, ROW_DT, k, 12))
        if not parts:
            return 0
        r = np.concatenate(parts) if len(parts) > 1 else parts[0]
        n = len(r)
        self.prog = bool(r["flags"][-1] & 2)
        rows = r["row"].astype(np.intp)
        if self.prog:
            rows = np.concatenate([2 * rows, 2 * rows + 1])
            src = np.concatenate([np.arange(n), np.arange(n)])
        else:
            par = int(rows[-1] & 1)
            if par != self.parity or (rows & 1 != par).any():
                self.fields += 1
            self.parity = par
            self.row = int(rows[-1])
            src = np.arange(n)
        ok = rows < self.H
        rows, src = rows[ok], src[ok]
        self.y[rows] = r["y"][src]
        self.cb[rows] = r["cb"][src]
        self.cr[rows] = r["cr"][src]
        return n

    @staticmethod
    def to_bgr(y, cb, cr):
        h = y.shape[0]
        # window i centred on pixel 8i + 4: cv2's half-pixel linear resize lines up with it
        cr = cv2.resize(cr, (W, h), interpolation=cv2.INTER_LINEAR)
        cb = cv2.resize(cb, (W, h), interpolation=cv2.INTER_LINEAR)
        return cv2.cvtColor(cv2.merge((y, cr, cb)), cv2.COLOR_YCrCb2BGR)   # full-range BT.601

    def field(self, p):
        """Field of parity p at full height: its line k on frame row 2k + p, the rows between the
        mean of the field lines above and below (the edge row repeats its neighbour)."""
        f = self.to_bgr(self.y[p::2], self.cb[p::2], self.cr[p::2])
        out = np.empty((self.H, W, 3), np.uint8)
        out[p::2] = f
        n = out[1 - p::2].shape[0]
        lo = f.astype(np.uint16)
        if p == 0:      # row 2k + 1 between lines k and k + 1
            nb = np.concatenate((lo[1:], lo[-1:]))
            out[1::2] = ((lo + nb + 1) >> 1)[:n]
        else:           # row 2k between lines k - 1 and k
            nb = np.concatenate((lo[:1], lo[:-1]))
            out[0::2] = ((lo + nb + 1) >> 1)[:n]
        return out

    def bgr(self, mode):
        if self.prog or mode == WEAVE:
            return self.to_bgr(self.y, self.cb, self.cr)
        if mode == WHOLE:
            return self.field(1 - self.parity)
        out = self.field(1 - self.parity)                  # NEWEST: previous field below,
        cut = min(self.row + 1, self.H)                    # the current one down to its newest row
        out[:cut] = self.field(self.parity)[:cut]
        return out


def status_bar(rx, s, w, rates, now):
    bar = np.full((BAR, w, 3), 32, np.uint8)
    g = s.get
    if now - rx.last_rx > 2:
        l1 = f"waiting for {rx.dst[0]}:{PORT} ..."
    else:
        l1 = (f"{g('ch', '?')} {g('f', '?')} MHz  sig {g('sig', '?')}  C/N {g('cn', '?')}  "
              f"gain {g('gain', '?')} {g('gm', '')}  AFC {'on' if g('afc') == '1' else 'off'} {g('cfo', '?')} kHz  "
              f"{g('dem', '?')}  lock {g('lock', '?')}  {g('fields', '?')} fields/s"
              + ("  MENU" if g("menu") == "1" else "") + (f"  SCAN {g('sp')}%" if g("scan") == "1" else "")
              + ("" if g("c5") == "1" else "  C5 OFFLINE"))
    for i, t in enumerate((l1, rates)):
        cv2.putText(bar, t, (6, 18 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    return bar


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", help="the P4's address (c5vrx-p4.local, USB NCM 192.168.7.1, ...)")
    ap.add_argument("--height", type=int, default=720, help="picture height drawn (the window scales it)")
    ap.add_argument("--deint", choices=("whole", "newest", "weave"), default="whole",
                    help="deinterlacing at the start (F2 cycles)")
    a = ap.parse_args()

    host = socket.gethostbyname(a.host)     # resolve .local once, not per datagram
    rx = Receiver(host)
    rx.start()
    fr = Frame()
    deint = {"whole": WHOLE, "newest": NEWEST, "weave": WEAVE}[a.deint]
    full = False
    ph = a.height
    pw = ph * 4 // 3
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(WIN, pw, ph + BAR)
    cv2.setWindowTitle(WIN, f"{WIN} - {a.host}")

    t_rate = time.monotonic()
    n_rows = n_draw = 0
    drop0 = None
    rates = ""
    img = None
    drawn_field = -1
    while True:
        new = fr.add(rx.take())
        n_rows += new
        now = time.monotonic()
        if now - t_rate >= 1:
            dt = now - t_rate
            try:
                drop = int(rx.status.get("drop", "0"))
            except ValueError:
                drop = 0
            p4drop = 0 if drop0 is None or drop < drop0 else drop - drop0
            drop0 = drop
            rates = (f"rows {n_rows / dt:6.0f}/s at the PC   P4 dropped {p4drop / dt:5.0f}/s   "
                     f"missing datagrams {rx.lost:4d}   drawn {n_draw / dt:4.1f}/s   "
                     f"{'prog' if fr.prog else 'interl'} {fr.H}  {DEINT_NAME[deint]} (F2)")
            rx.lost = 0
            n_rows = n_draw = 0
            t_rate = now
        # whole fields: draw once a field is complete; otherwise whenever rows arrived
        if img is None or (new and (fr.prog or deint != WHOLE or fr.fields != drawn_field)):
            drawn_field = fr.fields
            pic = cv2.resize(fr.bgr(deint), (pw, ph), interpolation=cv2.INTER_LINEAR)
            img = np.vstack((pic, status_bar(rx, rx.status, pw, rates, now)))
            cv2.imshow(WIN, img)
            n_draw += 1
        k = cv2.waitKeyEx(1)
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break
        if k == -1:
            continue
        try:
            idx = int(rx.status.get("idx", "-1"))
        except ValueError:
            idx = -1
        ch = chr(k) if 32 <= k < 127 else ""
        if k == 27:
            if full:
                full = False
                cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            else:
                break
        elif k == K_F11:
            full = not full
            cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if full else cv2.WINDOW_NORMAL)
        elif k == K_F2:
            deint = DEINT_CYCLE[(DEINT_CYCLE.index(deint) + 1) % len(DEINT_CYCLE)]
            img = None
        elif k == K_F5:
            name = time.strftime("p4_%Y%m%d_%H%M%S.png")
            pic = fr.bgr(WEAVE)
            cv2.imwrite(name, cv2.resize(pic, (pic.shape[0] * 4 // 3, pic.shape[0]), interpolation=cv2.INTER_LINEAR))
            print("saved", name)
        elif k in (K_LEFT, K_RIGHT) and idx >= 0:
            rx.send(f"C{idx // 8 * 8 + (idx % 8 + (1 if k == K_RIGHT else 7)) % 8}".encode())
        elif k in (K_UP, K_DOWN) and idx >= 0:
            rx.send(f"C{(idx + (8 if k == K_UP else LINK_CHANNELS - 8)) % LINK_CHANNELS}".encode())
        elif ch == "a":
            rx.send(b"A")
        elif ch == "t":
            rx.send(b"T")
        elif ch in C5_KEYS and ch:
            rx.send(b"K" + ch.encode())
    rx.running = False
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
