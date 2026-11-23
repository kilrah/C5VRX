# P4 Quest: C5VRX picture in a Meta Quest

A variant of the [P4 link](P4LINK.md) build for a Waveshare ESP32-P4-Module-DEV-KIT
(no display): the P4 decodes the C5's bus exactly as on the JC1060P470C, but
instead of building frames for an LCD it sends **every line as soon as it is
decoded**, uncompressed, as UDP to a Meta Quest. No encoder, no frame buffer,
no waiting for a field: a line leaves the P4 about a millisecond after the C5
received it.

Status (2026-11-23, bench, PAL VTX on E5, C5 decoder U80P64-R1 codes): the
Quest receives all 14,400 lines a second (50 fields/s), none dropped on the
P4, none lost on the link; colour picture in the PC receiver's snapshot, not
later looked at in the headset (operator: fine). Raw IQ (`dem=RAW`, the P4's
BitScrambler demodulates) and U85Q: also full rate, locked, after a bad D0
(GPIO2) wire was replaced; before that raw IQ found almost no hsyncs.

## Pipeline

```text
C5 (FireBeetle, C5VRX-5 P4-link build, unchanged)
  -> 8-bit bus + clock, UART status/commands            (docs/P4LINK.md)
P4 (Waveshare): PARLIO RX -> fill (core 0) -> decoder (core 1, cvbs.c)
  -> per line: Y 512 + Cb 64 + Cr 64 bytes into a 96-row ring (stream.c)
  -> sender (core 0): Ethernet/IP/UDP headers written by hand, two lines a
     frame, a millisecond's lines in one USB transfer (ncm_host.c)
  -> USB high-speed host -> board hub -> Type-A -> Quest (CDC-NCM, 192.168.7.x)
Quest: app "C5VRX FPV" (DJI-on-Quest repository, quest-view flavor fpv): rows
  into textures as they arrive, YCbCr -> RGB in a shader, 4:3, drawn at every
  display refresh; deinterlacing "bob" (default) shows the newest field
  everywhere (the field being received above its newest line, the previous one
  below, each interpolated to full height) or "weave"; buttons for channel,
  band, scan, AFC and the C5's menu
```

- 14,400 lines/s (PAL 288 x 50, NTSC 240 x 60) x 644 bytes = ~75 Mbit/s with
  headers. The P4's USB host carries ~93 Mbit/s of UDP to the Quest once it
  packs several frames into one NCM transfer.
- Chroma: the decoder's 64 colour windows across the line, both Cb and Cr on
  every line (the LCD build keeps one per row, YUV420), window i centred on
  pixel 8i + 4; the Quest's linear texture filtering interpolates.
- Through lwIP a UDP send cost 70 us on an idle P4 and 250-600 us next to the
  capture and decoder: only half the lines got out. To the Quest the rows go
  past lwIP (headers by hand, UDP checksum 0, allowed in IPv4); status and
  commands still use lwIP, which also keeps the Quest's ARP entry. Ethernet
  (PC tools) uses lwIP: about half the lines.
- The sender runs on core 0 next to the fill task. On core 1, below the
  decoder, the decoder's bursts held it off long enough to overflow the ring.

## Wiring

As [P4LINK.md](P4LINK.md), on the Waveshare 40-pin header (Raspberry Pi
numbering: the 3V3 row is the odd pins), except the UART:

| Signal | C5 GPIO | P4 GPIO | Header pin |
|---|---|---|---|
| D0..D2 | 2, 3, 4 | 2, 3, 4 | 21, 19, 18 |
| D3, D4 | 5, 6 | 32, 33 | 26, 29 |
| D5, D6, D7 | 7, 26, 27 | 47, 46, 45 | 37, 36, 40 |
| Clock | 8 | 20 | 13 (GND 14 next to it) |
| C5 TX -> P4 RX | 11 | **21** | **11** |
| P4 TX -> C5 RX | 12 | **22** | **12** |
| GND | GND | GND | 14, 30, 34, 39 |

GPIO37/38 (header TXD/RXD) carry the board's own CH343 USB-UART: its TX held
GPIO38 against the C5's even with its USB unplugged, and the P4 never heard
the C5 (2026-11-23). Each board on its own USB power; no 5 V between them.

USB_SEL jumper in the position labelled "device" (it routes the high-speed
port to the hub and the Type-A ports). The Quest goes into a Type-A port with
its default USB function set to NCM once:

```sh
adb shell svc usb setScreenUnlockedFunctions ncm
```

## Build and flash

```sh
cd p4
idf.py -B build-quest -DSDKCONFIG=build-quest/sdkconfig "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.quest.defaults" build
idf.py -B build-quest -p PORT flash
```

Console on the board's USB Serial/JTAG Type-C port (opening it resets the
board). `STREAM` lines: client, `raw=1` (rows past lwIP), rows sent,
`ring_full` (rows dropped because the sender fell behind).

## Wire format

UDP from the P4's port 30210 to whoever last sent `H` there (at least every
3 s); see `p4/main/stream.h`. Type 1: rows (`u16 row, u8 flags, Y[512],
Cb[64], Cr[64]`), type 2: the C5's and the decoder's status as text, 4/s.
Commands to the P4: `K<key>`, `C<idx>`, `A` as the link's (`link.h`).

PC receiver: `python p4/tools/stream_view.py P4_ADDRESS` (rates, the P4's
status, a PNG of the newest picture).

## Open items

- Latency, glass to glass (LED in front of the camera, Quest and LED filmed
  at 240 fps); the Quest view is a 2D panel for now.
- Horizontal sync tears on sharp transitions to white with one camera
  (operator: probably a too-hot signal); compare with other systems.
