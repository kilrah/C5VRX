# P4 link: FireBeetle 2 ESP32-C5 tuner + ESP32-P4 display

The C5VRX-5 receiver runs on a DFRobot FireBeetle 2 ESP32-C5 (DFR1284,
ESP32-C5-WROOM-1U-N4: 4 MB flash, no PSRAM, U.FL antenna) exactly as on the
XIAO (gain, AFC, calibration, DC servo, U80P64-R1/U85Q, menu). Its raw IQ
(decoder RAW, the default) or its six-bit DAC codes (P64/U85Q) leave on
eight header pins together with the PARLIO TX clock, and a UART carries
status and commands. A Guition JC1060P470C (ESP32-P4, 1024 x 600 MIPI-DSI
LCD, GT911 touch) samples that bus, demodulates raw IQ with its own
BitScrambler, decodes the composite signal,
shows the picture 4:3 on the left 800 x 600 of its screen with a touch
settings and status column on the right 224 px, and streams the same
picture as a USB webcam (UVC, MJPEG 800 x 600) on its high-speed USB port.

Status (2026-11-01, raw IQ): the C5 sends its raw ring (RAW) and the P4's
BitScrambler runs U80P64-R1 on it; PAL locked at 50 fields/s with the same
levels as the C5's own P64 (sync ~11, blanking ~21 codes), and live
switches RAW -> P64 -> U85Q -> RAW each followed by the P4. The C5's
sampling phase settled on the moved IQ pads. Picture comparison by eye,
weak signal and NTSC are not checked yet.

Status (2026-10-31, code bus, bench, one FireBeetle + one JC1060P470C, PAL
VTX on B5): the UART link, status column and touch controls work; the P4 shows a
stable PAL colour picture at the full rate (15,625 lines/s, 50 fields/s,
25 pictures/s shown, no overruns; decode task ~75 % of core 1), and the
C5's own menu (progressive) at 50 fields/s. NTSC, UVC at full rate and
range are not checked yet. The P4 decoder also passes a host test on
synthetic PAL and NTSC (`p4/test/test_cvbs.c`: lock, standard, field parity
and row order, levels, horizontal position, colour bars, +-300 ppm clock,
noise). See [Open items](#open-items).

## Why this split

The C5 keeps all of C5VRX-5: the control loop (Direct Gain/LEVEL, AFC, DC
servo, calibration, profiles) is hardware-proven on the C5's own IQ ring
and stays there, unchanged. What crosses the link is chosen by the C5's
decoder setting (console `m`, menu DEMOD, the P4's decoder row):

- **RAW** (default in this build): the raw ring bytes, Q4/I4 at 40 MS/s,
  through a pass-through TX program (`main/raw.bsasm`). The P4 demodulates.
  Today it runs the same U80P64-R1 in its own BitScrambler, so the picture
  should match the C5's P64; the point is that P4-side demodulators can now
  be tried without touching the C5.
- **P64 / U85Q**: the C5 demodulates as on the XIAO and the bus carries its
  six-bit DAC codes on bits 0..5 (bits 6/7 carry the program's token bits,
  which the P4 masks). Kept for A/B against RAW, and a resistor DAC on
  GPIO2..7 then still gives a composite output.

Both are source-synchronous (data and clock from the same PARLIO TX unit,
data changing on the falling edge): no sampling-phase search on the P4,
unlike raw MODEM_DIAG lanes that change at 80 MS/s. zerowidth's `p4usb`
fork moves raw 7+7 IQ lanes with its own minimal C5 firmware; here the IQ
is the C5VRX-5 ring the C5's own control loop reads.

Why the P4's BitScrambler and not its CPU: a P4 load costs several cycles,
and the plain-C ring fill alone (two loads and three stores a 32-bit word)
took 55 % of a 360 MHz core (2026-10-31). P64 is a per-pair recursion
through two table lookups at 20 M pairs/s; on the CPU that cannot fit next
to the decoder (75 % of core 1). The BitScrambler does it in the receive
path at no CPU cost, as `p4usb` did with its phase lookup.

No C5VRX-5 code path changes outside `#ifdef C5VRX_P4LINK`: no gain-tick,
task-priority or AFC-period changes as in the Waveshare LCD build.

## Pipeline

```text
C5:  MODEM_DIAG Q4/I4 40 MS/s -> PARLIO RX -> raw 32 KiB ring              (unchanged)
       -> TX BitScrambler: RAW pass-through (raw.bsasm)
                           or U80P64-R1 / U85Q (two lookups, unique CVBS20 as [D,D])
       -> PARLIO TX 40 MHz: 8 bits on GPIO2..7/26/27, clock out on GPIO8
     p4link.c: UART1 115200, status every 250 ms, commands -> s_commands
P4:  PARLIO RX, external 40 MHz clock, 8-bit
       -> RAW: RX BitScrambler U80P64-R1 (p64_rx.bsasm, two lookups, [D,D] out)
          codes: no BitScrambler
       -> 24 x 4032 B ring (2.4 ms, internal RAM)
       -> core 0 fill task (PIE, fill.S): even bytes = 20 MS/s codes (raw history),
          pair sums = 10 MS/s samples (history)
       -> core 1 decode task: sync slicer, line PLL, broad pulses / field parity on the
          10 MS/s samples; per line chroma from the burst (PIE correlations, corr.S) and
          Y (PIE, luma.S) at source resolution (512 pixels) -> YUV420 rows
       -> core 0 copier task: rows into woven YUV420 frames (PSRAM, 3 buffers),
          written back in 32-row blocks
       -> PPA: YUV420 -> RGB565, 512 -> 800 columns (25/16), 480 -> 600 / 564 -> 600 rows
          (20/16, 17/16; a progressive field 240/282 rows, 40/16, 34/16) into one of two DPI
          frame buffers swapped on the panel's refresh (and the UVC frame); core 0 adds
          overlays and the settings column
       -> UVC task: hardware JPEG (q80) of the newest frame, 30 fps paced
```

- Source/tap: the C5's TX output: its raw ring (RAW, Q4/I4 40 MS/s) or its
  DAC codes. Acquired rate 40 MHz bus, 8 bits; after the P4's decoder (RAW)
  or the C5's, 20 MS/s unique codes. Ring: 94.5 KiB of whole 4032-byte DMA
  nodes, partial receive, EOF every eight nodes. It is allocated first at
  boot: it needs one block of internal DMA memory (32 nodes no longer fitted
  next to the colour histories: `capture=ESP_ERR_NO_MEM`).
- Bus mode (`cvbs.c` mode_vote): raw IQ while the C5's status says
  `dem=RAW` and the bus shows I's two high bits (bits 6/7) on at least 1/8
  of the bytes; otherwise codes. The P4's P64 copy passes those two input
  bits into its spare output bits, so the C5 menu raster or a status screen
  (codes 0..63, bits 6/7 never set) switches the P4 to codes within a few
  milliseconds; a decoder change waits for the next status line (250 ms).
  A switch restarts the capture with or without the BitScrambler. Console:
  `FILL ... bus=raw-iq|codes c5dem= act= switches= bs_errors=`.
- Ownership: the CPU never touches the IQ or the codes before the ring on
  either chip. P4 core 0 fills the
  histories, copies rows to PSRAM and renders; core 1 decodes. The P4 never
  paces the C5.
- Boundaries: a C5 verified restart (OUTPUT, DEMOD, profile) or its menu
  stops TX and the clock; the P4 ring then stalls, the picture holds and
  `NO VIDEO DATA` shows until the clock returns. A DMA lap (decoder late by
  more than a ring) drops the line lock and searches again.
- A reset through USB (flashing) does not clear the P4's BitScrambler
  routing (`HP_SYSTEM.bitscrambler_peri_sel`): after raw IQ the code bus
  then went into a stopped BitScrambler and no data came until a reset by
  the button (2026-11-01). `cvbs_start` clears it (log: `receive still
  routed through the BitScrambler`).

### Decoder

Six-bit codes, 0 = sync tip, about 20 = blanking (`DAC_IDLE_CODE`):

- Slice level: midpoint of the measured sync tip (1.5-3.5 us into each valid
  hsync) and blanking (8-9.2 us, after the burst), on 4-sample sums.
- Valid hsync: a falling crossing with the line still low 0.4-4 us later and
  blanking at 8-9.2 us, which rejects equalising (2.35 us) and broad pulses.
- Line clock: searched within +-1.5 us of the prediction; phase pulled half
  way to each valid edge, period by 1/32 of the error; it coasts through the
  vertical interval and missing pulses, unlocks after 40 missed lines.
- Vertical sync: accepted near the expected line count (+-6 lines) only,
  so dark picture or dropouts read as broad pulses cannot restart a field;
  after three fields without one, or a capture restart or lost line lock,
  the next one anywhere. Until 2026-11-01 that re-acquisition still ignored
  a field's first 100 lines: after a C5 menu on/off the progressive menu
  could stay shifted for good, and video crawled half a line a field
  until its vsync passed line 100 (host test `g_start_line`).
- Field start: a line more than 35 % below the slice level; a broad-pulse
  run starting mid-line marks the half-line field. PAL: the whole-line field
  is the top one; NTSC: the half-line field. A frame is published after its
  second field (or every field if the parity does not alternate). Menu
  FIELDS swaps the two. Fields that keep one parity (the C5's menu raster,
  progressive in this build) go into the frame as their own rows and the
  PPA scales them twice as much; doubling their lines on the CPU doubled the
  rows to copy and the decoder fell behind.
- Standard from the line period (64 vs 63.56 us): PAL 576 rows from line
  23/336, NTSC 480 rows from line 21/283.
- Levels: black at blanking, white at blanking + 2.33 (PAL) / 2.5 (NTSC)
  times the sync depth; CONTRAST and BRIGHTNESS on top, as fixed-point
  weights for the PIE routine. Each line starts at its nearest sample
  (+-25 ns); `[1 2 1]` on colour lines damps the subcarrier (-30 dB at
  4.43 MHz, -15 dB at 3.58 MHz).
- 10 MS/s for sync and luma: each history sample is the sum of two
  successive 20 MS/s codes, a free low-pass (about 5 MHz of luma).
- Clicks (black streaks in highlights, 2026-11-01): U80P64-R1 clicks,
  the same with the C5 or the P4 decoding. In the host model
  (`research/p4_clicks.py`, `research/p4_luma_clicks.py`) a click is one
  20 MS/s code ~13 low with ~5 high on either side (phase kept), which a
  `[1 3 3 1]` history and `[1 6 1]` on lines without a burst cancel. On 84
  real C5 snapshots (`research/p4_board_clicks.py`, gains 50-69, VTX at the
  bench) dips of 10+ codes hit ~3 % of picture samples, a third of them 2-5
  codes long, and the longer ones carry 0.7-1.4 turns of phase, about half
  still 0.5 turn or more net: FM threshold clicks (cycle slips), which a
  linear filter only spreads. That filter was tried on the board and showed
  no visible gain (operator), so it is not in. Another camera/VTX pair
  shows hardly any (operator): the first one's signal may be the cause
  (its deviation read 1.24). Remedies, if needed: concealment
  (a dip far below both sides, flagged by the IQ radius) or a demodulator
  with a lower threshold in the P4's BitScrambler.
- Colour (PAL and NTSC), on the 20 MS/s codes (at 10 MS/s the demodulated
  chroma's 2 fsc image would alias to about 1 MHz): per line the burst and 64
  windows across the active line (one per 8 pixels), each correlated with
  zero-mean sin/cos tables over two subcarrier cycles, so luma and the 2 fsc
  term cancel. The line's own burst gives the carrier phase; PAL's V switch
  comes from the burst's +-45 degree swing between successive lines (a vote
  keeps the alternation), then the PAL delay line (two lines' mean) cancels
  phase errors. The burst amplitude sets the colour gain; a line without a
  usable burst stays grey. YUV420 keeps Cb on even rows and Cr on odd ones,
  so the two fields of a frame carry one each. SATURATION defaults to 30 %:
  the FPV link's chroma is noisy, and at 100 % (nominal) the noise showed
  (operator, 2026-10-31).
- CPU: a P4 load costs several cycles (p4usb found ~3-5), so scalar
  per-pixel loops ran 10-20 times slower than their instruction counts
  (the luma row ~23 cycles a pixel, a chroma multiply-accumulate ~17). The
  hot loops use the P4's PIE vector unit, 16 bytes an instruction: the
  fill (55 -> 30 % of core 0), the chroma correlations and the luma row.
  Rows go to PSRAM through a copier task on core 0, since a 768-byte copy
  into busy PSRAM stalled the decoder 3.3-5k cycles (esp_async_memcpy cost
  39k cycles a call). Scaling on the CPU managed 11 pictures/s; the PPA does
  it in ~17 ms, so the panel shows all 25.

PSRAM bandwidth (board, 2026-10-31): the panel scans its frame buffer out
of PSRAM 61 times a second. With the PPA running, writing the decoder's
whole 590 KB frame back from the cache in one burst at the end of each frame
starved the scanout now and then: a panel frame took 31.7 ms instead of
16.4 and showed white (about 6 a minute; 0 in 90 s without that burst).
The frame now goes back in 32-row blocks as fields finish them (0 late
frames in 2 minutes). Not the cause: the 1.2 MB cache invalidates (19 us
each). No help: rate-limiting the PPA's DMA2D, 32-byte PPA bursts (worse).
The console's `PANEL late_frames=` line counts frames that ended more than
1 ms late.

The continuous receive and the DMA position follow zerowidth's `p4usb`
decoder: the AXI-GDMA descriptor register names the next node as soon as
the current one starts, so data is complete up to the start of the node
before it (the same effect was measured on the C5's own RX ring with a
marker probe on the `waveshare-c5-lcd-1.47` branch).

## Wiring

| Signal | C5 (FireBeetle) GPIO | P4 GPIO | P4 header JP1 pin |
|---|---|---|---|
| D0 (RAW: Q0; codes: LSB) | 2 | 2 | 9 |
| D1 (Q1) | 3 | 3 | 11 |
| D2 (Q2) | 4 | 4 | 13 |
| D3 (Q3, Q sign) | 5 | 32 | 19 |
| D4 (I0) | 6 | 33 | 21 |
| D5 (I1; codes: MSB) | 7 | 47 | 10 |
| D6 (I2) | 26 | 46 | 12 |
| D7 (I3, I sign) | 27 | 45 | 14 |
| Clock | 8 | 20 | 17 |
| UART C5 TX -> P4 RX | 11 | 38 | - (UART0 RXD) |
| UART P4 TX -> C5 RX | 12 | 37 | - (UART0 TXD) |
| GND | GND | GND | 5, 6 or 16 |

Until 2026-10-31 the bus had six data bits and the UART used P4 GPIO46/45;
the raw-IQ wiring moved the UART to GPIO37/38 (UART0's default pins) and
gave GPIO46/45 to C5 GPIO26/27. The P4 drives them with UART1 through the
GPIO matrix. Its console stays UART0 primary with USB-Serial-JTAG secondary
(read there): USB-Serial-JTAG as the primary console boot-looped this board
(cache error during startup, 2026-11-01). So the P4's boot log goes out on
GPIO37 until the link starts; the C5 only acts on whole command lines and
ignores it.

Keep the wires short (a few cm) and run a ground with the bundle, ideally
next to the clock. The P4 pins avoid what the JC1060P470C already uses:
GPIO0/5 (LCD reset), 1 (the power button of the camera example), 7/8
(touch/codec I2C), 21-23 (touch INT/RST, backlight), 26/27 (RS-485), 9-13
and 48 (audio), 28-31/34/35/49-52 (Ethernet), 39-44 (TF card).

GPIO39-48 of the P4 are powered by its on-chip LDO VO4 (also the TF card's
supply); the P4 firmware sets it to 3.3 V before it uses GPIO45-47.

Power: the FireBeetle from its own USB-C, or 3.3 V from JP1 pin 1/3 to its
3V3 pin with its USB-C unplugged. Power both boards together: the C5's
outputs should not drive an unpowered P4.

### The C5 side in this build

On the FireBeetle and its IO board the link takes GPIO2..8, 26/27 and
11/12 (GPIO2..7 are the same pins as a resistor DAC, LSB first, so an
XIAO-network DAC may stay connected for a composite output in P64/U85Q).
The IQ loopback (`RF_IQ_GPIO_NUMS` in `main/rf.h`) uses the pads left:
Q = 1/9/10/23, I = 15/28/24/25 (the GDI display connector's I2C, SPI and
control lines with no display fitted, IO1, the LED and BOOT). The LSBs and
I1 take the loaded pads: IO1 (the battery-voltage divider), GPIO15 (the
LED, which glows dimly) and GPIO28 (the BOOT button); the sign bits are on
clean pads. **The BOOT button does nothing in this build (it would short
an IQ lane: do not press it while running); the GDI connector and microSD
stay empty.** Kept free: GPIO0 (switches the 3V3_C output), 13/14 (USB:
console, web menu, `tools/flash.py`). Until 2026-10-31 the loopback was
Q = 1/9/10/23, I = 27/24/25/26. Board check 2026-11-01 with a PAL carrier:
`SPHASE state=settled excess_mc2=89` (limit 300). This loopback set is not
the one of the `dfrobot` branch (GPIO8..14/23), which also used 11/12 and
the USB pads.

The C5 menu and boot screen always use the progressive raster in this
build (`menu.c`): the interlaced descriptor chain, when it fitted, left the
gain tasks no heap at boot on the FireBeetle (no PSRAM; `gain_obs`/
`gain_fast` `ESP_ERR_NO_MEM` boot loops). With it, and the UART link started
after the splash, `heap_min` is about 9.8 KB (2026-10-31). The P4
line-doubles progressive fields. Build from `sdkconfig.p4link` (or delete a
stale `sdkconfig`): `CONFIG_FREERTOS_IN_IRAM=y` from an older configuration
also boot-looped it.

GPIO2 is a C5 strapping pin and goes to P4 GPIO2; if the C5 ever fails to
boot normally with the link wired, check that pin first.

## Build and flash

C5 (ESP-IDF 6.0.2, as CI):

```sh
idf.py -B build-p4link -DSDKCONFIG=sdkconfig.p4link "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.p4link.defaults" build
python tools/flash.py PORT --build build-p4link
```

P4 (ESP-IDF 5.5, as the board vendor's examples; the chip is an early
revision, `CONFIG_ESP32P4_SELECTS_REV_LESS_V3`):

```sh
cd p4
idf.py build
idf.py -p PORT flash monitor
```

Flash the P4 through its full-speed USB-C (USB-Serial-JTAG, the console as
well). The other USB-C (the P4's high-speed PHY) is the webcam "C5VRX
Receiver", MJPEG 800 x 600 at 30 fps (PAL pictures repeat to fill 30).

Host test of the P4 decoder:

```sh
cd p4/test
cc -O2 -std=c11 -Istub -I../main test_cvbs.c -lm -o test_cvbs && ./test_cvbs
```

## Screen

Left 800 x 600: the picture (PAL 576 or NTSC 480 lines onto 600 rows; the
800 columns cover the 52 us active line, so 4:3). Short notices at the bottom:
`NO C5 LINK`, `NO VIDEO DATA` (no bus clock), `NO SYNC`, `SCANNING`.

Right 224 px, status: channel and MHz, VIDEO/NO VIDEO and the detected
standard, signal bar, C/N and gain index, AFC state and offset, decoded
pictures per second (and USB frames per second while a host streams).

Touch rows (receiver page; each sends a command to the C5, which saves its
settings as before):

| Row | Action | C5 command |
|---|---|---|
| CHANNEL - / + | channel within the band | `C<idx>` (new, tunes directly) |
| BAND - / + | band, same channel number | `C<idx>` |
| GAIN | next gain model | `G` |
| FILTER | next filter profile | `f` |
| VIDEO STANDARD | AUTO / PAL / NTSC | `v` |
| SCAN, AFC | band scan; AFC on/off | `s`; `A` (new) |
| C5 MENU, MORE > | opens the C5's own menu, which then shows in the picture | `o` |

While the C5 menu is open the rows become NEXT (`n`), SELECT (`x`) and EXIT
(`e`), so every C5VRX-5 menu item stays reachable.

MORE > (display page, kept in the P4's NVS unless noted): BACKLIGHT,
CONTRAST, BRIGHTNESS, SATURATION (0 = B/W, default 30 %), CAL QUICK / CAL
FULL (`F` / `B` on the C5), CLK RISE/FALL (the P4's sampling edge; restarts
the capture) and FIELDS (swap the field order), the decoder (`m` on the C5:
U80P64-R1 -> U85Q -> RAW, shown as `P4 P64`) and BACK. OUTPUT stays in the C5's own menu.

## Link protocol

UART, 115200 8N1, text lines (`main/p4link.c`, `p4/main/link.c`):

- C5 -> P4: `S idx= ch= f= lk= vid= std= det= sig= gm= g= menu= busy= scan= sp= prof= dem= out= cn= afc= cfo= dc= up=` every 250 ms
  and on `?`; `L <idx> <band> <MHz>` for each channel on `L`.
- P4 -> C5: `K<key>` (a console key from the set `o e v f n x c b s g G m`,
  or `F`/`B` for calibration), `C<idx>` (tune), `A` (AFC on/off), `?`, `L`.

Unknown lines are ignored on both sides (the C5's ROM boot log also goes
out on GPIO11, the P4's on GPIO37). The C5 acts only on whole lines (`A`,
`?`, `L` alone, `K` plus one key, `C` plus digits). `dem=RAW` tells the P4
that the bus carries raw IQ. Commands go through the same `s_commands`
queue as the USB console, so the same busy/calibration refusals apply.

## Open items

Board checks, in order:

1. C5: boot, `P4LINK ... ready`, `T` heap (`heap_min`) and the C5 menu
   opening (internal SRAM is as tight as on the XIAO without PSRAM; the
   UART adds a 3 KiB task and a 256 B buffer), IQ sampling phase on the
   loopback pads (`T` SPHASE, especially IO1 and GPIO27 with their loads),
   reception against a XIAO.
2. P4: the panel and touch (init and pins as the vendor example), the
   PARLIO RX on a 40 MHz external clock over the wires (`NO VIDEO DATA`
   gone; then CLK RISE vs FALL), the DMA position method on this board.
3. Picture: lock, field parity (FIELDS), vertical/horizontal position,
   levels with each OUTPUT setting, both standards, the C5 menu raster.
4. UVC on the high-speed port with a host (Windows/macOS/Linux).

Raw IQ, next: compare RAW and C5-P64 pictures by eye and at weak signal;
NTSC; then P4-side demodulators (other BitScrambler programs, PIE after
them). The C5's `heap_min` with RAW is 7956 B (9.8 KB with the code bus;
2026-11-01): the menu still opens, but watch it.

Not done: CI for the `p4/` project.

## Credits

- C5VRX-5 receiver: Twotoz and the C5VRX contributors.
- zerowidth's `p4usb` fork (https://github.com/zerowidth/C5VRX, branch
  `p4usb`, commit 3dfac08): the continuous PARLIO RX receive on the P4 with
  the descriptor-register DMA position (`p4usb/main/decode.c`), adapted in
  `p4/main/cvbs.c`; its use of the P4's BitScrambler in that receive
  path (`start_bs`), followed for the raw-IQ decoder; its decoder's
  4-sample sync sums; its PIE kernels
  (`chroma.S`, `luma.S`) as the model for `corr.S` and `luma.S`; the P4
  USB/UVC and revision settings as a reference. Its architecture (raw IQ to the P4,
  own C5 firmware) is the alternative discussed above.
- Panel init sequence, DSI timing, MIPI LDO and backlight: Guition's
  JC1060P470C `video_lcd_display` example (Espressif example code,
  CC0-1.0). The settings column's style and 5x7 font follow the board
  owner's settings panel in that example (`ui_panel.c`).
