# P4 link: FireBeetle 2 ESP32-C5 tuner + ESP32-P4 display

The C5VRX-5 receiver runs on a DFRobot FireBeetle 2 ESP32-C5 (DFR1284,
ESP32-C5-WROOM-1U-N4: 4 MB flash, no PSRAM, U.FL antenna) exactly as on the
XIAO (gain, AFC, calibration, DC servo, U80P64-R1/U85Q, menu). Its six-bit
DAC codes leave on header pins together with the PARLIO TX clock, and a UART
carries status and commands. A Guition JC1060P470C (ESP32-P4, 1024 x 600
MIPI-DSI LCD, GT911 touch) samples that bus, decodes the composite signal,
shows the picture 4:3 on the left 800 x 600 of its screen with a touch
settings and status column on the right 224 px, and streams the same
picture as a USB webcam (UVC, MJPEG 800 x 600) on its high-speed USB port.

Status (2026-10-31, bench, one FireBeetle + one JC1060P470C, PAL VTX on
B5): the UART link, status column and touch controls work; the P4 captures
the code bus and its decoder locks at the full PAL rate (15,625 lines/s,
50 fields/s, 25 pictures/s shown, no overruns, decode task ~64 % of core 1).
Picture quality, NTSC, UVC and range are not checked yet. The P4 decoder
also passes a host test on synthetic PAL and NTSC (`p4/test/test_cvbs.c`:
lock, standard, field parity and row order, levels, horizontal position,
+-300 ppm clock, noise up to +-12 codes). See [Open items](#open-items).

## Why this split

The P4 could also take the raw IQ and demodulate it (zerowidth's `p4usb`
fork does that with its own minimal C5 firmware and 7+7 IQ lanes). Here the
C5 keeps all of C5VRX-5 and the P4 receives its demodulated output:

- The C5VRX-5 control loop (Direct Gain/LEVEL, AFC, DC servo, calibration,
  profiles) is hardware-proven on the C5's own IQ ring; it stays where it
  was measured, unchanged.
- The code bus is source-synchronous (data and clock from the same PARLIO
  TX unit, data changing on the falling edge): no sampling-phase search on
  the P4, unlike raw MODEM_DIAG lanes that change at 80 MS/s.
- It fits the pins: the P4 board's 26-pin header has nine free GPIOs, which
  is six data bits + clock + UART RX/TX.
- No C5VRX-5 code path changes outside `#ifdef C5VRX_P4LINK`: no gain-tick,
  task-priority or AFC-period changes as in the Waveshare LCD build, so the
  codes on the link are exactly the XIAO's DAC codes.
- The picture the P4 shows is the same waveform the XIAO's resistor DAC
  outputs; picture-quality findings carry over in both directions.

The price: the P4 sees six-bit codes at 20 MS/s, not IQ, so a better
demodulator on the P4 is not possible over this link. The TX unit is eight
bits wide; a raw-IQ mode would need two more P4 pins.

## Pipeline

```text
C5:  MODEM_DIAG Q4/I4 40 MS/s -> PARLIO RX -> raw 32 KiB ring              (unchanged)
       -> TX BitScrambler U80P64-R1 / U85Q (two lookups, unique CVBS20 as [D,D])
       -> PARLIO TX 40 MHz: codes on GPIO2..7, clock out on GPIO8         (clock pin added)
     p4link.c: UART1 115200, status every 250 ms, commands -> s_commands
P4:  PARLIO RX, external 40 MHz clock, 8-bit, 32 x 4032 B ring (3.2 ms, internal RAM)
       -> core 0 fill task: bytes 0+2 of each 32-bit word summed -> 32 Ki-sample history (10 MS/s)
       -> core 1 decode task: sync slicer, line PLL, broad pulses / field parity, active line
          at source resolution (512 samples, sub-sample phase) through the level table
          -> woven RGB565 frames (PSRAM, 3 buffers, written back once per frame)
       -> PPA: 512 -> 800 columns (25/16), 480 -> 600 / 565 -> 600 rows (20/16, 17/16) into
          one of two DPI frame buffers (and the UVC frame); core 0 adds overlays and the column
          swapped on the panel's refresh; settings column; same rows to the UVC frame
       -> UVC task: hardware JPEG (grey, q80) of the newest frame, 30 fps paced
```

- Source/tap: the C5's TX output (the DAC codes), not IQ. Acquired rate
  40 MHz bus, 20 MS/s unique codes. Ring: 129 KiB of whole 4032-byte DMA
  nodes, partial receive, EOF every eight nodes.
- Ownership: the C5 CPU never touches the codes; P4 core 0 copies the ring
  into the history (one 32-bit store per four ring words), core 1 decodes, core 0
  renders and serves USB. The P4 never paces the C5.
- Boundaries: a C5 verified restart (OUTPUT, DEMOD, profile) or its menu
  stops TX and the clock; the P4 ring then stalls, the picture holds and
  `NO VIDEO DATA` shows until the clock returns. A DMA lap (decoder late by
  more than a ring) drops the line lock and searches again.

### Decoder

Six-bit codes, 0 = sync tip, about 20 = blanking (`DAC_IDLE_CODE`):

- Slice level: midpoint of the measured sync tip (1.5-3.5 us into each valid
  hsync) and blanking (8-9.2 us, after the burst), on 4-sample sums.
- Valid hsync: a falling crossing with the line still low 0.4-4 us later and
  blanking at 8-9.2 us, which rejects equalising (2.35 us) and broad pulses.
- Line clock: searched within +-1.5 us of the prediction; phase pulled half
  way to each valid edge, period by 1/32 of the error; it coasts through the
  vertical interval and missing pulses, unlocks after 40 missed lines.
- Field start: a line more than 35 % below the slice level; a broad-pulse
  run starting mid-line marks the half-line field. PAL: the whole-line field
  is the top one; NTSC: the half-line field. A frame is published after its
  second field (or every field if the parity does not alternate). Menu
  FIELDS swaps the two.
- Standard from the line period (64 vs 63.56 us): PAL 576 rows from line
  23/336, NTSC 480 rows from line 21/283.
- Levels: black at blanking, white at blanking + 2.33 (PAL) / 2.5 (NTSC)
  times the sync depth; CONTRAST and BRIGHTNESS on top.
- 10 MS/s: each history sample is the sum of two successive 20 MS/s codes,
  a free low-pass (about 5 MHz of luma). At 20 MS/s the decoder took all of
  a core for half the data on the board (2026-10-31: a P4 load costs ~3-5
  cycles, as p4usb found); now the decode task uses about 64 % of core 1 at
  full PAL rate (15,625 lines/s, 50 fields/s, no overruns). Scaling on the
  CPU managed 11 pictures/s (77-98 ms each); the PPA does it in ~17 ms, so
  the panel shows all 25 pictures/s.
- Monochrome. The codes carry the colour subcarrier; decoding it is a
  possible next step (the pair sum damps it only a little: -2.3 dB at
  4.43 MHz, -1.5 dB at 3.58 MHz).

PSRAM bandwidth (board, 2026-10-31): the panel scans its frame buffer out
of PSRAM 61 times a second. With the PPA running, writing the decoder's
whole 590 KB frame back from the cache in one burst at the end of each frame
starved the scanout now and then: a panel frame took 31.7 ms instead of
16.4 and showed white (about 6 a minute; 0 in 90 s without that burst).
The decoder now writes back 32-row blocks as it finishes them (0 late
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
| D0 (code LSB) | 2 | 2 | 9 |
| D1 | 3 | 3 | 11 |
| D2 | 4 | 4 | 13 |
| D3 | 5 | 32 | 19 |
| D4 | 6 | 33 | 21 |
| D5 (code MSB) | 7 | 47 | 10 |
| Clock | 8 | 20 | 17 |
| UART C5 TX -> P4 RX | 11 | 46 | 12 |
| UART P4 TX -> C5 RX | 12 | 45 | 14 |
| GND | GND | GND | 5, 6 or 16 |

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

On the FireBeetle and its IO board the link takes GPIO2..8 and 11/12 (the
same GPIO2..7 as a resistor DAC, LSB first, so an XIAO-network DAC may
stay connected for a composite output next to the P4). The IQ loopback
(`RF_IQ_GPIO_NUMS` in `main/rf.h`) moves onto pads nothing else uses here:
Q = 1/9/10/23, I = 27/24/25/26, which are the GDI display connector's
I2C, SPI and control lines (no display fitted) and IO1. The LSBs take the
two loaded pads: IO1 (the battery-voltage divider) and GPIO27 (GDI LCD_CS).
**Leave GPIO26/27 on the IO board unconnected and the GDI connector and
microSD empty.** Kept free: GPIO0 (switches the 3V3_C output), 13/14 (USB:
console, web menu, `tools/flash.py`), 15 (LED), 28 (BOOT, still next
channel / menu). This loopback set is not the one of the `dfrobot` branch
(GPIO8..14/23), which also used 11/12 and the USB pads.

The C5 menu and boot screen always use the progressive raster in this
build (`menu.c`): the interlaced descriptor chain, when it fitted, left the
gain tasks no heap at boot on the FireBeetle (no PSRAM; `gain_obs`/
`gain_fast` `ESP_ERR_NO_MEM` boot loops). With it, and the UART link started
after the splash, `heap_min` is about 9.8 KB (2026-10-31). The P4
line-doubles progressive fields. Build from `sdkconfig.p4link` (or delete a
stale `sdkconfig`): `CONFIG_FREERTOS_IN_IRAM=y` from an older configuration
also boot-looped it.

GPIO2 is a C5 strapping pin and goes to P4 GPIO4; if the C5 ever fails to
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
CONTRAST, BRIGHTNESS, FIELD ORDER (swap), CAL QUICK / CAL FULL (`F` /
`B` on the C5), CLK RISE/FALL (the P4's sampling edge; restarts the
capture) and the C5's decoder (`m`, P64/U85Q), OUTPUT (`g`), BACK.

## Link protocol

UART, 115200 8N1, text lines (`main/p4link.c`, `p4/main/link.c`):

- C5 -> P4: `S idx= ch= f= lk= vid= std= det= sig= gm= g= menu= busy= scan= sp= prof= dem= out= cn= afc= cfo= dc= up=` every 250 ms
  and on `?`; `L <idx> <band> <MHz>` for each channel on `L`.
- P4 -> C5: `K<key>` (a console key from the set `o e v f n x c b s g G m`,
  or `F`/`B` for calibration), `C<idx>` (tune), `A` (AFC on/off), `?`, `L`.

Unknown lines are ignored on both sides (the C5's ROM boot log also goes
out on GPIO11). Commands go through the same `s_commands` queue as the USB
console, so the same busy/calibration refusals apply.

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

Not done: colour decoding, a raw-IQ mode, CI for the `p4/` project.

## Credits

- C5VRX-5 receiver: Twotoz and the C5VRX contributors.
- zerowidth's `p4usb` fork (https://github.com/zerowidth/C5VRX, branch
  `p4usb`, commit 3dfac08): the continuous PARLIO RX receive on the P4 with
  the descriptor-register DMA position (`p4usb/main/decode.c`), adapted in
  `p4/main/cvbs.c`; its decoder's 4-sample sync sums; the P4 USB/UVC and
  revision settings as a reference. Its architecture (raw IQ to the P4) is
  the alternative discussed above.
- Panel init sequence, DSI timing, MIPI LDO and backlight: Guition's
  JC1060P470C `video_lcd_display` example (Espressif example code,
  CC0-1.0). The settings column's style and 5x7 font follow the board
  owner's settings panel in that example (`ui_panel.c`).
