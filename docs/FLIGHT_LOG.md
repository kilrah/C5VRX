# Flight log (design, C5VRX-5)

C5VRX by Twotoz and the C5VRX contributors. Status: built (PR #224): `main/flightlog.c`, `main/flightlog_format.h`,
`tools/flight_log.py`, web flasher Terminal page "Flight log"; host tests
`test/test_flightlog.c`, `web/flight-log.test.mjs`.

"Some boots are great, some terrible" happened in the field, without a
laptop. A log has to answer four questions afterwards, and nothing goes in
that does not help answer one of them:

1. How good was the link, and when did it go bad?
2. Was the receiver in its best state then, or did a fixable impairment
   (DC, gain, filter, sampling phase, AFC, profile) cost dB?
3. What changed just before (boot, channel, profile, calibration, temperature,
   a reset)?
4. What did the raw signal look like at the worst moments?

## Storage

A `flightlog` data partition of 1.5 MiB (the 8 MiB flash uses ~3.1 MiB):

| region | size | content |
| --- | ---: | --- |
| header | 4 KiB | magic, layout, format version |
| record ring | 1 MiB - 4 KiB | sessions, per-second summaries, events |
| snapshot ring | 512 KiB | 64 IQ snapshots (64-byte head + 8128 IQ bytes) |

Records are buffered in RAM and written as one 4 KiB block (about every two
minutes); a sector is erased only when the ring wraps into it, as often as
the gain map is already saved. Video is hardware and never waits for flash.
A battery pulled mid-write loses at most the unwritten block.

## Versioning and robustness

- **Header** (first sector): magic `C5LG`, `log_format`, region sizes. A
  firmware that finds another layout reformats the partition once (the log
  is diagnostic data, not configuration).
- **Block** (4 KiB): magic, `log_format`, session id, block sequence number,
  uptime of the first record, record count, CRC-32 over the block. The head
  is found at boot by the highest valid sequence number; a block with a bad
  CRC is skipped, never trusted.
- **Record**: `type` (1 byte), `length` (1 byte), payload. A reader skips
  unknown types by their length; a newer firmware only appends fields at the
  end of a payload, never reorders or reuses them, so an old reader still
  reads the fields it knows. `log_format` changes only when that rule cannot
  be kept.
- **Units** are fixed per field below and never change meaning.

## Session record (once per boot)

What the receiver was, how it was set and calibrated, and why it started.

| field | why |
| --- | --- |
| session id (counter), `log_format` | order sessions without a real-time clock |
| firmware version (`5.0.N`), git hash, build flags | which code produced the log |
| `CAL_GENERATION`, filter blob version, DC index version | which calibration semantics |
| reset reason (power-on, brownout, watchdog, panic, software) | a brownout in flight explains a black screen without any RF cause |
| chip revision, board id (hash of the MAC, not the MAC) | unit-to-unit differences without identifying a person |
| PHY context: gain table maximum, band5, IQ scale | the DC rows and the gain table the log refers to |
| settings: channel, standard mode, output level, gain mode, flywheel (removed 2026-10-25, always 0), live profile switch, AFC sign | user settings that cost range (fixed gain) are visible |
| calibration this boot: what ran (filter, DC rows per filter, sampling phase, deviation), result, duration | "this boot calibrated" vs "loaded" |
| filter settings NORMAL/NARROW/WIDE (bw20, code, skirt, width, noise bandwidth) | calibration-to-calibration differences |
| DC tables: rows, template rows, ready per filter; repair flags (retired, always 0) | a stale or partial table |
| sampling phase: state, chosen position, contrast | the per-boot choice our noise model cannot see |
| boot DC (before the servo) and temperature | the boot offset the servo had to remove |

## Carrier record (when the VTX or its measurement changes)

What was received, so a log can be compared across VTXs and cameras.

| field | why |
| --- | --- |
| channel, frequency, AFC offset applied | where the receiver listened |
| porch frequency (raw, kHz), AFC residual | the VTX's real frequency and its drift |
| deviation (x100) and the batches it came from | which profile fits; a VTX outside the profiles |
| profile chosen and reason | why MATCHED / DEFAULT / WIDE |
| standard (PAL/NTSC), colour burst present, line period, sync width, polarity | camera type and settings |

Written at the first carrier of a session, after every deviation measurement
that changes the result, and on a channel change.

## Per-second summary (34/38/42/50 legacy bytes; 69 with output telemetry)

One per second while the receiver runs; about 4 hours in the ring at the
current 71-byte payload (less when rare events occupy blocks).

| group | fields (unit) |
| --- | --- |
| time | uptime (s) |
| link | `VIDEO_Q` SNR (0.1 dB), lines found per window (x100), C/N estimate median (0.1 dB), share of windows with sync (%), dropout category |
| gain | min / median / max index, max clip (per mille / 4), gain writes, overloads |
| DC | residual I and Q of the active row (0.05 cells, signed), offset I and Q of the active row (codes), servo corrections |
| AFC | offset applied (kHz), porch residual (kHz), corrections |
| demodulator | analog filter profile, reserved historical kp/LP slots (P64 writes unknown kp and LP off), sampling-phase state and last probe excess |
| received level | carrier at the chip pin (0.1 dBm), the noise it is measured against (dBm), flags: valid, above the noise, exact (G54+), noise learned (`main/rx_level.h`) |
| picture (71-byte records, since 2026-10-28) | sync pulses found / expected in the decoded control windows (%, `main/picture_sync.h`: the live decoder's LUT in software, a goggle-like count with none of the AFC's gates; 0xff unknown) and how many windows. The link row's share of windows with sync is the AFC's: it reads 20-40 % at the edge and 60-90 % on a strong carrier while moving, where the picture is clean |
| gain write reasons | writes down (attack), writes up (release) in every gain model (LEVEL, RSSI and fixed gains count their applied writes; before 2026-10-28 only KEEL-1 counted, so LEVEL logs read 0 here and in `gain_writes`), fast-path overloads forwarded (sentinel), overloads within 5 ms of a write |
| C/N-AUTO | decision C/N plus power, M2M4 and envelope estimates (0.1 dB, -990 unknown), confidence and source (+32: a small carrier on a 4-bit-limited row, not used for decisions), flags (`main/cn_auto.h`); only noise-referenced power may drive AFC/profile decisions; kp/LP consumers are retired |
| verified output | verified demodulator ID: 3 = `U80P64-R1`, 4 = `U50P64-R1`, 5 = `D32-R2`; historical IDs 1/2 = `U85Q`/`U85Q-LP` (instruction hash `1a09f5c092e8`), applied output level (0..9 manual, `0x80 | gain x8` with AUTO OUTPUT; the tool adds `output_auto_gain`); flag format, guard standard and requested/active flywheel are history (removed 2026-10-25, read 0); identity is unknown unless the live transport state is verified |
| output observer | history (removed with the flywheel 2026-10-25; current firmware writes 0, lock 0xff): task-side samples and fresh samples, native H/V trust (% of fresh samples), native in-phase H lines, repaired and suppressed sync tips, longest continuously observed native-H unlock (ms), observer gaps |
| environment | temperature (degC) |
| flags | menu, scan, calibrating, paused, fault latched, flywheel active, status screen |

Counters are per second (saturating), not per event. The observer is sampled
from existing cumulative RMT/ISR counters by the 2-ms control task; the new
logging adds no line-rate or ISR work. Missing/stale observation is
`UNKNOWN`: it lowers observer coverage and ends an unlock run, but is never
counted as an unlocked picture. `VIDEO_Q` keeps its existing AFC-oriented
line/evidence meaning; it is not renamed or reinterpreted as output lock.
Neither metric proves lock in a particular monitor or goggles.

Readers take any record of 34 bytes or more. Records shorter than 69 bytes
have unknown output identity, coverage and lock; they are not filled with a
false zero measurement.

## Events (rare, ~20-200 bytes)

uptime (ms) and type, then:

- channel change (from, to, why: menu, console, scan), profile switch (from,
  to, reason, deviation); historical kp/LP switch records remain readable
  but are no longer emitted;
- calibration start/end (manual or startup, what, result), `DC CAL NEEDED`
  (reason, filter), gain mode change, menu open/close;
- fault or latch (DC restore, transport rollback, live off);
- output-observer gap (count and current validity bits), and a native-H
  unlock that remains continuously observed for at least 100 ms;
- **dropout onset**: the existing 16-before/16-after trace at 50 ms
  (`diag_trace.h`), and its end with duration;
- snapshot taken (index, trigger);
- **gain step down** (`attack`, at most 4 a second): source (observer or
  sentinel), from and to row, the window's P95/P50, clip and rails, time since
  the previous write; since 2026-10-28 also that window's fold radius
  (agc_level_fold_radius): the radius LEVEL could take (x16 cells, 0 none),
  the best radius before the fit check (0: the row too noisy to estimate) and
  the fit (1/192 nat per sample, taken at >= -38). LEVEL logs it only
  (`LEVEL_FOLD_RADIUS` 0 in agc.c).
- **AUTO OUTPUT applied** (`output_auto`): gain x8, source sync-tip code,
  sync depth in DAC codes x4 (`main/output_auto.h`);
- **receiver pause ended** (`pause`): duration in ms and the first eight
  characters of the owner tag (`SCAN`, `DCO`, `SPHASE`, ...), so a second
  marked owned or paused names who held the receiver.

## IQ snapshots (64 x 8 KiB ring)

8128 bytes of contiguous IQ (the console `I` format, Ihigh_Qlow, 40 MS/s; 62
bytes shorter than `I` so a slot is exactly two sectors)
with a header: session, uptime, trigger, gain, frequency, filter key,
profile, reserved historical kp/LP slots, DC offsets of the row, sampling-phase position, and the
verified output program/level/validity snapshot.

When the record ring is full the oldest block is erased and reused: the most
recent ~4 hours are always there at the current record size. The snapshot ring keeps the moments that
went wrong longest: a new snapshot takes an empty slot, else the oldest slot
of the lowest priority not above its own (dropout/output unlock > picture drop > first
carrier > periodic); a periodic one never replaces a dropout and is skipped
(`snapshots_skipped` in `Y`) when every slot is more important. A request is
also dropped, and counted, when the heap has no block for the IQ copy plus
8 KiB left free for the menu (`snapshots_no_memory`), or when the receiver
is busy (menu, pause, startup) or its gain is settling (`snapshots_refused`).

Triggers, at most one per 10 s. A request that arrives inside the gap waits;
a more important one replaces it, at equal priority the first one stays:

- dropout onset or a continuously observed native-H unlock of at least 100 ms;
- `VIDEO_Q` more than 6 dB below its 30-second median;
- the first carrier of a session (reference), and one every 10 minutes.

Snapshots can be replayed with the recorded demodulator identity: use the
P64 reference in `research/p64_model.py` for U80P64-R1 and the archived
U85Q/U85Q-LP references for historical logs. Keep the captured context and
model identity with any regression case; do not decode P64 as U85Q.

## Deliberately not logged

- Per-window or per-probe data and every gain write: the per-second summary
  and its counters answer the questions; more is noise.
- Anything that can be derived from other fields.
- The MAC address, location or anything else that identifies a person.

## Reading it

- Web flasher, Terminal page, "Flight log": read it over USB (the receiver
  keeps running), a table per session, a chart (picture SNR, native H lock,
  gain, DC, temperature, dropouts), download `.c5log` (the partition image) and CSV,
  erase.
- Console `Y` status, `W` dump (`FLOG_BEGIN` / `FLOG_D <offset> <hex>` /
  `FLOG_END ... ms=<dump time> usb=driver|fifo heap=<largest free block>`),
  `X` twice within 3 s erases. The dump goes through the USB driver's 4 KiB
  ring buffer while it runs: a full log in 6.5-9 s with the receiver live or
  starting (board 2026-10-23). With the on-screen menu open there is no heap
  for it (`usb=fifo heap=1664`) and it uses the console's byte path, 10-13 s
  there. That path took about 40 s with the receiver live and lost lines.
- `tools/flight_log.py dump COM10 f.c5log` and `decode f.c5log dir`: CSV per
  session, JSON with boot, calibration, carrier and events, and the snapshots
  as `.bin` + `.json` for `research/sensitivity_model.py`.
- The partition table changed: flash the full image once (the web flasher's
  "Full firmware"); an app-only update leaves the log off and says so.
