# C5VRX-5 findings

C5VRX-5 keeps only what the C5VRX-4 research on PR #190 (branch
`codex/c5vrx4-pair-range`) showed to help the picture. The full study, its
scripts and data live in that branch under `v4/docs/PICTURE_CLIFF_STUDY.md`,
`v4/tools/dsp_search/` and `v4/docs/data/`. This page is the summary.
Synthetic results are host simulations; board results are labelled as such.
Measured range is not established.

## RX descriptor register one node ahead (board, 2026-10-30)

On the Waveshare ESP32-C5-LCD-1.47 build ([LCD147.md](LCD147.md)), bytes
painted into the two RX descriptors after the one `in_dscr_bf0` names, and
polled about every microsecond, showed in all six runs: the register names
node N+1 within 3 us of the GDMA writing node N's first byte. The register
is the descriptor already *fetched*, one ahead of the one being written.
The LCD grabber now takes the node before the register's as the write
position (`transport_rx_write_offset`); its black streaks, the earlier
`tb_bad` timebase disagreements and most wide-search fallbacks went away.

Not changed, needs a decision: `control_copy_plan()` (control windows,
probe, sentinel, AFC copies) treats the register's node as active and
copies the node before it as the newest completed one. By this
measurement that node is the one still being written: such a window can
end in samples from the previous ring lap (819 us older, 0.8 line out of
phase). Measured on one board and one channel; whether it moves gain, DC
or AFC results is not measured.

## U85Q on the board and a picture measure for the flight log (blackbox, 2026-10-28)

**U85Q against P64**, same firmware, LEVEL, VTX at 25 mW, NTSC, deviation
0.66 (walk 296 against walks 286-292 pooled), AFC-gated video % per 5 dB:
-90..-85 dBm 21 against 23, -85..-80 43 against 58, -80..-75 60 against 70.
No gain for this VTX (host model: equal at NTSC 0.7; P64 holds sync 1-2 dB
longer at deviation 1.0-1.4). U85Q stays selectable (menu DEMOD, console m),
P64 the default. Most of the range lost since 2026-10-20 is the VTX at 25
instead of 100 mW (6 dB, twice the distance), not the receiver: the pin level
where the picture goes moved 2-3 dB at most, within the meter's +-1-2 dB.

**video % is the AFC's, not the picture's.** A control window counts only
with a stationary envelope and timing-confirmed sync and porch pairs within
~3 lines, so fading, a gain write or one noisy sync fails it. Walk 296: a
clean picture to -85..-90 dBm by the operator's eye where video % read 33.
`picture_pct` (`main/picture_sync.h`) decodes every control window in
software with the live decoder's LUT and counts sync pulses one by one, as
a goggle's line PLL rides through missing ones. Walk 300 (U85Q): -90..-85
dBm 58 against 18 %, -85..-80 77 against 54, stronger 95-97 against 60-90;
the operator's desyncs and noise were at -90..-92 dBm, where picture_pct read
13-40 %. Cost: ~2 ms wall per window (~18 a second), with a yield every 512
pairs; the gain observer still took 2873 windows a second (~2700 without).

**Open.** 11 % of walk 300's seconds (and of earlier walks) were not
recorded because the receiver was owned or paused, up to 4-7 s in a row
(the second after such a stretch logged 85-98 picture windows, fixed); no
menu, retune or AFC step was logged then, so which pause it was is not in
the log yet.

## Next steps (2026-10-29)

Ordered by expected gain per effort; measure every change with `picture_pct`
and the operator's goggles, not video %.

1. **Log every receiver pause in the flight log** (who, when, how long).
   11 % of walk 300 was owned or paused, up to 4-7 s in a row; it may be
   part of the desyncs the operator saw.
2. **LEVEL at the edge:** fold memory (kept on branch-local work, off by
   default), the fold radius (built, logged, off), and a measured row table
   (pair sweep on the bench) instead of the model. Host: with 3 dB of
   row-model error LEVEL falls under KEEL-1 at the edge.
3. **OUTPUT level for a low-deviation VTX** (1.5x at deviation 0.66): the
   goggles' sync slicer sees a nominal amplitude. One menu setting.
4. **A decoder table for low deviation / radius-aware erasures**: host LUT
   search on the bit-exact model, then a board A/B through the DEMOD
   selection.
5. **Sync clean-up in the decoder** (recognised sync tips out clean, nothing
   invented): the removed flywheel did more and failed; this part alone was
   never tested.
6. **Open PHY items:** spur slots 2..3 (measurement incomplete), the AFC was
   off on the operator's board, the temperature never read (-128), LEVEL
   listens on G79 instead of G83 without a carrier.

Not reachable in firmware (board runs 2026-10-28, PHY_NOISE_RESULTS): the
~6 dB noise figure. LNA bias, current level, loop cap and spur slot 0 moved
the noise by no more than the shams. More sensitivity needs the external
LNA (+5 dB measured) or better antennas/diversity.

## LEVEL walks: blind counters, a frozen DC servo and folded windows (blackbox, 2026-10-28)

Walks on LEVEL, 5865 MHz NTSC: sessions 279 (before PR #287), 286 (write
counters) and 288 (DC servo fix).

**Blind counters.** Only the KEEL-1 core counted its writes: on LEVEL the
gain moved G20..G83 while `gain_writes`, `writes_attack/release` and the
attack events read 0. `apply_target` now counts every model's writes.

**Frozen DC servo on the top rows.** Session 286, VTX out of range: G83 for
12 s with 2.4-4.2 cells of DC and 0 servo writes. IQ snapshot: the receiver
noise cloud (2.1 cells per axis, as clean G83 noise) centred at +4.1/-3.7
cells, half on the rails. The passive health block demoted the sampling
phase to `unverified` on every block whose absolute HF statistic exceeded
300, and receiver noise on the top rows reads ~725 on a clean position; the
servo only corrects on a settled phase. A high block no longer demotes the
phase (only scans do) and NOISE_LIMITED no longer stops the servo. Session
288: the phase stayed settled all walk, G83 held 0.1-0.6 cells with 1-5
servo writes a second, and the worst window without a carrier clipped 0.7 %
instead of 4.6 %. The range cliff stayed at ~-85 dBm (video per 5 dB bin
within walk-to-walk spread): that walk never met a weak carrier on a
drifted top row, which is the case the fix removes.

**Video follows the gain, not the walking.** Seconds with a carrier over
sessions 279 and 286, video %:

| | gain held | gain moved |
| --- | ---: | ---: |
| strong (> -78 dBm), level steady | 89.6 (n=23) | 73.8 (n=29) |
| strong, level changing | 87.8 (n=8) | 72.5 (n=49) |
| edge, level steady | 60.2 (n=30) | 51.5 (n=13) |

The ~16 points (160 ms a second) are far more than the writes themselves
(1-5 a second); the attack events show what surrounds them.

**A fold steps down one home row at a time.** Session 286 attack events:
G83 -> G79 -> G75 -> G71 -> G68 in 100 ms, a fold (14-38 % of the samples
clipped, all four rails) before each step. The ultrafine lane keeps each
component's sign and folds its magnitude modulo 8 cells (+9 cells reads +1,
board PR #228), so a folded window reads small, and LEVEL counted it at its
own power + 3 dB: one home row lower each time. The host models folded all
four bits instead (`((c + 8) & 15) - 8`: +9 cells read -7, near the far
rail), where a folded window reads large; they now share the hardware model
(`test/adc_fold.h`). That alone hardly moves the host scores: with an exact
gain model LEVEL seldom folds.

**Fold radius.** An FM carrier is a circle, so the class probabilities of
(|I|, |Q|) tokens per radius on the exact fold model
(`research/fold_radius.py`, `main/fold_radius_table.h`, residual DC 0.3 cells
per axis included) give the carrier radius of a folded window by maximum
likelihood (`agc_level_fold_radius`). `test/test_fold_radius.c`: 96-100 % of
folded carriers (9-24 cells, noise 0.3-0.6 cells, 64 or 256 samples) within
half a cell, the rest no estimate (old reading), noise clouds 0-1 of 200.
LEVEL counts a folded window at that radius. `test_agc_compare`, hand on the
antenna (the folding case), without -> with:

| gain model | close | mid | score, writes/s |
| --- | --- | --- | --- |
| exact | 79.1 -> 81.5 % | 63.9 -> 68.6 % | 70.47 -> 70.62, 1.6 -> 1.4 |
| CI error (1 dB, low rows 6) | 77.8 -> 80.7 % | 63.9 -> 66.8 % | 70.07 -> 70.20, 2.0 -> 1.7 |
| 3 dB per row | 77.8 -> 80.7 % | 64.6 -> 69.6 % | 69.21 -> 69.40, 3.1 -> 2.6 |

Other scenarios within +-0.6 points; the edge is unchanged. With 3 dB of
gain-model error LEVEL falls under KEEL-1 at the edge (39.9 against 41.2 %
at 3 dB C/N, 9 writes a second at 6 dB): the board measures 13 rows from
receiver noise and models the rest, so a measured row table is the next
step.

**Board walk 290 with LEVEL acting on it: no gain, so LEVEL only logs it.**
Steps down from G77 and below (where the row noise allows an estimate) grew
from one home row (median 3 rows, 14 % over several homes, walk 286) to a
median 9 rows (59 %): the radius is used. But LEVEL climbed back (releases
50 -> 90 per walk, folds a median 77 ms after the previous write against
107) and folded again: folds at > -78 dBm 14.0/s (14.5 and 12.3 in walks 286
and 288), video while the gain moved 70.8 % (73.2, 71.3), and -85..-80 dBm
46 % (n=26) against 77 % (n=8) and 68 % (n=18). Either the radius
overestimates on board IQ (the 0.7-cell table separates noise from a circle
far less than the 0.35-cell one) or LEVEL's 200 ms history pulls it back to
the pre-fold rows. `LEVEL_FOLD_RADIUS` is 0: LEVEL reads folds as before,
and every attack event carries the radius it could have taken, the best one
before the fit check and the fit, to tell the two apart on the next walk.

## DC fault after a walk: vendor bank 1 copied into the hold (board, 2026-10-27)

**Symptom.** A walk on LEVEL (session 272) lost the picture at -50..-70
dBm (C/N 8-16 dB): DC 1.5-4.4 cells for seconds, video 23 %. Afterwards,
VTX off: G83 clip_pm 1000, the servo trim at +96/+71 (its limit), the guard
releasing G83 over and over (`DCO hold abort`), no DC search, nothing in
the menu. A manual calibration with the VTX off ended `CAL_RESULT
dc=incomplete why=need_vtx_off`.

**Evidence.** IQ snapshots per fixed row (VTX off): G83 I = -8 in all 8190
samples (the ADC at negative full scale, not 7.5 cells as the servo read),
G72 I half on the rail, G66 and below fine: one I offset ahead of the gain.
`DCO_BANKS`: bank 1 (the vendor's coarse DC words, blocks 2/3 bank 1) was
245/224; after a reboot 279/227, the same for every row G54-G83 (one RF
stage; G48 275/246, G42 270/258), and every row was fine again. The boot RF
calibration measured 279 again, so 245 was no physical change.

**Root cause.** `phy_dco_hold()` copied the live PBUS words, bank 1
included, into debug mode at every hold, and every gain write releases and
re-holds (`rf_set_gain`). The table's bank-2 codes are only valid on the
bank 1 they were measured with; a vendor rewrite of bank 1 while released
was frozen in and moved every row at once, beyond the servo's +-96 trim.

**Fixes.**
- `phy.c pin_bank1()`: bank 1 is pinned per DC table (channel, filter,
  gain span; `phy_dco_bank1_reset` from `dco_select`): the first hold takes
  it, every later debug entry (hold, searches, trials) forces it. `T`
  `DCO_BANKS ... bank1_pin= vendor_moved_to= overrides=`.
- The quiet test names IQ on one rail per axis `dc_rails` (DC pushing the
  cloud) instead of `rails` (a large signal on both): the DC calibration
  runs on it (`dc_searchable`) instead of stopping as `need_vtx_off`.
- A row the servo cannot hold is a DC fault: the guard released it, or its
  trim sits at the limit with more than 1.5 cells left. `DC CAL!` in the
  menu for 10 s after the last evidence, `DC_FAULT` in `T` and on the
  console, and one `dc_cal_needed` flight-log event per episode.

**After the fix** (session 275, same walk and a hand test): video 75.9 %
against 40.2 % (-70..-45 dBm: 76.0 against 23.3 %), largest DC 0.95 cells
against 6.35, no second with more than 1 cell, no DC fault. The recorded
part ends at 36 s (the open flight-log block was lost at power-off), so the
range edge is not in it.

**Where bank 1 comes from** (static analysis of the pinned libphy, then a
board check, 2026-10-27). Static: in work mode bank 1 is not computed live.
`phy_wr_rx_gain_mem` writes one pair per AGC memory row from
`phy_param+0x180`, a table of 7 frequency groups (`phy_freq_to_index`:
borders 5240/5320/5560/5640/5755/5835 MHz) x 9 RF stages x I/Q. Every
channel setup (`phy_chip_set_chan`, from `esp_wifi_set_channel` and
`phy_set_freq`) reloads it; only the boot RX DC calibration
(`phy_rfrx_rxdc_cal_new` from `phy_bb_init`) writes the table. The forced
bank-1 writes in libphy (`phy_pbus_set_dco` and six direct writers) are all
calibration paths under `phy_bb_init`/`phy_wakeup_init`; none is reachable
from the PHY calls the receiver makes at run time. Board (VTX off, R7): the
AGC memory rows decode to the table (84/84, `arc_gain_entry_dc`), the bus
shows a forced gain's bank 1 at once (32/32 at 0 us, no race on a gain
write), and every I in the table is 260 or more. So 245/224 was no table
reload: the table or the AGC memory changed during that session, or
something outside libphy wrote it; still open. `T` `BANK1_CHECK` compares
the table with a hash taken at boot and every AGC memory row with the table,
and the console prints `BANK1_CHECK CHANGED` within 5 s when either moves.

## LEVEL on the measured demod quality (2026-10-27)

**Blackbox (sessions 245-249, walking with the VTX on, GAIN AUTO = KEEL-1).**
At -40..-70 dBm and C/N 10-20 dB the video was only 60-90 % while KEEL-1
wrote 10-30 times a second, nearly always for `overload`. Over the seconds
with C/N >= 10 dB: video % = 87.9 - 0.74 x writes/s + 0.15 x overloads/s
(n = 141), i.e. ~7 ms of picture per gain write, and no extra cost for an
overload (fold). 0-2 writes/s: 95 % video; more than 6: 78 %. A
correlation (walking also fades), but every AGC host model had assumed
2 ms per write and a fold as a lost moment.

**Demod quality grid** (`research/level_quality_grid.py`, 1080 cases:
bit-exact U80P64-R1, physical-ADC model with the ultrafine fold, 12 MHz
filter, PAL/NTSC, C/N 15 values, total rms radius 0.75-13 cells):

- best radius 2.5-6 cells rms, flat within ~0.5 dB SINAD at every C/N;
- at C/N <= ~6 dB after the filter the radius matters little from 1.25
  to 7 cells (within ~1 dB): the row choice hardly matters at the edge;
- a small radius is mild (1.5 cells: lock kept, ~5 dB SINAD less on a
  strong signal); overshoot is not: 8 cells drops SINAD to 2-5 dB and
  10 cells loses the line lock completely from C/N ~6 dB.

`research/level_quality.py` turns it into `main/level_quality.h` (lock
first, then luma SINAD, monotonic around each C/N's best radius; CI checks
it is current).

**LEVEL** (`main/agc.c`) now scores every home row with that table at the
row's C/N and total radius, instead of the hand-made sigmoid/fold formula.
The level history is 200 ms (was 20 ms, which followed the fading itself),
and a folded window counts its own measured level 3 dB over when the
carrier dominates it (a fixed level at the rails read like ~6 cells rms,
the table's best, and held the top row on a strong folding carrier).

**First board check (session 267, hand on the antenna, -30..-60 dBm, C/N
10-20 dB)** with the 200 ms history alone: 272 moves in ~45 s (~6 a
second), 8176 folded windows, video 40-85 %. The slow history held a row
that folded when the hand came off and climbed late when it covered. The
harness had no such scenario; it now has "hand on the antenna" (-25 dB in
random 40-250 ms spells) and reproduced it (close: LEVEL 72.1, 8.7 writes/s,
against KEEL-1 78.5). Fix: a second, fast history (20 ms); when the best
rows of the two are 9 dB (three home rows) apart the level jumped rather
than faded, and the slow history restarts from the fast one (`jumps` in
`T`). Close hand: 78.5 at 15.9 writes/s (KEEL-1 78.5 at 29.2); mid hand
66.0 against 68.8, the one scenario LEVEL still loses. The blackbox
`gain_writes` column counts KEEL-1 writes only; LEVEL moves show in the
gain min/max per second and in `T`.

**Host comparison** (`test/test_agc_compare.c -DPICTURE_Q
-DWRITE_COST_US=7000`, CI): score (mean picture quality), writes/s, edge
3 dB C/N.

| | nominal | gain-model error 1/6 dB | old threshold judge |
| --- | ---: | ---: | ---: |
| LEVEL (this) | **70.5**, 1.7/s, 42.6 | **70.1**, 1.9/s, 43.1 | 58.9, 35.0 |
| KEEL-1 (AUTO) | 68.7, 6.1/s, 40.7 | 68.4, 6.6/s, 41.1 | 62.3, 28.7 |
| STATIC (best fixed row, a posteriori) | 70.9, 44.0 | 70.9, 44.0 | 60.4, 35.0 |
| IDEAL (no write cost, any row per tick) | 74.1, 44.6 | 74.0, 44.6 | 74.7, 41.8 |

(With the hand scenarios; the old-judge column is from before them.)

With the measured demod one well-chosen row is almost the ideal, so the
best AGC writes rarely and only on a clear change. LEVEL beats STATIC on
steps (+15 dB closer: 94.3 against 82.6; -12 dB obstacle 88.3 against
85.6). The old threshold judge (fold = a lost moment, 10 dB SNR at close
range) disagrees: it is the hand-made model the blackbox does not support.
Tried and dropped: averaging windows before the histogram (no change), a
write-payback rule instead of the fixed hysteresis (70.1-70.2, more writes).
At the edge LEVEL sits a home row under STATIC's top row (1.4 points, ~0.15
dB); the difference is under the hysteresis.

LEVEL is the default gain mode (operator decision 2026-10-27, after the
hand test above: "KEEL regulates too much"); KEEL-1 stays selectable as
KEEL. Open: a walk to the edge on LEVEL against the KEEL-1 walk of sessions
246/247 (video % per dBm, writes at the edge).

## Range edge audit: DC lockout, NTSC porch window, PHY FILT (blackbox, 2026-10-27)

Board b26ef545, A1 5865 MHz, NTSC VTX, firmware `5.0.0-menu+local` (the
#280 menu build). Blackbox sessions 237-251 and 64 IQ snapshots, read with
`tools/flight_log.py` and `tools/edge_audit.py`. Operator report: "range is
bad, gone at about -84 dBm".

**Where the edge is.** Walking (sessions 246-249), binned by RX_LEVEL:
video 70-100 % at -84 dBm, ~50 % at -86..-87, 10-25 % at -88..-90, against
the 2026-10-20 rating of ~-88..-90 walking (U85Q). The meter carries
+-1-2 dB, so the walk gap is 2-3 dB, not 6-8.

**Ruled out on the data:**
- I-LSB on the user-LED GPIO27: across all snapshots the I and Q LSB have the
  same duty (~50 %), the same lag-1 agreement and the same odd/even code
  balance, and the same correlation with the upper bits' slope at lags -2..2
  (+-0.40 at -1/0 for both): no load, no late edge.
- U80P64-R1: edge snapshots that the record calls lost (video 11-37 %)
  hold clean syncs on every line through P64, U85Q and a float
  discriminator alike (C/N 8.7-11 dB in the IQ, M2M4).
- XNARROW: chosen by the smallest -3 dB width; its noise bandwidth (11.7
  MHz in the mode 12 sweep) is NARROW's, so it cannot add range.

**What costs the edge.**
1. *DC rows locked out.* `dco_service()` released a held DC pair after
   three windows of >= 50 % clipping and banned the row for 2-32 s, during
   which `servo_service()` skips it. A row the servo could still trim fell
   back to the chip's own DC (4.8/2.2 cells at one boot on this board) and
   could not be corrected: session 247, G73 with no carrier, Q at +4.4
   cells for about a minute at the same held trim that read -0.2 cells with
   a carrier later; RECOVER lasted 3-10 s after each fade while the level
   was already back at -85..-88 dBm.
2. *Slow rows.* The edge rows G73-G83 needed 13-48 codes of trim after the
   manual calibrations (old build, session 230: 0-5). With 8-code steps on
   40000-sample means a row hopped through at the edge took seconds per
   row; rows with noise under a cell hunted +-8 codes around zero.
3. *Telemetry.* The per-second DC (`s_drift_avg`, the health block's circle
   fit) holds its last qualified value: session 249 logged -4.15/+1.44 cells
   for 9 s while the snapshot IQ read -0.09/-0.09.
4. *Calibration results.* Session 240: six manual runs, final table 30/30
   template rows; session 242: three runs, 15/30. The only sign was an
   0.8 s DC NOT COMPLETE line.
5. *NTSC AFC.* Board snapshots, `afc2_measure` replay with the porch window
   moved: the old window (3.4-4.4 us after the detected sync end, set for
   PAL) read the NTSC porch at -1019 kHz; 1.2-3.6 us reads -1248 (PAL lab
   snapshots: -1044 vs -1040). NTSC picture starts ~4.0 us after the
   detected end (float discriminator profile), so the old window averaged
   video and took a VTX ~230 kHz off for centred: `afc_locked=0` all session.

**Changes.**
- Sign-bit bracket (`dc_servo.h`): where the sign share saturates the step
  doubles while the cloud stays on one side (8 -> 16 codes) and halves
  across zero down to one code, where the row is held until its mean moves
  0.8 cells. The sign bit is the ADC's own and never folds. Steps above
  `PHY_DCO_TRY_MAX` are written as verified 8-code trials.
- Fast decisions: a saturated share decides on 4096 samples (> 9 sigma from
  one half) instead of 40000 (`receiver_dc_peek_row`).
- The hold guard releases and bans only rows the servo cannot trim (trim
  limit, no row, fault); otherwise it wakes the servo
  (`DCO_BAN deferred_to_servo=`).
- Blackbox DC: the servo's fresh active-row mean, the circle fit only as a
  fallback.
- Manual calibration ends on CALIBRATION OK / INCOMPLETE for 3 s with the
  template and missing rows and the reason (`CAL_RESULT`).
- AFC porch window 1.2-3.6 us (2.4 us spans 8.6 NTSC / 10.6 PAL subcarrier
  periods, so the burst averages out).
- Menu PHY FILT: digital RX filter mode 12 -> 0 -> 4 -> 8 (noise maps and
  the C/N reference are measured in 12; another mode shows LAB), reapplied
  on every RF restore, read back, previous mode kept on a failed readback;
  the DC row sums restart on a change.
- Menu choices kept across reboots (NVS `prefs`, operator request): FILTER,
  PHY FILT and the GAIN model (AUTO/LEVEL/RSSI). MAX and fixed gains still
  return to AUTO at boot (2026-10-20: a forgotten fixed gain costs range
  without a sign). Channel, VIDEO, OUTPUT and AFC were already kept.

**Host evidence.** `test/test_dc_servo.c`: template rows 6-12 cells off
(sigma 0.8) within 0.35 cells after <= 5 decisions, sub-cell noise settles
within one code and holds, all earlier cases pass (an inverted polarity is
now found by a fold turn within 10 decisions). `test_dfilt_runtime.py`
covers the mode choice through RF restores; `test_menu_ui.c` every PHY
FILT mode. **Board, VTX off** (same board, this build, fixed G78/G72/G66,
three 8190-byte snapshots each): DC +0.06/+0.21, <= 0.25 and <= 0.33 cells,
servo offsets 10/-15, 5/-6, 2/-3 codes, `fault=0`, `unbans=0`,
`deferred_to_servo=274` (holds the old guard would have released and
banned). PHY FILT 0/4/8/12 applied and read back. The walk with the VTX
on is the deciding test; no dB or range claim.

## Boot heap margin: about 2 KB at the boot peak (board, 2026-10-26)

Board COM6 (XIAO ESP32-C5), main `9699550` and `07ece42`, with temporary
heap prints after every boot stage in `app_main` and inside `rf_start`
(not committed). Internal heap free / largest block:

| After | `07ece42` | `9699550`, fresh sdkconfig |
|---|---|---|
| `rf_start` (NVS 5.8 KB, Wi-Fi init 23.2 KB, start 2.4 KB) | 41,752 / 38,912 | 41,640 / 36,864 |
| `transport_start` | 28,016 / 25,600 | 27,904 / 25,600 |
| menu splash (DMA descriptor chain) | 9,780 / 8,192 | 9,668 / 7,936 |
| receiver and control tasks | 2,216 / 1,536 | 2,104 / 1,536 |
| splash end (chain freed) | 20,452 / 17,408 | 20,340 / 17,408 |

The splash chain (about 18 KB, interlaced) is held while `receiver_start`
creates `gain_obs` (4 KB stack) and `gain_fast` (3 KB), so the lowest free
heap of a boot is about 2.1 KB, while `T` a few seconds later reads 17 KB
largest. Any extra ~2 KB allocated before `receiver_start` makes
`gain_fast` fail with `ESP_ERR_NO_MEM` and the board boot-loops. That
happened here: an `sdkconfig` generated before the defaults cut
`CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM` to 2 still had 4, and `esp_wifi_init`
took 3,440 bytes more. ESP-IDF does not reapply `sdkconfig.defaults` to an
existing `sdkconfig`; delete it and rebuild after a defaults change.

`T` now prints `heap_min` (lowest free since boot, 2,096 on `9699550`
plus this change), and `tools/flash.py --check` warns below 1024 bytes; it
also warns before flashing when the build's configuration differs from
`sdkconfig.defaults`. Not changed here: the splash could be built after the
receiver tasks (it already skips itself when the raster cannot be built),
which would trade the margin against a progressive splash on some boards.

## PSRAM: 8 MB on the XIAO, now mapped (board, 2026-10-26)

esptool on COM6 reads `ESP32-C5HR8 (revision v1.0)`, "Embedded PSRAM 8MB
(AP_3v3)". Every generation so far left it off. Same source (`ab2edee`),
VTX off, three PSRAM configurations against the baseline:

| Configuration | `heap_min` | `heap_internal` live | free with the menu open | without PSRAM |
|---|---|---|---|---|
| baseline | 2,104 | 24,672 | ~6,400 (computed) | boots |
| Wi-Fi `.bss` in PSRAM (`ALLOW_BSS_SEG_EXTERNAL_MEMORY`) | 14,564 | 37,120 | 18,884 | panics |
| Wi-Fi heap in PSRAM (`TRY_ALLOCATE_WIFI_LWIP`, `IGNORE_NOTFOUND`) | 6,348 | 28,904 | 10,668 | boots |
| both | boot loop (`gain_obs` `ESP_ERR_NO_MEM`) | | | |

The Wi-Fi BSS configuration is available in the opt-in
`sdkconfig.psram.defaults` overlay (quad, 80 MHz, `USE_CAPS_ALLOC`). The
standard image keeps PSRAM disabled so boards without it still boot. The
linker moves the `.bss` of `libnet80211`/`liblwip` (`g_cnxMgr` 5.9 KB,
`s_wifi_nvs`, TWT timers; 16,456 bytes), which costs 3.4 KB of extra IRAM
code: static HP SRAM 288,850 -> 276,518 bytes. `--check` passed with no
transport faults, ready time 3.84 s against 3.93 s, and the menu opened
and closed with no faults and no leak. With both options ESP-IDF forces 16
static Wi-Fi TX buffers (`esp_wifi/Kconfig`), which is the likely cause of
the boot loop (not investigated). `T` prints `heap_psram`.

The contributor subsequently tested about two minutes of R7 NTSC video:
three menu cycles without a leak, five of five IQ snapshots, eleven flight-log
blocks and one snapshot with no flash errors or drops, and no transport faults
([PR #283 board report](https://github.com/Twotoz/C5VRX/pull/283#issuecomment-6432708418)).
The PAL menu and long runs remain untested. PSRAM shares the MSPI bus with the flash; the video
path (PARLIO, GDMA, BitScrambler, the RX ring) stays in internal SRAM.

Three cold static buffers moved on top (`EXT_RAM_BSS_ATTR`, 7.4 KB): the
lab's `s_dyn`, the diag trace and the flight-log queue. Static HP SRAM
276,518 -> 269,118 bytes; on COM6 `heap_min` 14,576 -> 21,956,
`heap_internal` 37,132 -> 44,512, largest block 17,408 -> 19,456, 26,276
free with the menu open, no transport faults, 19 of 20 `I` snapshots.

What stays internal, and why (board, same session):
- IQ snapshot buffers. Copying the 8,190 bytes into PSRAM missed the
  copy deadline (about 500 us before RX reaches the window): 1 of 10 `I`
  snapshots against 19 of 20 into internal RAM. CPU writes into PSRAM at
  80 MHz therefore run below about 16 MB/s.
- Flash-write sources (`s_flush_buf`, snapshot buffers): from PSRAM,
  esp_flash copies 32 bytes per page program with the cache off.
- The gain loop's state, the control window scratch (copied under a
  microsecond deadline), transport restart buffers, and the AGC table
  check buffer (filled in a critical section that changes clocks).

Long gapless IQ captures into PSRAM do not work while RX runs (board,
same session). 8 MB would hold ~200 ms of the 40 MB/s IQ stream (a
snapshot is 204.75 us), so a console capture was tried: an AHB GDMA
memory-to-memory channel (`esp_async_memcpy`) copies each completed part
of the RX ring into PSRAM, 32-byte aligned (PSRAM burst limit and cache
line), with every chunk checked against the time RX needs to come back
to it. Result on COM6 at 80 MHz: 20-22 MB/s, 16-24 KiB before the copy
fell behind, and each capture added one `rx_overflow` and one
`tx_empty`: the PSRAM writes starve the RX DMA, so live video glitches
and the stream itself has a gap. 16-byte chunks were worse (6.6 MB/s;
the driver splits them through a stash buffer its ISR merges). 120 MHz
PSRAM does not build with this flash configuration (ESP-IDF's MSPI
timing tables). A long capture needs another path, such as a decimated
stream or a capture with the RX ring paused.

## VTX-on PHY lab, AFC window and digital mode 12 (board, 2026-10-26)

Board COM10 (NiceMCU), 5865 MHz / A1, PAL VTX on a table, firmware
`5.0.0-pr.273+7ca5725`. Data: `research/data/vtx_on_lab_2026-10-26/`
(lab console logs, `combined.txt`, 30 IQ snapshots in `iq/`).

**Bench lesson: the operator's body.** With someone at the bench the carrier
at fixed, read-back gain stepped 4-11 dB between plateaus of seconds
(~1 step per 10 s) while the noise floor stayed within 0.2-0.5 dB, so the
signal was lost before the LNA. With nobody within ~3 m the carrier held
11.6-11.7 dB for two minutes (`5865_mon_07/08.log`, from 18:08:21); on the
return the steps resumed within seconds. Not a receiver, AFC (frozen in a
lab run) or VTX-frequency effect: the porch estimate sat on the -1000 kHz
target, residual 0. Lab runs need the operator away; with that the sham
spread was 0.12-0.21 dB C/N (`5865_sham6.log`). A head next to the goggle
antenna shadows the link the same way in the field.

**Sweep (console `S` `a`, baseline C/N 7.05 dB, G73, 25 shams):** no PHY
candidate improves clearly on stock. RX IQ correction off costs
0.43-0.48 dB (keep it); gain rows G67/G79 cost 0.86/0.24 dB against the run
gain; analog filter code +-1/+-2, ADC selector, LO cap +1, TX clock and all
packet/RX policy bits show no effect; NF-auto 8..F do not latch, 0..7 do
not reproduce. One weak-signal level and one channel: screening, not range.

**Digital mode 12 (suite 3, repeated):** C/N +0.20 / +0.28 / +0.52 dB and
FM clicks -0.52 / -0.34 / +0.01 per mille against mode 0 (the last run
saturated at 14.9 dB C/N); verdict REPRODUCED_BETTER. Mode 4 (+0.28 /
+0.19 dB C/N, no click change) and mode 8 did not reproduce. VTX-off noise
(PR #244) read mode 12 ~1 dB noisier, so it was not picked then: with a
carrier the wider response costs less than it keeps. Production now applies
mode 12 (`PHY_DFILT_MODE_DEFAULT`) with the same readback and reapply path;
calibration generation 4, gain map 8, C/N store 5 and gtab 5 discard
mode-0 data, so a board needs one manual calibration (VTX off) after the
update. Measured only at 5865 MHz (ADC selector 1); 5805 MHz and an
attenuator threshold test are still open. ~0.25 dB is about 3 % distance in
free space at best.

**AFC window: why the AFC never locked.** `sync_valid_pct` stayed <= 68 %
for minutes and `afc_locked=0`, also at 17 dB C/N without clicks. The
colour burst was never detected (`burst_x10=0` in all 30 snapshots), so a
sync counts only with a second one exactly one line earlier
(`afc2_measure`), and one 4092-byte control descriptor (1.6 lines) holds
such a pair only at some line phases. Replay of the real snapshots through
the production `afc2_measure` (`research/sync_window_replay.c`): 4092-byte
windows at 65 offsets per snapshot 45 % valid (28-66 %), the full
8190-byte window 30/30 (PAL, 2 lines). Host model, B/W PAL at 40 line
phases: 17/40 vs 40/40 (`test/test_afc_measure.c`). With ~1 sync per
window the porch MAD sat at 80 kHz, the lock limit. The control loop now
measures the AFC on an 8190-byte contiguous window (`copy_snapshot()`
under the scheduler hold, ending where the active descriptor began, so it
covers the gain window); gain observation keeps the 4092-byte descriptor.
Noise windows of both lengths still give no sync. A longer window passes
the envelope-stationarity gate less often near the noise floor; AFC
re-acquisition already waits for >= 6 dB C/N.

Board check of this firmware (same day, after one manual calibration with
the VTX off): `DFILT want=12 hw_mode=12`, `cal_generation=4`, picture fine
in the goggles. The source was now NTSC with a detected colour burst
(`burst_x10` ~53, period 2542), deviation 0.73. `sync_valid_pct` averaged
82 % (was ~30 %), but the AFC still did not lock: porch MAD 185-284 kHz
with the median residual inside the deadband. Replay of 18 fresh snapshots
and a mean frequency profile after the sync (`runs` not kept; method in
`research/sync_window_replay.c`) show why: NTSC's back porch is ~1 us
shorter than PAL's, so the porch window (3.4-4.4 us after the detected sync
end, set for PAL) already reaches the active video. The estimate follows
picture content, and it reads the VTX as centred while its flat porch sits
at about -1620 kHz, ~600 kHz from the target. An earlier window (start
~100 samples) reads -1605..-1627 kHz for NTSC and keeps PAL at MAD 44-60,
but the NTSC MAD stays 100-135 kHz; a subcarrier-integer length (45
samples) did not lower it, so the burst is not the remaining cause. On the
earlier PAL snapshots the new window gives MAD 48 kHz and residual -44 kHz,
inside the lock gates. The NTSC porch window is a follow-up; it predates
this change.

## GPIO27 IQ loopback experiment (2026-10-25)

Operator request: a standalone GPIO-only PR directly on main, reserving
D10 and D3 for future seventh/eighth DAC branches. No TBC is included.
The operator reported unsatisfactory behavior with the LED-pin IQ route;
this separate build isolates the reroute from TBC. The electrical cause is
not established. This is an experimental comparison, not acceptance.
Move the I-LSB MODEM_DIAG loopback from D10/GPIO10 to GPIO27, and the
Q-sign loopback from D3/GPIO7 to GPIO2. This extends the existing C5VRX by Twotoz and
contributors' GPIO-matrix/PARLIO loopback and shared-pin-map precedent in
`legacy/c5vrx3/docs/custom-board.md`; it is not another RF sample source.
`RF_IQ_GPIO_NUMS` in `main/rf.h` supplies both RF routing and PARLIO RX,
including the bench coarse/fine/ultrafine routes and RF restores.

| PARLIO RX bit | Component / ADC bit (ultrafine) | GPIO |
| --- | --- | --- |
| 0..3 | Q[4], Q[5], Q[6], Q[9] | 1, 0, 25, 2 |
| 4..7 | I[4], I[5], I[6], I[9] | 27, 5, 3, 4 |

D10/GPIO10 and D3/GPIO7 are reserved, not enabled DAC output lanes.
Reserve a 16 kOhm D10-to-CVBS footprint for the seventh physical branch
and a 32 kOhm D3-to-CVBS footprint for the eighth, both DNP in this build.
Update 2026-10-25 (operator decision, DIY REV B PCB): the nominal values
are now 15 kOhm (D10, R8) and 33 kOhm (D3, R9); the six existing branches
and the 200 Ohm load are unchanged. Not built or measured.
The new branches are low bits: D10=bit0 for 7-bit; D10=bit1 and D3=bit0
for 8-bit. The existing six branches shift up in logical significance.
This records the future pin allocation; no PCB files or wider DAC firmware
are changed. Wider output requires loaded voltage/linearity verification,
including boot-strap loading on D3/GPIO7. The D4..D9
six-bit resistor DAC, U80P64-R1, raw 32-KiB ring, RX40 capture, two-lookup
hardware demodulator and unique DAC20 duplicated at TX40 are unchanged.
No per-sample CPU work or runtime pin-switching control is introduced.
GPIO27 is exclusively IQ-owned, including while menus own TX; no status-LED
writer may reconfigure it. No active firmware LED writer was found.

The [Seeed XIAO C5 schematic](https://files.seeedstudio.com/wiki/XIAO_ESP32C5/res/Seeed_Studio_XIAO_ESP32C5.pdf)
connects GPIO27 to the user LED through R15 (1 kOhm). The LED may glow with
IQ activity. It is not an unbonded or electrically unloaded virtual GPIO.
GPIO6/26 remain assigned to the battery measurement circuit; USB, BOOT,
flash/PSRAM are not repurposed. GPIO2 is now exclusively IQ-owned after
main's sync flywheel removal. It cannot also carry PR #257's optional
40 MHz sample clock; combining the features requires another clock pin.

**Hardware validation pending:** compare the main baseline with this build
on an unmodified XIAO at 5805 and 5865 MHz, using repeated cold boots, VTX-off
noise and VTX-on PAL/NTSC. Check IQ snapshots (`I`), sampling-phase status,
the lane comparison (`L`), menu exit, channel changes and weak-signal recovery.
Confirm the moved I-LSB and Q-sign bits toggle normally, with no extra grain, phase faults,
sync loss or transport errors. Compare under controlled attenuation before
claiming unchanged sensitivity. If either new route reduces timing margin,
revert the mapping; do not assume that a lit LED or passing build proves
correct high-speed capture. No physical IQ/video/range result is claimed.

## Sync flywheel removed (operator decision, 2026-10-25)

The C5VRX-5 sync flywheel (PR #212, witness fix PR #218, bounded sync-loss
scheduling 2026-10-18) is removed from the firmware. Operator report on the
board: with it the picture showed extreme desyncs and range was poor. This
is an operator observation, not a controlled attenuation A/B; it agrees with
the earlier board evidence below (flywheel ON rolled the picture, 2026-10-18)
and with the range study's conclusion that a sync flywheel is not a range
lever (`research/README.md`).

Removed: the native comparator observer (`sync_output_probe.c`, GPIO2/RMT),
the shared edge clock and live gate/output (`sync_edge_clock.c`,
`sync_live_gate.c`, `sync_live_output.c`, GPTimer alarms writing the DAC OE),
the comparator flag LUT format and GPIO23 clamp in `transport.c`, the
coasting restart lease, the 2-ms control-task tick (back to the plain
50-ms tick), the menu FLYWHEEL item and its PHY-release/retry handshake,
console `U`, `SYNC_LIVE`/`SYNC_FLYWHEEL*` status lines, NVS key `flywheel`
(now ignored if present), their host tests and `tools/check_sync_output_iram.py`.
The flight-log V1 wire slots stay so old logs decode: the session byte 44
and output flag bits 1..7 read 0, and the observer lock percentages 0xff.
The live path again only recovers the transmitted composite waveform.
Not changed: the P64 program/LUT, gain, AFC, filters, OUTPUT scaling and the
verified halted restart/rollback. The flywheel sections below are history.

## P64 demodulator on mode 0 (PR #250 model, 2026-10-23)

This branch repairs the P64 model from PR #250 on top of the digital-filter
work in #248. U80P64-R1 is now the only firmware model by the explicit
operator decision of 2026-10-23; U85Q, kp switching and LP are archived.
The 7.2-degree number describes the 50-video-state grid, not measured phase
precision. The full table and assembled output are host-checked; combined
firmware timing and range remain unproven. The initial board comparison on
2026-10-23 had the VTX off (operator clarification), so it does not establish
picture quality. A separate host tone test found the original P64 latching
porch/zero frequency into sync code 15. R1 corrects phase-advance output,
exit behavior and token allocation; 588 clean-tone cases and 12 PAL/NTSC
validation cases pass. The unchanged R1 table later had one positive VTX-on NTSC observation
(build `d894bef`, detailed in P64.md). PAL, weak-signal/range and this final
cleanup have not been hardware-qualified.
See [P64 integration, source provenance and board checks](P64.md).

## 11p receive baseline (PR #254 integration, 2026-10-24)

Operator request: bring the 11p method from PR #254 into PR #251. This
extends C5VRX by Twotoz and contributors' pinned PHY lab in
`legacy/c5vrx4/main/phy_rx_lab.c` (#150-155); the original 11p observation
is credited to SushiDude (@Ready4Sushi). Source implementation and noise-only
measurements: [PR #254, commit 421c690](https://github.com/Twotoz/C5VRX/tree/421c690dd33d03d03efc0517aeedb46e0736308a).
The reported paired 5865 MHz noise deltas were -0.291/-0.392/-0.223 dB;
5805 MHz included an inconclusive long-dwell result. Those are VTX-off
observations, not proof of improved sensitivity, preserved wanted signal,
video SINAD or range. This integrated build has not been flashed or measured.

The pinned `phy_11p_set(1,0)` is applied after vendor bandwidth/gain setup
on every RF restore, then the measured analog profile, digital mode 0 and
DC hook are replayed. Entry verifies both `phy_param` flags, all eight BBTOP
0x67 bytes 6..13 (=60) and the four recovered MMIO masks. On entry failure,
the exact saved flags, bytes and masked MMIO fields are restored and verified;
any supported-PHY entry/rollback failure stops through `ESP_ERROR_CHECK`
instead of continuing with an unknown receive context. Unsupported PHYs
refuse the private helper without writes, as with the existing filter code.
The realtime IQ/BitScrambler/DAC path is unchanged; the bench suites are not
imported and no 11p menu/NVS toggle is added.

The default analog bytes (`PHY_FILTER_CALIBRATED` code / zero skirt) now
mean 60, matching the full 11p baseline. Explicit capacitor codes and nonzero
skirts still use the original boot-captured factory bytes as their reference.
This lets calibration measure wider candidates rather than saturating every
skirt candidate at 60. Thus a selected calibrated profile may intentionally
override part of 11p's analog bytes; the original noise-only delta cannot be
assigned to that combined profile. `PHY11P` diagnostics verify persistent
flags/MMIO; `FILTER_HW` separately checks the actual requested analog profile.
Digital mode 0 is still verified and the vendor ADC-rate choice is retained.

Migration: filter version 6, DC version 8, calibration generation 3, saved
gain maps version 7, C/N noise/anchor maps version 4 and noise-derived `gtab`
version 4 discard pre-11p measurements. Settings, OUTPUT and flywheel are
preserved. A fresh quiet startup establishes new RF references; without one,
C/N remains unknown. Software regression tests execute the production helper
with fault injection at all 14 entry fields, exact rollback, failed rollback,
unsupported PHY refusal, nested restores and default/explicit analog profiles.
They do not establish physical signal quality. Bench acceptance still needs
VTX-on PAL/NTSC, weak-signal attenuation, chroma/detail, retunes at 5805/5865 MHz
and loaded CVBS comparisons for the final calibrated combination.

## Digital RX filter mode 0 (PR #244 follow-up, 2026-10-23)

Source: [Twotoz's bench report on PR #244](https://github.com/Twotoz/C5VRX/pull/244#issuecomment-6345839653),
COM10 NiceMCU, 5865 MHz / A1, 2026-10-22. This extends the digital-filter
and coupled ADC-selector investigation in
`legacy/c5vrx4/PREDEMOD_LAB.md`, rather than introducing another IQ source.

With the VTX off, the reported baseline mode 8 had noise power m2=7.691
and noise bandwidth 13.210 MHz. Mode 0 measured m2=6.757 (-0.56 dB) and
12.907 MHz; modes 4/8/12 measured 7.516/7.457/8.490 in that sweep.
Modes with low two bits set stalled IQ (m2=0); the observed working values
of `0x600A0430[21:18]` were 0, 4, 8 and 12. These are bench observations,
not a general specification of this undocumented register.

The report also describes live NTSC lock, `sync_valid_pct=100`, coherence
99 and gain 23..26. Its envelope figures alone do not establish an unchanged
signal transfer function. The noise-only power difference is not a measured
input-referred sensitivity, C/N or range gain. Repeated paired measurements,
controlled RF attenuation and PAL/NTSC checks across channels are still needed.

Production now applies only mode 0 through the pinned vendor helper, reads
back the full four-bit field, and reasserts it after boot, channel, bandwidth
and AFC restores. It preserves the vendor-selected ADC rate (selector 0 up
to 5830 MHz and 1 above in the pinned PHY). Unsupported PHYs refuse the
private write; a readback mismatch is logged and cannot pass
`rf_filter_applied()`. Console calibration diagnostics report wanted/actual
mode, with -1 for unsupported. There are no live mode/ADC experiments or
noise sweeps in this change.

Upgrades invalidate filter/DC calibration (generation 2), saved gain maps
(version 6), C/N noise/anchor maps (version 3), and the separate noise-derived
gain/dBm table `gtab` (version 3), whose transfer context
otherwise omits the digital mode. First boot recalibrates through the existing
bounded startup procedure. C/N remains unknown until a fresh noise reference
is available; old powers must not be reused with mode 0.

Host regression tests compile the production ownership, DFILT and RF restore
code with mocked SDK/vendor/MMIO boundaries, including unsupported builds,
all four-bit readback values, nested generation/invalidation ordering, and
failed writes. They do not establish RF response or physical video quality.
On the board, check `DFILT want=0 hw_mode=0` after boot, A3/A2 channel changes
across 5830 MHz, BW20/BW40 profile changes and AFC offsets; confirm fresh
calibration, VTX-off/on recovery, colour/detail and sync on PAL and NTSC.

## The problem: the whole picture dropped out

With slightly bad video the C5VRX-4 picture did not get noisy, it vanished.
A goggle model (sync low-pass, slicer, H-PLL flywheel, ideal and peak-clamp
AGC) over full fields showed why: the trained 32-phase trackers (PAIR,
RANGE32) slip on VTXs with a larger deviation (>= ~1.05) or an off-centre
carrier and never re-acquire. They lost the whole picture in 8-25 % of cases
even at 20-25 dB C/N. The board logs added two contributors: the DC search
running under weak video, and gain hunting at the range edge.

## Where information is lost, and where clicks come from

An information ledger added one loss at a time to identical signals:

- The 4-bit IQ lanes, folding included, lose essentially nothing against
  the analog IQ. Using one sample per 50 ns span is not a loss either.
- Clicks are FM threshold physics: the noise phasor circles the origin. The
  ideal analog discriminator (an RX5808) has them too. The IQ shows when
  they happen: the sample lands in the origin cells.
- The loss is the 32-sector phase lattice (625 kHz per output step). A
  tracker with clipping recovers it; the flashed LUT reaches its own float
  bound below 14 dB C/N.

## U85Q: the demodulator

An untrained search chose among closed-form formulas only (no fitted LUT
entry), with held-out confirmation:

- state: 32 tracked phases, `p' = p + 0.85 * wrap(observation - p)`;
- observation: the PAIR4411 shared word (sample A full byte + sample B
  signs);
- origin cells: four coarse quadrant tokens with their own gain 0.5. Erasing
  them (U85E) was sharper but failed under multipath, because fades put
  real signal into the origin cells;
- output: the tracked advance, linear over -6..+9 MHz onto the DAC.

Held-out results (picture = correctly displayed lines, mean):

| model | nominal picture / SINAD | mixed robustness picture / SINAD |
| --- | --- | --- |
| **U85Q** | 0.998 / 3.55 dB | **0.893** / 1.93 dB |
| U85E (origin erasure) | 0.995 / 4.26 dB | 0.706 / 2.48 dB |
| HC50 (stateless) | 0.981 / 2.17 dB | 0.872 / 0.76 dB |

The mixed set covers echo + fading, carrier +-2 MHz, DC, I/Q 1.4/20 deg and
a CW spur. Trained PAIR/RANGE32 scored 0.83 / 0.82 on the nominal set, with
0.00 in their worst cases. Under multipath U85Q matches its float upper
bound; what remains there is in the signal, not the LUT.

The output window is the one trade-off left: narrower is sharper but clips
the sync tip when the carrier is off centre.

## Board validation (2026-10-11, real video, A1)

U85Q with the narrow filter and the DC latch, 12 snapshots:

- the carrier's sync tip sat at -5.0 MHz and white at +6.25 MHz. U85Q's
  -6..+9 MHz window holds both; U85E's -3.5..+7.5 MHz clipped the sync tip,
  so a narrower window is not safe;
- clicks per 1000 samples: U85Q 83.0, U85E 85.7, HC50 101.5.

## AFC target matched to U85Q (C5VRX-5)

C5VRX-4 centred the VTX blanking level at -436 kHz, the design centre of
the trained trackers. U85Q's window is -6..+9 MHz, and the board's real VTX
swung wider than the deviations searched (up to 1.4). Shifting the AFC
porch target, held-out simulation (board conditions with +-1 MHz carrier
error, stress, 20 cases per cell at C/N 4-20):

| porch target | picture (all) | SINAD | deviation 1.4 | deviation 1.8 |
| --- | ---: | ---: | ---: | ---: |
| -436 kHz (C5VRX-4) | 0.908 | 2.42 dB | 1.00 | 0.64 |
| **-1036 kHz (C5VRX-5: -1.0 MHz)** | **0.969** | **3.27 dB** | 0.98 | 0.90 |
| -1436 kHz | 0.961 | 3.89 dB | 0.95 | 0.91 |

The larger shift buys more SINAD but starts to drop pictures; -1.0 MHz is
the balanced choice. Board confirmation is pending.

The Direct Gain V5 band also fits U85Q: same simulation, mean over C/N
4-20 and deviation 0.69-1.4, by IQ radius (cells):

| radius | 3.0 | 3.6 | 4.2 | 5.0 | 5.7 | 6.5 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| picture | 0.992 | 0.997 | 0.987 | 0.995 | 0.993 | 0.980 |
| SINAD dB | 3.16 | 3.09 | 3.32 | 3.03 | 2.72 | 1.75 |

The healthy band (P50 13..32, radius ~3.6..5.7, targets 4.1 / 5.2) sits on
the flat part; folding only costs above it. The gain controller is kept.

## The narrow analog filter

FM threshold follows the noise bandwidth before the demodulator. The board
ran ~19.4 MHz (noise bandwidth 20.7 MHz) where Carson's rule for typical
VTX video is ~13-14 MHz. Host results, U85E:

| filter | lock at 2 dB | lock at 4 dB | mean SINAD |
| --- | ---: | ---: | ---: |
| ~20 MHz | 0.66 | 0.92 | 4.26 dB |
| 14 MHz | 0.95 | 0.99 | 4.55 dB |
| 12 MHz | 0.98 | 0.99 | 4.50 dB |

14 MHz costs <= 0.3 dB on a strong signal for every tested deviation, and
helps every weak case. C5VRX-5 measures the setting per board (VTX off,
once) and keeps the one with the lowest noise bandwidth that still covers
14 MHz. (Later lowered to a 12 MHz default class, plus 10 MHz narrow and
16 MHz wide classes; `calibration.c` FILTER_MIN_KHZ.)

## Receive-chain fixes carried over

- DC: the per-gain DC DAC pair is searched only in confirmed quiet, never
  on a channel where a carrier was seen this boot, and held after every gain
  write (board: DC median ~134 LSB without the hook, ~11 with it).
- Sampling phase: about 3 boots in 10 sampled MODEM_DIAG mid-transition;
  a fourth-difference probe finds it and slips the RX clock.
- Direct Gain V5: healthy-band zero-write zone, 20 ms persistence for
  moderate excursions, burst gate and staged overload recovery.

## What did not help (kept out)

Trained LUT demods, FusionDemod/AutoFit program swaps (3.8-5.1 ms engine
halts), the sync flywheel, the level servo, adaptive lanes, the radius
boost, the vendor RX recalibration (grain on strong pictures), one lookup per
IQ40 sample, frequency-only trackers, MMSE outputs with a narrow window.

## What is left

Within this demodulator class the C5 is close to its ceiling. The remaining
dB are outside the LUT: the RF front end (LNA, filter, antenna, diversity) or
hardware with line memory at sample rate. A flight comparison against an
RX5808 is the open measurement.

## PR #196 ladder and LP handover (2026-10-12)

C5VRX by Twotoz and the C5VRX contributors. Initial operator evidence:
video appeared stable during five requested kp steps, but telemetry reached
1759 verification failures in 1760 writes. The old software shadow still
acknowledged them, so reported levels did not prove hardware contents.
Paused instruction rewriting with P produced black video while USB lived.

The first candidate (65946f1) inferred a fixed 32-bit CPU LUT port from
ESP-IDF's LL comment. Physical tests disproved it. The ESP32-C5 TRM states
that host addressing/data size must follow the configured LUT width too.
Stopped-core probes showed correct LUT16 low-half reads and neighbour bits
in some upper read halves; mode 32 accepted packed writes. Keep LUT16 and
compare only the low 16 bits. RUN writes returned unrelated RAM values and
failed almost entirely; PAUSE-only updates likewise failed and caused TX
empty events. HALT with the normal load/reset/start topology verified every
entry. The exact internal arbitration/read-latch cause is not established.
Source: [Espressif ESP32-C5 TRM, BitScrambler memory access](https://documentation.espressif.com/esp32-c5_technical_reference_manual_en.pdf).

The corrected path loads the complete target while halted, reselects each
CPU address before readback, and only publishes the shadow after all 1024
LUT16 entries verify. Failure restores baseline video. Both kp and LP
changes briefly restart output; no sample-seamless handover is claimed.
`test_kp_lut_io.c` models LUT16, address refresh, unrelated upper read bits,
a failed target and rollback, and every real kp table in both directions.
`kp_writes` counts attempted LUT16 writes.

Operator `lutfix2` telemetry reached 32,768 writes with zero verification
errors, one kp change and 31 program switches, zero transport/restart faults.
P switches measured about 5.5 ms; kp about 6.3 ms. The operator could barely
see the change. These timings belong to that build/setup, not a universal
latency or flight/range result. Continued P spam eventually hung in IDF
`parlio_tx_do_transaction` at its unbounded FIFO-ready spin (MEPC 408074e0,
control task, repeated IDLE watchdog warnings, USB heartbeat still alive).

Live TX now uses an owned nine-descriptor cycle covering the same 32-KiB
raw ring, instead of re-submitting driver loop transactions. Descriptor
ownership/zero EOF are restored with TX stopped; the FIFO-ready wait has a 20-ms timeout and
vTaskDelay so IDLE can run. A timeout returns failure and attempts baseline
recovery. The original boot/menu/RX topology and DAC pins are retained.

Every actual live change shares a five-second cooldown; failed restarts
also consume it. AUTO requires an unchanged combined kp/program target for
two seconds of usable observations in the same context. A combined target
uses one restart, not separate kp and program interruptions. P during
cooldown is refused and requires a new press later. Policy/mode reset
cannot erase the physical cooldown. `test_kp_switch_guard.c` checks boundary
times, target oscillation, coverage gaps and context changes. Diagnostics
expose switch_wait_ms, kp/prog restart times and failure counters.

Guarded owned-ring build hardware test (COM10, XIAO ESP32-C5 rev 1.0,
IDF 6.0.2, 2026-10-12): six bursts of 32 P requests, spaced 5.5 s apart,
produced six accepted program changes and 90 explicit cooldown refusals;
excess commands can be dropped by the bounded queues. Three manual kp
steps then reached verified levels 1, 2 and 3. Final totals: 9216 LUT16
writes, zero kp/prog verification failures, zero restart failures, zero
TX-empty/RX-overflow/DMA/BitScrambler faults, and no watchdog/abort. Program
restarts measured 5.94-6.19 ms, kp 5.78 ms. The observer remained alive.
This proves that tested manual sequence, not all flight conditions or the
AUTO target curve; continuous-video quality and the two-second AUTO dwell
still need field observation. Firmware was app-only flashed with a matching
partition layout and NVS/PHY left intact.

Sampling drift was observed separately and is not claimed fixed here.

## Review follow-up: recovery, calibration and output level (2026-10-12)

PR comments 6123181977 and 6123278553 identify remaining safety and
compatibility gaps. The first comment reviewed obsolete 65946f1; its
32-bit LUT-port endorsement was contradicted by the physical tests above.
The corrected LUT16/HALT access is retained.

`transport_recovery.h` drives the production restart: checked teardown,
BS enable, program load, instruction readback, complete LUT16 write/readback,
BS reset/start, RX/TX enable, RX start and bounded TX start. The last good
LUT/configuration is published only after the whole chain succeeds. Each
failed candidate is cleaned up before exactly one fully verified rollback;
failed rollback or teardown latches live OFF and reports an error without
aborting/retrying indefinitely. USB runs independently. Menu return and boot
use this same verification path. `test_transport_recovery.c` injects every
candidate and rollback stage failure, initial-load failure and failed cleanup,
and verifies publication only after successful final DMA start. These are
host coordinator tests, not physical SDK failure-injection evidence.

Manual P/k save AUTO OFF once, and AUTO is stopped/reconciled after restart
errors. Missing/invalid settings select AUTO kp+LP by the operator's explicit
decision; existing saved OFF is respected. Invalid probe/menu/scan windows
clear target confirmation. Five-second backend cooldown and two-second
continuous usable-evidence confirmation remain in force.

Issue #199 has no attached raw snapshot. A matching synthetic +1.23/-3.26
cell DC and 1.7-cell complex noise test now runs the exact shared quiet and
filter-measurement helpers, with no threshold change and no prior DC search.
100 synthetic bandwidth settings complete without a definite-carrier abort
and leave eligible quiet candidates. A too-narrow/noisy setting can be
skipped, just as in the real sweep. Weak FM (-1..3 dB model C/N) is refused
as quiet; failed/short probes, recent sync, origin and rails retain separate
refusals. These 100 synthetic settings are not the actual PHY register sweep.
The reporter's complete physical sweep and subsequent DCO remain pending.

Issue #200 manual OUTPUT is integrated via g/menu. Boot is always 1x;
1.5/2/2.5x change only DAC6 low bits. Transformation is applied to original
kp/program tables, never cumulatively to an already-scaled table. Verified
level/program/output survive menu return and rollback; T reports clipping
as the count of transformed LUT entries, not live sample clipping. HDZero
acceptance with the reporter's low-deviation VTX and a wide VTX is pending.

`research/output_level.py` uses the existing bit-exact U85Q and LP models
over PAL/NTSC, deviations .69/1.0/1.7 and C/N 6/30 dB, plus an unequal
sync/picture swing approximating issue #200 (-2.82/-1.5/+3.7 MHz): 160 model rows,
including four manual output levels and an ideal sync/white-referenced
candidate. All eight kp LUTs additionally pass identity and metadata
preservation checks. The DAC model assumes ideal 3.3 V GPIO, nominal
resistors, 200-ohm series resistor and 75-ohm termination (maximum .623 V
loaded). This differs from the issue reporter's nominal voltage model:
neither is a measured board voltage, and actual output impedance/load must
be established before interpreting compliance or selecting a voltage target.

Example, PAL strong carrier, U85Q, deviation 1.7: manual 2x clips 23.65%
of pre-LP samples and luma SINAD falls from 15.63 to 11.55 dB. The ideal
sync/white-reference candidate selects 1.19x, clips .64% and scores 15.68 dB.
At deviation .69, the nominal loaded sync depth rises from .060 V (1x) to
.127 V with the ideal 2.5x candidate, with .66% clipped samples. The high
gain cap prevents this example from reaching a nominal .286-V sync target;
gain does not increase the DAC's physical voltage span. Noise and transient
excursions can still clip even with ideal reference headroom.

The existing goggle metric fits analogue amplitude away, so its normalised
picture score cannot prove AV-input compatibility. The simulation separately
reports unnormalised nominal depth/swing/clipping. It demonstrates potential
benefit and the harm of blind 2x, not a tested live estimator, HDZero model,
actual sensitivity gain or flight-safe automatic output. Automatic OUTPUT
is consequently not enabled. A future implementation needs reliable sync,
porch **and white/burst headroom** measurements, context resets, failed-write
rollback, at least 2 s stable evidence and the shared 5 s application gap;
it must hold on invalid/weak observations rather than follow a fade.

Follow-up board evidence: firmware `5.0.0-pr.196-output2` passed all eight
kp levels in both directions including wraps, two P-spam bursts, 1x/2x menu
return, 2x preservation across kp/LP and an OFF-persistence reboot. Before
reboot: 16 kp and 3 program changes, 25,600 total verified LUT writes,
zero verification/restart/transport faults and no watchdog/abort. Kp changes
were about 4.2-4.4 ms and program changes about 3.8 ms in this VTX-off run.
After testing, AUTO kp+LP was explicitly restored and OUTPUT was 1x.

Manual F first caught a follow-up regression: full recovery teardown was
also stopping RX during the menu/status raster, so all calibrator copies
failed. The correction keeps RX and its descriptor metadata alive while
normal menu/status screens own TX; failed pipeline recovery still stops all
enabled parts. The production stop-parts helper now has a host regression
assertion for retained RX plus complete cleanup even when a stop fails.

Repeating F on the board (operator confirmed VTX OFF) then measured all
100 settings, plus calibrated baseline/recheck (102 setting log records),
and stored normal BW40 code48/skirt8, width 13.125 MHz, NBW13.280 MHz;
narrow code60/skirt0, width11.875 MHz, NBW11.899 MHz; wide code24/skirt0,
width18.125 MHz, NBW18.245 MHz, calibrated NBW20.612 MHz. It subsequently
searched 12 DC rows and measured a floor at gain72; no missing rows remained.
The bounded startup finished in 119.205 s without no_quiet_channel,
sample_failure or new transport/verification faults. Sampling phase still
reported unsettled and is an explicitly added investigation, not a passed
test. This is this board's VTX-off acceptance, not the #199 reporter's capture.

Multi-Rescue proposal test (`research/rescue_study.py`, data/rescue_study.json
and standalone plots): 64 identical-Q4 cases x 17 candidates = 1088 results,
with eight kp values for U85Q/LP and a floating adjacent-FM same-Q4 reference.
Clean cases have four seeds (two PAL/two NTSC), deviations .69/1.0/1.4/1.7,
C/N1/4/30 dB and the existing matched filter/baseline. CFO +/-750 kHz, DC1
cell and 3-ms/-30-dB fades use two seeds. Ranking uses worst-seed absolute
model line-lock >=.90, no more than .02 lock loss vs baseline, relative burst
SINAD >= baseline-1 dB and no additional clean DAC-rail exposure, plus <=1 dB
strong-signal loss. This relative burst gate does not prove decoded colour;
negative burst SINAD may still pass if the baseline is equally impaired.

Examples: .69/CN1 LP kp.50 gives +1.98 dB worst luma SINAD vs MATCHED,
worst modeled lock .960; 1.0/CN4 LP kp.70 gives +1.17 dB, lock .995;
1.4/CN4 LP kp.85 gives +1.79 dB, lock .989. No candidate meets the gates
at 1.4/1.7 CN1, the modeled 3-ms fades, or wide/DC1 at CN4. These are
conditional noise/detail improvements, not attenuation-margin measurements
or recognisable moving-video proof. Four new hardware programs are not
justified by this result. The matched-topology restriction is reconciled
since (research README section 17); no universal automatic rescue claim.

The first non-oracle OUTPUT probe (`research/output_probe.c`) runs the
production AFC/Direct Gain helpers passively. It rejects all modeled CN-10,
CN1 and two windows of a real board VTX-off snapshot. Strong narrow PAL
accepts191/196 windows; median sync bias .86 kHz, p95-p5 spread15 kHz,
candidate gain1.61 with spread.039. The conservative five-sync-span white
reserve refuses normal/wide candidates; the global C/N proxy also
underestimates wide strong-carrier confidence. This prototype therefore
does not qualify a universal automatic OUTPUT servo. White percentiles
from a few lines cannot guarantee headroom for a subsequent bright scene.

## The ladder froze at the range edge (2026-10-13)

C5VRX by Twotoz and the C5VRX contributors. The kp + LP ladder only helps if
the control loop lets it reach the edge, and two parts of `control.c`
stopped it there (research README section 18, `research/ladder_gate.py`):

- The ladder and its switch confirmation shared the AFC window gate, which
  includes the envelope-stationarity test. Receiver noise near the FM
  threshold fails it: on the 12 MHz class 46 % of control windows pass at
  0 dB C/N and 85 % at 3 dB (sim.py IQ). Each failure held the ladder and
  restarted the confirmation. `kp_ladder_window_usable()` now gates the
  ladder on settle, gain write and context only; the AFC keeps the test.
- The confirmation wanted the exact (kp level, program) for 2 s while the
  ladder steps a level every 0.2-1 s. `kp_switch_confirm_toward()` confirms
  the program and the direction instead and applies the latest level;
  oscillation around the running pair still never switches.

Control-loop model with the real headers and the best_combo picture model
(16 seeds): arriving at the edge and staying there, the C/N where half the
windows still show a picture improves by 1.0-2.1 dB in 10 of 15 cases
(deviation 0.69 / 1.0, 12 MHz and MATCHED, with and without fades) and lands
within 0.2 dB of an instant oracle; deviation 1.4 under fades and MATCHED 1.0
under fades gain 0..0.3 dB. A slow steady fly-out changes -0.2..+0.2 dB, which
is why a walk-away test can hide it. Simulation only; board check: `kp_steps`
and `lp_switches` in `T` should follow the C/N at the edge.

## Sampling phase on the carrier, WIDE rows (2026-10-13)

C5VRX by Twotoz and the C5VRX contributors. Host model (research README
section 19): a mid-transition MODEM_DIAG read costs 0.3-0.6 dB of range
when one LSB lane mixes and 1.4-5 dB when several bits do. The boot check
ran on receiver noise when the VTX was off and only reported in-flight
drift. The firmware now chooses the cleanest of the three positions on the
carrier once per boot, probes it every 10 s against half the old limit and
repairs a confirmed excursion with a short scan (no per-boot cap; false
alarms back off to 16 min). The WIDE profile gets its own 16 MHz kp/LP rows
(sharper picture; no range gain exists for wide VTXs in this demodulator),
MATCHED uses the 10 MHz rows only for a filter measured <= 11 MHz, and
FIELD_DIAG logs kp and LP per second. Host tests and a local ESP-IDF 6.0.2
build pass; board check pending.

## Board walk: the ladder never switched (2026-10-13)

C5VRX by Twotoz and the C5VRX contributors. Board check of this PR
(`5.0.0-pr.211`, kp_auto=2, A1 5865 MHz, DEFAULT 12 MHz profile), VTX
walked through the house to spots where the picture was certainly gone.
Data: `research/data/walk_2026-10-13/` (`walk_serial.log`: FIELD_DIAG 1/s
plus `T` every 5 s; VTX on ~57-171 s; `off_*.bin`: IQSNAP with the VTX off).
Findings only, no fix yet.

- **Nothing was applied.** `prog_switches=0`, `kp_switches=0`, `kp_level=0`,
  `program=U85Q` the whole run. The internal ladder moved (`kp_steps` 1 -> 8,
  `lp_switches` 0 -> 6) but `kp_switch_confirm_toward()` never confirmed.
- **The gain never left G59-60 for a weak carrier.** Table maximum is 83.
  With the VTX on: 76 of ~114 s at G59, none above 60, `sync_valid_pct=0`
  in all but 17 s; AFC never locked, deviation never measured.
- **With no carrier the IQ is a constant DC point, not noise.** VTX off,
  G59, four snapshots (`research/dc_coherence_check.py`): DC
  (-1.5, +2.9..3.3) cells, spread 0.06-0.08 (I) / 0.4-0.5 (Q) cells, 3 I
  and 2 Q codes in 8190 bytes. Receiver noise is below one LSB at G59.
- **The DC point reads as a strong coherent carrier.** dg_measure counts a
  phase step <= 45 degrees on power >= 8; a fixed point has no phase step,
  so coherence 99-100 %. `cn_estimate_x10` turns that into 14-20 dB
  (FIELD_DIAG `cn_est_x10=200` with the VTX off, before and after the walk).
  With the window DC removed the same captures read coherence 0 (noise
  floor). `carrier()` in gain.c (coherence >= 55, P50 >= 5) is also
  satisfied by the DC point, consistent with the gain parking at G59.
- **So the ladder steers on the DC, not on the VTX.** With the VTX weak the
  C/N estimate swung 0-12 dB second to second with `sync_valid_pct=0`; the
  wanted program crossed the 8.5 dB LP threshold (1.5 dB hysteresis) back
  and forth, and the 2 s directional confirmation restarted every time.
- **During the fade itself the ladder was blind.** 57-77 s: 50-120 gain
  writes per second; `kp_ladder_window_usable()` rejects any window with a
  write in the previous 100 ms, so the smoothed C/N was not even updated.

Where the DC comes from and why the gain stops at G59
(`range_diag_vtx_off.txt`, `T` DCO line; gain.c, calibration.c):

- **G59-60 are DCO FLOOR rows, so their DC is never corrected.** The DCO
  table covers the top RF stage, G54-83 (`top_rf_stage_from=54`): 14 rows
  OK, 16 FLOOR. `dco_quiet_search()` marks a row and every row below it
  FLOOR when the receiver noise there is below one quantizer step ("nothing
  to correct"), so FLOOR is G54-69. Below the noise step the DC is still
  there: (-1.5, +3) cells at G59, `dc_ok=1`, `dc_held=0`. The noise being
  below one step is exactly what makes the DC a clean constant.
- **That DC point sits inside the Direct Gain healthy band.** `healthy()`
  wants `carrier()` (coherence >= 55, P50 >= 5, origin < 650) and P50
  13..32; the DC point gives coherence 99-100, P50 ~15, origin 0. The gain
  climbs while the DC is below the band and holds as soon as the DC alone
  reaches it: G59-60, about 23 steps below the maximum. `no_carrier`
  (P50 <= 4, origin >= 650, coherence < 20) can never fire on it, so the
  "listen at the table maximum" path is never taken.
- **A weak VTX is therefore invisible.** At G59 even receiver noise is
  below one LSB, so a carrier near the sensitivity limit is too; noise
  only dithers the quantizer from ~G70 (first OK row). The range edge is
  set by where the DC point parks the gain, not by the demodulator.
- `RANGE_DIAG verdict=OK strong carrier with gain headroom` with the VTX
  off, for the same reason.

This is not PR 211 code: gain.c and the DCO FLOOR rule are unchanged by
it. PR 211's ladder only made it visible, because its C/N input is the same
coherence.

At the sync edge, after a reboot (`edge_serial.log`, `weak_*.bin`; VTX
placed where the picture is between desync and just video):

- The gain now sits at G70-76 (OK rows, `DCO held=1`) and receiver noise
  is visible (spread 1.8-2.5 cells, 11-13 codes per axis), DC still
  (+1.4..2.2, -0.8..-1.3) cells. Coherence 54-68 %, `cn_est_x10` mostly
  CN_UNKNOWN or -2..+3 dB, briefly 3-6 dB when sync returns.
  `dc_coherence_check.py` reads 3-17 points lower without the DC (50-68
  vs 33-65 %): the DC still lifts it, but no longer dominates.
- Still no switch in 40 s (`prog_switches=0`, `kp_switches=0`), internal
  `lp_switches` 4 -> 6. Here this is the curve, not a gate: the deviation
  is unknown (`deviation_x100=0`), so the ladder uses the wide class, whose
  LP window is 2.0-8.5 dB (`kp_lp_floor_x10` 1.5 dB + hysteresis/3) and
  whose kp row is 0.85 up to 2 dB. In that firmware a noise-like window
  counted as 0 dB, so at the edge the wanted pair was U85Q kp 0.85, the pair
  already running. This historical behavior is superseded by the 2026-10-21
  provenance fix below: UNKNOWN now holds C/N-driven decisions.
- The deviation is never measured because `profile_observe()` needs
  `s_afc_locked`, and the AFC never locked in either boot, even close in
  (sync 52-62 %, `afc_corrections=0`). With no deviation the ladder can
  only ever use the wide class.

Open: whether the G59 park returns in this boot with the VTX off; why the
AFC never locks (burst gate `AFC2_BURST_MIN_X10`, CFO -476 kHz seen once);
an IQ capture of the weak carrier at G59 for comparison.

Hard check of the steering (`research/field_walk_analysis.py`,
`research/ladder_replay.py`: the C ladder and guard in Python, wide class,
each 1 s FIELD_DIAG C/N held for its 20 windows):

- **The C/N alone would have switched LP 15 times in the walk.** Best case
  (every window usable): first apply at 60.8 s, 15 internal LP changes, 15
  applies. Board: 6 internal changes (from 85 s), 0 applies. So a gate
  outside the C/N held the ladder.
- **One unusable window resets everything.** The guard wants 40 unbroken
  usable windows (2 s), the LP hold 20 (1 s). Random unusable windows in
  the replay: 1 % -> 13 applies, 5 % -> 3, 10 % -> 2, 20 % -> 0 (6 internal
  changes at 10 %, as on the board). `measurement_coverage_pct` while the
  C/N was in the LP band was 50 % (walk) and 49-62 % (edge); that counts
  the AFC envelope test and failed window copies too, so it bounds the
  ladder's share from below only. 57-77 s add the gain writes (50-120/s).
  Which gate fails the other windows is not visible in the log: `T` has no
  counter for unusable ladder windows or guard resets.
- **Even the best case flapped in that firmware.** 15 program restarts in
  114 s in the walk; at the edge LP was on for 2-4 s and off again, because
  a noise-like window counted as 0 dB, below the wide class's 1.5 dB LP
  floor. UNKNOWN no longer has that meaning (2026-10-21 fix below).
- **The deviation decides most of the range and is never known.** The
  wide class (unknown deviation) keeps U85Q kp 0.85 below 2 dB and gains
  0.36 dB from the ladder in simulation; for deviation 1.0 the kp + LP
  ladder is worth 1.6 dB and for 0.69 3.35 dB (research README section
  16). The board never measures it because the AFC never locks.
- **Max gain is not the issue in boot 2.** At G70-78 receiver noise spans
  1.8-2.5 cells per axis, so the 4-bit quantizer is not what limits C/N
  there; more gain would mainly move the envelope out of the band. The
  G59 park of boot 1 is the gain problem (DC point, above).


### First fixes (same PR, 2026-10-13)

- **Fixed IQ point is no carrier** (`gain.h` `dg_stuck`, `gain.c`):
  `dg_measure` counts samples equal to the one before. VTX off at G59:
  476-873 per mille per 64 bytes; weak carrier in noise at G73-75: 15-301.
  At >= 400 per mille with P50 <= 32 the window is no carrier: Direct Gain
  listens at the table maximum (host test: G59 -> G83 on the board's DC
  point), and `receiver_obs_cn_x10()` returns CN_UNKNOWN for the ladder,
  FIELD_DIAG, the profile gate and the AFC strength test. This removes the
  G59 park and the 14-20 dB reading without a VTX; it does not correct the
  DC of the FLOOR rows (that needs a DCO search that works without noise
  dither, still open).
- **Unusable windows hold, they do not erase** (`kp_ladder.h`,
  `kp_switch_guard.h`, `control.c`): the pending kp/LP change and the switch
  candidate survive unusable windows and failed copies; a gap of more than
  500 ms without a usable window restarts them. Replay on the walk C/N
  (`ladder_replay.py ... p_bad new`): 10 % unusable 15 applies (old: 2),
  20-30 % 13 (old: 0).
- **Gain-write guard 5 ms** instead of 100 ms plus "no write during the
  window": the C/N is one observer probe taken right before it is read, and
  Direct Gain settles a write in 0.3-1 ms.
- **`T` LADDER line**: windows, unusable by reason (settle, gain write,
  context, copy failure), fixed-point windows, guard confirms and resets,
  current level, LP and the smoothed C/N.

Not changed yet: the wide-class curve with an unknown deviation (U85Q kp
0.85 below 2 dB, LP only 2-8.5 dB) and with it the LP flapping at the edge
(the replay best case still applies 15 program changes in 114 s); the AFC
never locking, so the deviation is never measured; the FLOOR-row DC itself.

### Walk 3 on 2d0088f, and a revert (2026-10-13)

`walk3_close_check.log`, `walk3_gain_loop.log`:

- **Close in (G25-36, 0-120 s):** no fixed-point windows (`stuck_iq=0`),
  but the C/N model still dipped below 8.5 dB in 19 of 115 s with a perfect
  picture, and LP was applied twice (46-51 s, 124-130 s). Next to the VTX
  the smoothed C/N sits at 9-13 dB, only just above the LP threshold: the
  coherence model reads too low on a strong real carrier (sim tables:
  deviation 1.0 PAL; a 45-degree step test also fails on fast FM swings).
- **Direct Gain looped** after the fixed-point change: with the VTX off
  ~1400 writes and hundreds of overloads a second, at the weak spots
  75-600/s. The fixed point sent the gain to G83; the first window after
  the write still carried the row's uncorrected DC and clipped, the
  overload dropped the gain into the FLOOR rows (18 this boot, G54-71, DC
  up to (-4.3, +3.8) cells, P50 51-87 with clipping) and round again.
  **Reverted for Direct Gain**: `dg_stuck` now only feeds the C/N model.
  The G59 park with the VTX off is back; the fix belongs in the DCO path
  (correct the FLOOR rows' DC), not in the gain controller.
- **Weak spot 147-195 s:** sync 0 at G60-67 (FLOOR rows) with coherence
  85-98 and C/N 4-12 dB: the DC point with a little carrier on it (span > 2)
  still inflates the coherence. Another reason to fix the DC itself.
- 12 program and 11 kp switches in 250 s; `copy_fail` 44 % of control
  windows (1812 of 4132).

### Second round of fixes (2026-10-13, not yet board-tested)

- **FLOOR-row DC** (`phy_dco_search_flat`, `calibration.c`, DCO_VERSION 7):
  rows whose noise is below one quantizer step get DCO codes by bisecting,
  per axis, the code where the fixed output code flips (DC within half a
  cell). Only rows without a flip in +-96 codes stay FLOOR. This targets the
  root of the G59-67 park ("black screen, gain stuck at 65"). Board check:
  boot log `DCO flat ... ok` lines, `T` DCO `floor_rows` near 0, gain
  reaching G75-83 with the VTX off.
- **LP follows the gain** (`kp_ladder.h` KP_LP_GAIN_ON/OFF): LP from G70,
  back below G65, with the fast entry and adaptive release above. kp within
  a program still follows the C/N curve.
- **AFC through missing windows** (`afc2_ctrl_miss`): failed copies,
  non-stationary windows and windows without a burst no longer clear the 16
  estimates; more than 20 in a row (1 s) do. Host test: two thirds missing
  keeps 14 of 16.
- **Control-window copy** holds the scheduler for the copy. Board check:
  `T` LADDER `copy_fail` near 0.
- `ladder_gate.py check` passes with the new timing (after 0.48-0.82 dB vs
  oracle 0.50-0.90).

Follow-up: AFC now also accepts syncs confirmed by the preceding line's
PAL/NTSC period, so a B/W camera can supply measurements without a colour
burst (`d199d02`). The short control window may contain only one sync;
such windows remain missing evidence. Host tests/build passed; board test
pending.

### Envelope C/N instead of phase coherence (2026-10-13, board test pending)

The phase-step coherence model read 9-13 dB next to the VTX and could select
weak-signal kp levels on a strong carrier. A fast FM swing also fails its
45-degree phase-step test, so the model depends on picture/deviation.

- `dg_measure` now computes the DC-removed envelope statistic
  `100 * mean(|z|^2)^2 / var(|z|^2)` on the existing observer probe.
  A fixed IQ point has no residual power and remains unknown. This extends
  the envelope statistic already used by calibration, outside the hardware
  sample path.
- `cn_estimate_env_x10` uses four filter-class tables. All runtime C/N
  consumers (kp, AFC correction, profile gates, diagnostic traces and
  per-second records) use the envelope estimate. Fixed-point windows are
  excluded from the per-second envelope average; a majority makes C/N
  unknown. `T` GAIN adds `env_x100`, `stuck` and `cn_x10`.
- Reproduction: `python research/cn_envelope.py`; full results in
  `research/data/cn_envelope.txt`. PAL, two seeds, deviations 0.69/1.0/1.4,
  256-byte probes matching `copy_probe_window`. Deviation 1.0 medians exactly
  reproduce the tables. Noise-only medians are 114-116; the unknown cutoff
  is 130. Deviations 0.69/1.4 differ by up to ~1.1 dB through 12 dB C/N
  (the earlier ~0.5 dB claim was too optimistic). Above ~15 dB, filter AM
  and quantisation limit accuracy; values saturate at 20 dB.
- Host checks cover analytic envelope moments, DC translations on both
  axes, fixed and rotating IQ, monotonicity over the full uint16 range,
  and the 6 dB AFC boundary for every filter class.
- Log audit: normal and narrow sometimes share the same setting at
  13.125 MHz, but the old class lookup matched narrow first and reported
  10 MHz. Normal now wins a shared setting; a distinct narrow setting
  measured wider than 11 MHz also uses the 12 MHz C/N model, matching the
  kp ladder's existing width boundary.
- `range_diag_vtx_off.txt` said `OK strong carrier with gain headroom`
  while `video_recent=0` and the last 60 s had no sync. The strong-estimate
  verdict now requires recent video evidence and is labelled
  `VIDEO_EVIDENCE`; it is not a claim about displayed picture quality.

Validation: all 13 C host executables pass (including 68 kp-ladder and 24
gain-fade checks), diagnostic-log tests pass, the full envelope sweep
reproduces the tables, and `ladder_gate.py check` passes. ESP-IDF v6.0.2
build passes without warnings.

### Gain-driven LP independent of IQ windows (2026-10-13)

Operator: from G70 the receiver should stay on LP. The gain thresholds were
already present, but failed copies exited the control loop before the policy
and gain-write guards held its confirmation. A decision based on gain still
depended on C/N window availability.

- Separate evidence for LP and kp: in AUTO kp+LP, G70 enters LP, G65-69
  retains it, and below G65 releases after the adaptive hold. The existing
  rescue confirmation/restart gap remains. Copy failures, unknown C/N and
  gain writes do not block the program policy; missing C/N holds kp.
- Retune settle, scan/calibration/menu ownership, unknown verified transport
  state and manual modes still block automatic switching. Pending program
  and kp evidence have independent stale timers. Failed copies remain
  missing AFC evidence and never read the stale sample buffer.
- Host regression follows G70 with no usable IQ through the actual switch
  confirmation: LP is confirmed within 1 s, without creating C/N or moving
  kp. It also covers G65-83 staying on LP, G64 release, ownership and manual
  mode. No hardware/range claim; board test pending.
- The envelope calculation uses integer raw moments in the existing sample
  loop and translates them to the DC centre once per probe. This removes
  the unfinished implementation's two extra passes and per-sample floating
  point. A direct centred oracle agrees within one ratio unit on 64
  asymmetric/correlated probes. Actual C5 observer timing remains to be checked.

These are simulation-derived decision estimates, not independently measured
RF C/N. After flashing, compare VTX-off and close-VTX logs, kp selection,
`afc_locked`, and `obs_us_max` to check the added observer cost on the C5.

## PR #212 receiver update (2026-10-18)

Direct Gain V6 adds same-context carrier memory to V5: a 10-ms fade hold,
eight-index climbs per 3 ms, and a one-write overload return to the remembered
healthy gain. Host models cover this behavior; measured range benefit remains
unproved. GAIN (`G`) saves AUTO (V6, default), MAX or fixed G78..G24.
FLYWHEEL (`U`) is saved and restores a saved ON or OFF. Default without a
saved setting: ON since 2026-10-18 (operator: the PR #218 witness fix held on
the board; was OFF).
Menu entry checks flywheel release before reserving 384-byte internal DMA
descriptor chunks: about 18/22 KiB for interlaced NTSC/PAL, with a 9/11-KiB
progressive fallback. Earlier board tests exposed heap fragmentation and
camera/flywheel drift; boot/menu and flywheel ON/OFF must be checked on the
cleaned build. [PR212](PR212.md) records the current scope and evidence limits.



## Bounded background work during sync loss (2026-10-18)

Extends C5VRX by Twotoz and the contributors' existing hardware dataplane
and native-pulse flywheel on main `275606a64508`; see the legacy
`pr-derived-findings.md` CPU-plane boundary and PR212's ownership rules.

The output alarm used to spin from its early admission time to every tip,
even without camera authority, while yielding to a returning camera, or
without the H/V evidence needed to repair that tip. Those early alarms now
advance the same clock and apply pending publications immediately. Allowed
repairs keep their timing checks, original end time and native suppression.
Already late alarms retain their miss/phase accounting. This avoids spending
the admission lead in an ISR when no DAC write is permitted.

RMT draining snapshots the producer head. Batches arriving during parsing
remain queued for a subsequent drain instead of extending the current PHY
lease. The existing overflow, source invalidation and rearm paths remain in
force. The gain timer also coalesces overdue callbacks: a notification asks
for the current completed IQ window; it cannot recover historical samples.

Evidence: production-source host fixtures check early return in all three
non-repair states, camera acquisition, wrong-standard recovery, noisy PLL
holdover, delayed interrupt/backlog handling, and an RMT callback publishing
a batch during parsing. These are software regressions, not measured CPU
occupancy, physical H/V lock or RF sensitivity. No demod LUT, filter, gain
policy, or acquired sample rate changes. Board checks still needed: compare
IDLE availability, gain response, RMT overflow/rearms, output late/miss
counters and goggle rolling on the same weak-signal setup, with flywheel
both OFF and ON. Absolute RF/demod sensitivity remains unchanged.


## Flywheel witness freshness and delayed observations (PR #218, 2026-10-18)

C5VRX-5 version 5.0.229 reported NTSC/AFC lock and zero transport faults,
but flywheel ON rolled the operator's picture. Two snapshots showed
`camera_present=0`, `h_trusted=1`, `v_trusted=1`; repairs rose by 53,960
and missed tips by 978 between host requests approximately six seconds apart.
Maximum IRQ delay was 32,600 ticks at 20 MHz (1.63 ms). Turning flywheel OFF
cleanly restored ordinary live output; the operator confirmed a stable picture.
Selected original lines and full-log hashes are in
[`flywheel_desync_2026-10-18.json`](../research/data/flywheel_desync_2026-10-18.json).
The strict model's `valid=0` alone is not proof of a bad grid: that diagnostic
model is deliberately separate from the camera-phase guard.

The reproducible software defect was a mismatch between camera presence and
repair authority. Instantaneous GPIO2 sync levels only suppressed clamps while
the 20-ms presence flag was fresh, although H/V authority survives five seconds.
An unreadable source also returned the same negative edge result as a missing
pulse. Production-source host fixtures reproduce a clamp over an asserted sync
level (or unavailable source) after presence expires with authority still set.

Live sync levels now always win, including during a partially emitted repair.
Unreadable source ownership and raw observer coverage older than 9 ms refuse
repairs before any clamp. The existing observer model already uses this 9-ms
age bound. Completed non-sync runs count as coverage: real noise with no camera
H can still use qualified holdover; observer blindness cannot. Coverage gaps
revoke old H/V authority and require fresh acquisition. Old, overflowing or
implausibly future runs cannot renew authority or steer the grid. Runs from
before a pending grid change was applied cannot adjust the replacement grid.
PLL baselines use the observed halfline, not the later ISR tip at delivery.
`SYNC_FLYWHEEL_WITNESS` exposes freshness, sample age and yield/gap/rejection
counters separately from the existing status format.

The actual-source runtime fixture covers fresh native priority, stale/unreadable
witnesses, reacquisition, PAL/NTSC switching, and completed observations in 6-ms
batches with three noise pulses per line. Its 10-second noisy scenarios retain
zero H reanchors and less than 9 ticks of late-run phase error after settling;
one-second camera holdover stays within the existing 40-tick drift limit.
These are software checks. They do not prove physical goggle lock or recovery
of the measured 1.63-ms interrupt stalls. The bounded background work from
PR #217 is included with its original authorship; pulse deadlines and native
priority remain strict. No demod LUT, RF gain policy or sample path changes.

Review follow-up (PR #218 comments): `SYNC_FLYWHEEL_LOAD` reports measured
`output_alarm` occupancy since the previous print: ISR calls, worst single ISR
(us) and per-mille CPU for the whole ISR, the early-alarm spin, the spin before
tips a native pulse then suppressed, and clamp holds. The ~19 % figure in the
review is a ceiling estimate; these counters replace it with a measurement per
scenario (strong camera, weak, none, flywheel OFF). Board numbers are pending;
hardware-timed OE gating is not attempted before them. Separately, gain-driven
LP (G70 on / G65 off) now applies only under Direct Gain AUTO: a manual MAX or
fixed G70+ forced U85Q-LP on a strong carrier; manual gain falls back to the
C/N rule (`kp_ladder_gain_evidence`, `test/test_kp_ladder.c`). Host tests only.

Before claiming the board issue fixed, compare this PR build OFF/ON on the same
camera: boot/menu/restart, native return, witness age/gaps, RMT rearm/overflow,
IRQ delay and missed tips, plus operator-visible rolling. The board remains
on the stable OFF setting while this software change is prepared.

## Deviation measured without the AFC lock (2026-10-18)

C5VRX by Twotoz and the C5VRX contributors; extends the PR #196 VTX
matching and the PR #211 ladder. The kp + LP ladder is worth 1.6 dB
(deviation 1.0) to 3.35 dB (0.69) in simulation only when the deviation is
known; unknown, it runs the wide class (~0.4 dB). The board never knew it.

- **The lock gate kept every window out.** `profile_observe()` returned
  before measuring while the AFC was unlocked. Walks of 2026-10-13: the
  PROFILE line reads `observed=0` in every log, also `walk3_close_check.log`
  close in (254 FIELD_TRACE windows with sync 100 %). Board `5.0.0-pr.218`
  (2026-10-18, 2.5 h uptime): `PROFILE_GATES unlocked=1499` against
  `observed=537`; `deviation_x100` for the session stayed -1.
- **The span does not need the lock.** A carrier offset moves sync tip and
  porch together. The real `afc2_measure()` on `research/sim.py` video
  (4092-byte windows, 12 and 10 MHz filter, PAL/NTSC, deviation
  0.69/1.0/1.4/1.7, C/N 8 and 12 dB, offset -1..+1 MHz): median span within
  19 kHz (1.4 %) of the centred carrier, inside the estimator's 8 % batch
  spread and 5 % agreement; no wide VTX reads narrow. Host test
  `test_afc_measure.c` checks the span over -1..+1 MHz in 250 kHz steps.
- **Change:** the measurement no longer waits for the lock, so the session
  deviation (the ladder's class) is set as soon as the other gates pass
  (confirmed sync, C/N >= 8 dB, healthy band, no clipping, held gain).
  Narrowing the filter still needs the lock (`profile_narrowing`,
  `profile_live_switch_wanted`; README section 3: a -0.5 MHz residual costs
  the narrow settings at the edge). A startup result that asks for a narrow
  profile while unlocked keeps DEFAULT (`narrow_awaits_afc_lock`); the live
  switch narrows after the lock and two agreeing results. Widening never
  waits. `T` PROFILE_GATES now reports `observed_unlocked`.

Why the AFC does not lock (board IQ 2026-10-18, cause still open):

- Lock needs 16 window estimates with a median absolute deviation <= 80 kHz.
  Close in (`walk3_close_check.log`, 254 FIELD_TRACE windows with sync
  100 %) the porch estimate has MAD 211 kHz; the 16-window gate passes 2 of
  238 times.
- It is not estimator noise. One control window holds 1-2 lines (4092
  samples; `afc_bias.py` models 8), yet the same model with 1-2 lines and
  `afc2_measure()` on sim.py video both stay at MAD <= 30 kHz at >= 10 dB.
- **The whole signal moves.** 14 IQ snapshots with the VTX close in on A1
  (`research/data/afc_2026-10-18/`, gain G27-42, AFC offset +500 kHz,
  `research/afc_snapshot_levels.c`): within one 205 us snapshot the levels
  agree within ~90 kHz, between snapshots the porch reads -903..-1472 kHz
  and the sync tip moves with it (-2225..-2739 kHz); the span stays
  1237-1345 kHz (deviation ~0.68). Sync and blanking shift together, so
  this is a frequency shift of the received signal, not a misplaced porch
  window.
- **Timing** (`field_trace_20s.log`, 50 ms FIELD_TRACE windows, 20 s): the
  porch sits near -1000 kHz, then every 0.4-0.8 s ramps down to
  -1300..-1650 kHz over 0.1-0.3 s and snaps back. Gain (G35-37), gain
  writes, kp and LP do not change with it.
- Two candidates fit and these data cannot separate them: the VTX carrier
  itself (supply/VCO pull) or an AC-coupled VTX modulator moving the
  blanking level with picture brightness; the C5 LO moving by ~85 ppm is
  less likely. Next test: the same 20 s with the camera lens covered (flat
  black picture). If the excursions stay, the carrier moves; if they go,
  picture content moves the blanking level.
- Consequence for this PR: a carrier that swings ~0.5 MHz is exactly the
  case in which a narrow filter hurts, so keeping narrowing behind the lock
  is right for this VTX. The deviation itself is unaffected (span constant).
  No AFC threshold is changed before the cause is known.

The same stream also showed one task-watchdog report on `5.0.0-pr.218`
(IDLE on CPU 0 starved, `gain_obs` running, MEPC 0x4202b688) in ~26 s of
log; not investigated here.

Same board, same status: the second-round fixes hold where they can be read
passively: `DCO floor_rows=0` (30 rows valid), `LADDER copy_fail=0` in
180,562 windows. Host tests and an ESP-IDF 6.0.2 build pass; the change is
not board-tested and no range gain is measured.

## Task watchdog: the gain observers took half the CPU (2026-10-18)

C5VRX by Twotoz and the C5VRX contributors; extends Direct Gain V6 and its
200 us observer/sentinel (`receiver.c`, `gain.c`).

- **Symptom.** `task_wdt: IDLE (CPU 0)` with `gain_obs` running, about once
  every few minutes on `5.0.0-pr.218` and on main after PR #220 (MEPC in
  `measure()`, `gain.c:106`). The calibration task (priority 1) starves
  with IDLE.
- **Measured** (local diagnostic build: FreeRTOS run-time counters per task
  and cycle counters per section, 10 s windows, not committed). `gain_obs`
  32-54 %, `gain_fast` 20 %, `control` 18-23 %, `esp_timer` 6.5 %; IDLE
  0.2-17 % (0.3 % in the window before a watchdog report). `measure()`
  costs 28-29 us for the sentinel's 64 bytes and 100-155 us for the
  observer's 256 bytes: ~125 instructions per byte (-O2 disassembly),
  ~1.3 MB/s in total.
- **Ruled out on the board:** flash cache (`measure()` in IRAM and the
  phase table in DRAM: unchanged, 29 us) and compiler level alone (-O2:
  29 -> 24 us).
- **Fix.** Every per-sample quantity depends on the byte or on the byte and
  its predecessor: power, flags and phase from tables, I/Q span from one-hot
  bits OR-ed per sample (no sign extension or min/max), the 45-degree test as
  one unsigned compare, and the envelope moments in their own loop that runs
  only for the 20 ms C/N probe. `test/test_gain_measure.c` compares every
  field with the former implementation (kept verbatim in the test) on 13,800
  windows: random bytes of sizes 1-8190, fixed points/rails, a rotating
  carrier at every phase step and the board IQ captures; a deliberate
  one-code change makes it fail.
- **Board** (diagnostic build with the table-driven measure): sentinel
  15-16 us, observer 49-60 us; IDLE 18-33 % in the worst of three windows
  (was 0.2-17 %); no watchdog report in that run.
- **Board, this version** (cold start via RESET, ELF `127d6b3aa`): normal
  startup and console, 0 task-watchdog reports in 5 minutes of continuous
  heartbeat (VTX not received in that run).
- **Not caused by this change: silent USB console after the flash route.**
  After `` ` `` -> ROM -> flash -> `write-mem 0x600b1034` + watchdog reset
  the USB serial console sometimes stays silent while the receiver keeps a
  picture. It first looked tied to this change (3 failures with it), but
  the unchanged main build failed the same route too (2026-10-19). After a
  replug the board came up in the ROM loader twice, so the board may stay
  powered through the video side; a RESET tap recovers it. Cause open.

## DC table from another filter: 3.7 cells of DC and the gain parked on it (2026-10-19)

C5VRX by Twotoz and the C5VRX contributors; extends the per-gain DC tables
(`dco_slots.h`, PR #196) and the FLOOR-row work of PR #211. Relates to
issue #222 track A (hidden loss in the gain allocation).

- **Board** (main after #221, A1, no VTX received, four IQ snapshots at
  G71): DC (-0.9, -3.6) cells, receiver noise 0.68 cells per axis. `T`:
  `DCO held=1`, `DCO_CONTEXT table_bw20=0 table_code=-1 table_skirt=0`
  against the live `active_bw20=1 active_code=16 active_skirt=32`,
  `filter_same=0`; `SPHASE dc_mcell` -1.1..-2.6 / -2.1..-3.7 cells in every
  status of 2026-10-18/19. The codes are held, but they were measured with
  another analog filter.
- **The gain parks on it.** p50 = 15 comes from the DC point (3.7^2 ~ 14),
  inside the Direct Gain healthy band (13..32): the gain stays at G71
  instead of the table maximum, the same mechanism as the 2026-10-13 G59
  park, now through the filter context.
- **Cost** (`research/dc_board_study.py`, `data/dc_board_study.json`;
  bit-exact U85Q, deviation 0.69, 12 MHz, quantiser scaled to a fixed
  receiver noise per axis instead of the Direct Gain radius used by
  `situations.py`, two seeds): range limit (picture >= 0.9 and SINAD >= 0)
  **14 dB C/N with the measured DC, 4-6 dB without it**; below 10 dB the
  SINAD stays at -1.6 dB (the demodulator follows the DC point). The
  2026-10-13 walk-edge state (DC 2.1 cells against 2.2 cells of noise at
  G74) costs 0-0.3 dB in the same model. Simulation, not measured range.
- **Cause.** `dco_matches()` compares frequency, gain table and IQ scale but
  deliberately not the filter ("an assumption, not measured equivalence").
  The board refutes the assumption.
- **Fix.** `dco_filter_same()` / `dco_filter_stale()` (`dco_slots.h`, host
  test `test/test_dco_slots.c`): at startup a table with rows measured under
  another filter is measured again in full, like a manual CALIBRATE of the
  DC rows, through the existing own-channel (VTX off) and quiet-channel
  (VTX on) paths; the table takes the live filter only after a complete
  pass (`done` or `floor`), so an interrupted pass is repeated at the next
  boot. Live profile switches still keep the table (unchanged, see the
  `dco_matches` comment); a WIDE profile with another filter therefore still
  runs on codes of the normal filter until a boot measures it.
- **Board check** (2026-10-19, ELF `6722a826d`, A1, no VTX; data in
  `research/data/dc_2026-10-19/`): the first boot logged `DCO startup
  filter_stale=1 table_bw20=0 table_code=-1 ... live_code=16 live_skirt=32`
  and measured all rows on the own channel (`searched=30 result=done`); `T`
  then showed `DCO_CONTEXT ... filter_same=1`. Six IQ snapshots: gain **G82**
  (was G71), receiver noise **2.15 cells** per axis (was 0.68), DC
  **1.5-2.4 cells** (was 3.7), DC/noise **0.7** (was 5.3). In the model above
  that state costs 0-0.3 dB instead of 8-10 dB. A residual Q offset of about
  -1.5 cells stays after the search; a DC search should end within half a
  cell, so this is open. No range measurement yet.

## Analog filter branches, per-filter DC, a live DC trim that runs (2026-10-19)

C5VRX by Twotoz and the C5VRX contributors; extends ESPARGOS esp-sdr's C5
filter control (GPL-3.0), PR #196's filter sweep and the DC work above.
Board: XIAO ESP32-C5, A1, no VTX; data in `research/data/filter_2026-10-19/`.

- **Two analog filter branches.** The PHY builds its analog tables at
  channel setup from the bandwidth mode then in force: set up in BW20 the
  RX0 code sets the width and the skirt does nothing, set up in BW40 the
  reverse (same boot: code 63/skirt 0 11.9 vs 19.7 MHz noise bandwidth,
  code 16/skirt 32 21.1 vs 13.0 MHz). `rf_set_channel` tuned before
  `phy_wifi_fbw_sel` and `rf_set_filter` never redid the setup, so the sweep
  measured settings in whatever branch was active: the board flew a stored
  "12.5 MHz" setting at 19.7 MHz noise bandwidth (live IQ spectrum; the
  firmware's own PSD method agrees). `channel_setup()` selects the mode
  first and `rf_set_filter` repeats the setup on a mode change;
  FILTER_VERSION 5 re-sweeps. After: stored 13.0 MHz, live 13.0-13.4 MHz.
  `FILTER_HW` reads the filter registers back (they always held what was
  set; the branch was the problem, not the registers).
- **DC per filter.** One table per (channel, filter key), index version 3
  with an upgrade that keeps measured tables; startup measures the NORMAL,
  NARROW and WIDE filters' tables (resumable, `DCO_READY`); narrowing in
  flight waits for the narrow table. Stale tables are re-measured row by row
  and stamped only when complete (review P1/P2).
- **DC moves at every channel setup.** Same DC words in both PBUS banks
  (`DCO_BANKS`), yet DC (+1.5, -0.1) -> (-0.5, -3.0) cells after 8 retunes:
  a stored table only fits the setup it was measured in. The live trim never
  ran (`DC_LIVE trials=0`): it needed a carrier circle, its sample missed a
  2 ms deadline under the PHY lease, gain writes and sampling-phase probes
  re-held the bare table codes, steps were one code within +-4, and two
  single windows had to agree within 150 mcells while the DC wanders about
  half a cell. All five fixed (noise-mean estimate, lease-free sample,
  trimmed holds, sized steps, 8-window mean, deadband 600 mcells).
- **Sensitivity model** (`research/sensitivity_model.py`): required C/N
  referred to the fixed 20.7 MHz chip channel, from no-carrier board IQ (live
  filter shape, noise, DC, digital floor) through the bit-exact U85Q chain;
  dBm only with an assumed NF. Deviation 0.69, 2 dB C/N steps:

  | board state | live NBW | DC | range limit |
  | --- | ---: | ---: | ---: |
  | before #223 (wrong DC table, G71) | 20.5 MHz | 3.6 cells | 14 dB |
  | DC table fixed (G82) | 20.7 MHz | 1.7 cells | 4 dB |
  | filter branch fixed (G83) | 13.4 MHz | 1.7 cells | 4 dB (2 without DC) |
  | after a channel round, no trim | 13.7 MHz | 3.1 cells | 6 dB |
  | after a channel round, live trim | 13.2 MHz | 1.2 cells | 4 dB |

  Simulation at measured board levels; no range walk yet. The digital path
  (quantisation, sampling phase, folding) costs 0.03-0.16 dB.
- **1 dB steps** (`data/sensitivity_2026-10-19_fine.json`): with the live
  trim the limit is **3 dB C/N** (DC 1.16 cells, of which the DC costs 1 dB;
  2 dB without it), against 14 dB before #223: 11 dB in this model.
- **Open:** a DC wander of about half a cell over seconds remains below the
  trim's reach (2 dB C/N steps cannot resolve its cost); carrier-on checks of
  the live profile switch onto the narrow table and of the trim at the edge.

## Boot-to-boot range: the DC of every channel setup (2026-10-19, PR #224)

Operator: "some boots or calibrations the range is great, some terrible".
`tools/boot_loop.py` reboots the board N times with the VTX off and records
IQ snapshots and `T` per boot; `research/sensitivity_model.py` turns every
snapshot into a range limit (required C/N, 20.7 MHz chip channel).

- **The variation is all DC.** Baseline (trial trim), six boots, 60 s after
  startup: DC 1.0-3.8 cells, range limit **2-8 dB C/N**; without the DC every
  boot is 2-3 dB (noise bandwidth 13.0-13.5 MHz, noise 1.9 cells, digital
  path 0.03-0.1 dB). Up to 6 dB, about twice the range, between boots of one
  calibration.
- **Not in any register.** PBUS bank 1 (vendor) and bank 2 (ours) read the
  same on every boot (279-280/226-227, 253/224) while the boot DC ranged
  0.7-3.7 (I) and -1.8 to -4.2 cells (Q): analog state of the channel setup,
  no deterministic correction. Within a boot it wanders 1-2 cells over
  seconds.
- **The trial trim could not follow.** 0-9 accepted steps per boot (0-3 with
  a median estimator for clipped clouds): single steps on 0.1-0.9 s samples of
  a low-priority task, verify-or-revert, a deadband.
- **Live DC servo** (`main/dc_servo.h`, test `test_dc_servo`): the mean of
  every settled observer probe per gain row (the AGC already sums I and Q),
  a computed step on both DAC codes per row from the learned slope. Clipping
  biases the mean towards zero but never across it, so the loop converges.
  Six boots: DC 0.3-0.8 cells after 60 s, range limit **2-4 dB** on every
  boot. Learned slope 170-550 mcells per code: one code is ~0.4 cells, the
  analog floor of the correction.
- **Channel round** (8 x next channel, back): 2-4 dB on four of six boots;
  two were 7-8 dB. One learned a negative slope from the new table's DC jump
  (now: steps are forgotten on any context change, a measured slope never
  flips sign on one measurement); in the other G83 sat at 0.3 cells while
  G82 held 3.9, with Direct Gain moving between rows faster than one row
  collected a mean (now: one probe sum per row, every row corrected). Board
  repeat of both fixes with a channel round and a hard calibration: pending.
- **Video quality** (`main/video_quality.h`, `T VIDEO_Q`): the spread of the
  demodulated frequency on the sync tips and the back porch, which are flat
  by definition, against the sync-to-blanking distance; and sync lines found
  per window. It sees what the no-carrier model cannot (sampling phase,
  profile, offset): the closed loops on it follow. The 2026-10-19 edge
  snapshots with 3-4 cells of DC find no line at all.
- **Live LUT re-centring is no fine DC actuator:** a kp/LUT restart costs
  5.5-7.5 ms of video (`kp_restart_us`) behind a 5 s guard.

## Blackbox reads, flicker noise and bursts (2026-10-19, PR #225)

- **First flight-log reads** (five sessions): `VIDEO_Q` read unknown in every
  second (20-window threshold = the AFC's maximum rate; now 5), and dropout
  traces filled ~75 % of the log with an unstable picture (now one per
  30 s). Without a VTX Direct Gain held G83 for 741 of 794 s, with bursts of
  255+ writes and overloads a second in ~50 s that coincided with 2-3 cells
  of DC; `T GAIN_SAT` now splits saturated windows by rail side, and a burst
  stores an IQ snapshot in the log. Six minutes, VTX off: 1034 one-sided,
  550 two-sided saturated windows, 0-1 overloads (the burst gate held).
- **Flicker (1/f) noise is before the gain.** Low-frequency excess (4-30 kHz
  against 1-3 MHz), VTX off, fixed gains: G83 7.1 dB (noise 1.9 cells), G78
  6.5 dB (1.1), G66 6.0 dB (0.5). A noise added after the gain stages would
  rise to +11..+18 dB at the lower gains; it stays at 6-7 dB, so it scales
  with the gain and no RF/baseband gain split can reduce it. Not a firmware
  fix; ~0.5-1 dB at the edge in the model.
- **Packet-like bursts.** One snapshot at fixed G72 held two strong ~55 us
  bursts ~70 us apart (DC +3.5/+4.5 cells, 16-38 % on the rails); TX is
  locked (`lock_rx_only`), so they are external. 51 snapshots on A1, A4 and
  A8 at G83 held none: rare. The blackbox overload snapshots will show their
  rate, length and channel.

## Direct Gain V7: a peak AGC (2026-10-19, PR #226)

- **Blackbox, VTX on, handheld (session 13):** close in at a C/N estimate of
  15-20 dB the V6 gain swung 10-30 steps within a second with 255+ writes and
  overloads; sync fell to 60-80 % in those seconds (94-95 % in calm ones).
  At the range edge the gain sat at G66-G81.
- **Front end:** the RF/LNA stage is 8 (the highest) over G54-G83, and the
  noise follows the baseband gain (-4.4..-4.9 dB per 5-6 steps) down to the
  quantisation floor (~0.3 cells at G60). The noise is set before the
  baseband: no range is hidden in another gain split.
- **Cause in V6:** the healthy band reached P50 32 (radius 5.7 cells, ~2.5 dB
  under the rails), so multipath clipped, the emergency drop overshot, the
  next window read weak and the climb (8 steps per 3 ms) folded again; and
  2 % of clipping without a rail-side check also lowered the gain.
- **V7 (gain.c):** the classic radio AGC on envelope peaks.
  - Attack only on folding: two-sided clipping in two probes, or 1 ms of
    saturation.
  - Release only when the highest P95 of a 16 ms memory leaves room under
    the target (P95 30). Steps come from a tuple gain model.
- **Model (test_gain_track: Rician/Rayleigh at 30 Hz, Gaussian noise peaks
  as on the board, each write 0.4 ms of garbage):**

  | metric | V6 | V7 |
  | --- | ---: | ---: |
  | close in, picture SNR >= 10 dB | 93.5 % | 94.1 % |
  | Rayleigh fading | 87.3 % | 87.0 % |
  | mid range | 94.5 % | 94.7 % |
  | usable edge (~3 dB C/N), above threshold | 38.3 % | 39.8 % |
  | writes per second, close in / edge | 122 / 90 | 46 / 29 |

  A 30 ms multipath null takes 3 writes instead of 10.
- **Conclusion:** the gain loop is not where range is lost (1.5 points at the
  edge); V7 is about stability close in. Board, VTX off: G83 with 0 writes
  in 2 minutes. Carrier test pending.
- **Tuning lessons:**
  - A level loop on a mean followed every fade both ways.
  - A fixed release level under the receiver-noise peaks (P95 15-19 at G83)
    held the gain a step below the maximum.
  - P95 alone as an attack trigger stepped down at the edge on noise.

## The whole chain against an absolute reference (2026-10-19, PR #226)

The question: the C5VRX loses range against an RX5808, so where between
antenna pin and goggle are dB lost? Every step was measured against the Wi-Fi
PHY's own calibrated per-packet RSSI and noise floor (research/nfprobe,
research/wifi_reference_snr.py).

| step | measurement | loss |
| --- | --- | --- |
| chip front end (Wi-Fi mode) | noise floor -94..-96 dBm per 20 MHz from 5180 to 5845 MHz; packets decoded to -97 dBm | none: NF ~6 dB |
| our receive state (forced G83, AGC off, dump, clocks, IQ and DAC pins) | C5VRX tuned to 5180 MHz; RSSI histogram (vendor AGC, 3 s) interleaved with IQ snapshots at G83 | none within ~1-2 dB |
| self-desense (each ingredient alone) | weak packets still decoded to -93..-96 dBm with IQ pins, all clocks, dump, DAC pins | none visible (traffic limits it to a few dB) |
| 4-bit IQ, U85Q, DAC | bit-exact model against an analog limiter-discriminator, same signals | none: the goggle holds ~1 dB lower, the picture is 4-7 dB cleaner |
| field edge captures | the picture went at -1..+4 dB C/N, where the model loses it | as modelled |

How the receive-state check works:
- The vendor AGC's RSSI histogram had its bulk at -94..-85 dBm (mode -91).
- In our IQ at G83 those packets form a cluster 6-7 dB over our noise
  (4.93 cells^2, NBW 13.6 MHz).
- The expected value from the PHY's -168 dBm/Hz is +6.3 dB.

**Result:** the chain from antenna pin to video reaches about -93 dBm (NF 6 dB,
~1.5 dB C/N in 20.7 MHz), RX5808 class or better. With this calibration,
4.93 cells^2 at G83 is -96.7 dBm, so the receiver can report the absolute
carrier power at its pin. Range lost against an RX5808 at equal distance is
signal that never reaches the pin: that is the next measurement (VTX at a
known distance and power against free space; FPV channel by channel).

## Received level at the chip pin (RX_LEVEL, 2026-10-19)

`main/rx_level.h`, host test `test/test_rx_level.c`. Shown in console `T`
(`RX_LEVEL carrier_dbm=...`) and on the menu's CHANNEL page.

- **What it reads:** the absolute carrier level at the chip pin in dBm.
- **How:** every observer probe's DC-removed mean power is referred to the
  pin with the Direct Gain model, then calibrated against the Wi-Fi PHY
  (4.76 cells^2 of thermal noise at G83 = -96.7 dBm).
- **Accuracy:** +-1 dB over G54-G83 (one RF stage, the checked model). Below
  G54 the RF stage steps are model values, flagged `exact=0`.
- **Noise:** learned per channel with no carrier at the table maximum. At
  5865 MHz (A1) it reads -95.0 dBm, 1.7 dB above the 5180 MHz calibration.
  A1 is at the top of the C5's documented range.

**Use:** a VTX at a known distance and power against free space gives the
dB lost before the chip. For example, 25 mW with 2 dBi each side over 10 m at
5.8 GHz is ~-50 dBm. To stay in the exact range (below about -67 dBm at G54),
test far enough away or at the lowest VTX power.

## KEEL-1: the gain on 3 dB home rows (2026-10-19)

The successor of Direct Gain V7.

**Cause.** Every gain row has its own DC, and the live servo learns a row
only while the gain stays on it. A gain that visits many rows briefly keeps
landing on stale DC. Blackbox session 13 at the edge:
- DC above 1 cell gave 15-25 % video, below 0.5 cell 35-40 %;
- a steady gain gave 75 % video, V6 hopping 28 %.

**Design.** The V7 peak AGC, restricted to home rows 3 dB apart from the
table maximum down. A release only goes to the next home row that fits under
the target, so there is no hunting between rows. The peak memory is 16 ms:
the rows do the steadying.

**Search harness.** `test/test_keel_track.c`:
- 12 scenarios: close / mid / edge, K 0-10, 5-80 Hz, a -12 dB obstacle;
- DC per row shrinks with settled dwell;
- each write costs 2 ms of picture;
- score = usable %, edge weighted twice.

| servo time constant | V6 | V7 | KEEL-1 |
| --- | ---: | ---: | ---: |
| 0.1 s | 52.5 | 56.1 | 57.9 |
| 0.3 s | 49.9 | 51.6 | 56.4 |
| 1 s | 44.0 | 41.6 | 52.6 |
| 3 s | 37.2 | 34.3 | 46.3 |

At the usable edge KEEL-1 settles on one row: 0-2.5 writes a second, 40 %
against V7's 38 % above threshold. With a slow servo, V7 did worse than V6:
it used fewer writes but over more rows.

**Open.**
- The real per-row servo time is not measured; the blackbox (DC, gain,
  writes and dBm per second) will say.
- Board, VTX off: G83 with the owner `keel1`. Carrier and flight test pending.

**CI.** A host-test line `cc ... && ./t` let a failed compile pass, because
`-e` ignores failures inside an `&&` list. Each compile is now its own line.

## First KEEL-1 board session and the real RF stage gains (2026-10-19)

Blackbox session 24: KEEL-1, VTX close, hand on the antenna, -25..-56 dBm
at the pin.

- **Close in the gain hunted:** G7-G61 within a second, 255 writes and
  overloads in about half the seconds, picture SNR 0-8 dB at C/N >= 20 dB.
- **After the carrier arrived or left at G79/G83:**
  - those rows held 2-3 cells of DC (IQ snapshots);
  - the picture was gone for 19 s.
  - Cause: observer windows with the carrier folding fed the DC servo. Fixed
    by keeping windows that reach both rails of an axis out of the servo
    (T: `DC_FEED rejected_folded`).
- **The model's RF stage steps are wrong:**
  - IQ snapshots at fixed gains (`research/gain_step_measure.py`), VTX at
    ~2 m: G42 -> G36 (RF stage 5 -> 4) is about -13.5 dB, the model says
    -3.3 dB.
  - An overload snapshot at G2 read ~10 dB above the model.
  - Close in KEEL-1 (and V6/V7) computed steps with the wrong size: the
    hunting. Above G54 (one RF stage) the model holds.
- **Next:** measure every RF stage with the VTX at ~0.3 m and ~8 m, give the
  gain core a measured per-row table, and repeat the session.

## KEEL-1 on the board: what the logs showed and fixed (2026-10-19/20)

Blackbox sessions 24-34, walking with a VTX. Each item was found in the
per-second log or the new evidence records, then fixed at its cause.

| finding | cause | fix |
| --- | --- | --- |
| 255 writes/s at the edge | the 64-sample sentinel forwarded noise rail touches as overloads | gross saturation only |
| wrong gains, '255' seconds | AFC steps and refused writes reset the whole core; counters wrapped | keep core state on the same receiver; resync refused writes |
| rows with 2-3 cells DC after carrier changes | folding windows fed the DC servo | windows reaching both rails are kept out |
| gain 7-10 dB low at the edge | overloads with clean snapshots; a 9 dB step and 1 s memory each | no saturation evidence across a write; 3 dB steps; 300 ms memory |
| demod on its narrowest tracker with LP, no AFC at the edge | the envelope C/N read 3-12 dB low (snapshots: 14-19 dB true) | C/N from power against the learned noise per filter |
| VTX MATCHED screen in flight | status screen at the startup profile switch | the live switch, no screen |
| folds ~41 ms after a release (P50 ~55) | releases too optimistic for slow (walking) fades or a table step larger than the model | learned headroom per row: +3 dB per folded release, decays 1 dB / 2 s |

**Evidence in the blackbox now:**
- every step down with its window (FLOG_E_ATTACK);
- per second: writes down and up, sentinel forwards, overloads within 5 ms
  of a write, the received level and its noise;
- in T: refused writes, rebinds, heat events.

**Still open (needs a VTX):**
- The pair sweep at ~10 m to measure the real steps of G40-G83 (the model
  holds over G29-G39 only).
- A walk with this build.

## DC on every channel: folding, unconverged rows and templates (2026-10-20, PR #228)

Board, VTX off, all 48 channels surveyed 4 s after each switch (`T`:
RX_LEVEL clipped_pm, DC_SERVO dc). 33 of 48 channels had the DC on a rail
(3-6 cells). Four causes, each fixed at its root:

- **Folding, not clipping.** The ultrafine lane keeps the sign bit and folds
  the magnitude bits (`legacy/c5vrx3/main/rf.c`: +288 reads as +32). With a
  few cells of DC the mean of the folded cloud stops growing and then
  shrinks: every channel with a template 4+ cells off read a flat 3.0-3.5
  cells whatever the code. The servo learned a slope of the wrong sign
  there and walked to its limit (Q +188 codes, DAC flat end). The sign bit
  never folds: the observer now counts sign bits per axis (2 operations a
  byte), and the servo (`dc_servo_decide_sign`) learns and steps
  proportionally only where the sign share is 10-90 % (|DC| < 1.28 sigma,
  nothing folds); outside it takes full steps in the sign's direction,
  checks the polarity on the share and flips it on an expanding window.
  Host model with folding (`test_dc_servo.c`, 16 checks): 9 cells pinned
  with a lost slope ends at 0.15 cells (was 10.9-28 cells). Board A1: from
  3.4 cells at the trim limit to < 0.5 cells, trims -17/-16.
- **Unconverged search rows stored as OK.** A startup search from the chip's
  codes with 4+ cells of DC saw no slope (folded) and ran to its clamp
  (G83 on 5805 MHz: codes 316 = base+96, residual 3.5 cells); the row was
  stored and became every other channel's template. Now a row left with
  more than 1.2 cells is missing, stored tables drop such rows on load, and
  missing rows are searched again from a converged neighbour's codes
  (`phy_dco_search_from`, rows ~1 dB apart): 5805 G78-G83 all repaired,
  residual <= 0.3 cells, G83 225/186. A missing row holds the nearest
  converged row's codes.
- **Template choice.** The codes move with the frequency (G83 225/186 at
  5805 MHz, 144/279 at 5925: ~0.7 codes a MHz) more than with the filter
  (15-40 codes): `dco_nearest_slot` now minimises an expected code distance
  (0.7/MHz, +30 for another filter, +20 for a table with template rows)
  instead of "same filter first". Eviction drops the table with the most
  template rows first.
- **Channel change.** Urgent DC wakes skipped the periodic services, so the
  new channel's table was never loaded while one-sided clipping kept waking
  the servo; the servo also loads it itself now.

Template rows trim +-192 codes (own rows +-96), and a template row the servo
holds within 0.3 cells for 8 estimates on receiver noise becomes the
channel's own row (`DCO_ADOPT`): channels calibrate themselves in use.

## Range audit, first pass: the 2026-10-19 walk (session 34, PR #228 data)

`tools/edge_audit.py` on blackbox session 34 (build keel1h, before the
PR #228 DC and gain fixes), every second binned by the envelope C/N, plus
the ten raw edge IQ snapshots (M2M4 on the 4-bit samples):

- **The dropout seconds had the DC on a rail.** The 25 seconds at C/N
  bins <= 2 dB (23 dropouts) sat at 2.6-4.2 cells of DC; the seconds at
  4-20 dB at 0.13-0.26. That is the fold runaway of the DC servo fixed in
  PR #228; the edge of that walk was set by it, not by the RF.
- **No excess noise in the chain.** The snapshots' noise equals the
  startup noise map of their row (G71 0.69 vs 0.687 cells^2, G77 2.09 vs
  2.14): the digital noise is the receiver's thermal noise.
- **The gain was far too low at some moments.** G59 with a carrier of 0.66
  cells radius, G46 with 0.99, G71 with no carrier left: the carrier sat
  within about one 4-bit step, 10-20 dB of sensitivity unused at that
  moment (after attacks, during fades).
- **Two C/N readings disagree by ~7 dB.** In clean-DC seconds the video
  share fell to 35-56 % at an envelope C/N of 4-8 dB, while the level
  meter put the carrier at -86..-78 dBm over a -97 dBm noise floor (11-19
  dB). One of the two is wrong at the lower rows; C/N-AUTO (PR #228)
  replaces the envelope estimate and is checked on the next walk.
- The video share is ~75-89 % even on a strong signal, so it is a relative
  measure only.

## Range audit: static edge, restart gap, self-noise (2026-10-20, PR #230)

- **Static edge (VTX fixed, A1):** the picture fails at C/N -3..+3 dB on the
  20.7 MHz scale, carrier -90..-93 dBm over a -96.7 dBm noise floor (12.9
  MHz): about where the bit-exact simulation puts the U85Q+LP threshold
  (picture 0.03-0.25 at 0 dB, 0.95 at 2 dB). The 8-10 dB read on the walk
  was the old C/N scale plus walking fades. Rating: ~-92 dBm static,
  ~-88..-90 dBm walking (meter calibrated against Wi-Fi RSSI, +-1-2 dB).
- **Self-noise:** console N alternates the receiver noise at G83 with the
  video output (DAC, PARLIO TX) on and off: +0.00 / -0.04 dB over 8 pairs.
  The board does not raise its own floor; the floor is the chip's (noise
  figure ~6 dB from kTB in 12.9 MHz).
- **Restart gap:** T TRANSPORT_STAGES_US. Of ~5.5 ms without picture: LUT
  2.7-2.9 ms (2.2-2.5 with the delta load), program load 0.8, TX start 0.8.
  The IDF program load writes the program's own LUT in 32-bit mode first;
  a target LUT written into the program image would save the second pass.
- **Live LUT writes:** console H. The CPU readback returns nothing valid
  while the BitScrambler runs (1023-1024 of 1024 mismatch with the same
  values written), so a live kp switch cannot be verified; the halted
  restart stays.
- **Gain writes on a still signal:** -47 dBm, VTX and receiver still: 0
  writes, 0 overloads, sync 90 %. The walk's 10-29 writes a second close
  in came from walking fades.

## Antenna-to-pin budget and the gain table below G72 (2026-10-20, PR #231)

VTX 100 mW, whoop antenna (linear) at 2 m indoors, receiver Lumenier AXII 2
(RHCP) on the XIAO ESP32-C5 (U.FL straight to LNA_IN, no antenna switch).
Free space: +20 dBm + ~1 dBi - 3 dB (linear to circular) + ~1.6 dBi - 53.8
dB = about -34 dBm at the pin.

- The pair sweep on the carrier (console Q) measured **~1.0 dB a row** over
  G19-G31 (11.6 dB for 12 rows), as on noise at G72-G83 and the earlier
  G29-G39 sweep. The RF-stage/BB/fine model has steps of -3.3 and +2.7 dB
  there and is 4.2 dB short at G24.
- Corrected for it, the carrier read -38.7 -> about -34.5 dBm: **the
  antenna-to-pin path delivers what free space predicts (within ~1 dB)**.
  An earlier -46.5 dBm at "2 m" (G37, model error ~0.7 dB) was the
  placement indoors: 8 dB between two placements is ordinary multipath.
- Below the noise-measured rows the gain table now steps 1.0 dB a row
  (gain.c dg_apply_measured_gain), for the dBm meter and the KEEL-1 home
  rows.


## Single program-image LUT load (2026-10-20, PR #231)

The restart now prepares the selected LUT in an IDF V1 program image before
quiescing the pipeline. Its LUT storage replaces the previous work LUT; the
header/instruction prefix adds 300 bytes. The SDK writes the target once;
full instruction and LUT readback still gate publication. A mismatch enters
the existing bounded rollback, whose image restores the verified instructions
and transformed LUT (including output scaling and flywheel flag format).

Board COM10, VTX off, build from 270553e plus this change: four manual LP
switches took 3437, 3614, 3776 and 3238 us. Two kp switches took 4712 and
5327 us including target preparation and scheduling. Recorded LUT verify
stages were about 1195-1304 us versus 2463 us in the pre-change status;
these are small bench samples, not a controlled RF or visible-picture A/B.
All six switches left instruction/LUT verification, restart failures,
TX empty, RX overflow, DMA fault and BS overload counters at zero.
Gain returned to AUTO/G83; original kp_auto=2 restored after manual tests.
The counter kp_writes now includes all 1024 SDK table writes per successful
program load, rather than only subsequent corrective writes.

Validation: host C suite, runtime diagnostic suite (including actual
format/publication/rollback branches), research CI checks, ESP-IDF v6.0.2
firmware build and linked IRAM check passed. Read-only verification tests
inject corruption at the first and last entries, checking no hidden repair.
The format harness checks the selected image prefix on forward and rollback
paths for every supported kp/output/program combination. Physical restart
failure injection and an RF/goggle walk remain unmeasured.


## Detector comparison and range work plan (2026-10-20, PR #231)

The next work is recorded in [PR231_RANGE_PLAN.md](PR231_RANGE_PLAN.md),
including the operator's -100 dBm ambition and flywheel/restart hypothesis.
`research/pll_compare.py --physical --workers 4 --output
research/data/pll_compare_physical.json` compares identical input bytes
across bit-exact kp 0.85/0.60/0.50, with/without LP, a discriminator and
float second-order PLLs. PAL/NTSC, two seeds each, deviation 1.0, 12 MHz
model filter, fixed physical-ADC gain 2.0 shared by noisy/clean cases.
The same seed preserves raster/noise realization across C/N points.

At model C/N 0 dB, mean correctly retained line fractions are 0.38 for
U85Q-LP, 0.98 for PLL bandwidth parameter 0.04, and 0.96 with its LP20
postfilter. Luma SINAD is respectively -0.7, -3.4, -1.5 dB. At 2 dB C/N,
U85Q-LP and PLL+LP20 give 2.0 and 1.8 dB SINAD, with line fractions 0.94
and 1.00. There is useful sync information beyond U85Q, but no demonstrated
10 dB picture improvement. Retaining lines is not complete picture quality.

Limitations: synthetic input; one filter/deviation/gain; clean-reference
alignment/level calibration; float PLL has neither six-bit DAC quantization
nor a proven BitScrambler implementation; LP20 decimates the PLL output
before its IIR. A finite candidate comparison is not an ideal detector or
an information-theoretic upper bound. Existing sensitivity JSON records
explicitly assume NF (5 dB); previous claims of a measured ~6 dB hardware NF
or an independently proved lossless RF input path were too strong.


## Session 55 origin-cloud gain park (2026-10-20, PR #231)

The initial hypothesis of a refused hardware write was not established.
A VTX-off board check on the pre-fix current firmware returned G24 -> AUTO
successfully to G83. A constant-code host probe also recovered, but did not
reproduce the recorded signal.

Replaying snapshot `snap_s0055_00130` (134930 ms, G21, 8128 bytes) through
actual `dg_measure_gain` yields P50=1, origin 863-980 per mille, coherence
0-7 and no clipping, but 28 of 31 256-byte windows have P95=43. Sparse
outliers, primarily I=6 with Q=0, keep peak memory above the release gate.
The existing `no_carrier` classification is true but did not influence
release. Repeating these recorded windows at the controller's 200 us test
cadence parks the old core at G21 indefinitely, with no refused write.
This repetition is a control regression, not a physical gain-response model.

The change uses P50 for release peak memory only with P50<=1,
origin>=850/1000, repeat>=300/1000, coherence<20 and zero clipping. It
retains normal 16 ms peak memory, home-row steps, settle and write pacing;
all overload decisions still use original observations. Off-origin fixed DC
is excluded, unlike the reverted unconditional fixed-point -> max change.
On the 64 available bb10dec snapshots (1984 windows), 55 high-P95 qualifying
windows occur, all in the two suspect session55 captures (125 and 130).
Other qualifying near-origin windows already had low P95.

The real capture and its metadata are preserved in test/data. Added
regressions fail on the prior gain.c and pass with the fix: recovery,
retained 16 ms hold, no gain pumping on a 5 ms fade, no boost on an
off-origin DC point, and raw overload handling. The full host suite,
ESP-IDF build and linked IRAM check pass. Weak-VTX boot and physical RF
recovery are not yet verified; the source of the sparse code outliers is
not established by these recordings.


## Restart/dropout association in existing blackboxes (2026-10-20)

`research/restart_dropout_audit.py '../bb*/session_*.json' --output
research/data/restart_dropout_audit.json` merges overlapping saved exports
by session and exact event identity, checks boot provenance, and records
input hashes. 56 sessions contain 368 kp/program events and 54 dropouts.
A restart precedes 16 dropout events within 1 s, 27 within 2 s, and 33
within 5 s. None precedes within 100 ms. This does NOT exonerate brief
restarts: diag.c emits dropout events from per-second observations, not
physical sync edges. Conversely the associations do not prove causation;
weak input itself can cause both a policy switch and loss of video.

55 sessions have boot flywheel=1; one lacks boot metadata. Boot settings
are not continuous intervention telemetry. These logs cannot identify an
unbiased flywheel on/off sensitivity difference, and circular exports may
omit events. The output retains session firmware identity and missing-data
information rather than treating all historical revisions as current.

Code inspection: normal kp/LP changes use the shared verified restart;
`transport_sync_restart_begin/finish` preserves the flywheel clock lease
across flagged restarts but invalidates native source evidence. A stale or
unreadable source does not authorize repair. `transport_sync_live_tick`
changes the demodulator format when entering/leaving flag mode, not on
every ordinary loss of sync. This does not rule out CPU contention, wrong
repair timing or goggle recovery losses. Existing runtime ownership tests
pass; direct RF range attribution is still open.


## Uncalibrated envelope C/N is no longer actuator evidence (2026-10-21)

Issue #222's remaining control-path audit found one real firmware path that
could manufacture a dB number. C/N-AUTO normally compares IQ power with the
thermal-noise map for the exact channel/filter/gain context. When that map
was absent, `receiver_obs_cn_x10()` bypassed C/N-AUTO's source/confidence and
returned the older envelope estimate directly to AFC, profile matching and
the kp/LP ladder.

That fallback is not input-referred C/N. For the issue's deliberately simple
noiseless two-level power sequence (13 and 61), mean-squared/variance is 238;
the 12 MHz table calls it about 3.1 dB C/N. That is below the normal-profile
LP entry threshold, despite no receiver noise being added. Separately, the
2026-10-20 walk already showed 5-9 dB jumps when control switched between
power and envelope estimators. The counterexample proves ambiguity and the
walk proves the scale discontinuity; neither alone measures a range loss.

`cna_control_x10()` now admits only `CNA_SRC_POWER` backed by a thermal-noise
reference. Envelope and M2M4 estimates remain in `CN_AUTO` telemetry as
`estimate_x10`, while `control_x10` is unknown without the reference. Unknown
holds the C/N-driven kp/LP policy, refuses an AFC correction and prevents
profile narrowing; it is never converted to a low-C/N rescue request. The
host regression checks the false LP request and the provenance gate.

Review found two cache paths behind that boundary. The kp/LP ladder now resets
and rebinds to the applied program on every channel/filter/offset/RF-table/PHY
context change, not only on a profile-name change. It also treats a current
UNKNOWN as an immediate kp hold and cancels pending C/N-driven steps; cached
C/N can no longer finish a decision. AUTO gain remains independent evidence
for the optional gain-driven LP policy.

Evidence boundary: this prevents an uncalibrated control-induced loss; it is
not a measured dBm or range gain. In an already calibrated context the active
power path is unchanged. A VTX-on first boot without a stored noise map now
keeps conservative defaults until a quiet/no-carrier measurement exists;
that boot case and the number of suppressed decisions need board observation.


## Output telemetry V1 for C5MAP evidence (2026-10-22)

Issue #233 cannot be answered from the historical `VIDEO_Q` field alone. That
field is evidence from the AFC video observer and remains useful, but it does
not identify the verified program driving the output or distinguish native
sync from repaired tips. V1 therefore appends a task-side output-observer
rollup to each flight-log second: exact verified U85Q/U85Q-LP program family
and output level, observer coverage, native H/V trust, native H lines,
repaired/suppressed tips, the longest continuously observed native-H outage,
and observer gaps. A fresh observed outage lasting 100 ms also records an
event and requests a high-priority IQ snapshot.

The real-time boundary is unchanged. Existing ISR/RMT counters are sampled by
the 2-ms control task; no logging, allocation or new calculation was added to
the line-rate path. Stale/missing observer data is recorded as `UNKNOWN` and
breaks, rather than extends, an unlock interval. Old 34/38/42/50-byte records
remain readable and decode the new fields as unknown.

Evidence boundary: these are receiver output-observer and repair metrics, not
a claim that any particular monitor or goggles stayed locked, and not a SINAD
measurement. Host format/rollup/browser/Python tests validate semantics and
compatibility. Board CPU margin, real RF correlation, monitor lock and a
reference SINAD path remain required follow-up work before C5MAP can optimize
on these observations.

PR #241 review follow-up: a busy PHY skips observer servicing, so cached
freshness cannot qualify that tick. Busy/fault ticks now explicitly mark the
observation unavailable, break unlock continuity and clear the summary's
current observer-validity flags while retaining cumulative-counter deltas.
Host regressions cover cached locked/unlocked states and resumption without
recounting counters. This does not change the output guard or ISR behavior.


## PR #254: retained PHY research (2026-10-25)

[The knowledge-only extraction](PR254_PHY_FINDINGS.md) preserves the 11p
noise-bandwidth measurements, negative PHY tests, corrected live-G83/DC
interpretation, recovered helper semantics and measurement limits, with frozen
source links. It imports no experimental firmware or lab tooling and changes
no production defaults. Noise-only variance changes are not RF sensitivity
or range measurements.

## Faster verified startup calibration (2026-10-25, hardware pending)

Extends C5VRX by Twotoz and contributors' startup calibration, neighbouring-row
repair and exact filter-context work. DC now uses short search probes with a
full final measurement and exhaustive fallback; the filter grid screens every
setting and fully measures contenders, with full-grid fallback and independent
winner rechecks. A three-position phase fast path requires repeated confirmation;
ambiguous evidence retains the nine-position scan. Fresh exact-context DC noise
statistics can serve the startup gain map. Absent-VTX grace is 2 s instead of
20 s; deviation sample/quality requirements remain unchanged.

See [FASTER_CALIBRATION.md](FASTER_CALIBRATION.md) for bounds, tests and board
comparison. This is host-tested search/control work, not a measured speedup,
sensitivity gain or proof of unchanged filter selection on all C5 boards.

## PR #272 policy update: baseline boot, manual calibration (2026-10-26)

Operator requires a <=10-second boot target and calibration through the menu.
The branch loads a fixed filter baseline/native or exact saved DC, removes
RF/DC sweeps and automatic DC servo/adoption, and adds passive qualified health
recommendations. Reset-dependent sampling alignment alone gets a bounded 600 ms
opportunity under the existing splash. Menu F handles one filter; explicit B
runs full optimization. Read-only C/web JSON export supports multi-board work.

No measured population baseline has been invented; the gain-map preset still
has zero contributing boards. Mock-PHY execution of actual main/PR DC searches
shows 2.054x/1.407x/1.615x less scheduled waiting in three cases with equal final
residuals. End-to-end board timing, residual distributions, standard-preset
bandwidth and RF/video performance remain to be measured. This supersedes the
initial PR #272 startup-policy note; docs/FASTER_CALIBRATION.md is current.

## Wrong sampling phase after boot, live DC servo and base map (2026-10-26)

Board b26ef545, A1 5865 MHz, VTX off, PR #272. Supersedes the "no automatic
DC servo" part of the policy note above (operator decision the same day).

- **Wrong sampling phase is a blind receiver.** Console `S` (one RX slip)
  over 12 positions: 3 read mixed IQ bits, excess_mc2 4588-6597 against
  44-62, with the IQ 989-998 per mille clipped. One boot in five stayed on
  such a position after the splash (`SPHASE unverified excess_mc2=1883..3149`);
  ten OUTPUT restarts never moved a settled phase. Nothing repaired a phase
  found wrong after the boot check: `phase_service()` now runs the confirmed
  scan live. Main has the same exposure; the host model puts mixed bits at
  1.4-5 dB, the board shows near-total clipping. Range not yet measured.
- **Live DC servo restored, gated on the phase.** On the four settled boots:
  0.26-0.37 cells mean from starts up to 1.65 cells, learned slope 200-420
  mcells/code. On the wrong-phase boot (before the gate) it walked I to its
  -96 limit. Servo state is 36 rows in LP RAM: 5.2 KB more static HP RAM left
  the USB console silent from boot (heap ~34 KB free after boot).
- **DC batch stop at low gain.** F stopped at G68-G73 as carrier_detected on
  quantized receiver noise (envelope 1.22); only a definite carrier stops it
  now. 30/30 rows, code spread 0.8 per row within a session; between boot
  sessions high-gain Q moved 10-15 codes (~1-2 cells), which the servo trims.
- **Base map** `main/dco_preset.h` from nine F runs of one board, filled as
  template rows in task context. Looking it up in the gain hook (which also
  runs in the vendor RF restore) left the console silent from boot.

## AGC research: LEVEL, an input-level AGC, and an RSSI baseline (2026-10-25, PR #266)

**Question.** KEEL-1 grew in layers (V3 core, V5, V6 carrier memory, V7
peak/pacing, home rows, heat, anchors, fade hold). Is a model built from
what we now know better?

**Premise from earlier findings.** The receiver noise is set before the
baseband gain ("Direct Gain V7"): a row scales carrier and noise alike, so
the gain does not change C/N. What costs picture is a fold of the ultrafine
lane, each write (~2 ms), stale per-row DC after a row change, and moments
with the gain far too low.

**LEVEL** (`main/agc.c`). The carrier level referred to the input is then
row-independent. LEVEL keeps a ~20 ms decaying histogram of it (2 dB bins)
and scores every KEEL-1 home row over that history: sync and picture SNR
from C/N, 4-bit quantisation and the detector's small-signal loss (bit-exact
U85Q, PR #230), or a fold. It moves when another row is better by a
hysteresis, at once on a fold. Integer per window; the per-row quality table
is built at reset. It learns the gain-model error of the row it moves to
from the input level just before and after the move. **RSSI** (`main/agc.c`):
a classic mean-power AGC (target radius ~3.5 cells, +-3 dB dead band, 6 dB
drop on a fold), as a simple baseline.

**Host comparison** (`test/test_agc_compare.c`: the test_keel_track.c
receiver, DC and fading model, 24 scenarios, 3 seeds; usable %, edge
weighted twice):

| model | score | writes/s | edge 3 dB C/N |
| --- | ---: | ---: | ---: |
| KEEL-1 | 61.6 | 4.5 | 28.7 % |
| LEVEL | 63.6 | 5.5 | 34.0 % |
| RSSI | 61.1 | 48.4 | 30.6 % |
| best static home row (a posteriori) | 60.7 | 0 | 35.0 % |
| ideal per-tick oracle (bound) | 74.1 | 0 | 41.8 % |

- At the edge KEEL-1 averages G77-78 and loses ~6 points against simply
  holding G79 (35.0 %); the fast-drone edge case 25.6 % against 34.9 %.
  LEVEL closes most of that.
- Close in with slow fades (walking, 2-5 Hz) following the fades pays:
  KEEL-1 95.7 % against 79 % for a static row. LEVEL needed a short memory
  (~20 ms) to match it; a 0.6 s memory scored 57.8.
- Fast fades (drone, 540-1070 Hz): LEVEL rides one row (87.4 % against
  KEEL-1 78.0 % mid range).
- 4 dB bins cost 2.5 points; 1 and 2 dB bins score alike.

**Robustness** (same draws):

| perturbation | KEEL-1 | LEVEL | RSSI |
| --- | ---: | ---: | ---: |
| fold rule 0.8 / 1.3 noise radii | 61.6 / 61.6 | 63.9 / 63.6 | 61.1 / 61.0 |
| twice the DC | 60.9 | 63.4 | 60.5 |
| DC servo 1 s | 56.9 | 60.7 | 55.8 |
| write costs 5 ms | 60.9 | 62.6 | 54.3 |
| gain model +-1 dB (top 24 dB), +-6 dB below | 60.6 | 61.7 | 57.4 |
| gain model +-1 dB, +-10 dB below | 59.5 | 60.3 | 56.5 |
| gain model +-3 dB on every row | 57.7 | 53.9 (edge 8.7 %) | 46.4 |

The learned row correction halves LEVEL's writes with a poor low-row model
(11.9 -> 6.8 and 17 -> 9.7 a second) and keeps the edge at 33.4 % (31.0 %
without). **Limit:** LEVEL needs the measured noise map of the top rows
(`gtab`, startup); with +-3 dB errors there, it subtracts the wrong noise at
the edge and loses to KEEL-1, which measures the envelope directly.

**Firmware.** Menu GAIN row and console `G` cycle AUTO (KEEL-1, default) ->
LEVEL -> RSSI -> MAX -> fixed; not stored, every boot starts AUTO. Console
`T` prints `AGC model=... moves folds learned`. +2.3 KB static RAM (one
model at a time, union). **Evidence:** host simulation and firmware build
only; the harness shares its picture model with LEVEL's quality function
(the physics from PR #230), so a board A/B is the deciding test. No range
or picture claim.
