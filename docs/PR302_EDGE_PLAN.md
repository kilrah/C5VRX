# PR #302 edge plan: a gain change without a disturbance

C5VRX by Twotoz and the C5VRX contributors. Work plan for the range that
PR #302 does not touch: the 4-7 dB between the static threshold (-92 dBm,
2026-10-20) and the walking edge (-85..-88 dBm). Evidence and tables are in
[FINDINGS.md](FINDINGS.md), "Edge window" and "LEVEL walks"; the scripts are
`research/edge_window.py` and `research/output_tip_headroom.py`. This extends
the Direct Gain and DC hold work (`main/rf.c`, `main/phy.c`), the C5VRX-3/4
lane switch (`legacy/c5vrx4/LANE_HANDOVER.md`) and IQLaneFusion.

**Status: a plan. Nothing below is built, flashed or measured.** Each step
names what it needs from the board and what decides it.

## Why the gain is the place

- Video is 16 points lower in the seconds the gain moves (89.6 against 73.8 %
  strong, 60.2 against 51.5 % at the edge), far more than the 1-5 writes a
  second explain by themselves.
- On the top row the ultrafine lane folds from about 8 dB C/N, so the gain
  has to move exactly in the edge zone, where fading crosses that level all
  the time; every move lands on another DC row.
- A folded window reads small, so the model steps down one home row per fold
  (G83 -> G79 -> G75 -> G71 -> G68 in 100 ms on walk 286).

Three causes, three steps: make the write itself clean (1-2), need fewer
writes (3-4), and stop reading a fold as a weak carrier (4).

## Step 1: see one gain write (bench, read only)

No raw IQ across a gain write exists. Without it nobody knows whether the
cost is the 2 us of vendor gain 50, the interval on the chip's own DC, the
analog settling or the observer's hold-off.

- Console command (bench only, receiver paused by its owner tag): write the
  neighbouring row and back, freeze the raw ring right after each write, dump
  it framed and checksummed like the existing IQ snapshot.
- In the same command, per gain row: `phy_pbus_rd` of the owned blocks
  (0, 1, 2, 3, 8; both banks) after the vendor write, printed as one line a
  row. This is the register diff the direct write needs.
- Host tool: time from the write to radius and DC back within 0.3 cells, per
  row pair (same RF stage, across an RF stage).

Needs: board on USB, VTX on a still carrier at about -60 dBm and once off.
Decides: whether step 2 can gain anything, and which words differ per row.

## Step 2: a gain write that stays in debug mode

Today (`rf_set_gain`): release the DC hold (`phy_pbus_workmode`: vendor gain
50 for ~2 us, then the old gain), `phy_force_rx_gain` (the chip's own DC of
the new row), then the post-gain hook enters debug mode again and holds the
calibrated DC. Four analog states for one change.

Design, without guessing a register mapping:

- **Learn the row once through the vendor path.** The hold already reads the
  live words of the owned blocks before it enters debug mode (`read_live`,
  `pin_bank1`). Keep them per gain row (5 blocks x 2 banks, 20 bytes a row,
  in LP RAM like the other CPU-only state).
- **Later visits write those words directly.** Still in debug mode:
  `phy_pbus_force_test` of the row's cached gain words, then its DC pair.
  The end state is word for word what the vendor path produced on the first
  visit; only the path between the two rows changes.
- **Verify, else fall back.** Read every written word back. A mismatch, a row
  without a cache entry, or a cache from another context takes today's vendor
  path, counted in `T`.
- **Invalidate** the cache on every `restore_lock()`, table capture, bank 1
  pin change and experimental PHY patch.
- **Keep the vendor's gain index true.** `pbus_workmode_keep_gain()` re-forces
  the index it reads from 0x600A702C; after a direct write that is the old
  row. The release must force the row the firmware tracks instead.
- Order of the words: going down, gain words before DC; going up, DC first.
  Step 1's captures decide whether that matters.

EXPERIMENTAL item `GAIN WRITE: VENDOR / DIRECT`, default VENDOR, logged per
second like the other items. Host tests: cache validity across retune and
filter change, fallback on readback mismatch, the tracked index after a
release.

Needs: step 1's register lines (to confirm nothing outside the owned blocks
changes with the row), then a walk A/B on the menu item.
Decides: video % in the seconds the gain moves, DIRECT against VENDOR.

## Step 3: fewer writes at the edge (exists, needs the walk)

GAIN CAP G79 / G75 on the ultrafine lane gives the same noise-to-fold room
by analog gain that the fine lane gives on the top row, with the DC servo,
phase check and C/N reference all running. `research/edge_window.py` puts the
cost at 0.2-0.4 dB SINAD from 2 dB C/N and the last dB of lock at 0 dB.

Needs: one walk per setting, same VTX and route.
Decides: whether the edge wants room (cap) or the last dB (top row). If the
cap wins, LEVEL's survival row moves there and the top row is only used with
no carrier at all.

## Step 4: lanes as the fast gain step

One analog gain decodes 2-24 dB C/N across the three lanes with the unchanged
decoder table (its token plane is an angle). A lane change is a GPIO-matrix
route: no analog step, no other DC row, no restart.

In order, each gated by the one before:

1. **Route change on a carrier** (bench): switch ultrafine <-> fine <-> coarse
   on a still carrier with the raw ring frozen across it. Count the pairs
   with mixed bits and look at the decoded video across the change. The
   C5VRX-4 router changed only the six magnitude selectors in one critical
   section; start from that.
2. **Lane-aware support.** Today everything is in ultrafine cells: noise maps
   (`cnmap`, `gtab`), C/N-AUTO, RX_LEVEL, the DC servo's thresholds, fold
   detection, the gain map context. Scale by the lane's step (cells^2 x 4 a
   lane) at the observer, so stored maps stay in one unit, and stop treating
   a standard lane as a foreign transfer (`experimental_transfer_active`).
   Host tests per subsystem; a boot on FINE must then equal ULTRAFINE on the
   bench noise numbers.
3. **Coarse as the unfolded view.** The coarse lane never folds. On a fold
   suspicion go to coarse first, read the real radius there, then choose lane
   and row in one decision instead of one home row per fold. This replaces
   the fold-radius estimate, which had to guess what coarse simply shows.
4. **Lane first, analog second.** Within the lanes' reach change the lane;
   write the analog gain only to move the whole window (a few anchor rows
   ~18 dB apart). The DC tables then need those few rows, measured on the
   own channel, instead of thirty with templates.

Needs: 1 on the board before any of 2-4 is worth building.
Decides: step 1's capture (is a route change invisible in the video), then a
walk of lane-first against LEVEL.

## Step 5: DC

- With few anchor rows (step 4) every row can be an own, converged row; the
  template and base-map rows that cost the edge on 2026-10-27 go away.
- The same analog DC is half as many cells on the fine lane and a quarter on
  coarse, so the servo's 0.3 cells is easier to hold where the carrier is
  strong enough for those lanes.
- A token plane computed around the DC point recovers most of a 2-cell DC
  from 9 dB C/N up (`research/edge_window.py`). It costs a table restart per
  change: only for the top row with no servo authority left, as a last
  resort, never per gain write.

## Not planned

- A second-order loop or another decoder search: the static threshold is
  already at the model's (FINDINGS, "Range audit: static edge").
- Using sample B of each pair: the noise 25 ns apart is ~0.84 correlated in a
  12.9 MHz noise bandwidth, under 0.5 dB to gain.
- The noise figure: not reachable from firmware (PHY_NOISE_RESULTS).
- Any CPU loop that holds or pins the gain around the write: the write
  itself is what steps 1-2 repair.
