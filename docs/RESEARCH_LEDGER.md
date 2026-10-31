# C5VRX research ledger (C5VRX-1 to C5VRX-5)

C5VRX by Twotoz and the C5VRX contributors. This is the project's research
provenance ledger, moved verbatim from `AGENTS.md` on 2026-10-28 so that the
agent instructions there carry only the current C5VRX-5 rules. The
attribution and responsible-reuse rules in [`AGENTS.md`](../AGENTS.md) apply
to everything recorded here. Entries keep their original dates and evidence
boundaries; where they disagree with the current rules in `AGENTS.md` or with
`docs/FINDINGS.md`, those win. Paths are relative to the repository root.

## C5VRX-1 through C5VRX-4: research provenance and pipeline ledger

**Twotoz has investigated these C5VRX pipeline methods,
data sources, CPU/hardware processing routes and receiver experiments.**
This is the project's accumulated research programme, with contributions and
assistance recorded in the source/history. Preserve that provenance when using
its findings; do not describe an already investigated route as a new independent
discovery. This credit does not assert sole authorship of every implementation,
ownership of Espressif technology, or a new license restriction. The attribution
rules in `AGENTS.md` still apply.

Ledger recorded on 2026-10-01 against main `8ed1045d74ee6921052bc2fd32fb77022a9ae0f6`
and the explicit C5VRX-4 branch revisions below. Names identify generations and
experiments, not four interchangeable production builds. **Investigated** does
not mean **implemented**, **host-proven**, **hardware-tested**, or **range-proven**.
Older documents retain their original evidence boundaries; later measurements
can supersede them. In particular, an early C5VRX-1 "no continuous source proven"
verdict predates the C5VRX-2 autonomous-writer and MODEM_DIAG hardware findings.

### Data sources and acquisition methods already investigated

| Source / method | What it supplies or attempts | Evidence and limitation | Start here |
| --- | --- | --- | --- |
| Wi-Fi promiscuous RX / certification RX | Packets, metadata, RSSI, receive/error counts | Not a continuous phase-bearing analog waveform source | `legacy/c5vrx1/research/continuous-rf-verdict.md` |
| Wi-Fi CSI / FFT / subcarrier views | Packet-training channel estimates or diagnostic PHY information | CSI is not continuous time-domain ADC IQ. FFT-scale and phase-tap probes do not establish a new live source | Same verdict; `legacy/c5vrx3/main/phy_phase_tap_probe.c`, `legacy/c5vrx3/tools/phy_phase_tap_probe.md`, `legacy/c5vrx3/docs/pre-q4-lab.md` |
| SAR ADC continuous / DMA | Samples from documented ADC GPIO channels | This SAR ADC is not the Wi-Fi RF ADC; no direct public RF-to-GDMA endpoint was found | `legacy/c5vrx1/research/continuous-rf-verdict.md` |
| Vendor `adctrig` finite RF/FE dump | Packed signed Q10/I10 in a 16,384-word, 64-KiB window at `0x40830000` | Finite wrapper stops capture before returning; repeated captures are not gapless RF | `legacy/c5vrx1/research/adc-dump-format.md`, `legacy/c5vrx1/research/rf-dump-producer.md` |
| Dump source mux / mode 0 / mode 11 / mode 12 | Recovered source/trigger/debug selector states | Mode 0 is software-triggered; mode 11 has no established rate advantage; mode 12 starts BLE RX and is rejected for 5.8 GHz. FE/BB labels and tap/filter ordering must retain their uncertainty | `legacy/c5vrx1/research/rf-dump-source-mux.md`, `legacy/c5vrx1/research/rf-iq-dump-verdict.md` |
| Autonomous dump-first / TX_START RF writer | One-start circular RF writer, roughly 80 MS/s in the measured setup | Writer wraps without rearm are hardware-proven; this is not proof of a simultaneously readable CPU/DMA IQ source | `legacy/c5vrx3/docs/continuous-iq-findings.md` |
| HP-CPU reads / guarded immutable ring copies | Lagged SRAM windows with pointer, wrap, deadline and ownership checks | Implemented historical reader; active MAC-owned SRAM later returned stale/non-live views. Address eligibility alone does not solve access | `legacy/c5vrx1/research/live-rx-pipeline.md`, `legacy/c5vrx1/research/rf-ring-memory-path.md`, `legacy/c5vrx3/docs/continuous-iq-findings.md` |
| Direct AHB-GDMA / zero-copy dump-ring reads | Avoid CPU copies from the physical RF bank | Both banks, CPU-owned variant and APM exception checks were investigated; successful DMA completion still returned static views while RF wrote | `legacy/c5vrx3/docs/continuous-iq-findings.md` |
| MODEM_DIAG via GPIO CPU snapshots | RF-dependent diagnostic bits compared with stopped Q10/I10 captures | Useful bounded source-mapping proof; asynchronous CPU polling is not sustainable source-synchronous capture | `legacy/c5vrx3/docs/continuous-iq-findings.md`, `legacy/c5vrx3/tools/phy_phase_tap_probe.md` |
| MODEM_DIAG via PARLIO RX | Eight simultaneous routed bits; normal packed Q4/I4 at 40 MS/s | Bounded bit-exact capture and live NTSC demonstrated. The approximately 80-MS/s modem bus and 40-MS/s acquired stream are different rates | `legacy/c5vrx3/docs/continuous-iq-findings.md`, `legacy/c5vrx3/main/rf.c`, `legacy/c5vrx3/main/video.c` |
| Coarse / fine / ultrafine sign-preserving IQ slices | Per component ADC bits `{9,8,7,6}`, `{9,7,6,5}`, `{9,6,5,4}` | Still four captured bits per component, with finer steps and smaller unfurled windows. Folding is not ordinary amplitude clipping; finer lanes are not added RF gain | `legacy/c5vrx3/main/rf.c`, `legacy/c5vrx3/main/direct_gain.c`, `legacy/c5vrx3/docs/range-max.md` |
| Wider I5/Q5 or I6/Q6 and alternate phase taps | More simultaneous IQ information, source-side preprocessing or an already decoded phase bus | C5 PARLIO RX has eight data lines. A 16-bit setting cannot supply 10/12 lanes. Bit correlation does not prove simultaneous wider capture or a ready-made phase bus | `legacy/c5vrx3/tools/phy_phase_tap_probe.md`, `legacy/c5vrx4/RESEARCH.md` |
| Synthetic IQ, replay, frozen captures and loopback | Independent DSP/DAC/reference/oracle stimuli | Essential algorithm and transport diagnostics; they do not demonstrate live RF continuity or RF sensitivity | Legacy host tools, `legacy/c5vrx3/docs/pr-derived-findings.md`, `legacy/c5vrx3/tools/validate_build.py` |

### C5VRX-1: CPU, finite-capture and guarded-ring architectures

Evidence lives in `legacy/c5vrx1`; it is an archived implementation, not root
production firmware.

- **CPU/host reference DSP:** complex-conjugate adjacent FM, phase/LUT
  approximations, filtering, DC/gain/polarity conditioning, decimation and
  scanline reconstruction. The `240 MHz / sample-rate` tables compare all-sample
  and sparse `/2`, `/4`, `/8` CPU reads; they are feasibility budgets, not measured
  throughput. See `research/dsp-pipeline.md` and `research/realtime-feasibility.md`
  under that archive.
- **Bounded dump kernel in LP SRAM:** `legacy/c5vrx1/main/c5vrx_lp_capture.S` moves execution
  and its stack into LP RAM during HP-SRAM ownership handoff. This is an HP-CPU
  capture routine placed in LP memory, not proof of a separate LP-core live DSP
  engine. Capture stops and ownership is restored before normal processing.
- **Finite / NEARLIVE chain:** acquired immutable dump blocks feed hardware WBFM,
  conditioning and PARLIO/DAC output. This can exercise the downstream chain but
  cannot turn finite vendor dumps into continuous reception.
- **Guarded streaming candidate:** RF writer -> guarded short CPU copy -> owned
  queue -> persistent BitScrambler 4:1 phase discriminator -> CPU conditioner /
  fixed-size chunker -> two-buffer PARLIO/GDMA -> six-bit passive DAC. The sparse
  4:1 detector keeps every fourth input and has no pre-decimation anti-alias
  filter; its source/bandwidth/contention gates remained explicit.
- **Zero-copy ring-to-BitScrambler candidate:** reusable M2M DMA descriptors and
  synchronous lag checks instead of immutable copies. Historically deferred
  pending copy-occupancy evidence; later active-SRAM visibility findings block
  treating it as an available live source.
- **CVBS output / USB preview:** synthetic monochrome/colour/OSD output, recovered
  CVBS conditioning, sync tracking, sample-domain overlays and optional bounded
  grayscale USB preview. The preview is a side consumer; it must not own AV
  timing. A synthetic raster or replay is not recovered live PAL/NTSC.

Read `legacy/c5vrx1/research/live-stream-architecture.md`,
`legacy/c5vrx1/research/live-rx-pipeline.md`,
`legacy/c5vrx1/research/rf-ring-memory-path.md` and
`legacy/c5vrx1/research/video-output.md` before retrying these topologies.

### C5VRX-2: autonomous writer, diagnostic bus and TX hardware demodulation

Evidence lives in `legacy/c5vrx2` and the source/quality findings in `docs`.

- **Finite rearm / REGDMA control research:** `legacy/c5vrx2/main/regdma_rearm.c`
  configures an LP-SRAM-resident register-write chain. Register rearming is a
  control mechanism, not a direct RF-sample DMA endpoint or proof of gapless
  capture. The later proven dump-first live source does not periodically rearm.
- **Dump-SRAM/direct WBFM attempts:** producer, physical-wrap phase and GDMA
  visibility diagnostics investigated a high-resolution SRAM-fed hardware path;
  normal active-SRAM reading was retired after the stale-view hardware result.
- **RX-attached BitScrambler adjacent FM:** Q4 capture, every-sample phase
  discrimination and real-domain 2:1 reduction were investigated. The original
  program did not sustain the required input cadence; distinguish this design
  from the eventual live TX decorator.
- **Proven TX decorator:** MODEM_DIAG -> PARLIO RX40 -> raw 16-KiB cyclic ring ->
  persistent TX BitScrambler -> PARLIO TX -> resistor DAC. First compact IQ5 used
  asymmetric Q3/I2 state and a 32x32 endpoint LUT. Full Q4/I4 -> uniform Phase5
  then improved phase precision, colour and static. The early TX20 topology and
  later duplicated-code TX40 geometry are separate historical configurations.
- **Other investigated kernels/output geometries:** fast/compact and three-LUT
  candidates; Phase5 50-ns and 100-ns endpoint spans; True40 adjacent25;
  middle-sample trajectory; four-point Linear80 reconstruction; RX edge changes;
  bounded raw/live/DAC captures and TX80 throughput/address oracles. These have
  different quantization, response and throughput limits, not automatic quality
  gains from a higher sample rate.

Start with `legacy/c5vrx2/main/Kconfig.projbuild`, its `legacy/c5vrx2/main/*.bsasm`,
`legacy/c5vrx3/docs/continuous-iq-findings.md`, `legacy/c5vrx3/docs/issue-17-true40-cadence-and-interleaved-phase5.md`
and `legacy/c5vrx3/docs/monotone-linear40-and-reconstruction.md`.

### Public evidence: continuous IQ/video pipeline and Espressif EOF issue

Preserve the attribution and exact scope of
[espressif/esp-idf#19091](https://github.com/espressif/esp-idf/issues/19091)
when describing or adapting C5VRX's continuous-stream work:

- **16 September 2026:** Twotoz reported that steady-state
  eight-bit PARLIO TX / GDMA loops with BitScrambler emitted a real EOF boundary
  on every ring wrap. A live-hardware A/B test cleared cyclic `suc_eof` and
  eliminated the observed wrap-related video artifacts.
- **17 September 2026:** Espressif's maintainer
  [confirmed “Your analysis is correct.” and supplied a driver patch](https://github.com/espressif/esp-idf/issues/19091#issuecomment-5712811733).
  EOF was used for AHB-GDMA buffer-switch notifications, but BitScrambler also
  interpreted it as a stream boundary. The general driver solution suppresses
  EOF in steady state and emits it once when switching buffers; do not remove
  required switch notifications indiscriminately.
- Credit C5VRX by Twotoz and the contributors for the investigation, live
  workaround and application evidence; credit Espressif for its explanation and
  supplied driver patch. This is a specific C5VRX-2/3 continuous hardware
  RF-to-video contribution, not a claim of sole ownership of RF dump technology
  or worldwide priority for all continuous ESP32 SDR methods.
- Keep three evidence levels separate: autonomous RF writer (5/6 September),
  first live NTSC chain (9 September), and removal of cyclic output EOF seams
  (issue above). See `legacy/c5vrx3/docs/continuous-iq-findings.md` and
  `legacy/c5vrx3/docs/pr-derived-findings.md`. Output EOF suppression alone does not prove
  sample-perfect RF acquisition across every private dump-SRAM boundary.

### C5VRX-3 (archived 2026-10-12): hardware dataplane and demodulator investigations

The C5VRX-3 dataplane (`legacy/c5vrx3/main`) was MODEM_DIAG -> PARLIO RX40 -> raw cyclic IQ ring ->
TX BitScrambler -> PARLIO TX40 -> passive DAC. Golden emits unique CVBS20 as
`[D,D]`; Zero-EOF descriptors remove periodic transport bubbles. The CPU runs
control, diagnostics, observers, USB and menu setup, rather than a per-sample
40-MS/s software demodulator. Read actual build flags and the selected program:
checked-in experiments are not automatically the default.

| Investigated method | Processing route / finding | Evidence entry |
| --- | --- | --- |
| Golden / Phase5 endpoint | Full Q4/I4 phase decode and 50-ns endpoint delta; established live visual baseline | `legacy/c5vrx3/main/fm.bsasm`, `legacy/c5vrx3/docs/pr-derived-findings.md` |
| Phase8 HR / full signed delta / HC | TX-only counter-assisted higher-precision phase terms; optional history-conditioned decoder; compile-time/NVS comparisons, not the ordinary Golden default | `legacy/c5vrx3/main/fm_phase8_hr_live.bsasm`, `legacy/c5vrx3/main/fm_hc.bsasm`, `legacy/c5vrx3/main/Kconfig.projbuild`, `legacy/c5vrx3/tools/test_phase8_hr_live.py` |
| Trajectory V1/V2 and model priors | Use middle/history information to infer motion in bounded LUTs; quantizer/training/held-out RF evidence matters | `legacy/c5vrx3/docs/issue-9-trajectory.md`, `legacy/c5vrx3/docs/trajectory-v2.md`, `legacy/c5vrx3/main/fm_traj.bsasm` |
| Exact adjacent FM / Alpha | Raw/phase M2M blocks, both 25-ns wrapped deltas summed without a second wrap; confidence-aware prediction investigated | `legacy/c5vrx3/docs/pr-derived-findings.md` (PRs #53-55, #67) |
| LIFT-FM / Adjacent50 | Exact Phase5-domain factorization into 24 tokens / one shared LUT; raw-to-phase ingress still needs a feasible live route | Same ledger (PRs #64-65, #70, #72) |
| CPU/raw-pair bridge and single-engine M2M | CPU/DMA handoff and sequential raw->Phase5->LIFT avoid simultaneous RX/TX BS use, but deadlines, finite-block state and artifacts remain independent gates | Same ledger (PRs #67, #72) |
| Dual RX/TX BitScrambler preprocessing | Phase/Polar encoding before TX demod; tested simultaneous use failed/collapsed the ring | Same ledger (PRs #65-66, #69-70) |
| Phase6 / Phase5+ | Higher-precision or no-second-wrap variants; TX-only four-bundle path produced empty FIFO/black video in the tested 50-ns geometry | Same ledger (PRs #66, #68, #70) |
| Polar11 / PolarState8 | Nested phase/residual geometry or TX-only stateful LUT; dual-core route blocked, TX-only prototype showed desync/rainbow artifacts | Same ledger (PRs #69, #71) |
| Relative Golden / Counter-A / Static-A | Unanchored integration drifted; anchoring to Golden DAC levels retained useful live sync/colour | `legacy/c5vrx3/tools/relative_golden_live.md`, `legacy/c5vrx3/main/fm_relative_golden.bsasm`, ledger PRs #73-76 |
| Golden360 / Phase5-360 / middle routing / FSM | Endpoint winding identity, compression/capacity proofs, alternating-middle and capture/routing probes | `legacy/c5vrx3/docs/golden360-feasibility.md`, `legacy/c5vrx3/docs/phase5-360-comprehensive-findings.md`, `legacy/c5vrx3/tools/phase5_fsm_capture_result.md`, `legacy/c5vrx3/main/fm_phase5_360.bsasm` |
| 6BIT@40 / 4BIT@80 / interpolation | Distinguish sample resolution, unique-code rate and physical transport; assembly/host success does not prove FIFO throughput | `legacy/c5vrx3/main/fm4.bsasm`, `legacy/c5vrx3/docs/pr16-rate-followup.md`, `legacy/c5vrx3/docs/pr-derived-findings.md` |
| Menu raster / sync flywheel / semantic repair | Standalone generated menu is a separate TX owner. Live waveform regeneration/repair is separate research, not the normal recovered-CVBS path | `legacy/c5vrx3/docs/c5vrx3-menu-and-raster-architecture.md`, `legacy/c5vrx3/docs/range-max.md` |

Golden360 capacity bounds apply to their exact target and tested factorization,
not every architecture or an unnecessary byte-exact Golden requirement.
`legacy/c5vrx3/main/fm_phase5_360.bsasm` explicitly retains Golden DAC values in its worker
paths; its name alone does not demonstrate live winding correction. Likewise,
older `legacy/c5vrx3/docs/realtime-iq-plan.md` every-adjacent/RX-BS wording describes a design
stage; inspect the later findings and actual TX program before claiming the
middle sample participates in production output.

Control research also belongs to this lineage: fixed/manual gain, adaptive
active/shadow modes, Range/ARC V2-V5, Direct Gain V1-V5, native packet AGC,
paced native hold windows, IQ/DC calibration, finer-lane selection, bandwidth,
AFC/frequency sweeps, PHY patch/timer/FFT probes, fusion/PLL-lite observers and
TX/DAC self-noise comparisons. These are gain/source/control investigations;
they do not constitute CPU replacement of the live hardware demodulator. Read
`legacy/c5vrx3/docs/arc-receive-chain.md`, `legacy/c5vrx3/docs/native-agc-v2.md`, `legacy/c5vrx3/docs/native-agc-paced.md`,
`legacy/c5vrx3/docs/direct-gain-v3-core.md`, `legacy/c5vrx3/docs/fusion-receiver.md` and `legacy/c5vrx3/docs/range-max.md`.

### C5VRX-4: three-bundle span75 and wider-pipeline research

`legacy/c5vrx4` is an isolated build, not the root application. The main
snapshot contains the first Phase6 span75 implementation; later open branches
must be read separately. Keep their gain and lane policies attached to their
own revision.

- **Initial Phase6 span75:** raw IQ40 and existing ring -> three TX bundles ->
  one unique CVBS code per 75 ns, emitted `[D,D,D]` at 40 MHz (13.333 MS/s unique).
  Nominal resistor-DAC inversion LUT; paced native AGC (1 ms / 20 us) in the
  first prototype. Usable video was reported, less clean than C5VRX-3; this did
  not establish extra range. See the local experiment README/DESIGN/RESEARCH.
- **Direct Gain V5 comparison:** branch `fix/c5vrx4-direct-gain-v5`, revision
  `9736f1941add0e1b7cb21e743ea03628d1368bf2`, separates the initial native-gain
  policy from the detector. Do not attribute the first picture comparison
  entirely to demodulation or entirely to gain.
- **Phase8 STATIC/HISTORY span75:** branch `feat/c5vrx4-phase8-history`, revision
  `e2dc83d4b2ab3ae62c50cf6fbf2b184941acd71b`, keeps three bundles / `[D,D,D]`,
  uses a 512x32 shared LUT and compares static phase decode with bounded
  near-origin history. Read its `legacy/c5vrx4/PHASE8_THREE_BUNDLE.md`.
- **Protected lane handover:** branch `feat/c5vrx4-lane-handover`, revision
  `fae4dbc514c6141d7a05a58e0cce6b2f066d733b`, adds overlap-window checks,
  one-step finer upgrades, immediate coarse recovery and persistent phase
  history. Six routing writes remain sequential; no lane tags prove seamless
  per-sample switching. Read its `legacy/c5vrx4/LANE_HANDOVER.md`.
- **Unwrap75 (PR #145):** branch `feat/c5vrx4-unwrap75`, revision
  `7564c117e569a558c4dc24bfc7d40e1922724d88`, uses P/M1/M2/C sign trajectories
  plus full Phase8 endpoints, LUT16 parity/counter banking and three bundles.
  Both middle samples contribute to winding classification, not final video
  amplitude. Host exactness is bounded to decoded adjacent steps <=63 Phase8
  bins (<90 degrees); final DAC transfer uses a four-bin midpoint (up to two
  bins error). Ambiguous opposite-quadrant paths go neutral; corrected outer
  trajectories saturate. The exhaustive 524,386,048-trajectory oracle is host
  evidence, not a measured range/FIFO/colour result. Read its
  `legacy/c5vrx4/UNWRAP75.md`; it is not a file on this main snapshot.
- **Fixed ultrafine comparison:** branch `feat/c5vrx4-ultrafine-test`, revision
  `ab10140f438a7eb0bfa8d5dc051e15608836a5a9`, keeps Unwrap75 and fixes lane2
  `{9,6,5,4}` at every distance, deliberately without automatic coarse fallback.
  ADC-window folding and phase-endpoint winding are different failure modes.
  Read its `legacy/c5vrx4/ULTRAFINE_TEST.md`.
- **Sync flywheel (C5VRX-4, 2026-10-05):** extends the C5VRX-3
  `feat/sync-flywheel` idea (PLL line tracker, repair in the raw ring before
  the TX read).
  - It detects without demodulation, from the 75 ns endpoint phase step at a
    few points per line (~32 evaluations per clean line in the host model).
  - It synthesizes a constant per-sample phase step, so the result is valid
    for every 3-byte span alignment.
  - It also rebuilds vertical-interval slots, keeping the field timing on the
    PLL line grid.
  - Host evidence: `legacy/c5vrx4/tools/test_sync_flywheel.c`, PAL/NTSC fades, weak carrier,
    re-lock and mask-safe bytes. Goggle behaviour is not measured.
  - Hardware, 2026-10-06: default on starved IDLE (task watchdog in
    gain_v3_obs/fusion_obs/analog_agc), the USB console and the menu. Now
    default off, `w` opts in; the CPU gate in `SYNC_FLYWHEEL.md` must pass
    first.
- **Span75 post-detection aliasing (host model, 2026-10-05):**
  `legacy/c5vrx4/tools/postdetect_alias_model.py` runs the generated
  Unwrap75 LUTs on simulated Q4/I4 bytes.
  - The 75 ns endpoint delta is resampled at 13.33 MS/s with no anti-alias
    filter. FM noise from 6.67–20 MHz folds into the video band: about
    -3 dB total and -3.5 to -4.6 dB per 1 MHz band versus a filtered
    detector.
  - avg3 (span-mean phase difference, still 13.33 MS/s) recovers it, but
    needs about five lookups per span, beyond the measured TX BitScrambler
    throughput.
  - A narrower pre-detection width also reduces the loss. The C5VRX-4 V5
    gear therefore uses a measured edge profile (`BW_EDGE`: ≥ 14 MHz, ≥ 0.5 dB
    lower noise bandwidth) at the range edge.
  - Host evidence only; see `legacy/c5vrx4/docs/range-max.md`.
- **Wider IQ / complex filter / FM tracking / external DSP:** investigated C5-only
  tap/filter/lane options and a twelve-lane I6/Q6 external-processing route with
  DC/IQ correction, complex channel filtering, adjacent/tracking FM, matched
  de-emphasis, anti-alias filtering and calibrated DAC reconstruction. This is
  a candidate architecture in `legacy/c5vrx4/RESEARCH.md`, not implemented
  FPGA/CPLD hardware, a selected component, or a demonstrated sensitivity gain.

For branch-only documents, use `git show <recorded revision>:<path>` or the
canonical repository at that revision. Never invent a same-named main file or
silently merge an experiment's assumptions into the production contract.

### C5VRX-5: the distilled receiver (repository root)

Operator decision 2026-10-11: a separate, simple generation with only what
PR #190 showed to help. MODEM_DIAG fixed ultrafine Q4/I4 -> PARLIO RX40 ->
raw 32 KiB ring -> TX BitScrambler U85Q (model `1a09f5c092e8`: untrained
32-phase tracker, PAIR4411 shared word, origin quadrant tokens, -6..+9 MHz;
two bundles, LUT16, unique CVBS20 as [D,D] at TX40) -> 6-bit DAC. Direct
Gain V5 (fixed lane, staged recovery, persisted map), per-gain DC hold with
the carrier latch, DC drift nudges, sampling-phase check, AFC centring the
porch on -1.0 MHz (matched to the U85Q window, simulation; was -436 kHz).
PR #196 (`research/c5vrx5-range`): all picture-interrupting calibration runs
only in a bounded STARTUP phase of a boot (or after the menu CALIBRATE item),
never in live video, and only for what is missing: sampling-phase repair, one
filter sweep (12 MHz, 10 MHz and 16 MHz classes, lowest noise bandwidth; own channel
with the VTX off, else a quiet channel >= 40 MHz away), DC rows (noise: all
rows still missing; carrier: current row), and the VTX deviation. Quiet = envelope
statistic of receiver noise (not phase coherence: 4-bit noise reads 36-61 %
coherent, which rejected every channel, board 2026-10-11). Optional receive
profiles per channel (12 MHz + kp 0.85 default; 10 MHz + kp 0.6 for
deviation <= 1.05; 10 MHz + kp 0.5 for <= 0.80; 16 MHz + kp 0.85 for
>= 1.60), chosen from three
consistent deviation batches (highest batch median; host test
`test/test_profile.c`: no wide VTX >= 1.15 enters a narrow profile),
applied while the menu/status screen owns TX at startup (in flight only
by the optional live switch below). A stored deviation is
not a VTX identity: every boot and channel change starts DEFAULT; a matched
profile needs a fresh measurement in the bounded startup phase (waits up to
20 s for a VTX switched on after a VTX-off calibration). Afterwards
continuous VTX matching (`profile_logic.h` dev_monitor, host test
`test/test_profile.c`) starts a passive measurement every 30 s on live
video, gated on burst-confirmed sync, C/N >= 8 dB (a weak carrier
reads the deviation narrow), no clipping and held gain; not on the AFC lock
(2026-10-18: the sync-to-porch span is offset-free, and the lock gate kept
the deviation unknown on the board; narrowing the filter still needs the
lock, `profile_live_switch_wanted`); a cycle that cannot
decide within 120 s restarts. Each result updates the session deviation
(kp ladder class), is logged with agree/mismatch streaks and kept as PENDING
(NVS only on a class change or >= 0.03 change, at most every 5 min).
Live profile switch (operator decision 2026-10-12, NVS `prof_live` default
on, console `A`; off = report only): widen (narrow -> DEFAULT -> WIDE)
after one reliable result, narrow only after two agreeing results
(`profile_live_switch_wanted`), via a filter relock plus
`transport_apply_auto` to the profile start level (verified restart, shared 5 s guard, filter
restored on failure); never during startup, menu or a latched fault.
Host-tested; in-flight board test pending. Filter writes are read back (requested vs
applied); a sweep is stored only after an exact channel/offset restore, a
read-back best setting, nonzero noise bandwidth and a successful NVS write.
Passive FIELD_DIAG (`main/diag.c`, `diag_logic.h`, host test
`test/test_diag.c`, `tools/field_diag_compare.py`, protocol
`docs/FIELD_TEST.md`): per-second records from existing statistics and a
dropout classifier (CALIBRATION_OR_PHY / FAST_EVIDENCE_LOSS /
AT_RF_FLOOR / DEMOD_CLIFF / UNKNOWN), evidence categories, not causes or
dBm. Coverage and sync use all control observations, with conditional sync
reported separately; AFC sync is not displayed-image quality. `diag_trace.h`
keeps bounded timestamped 16-pre/16-post onset observations at the existing
nominal 50-ms control cadence, including gain/tuple/filter/profile and counters.
Console-only output, oldest unread event retained with skip count; gaps and
ownership/retunes cut history. Faster fades and a luma cliff retaining sync
still need IQ/CVBS captures. Unknown coverage/C/N stays UNKNOWN; coherent IQ
gets a DSP/output investigation before any headroom/fade hypothesis. Console `L` is a bench-only raw-IQ lane comparison (ultrafine / fine /
coarse, ABBA, behind a status screen); production stays fixed ultrafine.
Channels above 5885 MHz (R8, E6-E8) are flagged out of the documented C5
range. AFC re-acquisition retunes only at a C/N estimate >= 6 dB
(`main/cn_estimate.h`: DC-removed envelope -> C/N per filter class;
the historical coherence model is retained only for comparison): below it the porch
estimate is pulled toward the receiver centre (`research/afc_bias.py`,
host model: +65..250 kHz at 0-2 dB C/N, which passed the MAD gate), so the
old gate could walk the offset the wrong way at the range edge. Open-field
propagation (two-ray/Fresnel with low antennas: +6 dB at 500 m, +12 dB at
1 km for 1 m heights) is documented in `research/README.md` section 7;
textbook numbers, not C5VRX measurements. Direct Gain fade hold (`main/gain.c`
`DG_FADE_HOLD_US` 10 ms, host test `test/test_gain_fade.c`): within
10 ms of a healthy window an empty or weak window does not raise the gain,
so a motion null no longer jumps to the table maximum and folds the
returning carrier (model: 1.8 ms and 4 writes per null before, 0 after;
a carrier that stays weaker recovers up to ~10 ms later). NO_CARRIER
still returns to the table maximum, after at most the hold. Open-field
link model (`research/field_antenna.py`, README section 9, textbook
physics): at FPV grazing angles the ground bounce of an RHCP VTX returns in
the opposite hand, so an RHCP receive antenna rejects it while a linear one
gets the full two-ray nulls (model: ~0.4-0.55 km linear vs ~1.1-1.25 km RHCP
omni vs ~3 km 8 dBi RHCP patch at 25 mW). Identify the actual bundled FPC antenna before asserting its band.
The link model's RF threshold/NF inferred from Wi-Fi PER sensitivity is an
assumption, not measured C5 WBFM sensitivity or proof that RF is not limiting. WIDE profile (filter class >= 16 MHz, kp 0.85, deviation
>= 1.6; filter blob v3): the board VTX swung -5.0..+6.25 MHz (deviation
~1.7), and the 12 MHz default clips wide VTXs (bit-exact U85Q,
`research/data/wide.txt`: 1.7 3.64 vs 3.26 dB at 16 MHz, 2.0 8.03 vs
5.27 dB; 1.4 stays best at 12 MHz). Simulation; board test pending. AFC
re-arm (`main/afc_ctrl.h` `AFC2_CTRL_REARM_WINDOWS`, host test
`test/test_afc.c`): the 4 x 250 kHz budget only came back after a lost
lock, so a VTX > 1 MHz off never centred or locked (0.2-0.45 MHz residual
all session); it now re-arms after 3 s while unlocked, and the offset clamp
is +-2.5 MHz (was 1.5; vendor helper range not yet hardware-tested). AFC
offset sign self-check (`AFC2_CTRL_SIGN_CHECK_KHZ`, NVS `afc_sign`, `T`
afc_sign): the sign was never hardware-confirmed (C5VRX-4 INTEGRATION.md),
and with the opposite sign the AFC walked to the clamp (host test: 2.8-4.5
MHz residual); after a step >= 150 kHz the next stable estimate must move
toward the target, else the sign flips and is stored. DC startup: a failed
probe is no longer recorded as a noise floor (it marked every lower row
FLOOR in NVS and stopped the bar half way); the floor end now completes the
bar with an explanation. With the VTX on at boot, missing DC rows (the high
gain rows used far out) are measured on a quiet channel >= 40 MHz away with
an exact, checked restore (startup only): before, they stayed uncorrected
(~1.1 cell, C5VRX-4 video_dco.c), which in the stress matrix broke the
goggle line lock at 4 dB C/N (lines shifting in bands). Codes measured on
another channel are assumed close (baseband offsets per gain row); live
drift nudges trim the rest; board confirmation pending. DC tables are stored
per channel (`main/dco_slots.h`, host test `test/test_dco_slots.c`): six
LRU slot blobs `dco0`..`dco5` plus index `dco_idx`, the legacy single `dco`
table adopted once; a channel without its own table uses the nearest
measured one as a template (tables measured on their own channel preferred),
so a channel change no longer forces a DC search or a quiet-channel trip.
The index keeps a per-row template mask: a boot that finds the own channel
quiet (VTX off) re-measures exactly the template rows; CALIBRATE and carrier
searches also replace them. With the VTX on, template rows are used as is. Direct Gain learns
its gain map only with a confirmed VTX (burst-confirmed sync within 500 ms,
`learn_enabled`): the coherence test alone passes plain 4-bit noise through
the 12/10 MHz filters, so VTX-off noise ratios were learned and persisted;
map version 4 drops such maps once. The map is saved every 30 s when it
changed. Gain map preset (`main/gain_map_preset.h`, built by
`tools/gain_map_preset.py` from console `M` GAINMAP lines of measured
boards, host test `test/test_gain_preset.c`): a lowest-confidence prior
for boards without a saved map; empty until board data exists (not
simulated: RF-stage/BB dB values are not known well enough to model).
Stress matrix `research/situations.py` (README section 11): -0.75 /
-1.0 MHz residual already costs the picture at 4 dB C/N; long echoes hurt
all profiles alike (no equaliser possible). Profile gains (+1.7 / +3.2 dB) are SIMULATION only (`research`);
operator report 2026-10-11: much less static but little more range, so no
range gain is claimed; firmware is build-tested, board test pending. No options menu, labs, native AGC, AutoFit/Fusion,
flywheel or idle raster. Manual OUTPUT (PR #196 review follow-up, 2026-10-12)
scales only DAC6 low LUT bits, with 1x boot default and shared 5-second
restart guard; high gains may clip wide VTXs. Automatic output-level servo
is simulation-only (`research/output_level.py`): ideal sync/white references
are not a proven live estimator or measured HDZero compatibility. Fresh
settings default to AUTO kp+LP by explicit operator decision; saved manual
OFF is respected. Live restart errors perform checked teardown and one
verified rollback, then latch live OFF on failure while USB remains available
(`transport_recovery.h`, host fault-injection tests). NVS namespace `c5vrx5`. CI:
`.github/workflows/c5vrx5.yml` publishes `c5vrx5-v5.0.N` and `c5vrx5-pr-N`
for the flasher's C5VRX-5 tab. Board: runs and holds video; range is not
yet measured. Evidence: `docs/FINDINGS.md`.

Envelope C/N (2026-10-13, `research/cn_envelope.py`,
`research/data/cn_envelope.txt`): extends the existing calibration envelope
statistic to the gain observer's 4 x 64-byte probe. Phase-step coherence read
9-13 dB next to the VTX on the board; runtime kp, AFC, profile gates and
diagnostics now use DC-removed power mean-squared/variance instead. Four
filter-class tables reproduce from PAL, two seeds, deviation 1.0; deviations
0.69/1.4 differ by up to ~1.1 dB through 12 dB C/N. Fixed IQ points stay
unknown and do not inflate per-second envelope averages. Above ~15 dB,
filter AM and quantisation limit accuracy. Host model/tests and firmware
build only; new estimator hardware validation and observer timing pending.
Gain-driven LP now has independent program evidence: AUTO kp+LP enters from
G70 even with missing IQ or gain writes, retains LP at G65-69, and releases
below G65 after the adaptive hold. Missing C/N holds kp; retunes and PHY
ownership hold LP. Host regressions follow this through the switch guard.

U85Q-LP (`research/u85q_lp.py`, README sections 12 and 16, `main/u85q_lp.bsasm`;
the range-edge program of the kp + LP ladder, K mode 2): U85Q plus a post-detection IIR
low-pass with pole 1/2 in counter A (controller LDCTIA, worker ADDCTIAH). It
uses the same LUT and the same two bundles. It is the only post-detection
filter found to fit: in LUT16 mode counter B cannot be routed next to LUT
bits, and the counter operand shares output[25:16] with the LUT address.
- Evidence: an emulator on the ESP-IDF assembler parse reproduces U85Q and
  U85Q-LP bit-exactly; CI assembles the program for esp32c5.
- Simulation: the range limit improves by 1.6 dB at deviation 1.0 and by
  0.4 dB at deviation 1.4 (lock-limited). The low-pass wins below ~9-12 dB
  C/N and costs ~2-3.5 dB SINAD on a strong signal.
- Program switch (`transport_set_lp`): the paused instruction rewrite
  produced persistent black video on the board (operator, 2026-10-12),
  while USB stayed alive. Use the existing stop/load/reset/start topology,
  restore and verify the current kp LUT before starting, and fall back to
  the previous program with its baseline LUT on verification failure.
  This briefly interrupts video; emulator convergence does not prove a
  seamless hardware handover. `prog_restart_us` records the duration;
  guarded owned-ring bench evidence is in `docs/FINDINGS.md`.
- Policy: LP below ~6.5 dB C/N (7 dB mid class, 8.5 dB wide/unknown; smoothed, 1.5 dB hysteresis); the shared
  five-second actuator cooldown and two-second target confirmation apply.
  A wide VTX returns to U85Q at the floor.
- Simulated limit vs today: +3.35 dB (deviation 0.69), +1.6 dB (1.0),
  +0.36 dB (1.4).
- Bench gate on `P` toggles: `prog_restart_fail`, `prog_verify_fail`,
  `kp_verify_fail`, `tx_empty`, `bs_overload`. Board timing and the goggle colour killer are
  untested.

Verified kp ladder (`main/kp_ladder.h`, `kp_tables.h`, `transport_kp_tick`,
`research/kp_morph.py`, README section 13; fresh settings default to
AUTO kp+LP, a saved OFF is respected; `K` cycles OFF/AUTO kp/AUTO kp+LP,
`k`/`j`/`P` are manual and save OFF; active on every profile):
- One tracker topology (2026-10-12, extends PR #196): `u85q_kp060/kp050.bsasm`
  have the U85Q instructions and their LUTs are `kp_tables.h` levels 5/7
  bit-exactly, so MATCHED is a narrow filter plus a start level and the
  ladder (with LP) also runs there; the two programs are no longer built.
  Narrow-filter curves come from `research/best_combo.py` FILTER_MHZ=10
  (`data/best_combo_10mhz.json`): range limit 0.59 vs 1.62 dB (deviation
  0.69) and 0.59 vs 1.10 dB (1.0) against the fixed MATCHED tracker.
  Simulation only; board test pending. The filter sweep adds codes 52/62/63
  (blob v4): the board's narrow class sat on the old grid end (code 60,
  11.9 MHz noise bandwidth, not the modelled 10 MHz).
- Original entry-by-entry RUN writes and a PAUSE-only experiment both failed
  LUT verification on the physical C5 (2026-10-12). Do not restore them from
  host models. CPU addressing follows LUT width too (ESP32-C5 TRM): keep
  LUT16, reselect the address and check the low 16 read bits. The first
  candidate 65946f1 incorrectly inferred a fixed 32-bit CPU width from an
  IDF comment and failed on the board; that inference is superseded.
- Actual kp/program targets are loaded and fully verified while halted,
  then restarted with the current raw-ring geometry. Only a complete
  verification updates the shadow. Failed switches return to baseline
  video. Changes briefly interrupt output; no seamlessness/range claim.
- Every live kp/program change (including manual P/k, K mode changes and
  failures) shares a five-second cooldown. AUTO additionally requires the
  same combined kp/LP target for two seconds of usable observations in the
  same context. K cannot reset the physical cooldown. Manual P during it
  is refused, not queued. Test `test_kp_switch_guard.c` and `test_kp_lut_io.c`.
- Apply an automatic kp/LP pair in one restart, not two successive restarts.
  The curve and smoothed C/N still choose the target. Unknown/wide deviation
  remains conservative; lower kp does not automatically mean more lock.
- Operator `lutfix2` evidence: 32,768 verified LUT writes, one kp step and
  31 program changes with zero readback/transport failures; the visible
  program change was barely noticeable. Subsequent P spamming hung in
  ESP-IDF `parlio_tx_do_transaction`'s unbounded FIFO-ready wait and starved
  IDLE. Use an owned, fixed nine-descriptor raw-ring TX chain, do not
  re-submit driver loop transactions; wait at most 20 ms with vTaskDelay,
  then return failure and attempt baseline recovery. Cooldown alone is not
  a substitute for bounding hardware waits.
- Edge gating (2026-10-13, `research/ladder_gate.py`, README section 18):
  the ladder and its confirmation must not use the AFC's envelope-
  stationarity test (receiver noise near threshold fails it; 46 % of
  12 MHz windows pass at 0 dB C/N), only settle / gain write / context
  (`kp_ladder_window_usable`). AUTO confirms program + direction
  (`kp_switch_confirm_toward`), not the exact level. Control-loop model:
  1.0-2.1 dB at the edge in 10 of 15 cases, within 0.2 dB of an oracle;
  slow fly-out about unchanged. Simulation; board check pending.
- Evidence limits: host simulations still describe the DSP curve, not
  continuous hardware output; goggle behaviour and final spam-test evidence
  are recorded in `docs/FINDINGS.md`. See README sections 13-16 for the
  positive kp curve and negative software-replica selection results.

U85-CVT (`research/u85_cvt.py`, `cvt_precision.py`, `CVT_STATUS.md`,
`CVT_HARDWARE.md`; PR #196 comments 6124064221/6124244391): an immutable
observation-dependent phase law fits the U85Q 32x32 LUT16 two-bundle schedule
(20 + 5 candidates bit-exact in the emulator). Held-out validation (3360
same-Q4 results) is negative versus MATCHED (-0.89 / -1.09 dB mean); its
weak-signal gain with a fixed LP is the LP's, not the law's. No exact
variable-alpha post-LP fits two bundles; periodic LP/bypass stays rejected.
The generated programs are archived in `research/`; the firmware bench
build (console V) was removed 2026-10-12. Sampling phase
(`main/sphase_logic.h`, host test `test/test_sphase.c`): above the
absolute limit only a nine-position scan with contrast is evidence of a
mid-transition read; positions that read alike are `noise_limited` (no slip,
not a phase fault). Host model: 0.6-cell intrinsic white noise lifts the clean
position to ~725 (limit 300), the board's repeated `unsettled` pattern. DC row
held during scans; final qualification in the live TX context. Board repeats
pending; no normalized gate (phasefix6 failed board repeat scans).
Carrier sampling phase (2026-10-13, `sphase_live_*`, research README section
19, `research/sphase_study.py`): host model, one mixing LSB costs ~0.3-0.6 dB
and multi-bit mixing 1.4-5 dB. On a carrier (sync, gain >= 6 below max) the
cleanest of three positions is chosen once per boot, probed every 10 s
against 150 and repaired after two excursions (no per-boot cap, false alarms
back off 60 s -> 16 min). A source-synchronous MODEM_DIAG clock would remove
the cause but is not established. WIDE profile own 16 MHz rows (kp 0.85, LP
< 9.5 dB; no range gain, sharper picture); MATCHED uses 10 MHz rows only for
a narrow filter measured <= 11 MHz; FIELD_DIAG logs kp_x100/lp. Simulation
and host tests; board check pending.

VTX-on PHY sensitivity lab (console `S` + suite key, `main/sense_lab.c`,
`main/sense_lab_logic.h`, `tools/phy_sense_lab.py`, docs/PHY_SENSITIVITY_LAB.md):
extends the PR #254 VTX-off noise lab with a weak VTX carrier present and
scores carrier and noise together (M2M4 C/N, AFC-decoder video SNR, FM
clicks) in STOCK-TEST-TEST-STOCK with shams. Bench only, pinned PHY only,
every write restored and verified, nothing saved. Board 2026-10-26 (5865 MHz,
docs/FINDINGS.md "VTX-on PHY lab, AFC window and digital mode 12"): only
digital mode 12 reproduced better (+0.2-0.5 dB C/N, fewer clicks); a person
at the bench moves the carrier 4-11 dB, so runs need the operator away.
AFC window (same date): one 4092-byte descriptor holds a timing-confirmed
B/W sync pair in only ~45 % of windows (board IQ replay), so the AFC never
locked; the control loop measures the AFC on the 8190-byte contiguous
snapshot instead (30/30 valid). Gain observation keeps the descriptor.
Board lock/profile check pending.

### IQLaneFusion research

IQLaneFusion research extends U85 and the older adaptive-lane/history work:
`docs/IQLANE_FUSION.md`, `research/iq_lane_fusion.py`. Exact Q10 cell
likelihoods, mathematical bitset/gain screening, factorized phase/frequency
reference, route-transition replay and a fixed-context compressed BS experiment
are host research. The read-only firmware shadow (console J) was removed
2026-10-12 as unused bench code (git history keeps it). Full posterior and compressed BS
program are not equivalent. Preserve the negative fade comparison. Safe live
arbitrary routing/decoder handover, hardware timing and range remain unproved;
do not treat model-only proposals or unused counters as physical validation.

## Archived C5VRX-3/4 invariants

Historical, for reading the legacy trees (moved from `AGENTS.md`):

- **Direct Gain V5 is the default gain owner** (Direct Gain V3 core, direct
  first-window correction, table-maximum listening without a carrier, 200 us
  observer with descriptor dedupe, anti-hunt damping). Native
  hardware AGC is an opt-in menu/`N` option only: on a continuous carrier the
  C5 packet AGC re-acquires every ~21 us on a different gain, which caused
  the line noise and grain (legacy/c5vrx3/docs/native-agc-v2.md). Do not make native AGC
  the default again without a new hardware comparison that beats V4.

- VTX presence and USB must never gate or pace IQ production.
- Native ESP32-C5 hardware AGC is opt-in (NVS `c5vrx/native_agc = 1`, set by
  `N` or the RF page profile cycle). In that mode never call
  `phy_disable_agc()` / `phy_rfagc_disable()` and never force RX gain; every
  firmware gain write must stay refused at `rf_set_rx_gain()`.
- The normal live source is MODEM_DIAG Q4/I4 captured by PARLIO RX; active
  MAC-owned dump SRAM is a diagnostic writer, not a readable live source.
- Do not turn a physical SRAM or DMA block boundary into a DSP reset.
- Do not claim sample-gapless RF or AV transport without its physical proof.
- The normal live path recovers the transmitted composite waveform; it does not
  decode pixels or regenerate PAL/NTSC. (A sync-flywheel experiment that
  repairs broken H-sync is parked on branch `feat/sync-flywheel`; see
  legacy/c5vrx3/docs/range-max.md for why it is not in main.)
  **Exception, C5VRX-4 only (operator decision 2026-10-05):** the goggles must
  never see a moment without valid PAL/NTSC sync, so the C5VRX-4 sync flywheel
  (`legacy/c5vrx4/SYNC_FLYWHEEL.md`) rebuilds missing or noisy H-sync
  pulses (with part of the front porch) and vertical-interval slots in the raw
  ring ahead of the TX read, on a PLL line grid locked to the real VTX sync.
  It must keep these properties:
  - picture content is never decoded or altered;
  - clean pulses are left untouched;
  - the colour burst is not killed;
  - it never writes the newest completed RX descriptor, which the control
    observers (V5 NO_CARRIER, idle raster, level servo, AFC) copy.
  It is off by default since its first hardware run starved the CPU
  (`w`, NVS `sync_fw` or the menu SETUP page opts in) and hardware-pending.
  The root C5VRX-3 path is unchanged.
- Keep USB/debug outside realtime pacing.
- Do not silently change the tested XIAO D4..D9 DAC pin order or the physical
  8.2k/3.9k/2k/1k/470R/240R plus 200R network.
- Keep live output compatibility explicit: GOLDEN is the selectable live
  demodulator and supports both `6BIT@40` and experimental `4BIT@80`.
  TRAJ V2 remains a research artifact and must not appear in the live menu.
- The standalone menu raster is always emitted through the byte-oriented
  40 MHz TX geometry (six physical bits by default, eight for the experimental
  custom-board Phase8 option). On menu exit, recreate the live TX unit for the
  selected output mode before restarting the flight BitScrambler.
- Custom-board pin mappings and the experimental full-byte Phase8 output are
  described in `legacy/c5vrx3/docs/custom-board.md`. Preserve the default XIAO mapping, shared
  IQ routing and startup conflict checks. Eight-bit host simulation is not a
  hardware validation or an improvement in RF acquisition resolution.
- Do not move the full menu GDMA scatter chain back into static BSS. PAL needs
  up to 6,348 12-byte AHB-DMA descriptors (~76 KiB), but that chain is used
  only while the standalone menu owns TX. Count the active raster first,
  allocate exactly that many descriptors from
  `MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL`, and free them only after
  live TX has been restarted and GDMA no longer references the menu chain.
  This is what keeps the widened menu inside the ESP32-C5 static DRAM limit.

## IQ Fusion Engine experimental contract (archived C5VRX-3)

Historical contract of the C5VRX-3 `FUSION EXP` profile; its principles
(no CPU in the 40 MS/s path, NO_CARRIER returns to high gain, zero PHY writes
on clean IQ) still apply to any future C5VRX-5 control work.

The experimental `FUSION EXP` profile may combine multiple estimators from a
completed Q4/I4 control snapshot, but it must never insert CPU processing into
the 40 MS/s realtime path.

Useful fusion evidence includes adjacent 25 ns phase deltas, 50 ns endpoint
winding disagreement, lag-4 disagreement, robust local phase-slope consensus,
near-origin confidence, Q_phase, clipping, I/Q centering/skew and bounded
semantic-video validation.

The slow learner may select only PHY states whose actuator semantics are
already established. A symbol name in the closed PHY blob is not sufficient
evidence for production use. Undocumented LNA/BB/filter controls require a
prototype, register-diff and raw-Q4 A/B before becoming learner actions.

At the range edge, loss of sync/video is never by itself evidence to reduce
sensitivity. NO_CARRIER must return to the known high-gain survival state.
Clean/high-confidence IQ should produce zero PHY writes.

Exact adjacent-FM waveform fusion remains gated by issue #23. Every 40 MS/s IQ
sample must participate before 2:1 reduction, and a live implementation must
prove sustained hardware throughput plus state continuity before replacing the
current gapless BitScrambler path.

## PHY noise/current/loop-cap/spur bench work (2026-10-28)

Twotoz's requested continuation of PR #254: VTX-off paired-noise controls,
targeted witnessed bias states, the real current-level setter, recovered
loop-cap PBUS/cache footprint, and inferred spur-slot experiments. Findings,
negative results, the invalid empty-test -5.4 dB correction and evidence
limits are in [PHY_NOISE_RESULTS_2026-10-28.md](PHY_NOISE_RESULTS_2026-10-28.md),
with per-profile data in `research/data/phy_noise_lab_2026-10-28.json`.
These bench controls do not establish a lower absolute NF and are not
production boot settings. The current-level argument is not calibrated mA.

## LCD output route: Waveshare ESP32-C5-LCD-1.47 (2026-10-30)

A board build without a DAC ([LCD147.md](LCD147.md)). Source/tap: the
unchanged MODEM_DIAG Q4/I4 ring at 40 MS/s (raw 32 KiB, nine descriptors).
Processing: CPU, read-only: a phase-step FM discriminator with sync
flywheels grabs 228 x 172 (4:3) grey pictures from two fields; a second task
converts them (or the menu's logical-pixel UI rows) to RGB565 and sends
4-row strips over SPI2 DMA to the ST7789. The TX BitScrambler/PARLIO path
keeps running with no pins, so its verified restarts are unchanged; it no
longer reaches a display. To give the grabber the CPU this build runs the
gain observer every 2 ms instead of 200 us and the control task below the
grabber (it runs in a 10 ms rest after every picture), with its AFC window
only every 300 ms while locked (17 ms of CPU each). This extends
KonradIT's `tembed-link` UART video link (CPU grabber on the same ring,
measured 24-25 PAL frames/s on a XIAO) and follows zerowidth's `p4usb`
direction of a software decoder; it is not the C5VRX-1 CPU-paced route
(the CPU never paces IQ here). Evidence: on one board, a live PAL picture
from a bench VTX (B5, -38 dBm), 240 of 256 pictures complete in 18 s (13.3
complete pictures/s), sync jitter 15 ns, AFC locked; at the edge (VTX in
foil) ~80 % of pictures complete at -87.5..-90 dBm (C/N 7-9.5 dB) with the
grabber's own I/Q DC compensation (61 % without, same position, A/B); a marker probe showing
the RX descriptor register one node ahead of the data (docs/FINDINGS.md);
host simulation of `grab.c` on synthetic FM PAL/NTSC. Not measured: NTSC,
the slower gain loop's fade behaviour in flight, or range in the field.

## Removed sync flywheel (PR #212/#218 history, removed 2026-10-25)

The C5VRX-5 flywheel (PR #212, witness fix PR #218, bounded sync-loss
scheduling 2026-10-18) is no longer firmware. Its findings stay in
`docs/FINDINGS.md` ("Bounded background work during sync loss", "Flywheel
witness freshness and delayed observations", "Sync flywheel removed") and
`research/data/flywheel_desync_2026-10-18.json`; the code is in git history
before the removal.
