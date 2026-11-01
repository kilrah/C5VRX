/* C5VRX-5 by Twotoz and the C5VRX contributors: receiver-wide state.
 *
 * Data plane (hardware only): MODEM_DIAG Q4/I4 (fixed ultrafine lane) ->
 * PARLIO RX 40 MS/s -> 32 KiB cyclic raw ring -> TX BitScrambler U80P64-R1
 * (50 ns span, two bundles, LUT16) -> PARLIO TX 40 MHz [D,D] -> 6-bit DAC.
 * The CPU only observes completed RX descriptors and owns gain, AFC, DC,
 * filter, menu and console. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/dma_types.h"
#include "gain.h"
#include "menu_raster.h"
#include "menu_ui.h"   /* MENU_* items */
#include "rf.h"

#define IQ_RATE_HZ          40000000u
#define RAW_RING_BYTES      32768u
#define LIVE_PIPELINE_LAG_BYTES (3u * RAW_RING_BYTES / 4u)
#define DAC_IDLE_CODE       20u          /* blanking level; sync is 0 */
#define CONTROL_BYTES       4092u        /* one completed GDMA descriptor */
#define PROBE_REGIONS       4u
#define PROBE_REGION_BYTES  64u
#define PROBE_BYTES         (PROBE_REGIONS * PROBE_REGION_BYTES)
#define MAX_RING_DESCRIPTORS 16
/* AFC centres the VTX blanking level here. The -6..+9 MHz output window
 * holds more of the sync-to-white swing with the porch at -1.0 MHz than at
 * the -436 kHz that suited the trained trackers: historical U85Q simulation,
 * 400 cases, picture 0.969 vs 0.908, +0.85 dB SINAD, deviation 1.8 VTXs
 * 0.90 vs 0.64 (docs/FINDINGS.md). */
#define AFC_PORCH_TARGET_KHZ (-1000)

typedef struct {
    dma_descriptor_t *dscr;
    uint8_t *buffer;
    uint32_t length;
} ring_node_t;

/* Cheap envelope/phase statistics of a raw Q4/I4 window. */
typedef struct {
    int p_median;      /* median i^2+q^2 of the signed nibbles */
    int q_phase;       /* % of adjacent pairs with a coherent phase step */
    int clip_pm;       /* rail codes, per mille */
    int origin_pm;     /* origin cells, per mille */
} metrics_t;
metrics_t measure_window(const uint8_t *s, size_t n);

typedef enum { STD_MODE_AUTO = 0, STD_MODE_NTSC = 1, STD_MODE_PAL = 2 } std_mode_t;

/* ---- transport.c ---- */
extern uint8_t s_raw_ring[RAW_RING_BYTES];
extern int s_rx_dma_ch, s_tx_dma_ch;
extern ring_node_t s_rx_nodes[MAX_RING_DESCRIPTORS];
extern int s_rx_node_count;
esp_err_t transport_start(void);
void transport_stop_tx(void);
void transport_resume_live(void);
void transport_start_chain(dma_descriptor_t *first);
uint32_t transport_switch_wait_ms(void);
const char *transport_program_name(void);
/* Decoder (transport.c): U80P64-R1 (default) or U85Q, each a fixed program
 * with its own LUT; kept in NVS "demod" (main.c, console m). */
#ifdef C5VRX_P4LINK
/* P4-link build (docs/P4LINK.md): RAW (the default) sends the raw ring,
 * Q4/I4 at 40 MS/s, and the P4 demodulates; P64/U85Q stay for A/B. Its own
 * NVS key, so a saved XIAO-style choice does not override RAW. */
enum { TRANSPORT_DEMOD_P64, TRANSPORT_DEMOD_U85Q, TRANSPORT_DEMOD_RAW, TRANSPORT_DEMODS };
#define TRANSPORT_DEMOD_DEFAULT TRANSPORT_DEMOD_RAW
#define DEMOD_NVS_KEY "p4demod"
#else
enum { TRANSPORT_DEMOD_P64, TRANSPORT_DEMOD_U85Q, TRANSPORT_DEMODS };
#define TRANSPORT_DEMOD_DEFAULT TRANSPORT_DEMOD_P64
#define DEMOD_NVS_KEY "demod"
#endif
uint8_t transport_demod(void);
const char *transport_demod_name(uint8_t d);
bool transport_set_demod(uint8_t d);
const uint16_t *transport_decoder_lut(unsigned *token_bits);
bool transport_live(void);
bool transport_restart_profile(void);
bool transport_faulted(void);
uint32_t transport_restart_errors(void);
unsigned transport_output(void);
const char *transport_output_name(void);
bool transport_set_output(unsigned level);
void transport_stop_chain(void);
void transport_poll_faults(void);
uint32_t transport_fault_count(void);
void transport_print(void);
/* Flight-log V1 output telemetry. Demod is 0 unknown, 1 verified U85Q,
 * 2 reserved for historical LP logs, 3 verified U80P64-R1. The observer
 * fields keep their wire slots for old logs; firmware writes them empty
 * (lock percentages 0xff) since the sync flywheel was removed. */
typedef struct {
    uint8_t flags, demod, standard, output_level;
    uint16_t samples, fresh_samples;
    uint8_t h_lock_pct, v_lock_pct;
    uint16_t native_h_lines, repaired_tips, suppressed_tips, longest_h_unlock_ms;
    uint8_t observer_gaps;
} transport_flog_output_t;
void transport_flog_output(transport_flog_output_t *out);
void transport_flog_identity(uint8_t *demod, uint8_t *output_level);
void sync_dma_c2m(const void *addr, size_t size);
void sync_dma_m2c(const void *addr, size_t size);
int ring_node_index(uint32_t addr);
/* Task-reader token + RX-overwrite/context guards. Nonnull private dst;
 * control 1..4092 B, probe 256 B, sentinel 64 B, snapshot 8190 B, endpoints
 * 1..4092 pairs (8184 B raw envelope). False=invalid/unavailable/stale; retry safely.
 * These bounds alone do not prove DMA publication or original provenance. */
bool copy_control_window(uint8_t *dst, size_t bytes);
bool copy_probe_window(uint8_t dst[PROBE_BYTES], int *node);
bool copy_sentinel_window(uint8_t dst[64]);
bool copy_snapshot(uint8_t *dst);
bool copy_recent_endpoints(uint8_t *dst, size_t n);
void rx_clock_slip(uint32_t us);

/* ---- receiver.c (Direct Gain V5 owner) ---- */
extern direct_gain_t s_gain;
extern volatile uint8_t s_current_gain;
extern volatile uint32_t s_gain_epoch;      /* bumps on every gain/offset write */
extern volatile uint32_t s_control_epoch;   /* bumps on retune/scan/calibration */
extern volatile int64_t s_last_gain_write_us;
extern volatile int s_obs_p50, s_obs_p95, s_obs_origin_pm, s_obs_clip_pm, s_obs_coherence;
extern volatile bool s_obs_stuck;   /* fixed IQ point, not a carrier (gain.h dg_stuck) */
extern volatile int s_obs_env_ratio_x100;   /* gain.h env_ratio_x100 of the latest window */
extern volatile uint32_t s_obs_us_max, s_obs_windows;
extern dg_map_blob_t s_gain_map_saved;
extern bool s_gain_map_saved_valid;
/* Observer-owned context snapshot; refuses unsettled/changed model state. */
unsigned receiver_export_gain_map(dg_map_blob_t *blob);
void receiver_start(void);
bool receiver_active(void);
bool receiver_paused(void);
/* Pause gain control for a calibration/scan that owns the gain itself. */
bool receiver_pause(const char *tag);
void receiver_resume(void);
void receiver_set_gain(uint8_t gain);
void receiver_print(void);
void receiver_print_map(void);   /* console M: GAINMAP line */
/* Gain mode: GAIN_MODE_LEVEL (agc.h, the measured-quality AGC, the
 * default), GAIN_MODE_AUTO (KEEL-1, gain.c; shown as KEEL), GAIN_MODE_RSSI
 * (agc.h, a baseline), GAIN_MODE_MAX (table maximum) or a fixed gain index
 * 20..max. Fixed modes skip the controller entirely; paused owners
 * (calibration, scan) still write and the mode returns after. The codes
 * are stored (prefs) and logged (flight log), so they keep their values. */
#define GAIN_MODE_AUTO  0u
#define GAIN_MODE_RSSI  253u
#define GAIN_MODE_LEVEL 254u
#define GAIN_MODE_MAX   255u
/* The boot mode without saved prefs, and after MAX or a fixed gain: LEVEL
 * (operator decision 2026-10-27, docs/FINDINGS.md "LEVEL on the measured
 * demod quality": KEEL-1 wrote 10-30 times a second on a strong signal and
 * kept writing on noise at the edge). */
#define GAIN_MODE_DEFAULT GAIN_MODE_LEVEL
/* A mode with an automatic controller (not a fixed gain). */
static inline bool receiver_gain_mode_auto(uint8_t mode)
{
    return mode == GAIN_MODE_AUTO || mode == GAIN_MODE_LEVEL || mode == GAIN_MODE_RSSI;
}
uint8_t receiver_gain_mode(void);
void receiver_set_gain_mode(uint8_t mode, bool save);
uint8_t receiver_gain_mode_next(uint8_t mode);
const char *receiver_gain_mode_name(uint8_t mode, char *buf, unsigned size);

/* ---- control.c ---- */
extern volatile bool s_menu_active;
extern volatile int s_signal_strength;
extern volatile bool s_scan_active;
extern volatile unsigned s_scan_progress;
extern volatile int s_cfo_khz;
extern volatile bool s_afc_locked;
extern volatile bool s_afc_enabled;
extern volatile std_mode_t s_std_mode;
extern volatile video_standard_t s_video_std;      /* standard the menu emits */
extern volatile bool s_detected_std_valid;
extern volatile video_standard_t s_detected_std;
extern volatile int64_t s_last_video_us;           /* last burst-confirmed sync */
extern QueueHandle_t s_commands;
void control_start(void);
bool control_prepare(void); /* reserve unchanged control stack before driver heap fragmentation */
video_standard_t menu_standard(void);
void control_print(void);
void control_print_beat(void);   /* STATUS line every 2 s (console.c) */
#ifdef C5VRX_P4LINK
/* Commands only the P4 link sends (p4link.c, docs/P4LINK.md), queued on
 * s_commands next to the console keys: tune channel index n, AFC on/off. */
#define P4LINK_CMD_CHANNEL 0x1000
#define P4LINK_CMD_AFC     0x2000
void p4link_start(void);
#endif

/* ---- calibration*.c (module-internal: calibration_internal.h) ---- */
void calibration_start(void);
bool filter_calibrate(bool automatic);
void filter_request(void);
void filter_request_full(void);
void calibration_dump_request(void);
void firmware_boot_timing(int64_t *radio_ms, int64_t *ready_ms);
bool receiver_export_noise(uint8_t row, float *cells2, uint16_t *samples, int64_t *measured_us);
/* True during bounded splash-only phase alignment or an explicitly
 * requested F/B calibration. Normal boot never opens RF/DC searches. */
bool calibration_in_startup(void);
/* Calibrated analog filter classes: normal (>= 12 MHz, the default),
 * narrow (>= 10 MHz, low-deviation VTXs), wide (>= 16 MHz, wide VTXs).
 * false when that class was not calibrated. */
enum { FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE, FILTER_XNARROW };   /* XNARROW: menu only */
bool filter_setting(int filter_class, rf_filter_t *out);
/* The DC table of filter f is complete on the current channel (startup). */
bool calibration_dc_ready(rf_filter_t f);
/* Flight log (flightlog.c): the active row's DC residual (mcells) and offset
 * (codes), servo writes so far, sampling-phase state code, temperature. */
void calibration_flog_live(int dc[2], int off[2], uint32_t *servo_writes, uint8_t *sphase, int8_t *temp_c);
uint16_t calibration_generation_id(void);
/* gain observer: one-sided clipping (DC on a rail), run the DC servo now. */
void calibration_dc_urgent(void);
/* control.c: AFC offset applied, porch residual (kHz), corrections so far. */
void control_flog_afc(int16_t *offset_khz, int16_t *residual_khz, uint32_t *corrections);
/* Observer DC sums per gain row (receiver.c) for the live DC servo: a row's
 * sums once they hold min_samples (then it restarts); reset a row whose codes
 * changed, or every row. */
/* neg: samples with the I / Q sign bit set (the folding-proof DC sign). */
bool receiver_dc_take_row(uint8_t gain, uint32_t min_samples, int64_t *si, int64_t *sq, uint32_t neg[2],
                          uint32_t *samples);
void receiver_dc_reset_row(uint8_t gain);
void receiver_dc_reset_all(void);
/* A row's sample count and sign counts so far, without taking them: the
 * servo decides early on a saturated sign share (dc_servo.h). */
bool receiver_dc_peek_row(uint8_t gain, uint32_t neg[2], uint32_t *samples);
/* DC CAL NEEDED on this channel: nonzero while a live DC fault or a
 * sustained DC finding the servo cannot fix recommends CALIBRATE; buf gets
 * "FAULT" or "BASEMAP". */
unsigned calibration_dc_needed(char *buf, size_t size);
void calibration_print(void);
void dco_post_gain(uint8_t gain);
/* Chain state for the diagnostics (no side effects). */
const char *calibration_filter_result(void);
int calibration_filter_class_mhz(void);
/* C/N model of the latest observer window; CN_UNKNOWN_X10 on a fixed IQ point. */
int receiver_obs_cn_x10(void);
#include "rx_level.h"
void receiver_level_roll(bool no_carrier, bool video);   /* video: sync seen this second */   /* 1 Hz, the supervisor (diag.c) */
rx_level_t receiver_level(void);    /* last full second */
bool receiver_noise_learned(void);
void receiver_pair_sweep_toggle(void);   /* bench, console Q */
#include "cn_auto.h"
cna_reading_t receiver_cn_reading(void);   /* C/N-AUTO: value, confidence, source */
void receiver_cn_noise_store(uint8_t row, float cells2);   /* startup noise map (calibration_dc.c) */
void receiver_cn_noise_save(void);
/* The startup noise map as the gain table (noise per row, 0 = not measured). */
void receiver_gain_table_measured(const float noise_cells2[DG_STATES]);
unsigned calibration_filter_nbw_khz(void);   /* measured noise bandwidth of the active filter, 0 unknown */
uint32_t receiver_sentinel_forwarded(void);
void receiver_level_feed(const uint8_t *window, size_t bytes);   /* the menu's control windows */
unsigned calibration_narrow_width_khz(void);   /* measured -3 dB width, 0 = none */
unsigned calibration_filter_width_khz(void);   /* measured -3 dB width of the active filter, 0 unknown */
const char *calibration_sphase_state(void);
bool calibration_sphase_settled(void);
/* DC correction in place for this gain: held, or not needed (outside the
 * calibrated rows, or below the measured noise floor). */
bool calibration_dc_ok(uint8_t gain);
/* Hold the current gain's calibrated DC row after a paused gain write (the
 * post-gain hook is silent while paused). true when a pair is held. */
bool calibration_dc_hold_current(void);
/* Explicit bench-only lane comparison (console L): video stops ~2 s. */
void lane_compare_request(void);
void self_noise_request(void);   /* bench: noise floor with the video output on vs off (console N) */

/* ---- menu.c ---- */
extern volatile int s_menu_cursor;
extern volatile int s_exp_sub; /* -1 closed, else EXP_* */
extern bool s_exp_failed;
extern volatile int s_cal_sub;   /* CALIBRATE submenu cursor, -1 = closed */
bool menu_open(void);
void menu_close(void);
void menu_render(void);
/* Boot logo while the receiver tasks start (main.c); Direct Gain runs
 * under it, the menu buttons and the startup phase wait for its end. */
bool menu_splash_begin(void);
void menu_splash_progress(int percent);
void menu_splash_end(void);
bool menu_splash_active(void);
void menu_set_standard_mode(std_mode_t mode);
/* DAC TEST item (dac_check.h): advance pattern -> one flat level per bit ->
 * all -> none -> pattern. Bench only: a flat level carries no sync. */
void menu_dac_step(void);
void status_screen(const char *title, const char *line1, const char *line2, int percent);
void status_screen_end(void);
bool menu_status_active(void);

/* ---- profile.c ---- */
#include "afc.h"
#include "video_quality.h"
void control_video_quality(vq_t *out);   /* control.c: last second */
const char *profile_name(int profile);
int profile_active(void);
int profile_deviation_x100(size_t channel);
int profile_live_deviation_x100(void);     /* measured this channel session, -1 = not yet */
int profile_for_deviation(int dev_x100);
bool profile_set(int profile);
void profile_load(void);
void profile_boot(void);
void profile_channel_changed(void);
void profile_measure_begin(void);
void profile_measure_end(void);
int profile_measure_result(void);
void profile_observe(const afc2_result_t *r);
void profile_commit(int dev_x100);
void profile_live_begin(void);
const char *profile_reason(void);
void profile_set_reason(const char *reason);
void profile_print(void);
/* Menu FILTER: PROFILE_SELECT_AUTO or the chosen profile (this boot only). */
int profile_selected(void);
bool profile_selectable(int profile);      /* calibrated filter with its own DC table (DEFAULT: always) */
int profile_select_cycle(void);            /* hold on FILTER / console f: next choice, applied */
/* Boot: the saved FILTER choice (settings prefs); a class without its own
 * calibrated DC table flies DEFAULT until it has one. */
void profile_select_restore(int select);
bool profile_live_mode(void);              /* live profile switch on (NVS prof_live) */
void profile_set_live_mode(bool on);

/* ---- diag.c: passive field diagnostics (diag_logic.h) ---- */
void diag_window(bool measured, bool video_valid, bool afc_step);   /* control task, every window */
void diag_picture(unsigned found, unsigned expected);                /* control task, every decoded window */
void diag_poll(void);                   /* console task: prints queued records */
void diag_report(void);                 /* console D */
void diag_toggle_stream(void);          /* console R */


/* ---- settings.c ---- */
void settings_load(void);
void settings_save(void);
void settings_save_output(unsigned level); /* only after verified output application */
/* Menu preferences kept across reboots (NVS "prefs", operator 2026-10-27):
 * FILTER choice, GAIN model and PHY FILT mode. Only the automatic gain
 * models are kept: MAX and fixed gains still return to AUTO at boot (a
 * forgotten fixed gain costs range without a sign, 2026-10-20). */
#define SETTINGS_PREFS_VERSION 1u
typedef struct { uint8_t version; int8_t filter_select; uint8_t gain_mode, dfilt_mode; } settings_prefs_t;
bool settings_prefs_load(settings_prefs_t *p);
void settings_prefs_store(const settings_prefs_t *p);   /* written only when changed */
bool blob_load(const char *key, void *data, size_t size);
bool blob_store(const char *key, const void *data, size_t size);

/* ---- console.c ---- */
void console_start(void);
const char *firmware_id(void);   /* "<version>+<commit 12>", the fw= field of log lines */
void firmware_print(void);       /* FIRMWARE line: version, full commit, ELF SHA-256, build time */

/* ---- flightlog.c ---- */
#include "flightlog.h"
#include "flightlog_format.h"
