/* C5VRX-5 by Twotoz and the C5VRX contributors: Direct Gain runtime (V7 decision core, gain.c).
 * A 200 us timer wakes sentinel/gain; full C/N runs every 20 ms with
 * context/age validation. Bound a continuously runnable observer batch so
 * IDLE/calibration get time even if notifications accumulate. */
#include "receiver.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "agc.h"
#include "experimental.h"
#include "cn_estimate.h"
#include "cn_auto.h"
#include "cn_observation.h"
#include "esp_timer.h"
#include "gain_map_preset.h"
#include "phase8_gain_lut.h"
#include "phy.h"
#include "rf.h"

direct_gain_t s_gain;
volatile uint8_t s_current_gain = 62u;
volatile uint32_t s_gain_epoch, s_control_epoch;
volatile int64_t s_last_gain_write_us;
volatile int s_obs_p50, s_obs_p95, s_obs_origin_pm, s_obs_clip_pm, s_obs_coherence;
volatile bool s_obs_stuck;
volatile int s_obs_env_ratio_x100;
volatile uint32_t s_obs_us_max, s_obs_windows;
dg_map_blob_t s_gain_map_saved;
bool s_gain_map_saved_valid;

static TaskHandle_t s_observer, s_sentinel;
static volatile uint8_t s_gain_mode = GAIN_MODE_DEFAULT;

/* Menu/console cycle: LEVEL (the default), KEEL (KEEL-1), RSSI, MAX, then
 * fixed G78 down to G24. */
static const uint8_t s_gain_modes[] = {GAIN_MODE_LEVEL, GAIN_MODE_AUTO, GAIN_MODE_RSSI, GAIN_MODE_MAX,
                                       78, 72, 66, 60, 54, 48, 42, 36, 30, 24};

uint8_t receiver_gain_mode(void) { return s_gain_mode; }

/* MAX and fixed gains are never stored: such a boot starts LEVEL (operator
 * decision 2026-10-20: a fixed gain pressed once used to survive every
 * reboot and cost tens of dB of range without a warning). The automatic
 * models are kept by the settings prefs (control.c). user: an operator
 * change (logged in the flight log). */
void receiver_set_gain_mode(uint8_t mode, bool user)
{
    if (!receiver_gain_mode_auto(mode) && mode != GAIN_MODE_MAX && (mode < 2u || mode > ARC_VENDOR_GAIN_MAX)) return;
    s_gain_mode = mode;
    if (user) flightlog_event(FLOG_E_GAIN_MODE, &mode, 1);
    char buf[16];
    printf("GAIN mode=%s (%s)\n", receiver_gain_mode_name(mode, buf, sizeof(buf)),
           receiver_gain_mode_auto(mode) ? "kept across reboots" : "this boot only; the next boot starts LEVEL");
}

uint8_t receiver_gain_mode_next(uint8_t mode)
{
    for (unsigned k = 0; k < sizeof(s_gain_modes); ++k)
        if (s_gain_modes[k] == mode) return s_gain_modes[(k + 1u) % sizeof(s_gain_modes)];
    return GAIN_MODE_DEFAULT;
}

const char *receiver_gain_mode_name(uint8_t mode, char *buf, unsigned size)
{
    if (mode == GAIN_MODE_AUTO) return "KEEL";
    if (mode == GAIN_MODE_LEVEL) return "LEVEL";
    if (mode == GAIN_MODE_RSSI) return "RSSI";
    if (mode == GAIN_MODE_MAX) return "MAX";
    snprintf(buf, size, "G%u", mode);
    return buf;
}

/* Bench (console Q): the real gain step between neighbouring table rows.
 * The gain alternates between rows hi and hi-1 every PAIR_SWITCH_OBS
 * observations (~5 ms), so both see the same carrier however it fades, and
 * the ratio of their mean probe powers is the real step. Each pair runs
 * PAIR_OBS observations (~1 s), then hi moves down one row, G83 to G3.
 * Probes within 600 us of a write, clipping or folding are left out. One
 * GPAIR line per pair; research/gain_step_measure.py chains them. */
#define PAIR_SWITCH_OBS 25u
#define PAIR_OBS        5000u
static volatile bool s_pair_active;
static uint8_t s_pair_hi;
static uint32_t s_pair_obs, s_pair_n[2], s_pair_skip[2];
static uint64_t s_pair_sum[2];
static int64_t s_pair_last_write_us;
static uint8_t s_pair_last_gain;
void receiver_pair_sweep_toggle(void)
{
    if (s_pair_active) { s_pair_active = false; printf("GPAIR stopped\n"); return; }
    s_pair_hi = rf_gain_table()->max_index;
    s_pair_obs = 0;
    memset(s_pair_n, 0, sizeof(s_pair_n)); memset(s_pair_skip, 0, sizeof(s_pair_skip)); memset(s_pair_sum, 0, sizeof(s_pair_sum));
    s_pair_active = true;
    printf("GPAIR start hi=%u (keep the VTX still; ~%u s)\n", s_pair_hi, (unsigned)s_pair_hi);
}
static uint8_t pair_decision(const dg_observation_t *o)
{
    const int64_t now = esp_timer_get_time();
    if (s_current_gain != s_pair_last_gain) { s_pair_last_gain = s_current_gain; s_pair_last_write_us = now; }
    const int side = s_current_gain == s_pair_hi ? 0 : s_current_gain == (uint8_t)(s_pair_hi - 1u) ? 1 : -1;
    if (side >= 0) {
        const bool folded = (o->rails & 3u) == 3u || (o->rails & 12u) == 12u;
        if (now - s_pair_last_write_us < 600 || o->clip_pm >= 20u || folded) ++s_pair_skip[side];
        else { s_pair_sum[side] += o->pwr_x16; ++s_pair_n[side]; }
    }
    if (++s_pair_obs >= PAIR_OBS) {
        printf("GPAIR hi=%u lo=%u sum_hi=%llu n_hi=%lu skip_hi=%lu sum_lo=%llu n_lo=%lu skip_lo=%lu\n", s_pair_hi,
               s_pair_hi - 1u, (unsigned long long)s_pair_sum[0], (unsigned long)s_pair_n[0], (unsigned long)s_pair_skip[0],
               (unsigned long long)s_pair_sum[1], (unsigned long)s_pair_n[1], (unsigned long)s_pair_skip[1]);
        s_pair_obs = 0;
        memset(s_pair_n, 0, sizeof(s_pair_n)); memset(s_pair_skip, 0, sizeof(s_pair_skip)); memset(s_pair_sum, 0, sizeof(s_pair_sum));
        if (--s_pair_hi < 3u) { s_pair_active = false; printf("GPAIR done\n"); return s_current_gain; }
    }
    return (s_pair_obs / PAIR_SWITCH_OBS) & 1u ? (uint8_t)(s_pair_hi - 1u) : s_pair_hi;
}

/* The research AGC models (agc.h) on the KEEL-1 core's gain model and the
 * startup noise map (s_gain_th, below). Re-bound on a mode change, a
 * retune/scan/calibration (s_control_epoch) and a new gain table, so the
 * level history never spans two channels or two gain models. */
static union { agc_level_t level; agc_rssi_t rssi; } s_agc;   /* one model at a time */
static uint8_t s_agc_mode;
static uint32_t s_agc_control = UINT32_MAX;
static unsigned s_agc_rows = UINT_MAX;
/* Tentative declarations; defined with the noise-derived gain table below. */
static unsigned s_gain_rows_measured;
static float s_gain_th[DG_STATES];
#define AGC_NOISE_TOP_DEFAULT 4.8f   /* cells^2 at G83 when no noise map yet (Wi-Fi reference, FINDINGS) */

static uint8_t agc_decision(uint8_t mode, const dg_observation_t *o)
{
    const arc_gain_table_t *t = rf_gain_table();
    if (mode != s_agc_mode || s_agc_control != s_control_epoch || s_agc_rows != s_gain_rows_measured) {
        const float n = s_gain_th[t->max_index] > 0.0f ? s_gain_th[t->max_index] : AGC_NOISE_TOP_DEFAULT;
        if (mode == GAIN_MODE_LEVEL) {
            agc_level_reset(&s_agc.level, s_gain.model_db_q8, t->max_index, (uint16_t)(n * 16.0f));
            const unsigned mem = experimental_get(EXP_FOLD_MEMORY), target = experimental_get(EXP_LEVEL_TARGET);
            agc_level_options(&s_agc.level, experimental_get(EXP_FOLD_RADIUS), mem == 1 ? 300000u : mem == 2 ? 1000000u : 0,
                              target == 1 ? -3 : target == 2 ? 3 : 0, experimental_get(EXP_LEVEL_RELEASE));
        }
        else agc_rssi_reset(&s_agc.rssi, s_gain.model_db_q8, t->max_index);
        s_agc_mode = mode;
        s_agc_control = s_control_epoch;
        s_agc_rows = s_gain_rows_measured;
        printf("AGC model=%s bound top=G%u noise_top=%.2f cells2 rows_measured=%u\n",
               mode == GAIN_MODE_LEVEL ? "LEVEL" : "RSSI", t->max_index, (double)n, s_gain_rows_measured);
    }
    return mode == GAIN_MODE_LEVEL ? agc_level_tick(&s_agc.level, o, s_current_gain, o->observed_us)
                                   : agc_rssi_tick(&s_agc.rssi, o, s_current_gain, o->observed_us);
}

/* The carrier radius of a folded window on LEVEL (agc_level_fold_radius),
 * for LEVEL and the attack events; only folded windows pay for it. */
static uint32_t s_fold_taken, s_fold_rejected, s_fold_noisy;
static void fold_measure(dg_observation_t *o, const uint8_t *samples, unsigned n)
{
    if (rf_lane() != RF_LANE_ULTRAFINE || s_gain_mode != GAIN_MODE_LEVEL || s_agc_mode != GAIN_MODE_LEVEL || !agc_folded(o)) return;
    o->fold_r_x16 = agc_level_fold_radius(&s_agc.level, samples, n, s_current_gain, &o->fold_raw_x16, &o->fold_fit);
    if (o->fold_r_x16) ++s_fold_taken;
    else if (o->fold_raw_x16) ++s_fold_rejected;
    else ++s_fold_noisy;
}

/* Fixed modes bypass the controller; apply_target clamps to the live table. */
static uint8_t gain_decision(const dg_observation_t *o)
{
    if (s_pair_active) return pair_decision(o);
    const uint8_t mode = s_gain_mode;
    if (mode == GAIN_MODE_AUTO) return dg_tick(&s_gain, o);
    const uint8_t target = mode == GAIN_MODE_LEVEL || mode == GAIN_MODE_RSSI ? agc_decision(mode, o)
                         : mode == GAIN_MODE_MAX ? rf_gain_table()->max_index : mode;
    /* Once written and settled the model reads HOLD, so profile/DC/C-N gates
     * waiting for a held gain behave as with a steady carrier. */
    if (target == s_current_gain && s_gain.state != DG_HOLD &&
        o->observed_us > (uint64_t)s_last_gain_write_us && o->observed_us - (uint64_t)s_last_gain_write_us >= 2000u)
        s_gain.state = DG_HOLD;
    return target;
}
static volatile bool s_paused;
static volatile uint32_t s_overload_state;   /* 0 free, 1 writing, 2 published */
static dg_observation_t s_overload;
static uint32_t s_overload_gain_epoch, s_overload_control_epoch, s_overload_phy;
static uint32_t s_map_imports, s_preset_imports;
static bool s_map_bound;
static uint32_t s_map_control, s_map_phy, s_map_table;
extern unsigned char phy_param[];

static dg_map_context_t map_context(void)
{
    const rf_filter_t f = rf_filter();
    return (dg_map_context_t){.freq_mhz=rf_frequency_mhz(), .lane_mode=rf_lane(),
        .iq_scale=phy_param[650], .filter_skirt=f.skirt, .filter_bw20=f.bw20,
        .offset_khz=rf_offset_khz(), .filter_code=f.code};
}

/* No vendor calls or waits here. A nonblocking actuator lease rejects even
 * phy_try_actuator owners (which do not set phy_busy); short task exclusion
 * protects the observer's ~700-byte snapshot. Context stamping belongs to
 * that observer, never to a later calibration task. */
unsigned receiver_export_gain_map(dg_map_blob_t *blob)
{
    if (!blob) return 0;
    if (experimental_transfer_active()) { memset(blob, 0, sizeof(*blob)); return 0; }
    if (!phy_try_actuator(phy_generation())) { memset(blob, 0, sizeof(*blob)); return 0; }
    vTaskSuspendAll();
    const uint32_t control=s_control_epoch, phy=phy_generation(), table=rf_table_generation();
    const uint32_t gain_epoch=s_gain_epoch;
    const dg_map_context_t context=map_context();
    unsigned n=0;
    if (s_map_bound && receiver_active() && s_gain.state == DG_HOLD &&
        cn_gain_settled((uint64_t)esp_timer_get_time(), (uint64_t)s_last_gain_write_us, false) &&
        s_gain.current_gain == s_current_gain &&
        control == s_map_control && phy == s_map_phy && table == s_map_table &&
        dg_map_matches(&s_gain, rf_gain_table(), &context))
        n=dg_export_map(&s_gain, blob);
    const dg_map_context_t after=map_context();
    if (control != s_control_epoch || gain_epoch != s_gain_epoch || phy != phy_generation() || table != rf_table_generation() ||
        !dg_map_context_equal(&context, &after)) n=0;
    phy_end_actuator();
    (void)xTaskResumeAll();
    if (!n) memset(blob, 0, sizeof(*blob));
    return n;
}

static int64_t s_attack_prev_write_us;   /* the write before the attack's (attack_record) */
static uint32_t s_rebinds;
/* Evidence for every step down (FLOG_E_ATTACK, at most 4 a second): what
 * the gain core saw when it went down, at that moment, not a snapshot
 * milliseconds later (blackbox session 32: every overload snapshot was
 * clean). Bytes: source (0 observer, 1 sentinel), from, to, saturated run
 * (1) or folding (0), us since the last write / 10 (u16), P95, clip per
 * mille / 4, rails, P50; since 2026-10-28 the fold radius of that window
 * (LEVEL, agc_level_fold_radius): taken radius x16 (u16, 0 none), best
 * radius x16 before the fit check (u16, 0 not estimated), fit (s8). */
static void attack_record(const dg_observation_t *o, uint8_t source, uint8_t from, uint32_t severe_before)
{
    static int64_t window_us;
    static unsigned in_window;
    if (s_gain.writes_severe == severe_before || s_current_gain >= from) return;
    const int64_t now = esp_timer_get_time();
    if (now - window_us >= 1000000) { window_us = now; in_window = 0; }
    if (++in_window > 4u) return;
    const int64_t since = s_gain_epoch ? (int64_t)o->observed_us - s_attack_prev_write_us : 0;
    const uint32_t since10 = since < 0 ? 0u : since / 10 > 65535 ? 65535u : (uint32_t)(since / 10);
    const uint8_t d[15] = {source, from, s_current_gain, (uint8_t)(s_gain.saturated_run ? 1u : 0u),
                           (uint8_t)since10, (uint8_t)(since10 >> 8), o->p95, (uint8_t)(o->clip_pm / 4u > 255u ? 255u : o->clip_pm / 4u),
                           o->rails, o->p50, (uint8_t)o->fold_r_x16, (uint8_t)(o->fold_r_x16 >> 8),
                           (uint8_t)o->fold_raw_x16, (uint8_t)(o->fold_raw_x16 >> 8), (uint8_t)o->fold_fit};
    flightlog_event(FLOG_E_ATTACK, d, sizeof(d));
}
static void gain_map_bind(const dg_map_context_t *context, uint32_t control, uint32_t phy, uint32_t table)
{
    dg_reset_context(&s_gain, rf_gain_table(), s_current_gain, context);
    /* Every import checks the current transfer context and complete table,
     * including after A -> B -> A. A stored map for A remains available
     * while B runs but can never be relabelled/imported as B. */
    if (!s_gain.learned && s_gain_map_saved_valid && dg_import_map(&s_gain, &s_gain_map_saved)) ++s_map_imports;
    else if (!s_gain.learned && GAIN_MAP_PRESET_BOARDS &&
             dg_import_preset(&s_gain, c5vrx5_gain_map_preset_q10, GAIN_MAP_PRESET_MAX_INDEX + 1u,
                              GAIN_MAP_PRESET_MAX_INDEX, GAIN_MAP_PRESET_BAND5)) ++s_preset_imports;
    s_map_control=control; s_map_phy=phy; s_map_table=table; s_map_bound=true;
}
static volatile cn_observation_t s_cn;
static volatile uint32_t s_cn_sequence;
static uint32_t s_cn_windows;

bool receiver_active(void)
{
    return !s_paused && (!s_menu_active || menu_splash_active()) && !phy_busy();
}

bool receiver_paused(void) { return s_paused; }

bool receiver_pause(const char *tag)
{
    if (s_paused) {
        printf("%s refused=busy\n", tag);
        return false;
    }
    s_paused = true;
    vTaskDelay(pdMS_TO_TICKS(20));   /* let an in-flight window finish */
    return true;
}

void receiver_resume(void)
{
    ++s_control_epoch;
    s_paused = false;
}

/* Wall time of rf_set_gain() in the gain owner's writes (LCD build: T). */
static struct { uint32_t n, sum_us, max_us; } s_write_time;
void receiver_write_time(uint32_t *n, uint32_t *sum_us, uint32_t *max_us)
{
    *n = s_write_time.n; *sum_us = s_write_time.sum_us; *max_us = s_write_time.max_us;
    s_write_time.n = s_write_time.sum_us = s_write_time.max_us = 0;
}

/* Gain write by the owner of a pause (calibration, scan). */
void receiver_set_gain(uint8_t gain)
{
    const arc_gain_table_t *t = rf_gain_table();
    if (gain < 2u) gain = 2u;
    if (gain > t->max_index) gain = t->max_index;
    rf_force_gain(gain);
    s_current_gain = gain;
    s_last_gain_write_us = esp_timer_get_time();
    ++s_gain_epoch;
}

/* Why gain writes were refused (T: GAIN_REFUSED), counted only when the
 * core wanted another gain: 947 refusals in a 4-minute walk (session 32)
 * and no way to tell who held the PHY. */
static uint32_t s_refuse[6];   /* generation, busy, control, inactive, actuator lock, rf_set_gain */
static void apply_target(uint8_t target, uint32_t control, uint32_t phy)
{
    /* Every refused write puts the core back on the real gain: left ahead of
     * the hardware, the next probe saw a mismatch and reset the whole core
     * (blackbox session 29). */
    if (phy != phy_generation() || phy_busy() || control != s_control_epoch || !receiver_active()) {
        if (target != s_current_gain)
            ++s_refuse[phy != phy_generation() ? 0 : phy_busy() ? 1 : control != s_control_epoch ? 2 : 3];
        dg_write_refused(&s_gain, s_current_gain);
        return;
    }
    const arc_gain_table_t *t = rf_gain_table();
    if (target < 2u) target = 2u;
    if (target > t->max_index) target = t->max_index;
    const unsigned cap_mode = experimental_get(EXP_GAIN_CAP);
    const unsigned cap = cap_mode == 1 ? 79u : cap_mode == 2 ? 75u : t->max_index;
    if (target > cap) target = (uint8_t)cap;
    if (target == s_current_gain) { dg_write_refused(&s_gain, s_current_gain); return; }
    /* Rate limit release only: an overload must retain its emergency path. */
    const unsigned rate = experimental_get(EXP_GAIN_RATE);
    const int64_t min_us = rate == 1 ? 5000 : rate == 2 ? 20000 : 0;
    if (target > s_current_gain && min_us && esp_timer_get_time() - s_last_gain_write_us < min_us) {
        dg_write_refused(&s_gain, s_current_gain);
        return;
    }
    /* A task preemption between the first check and the write must not let
     * another PHY owner change the analog state underneath this write. */
    if (!phy_try_actuator_wait(phy, 1)) { ++s_refuse[4]; dg_write_refused(&s_gain, s_current_gain); return; }   /* <= 1 ms */
    if (control != s_control_epoch || !receiver_active()) {
        phy_end_actuator();
        ++s_refuse[control != s_control_epoch ? 2 : 3];
        dg_write_refused(&s_gain, s_current_gain);
        return;
    }
    /* A busy or changed PHY must not leave software ahead of hardware. */
    const int64_t write_t0 = esp_timer_get_time();
    const bool written = rf_set_gain(target, phy);
    const uint32_t write_us = (uint32_t)(esp_timer_get_time() - write_t0);
    ++s_write_time.n;
    s_write_time.sum_us += write_us;
    if (write_us > s_write_time.max_us) s_write_time.max_us = write_us;
    if (!written) {
        ++s_refuse[5];
        dg_sync_applied(&s_gain, s_current_gain, (uint64_t)esp_timer_get_time());
        phy_end_actuator();
        return;
    }
    /* Only the KEEL-1 core counts its own writes (start_write); LEVEL, RSSI,
     * fixed gains and the pair sweep write here without it, so the blackbox
     * and T read 0 writes while the gain moves (blackbox session 279, LEVEL:
     * G20..G83, gain_writes 0, no attack events). Counted as applied writes
     * by direction: down is the attack, up the release. */
    if (s_gain_mode != GAIN_MODE_AUTO || s_pair_active) {
        ++s_gain.writes;
        if (target < s_current_gain) ++s_gain.writes_severe; else ++s_gain.writes_moderate;
    }
    s_current_gain = target;
    s_last_gain_write_us = esp_timer_get_time();
    ++s_gain_epoch;
    dg_sync_applied(&s_gain, target, (uint64_t)s_last_gain_write_us);
    phy_end_actuator();
}

/* Received level at the pin (rx_level.h): probe power per gain row over one
 * second, rolled by the supervisor's 1 Hz step. */
static uint32_t s_lvl_sum[RX_LEVEL_ROWS], s_lvl_n[RX_LEVEL_ROWS], s_lvl_clipped;
static portMUX_TYPE s_lvl_lock = portMUX_INITIALIZER_UNLOCKED;
static rx_level_t s_level;
static float s_noise_rel;          /* learned noise, calibration units; 0 = not yet */
static uint16_t s_noise_freq;
static rf_filter_t s_noise_filter;   /* the noise belongs to this filter too (NARROW reads lower) */
static void level_accumulate(const dg_observation_t *o, uint8_t gain)
{
    if (gain >= RX_LEVEL_ROWS || s_obs_stuck) return;
    taskENTER_CRITICAL(&s_lvl_lock);
    if (o->clip_pm >= 20u) ++s_lvl_clipped;
    else { s_lvl_sum[gain] += o->pwr_x16; ++s_lvl_n[gain]; }
    taskEXIT_CRITICAL(&s_lvl_lock);
}
void receiver_level_roll(bool no_carrier, bool video)
{
    static uint32_t sum[RX_LEVEL_ROWS], n[RX_LEVEL_ROWS];
    uint32_t clipped;
    taskENTER_CRITICAL(&s_lvl_lock);
    memcpy(sum, s_lvl_sum, sizeof(sum)); memcpy(n, s_lvl_n, sizeof(n)); clipped = s_lvl_clipped;
    memset(s_lvl_sum, 0, sizeof(s_lvl_sum)); memset(s_lvl_n, 0, sizeof(s_lvl_n)); s_lvl_clipped = 0;
    taskEXIT_CRITICAL(&s_lvl_lock);
    const rf_filter_t f = rf_filter();
    if (s_noise_freq != rf_frequency_mhz() || f.code != s_noise_filter.code || f.skirt != s_noise_filter.skirt ||
        f.bw20 != s_noise_filter.bw20) {
        s_noise_freq = rf_frequency_mhz(); s_noise_filter = f; s_noise_rel = 0.0f;
    }
    rx_level_t r = rx_level_compute(sum, n, clipped, s_gain.model_db_q8, s_gain.table.max_index, s_noise_rel);
    /* Learn the noise with no carrier at the table maximum, inside -3..+4 dB
     * of the calibration (a carrier the supervisor missed stays out). */
    if (no_carrier && r.valid && r.gain_min == s_gain.table.max_index && r.rel > 0.5f && r.rel < 2.5f) {
        s_noise_rel = s_noise_rel > 0.0f ? s_noise_rel + (r.rel - s_noise_rel) / 8.0f : r.rel;
        r = rx_level_compute(sum, n, clipped, s_gain.model_db_q8, s_gain.table.max_index, s_noise_rel);
    }
    s_level = rx_level_gate(r, video);
}
rx_level_t receiver_level(void) { return s_level; }
bool receiver_noise_learned(void) { return s_noise_rel > 0.0f; }
/* The menu owns TX and the gain observer stands still, so the menu feeds
 * its control windows here instead (control.c), in 256-byte probes like
 * the observer's, and rolls once a second. */
void receiver_level_feed(const uint8_t *window, size_t bytes)
{
    for (size_t at = 0; at + 256u <= bytes; at += 256u) {
        const dg_observation_t o = dg_measure_gain(window + at, 256u, c5vrx_phase8_gain_lut, 0);
        level_accumulate(&o, s_current_gain);
    }
}

/* ---- C/N-AUTO (cn_auto.h) -------------------------------------------------
 * The C/N every consumer reads (AFC, profile gates, flight
 * log), self-calibrating and with a confidence:
 * - noise per gain row for this channel and filter: measured at startup
 *   with no carrier (calibration_dc.c), learned in service only on no-carrier
 *   evidence (envelope kurtosis + level + no video for 2 s), kept in NVS;
 * - power around each row's long-term DC (not the window mean);
 * - M2M4 and envelope (cn_estimate.h) as diagnostics only. Actuators use
 *   only input-referred power against a per-context thermal-noise map: an
 *   uncalibrated envelope can turn multipath amplitude changes into fake C/N.
 * Every 4th settled probe feeds it (~1.3 kHz, ~2 % CPU). */
#define CNA_PROBE_DIV      4u
#define CNA_EST_PROBES     16u      /* probes per estimate (~13 ms) */
#define CNA_ABSENT_PROBES  256u     /* probes per no-carrier decision (~0.2 s) */
#define CNA_STORE_SLOTS    6u
#define CNA_STORE_LO       54u      /* rows kept in NVS: 54..83 */
#define CNA_STORE_ROWS     30u
typedef struct {
    uint8_t version;
    struct {
        uint16_t freq; int8_t code; uint8_t skirt, bw20, seq;
        uint16_t noise_x256[CNA_STORE_ROWS];
        uint16_t anchor_x256[CNA_STORE_ROWS];   /* the clean startup value (cn_auto.h anchor), 0 = none */
    } slot[CNA_STORE_SLOTS];
} cna_store_t;
/* 2: the anchors are stored too; a map loaded for another channel or filter
 * kept learning without one (operator, 2026-10-20). Version 1 maps are
 * dropped and measured again at the next clean boot. */
/* 3: digital RX mode 0, 4: 11p, 5: digital RX mode 12; old noise/anchor
 * powers are another RF context. */
#define CNA_STORE_VERSION 5u
static cna_store_t s_cna_store;
static bool s_cna_store_loaded;
static cna_noise_map_t s_cna_map;
static uint16_t s_cna_freq;
static rf_filter_t s_cna_filter;
static bool s_cna_key_valid, s_cna_dirty;
static float s_rowdc_i[CNA_ROWS], s_rowdc_q[CNA_ROWS];
static uint16_t s_rowdc_n[CNA_ROWS];
static cna_sums_t s_cna_acc;
static uint8_t s_cna_acc_row = 255u;
static uint32_t s_cna_acc_probes, s_cna_acc_clip, s_cna_div;
static double s_abs_m2, s_abs_m4;
static uint32_t s_abs_n;
static uint8_t s_abs_row = 255u;
static cna_reading_t s_cna;
static volatile int64_t s_cna_us;
static volatile uint32_t s_cna_epoch;
static uint32_t s_cna_learned, s_cna_absent_rejects, s_cna_estimates;
static volatile bool s_cna_gain_dirty;

static int cna_slot_find(uint16_t freq, rf_filter_t f)
{
    for (unsigned k = 0; k < CNA_STORE_SLOTS; ++k)
        if (s_cna_store.slot[k].freq == freq && s_cna_store.slot[k].code == f.code &&
            s_cna_store.slot[k].skirt == f.skirt && s_cna_store.slot[k].bw20 == (uint8_t)f.bw20) return (int)k;
    return -1;
}
/* Keep this context's measured rows (54..83) in NVS. */
static void cna_save(void)
{
    if (experimental_transfer_active()) return;
    if (!s_cna_key_valid || !s_cna_dirty) return;
    int k = cna_slot_find(s_cna_freq, s_cna_filter);
    if (k < 0) {   /* a free or the oldest slot */
        unsigned best = 0;
        for (unsigned j = 0; j < CNA_STORE_SLOTS; ++j)
            if (!s_cna_store.slot[j].freq) { best = j; break; }
            else if ((uint8_t)(s_cna_store.slot[j].seq - s_cna_store.slot[best].seq) > 127u) best = j;
        k = (int)best;
    }
    uint8_t top = 0;
    for (unsigned j = 0; j < CNA_STORE_SLOTS; ++j) if ((uint8_t)(s_cna_store.slot[j].seq - top) < 128u) top = s_cna_store.slot[j].seq;
    s_cna_store.version = CNA_STORE_VERSION;
    s_cna_store.slot[k].freq = s_cna_freq; s_cna_store.slot[k].code = s_cna_filter.code;
    s_cna_store.slot[k].skirt = s_cna_filter.skirt; s_cna_store.slot[k].bw20 = (uint8_t)s_cna_filter.bw20;
    s_cna_store.slot[k].seq = (uint8_t)(top + 1u);
    for (unsigned r = 0; r < CNA_STORE_ROWS; ++r) {
        const unsigned row = CNA_STORE_LO + r;
        const float v = s_cna_map.n[row] >= CNA_MIN_SAMPLES ? s_cna_map.noise[row] * 256.0f : 0.0f;
        s_cna_store.slot[k].noise_x256[r] = (uint16_t)(v > 65535.0f ? 65535.0f : v);
        const float a = s_cna_map.anchor[row] * 256.0f;
        s_cna_store.slot[k].anchor_x256[r] = (uint16_t)(a > 65535.0f ? 65535.0f : a);
    }
    if (blob_store("cnmap", &s_cna_store, sizeof(s_cna_store))) s_cna_dirty = false;
}
/* A new channel or filter: save the old map, load the new one. */
static void cna_key_check(void)
{
    const rf_filter_t f = rf_filter();
    const uint16_t freq = rf_frequency_mhz();
    if (s_cna_key_valid && freq == s_cna_freq && f.code == s_cna_filter.code && f.skirt == s_cna_filter.skirt &&
        f.bw20 == s_cna_filter.bw20) return;
    cna_save();
    if (!s_cna_store_loaded) {
        if (!blob_load("cnmap", &s_cna_store, sizeof(s_cna_store)) || s_cna_store.version != CNA_STORE_VERSION)
            memset(&s_cna_store, 0, sizeof(s_cna_store));
        s_cna_store_loaded = true;
    }
    memset(&s_cna_map, 0, sizeof(s_cna_map));
    memset(s_rowdc_n, 0, sizeof(s_rowdc_n));
    s_cna_freq = freq; s_cna_filter = f; s_cna_key_valid = true; s_cna_dirty = false;
    s_cna_acc_row = s_abs_row = 255u;
    s_cna_gain_dirty = true;
    const int k = cna_slot_find(freq, f);
    if (k >= 0)
        for (unsigned r = 0; r < CNA_STORE_ROWS; ++r)
            if (s_cna_store.slot[k].noise_x256[r]) {
                s_cna_map.noise[CNA_STORE_LO + r] = s_cna_store.slot[k].noise_x256[r] / 256.0f;
                s_cna_map.n[CNA_STORE_LO + r] = CNA_MIN_SAMPLES;   /* loaded: measured on an earlier boot */
                s_cna_map.anchor[CNA_STORE_LO + r] = s_cna_store.slot[k].anchor_x256[r] / 256.0f;
            }
}
/* Read-only export: never cna_key_check() here, since that can save/load
 * maps. Return unknown when the current map belongs to another context. */
bool receiver_export_noise(uint8_t row, float *cells2, uint16_t *samples, int64_t *measured_us)
{
    if (experimental_transfer_active() || row >= CNA_ROWS || !s_cna_key_valid || s_cna_freq != rf_frequency_mhz() ||
        !rf_filter_equal(s_cna_filter, rf_filter())) return false;
    vTaskSuspendAll();
    *cells2 = s_cna_map.noise[row];
    *samples = s_cna_map.n[row];
    *measured_us = s_cna_map.us[row];
    (void)xTaskResumeAll();
    return *samples >= CNA_MIN_SAMPLES && *cells2 > 0;
}

void receiver_cn_noise_store(uint8_t row, float cells2)
{
    cna_key_check();
    if (row >= CNA_ROWS || !(cells2 > 0.0f)) return;
    s_cna_map.anchor[row] = 0.0f;   /* the startup value replaces the anchor, then anchors */
    for (unsigned k = 0; k < CNA_MIN_SAMPLES; ++k) cna_noise_learn(&s_cna_map, row, cells2, esp_timer_get_time());
    cna_noise_anchor(&s_cna_map, row, cells2);
    s_cna_dirty = true;
}
void receiver_cn_noise_save(void) { cna_save(); s_cna_gain_dirty = true; }

/* The noise map is a gain measurement too (receiver noise is a constant
 * input): rows whose thermal noise is well above the 4-bit step (>= 0.5
 * cells^2) give the gain core and the dBm meter their real gain
 * (dg_apply_measured_gain). Applied by the observer task, the core's only
 * writer, after every map change and every re-bind. */
/* The gain table comes from the startup noise map only (no carrier,
 * kurtosis-checked, calibration_dc.c): the gain per row does not depend on the
 * channel or the analog filter, so it is kept for the whole boot and stored
 * ("gtab") for a boot with the VTX on. The live map is not used: it follows
 * the channel and filter (measured_rows=0 after a profile switch, board
 * 2026-10-20) and its in-service learning took in a weak carrier (G83-G72
 * 9.1 dB against 10.8 measured at boot). */
#define GTAB_VERSION 5u   /* 4: 11p, 5: digital RX mode 12; reject old noise-derived gains */
typedef struct { uint8_t version, rows; uint16_t th_x256[DG_STATES]; } gtab_blob_t;
static unsigned s_gain_rows_measured;
static float s_gain_th[DG_STATES];
static bool s_gain_th_loaded;
static float s_gain_th_staged[DG_STATES];   /* calibration task -> observer (the core's only writer) */
static volatile bool s_gain_th_new;
static void cna_apply_gain(void)
{
    if (!s_gain_th_loaded) {
        s_gain_th_loaded = true;
        gtab_blob_t b;
        if (blob_load("gtab", &b, sizeof(b)) && b.version == GTAB_VERSION && b.rows >= 2u)
            for (unsigned k = 0; k < DG_STATES; ++k) s_gain_th[k] = b.th_x256[k] / 256.0f;
    }
    if (s_gain_th_new) {
        __sync_synchronize();
        memcpy(s_gain_th, s_gain_th_staged, sizeof(s_gain_th));
        s_gain_th_new = false;
    }
    s_gain_rows_measured = dg_apply_measured_gain(&s_gain, s_gain_th);
}

void receiver_gain_table_measured(const float noise_cells2[DG_STATES])
{
    float th[DG_STATES] = {0};
    unsigned rows = 0;
    for (unsigned k = 0; k < DG_STATES; ++k) {
        const float t = noise_cells2[k] - CNA_QUANT_CELLS2;
        th[k] = noise_cells2[k] > 0.0f && t >= 0.5f ? t : 0.0f;
        rows += th[k] > 0.0f;
    }
    if (rows < 2u) return;
    memcpy(s_gain_th_staged, th, sizeof(th));
    __sync_synchronize();
    s_gain_th_new = true;
    gtab_blob_t b = {.version = GTAB_VERSION, .rows = (uint8_t)rows};
    for (unsigned k = 0; k < DG_STATES; ++k) b.th_x256[k] = (uint16_t)(th[k] * 256.0f + 0.5f);
    gtab_blob_t old;
    if (!blob_load("gtab", &old, sizeof(old)) || memcmp(&old, &b, sizeof(b))) (void)blob_store("gtab", &b, sizeof(b));
    s_cna_gain_dirty = true;
}

static void cna_probe(const uint8_t *sample, size_t bytes, const dg_observation_t *o, uint8_t row, int64_t now)
{
    if (experimental_transfer_active()) return;
    if (row >= CNA_ROWS || ++s_cna_div % CNA_PROBE_DIV) return;
    cna_key_check();
    const bool folded = (o->rails & 3u) == 3u || (o->rails & 12u) == 12u;
    cna_sums_t ps = {0};
    for (size_t k = 0; k < bytes; ++k) cna_sums_add(&ps, sample[k]);
    float pi, pq;
    cna_mean(&ps, &pi, &pq);
    /* The row's long-term DC: probe means of unclipped probes, slowly. */
    if (!folded && o->clip_pm < 20u) {
        if (!s_rowdc_n[row]) { s_rowdc_i[row] = pi; s_rowdc_q[row] = pq; }
        else { s_rowdc_i[row] += (pi - s_rowdc_i[row]) / 32.0f; s_rowdc_q[row] += (pq - s_rowdc_q[row]) / 32.0f; }
        if (s_rowdc_n[row] < 65535u) ++s_rowdc_n[row];
    }
    /* No-carrier evidence at this row. */
    if (s_abs_row != row) { s_abs_row = row; s_abs_m2 = s_abs_m4 = 0.0; s_abs_n = 0; }
    {
        double m2, m4;
        cna_moments(&ps, pi, pq, &m2, &m4);
        s_abs_m2 += m2; s_abs_m4 += m4; ++s_abs_n;
    }
    if (s_abs_n >= CNA_ABSENT_PROBES) {
        const float known = s_cna_map.n[row] >= CNA_MIN_SAMPLES ? s_cna_map.noise[row] : 0.0f;
        const int64_t video = s_last_video_us;
        const bool quiet_video = !video || now - video > 2000000;
        if (quiet_video && cna_carrier_absent(s_abs_m2, s_abs_m4, s_abs_n, known)) {
            cna_noise_learn(&s_cna_map, row, (float)(s_abs_m2 / s_abs_n), now);
            ++s_cna_learned;
            s_cna_dirty = true;
        } else if (quiet_video) {
            ++s_cna_absent_rejects;
        }
        s_abs_m2 = s_abs_m4 = 0.0; s_abs_n = 0;
    }
    /* The estimate. */
    if (s_cna_acc_row != row) { s_cna_acc_row = row; memset(&s_cna_acc, 0, sizeof(s_cna_acc)); s_cna_acc_probes = s_cna_acc_clip = 0; }
    cna_sums_merge(&s_cna_acc, &ps);
    ++s_cna_acc_probes;
    s_cna_acc_clip += folded || o->clip_pm >= 20u;
    if (s_cna_acc_probes < CNA_EST_PROBES) return;
    uint8_t flags = 0;
    if (s_cna_acc_clip * 50u > s_cna_acc_probes) flags |= CNA_F_CLIPPED;     /* > 2 % of the probes */
    float di = 0.0f, dq = 0.0f;
    if (s_rowdc_n[row] >= 32u) { di = s_rowdc_i[row]; dq = s_rowdc_q[row]; }
    else { cna_mean(&s_cna_acc, &di, &dq); flags |= CNA_F_DC_UNKNOWN; }
    double m2, m4;
    cna_moments(&s_cna_acc, di, dq, &m2, &m4);
    float uncert;
    const float noise = cna_noise_at(&s_cna_map, row, s_gain.model_db_q8, now, &flags, &uncert);
    const int cls = calibration_filter_class_mhz();
    const int env = s_obs_env_ratio_x100 > 0 ? cn_estimate_env_x10(s_obs_env_ratio_x100, cls) : CNA_UNKNOWN_X10;
    s_cna = cna_estimate(m2, m4, noise, uncert, flags, calibration_filter_nbw_khz(), cls, env);
    s_cna_us = now;
    s_cna_epoch = s_gain_epoch;
    ++s_cna_estimates;
    s_cna_acc_row = 255u;   /* the next window starts fresh */
}
cna_reading_t receiver_cn_reading(void)
{
    const int64_t now = esp_timer_get_time();
    cna_reading_t r = s_cna;
    if (experimental_transfer_active() || s_cna_epoch != s_gain_epoch || now - s_cna_us > 60000 || !receiver_active()) {
        r.x10 = CNA_UNKNOWN_X10; r.confidence = CNA_CONF_NONE; r.source = CNA_SRC_NONE;
    }
    return r;
}

int receiver_obs_cn_x10(void)
{
    /* One actuator scale: power over the thermal-noise reference for this
     * channel/filter/gain context. Switching between power and envelope on
     * the 2026-10-20 walk jumped 5-9 dB; issue #222 also gives a noiseless
     * amplitude sequence that the envelope table calls ~3 dB C/N. Without a
     * noise reference, hold the last safe policy instead of inventing RF C/N.
     * M2M4/envelope remain visible in CN_AUTO telemetry. */
    if (experimental_transfer_active()) return CN_UNKNOWN_X10;
    const cna_reading_t r = receiver_cn_reading();
    const int x10 = cna_control_x10(&r);
    if (x10 == CNA_UNKNOWN_X10 || s_obs_stuck || !receiver_active() ||
        !cn_gain_settled((uint64_t)esp_timer_get_time(), (uint64_t)s_last_gain_write_us,
                         s_gain.state == DG_SETTLE)) return CN_UNKNOWN_X10;
    return x10;
}

/* DC of every settled observer probe, one sum per gain row, for the live
 * DC servo (dc_servo.h): the AGC already sums I and Q per probe. Per row,
 * because Direct Gain moves between rows faster than one row collects a
 * mean; short visits add up. A channel, PHY or table change restarts all
 * rows; the servo restarts a row whose codes it changed. The observer is
 * the only writer. */
typedef struct { int64_t si, sq; uint32_t ni, nq, samples; } dc_row_acc_t;
static uint32_t s_dc_rejected_folded;   /* windows kept out of the DC means (both rails) */
static dc_row_acc_t s_dc_rows[ARC_VENDOR_GAIN_MAX + 1u];
static uint32_t s_dc_rows_control, s_dc_rows_phy, s_dc_rows_table;
static portMUX_TYPE s_dc_acc_lock = portMUX_INITIALIZER_UNLOCKED;
static void dc_accumulate(const dg_observation_t *o, size_t bytes, uint8_t gain, uint32_t control, uint32_t phy,
                          uint32_t table)
{
    if (gain > ARC_VENDOR_GAIN_MAX) return;
    /* A window that reaches both rails of an axis holds a carrier folding
     * through the 4-bit window: its mean is no DC. Blackbox session 24 (VTX
     * close, a hand on the antenna): such windows at G79/G83, as the carrier
     * arrived or left before the gain moved, walked those rows' trims by up
     * to 88 codes; the rows then sat at 2-3 cells of DC (IQ snapshots) and
     * the picture was gone for 19 s. One rail per axis is the DC itself and
     * still counts. */
    if ((o->rails & 3u) == 3u || (o->rails & 12u) == 12u) { ++s_dc_rejected_folded; return; }
    taskENTER_CRITICAL(&s_dc_acc_lock);
    if (s_dc_rows_control != control || s_dc_rows_phy != phy || s_dc_rows_table != table) {
        memset(s_dc_rows, 0, sizeof(s_dc_rows));
        s_dc_rows_control = control; s_dc_rows_phy = phy; s_dc_rows_table = table;
    }
    s_dc_rows[gain].si += o->sum_i;
    s_dc_rows[gain].sq += o->sum_q;
    s_dc_rows[gain].ni += o->neg_i;
    s_dc_rows[gain].nq += o->neg_q;
    s_dc_rows[gain].samples += (uint32_t)bytes;
    taskEXIT_CRITICAL(&s_dc_acc_lock);
}

bool receiver_dc_take_row(uint8_t gain, uint32_t min_samples, int64_t *si, int64_t *sq, uint32_t neg[2],
                          uint32_t *samples)
{
    if (gain > ARC_VENDOR_GAIN_MAX) return false;
    taskENTER_CRITICAL(&s_dc_acc_lock);
    const bool enough = s_dc_rows[gain].samples >= min_samples;
    if (enough) {
        *si = s_dc_rows[gain].si; *sq = s_dc_rows[gain].sq; *samples = s_dc_rows[gain].samples;
        neg[0] = s_dc_rows[gain].ni; neg[1] = s_dc_rows[gain].nq;
        s_dc_rows[gain] = (dc_row_acc_t){0};       /* the next probe starts a fresh mean */
    }
    taskEXIT_CRITICAL(&s_dc_acc_lock);
    return enough;
}

bool receiver_dc_peek_row(uint8_t gain, uint32_t neg[2], uint32_t *samples)
{
    if (gain > ARC_VENDOR_GAIN_MAX) return false;
    taskENTER_CRITICAL(&s_dc_acc_lock);
    *samples = s_dc_rows[gain].samples;
    neg[0] = s_dc_rows[gain].ni; neg[1] = s_dc_rows[gain].nq;
    taskEXIT_CRITICAL(&s_dc_acc_lock);
    return *samples != 0;
}

void receiver_dc_reset_row(uint8_t gain)
{
    if (gain > ARC_VENDOR_GAIN_MAX) return;
    taskENTER_CRITICAL(&s_dc_acc_lock);
    s_dc_rows[gain] = (dc_row_acc_t){0};
    taskEXIT_CRITICAL(&s_dc_acc_lock);
}

void receiver_dc_reset_all(void)
{
    taskENTER_CRITICAL(&s_dc_acc_lock);
    memset(s_dc_rows, 0, sizeof(s_dc_rows));
    taskEXIT_CRITICAL(&s_dc_acc_lock);
}

static uint32_t s_sat_one_sided, s_sat_two_sided, s_sat_score;

/* Gain observer and sentinel period. LCD build: the software frame grabber
 * (lcd.c) needs about half the CPU. At 200 us the observer, sentinel and
 * timer took ~70 % of it and left the grabber 3 %; at 1 ms the observer
 * alone still took 31 % (board, 2026-10-30). 2 ms keeps a gain decision
 * every 2 ms; the gain core's holds and climbs are timed, not counted. */
#ifdef C5VRX_LCD
#define GAIN_TICK_US 2000
#else
#define GAIN_TICK_US 200
#endif

static void tick_cb(void *arg)
{
    (void)arg;
    if (s_sentinel) xTaskNotifyGive(s_sentinel);
    if (s_observer) xTaskNotifyGive(s_observer);
}

/* 64 bytes from the middle of the newest completed descriptor every 200 us:
 * a clipped envelope requests the observer's emergency path at once. */
static uint32_t s_sentinel_forwarded, s_sentinel_dropped;
uint32_t receiver_sentinel_forwarded(void) { return s_sentinel_forwarded; }
static void sentinel_task(void *arg)
{
    (void)arg;
    uint8_t sample[64];
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (experimental_get(EXP_FAST_GAIN) || !receiver_active() || s_overload_state || s_rx_dma_ch < 0 || s_rx_node_count < 2) continue;
        uint32_t gain_epoch = s_gain_epoch, control = s_control_epoch, phy = phy_generation();
        if (!copy_sentinel_window(sample) || gain_epoch != s_gain_epoch) continue;
        /* A gain write's transient is no overload (blackbox session 32). */
        if (esp_timer_get_time() - s_last_gain_write_us < 2000) continue;
        dg_observation_t o = dg_measure_gain(sample, sizeof(sample), c5vrx_phase8_gain_lut,
                                        (uint64_t)esp_timer_get_time());
        /* Only gross saturation: both rails of an axis in half the window.
         * A 64-sample window (1.6 us) of a weak carrier touches the rails on
         * noise peaks; forwarding that as an overload made the gain leave a
         * row and climb back, 255 writes a second walking out to the edge
         * (blackbox session 28; the 200 us observer and the 1 ms saturation
         * rule of the gain core see the real overloads). */
        const bool two_sided = (o.rails & 3u) == 3u || (o.rails & 12u) == 12u;
        if (!two_sided || o.clip_pm < 500u) { if (o.clip_pm >= 125u || o.p95 >= 95u) ++s_sentinel_dropped; continue; }
        if (control != s_control_epoch || phy != phy_generation() || phy_busy()) continue;
        ++s_sentinel_forwarded;
        if (!__sync_bool_compare_and_swap(&s_overload_state, 0u, 1u)) continue;
        fold_measure(&o, sample, sizeof(sample));
        s_overload = o;
        s_overload_gain_epoch = gain_epoch;
        s_overload_control_epoch = control;
        s_overload_phy = phy;
        __sync_synchronize();
        __sync_lock_test_and_set(&s_overload_state, 2u);
        if (s_observer) xTaskNotifyGive(s_observer);
    }
}

static void observer_task(void *arg)
{
    (void)arg;
    uint8_t sample[PROBE_BYTES];
    uint32_t seen_control = UINT32_MAX, seen_table = UINT32_MAX, seen_phy = UINT32_MAX;
    bool was_active = false;
    int last_node = -1;
    int64_t runnable_since = esp_timer_get_time();
    for (;;) {
        int64_t before_wait = esp_timer_get_time();
        if (before_wait - runnable_since >= 800) {
            /* taskYIELD cannot let priority-0 IDLE run. A real block can.
             * Do not disable/feed its watchdog to hide CPU saturation. */
            vTaskDelay(1);
            before_wait = runnable_since = esp_timer_get_time();
        }
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(GAIN_TICK_US > 1000 ? GAIN_TICK_US / 1000 : 1));
        /* IRQ time can make an already-pending notification look like a
         * blocked wait. Only the explicit delay above resets our runnable
         * budget, so periodic sync IRQs cannot suppress the IDLE opportunity. */
        if (!receiver_active()) {
            was_active = false;
            if (__sync_bool_compare_and_swap(&s_overload_state, 2u, 1u))
                __sync_lock_release(&s_overload_state);
            continue;
        }
        uint32_t control = s_control_epoch, table = rf_table_generation(), phy = phy_generation();
        const dg_map_context_t context = map_context();
        if (s_paused) continue;
        if (!was_active || control != seen_control || table != seen_table || phy != seen_phy ||
            s_gain.current_gain != s_current_gain || !dg_map_matches(&s_gain, rf_gain_table(), &context)) {
            ++s_rebinds;
            gain_map_bind(&context, control, phy, table);
            s_cna_gain_dirty = true;   /* the bind rebuilt the model gains */
            seen_control = control;
            seen_table = table;
            seen_phy = phy;
            was_active = true;
        }
        if (s_cna_gain_dirty) { s_cna_gain_dirty = false; cna_key_check(); cna_apply_gain(); }
        if (__sync_bool_compare_and_swap(&s_overload_state, 2u, 1u)) {
            __sync_synchronize();
            dg_observation_t o = s_overload;
            bool current = s_overload_gain_epoch == s_gain_epoch &&
                           s_overload_control_epoch == control && s_overload_phy == phy &&
                           (uint64_t)esp_timer_get_time() - o.observed_us <= 1000u;
            __sync_lock_release(&s_overload_state);
            if (current) {
                const uint32_t severe = s_gain.writes_severe;
                const uint8_t from = s_current_gain;
                s_attack_prev_write_us = s_last_gain_write_us;
                apply_target(gain_decision(&o), control, phy);
                attack_record(&o, 1u, from, severe);
            }
            continue;
        }
        uint32_t gain_epoch = s_gain_epoch;
        int node = -1;
        if (!copy_probe_window(sample, &node) || gain_epoch != s_gain_epoch) continue;
        /* Never the same descriptor twice: settling counts fresh windows. */
        if (node == last_node) continue;
        last_node = node;
        const int64_t t0 = esp_timer_get_time();
        cn_observation_t previous_cn = s_cn; /* sole writer: observer */
        bool measure_cn = cn_gain_settled((uint64_t)t0, (uint64_t)s_last_gain_write_us,
                                         s_gain.state == DG_SETTLE) &&
                         (!cn_observation_fresh(&previous_cn, (uint64_t)t0, gain_epoch, control, phy, table) ||
                          (uint64_t)t0 - previous_cn.us >= CN_OBSERVE_PERIOD_US);
        dg_observation_t o = measure_cn ? dg_measure(sample, sizeof(sample), c5vrx_phase8_gain_lut, (uint64_t)t0) :
                                         dg_measure_gain(sample, sizeof(sample), c5vrx_phase8_gain_lut, (uint64_t)t0);
        fold_measure(&o, sample, sizeof(sample));
        s_obs_p50 = o.p50;
        s_obs_p95 = o.p95;
        s_obs_origin_pm = o.origin_pm;
        s_obs_clip_pm = o.clip_pm;
        s_obs_coherence = o.coherence;
        s_obs_stuck = dg_stuck(&o);
        /* Saturated windows by the rails they reach: both rails of an axis
         * is a large signal; one side only is the cloud pushed against a rail
         * by DC (diagnostic, T GAIN sat=). */
        if (o.clip_pm >= 100u || o.p95 >= 95u) {
            const bool both = (o.rails & 3u) == 3u || (o.rails & 12u) == 12u;
            if (both) ++s_sat_two_sided; else ++s_sat_one_sided;
            if (o.clip_pm >= 100u && dg_one_sided(&o)) calibration_dc_urgent();   /* real clipping, not a touch */
            /* A leaky count, not a run: with oscillating gain the saturated
             * windows alternate and a run of 50 never came (blackbox
             * session 13: 7549 overloads, no snapshot). */
            if (both && (s_sat_score += 16u) > 400u) { s_sat_score = 0; flightlog_snapshot_request(FLOG_SNAP_OVERLOAD); }
        } else {
            s_sat_score -= s_sat_score >> 5;
        }
        const dg_map_context_t after_context = map_context();
        if (phy != phy_generation() || phy_busy() || gain_epoch != s_gain_epoch || control != s_control_epoch ||
            table != rf_table_generation() || !dg_map_context_equal(&context, &after_context)) continue;
        if (cn_gain_settled((uint64_t)t0, (uint64_t)s_last_gain_write_us, s_gain.state == DG_SETTLE)) {
            dc_accumulate(&o, sizeof(sample), s_current_gain, control, phy, table);
            level_accumulate(&o, s_current_gain);
            cna_probe(sample, sizeof(sample), &o, s_current_gain, t0);
        }
        if (measure_cn) {
            ++s_cn_sequence;
            __sync_synchronize();
            s_cn = (cn_observation_t){gain_epoch, control, phy, table, (uint64_t)t0, o.env_ratio_x100};
            s_obs_env_ratio_x100 = o.env_ratio_x100;
            __sync_synchronize();
            ++s_cn_sequence;
            ++s_cn_windows;
        }
        /* Learn the gain map only from a real VTX: burst-confirmed sync seen
         * by the AFC within the last 500 ms (never from receiver noise). */
        const int64_t video = s_last_video_us;
        s_gain.learn_enabled = !experimental_transfer_active() && video != 0 && t0 - video < 500000;
        {
            const uint32_t severe = s_gain.writes_severe;
            const uint8_t from = s_current_gain;
            s_attack_prev_write_us = s_last_gain_write_us;
            apply_target(gain_decision(&o), control, phy);
            attack_record(&o, 0u, from, severe);
        }
        uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
        if (us > s_obs_us_max) s_obs_us_max = us;
        ++s_obs_windows;
    }
}

void receiver_start(void)
{
    s_gain_map_saved_valid = blob_load("gain_map", &s_gain_map_saved, sizeof(s_gain_map_saved)) &&
                             s_gain_map_saved.version == DG_MAP_VERSION && s_gain_map_saved.context_valid;
    ESP_ERROR_CHECK(xTaskCreate(observer_task, "gain_obs", 4096, NULL, 3, &s_observer) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(sentinel_task, "gain_fast", 3072, NULL, 4, &s_sentinel) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    /* Notifications request a fresh snapshot, not replay of missed samples. */
    const esp_timer_create_args_t args = {.callback = tick_cb, .dispatch_method = ESP_TIMER_TASK,
        .name = "gain_tick", .skip_unhandled_events = true};
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, GAIN_TICK_US));
}

void receiver_print(void)
{
    if (s_gain_mode == GAIN_MODE_LEVEL)
        printf("AGC model=LEVEL decisions=%lu moves=%lu folds=%lu learned=%lu jumps=%lu fold_radius=%lu/%lu/%lu "
               "(agc.c; rows corrected from the level before/after a move; jumps: the fast history took over; "
               "fold_radius taken/rejected/row too noisy, logged in attack events, used by LEVEL=%u)\n",
               (unsigned long)s_agc.level.decisions, (unsigned long)s_agc.level.moves, (unsigned long)s_agc.level.folds,
               (unsigned long)s_agc.level.learned, (unsigned long)s_agc.level.jumps, (unsigned long)s_fold_taken,
               (unsigned long)s_fold_rejected, (unsigned long)s_fold_noisy, (unsigned)agc_level_fold_radius_used());
    else if (s_gain_mode == GAIN_MODE_RSSI)
        printf("AGC model=RSSI moves=%lu folds=%lu (agc.c)\n", (unsigned long)s_agc.rssi.moves,
               (unsigned long)s_agc.rssi.folds);
    const rx_level_t lv = s_level;
    if (lv.valid)
        printf("RX_LEVEL carrier_dbm=%.1f input_dbm=%.1f noise_dbm=%.1f noise_learned=%u cn_db=%.1f above_noise=%u exact=%u gains=%u-%u probes=%lu clipped_pm=%u "
               "(at the chip pin; calibrated against the Wi-Fi PHY RSSI: +-1 dB at G54-G83, a few dB below G54; "
               "free space for 25 mW, 2 dBi each side, 10 m at 5.8 GHz is ~-50 dBm)\n",
               lv.carrier_dbm_x10 / 10.0, lv.input_dbm_x10 / 10.0, lv.noise_dbm_x10 / 10.0, (unsigned)(s_noise_rel > 0.0f),
               lv.cn_x10 / 10.0,
               (unsigned)lv.above_noise, (unsigned)lv.exact, lv.gain_min, lv.gain_max, (unsigned long)lv.probes,
               lv.clipped_pm);
    else
        printf("RX_LEVEL unknown probes=%lu clipped_pm=%u\n", (unsigned long)lv.probes, lv.clipped_pm);
    printf("GAIN_SAT one_sided=%lu two_sided=%lu to_dc_servo=%lu (saturated observer windows: one rail per axis = DC-pushed, both = large signal; one-sided ones go to the DC servo, not the gain)\n",
           (unsigned long)s_sat_one_sided, (unsigned long)s_sat_two_sided, (unsigned long)s_gain.one_sided_clips);
    printf("GAIN_WRITES attack=%lu release=%lu refused=%lu rebinds=%lu overloads=%lu overloads_after_write=%lu heat_events=%lu sentinel_forwarded=%lu sentinel_dropped=%lu (why the gain moved; rebinds keep the core's memory on the same receiver; sentinel = 64-sample fast path, gross saturation only)\n",
           (unsigned long)s_gain.writes_severe, (unsigned long)s_gain.writes_moderate, (unsigned long)s_gain.writes_refused,
           (unsigned long)s_rebinds, (unsigned long)s_gain.overloads, (unsigned long)s_gain.overloads_after_write, (unsigned long)s_gain.heat_events,
           (unsigned long)s_sentinel_forwarded, (unsigned long)s_sentinel_dropped);
    {
        const cna_reading_t c = receiver_cn_reading();
        unsigned measured = 0;
        for (unsigned k = 0; k < CNA_ROWS; ++k) measured += s_cna_map.n[k] >= CNA_MIN_SAMPLES;
        printf("CN_AUTO estimate_x10=%d control_x10=%d power_x10=%d m2m4_x10=%d confidence=%u source=%u flags=0x%02x noise_cells2=%.3f rows_with_noise=%u learned=%lu absent_rejects=%lu estimates=%lu nbw_khz=%u (self-calibrating C/N, cn_auto.h)\n",
               c.x10, receiver_obs_cn_x10(), c.power_x10, c.m2m4_x10, c.confidence, c.source, c.flags,
               (double)c.noise_cells2, measured,
               (unsigned long)s_cna_learned, (unsigned long)s_cna_absent_rejects, (unsigned long)s_cna_estimates,
               calibration_filter_nbw_khz());
    }
    printf("GAIN_TABLE measured_rows=%u g83_g72_db=%.2f (top rows from the receiver-noise map; the rest the model, joined)\n",
           s_gain_rows_measured, (s_gain.model_db_q8[s_gain.table.max_index] - s_gain.model_db_q8[72]) / 256.0);
    {
        unsigned checked, match;
        int first_bad;
        rf_gain_table_hw(&checked, &match, &first_bad);
        printf("GAIN_TABLE_HW checked=%u match=%u first_bad=%d (reconstructed RF/BB/fine tuples against the PHY's AGC "
               "memory at boot; checked=0: not checked)\n", checked, match, first_bad);
    }
    printf("GAIN_REFUSED generation=%lu busy=%lu control=%lu inactive=%lu actuator_lock=%lu rf_set_gain=%lu (writes the core wanted but the PHY did not take, by reason)\n",
           (unsigned long)s_refuse[0], (unsigned long)s_refuse[1], (unsigned long)s_refuse[2], (unsigned long)s_refuse[3],
           (unsigned long)s_refuse[4], (unsigned long)s_refuse[5]);
    printf("DC_FEED rejected_folded=%lu (observer windows reaching both rails of an axis: a folding carrier, kept out of the DC servo)\n",
           (unsigned long)s_dc_rejected_folded);
    printf("CN_OBSERVER windows=%lu period_us=%u max_age_us=%u\n", (unsigned long)s_cn_windows,
           CN_OBSERVE_PERIOD_US, CN_OBSERVE_MAX_AGE_US);
    char mode[16];
    printf("GAIN owner=%s mode=%s gain=%u state=%u p50=%d p95=%d coherence=%d env_x100=%d stuck=%u cn_x10=%d origin_pm=%d clip_pm=%d "
           "anchor=%u anchor_valid=%u anchor_returns=%lu climbs=%lu "
           "writes=%lu holds=%lu overloads=%lu bursts=%lu fade_holds=%lu damp=%lu map_learned=%lu learn_refused=%lu "
           "map_imports=%lu preset_imports=%lu "
           "obs_windows=%lu obs_us_max=%lu\n",
           s_paused ? "paused" :
           s_gain_mode == GAIN_MODE_AUTO ? "keel1" : s_gain_mode == GAIN_MODE_LEVEL ? "level" :
           s_gain_mode == GAIN_MODE_RSSI ? "rssi" : "manual",
           receiver_gain_mode_name(s_gain_mode, mode, sizeof(mode)),
           s_current_gain, (unsigned)s_gain.state, s_obs_p50, s_obs_p95, s_obs_coherence, s_obs_env_ratio_x100,
           (unsigned)s_obs_stuck, receiver_obs_cn_x10(),
           s_obs_origin_pm, s_obs_clip_pm, (unsigned)s_gain.anchor_gain, (unsigned)s_gain.anchor_valid,
           (unsigned long)s_gain.anchor_returns, (unsigned long)s_gain.climbs, (unsigned long)s_gain.writes, (unsigned long)s_gain.holds,
           (unsigned long)s_gain.overloads, (unsigned long)s_gain.bursts_ignored, (unsigned long)s_gain.fade_holds,
           (unsigned long)s_gain.damp_events, (unsigned long)s_gain.learned, (unsigned long)s_gain.learn_refused,
           (unsigned long)s_map_imports, (unsigned long)s_preset_imports, (unsigned long)s_obs_windows,
           (unsigned long)s_obs_us_max);
}

/* Console M: the learned map on one line, for gain_map_preset.py. */
void receiver_print_map(void)
{
    dg_map_blob_t blob;
    unsigned n = receiver_export_gain_map(&blob);
    if (!n) { printf("GAINMAP refused=unsettled_or_changed_context\n"); return; }
    printf("GAINMAP v=%u max=%u band5=%u freq=%u entries=%u", DG_MAP_VERSION, blob.max_index, blob.band5,
           rf_frequency_mhz(), n);
    for (unsigned k = 0; k <= blob.max_index && k < DG_STATES; ++k)
        if (blob.confidence[k] && blob.power_q10[k])
            printf(" %u:%u:%u:%u", k, blob.power_q10[k], blob.confidence[k], blob.uncertainty_pm[k]);
    printf("\n");
}

