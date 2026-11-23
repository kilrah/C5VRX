/* C5VRX-5 by Twotoz and contributors: manual receive-chain calibration.
 * Boot loads a baseline or a matching saved calibration and returns at once.
 * Passive, context-qualified monitoring may recommend CALIBRATE; it never
 * searches, retunes, slips the sampling phase, or trims DC automatically.
 * Menu CALIBRATE / F checks and repairs the current filter's DC rows.
 * Explicit console B also optimizes the filter and prepares other profiles.
 * Gain control/AFC remain the normal receive controls, outside this policy. */
#include "calibration_internal.h"
#include "experimental.h"
#include <stdarg.h>
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_app_desc.h"
#include "flightlog.h"
#include "flightlog_format.h"

static int64_t s_boot_policy_ms, s_manual_ms, s_boot_phase_ms;
int64_t s_probe_deadline;
bool s_boot_phase_pending;
static int64_t s_boot_phase_until;
static bool s_last_manual_full;
static volatile bool s_dump_requested;
void calibration_dump_request(void) { s_dump_requested = true; }

volatile bool s_startup = true;
bool calibration_in_startup(void) { return s_startup; }

bool quiet_owner(void)
{
    return !s_menu_active && !s_scan_active && !receiver_paused() && !phy_busy();
}

/* Status screens of the startup phase own TX (menu raster), so the menu
 * check must not count them as a busy owner. */
static bool startup_owner(void)
{
    return (!s_menu_active || menu_status_active()) && !s_scan_active && !receiver_paused() && !phy_busy();
}

bool collect_stats(unsigned windows, window_stats_t *out, bool noise)
{
    uint8_t sample[PROBE_BYTES];
    dsp_circle_t circle = {0};
    unsigned rail_sum = 0;
    uint16_t hi[16] = {0}, hq[16] = {0};
    memset(out, 0, sizeof(*out));
    for (unsigned tries = 0; tries < windows * 3u && out->windows < windows; ++tries) {
        if (s_probe_deadline && (esp_timer_get_time() >= s_probe_deadline || !menu_splash_active())) break;
        vTaskDelay(2);   /* two ticks: one tick starved IDLE with the observer */
        if (!copy_probe_window(sample, NULL)) continue;
        for (unsigned r = 0; r < PROBE_REGIONS; ++r) {
            dsp_hf4_sums(sample + r * PROBE_REGION_BYTES, PROBE_REGION_BYTES, &out->hf4_d4);
            dsp_hf2_sums(sample + r * PROBE_REGION_BYTES, PROBE_REGION_BYTES, &out->hf2_d2);
        }
        out->hf4_count += PROBE_REGIONS * (PROBE_REGION_BYTES - 4u);
        dsp_circle_sums(sample, sizeof(sample), &circle);
        out->m = measure_window(sample, sizeof(sample));
        rail_sum += (unsigned)out->m.clip_pm;
        if (noise) {
            cna_sums_t p = {0};
            for (unsigned k = 0; k < sizeof(sample); ++k) {
                cna_sums_add(&p, sample[k]);
                ++hi[dsp_i(sample[k]) & 15]; ++hq[dsp_q(sample[k]) & 15];
            }
            float mi, mq;
            double m2, m4;
            cna_mean(&p, &mi, &mq); cna_moments(&p, mi, mq, &m2, &m4);
            out->noise_m2 += m2; out->noise_m4 += m4;
            cna_sums_merge(&out->noise_sums, &p);
        }
        ++out->windows;
    }
    if (!out->windows) return false;
    out->dc_circle = dc_live_measure(&circle, rail_sum / out->windows,
        &out->dc_i, &out->dc_q, &out->envelope_x100);
    unsigned ci = 0, cq = 0;
    const unsigned n = out->windows * PROBE_BYTES;
    for (unsigned k = 0; k < 16u; ++k) { ci += hi[k] * 20u >= n; cq += hq[k] * 20u >= n; }
    out->dc_noise_dither = noise && ci >= 3u && cq >= 3u;
    int ni, nq;
    out->dc_qualified = out->dc_circle || dc_live_noise_mean(&circle, rail_sum / out->windows, &ni, &nq);
    return out->windows * 2u >= windows;
}

bool collect(unsigned windows, window_stats_t *out)
{
    return collect_stats(windows, out, false);
}

/* Quiet test: dsp_quiet_reason() (dsp.h). */
#define QUIET_NO_SYNC_US   2000000LL
/* s_last_video_us starts at 0: "no sync seen yet" is not recent sync. */
bool video_recent(void)
{
    const int64_t t = s_last_video_us;
    return t != 0 && esp_timer_get_time() - t < QUIET_NO_SYNC_US;
}

/* Manual request (menu or console): runs the startup phase once more, in
 * this task (the console keeps draining USB). A live STRONG LANE step is no
 * PHY experiment: the calibration's pause returns to the ultrafine lane
 * (receiver_pause) before anything is measured. */
static volatile bool s_manual_requested, s_full_requested;
void filter_request(void)
{
    if (experimental_transfer_active() && !receiver_strong_lane()) { printf("CAL refused: reset PHY experiments first\n"); return; }
    if (s_startup) { printf("COMMAND F refused=busy\n"); return; }
    s_manual_requested = true;
}
void filter_request_full(void)
{
    if (experimental_transfer_active() && !receiver_strong_lane()) { printf("CAL refused: reset PHY experiments first\n"); return; }
    if (s_startup) { printf("COMMAND B refused=busy\n"); return; }
    s_full_requested = true; s_manual_requested = true;
}

/* VTX deviation for the receive profile (profile.c): measured fresh on
 * this boot's carrier while the DEFAULT picture keeps running (a stored
 * deviation is not a VTX identity); only the switch to a matched profile
 * happens behind a status screen. With no VTX yet (calibrated with the VTX
 * off) it gives one 2 s to appear; later the measurement goes
 * on passively and a result is kept as pending for the next boot. */
#define DEVIATION_WAIT_US    2000000LL
#define DEVIATION_MEASURE_US 15000000LL
static void deviation_startup(int64_t deadline)
{
    rf_filter_t matched;
    if (!filter_setting(FILTER_NARROW, &matched) && !filter_setting(FILTER_WIDE, &matched)) {
        profile_set_reason("no_matched_filter");
        printf("PROFILE startup skip=no_matched_filter\n");
        return;
    }
    const int64_t wait_until = esp_timer_get_time() + DEVIATION_WAIT_US;
    while (!video_recent() && esp_timer_get_time() < wait_until && esp_timer_get_time() < deadline)
        vTaskDelay(pdMS_TO_TICKS(200));
    if (!video_recent()) {
        profile_set_reason("no_video_at_startup");
        printf("PROFILE startup skip=no_video (measured passively later, pending for the next boot)\n");
        return;
    }
    profile_measure_begin();
    const int64_t until = esp_timer_get_time() + DEVIATION_MEASURE_US;
    while (profile_measure_result() < 0 && esp_timer_get_time() < until && esp_timer_get_time() < deadline)
        vTaskDelay(pdMS_TO_TICKS(100));
    profile_measure_end();
    int dev = profile_measure_result();
    if (dev < 0) {
        profile_set_reason("inconclusive_at_startup");
        printf("PROFILE startup result=inconclusive profile=%s\n", profile_name(profile_active()));
        return;
    }
    profile_commit(dev);
}

static void calib_record(bool manual, int64_t t0);   /* flight log, below */
#define STARTUP_BUDGET_US 120000000LL
/* A full calibration (first boot, a new calibration generation, CALIBRATE)
 * measures the filter and three DC tables: 2 minutes covered the filter and
 * 19 of 30 rows of one table (board 2026-10-19), and the rest took two more
 * boots. It gets the time to finish once; a normal boot is unchanged. */
#define STARTUP_FULL_BUDGET_US 480000000LL
static void startup_phase(bool full)
{
    const bool manual = true;
    s_startup = true;
    cal_dc_noise_cache_clear();
    {
        const uint8_t m = manual;
        flightlog_event(FLOG_E_CAL_START, &m, 1);
    }
    const int64_t t0 = esp_timer_get_time();
    const int64_t deadline = t0 + STARTUP_FULL_BUDGET_US;
    /* Direct Gain first finds its state (table maximum without a carrier). */
    for (unsigned k = 0; k < 40u && s_gain.state == DG_SETTLE; ++k) vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(manual ? 200 : 1500));
    for (unsigned k = 0; k < 100u && !startup_owner(); ++k) vTaskDelay(pdMS_TO_TICKS(100));
    if (!startup_owner()) {
        printf("STARTUP skip=owner_busy menu=%u scan=%u paused=%u\n", s_menu_active, s_scan_active, receiver_paused());
    } else {
        printf("STARTUP begin manual=%u filter=%s dco_rows=%u deviation_x100=%d\n", manual,
               cal_filter_measured() ? "stored" : "missing", dco_valid_rows(),
               profile_deviation_x100(rf_channel_index()));
        /* A manual run re-measures the filter and the VTX: leave a matched
         * profile first, so filter and tracker never disagree. */
        if (profile_active() != PROFILE_DEFAULT) {
            status_screen("CALIBRATING", "BACK TO THE DEFAULT PROFILE", "", -1);
            if (!profile_set(PROFILE_DEFAULT)) printf("STARTUP error: DEFAULT profile not applied\n");
        }
        printf("STARTUP step=sphase\n");
        sphase_qualify("startup", true);
        printf("STARTUP step=filter\n");
        if (phy_filter_calibrated_code() < 0) { s_filter_result = "unsupported_phy"; }
        else if (full) {
            (void)filter_calibrate(!manual);
        }
        printf("STARTUP step=dco\n");
        if (esp_timer_get_time() < deadline) dco_startup(manual, deadline);
        /* Reuse fresh NORMAL DC windows before the other filter passes. */
        printf("STARTUP step=noise_map\n");
        if (esp_timer_get_time() < deadline) noise_map_startup(deadline);
        if (esp_timer_get_time() < deadline) dco_startup_profiles(manual, deadline);
        else dco_ready_note();
        if (!full && esp_timer_get_time() < deadline) filter_measure_current();
        printf("STARTUP step=deviation\n");
        /* Video needs a moment back on the own channel before measuring. */
        status_screen_end();
        vTaskDelay(pdMS_TO_TICKS(1500));
        /* Final qualification in the live TX/RX context, before the VTX
         * deviation measurement that needs the picture. */
        printf("STARTUP step=sphase_final\n");
        if (sphase_on_carrier()) sphase_carrier_scan("final", false);   /* choose on the carrier */
        else sphase_qualify("final", true);
        if (full) {
            if (esp_timer_get_time() < deadline) deviation_startup(deadline);
            else profile_set_reason("manual_budget");
        }
    }
    /* A fixed gain is saved across boots and costs range without any sign
     * in the picture (a bench setting left behind): say so at every boot. */
    if (!manual && !receiver_gain_mode_auto(receiver_gain_mode())) {
        char mode[16], line[48];
        const char *name = receiver_gain_mode_name(receiver_gain_mode(), mode, sizeof(mode));
        snprintf(line, sizeof(line), "GAIN %s COSTS RANGE: USE AUTO", name);
        printf("GAIN_WARNING mode=%s: a fixed gain is saved and costs range; menu GAIN or console G to AUTO\n",
               name);
        status_screen("GAIN NOT AUTO", line, "MENU > GAIN, HOLD TO CHANGE", -1);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    /* A calibration never ends silently half done: blackbox 2026-10-27 had
     * six and three manual runs in a row (sessions 240, 242) leaving 30 and 15
     * of 30 DC rows from templates, with only an 0.8 s DC NOT COMPLETE line.
     * The end screen names the result for 3 s; the live servo still trims
     * every row, so an incomplete table costs time to converge, not range. */
    {
        const unsigned templ = dco_template_rows(), missing = dco_rows(0);
        const bool dc_ok = dco_current_complete();
        char line1[48], line2[48];
        if (dc_ok) {
            snprintf(line1, sizeof(line1), "DC %u ROWS MEASURED", dco_valid_rows() + dco_rows(DCO_ROW_FLOOR));
            snprintf(line2, sizeof(line2), "FILTER %s", s_filter_result);
        } else {
            snprintf(line1, sizeof(line1), "DC: %u TEMPLATE %u MISSING", templ, missing);
            snprintf(line2, sizeof(line2), "%s: VTX OFF, CALIBRATE AGAIN",
                     !strcmp(s_dco_last_why, "carrier_detected") || !strcmp(s_dco_last_why, "need_vtx_off") ? "CARRIER" :
                     !strcmp(s_dco_last_why, "timeout") ? "TIMEOUT" : "STOPPED");
        }
        printf("CAL_RESULT dc=%s why=%s template_rows=%u missing_rows=%u filter=%s\n", dc_ok ? "complete" : "incomplete",
               s_dco_last_why, templ, missing, s_filter_result);
        status_screen(dc_ok ? "CALIBRATION OK" : "CALIBRATION INCOMPLETE", line1, line2, dc_ok ? 100 : -1);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    status_screen_end();
    s_startup = false;
    calib_record(manual, t0);
    flightlog_event(FLOG_E_CAL_END, NULL, 0);
    profile_live_begin();   /* passive from here on: never switches */
    s_manual_ms = (esp_timer_get_time() - t0) / 1000;
    s_last_manual_full = full;
    printf("CAL_MANUAL kind=%s ms=%lld searches=%lu filter=%s\n", full ? "full" : "current_filter",
           (esp_timer_get_time() - t0) / 1000, (unsigned long)s_dco_searches, s_filter_result);
    printf("STARTUP done ms=%lld filter=%s dco_rows=%u dco_missing=%u sphase=%s profile=%s reason=%s\n",
           (esp_timer_get_time() - t0) / 1000, s_filter_result, dco_valid_rows(), dco_rows(0), s_sphase_state,
           profile_name(profile_active()), profile_reason());
}

/* Persist the measured gain map: at most every 30 s, while holding, only
 * when it changed. Short flights end with the battery pulled, so 2 minutes
 * lost most of a flight's learning. Flash wear: a ~540-byte blob in the
 * 24 KiB NVS partition, even written every 30 s, is > 30,000 flight hours
 * of page erase cycles; unchanged maps are never written. */
static void gain_map_service(void)
{
    static int64_t last;
    int64_t now = esp_timer_get_time();
    if (now - last < 30000000LL) return;
    dg_map_blob_t blob;
    if (!receiver_export_gain_map(&blob)) return;
    last = now;
    if (s_gain_map_saved_valid && !memcmp(&blob, &s_gain_map_saved, sizeof(blob))) return;
    if (blob_store("gain_map", &blob, sizeof(blob))) {
        /* Observer imports this cache; publish the complete verified blob. */
        vTaskSuspendAll();
        s_gain_map_saved = blob; s_gain_map_saved_valid = true;
        (void)xTaskResumeAll();
    }
}

/* Framed read-only JSON-lines report. Sequence + FNV cover every payload
 * byte, including newlines, so the browser refuses a truncated USB dump.
 * Only this task owns the DC/NVS scratch state; no calibration is requested.
 * Live readings are labelled observational and bracketed by context epochs. */
void cal_dump_line(cal_dump_t *d, const char *fmt, ...)
{
    if (d->failed) return;
    char line[512];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0 || (unsigned)n >= sizeof(line)) { d->failed = true; return; }
    for (int k = 0; k < n; ++k) d->hash = (d->hash ^ (uint8_t)line[k]) * 16777619u;
    d->hash = (d->hash ^ '\n') * 16777619u;
    printf("CALDUMP_LINE n=%u %s\n", d->lines++, line);
    vTaskDelay(2); /* bounded chunks, never suspend the live receiver */
}
static void calibration_dump(void)
{
    cal_dump_t d = {.hash = 2166136261u};
    const uint32_t context = health_context();
    const int64_t t0 = esp_timer_get_time();
    uint8_t mac[6] = {0};
    const bool have_id = esp_efuse_mac_get_default(mac) == ESP_OK;
    const uint32_t board = have_id ? flog_crc32(0x5a5a5a5au, mac, sizeof(mac)) : 0;
    esp_chip_info_t chip; esp_chip_info(&chip);
    printf("CALDUMP_BEGIN schema=1\n");
    cal_dump_line(&d, "{\"type\":\"device\",\"schema\":1,\"board_id\":\"%08lx\",\"board_id_valid\":%s,"
        "\"firmware\":\"%s\",\"idf\":\"%s\",\"chip_revision\":%u,\"cal_generation\":%u,\"uptime_us\":%lld}",
        (unsigned long)board, have_id ? "true" : "false", firmware_id(), esp_app_get_description()->idf_ver,
        chip.revision, calibration_generation_id(), t0);
    const rf_filter_t f = rf_filter();
    cal_dump_line(&d, "{\"type\":\"active\",\"freq_mhz\":%u,\"offset_khz\":%d,\"gain\":%u,\"lane\":%u,"
        "\"filter_code\":%d,\"filter_skirt\":%u,\"filter_bw20\":%s,\"filter_measured\":%s,"
        "\"filter_nbw_khz\":%u,\"dc_held\":%s,\"health\":%u,\"dc_i_mcells\":%d,\"dc_q_mcells\":%d,"
        "\"qualified_blocks\":%u,\"rejected_blocks\":%u,\"calibration_recommended\":%s}", rf_frequency_mhz(), rf_offset_khz(), s_current_gain,
        rf_lane(), f.code, f.skirt, f.bw20 ? "true" : "false", cal_filter_measured() ? "true" : "false",
        calibration_filter_nbw_khz(), phy_dco_held() ? "true" : "false", s_basemap_watch.state,
        s_basemap_watch.last_i, s_basemap_watch.last_q, s_health_blocks, s_health_rejected,
        calibration_dc_needed(NULL, 0) ? "true" : "false");
    int64_t radio_ms, ready_ms;
    firmware_boot_timing(&radio_ms, &ready_ms);
    cal_dump_line(&d, "{\"type\":\"boot_timing\",\"radio_ms\":%lld,\"ready_ms\":%lld,\"target_ms\":10000}",
                  radio_ms, ready_ms);
    int bank1[2] = {0}, bank2[2] = {0};
    const bool banks_valid = phy_dco_banks(bank1, bank2);
    cal_dump_line(&d, "{\"type\":\"dc_hardware\",\"valid\":%s,\"gain\":%u,\"bank1_i\":%d,\"bank1_q\":%d,"
        "\"bank2_i\":%d,\"bank2_q\":%d}", banks_valid ? "true" : "false", s_current_gain, bank1[0], bank1[1], bank2[0], bank2[1]);
    cal_dump_line(&d, "{\"type\":\"policy\",\"auto_calibration\":false,\"auto_dc_trim\":true,\"dc_adopted\":%lu,"
        "\"baseline_code\":%d,\"baseline_skirt\":%d,\"baseline_bw20\":%s,\"dc_preset_boards\":%u,\"dc_preset_holds\":%lu,"
        "\"boot_blocking_ms\":%lld,\"boot_phase_ms\":%lld,\"last_manual_ms\":%lld,\"last_manual_full\":%s,\"health_observation_us\":%lld}",
        (unsigned long)s_dco_adopted, CAL_BASELINE_FILTER_CODE, CAL_BASELINE_FILTER_SKIRT,
        CAL_BASELINE_FILTER_BW20 ? "true" : "false",
        (unsigned)DCO_PRESET_BOARDS, (unsigned long)s_dco_preset_holds,
        s_boot_policy_ms, s_boot_phase_ms, s_manual_ms, s_last_manual_full ? "true" : "false", s_health_us);
    static const int classes[4] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE, FILTER_XNARROW};
    for (unsigned k = 0; k < 4u; ++k) {
        const filter_entry_t x = filter_class_entry(classes[k]);
        cal_dump_line(&d, "{\"type\":\"filter\",\"class\":%u,\"valid\":%s,\"code\":%d,\"skirt\":%u,"
            "\"bw20\":%s,\"width_khz\":%u,\"nbw_khz\":%u}", k, x.valid ? "true" : "false", x.code,
            x.skirt, x.bw20 ? "true" : "false", x.width_khz, x.nbw_khz);
    }
    cal_dc_dump_tables(&d);
    dg_map_blob_t gain = {0};
    const bool gain_valid = receiver_export_gain_map(&gain) != 0;
    cal_dump_line(&d, "{\"type\":\"gain_context\",\"valid\":%s,\"version\":%u,\"freq_mhz\":%u,"
        "\"offset_khz\":%ld,\"filter_code\":%ld,\"filter_skirt\":%lu,\"filter_bw20\":%lu,\"iq_scale\":%lu}",
        gain_valid ? "true" : "false", gain.version, gain.freq_mhz, (long)gain.context.offset_khz,
        (long)gain.context.filter_code, (unsigned long)gain.context.filter_skirt,
        (unsigned long)gain.context.filter_bw20, (unsigned long)gain.context.iq_scale);
    if (gain_valid) for (unsigned g = 0; g <= gain.max_index && g < DG_STATES; ++g)
        cal_dump_line(&d, "{\"type\":\"gain_row\",\"gain\":%u,\"power_q10\":%u,\"uncertainty_pm\":%u,"
            "\"confidence\":%u}", g, gain.power_q10[g], gain.uncertainty_pm[g], gain.confidence[g]);
    for (unsigned g = 0; g <= ARC_VENDOR_GAIN_MAX; ++g) {
        float noise = 0; uint16_t samples = 0; int64_t measured_us = 0;
        if (receiver_export_noise((uint8_t)g, &noise, &samples, &measured_us))
            cal_dump_line(&d, "{\"type\":\"noise_row\",\"gain\":%u,\"cells2\":%.6f,\"samples\":%u,"
                "\"measured_us\":%lld}", g, (double)noise, samples, measured_us);
    }
    cal_dump_line(&d, "{\"type\":\"capture\",\"live_context_unchanged\":%s,\"duration_ms\":%lld}",
        context == health_context() ? "true" : "false", (esp_timer_get_time() - t0) / 1000);
    if (d.failed) printf("CALDUMP_ERROR reason=record_too_large\n");
    else printf("CALDUMP_END lines=%u fnv1a=%08lx\n", d.lines, (unsigned long)d.hash);
}

/* Reset-dependent sampling alignment is the sole bounded boot actuator.
 * It runs under the existing splash, never later on live video. A short
 * absolute check usually needs no slip; ambiguous evidence stays unknown.
 * No nine-position retry, filter sweep, gain table search or DC trimming. */
static void boot_phase_check(void)
{
    s_boot_phase_pending = false;
    if (rf_lane() != RF_LANE_ULTRAFINE) return;
    const int64_t t0 = esp_timer_get_time();
    s_probe_deadline = t0 + 600000;
    s_startup = true;
    window_stats_t w;
    const uint32_t context = health_context();
    if (collect(24, &w) && w.windows == 24u && sphase_record(&w) < SPHASE_LIMIT &&
        collect(24, &w) && w.windows == 24u && sphase_record(&w) < SPHASE_LIMIT &&
        context == health_context())
        s_sphase_state = "settled";
    else if (menu_splash_active() && esp_timer_get_time() < s_probe_deadline) {
        const sphase_verdict_t v = sphase_scan_n("BOOT_PHASE", 3u, 24u, 3u, false);
        s_sphase_state = v == SPHASE_SETTLED ? "settled" : "unverified";
    }
    s_probe_deadline = 0;
    s_startup = false;
    s_boot_phase_ms = (esp_timer_get_time() - t0) / 1000;
    printf("BOOT_PHASE ms=%lld budget_ms=600 state=%s\n", s_boot_phase_ms, s_sphase_state);
}

static void calibration_boot(void)
{
    const int64_t boot_t0 = esp_timer_get_time();
    dco_select();
    dco_ready_note();
    s_startup = false;
    s_boot_phase_pending = true;
    s_boot_phase_until = boot_t0 + 5000000;
    profile_live_begin();
    s_boot_policy_ms = (esp_timer_get_time() - boot_t0) / 1000;
    printf("CAL_BOOT mode=baseline ms=%lld searches=0 sweeps=0 dc_trim=0 boot_phase_budget_ms=600\n",
           (esp_timer_get_time() - boot_t0) / 1000);
}

static void calibration_task(void *arg)
{
    (void)arg;
    s_cal_task = xTaskGetCurrentTaskHandle();
    rf_set_post_gain_hook(dco_post_gain);
    printf("STARTUP task up\n");
    calibration_boot();
    int64_t periodic_us = 0;
    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250))) {
            /* Woken for a passive DC observation, and the rest still at its 250 ms
             * pace. The urgent wakes used to skip it altogether: after a
             * channel change the old DC table pushed the noise onto a rail,
             * one-sided clipping woke the servo every 100 ms, dco_service()
             * never ran to load the new channel's table, and the servo skipped
             * the foreign table: no DC correction until the next boot (board,
             * PR #228: A1 -> A2/A3, every probe clipped for minutes). */
            ++s_dc_urgent_wakes;
            servo_service();
            drift_service();
            if (esp_timer_get_time() - periodic_us < 250000) continue;
        }
        periodic_us = esp_timer_get_time();
        if (s_boot_phase_pending && periodic_us >= s_boot_phase_until) s_boot_phase_pending = false;
        if (s_boot_phase_pending && menu_splash_active()) boot_phase_check();
        if (s_manual_requested && !s_menu_active) {
            const bool full = s_full_requested;
            s_manual_requested = s_full_requested = false;
            startup_phase(full);
            if (dco_current_complete()) s_cal_notice = false;
            memset(&s_basemap_watch, 0, sizeof(s_basemap_watch));
            continue;
        }
        if (s_dump_requested) { s_dump_requested = false; calibration_dump(); continue; }
        if (s_lane_requested && !s_menu_active) { s_lane_requested = false; lane_compare(); continue; }
        if (s_selfnoise_requested && !s_menu_active) { s_selfnoise_requested = false; self_noise_test(); continue; }
        if (sense_lab_pending() && !s_menu_active) { sense_lab_run(quiet_owner()); continue; }
        dco_service();
        servo_service();
        drift_service();
        phase_service();
        gain_map_service();
    }
}

/* Calibration generation ("cal_gen" in NVS). A calibration made by an
 * earlier firmware is not trusted: what a table means changed more than once
 * (DC codes measured in the wrong analog filter branch, one table for every
 * filter; docs/FINDINGS.md 2026-10-19), and an upgrade path that keeps old
 * codes kept exactly those errors. A boot with another generation discards
 * the filter calibration and every DC table, and runs the
 * full first-boot calibration. Raise CAL_GENERATION whenever a firmware
 * changes what a stored calibration means. */
/* 3: 11p, 4: digital RX mode 12, 5: 4-class mode 12 filter and DC base map;
 * each changes the transfer function/noise seen by the stored tables. */
#define CAL_GENERATION 5u
static void calibration_generation(void)
{
    uint16_t stored = 0;
    if (blob_load("cal_gen", &stored, sizeof(stored)) && stored == CAL_GENERATION) return;
    printf("CALIBRATION generation stored=%u firmware=%u: calibration of an earlier firmware discarded, using baseline; "
           "manual calibration only\n", stored, CAL_GENERATION);
    cal_dc_discard();
    const bool ok = cal_filter_discard() && cal_dc_store_discarded();
    const uint16_t now = CAL_GENERATION;
    if (!ok || !blob_store("cal_gen", &now, sizeof(now)))
        printf("CALIBRATION error: generation not stored, discarded again next boot\n");
}

uint16_t calibration_generation_id(void) { return CAL_GENERATION; }
/* Flight log: what this startup phase calibrated and the state it left. */
static void calib_record(bool manual, int64_t t0)
{
    uint8_t p[80] = {0};
    p[0] = manual;
    p[1] = (uint8_t)(!strcmp(s_filter_result, "loaded") ? 1 : !strcmp(s_filter_result, "stored") ? 2 : 3);
    flog_put32(p + 2, (uint32_t)((esp_timer_get_time() - t0) / 1000));
    cal_filter_record(p);
    cal_dc_record(p);
    p[49] = sphase_code();
    flightlog_record(FLOG_T_CALIB, p, 56);
}

void calibration_start(void)
{
    calibration_generation();
    cal_filter_load();
    ESP_ERROR_CHECK(xTaskCreate(calibration_task, "calibration", 8192, NULL, 1, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
}

void calibration_print(void)
{
    const rf_filter_t f = rf_filter();
    cal_filter_print(f);
    cal_dc_print(f);
    cal_sphase_print();
}
