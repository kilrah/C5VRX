/* C5VRX-5 by Twotoz and contributors: analog filter calibration of the
 * calibration task (classes, sweep, stored settings). */
#include "calibration_internal.h"
#include "flightlog_format.h"

/* ---- Narrow analog filter --------------------------------------------------
 * The FM threshold follows the pre-detection noise bandwidth, and PARLIO
 * keeps every second sample of the ~80 MS/s bus unfiltered, so the noise
 * the demod sees is set by the analog filter, alias included. The
 * capacitor codes are per-chip calibrated, so the filter is measured, not
 * assumed: with no carrier at maximum gain every setting (digital BW40/
 * BW20 x RX0 code x second-stage skirt) gets a 64-point noise spectrum, and
 * the one with the lowest noise bandwidth whose -3 dB width still covers
 * 14 MHz (enough for every tested VTX deviation, docs/FINDINGS.md) and whose
 * noise stays incoherent (so Direct Gain still recognises "no carrier")
 * is kept. Built on ESPARGOS esp-sdr's C5 BANDWIDTH control (GPL-3.0). */
/* 3: wide class; 4: codes 52/62/63 (one new sweep after the update);
 * 5: settings measured in the analog branch they fly in (rf_set_filter sets
 * the bandwidth mode up before measuring; every earlier sweep mixed the two
 * branches, board 2026-10-19: chosen 12.5 MHz flew at 19.7 MHz) */
/* FILTER_VERSION: calibration_internal.h */
/* Two classes from one sweep (research: 12 MHz helps every tested VTX
 * deviation and keeps strong-signal sharpness; 10 MHz adds up to ~1.7 dB
 * for deviation <= ~1.1 but hurts wide VTXs, so it is used only when the
 * deviation was measured). */
#define FILTER_MIN_KHZ     12000u
#define FILTER_NARROW_KHZ  10000u
/* Wide VTXs (deviation >= 1.6): the 12 MHz class clips their spectrum
 * (research/data/wide.txt). */
#define FILTER_WIDE_KHZ    16000u
/* XNARROW (menu FILTER only, never automatic): the setting with the smallest
 * -3 dB width, operator 2026-10-26. Mode 12 sweep: BW20 code 63 skirt 0 at
 * 10.6 MHz, though its noise bandwidth (11.7 MHz) is no lower than NARROW's;
 * the floor keeps a broken probe from winning. */
#define FILTER_XNARROW_MIN_KHZ 8000u
static unsigned quiet_limit(int clip_pm) { return dsp_quiet_limit(clip_pm); }
#define CAL_WINDOWS        96u
typedef struct {
    uint8_t version;
    filter_entry_t normal, narrow, wide;
    uint16_t calibrated_nbw_khz;
} filter_blob_t;
static filter_blob_t s_filter_cal;
/* XNARROW in its own blob ("filter_xn"): the v6 filter blob and every board
 * calibration made with it stay valid; XNARROW comes with the next FULL. */
typedef struct {
    uint8_t version;
    filter_entry_t xnarrow;
} filter_xn_blob_t;
static filter_xn_blob_t s_filter_xn;
static uint16_t s_original_mhz;
const char *s_filter_result = "never";

/* Accumulate in-place: a promoted setting keeps its screening FFTs and
 * extends to all 96 windows, with no retune or extra settling delay. */
static bool noise_collect(cal_noise_t *a, unsigned windows)
{
    uint8_t sample[PROBE_BYTES];
    for (unsigned tries = 0; tries < windows * 3u && a->got < windows; ++tries) {
        vTaskDelay(1);   /* retain the existing filter collector cadence */
        if (!copy_probe_window(sample, NULL)) continue;
        cal_noise_add(a, sample, sizeof(sample), measure_window(sample, sizeof(sample)).clip_pm);
    }
    return a->got == windows;
}

/* On-screen progress: the measured time per setting gives the time left. */
static unsigned s_cal_done, s_cal_total;
/* Higher code = narrower. Board sweep (FINDINGS, 2026-10-12): the narrow
 * class landed on code 60, the old grid's last code, at 11.9 MHz noise
 * bandwidth instead of the modelled 10 MHz; the field takes 0..63, so 62
 * and 63 are tried too, and 52 fills the gap around the 12 MHz class. */
#define FILTER_CODES  13u
#define FILTER_SKIRTS 5u
static int64_t s_cal_t0;
static bool s_cal_own_channel;
static void cal_status(void)
{
    char line1[48], line2[48];
    int64_t spent = esp_timer_get_time() - s_cal_t0;
    unsigned left = s_cal_done ? (unsigned)(spent / s_cal_done * (s_cal_total - s_cal_done) / 1000000) + 1u : 30u;
    if (s_cal_own_channel) snprintf(line1, sizeof(line1), "KEEP THE VTX OFF");
    else snprintf(line1, sizeof(line1), "ON QUIET CHANNEL %s (VTX MAY STAY ON)", rf_channel()->name);
    snprintf(line2, sizeof(line2), "ABOUT %u S LEFT, THEN LIVE VIDEO", left);
    status_screen("CALIBRATING", line1, line2, (int)(s_cal_done * 100u / s_cal_total));
}

/* *quiet stays false for a setting the PHY did not read back, so it can
 * never be chosen; false only when the probe itself fails. */
static bool measure_setting(rf_filter_t f, unsigned *width, unsigned *nbw, bool *quiet, bool *carrier,
                            const unsigned best_nbw[3], bool *screen_fault)
{
    if (s_cal_total) { ++s_cal_done; cal_status(); }
    *quiet = *carrier = false;
    if (!rf_set_filter(f)) {
        printf("FILTER_CAL bw=%s code=%d skirt=%u skip=not_applied\n", f.bw20 ? "BW20" : "BW40", f.code, f.skirt);
        *width = *nbw = 0;
        return true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    unsigned env = 0;
    int clip = 0;
    cal_noise_t a = {0};
    if (best_nbw) {
        if (!noise_collect(&a, 24u)) return false;
        const unsigned sw = dsp_psd_width_khz(a.psd), sn = dsp_psd_nbw_khz(a.psd);
        env = a.env_sum / a.got; clip = a.clip_sum / (int)a.got;
        if (cal_setting_carrier(env, clip)) { *carrier = true; return true; }
        const bool contender = !cal_setting_quiet(env, clip) ||
            cal_filter_contender(sw, sn, FILTER_MIN_KHZ, best_nbw[0]) ||
            cal_filter_contender(sw, sn, FILTER_NARROW_KHZ, best_nbw[1]) ||
            cal_filter_contender(sw, sn, FILTER_WIDE_KHZ, best_nbw[2]);
        if (!contender && sn) {
            *width = *nbw = 0; /* screened out, never eligible for storage */
            printf("FILTER_CAL screen=reject width_khz=%u nbw_khz=%u\n", sw, sn);
            return true;
        }
        if (!noise_collect(&a, CAL_WINDOWS) || !cal_noise_finish(&a, width, nbw, &env, &clip)) return false;
        if (!cal_filter_screen_agrees(sw, sn, *width, *nbw)) *screen_fault = true;
    } else if (!noise_collect(&a, CAL_WINDOWS) || !cal_noise_finish(&a, width, nbw, &env, &clip)) return false;
    /* A setting with clipped noise is just not chosen; only a clearly
     * non-noise envelope means a carrier appeared. */
    *quiet = cal_setting_quiet(env, clip);
    *carrier = cal_setting_carrier(env, clip);
    printf("FILTER_CAL bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u env_x100=%u limit=%u clip_pm=%d\n",
           f.bw20 ? "BW20" : "BW40", f.code, f.skirt, *width, *nbw, env, quiet_limit(clip), clip);
    return true;
}

/* A channel is quiet when it shows receiver noise at maximum gain: no
 * burst-confirmed sync, noise-like envelope, not dead or railing IQ. */
static bool channel_quiet(void)
{
    unsigned env = 0;
    int clip = 0;
    const char *why = video_recent() ? "video_sync" : noise_check(&env, &clip);
    printf("FILTER_CAL channel=%s freq=%u gain=%u quiet=%u reason=%s env_x100=%u clip_pm=%d\n", rf_channel()->name,
           rf_frequency_mhz(), s_current_gain, why == NULL, why ? why : "-", env, clip);
    return why == NULL;
}

/* The filter is the chip's own baseband filter: its noise bandwidth does
 * not depend on the channel. With a VTX on the current channel the sweep
 * runs on the quietest other channel instead and returns afterwards. */
static bool find_quiet_channel(size_t original)
{
    if (channel_quiet()) return true;
    /* Channels within 40 MHz are skipped: a VTX leaks into its neighbours. */
    for (size_t k = 0; k < FPV_CHANNELS; ++k) {
        size_t ch = (original + FPV_CHANNELS / 2 + k) % FPV_CHANNELS;
        if (ch == original) continue;
        if (abs((int)rf_channel_freq(ch) - (int)s_original_mhz) < 40) continue;
        if (rf_set_channel(ch) != ESP_OK) continue;
        receiver_set_gain(receiver_gain_top());   /* useful top: under an external LNA G83 folds */
        vTaskDelay(pdMS_TO_TICKS(50));
        if (channel_quiet()) return true;
    }
    return false;
}

bool filter_calibrate(bool automatic)
{
    if (phy_filter_calibrated_code() < 0) { s_filter_result = "unsupported_phy"; return false; }
    if (!receiver_pause("FILTER_CAL")) { s_filter_result = "busy"; return false; }
    const uint8_t saved_gain = s_current_gain;
    const size_t original = rf_channel_index();
    const int original_offset = rf_offset_khz();
    s_original_mhz = rf_frequency_mhz();
    const rf_filter_t previous = rf_filter(), calibrated = {PHY_FILTER_CALIBRATED, 0, false};
    s_scan_active = true;              /* control loop and AFC stand aside */
    bool ok = rf_set_filter(calibrated);
    receiver_set_gain(receiver_gain_top());   /* useful top: under an external LNA G83 folds */
    vTaskDelay(pdMS_TO_TICKS(50));
    status_screen("CALIBRATING", "LOOKING FOR A QUIET CHANNEL", "ONE TIME ONLY, ABOUT 30 S", -1);
    const char *fail = ok ? NULL : "calibrated_bytes_not_applied";
    if (ok && !(ok = find_quiet_channel(original))) fail = "no_quiet_channel";
    s_cal_own_channel = rf_channel_index() == original;
    s_cal_done = 0;
    s_cal_total = ok ? 2u + 2u * FILTER_CODES * FILTER_SKIRTS : 0u;   /* checks + BW40/BW20 x codes x skirts */
    s_cal_t0 = esp_timer_get_time();
    printf("FILTER_CAL begin mode=%s freq=%u (own %u) target_width_khz=%u\n", automatic ? "auto" : "manual",
           rf_frequency_mhz(), s_original_mhz, FILTER_MIN_KHZ);
    unsigned width = 0, nbw = 0, cal_nbw = 0;
    bool quiet = false, carrier = false;
    if (ok && !(ok = measure_setting(calibrated, &width, &cal_nbw, &quiet, &carrier, NULL, NULL) && quiet))
        fail = carrier ? "carrier_detected" : "not_quiet";
    rf_filter_t best = calibrated, narrow = calibrated;
    unsigned best_nbw = ok && width >= FILTER_MIN_KHZ ? cal_nbw : 0, best_width = width;
    unsigned narrow_nbw = ok && width >= FILTER_NARROW_KHZ ? cal_nbw : 0, narrow_width = width;
    rf_filter_t wide = calibrated;
    unsigned wide_nbw = ok && width >= FILTER_WIDE_KHZ ? cal_nbw : 0, wide_width = width;
    rf_filter_t xnarrow = calibrated;   /* smallest -3 dB width, then lower noise bandwidth */
    unsigned xnarrow_nbw = ok && width >= FILTER_XNARROW_MIN_KHZ ? cal_nbw : 0, xnarrow_width = width;
    static const int8_t codes[FILTER_CODES] = {PHY_FILTER_CALIBRATED, 0, 8, 16, 24, 32, 40, 48, 52, 56, 60, 62, 63};
    static const uint8_t skirts[FILTER_SKIRTS] = {0, 8, 16, 24, 32};
    /* First pass screens all 130 settings. Any screen/full discrepancy
     * outside the conservative guard invalidates the pruning evidence:
     * restart exhaustively, including every previously rejected setting. */
    bool screen_fault = false;
    for (unsigned pass = 0; ok && pass < 2u; ++pass) {
        for (unsigned d = 0; ok && d < 2u; ++d)
            for (unsigned c = 0; ok && c < sizeof(codes); ++c)
                for (unsigned k = 0; ok && k < sizeof(skirts); ++k) {
                    rf_filter_t f = {codes[c], skirts[k], d == 1u};
                    const unsigned incumbents[3] = {best_nbw, narrow_nbw, wide_nbw};
                    if (!measure_setting(f, &width, &nbw, &quiet, &carrier,
                                         pass == 0u ? incumbents : NULL, &screen_fault)) { ok = false; fail = "probe_failed"; break; }
                    if (carrier) { ok = false; fail = "carrier_detected"; break; }   /* appeared mid-sweep */
                    if (quiet && nbw && width >= FILTER_MIN_KHZ && (!best_nbw || nbw < best_nbw)) {
                        best = f; best_nbw = nbw; best_width = width;
                    }
                    if (quiet && nbw && width >= FILTER_NARROW_KHZ && (!narrow_nbw || nbw < narrow_nbw)) {
                        narrow = f; narrow_nbw = nbw; narrow_width = width;
                    }
                    if (quiet && nbw && width >= FILTER_WIDE_KHZ && (!wide_nbw || nbw < wide_nbw)) {
                        wide = f; wide_nbw = nbw; wide_width = width;
                    }
                    if (quiet && nbw && width >= FILTER_XNARROW_MIN_KHZ &&
                        (!xnarrow_nbw || width < xnarrow_width || (width == xnarrow_width && nbw < xnarrow_nbw))) {
                        xnarrow = f; xnarrow_nbw = nbw; xnarrow_width = width;
                    }
                }
        if (!screen_fault || !ok || pass == 1u) break;
        printf("FILTER_CAL screen=fallback_full\n");
        best = narrow = wide = xnarrow = calibrated;
        best_nbw = best_width = narrow_nbw = narrow_width = wide_nbw = wide_width = 0;
        xnarrow_nbw = xnarrow_width = 0;
        s_cal_total += 2u * FILTER_CODES * FILTER_SKIRTS;
    }
    /* Re-check on the calibrated bytes before trusting the sweep. */
    if (ok && !(ok = measure_setting(calibrated, &width, &nbw, &quiet, &carrier, NULL, NULL) && quiet))
        fail = carrier ? "carrier_detected" : "recheck_not_quiet";
    /* Independent full rechecks of each distinct winner. Screening never
     * relaxes the stored width requirement, quiet gate or final precision. */
    rf_filter_t winners[4] = {best, narrow, wide, xnarrow};
    unsigned *widths[4] = {&best_width, &narrow_width, &wide_width, &xnarrow_width};
    unsigned *nbws[4] = {&best_nbw, &narrow_nbw, &wide_nbw, &xnarrow_nbw};
    static const unsigned minimum[4] = {FILTER_MIN_KHZ, FILTER_NARROW_KHZ, FILTER_WIDE_KHZ, FILTER_XNARROW_MIN_KHZ};
    for (unsigned k = 0; ok && k < 4u; ++k) {
        if (!*nbws[k]) continue;
        bool duplicate = false;
        for (unsigned j = 0; j < k; ++j)
            if (*nbws[j] && rf_filter_equal(winners[j], winners[k])) {
                *widths[k] = *widths[j]; *nbws[k] = *nbws[j]; duplicate = true; break;
            }
        if (!duplicate) {
            ++s_cal_total;
            if (!measure_setting(winners[k], &width, &nbw, &quiet, &carrier, NULL, NULL) || !quiet ||
                !cal_filter_screen_agrees(*widths[k], *nbws[k], width, nbw)) {
                ok = false; fail = "winner_recheck_failed"; break;
            }
            *widths[k] = width; *nbws[k] = nbw;
        }
        if (*widths[k] < minimum[k]) { ok = false; fail = "winner_width_failed"; }
    }
    /* The normal class must be a measured setting: a noise bandwidth of 0
     * means nothing reached the minimum width. */
    if (ok && !best_nbw) { ok = false; fail = "no_wide_setting"; }
    /* Back to the exact original tuning; a failed retune is retried and
     * reported, never assumed. */
    bool restored = rf_channel_index() == original;
    for (unsigned t = 0; !restored && t < 3u; ++t) {
        restored = rf_set_channel(original) == ESP_OK;
        if (!restored) vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!restored) printf("FILTER_CAL error: retune to %u MHz failed\n", s_original_mhz);
    if (rf_offset_khz() != original_offset) rf_set_offset_khz(original_offset);
    if (rf_offset_khz() != original_offset) {
        printf("FILTER_CAL error: offset %d kHz not restored (now %d)\n", original_offset, rf_offset_khz());
        restored = false;
    }
    /* Nothing is stored unless the receiver is back on its own tuning: a
     * sweep that ends elsewhere is discarded and tried on the next boot. */
    if (ok && !restored) { ok = false; fail = "restore_failed"; }
    if (ok) {
#define KHZ16(v) ((uint16_t)((v) > 65535u ? 65535u : (v)))
        const filter_blob_t cal = {FILTER_VERSION,
            {1u, best.bw20, best.skirt, best.code, KHZ16(best_width), KHZ16(best_nbw)},
            {narrow_nbw ? 1u : 0u, narrow.bw20, narrow.skirt, narrow.code, KHZ16(narrow_width), KHZ16(narrow_nbw)},
            {wide_nbw ? 1u : 0u, wide.bw20, wide.skirt, wide.code, KHZ16(wide_width), KHZ16(wide_nbw)},
            KHZ16(cal_nbw)};
        if (!rf_set_filter(best)) { ok = false; fail = "best_not_applied"; }
        else if (!blob_store("filter", &cal, sizeof(cal))) { ok = false; fail = "nvs_failed"; }
        else s_filter_cal = cal;
        const filter_xn_blob_t xn = {FILTER_VERSION, {xnarrow_nbw ? 1u : 0u, xnarrow.bw20, xnarrow.skirt,
                                                      xnarrow.code, KHZ16(xnarrow_width), KHZ16(xnarrow_nbw)}};
        if (ok) {   /* optional class: a failed store only leaves XNARROW out of the menu */
            if (blob_store("filter_xn", &xn, sizeof(xn))) s_filter_xn = xn;
            else printf("FILTER_CAL warning: XNARROW not stored\n");
        }
    }
    if (ok) {
        s_filter_result = "stored";
        printf("FILTER_CAL normal bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u | narrow bw=%s code=%d "
               "skirt=%u width_khz=%u nbw_khz=%u | wide bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u | "
               "calibrated_nbw_khz=%u\n", best.bw20 ? "BW20" : "BW40",
               best.code, best.skirt, best_width, best_nbw, narrow.bw20 ? "BW20" : "BW40", narrow.code,
               narrow.skirt, narrow_width, narrow_nbw, wide.bw20 ? "BW20" : "BW40", wide.code, wide.skirt,
               wide_width, wide_nbw, cal_nbw);
        printf("FILTER_CAL xnarrow bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u\n", xnarrow.bw20 ? "BW20" : "BW40",
               xnarrow.code, xnarrow.skirt, xnarrow_width, xnarrow_nbw);
    } else {
        if (!rf_set_filter(previous)) printf("FILTER_CAL error: previous filter not applied\n");
        s_filter_result = fail ? fail : "sweep_failed";
        printf("FILTER_CAL aborted=%s\n", s_filter_result);
    }
    printf("CAL_STEP step=filter ms=%lld settings=%u result=%s\n",
           (esp_timer_get_time() - s_cal_t0) / 1000, s_cal_done, ok ? "ok" : "failed");
    s_cal_total = 0;
    status_screen(ok ? "FILTER CALIBRATED" : "FILTER NOT CALIBRATED", ok ? "NARROW FILTER STORED" : s_filter_result,
                  ok ? "" : "USE MENU > CALIBRATE", ok ? 100 : -1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    receiver_set_gain(saved_gain);
    s_scan_active = false;
    receiver_resume();
    return ok;
}

const char *calibration_filter_result(void) { return s_filter_result; }

filter_entry_t filter_class_entry(int filter_class)
{
    const filter_entry_t *e = filter_class == FILTER_XNARROW ? &s_filter_xn.xnarrow :
                              filter_class == FILTER_NARROW ? &s_filter_cal.narrow :
                              filter_class == FILTER_WIDE ? &s_filter_cal.wide : &s_filter_cal.normal;
    const uint8_t ver = filter_class == FILTER_XNARROW ? s_filter_xn.version : s_filter_cal.version;
    if (ver == FILTER_VERSION && e->valid && e->nbw_khz) return *e;
    if (DCO_PRESET_BOARDS && filter_class >= 0 && (unsigned)filter_class < DCO_PRESET_CLASSES) {
        const dco_filter_preset_t *p = &c5vrx5_filter_preset[filter_class];
        return (filter_entry_t){1u, p->bw20, p->skirt, p->code, p->width_khz, p->nbw_khz};
    }
    if (filter_class == FILTER_NORMAL)
        return (filter_entry_t){1u, CAL_BASELINE_FILTER_BW20, CAL_BASELINE_FILTER_SKIRT, CAL_BASELINE_FILTER_CODE, 0u, 0u};
    return (filter_entry_t){0};
}

int dco_preset_class_of(uint16_t key)
{
    for (unsigned c = 0; c < DCO_PRESET_CLASSES; ++c) {
        const dco_filter_preset_t *p = &c5vrx5_filter_preset[c];
        if (dco_filter_key(p->code, (int)p->skirt, p->bw20 != 0) == key) return (int)c;
    }
    return -1;
}

/* Width class of the active analog filter, for model tables keyed on it
 * (cn_estimate.h): 10 narrow, 12 normal, 16 wide, 20 the chip's own ~20 MHz. */
int calibration_filter_class_mhz(void)
{
    rf_filter_t f, now = rf_filter();
    /* The sweep can select the same setting for normal and narrow (board:
     * both 13.125 MHz). Prefer normal, and do not call a measured >11 MHz
     * narrow entry 10 MHz. */
    const filter_entry_t normal = filter_class_entry(FILTER_NORMAL);
    const filter_entry_t narrow = filter_class_entry(FILTER_NARROW);
    if (normal.valid && filter_setting(FILTER_NORMAL, &f) && rf_filter_equal(f, now))
        return normal.width_khz && normal.width_khz < 11000u ? 10 : normal.width_khz >= 15000u ? 16 : 12;
    if (filter_setting(FILTER_NARROW, &f) && rf_filter_equal(f, now))
        return narrow.width_khz > 11000u ? 12 : 10;
    if (filter_setting(FILTER_WIDE, &f) && rf_filter_equal(f, now)) return 16;
    if (filter_setting(FILTER_XNARROW, &f) && rf_filter_equal(f, now)) return 10;
    return 20;
}

/* Measured noise bandwidth of the active filter (the startup sweep), 0
 * when unknown: C/N-AUTO converts to the 20 MHz scale with it. */
unsigned calibration_filter_nbw_khz(void)
{
    static const int classes[4] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE, FILTER_XNARROW};
    const rf_filter_t now = rf_filter();
    for (unsigned c = 0; c < 4u; ++c) {
        rf_filter_t f;
        const filter_entry_t e = filter_class_entry(classes[c]);
        if (e.valid && filter_setting(classes[c], &f) && rf_filter_equal(f, now)) return e.nbw_khz;
    }
    return s_filter_cal.calibrated_nbw_khz ? s_filter_cal.calibrated_nbw_khz : DCO_PRESET_CAL_NBW_KHZ;
}

/* Measured -3 dB width of the active filter (menu FILTER), 0 when unknown. */
unsigned calibration_filter_width_khz(void)
{
    static const int classes[4] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE, FILTER_XNARROW};
    const rf_filter_t now = rf_filter();
    for (unsigned c = 0; c < 4u; ++c) {
        rf_filter_t f;
        const filter_entry_t e = filter_class_entry(classes[c]);
        if (e.valid && filter_setting(classes[c], &f) && rf_filter_equal(f, now)) return e.width_khz;
    }
    return 0u;
}

unsigned calibration_narrow_width_khz(void)
{
    const filter_entry_t e = filter_class_entry(FILTER_NARROW);
    return e.valid ? e.width_khz : 0u;
}

bool filter_setting(int filter_class, rf_filter_t *out)
{
    const filter_entry_t e = filter_class_entry(filter_class);
    if (!e.valid) return false;
    *out = (rf_filter_t){e.code, e.skirt, e.bw20 != 0};
    return true;
}


/* Manual F: characterize the one active preset, without a grid search or
 * retune. Measure after DC is repaired; do not borrow a foreign table.
 * These actual widths, not the nominal preset name, go into board exports. */
void filter_measure_current(void)
{
    if (video_recent() || !receiver_pause("FILTER_CHECK")) return;
    const uint8_t restore = s_current_gain;
    const rf_filter_t live = rf_filter();
    receiver_set_gain(receiver_gain_top());
    const int64_t t0 = esp_timer_get_time();
    filter_blob_t cal = s_filter_cal;
    filter_xn_blob_t xn = s_filter_xn;
    cal.version = FILTER_VERSION;
    xn.version = FILTER_VERSION;
    if (!cal.calibrated_nbw_khz) cal.calibrated_nbw_khz = DCO_PRESET_CAL_NBW_KHZ;
    static const int classes[4] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE, FILTER_XNARROW};
    filter_entry_t *dst[4] = {&cal.normal, &cal.narrow, &cal.wide, &xn.xnarrow};
    unsigned ok_count = 0, last_w = 0, last_n = 0;
    for (unsigned k = 0; k < 4u; ++k) {
        rf_filter_t f;
        if (!filter_setting(classes[k], &f) || !rf_set_filter(f)) continue;
        dco_hold_for_scan();
        vTaskDelay(pdMS_TO_TICKS(50));
        cal_noise_t a = {0};
        unsigned width = 0, nbw = 0, env = 0;
        int clip = 0;
        const bool quiet = noise_like();
        const bool ok = quiet && rf_filter_applied() && noise_collect(&a, CAL_WINDOWS) &&
            cal_noise_finish(&a, &width, &nbw, &env, &clip) && cal_setting_quiet(env, clip) && width && nbw;
        if (ok) {
            *dst[k] = (filter_entry_t){1u, f.bw20, f.skirt, f.code, (uint16_t)width, (uint16_t)nbw};
            if (k == 0u) { last_w = width; last_n = nbw; }
            ++ok_count;
            printf("FILTER_CAL bw=%s code=%d skirt=%u width_khz=%u nbw_khz=%u env_x100=%u limit=%u clip_pm=%d\n",
                   f.bw20 ? "BW20" : "BW40", f.code, f.skirt, width, nbw, env, quiet_limit(clip), clip);
        }
    }
    (void)rf_set_filter(live);
    dco_hold_for_scan();
    if (ok_count) {
        if (blob_store("filter", &cal, sizeof(cal))) { s_filter_cal = cal; s_filter_result = "measured_current"; }
        if (xn.xnarrow.valid && blob_store("filter_xn", &xn, sizeof(xn))) s_filter_xn = xn;
    }
    printf("CAL_STEP step=filter_current ms=%lld result=%s width_khz=%u nbw_khz=%u\n",
           (esp_timer_get_time() - t0) / 1000, ok_count ? "measured" : "unknown", last_w, last_n);
    receiver_set_gain(restore);
    receiver_resume();
}

/* Boot: the stored filter calibration of this firmware, else the baseline. */
void cal_filter_load(void)
{
    rf_filter_t loaded;
    if (!blob_load("filter_xn", &s_filter_xn, sizeof(s_filter_xn))) memset(&s_filter_xn, 0, sizeof(s_filter_xn));
    if (blob_load("filter", &s_filter_cal, sizeof(s_filter_cal)) && s_filter_cal.version == FILTER_VERSION &&
        s_filter_cal.normal.valid && filter_setting(FILTER_NORMAL, &loaded)) {
        rf_set_filter(loaded);
        s_filter_result = "loaded";
    } else {
        memset(&s_filter_cal, 0, sizeof(s_filter_cal));
        const rf_filter_t baseline = {CAL_BASELINE_FILTER_CODE, CAL_BASELINE_FILTER_SKIRT, CAL_BASELINE_FILTER_BW20};
        s_filter_result = rf_set_filter(baseline) ? "baseline_unmeasured" : "baseline_not_applied";
    }
}

bool cal_filter_measured(void) { return s_filter_cal.normal.valid; }

/* A calibration of an earlier firmware generation: forget it and store the
 * empty blobs (calibration_generation). */
bool cal_filter_discard(void)
{
    const filter_blob_t none = {0};
    const filter_xn_blob_t no_xn = {0};
    s_filter_xn = no_xn;
    return blob_store("filter", &none, sizeof(none)) && blob_store("filter_xn", &no_xn, sizeof(no_xn));
}

/* Flight-log calibration record, bytes 6..29: the measured filter classes. */
void cal_filter_record(uint8_t *p)
{
    static const int classes[3] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE};
    for (unsigned c = 0; c < 3u; ++c) {
        rf_filter_t f;
        uint8_t *e = p + 6 + c * 8;
        if (!filter_setting(classes[c], &f)) continue;
        const filter_entry_t *x = c == 0 ? &s_filter_cal.normal : c == 1 ? &s_filter_cal.narrow : &s_filter_cal.wide;
        if (!x->valid) continue;
        e[0] = 1; e[1] = f.bw20; e[2] = (uint8_t)(int8_t)f.code; e[3] = (uint8_t)f.skirt;
        flog_put16(e + 4, x->width_khz); flog_put16(e + 6, x->nbw_khz);
    }
}

void cal_filter_print(rf_filter_t f)
{
    const filter_entry_t fn = filter_class_entry(FILTER_NORMAL), fnar = filter_class_entry(FILTER_NARROW),
                         fw = filter_class_entry(FILTER_WIDE);
    printf("FILTER %s active bw=%s code=%d skirt=%u | normal width_khz=%u nbw_khz=%u | narrow valid=%u "
           "width_khz=%u nbw_khz=%u | wide valid=%u width_khz=%u nbw_khz=%u | calibrated_nbw_khz=%u\n",
           s_filter_result, f.bw20 ? "BW20" : "BW40", f.code, f.skirt, fn.width_khz,
           fn.nbw_khz, fnar.valid, fnar.width_khz, fnar.nbw_khz, fw.valid, fw.width_khz,
           fw.nbw_khz, s_filter_cal.calibrated_nbw_khz ? s_filter_cal.calibrated_nbw_khz : DCO_PRESET_CAL_NBW_KHZ);
    uint8_t hw[8] = {0}, want[8] = {0};
    const int bad = phy_filter_hw_mismatch(hw, want);
    printf("PHY11P want=1 hw_enabled=%d (-1=unsupported; analog profile checked below)\n", phy_11p_hw_enabled());
    printf("DFILT want=%d hw_mode=%d (-1=unsupported)\n",
           phy_dfilt_mode(), phy_dfilt_hw_mode());
    printf("FILTER_HW mismatch=%d hw=%02x%02x%02x%02x%02x%02x%02x%02x want=%02x%02x%02x%02x%02x%02x%02x%02x\n", bad,
           hw[0], hw[1], hw[2], hw[3], hw[4], hw[5], hw[6], hw[7],
           want[0], want[1], want[2], want[3], want[4], want[5], want[6], want[7]);
}
