/* C5VRX-5 by Twotoz and the C5VRX contributors: passive field diagnostics.
 *
 * Nothing here touches the receive chain: no retune, gain, filter, DC,
 * lane or tracker write, and no extra IQ copy. The control task hands over
 * per 50 ms window it already processed whether the AFC could measure it
 * and whether it held burst-confirmed sync. Coverage and sync use ALL
 * observation windows as denominator; conditional sync is reported separately.
 * A bounded recorder preserves 16 pre-onset and 16 post-onset observations
 * at the existing control cadence (nominal 50 ms, actual timestamps printed).
 * These are sparse IQ/AFC observations, not continuous image-quality samples.
 * Console printing never blocks the control task or the hardware datapath.
 *
 *   FIELD_DIAG        one line per second, after console R (stream toggle)
 *   FIELD_DIAG_EVENT  dropout start (category) and end (duration), always
 *   RANGE_DIAG        health report, console D
 *
 * tools/field_diag_compare.py compares two captured logs. */
#include "esp_attr.h"
#include "receiver.h"
#include "rx_state.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cn_estimate.h"
#include "diag_logic.h"
#include "diag_trace.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "phy.h"
#include "rf.h"

#define DIAG_HISTORY   64u
#define DIAG_LOOKBACK  8u      /* seconds handed to the classifier */
#define DIAG_EVENTS    4u

typedef struct {
    uint32_t t_s, duration_s;
    int category;
    diag_record_t last_ok, drop;
    bool open;
} diag_event_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
/* Control-task only (no DMA, no ISR): LP SRAM, which GDMA cannot use. The
 * 4 KiB of HP SRAM it frees is what the interlaced menu chain needed next to
 * the (since removed) sync flywheel (board 2026-10-18: ~15 KiB DMA-capable
 * free, 18 KiB needed). */
static RTC_FAST_ATTR diag_record_t s_hist[DIAG_HISTORY];
static EXT_RAM_BSS_ATTR diag_trace_t s_trace;   /* control task and console only: PSRAM */
static uint32_t s_seq;                  /* records written */
static diag_event_t s_events[DIAG_EVENTS];
static uint32_t s_event_seq;
static uint32_t s_dropouts[DIAG_CATEGORIES];
static volatile bool s_stream;

/* Accumulator of the current second (control task only). */
static struct {
    int64_t start_us;
    unsigned windows, measured, video, gain_sum, stuck;
    uint8_t gain_min, gain_max;
    long coh, p50, origin, clip, env;
    uint32_t faults;
    bool owned;           /* menu / calibration / scan owned TX or gain */
    bool afc_step;
} s_acc;
static bool s_in_dropout;
/* Receiver state supervisor, shadow mode (rx_state.h): names the situation
 * each second for D and the flight log; never writes the PHY. */
static rx_state_t s_rx;
static bool s_rx_ready;
static void rx_supervise(const diag_record_t *r)
{
    static uint32_t overloads, one_sided;
    static bool have;
    if (!s_rx_ready) { rx_state_init(&s_rx); s_rx_ready = true; }
    vq_t vq;
    control_video_quality(&vq);
    int dc[2], off[2];
    uint32_t servo;
    uint8_t sph;
    int8_t temp;
    calibration_flog_live(dc, off, &servo, &sph, &temp);
    const int dcm = (int)sqrtf((float)dc[0] * dc[0] + (float)dc[1] * dc[1]);
    const rx_obs_t o = {
        .video_pct = r->video_pct, .cn_x10 = r->cn_x10, .vq_known = vq.known && vq.lines, .vq_snr_x10 = vq.snr_x10,
        .overloads = (uint16_t)(have ? s_gain.overloads - overloads : 0u),
        .one_sided_clips = (uint16_t)(have ? s_gain.one_sided_clips - one_sided : 0u),
        .dc_mcells = dcm, .fault = (r->flags & DIAG_F_FAULT) != 0 || receiver_paused(),
        .owned = s_menu_active || s_scan_active || calibration_in_startup(),
    };
    overloads = s_gain.overloads; one_sided = s_gain.one_sided_clips; have = true;
    receiver_level_roll(s_rx.state == RX_ACQUIRE && s_rx.reason == 1u, r->video_pct >= 20u);   /* reason 1: no_carrier */
    const uint8_t before = s_rx.state;
    (void)rx_state_step(&s_rx, &o);
    /* Raw IQ at the range edge (the blackbox's ground truth for the C/N
     * estimator and the sensitivity model): on entering EDGE or RECOVER,
     * and every 30 s while the receiver stays at the edge. */
    if (s_rx.state != before && (s_rx.state == RX_EDGE || s_rx.state == RX_RECOVER))
        flightlog_snapshot_request(s_rx.state == RX_EDGE ? FLOG_SNAP_EDGE : FLOG_SNAP_RECOVER);
    else if (s_rx.state == RX_EDGE && s_rx.dwell_s % 30u == 0u)
        flightlog_snapshot_request(FLOG_SNAP_EDGE);
}
static uint8_t s_last_category;
static uint32_t s_segment_start_seq;
static uint32_t s_seen_faults;

static uint8_t chain_flags(uint8_t gain, uint32_t faults)
{
    uint8_t f = 0;
    if (!rf_filter_applied()) f |= DIAG_F_FILTER_MISMATCH;
    if (!calibration_sphase_settled()) f |= DIAG_F_SPHASE_BAD;
    if (!calibration_dc_ok(gain)) f |= DIAG_F_NO_DC;
    if (faults) f |= DIAG_F_FAULT;
    if (s_afc_locked) f |= DIAG_F_AFC_LOCKED;
    if (!rf_freq_in_spec(rf_frequency_mhz())) f |= DIAG_F_OUT_OF_SPEC;
    return f;
}

/* Read-only metadata. Quantised envelope/coherence are existing observer
 * statistics, NOT calibrated raw ADC power or independent C/N confidence. */
static void snapshot(diag_record_t *r, uint8_t gain, uint32_t faults)
{
    const arc_gain_table_t *table = rf_gain_table();
    arc_gain_tuple_t tuple = {0};
    bool tuple_ok = arc_gain_tuple_decode(table, gain, &tuple);
    const rf_filter_t f = rf_filter();
    r->gain = gain;
    r->gain_max = receiver_gain_top();   /* the useful top: the table maximum, or under an LNA's noise */
    r->rf_stage = tuple_ok ? tuple.rf_stage : 0xff;
    r->rf_code = tuple.rf_code;
    r->bb_code = tuple.bb_code;
    r->fine_code = tuple.fine_code;
    r->frequency_mhz = rf_frequency_mhz();
    r->profile = (uint8_t)profile_active();
    r->filter_code = f.code;
    r->filter_skirt = f.skirt;
    r->filter_bw20 = f.bw20;
    r->offset_khz = (int16_t)rf_offset_khz();
    r->cfo_khz = (int16_t)s_cfo_khz;
    r->p95 = (int16_t)s_obs_p95;
    /* Reserved V1 kp/LP slots; P64 identity is recorded in output telemetry. */
    r->kp_level = -1;
    r->lp = false;
    r->gain_writes = s_gain.writes;
    r->fade_holds = s_gain.fade_holds;
    r->overloads = s_gain.overloads;
    r->faults = faults;
    r->flags = chain_flags(gain, faults);
}

/* Picture measure (picture_sync.h): sync pulses found / expected over the
 * control windows of the second, apart from the AFC-gated video_pct. */
static uint32_t s_pic_found, s_pic_expected, s_pic_windows;
void diag_picture(unsigned found, unsigned expected)
{
    s_pic_found += found;
    s_pic_expected += expected;
    ++s_pic_windows;
}

static void close_second(int64_t now)
{
    diag_record_t r = {0};
    r.t_s = (uint32_t)(now / 1000000);
    r.t_ms = (uint32_t)(now / 1000);
    diag_coverage(&r, s_acc.windows, s_acc.measured, s_acc.video);
    r.picture_pct = s_pic_expected ? (uint8_t)(s_pic_found * 100u / s_pic_expected) : DIAG_PICTURE_UNKNOWN;
    r.picture_windows = (uint8_t)(s_pic_windows > 255u ? 255u : s_pic_windows);
    s_pic_found = s_pic_expected = s_pic_windows = 0;
    snapshot(&r, (uint8_t)((s_acc.gain_sum + s_acc.windows / 2u) / s_acc.windows), s_acc.faults);
    r.coherence = (int16_t)(s_acc.coh / (long)s_acc.windows);
    /* Mostly a fixed IQ point this second: no carrier to estimate. */
    r.cn_x10 = (int16_t)(s_acc.stuck * 2u >= s_acc.windows ? CN_UNKNOWN_X10
                         : cn_estimate_env_x10((int)(s_acc.env / (long)(s_acc.windows - s_acc.stuck)), calibration_filter_class_mhz()));
    r.p50 = (int16_t)(s_acc.p50 / (long)s_acc.windows);
    r.origin_pm = (int16_t)(s_acc.origin / (long)s_acc.windows);
    r.clip_pm = (int16_t)(s_acc.clip / (long)s_acc.windows);
    r.flags |= s_acc.afc_step ? DIAG_F_AFC_STEP : 0u;

    taskENTER_CRITICAL(&s_lock);
    s_hist[s_seq % DIAG_HISTORY] = r;
    ++s_seq;
    taskEXIT_CRITICAL(&s_lock);
    rx_supervise(&r);
    flightlog_second(&r, s_acc.gain_min, s_acc.gain_max, s_in_dropout ? (uint8_t)(s_last_category + 1u) : 0u,
                     rx_state_byte(&s_rx));

    if (!s_in_dropout && diag_video_lost(&r)) {
        diag_record_t look[DIAG_LOOKBACK];
        unsigned available = s_seq - s_segment_start_seq;
        unsigned n = available < DIAG_LOOKBACK ? available : DIAG_LOOKBACK;
        for (unsigned k = 0; k < n; ++k) look[k] = s_hist[(s_seq - n + k) % DIAG_HISTORY];
        int last_ok = -1;
        for (int k = (int)n - 2; k >= 0; --k) if (diag_video_ok(&look[k])) { last_ok = k; break; }
        if (last_ok < 0) return;            /* no picture before: not a dropout */
        s_in_dropout = true;
        diag_event_t e = {.t_s = r.t_s, .category = diag_classify(look, n), .last_ok = look[last_ok], .drop = r,
                          .open = true};
        s_last_category = (uint8_t)e.category;
        flightlog_event(FLOG_E_DROPOUT, &s_last_category, 1);
        flightlog_snapshot_request(FLOG_SNAP_DROPOUT);
        taskENTER_CRITICAL(&s_lock);
        s_events[s_event_seq % DIAG_EVENTS] = e;
        ++s_event_seq;
        ++s_dropouts[e.category];
        taskEXIT_CRITICAL(&s_lock);
    } else if (s_in_dropout && diag_video_ok(&r)) {
        s_in_dropout = false;
        taskENTER_CRITICAL(&s_lock);
        diag_event_t *e = &s_events[(s_event_seq - 1u) % DIAG_EVENTS];
        e->duration_s = r.t_s - e->t_s;
        e->open = false;
        const uint32_t duration = e->duration_s;
        taskEXIT_CRITICAL(&s_lock);
        uint8_t d[4];
        flog_put32(d, duration);
        flightlog_event(FLOG_E_RECOVERED, d, sizeof(d));
    }
}

void diag_window(bool measured, bool video_valid, bool afc_step)
{
    const int64_t now = esp_timer_get_time();
    const bool owned = s_menu_active || s_scan_active || receiver_paused();
    static uint32_t trace_faults, trace_epoch;
    const uint32_t faults_now = transport_fault_count();
    diag_record_t point = {0};
    point.t_ms = (uint32_t)(now / 1000);
    point.t_s = (uint32_t)(now / 1000000);
    point.measured = measured;
    diag_coverage(&point, 1u, measured, measured && video_valid);
    snapshot(&point, s_current_gain, faults_now - trace_faults);
    trace_faults = faults_now;
    point.flags |= afc_step ? DIAG_F_AFC_STEP : 0u;
    point.coherence = (int16_t)s_obs_coherence;
    point.cn_x10 = (int16_t)receiver_obs_cn_x10();
    point.p50 = (int16_t)s_obs_p50;
    point.origin_pm = (int16_t)s_obs_origin_pm;
    point.clip_pm = (int16_t)s_obs_clip_pm;
    /* Retunes are context boundaries, including a channel changed to the
     * same frequency. Never attach old pre-trigger evidence to a new link. */
    bool boundary = trace_epoch != s_control_epoch;
    trace_epoch = s_control_epoch;
    taskENTER_CRITICAL(&s_lock);
    if (boundary) diag_trace_boundary(&s_trace);
    diag_trace_push(&s_trace, &point, owned);
    taskEXIT_CRITICAL(&s_lock);
    /* A gap (scan, stalled task) is not a second of reception. */
    if (!s_acc.start_us || now - s_acc.start_us > 2000000) {
        memset(&s_acc, 0, sizeof(s_acc));
        s_acc.start_us = now;
        s_in_dropout = false;
        s_segment_start_seq = s_seq;
        s_pic_found = s_pic_expected = s_pic_windows = 0;
    }
    if (owned || boundary) s_acc.owned = true;
    ++s_acc.windows;
    s_acc.measured += measured;
    s_acc.afc_step |= afc_step;
    s_acc.video += measured && video_valid;
    s_acc.gain_sum += s_current_gain;
    if (s_acc.windows == 1u || s_current_gain < s_acc.gain_min) s_acc.gain_min = s_current_gain;
    if (s_acc.windows == 1u || s_current_gain > s_acc.gain_max) s_acc.gain_max = s_current_gain;
    s_acc.coh += s_obs_coherence;
    if (!s_obs_stuck) s_acc.env += s_obs_env_ratio_x100;
    s_acc.stuck += s_obs_stuck;
    s_acc.p50 += s_obs_p50;
    s_acc.origin += s_obs_origin_pm;
    s_acc.clip += s_obs_clip_pm;
    if (now - s_acc.start_us < 1000000) return;
    const uint32_t faults = transport_fault_count();
    s_acc.faults = faults - s_seen_faults;
    s_seen_faults = faults;
    /* A second the menu, a calibration or the scanner owned says nothing
     * about the RF link; it also ends any dropout bookkeeping. */
    /* (Its picture windows go too: walk 300 logged 85-98 in the second after
     * an owned stretch.) */
    if (s_acc.owned) { s_in_dropout = false; s_segment_start_seq = s_seq; s_pic_found = s_pic_expected = s_pic_windows = 0; }
    else close_second(now);
    memset(&s_acc, 0, sizeof(s_acc));
    s_acc.start_us = now;
}

/* ---- Console side --------------------------------------------------------- */
static void print_record(const char *tag, const diag_record_t *r)
{
    printf("%s t=%lu t_ms=%lu sync_valid_pct=%u measurement_coverage_pct=%u sync_given_measured_pct=%u "
           "gain=%u max=%u rf_stage=%u rf_code=%u bb=%u fine=%u p50=%d p95=%d coherence=%d origin_pm=%d clip_pm=%d "
           "cn_est_x10=%d cn_model_valid=%u offset_khz=%d cfo_khz=%d afc=%u afc_step=%u filter_ok=%u "
           "filter_code=%d skirt=%u bw20_requested=%u freq=%u profile=%s sphase_ok=%u dc_ok=%u "
           "fault=%u faults_delta=%lu gain_writes=%lu fade_holds=%lu overloads=%lu in_spec=%u fw=%s\n", tag,
           (unsigned long)r->t_s, (unsigned long)r->t_ms, r->video_pct, r->coverage_pct, r->sync_measured_pct,
           r->gain, r->gain_max, r->rf_stage, r->rf_code, r->bb_code, r->fine_code,
           r->p50, r->p95, r->coherence, r->origin_pm, r->clip_pm, r->cn_x10, r->cn_x10 != CN_UNKNOWN_X10,
           r->offset_khz, r->cfo_khz, !!(r->flags & DIAG_F_AFC_LOCKED), !!(r->flags & DIAG_F_AFC_STEP),
           !(r->flags & DIAG_F_FILTER_MISMATCH), r->filter_code, r->filter_skirt, r->filter_bw20,
           r->frequency_mhz, profile_name(r->profile), !(r->flags & DIAG_F_SPHASE_BAD),
           !(r->flags & DIAG_F_NO_DC), !!(r->flags & DIAG_F_FAULT), (unsigned long)r->faults,
           (unsigned long)r->gain_writes, (unsigned long)r->fade_holds, (unsigned long)r->overloads,
           !(r->flags & DIAG_F_OUT_OF_SPEC), firmware_id());
}

static void print_event(const diag_event_t *e)
{
    printf("FIELD_DIAG_EVENT dropout t=%lu category=%s duration_s=%s%lu channel=%s freq=%u profile=%s\n",
           (unsigned long)e->t_s, diag_category_name(e->category), e->open ? ">=" : "",
           (unsigned long)(e->open ? (uint32_t)(esp_timer_get_time() / 1000000) - e->t_s : e->duration_s),
           "snapshot", e->drop.frequency_mhz, profile_name(e->drop.profile));
    print_record("FIELD_DIAG_EVENT last_ok", &e->last_ok);
    print_record("FIELD_DIAG_EVENT drop", &e->drop);
}

void diag_toggle_stream(void)
{
    s_stream = !s_stream;
    firmware_print();
    printf("FIELD_DIAG stream=%u (one line per second; R toggles)\n", s_stream);
}

/* Console drains one frozen event without holding the lock during USB I/O.
 * Producer keeps the oldest unread event; a slow host loses NEW events,
 * never reception. No large stack copy, no unbounded producer-side queue. */
static void print_trace(void)
{
    unsigned count, trigger;
    uint32_t sequence, skipped;
    taskENTER_CRITICAL(&s_lock);
    bool ready = s_trace.ready;
    count = s_trace.count;
    trigger = s_trace.trigger;
    sequence = s_trace.sequence;
    skipped = s_trace.skipped;
    taskEXIT_CRITICAL(&s_lock);
    if (!ready) return;
    printf("FIELD_TRACE onset id=%lu samples=%u trigger=%u skipped=%lu "
           "cadence=control_nominal_50ms evidence=afc_sync_not_picture cn=model_only\n",
           (unsigned long)sequence, count, trigger, (unsigned long)skipped);
    for (unsigned k = 0; k < count; ++k) {
        diag_record_t r;
        taskENTER_CRITICAL(&s_lock);
        r = s_trace.event[k];
        taskEXIT_CRITICAL(&s_lock);
        print_record("FIELD_TRACE", &r);
        flightlog_trace_add(&r, k, count);
    }
    printf("FIELD_TRACE end id=%lu\n", (unsigned long)sequence);
    taskENTER_CRITICAL(&s_lock);
    s_trace.ready = false;
    taskEXIT_CRITICAL(&s_lock);
}

void diag_poll(void)
{
    print_trace();
    static uint32_t printed_seq, printed_events, printed_closed;
    uint32_t seq, events;
    diag_record_t r;
    diag_event_t e;
    for (;;) {
        taskENTER_CRITICAL(&s_lock);
        seq = s_seq;
        if (seq - printed_seq > DIAG_HISTORY) printed_seq = seq - DIAG_HISTORY;
        bool have = printed_seq != seq;
        if (have) r = s_hist[printed_seq % DIAG_HISTORY];
        taskEXIT_CRITICAL(&s_lock);
        if (!have) break;
        ++printed_seq;
        if (s_stream) print_record("FIELD_DIAG", &r);
    }
    taskENTER_CRITICAL(&s_lock);
    events = s_event_seq;
    bool fresh = printed_events != events;
    if (fresh) e = s_events[(events - 1u) % DIAG_EVENTS];
    bool closed = !fresh && events && printed_closed != events && !s_events[(events - 1u) % DIAG_EVENTS].open;
    if (closed) e = s_events[(events - 1u) % DIAG_EVENTS];
    taskEXIT_CRITICAL(&s_lock);
    if (fresh) { printed_events = events; print_event(&e); }
    else if (closed) {
        printed_closed = events;
        printf("FIELD_DIAG_EVENT recovered t=%lu category=%s duration_s=%lu\n", (unsigned long)(e.t_s + e.duration_s),
               diag_category_name(e.category), (unsigned long)e.duration_s);
    }
}

/* One-command health report: what is really active, what is missing, and
 * plain-language verdicts. Read-only. */
void diag_report(void)
{
    const uint16_t mhz = rf_frequency_mhz();
    const rf_filter_t f = rf_filter();
    rf_filter_t normal, narrow, wide;
    const bool have_normal = filter_setting(FILTER_NORMAL, &normal), have_narrow = filter_setting(FILTER_NARROW, &narrow);
    const bool have_wide = filter_setting(FILTER_WIDE, &wide);
    const char *cls = have_narrow && rf_filter_equal(f, narrow) ? "narrow" :
                      have_normal && rf_filter_equal(f, normal) ? "normal" :
                      have_wide && rf_filter_equal(f, wide) ? "wide" : "uncalibrated";
    const arc_gain_table_t *table = rf_gain_table();
    const uint8_t g = s_current_gain;
    arc_gain_tuple_t t = {0};
    const bool tuple_ok = arc_gain_tuple_decode(table, g, &t);
    const uint8_t top = arc_gain_highest_rf_stage_start(table);
    const uint8_t gtop = receiver_gain_top();

    firmware_print();
    printf("RANGE_DIAG channel=%s freq=%u in_spec=%u%s\n", rf_channel()->name, mhz, rf_freq_in_spec(mhz),
           rf_freq_in_spec(mhz) ? "" : " WARNING above the documented C5 range (5180-5885 MHz), not characterised");
    printf("RANGE_DIAG filter requested code=%d skirt=%u bw=%s class=%s applied=%u (read back code=%d skirt=%d) "
           "calibration=%s\n", f.code, f.skirt, f.bw20 ? "BW20" : "BW40", cls, rf_filter_applied(), phy_filter_code(),
           phy_filter_skirt(), calibration_filter_result());
    profile_print();
    printf("RANGE_DIAG rx_state=%s reason=%s dwell_s=%lu track_s=%lu edge_s=%lu recover_s=%lu acquire_s=%lu "
           "(shadow supervisor: names the situation, never acts)\n", rx_state_name(s_rx.state),
           rx_reason_name(s_rx.reason), (unsigned long)s_rx.dwell_s, (unsigned long)s_rx.seconds[RX_TRACK],
           (unsigned long)s_rx.seconds[RX_EDGE], (unsigned long)s_rx.seconds[RX_RECOVER],
           (unsigned long)s_rx.seconds[RX_ACQUIRE]);
    printf("RANGE_DIAG afc locked=%u offset_khz=%d cfo_khz=%d\n", s_afc_locked, rf_offset_khz(), s_cfo_khz);
    printf("RANGE_DIAG gain index=%u max=%u rf_stage=%u rf_code=%u bb=%u fine=%u top_rf_stage_from=%u at_max=%u "
           "state=%u useful_top=%u\n", g, table->max_index, tuple_ok ? t.rf_stage : 0xffu, tuple_ok ? t.rf_code : 0u,
           tuple_ok ? t.bb_code : 0u, tuple_ok ? t.fine_code : 0u, top, g >= gtop, (unsigned)s_gain.state, gtop);
    printf("RANGE_DIAG chain sphase=%s dc_ok=%u dc_held=%u lane=%s faults=%lu\n", calibration_sphase_state(),
           calibration_dc_ok(g), phy_dco_held(), rf_lane_name(rf_lane()), (unsigned long)transport_fault_count());
    const int cls_mhz = calibration_filter_class_mhz(), cn = receiver_obs_cn_x10();
    const bool video_recent = s_last_video_us && esp_timer_get_time() - s_last_video_us < 2000000;
    printf("RANGE_DIAG iq p50=%d coherence=%d cn_est_x10=%d (model, %d MHz class) origin_pm=%d clip_pm=%d "
           "video_recent=%u\n", s_obs_p50, s_obs_coherence, cn, cls_mhz, s_obs_origin_pm, s_obs_clip_pm,
           video_recent);

    /* Verdicts: evidence, not a measured cause. */
    bool any = false;
#define VERDICT(cond, text) do { if (cond) { printf("RANGE_DIAG verdict=%s\n", text); any = true; } } while (0)
    VERDICT(!have_normal, "NOT_CALIBRATED filter (safe chip default in use; MENU CALIBRATE measures on request)");
    VERDICT(!rf_filter_applied(), "CALIBRATION_OR_PHY filter not read back as requested");
    VERDICT(!calibration_sphase_settled(), "CALIBRATION_OR_PHY sampling phase unverified (manual CALIBRATE only)");
    VERDICT(!calibration_dc_ok(g), "NOT_CALIBRATED DC row unmeasured (factory baseline; not evidence of a bad map)");
    char need[24];
    if (calibration_dc_needed(need, sizeof(need))) {
        printf("RANGE_DIAG verdict=DC_CAL_NEEDED profile=%s (in-flight DC health; manual calibration only, "
               "or CALIBRATE now)\n", need);
        any = true;
    }
    VERDICT(!receiver_gain_mode_auto(receiver_gain_mode()),
            "GAIN_FIXED a fixed gain is saved across boots and costs range (menu GAIN or console G to AUTO)");
    VERDICT(!rf_freq_in_spec(mhz), "UNKNOWN out-of-spec channel: compare with a channel <= 5885 MHz");
    /* "Maximum" is the useful top (receiver_gain_top): the table maximum, or
     * behind an external LNA the highest row whose noise does not fold. */
    VERDICT(g >= gtop && cn != CN_UNKNOWN_X10 && cn <= DIAG_FLOOR_CN_X10,
            "POSSIBLE_RF_FLOOR top gain and C/N estimate <= 2 dB (no carrier, or at the receiver noise)");
    VERDICT(cn == CN_UNKNOWN_X10, "UNKNOWN C/N model has no carrier evidence; not measured RF sensitivity");
    VERDICT(g >= gtop && cn > DIAG_FLOOR_CN_X10 && cn < DIAG_WEAK_CN_X10,
            "WEAK_CARRIER top gain, C/N estimate 2-8 dB: near the usable-picture threshold");
    VERDICT(g >= table->max_index && s_obs_p50 <= 4 && s_obs_origin_pm >= 650,
            "QUANTISER_LIMITED noise below one ultrafine step at the table maximum: check with the VTX off");
    VERDICT(video_recent && g + DIAG_FADE_HEADROOM <= gtop && cn >= DIAG_WEAK_CN_X10,
            "VIDEO_EVIDENCE strong C/N estimate with gain headroom");
    if (!any) printf("RANGE_DIAG verdict=UNKNOWN no specific evidence\n");
#undef VERDICT

    /* History: last 60 s and the dropouts of this boot. */
    /* Console is the sole caller. Snapshot only aggregates and ten scalar
     * samples; a second complete history wastes 4 KiB of scarce internal RAM.
     * Keep event records off the 6-KiB console stack. */
    static diag_event_t ev[DIAG_EVENTS];
    uint32_t seq, events, drops[DIAG_CATEGORIES];
    int cn_history[10], gn[10];
    unsigned n, m, ok = 0, max_gain = 0;
    taskENTER_CRITICAL(&s_lock);
    seq = s_seq;
    events = s_event_seq;
    n = seq < 60u ? seq : 60u;
    m = seq < 10u ? seq : 10u;
    for (unsigned k = 0; k < n; ++k) {
        const diag_record_t *r = &s_hist[(seq - 1u - k) % DIAG_HISTORY];
        ok += diag_video_ok(r);
        max_gain += r->gain + 1u >= r->gain_max;
        if (k < m) { cn_history[k] = r->cn_x10; gn[k] = r->gain; }
    }
    memcpy(ev, s_events, sizeof(ev));
    memcpy(drops, s_dropouts, sizeof(drops));
    taskEXIT_CRITICAL(&s_lock);
    /* Antenna / placement A/B: hold one position for 10 s, press D, change
     * one thing, repeat. Medians, so one fade does not decide. */
    {
        for (unsigned a2 = 1; a2 < m; ++a2)
            for (unsigned b = a2; b > 0 && cn_history[b - 1] > cn_history[b]; --b) { int t2 = cn_history[b]; cn_history[b] = cn_history[b - 1]; cn_history[b - 1] = t2; }
        for (unsigned a2 = 1; a2 < m; ++a2)
            for (unsigned b = a2; b > 0 && gn[b - 1] > gn[b]; --b) { int t2 = gn[b]; gn[b] = gn[b - 1]; gn[b - 1] = t2; }
        if (m) printf("RANGE_DIAG last_10s seconds=%u cn_est_x10 median=%d min=%d gain median=%d (model only; compare repeated controlled setups)\n", m, cn_history[m / 2], cn_history[0], gn[m / 2]);
    }
    printf("RANGE_DIAG history seconds=%u sync_evidence_ok_pct=%u at_max_gain_pct=%u dropouts=%lu", n, n ? ok * 100u / n : 0u,
           n ? max_gain * 100u / n : 0u, (unsigned long)events);
    for (int c = 0; c < DIAG_CATEGORIES; ++c) printf(" %s=%lu", diag_category_name(c), (unsigned long)drops[c]);
    printf("\n");
    for (uint32_t k = events > DIAG_EVENTS ? events - DIAG_EVENTS : 0; k < events; ++k) print_event(&ev[k % DIAG_EVENTS]);
}
