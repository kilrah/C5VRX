/* C5VRX-5 by Twotoz and the C5VRX contributors: 50 ms control loop.
 * BOOT button and menu, AFC acquisition, PAL/NTSC detection, signal
 * strength and the channel scanner. Observes completed RX descriptors only. */
#include "receiver.h"
#define PICTURE_YIELD() taskYIELD()   /* the gain observer shares priority 3 */
#include "picture_sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "afc_ctrl.h"
#include "output_level.h"
#include "profile_logic.h"
#include "cn_estimate.h"
#include "analog_video_detect.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "phase8_gain_lut.h"
#include "phy.h"
#include "snapshot.h"
#include "rf.h"
#include "flightlog.h"
#include "experimental.h"
#include "flightlog_format.h"

#define BOOT_BUTTON      GPIO_NUM_28    /* XIAO ESP32-C5 BOOT */
#define TICK_MS          50
#define AFC_LOCKED_PERIOD_US 300000   /* LCD build: AFC window period while locked */
#define LONG_TICKS       12             /* 0.6 s */
#define MENU_IDLE_TICKS  600            /* 30 s without a press closes the menu */
#define DAC_IDLE_TICKS   6000           /* 5 min on the bench-only DAC TEST page */
#define RETUNE_SETTLE_TICKS 10          /* 500 ms after a PHY transaction */
#define SCAN_PAIRS       4092u
#define CONTROL_STACK_BYTES 6144u

typedef struct {
    StaticTask_t task;
    StackType_t stack[CONTROL_STACK_BYTES/sizeof(StackType_t)] __attribute__((aligned(portBYTE_ALIGNMENT)));
} control_memory_t;
static control_memory_t *s_control_memory;

bool control_prepare(void)
{
    if(!s_control_memory) {
        s_control_memory=heap_caps_aligned_alloc(portBYTE_ALIGNMENT,sizeof(*s_control_memory),
            MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        if(s_control_memory)memset(s_control_memory,0,sizeof(*s_control_memory));
    }
    return s_control_memory!=NULL;
}

volatile bool s_menu_active;
static bool menu_open_checked(void)
{
    if (!menu_open()) return false;
    s_menu_cursor = 0;
    s_cal_sub = -1;
    return true;
}
volatile int s_signal_strength;
volatile bool s_scan_active;
volatile unsigned s_scan_progress;
volatile int s_cfo_khz;
volatile bool s_afc_locked;
volatile bool s_afc_enabled = true;
volatile std_mode_t s_std_mode = STD_MODE_AUTO;
volatile video_standard_t s_video_std = VIDEO_STD_NTSC;
volatile bool s_detected_std_valid;
volatile video_standard_t s_detected_std = VIDEO_STD_NTSC;
volatile int64_t s_last_video_us;
QueueHandle_t s_commands;

/* One control-task scratch buffer: the control window (gain), then the
 * longer AFC window over it, and during a scan (receiver paused) a separate
 * endpoint area. Saves the scan's malloc. */
#define SCAN_BUF_OFFSET 4096u
#define AFC_WINDOW_BYTES C5V4_LEVEL_SAMPLE_BYTES
#define CONTROL_SCRATCH_BYTES (SCAN_BUF_OFFSET + SCAN_PAIRS > AFC_WINDOW_BYTES ? \
                               SCAN_BUF_OFFSET + SCAN_PAIRS : AFC_WINDOW_BYTES)
static uint8_t s_control_scratch[CONTROL_SCRATCH_BYTES];
static uint8_t *const s_window = s_control_scratch;
static uint8_t *const s_afc_window = s_control_scratch;
static uint8_t s_pal_votes, s_ntsc_votes;
static uint8_t s_last_std = 0xFF;      /* last stable standard (NVS) */
static unsigned s_afc_corrections, s_afc_weak_skips;
typedef struct {
    afc2_evidence_t history;
    uint64_t sampled_us, qualified_us;
    uint32_t context, qualified_gain_epoch;
    int32_t raw_porch_khz, residual_khz;
    int cn_x10;
    bool latest_valid, sticky_lock;
    const char *window, *held;
} afc_evidence_snapshot_t;
static afc_evidence_snapshot_t s_afc_evidence = {.window="boot", .held="boot", .cn_x10=CN_UNKNOWN_X10};
/* Learned offset sign (afc_ctrl.h sign check), persisted once observed. */
static volatile int8_t s_afc_sign = 1;
static volatile uint8_t s_afc_sign_state;
static bool s_afc_sign_saved;

static void standard_reset(void)
{
    s_pal_votes = s_ntsc_votes = 0;
    s_detected_std_valid = false;
}

/* Burst-confirmed standard from the AFC estimator; a few agreeing
 * windows make it stable and remembered for the next boot's menu. */
static void standard_vote(unsigned afc_standard)
{
    bool pal = afc_standard == 2u;
    if (pal) { if (s_pal_votes < 8u) ++s_pal_votes; if (s_ntsc_votes) --s_ntsc_votes; }
    else { if (s_ntsc_votes < 8u) ++s_ntsc_votes; if (s_pal_votes) --s_pal_votes; }
    if (s_pal_votes >= 3u && s_pal_votes >= s_ntsc_votes + 2u) { s_detected_std = VIDEO_STD_PAL; s_detected_std_valid = true; }
    else if (s_ntsc_votes >= 3u && s_ntsc_votes >= s_pal_votes + 2u) { s_detected_std = VIDEO_STD_NTSC; s_detected_std_valid = true; }
    if (s_detected_std_valid && s_last_std != (uint8_t)s_detected_std) {
        s_last_std = (uint8_t)s_detected_std;
        (void)blob_store("last_std", &s_last_std, 1);
    }
}

video_standard_t menu_standard(void)
{
    if (s_std_mode == STD_MODE_PAL) return VIDEO_STD_PAL;
    if (s_std_mode == STD_MODE_NTSC) return VIDEO_STD_NTSC;
    if (s_detected_std_valid) return s_detected_std;
    return s_last_std == VIDEO_STD_PAL ? VIDEO_STD_PAL : VIDEO_STD_NTSC;
}

/* 0..100 from phase coherence, envelope and gain headroom. */
static int strength(const metrics_t *m, uint8_t gain)
{
    if (m->q_phase < 18 || m->origin_pm > 850) return 0;
    int coherence = (m->q_phase - 18) * 100 / 62;
    int power = ((m->p_median >> 4) - 6) * 100 / 28;   /* ultrafine lane: 4^2 */
    int max_gain = rf_gain_table()->max_index;
    int headroom = (max_gain - (int)gain) * 100 / (max_gain > 2 ? max_gain - 2 : 1);
    coherence = coherence < 0 ? 0 : coherence > 100 ? 100 : coherence;
    power = power < 0 ? 0 : power > 100 ? 100 : power;
    headroom = headroom < 0 ? 0 : headroom > 100 ? 100 : headroom;
    return (coherence * 2 + power + headroom) / 4;
}

/* Decoder P64 <-> U85Q (console m, menu DEMOD), kept in NVS "demod": live, a
 * verified restart behind the shared 5 s guard; in the menu, applied by the
 * restart on menu exit. */
static void demod_next(void)
{
    const uint8_t next = (uint8_t)((transport_demod() + 1u) % TRANSPORT_DEMODS);
    const bool ok = transport_set_demod(next);
    if (ok) (void)blob_store("demod", &next, sizeof(next));
    printf("DEMOD switch to=%s result=%s active=%s%s\n", transport_demod_name(next), ok ? "ok" : "refused",
           transport_program_name(), ok ? "" : " (guard, fault or rollback: T TRANSPORT and DEMOD)");
}

#ifdef C5VRX_LCD
/* Waveshare LCD board DAC 5 <-> 6 bit (menu DAC, console d), NVS "dac_bits".
 * 6 bit takes GPIO13 (USB D-) for the LSB: USB, and this console, go off. */
static void dac_bits_next(void)
{
    const uint8_t next = transport_dac_bits() == 6u ? 5u : 6u;
    if (next == 6u) printf("DAC 6 bit: USB off now; back to 5 bit in the menu, or flash with BOOT held\n");
    const bool ok = transport_set_dac_bits(next);
    if (ok) (void)blob_store("dac_bits", &next, sizeof(next));
    printf("DAC bits=%u result=%s\n", transport_dac_bits(), ok ? "ok" : "refused");
}
#endif

static void after_retune(void)
{
    ++s_control_epoch;
    {
        uint8_t e[3] = {(uint8_t)rf_channel_index()};
        flog_put16(e + 1, rf_frequency_mhz());
        flightlog_event(FLOG_E_CHANNEL, e, sizeof(e));
    }
    s_cfo_khz = 0;
    s_afc_locked = false;
    standard_reset();
    settings_save();
    profile_channel_changed();   /* another channel may carry another VTX */
    if (s_menu_active) menu_render();
    printf("[CHANNEL] %s %u MHz%s\n", rf_channel()->name, rf_channel()->freq_mhz,
           rf_freq_in_spec(rf_channel()->freq_mhz) ? "" : " (above the documented C5 range, not characterised)");
}

/* ---- Channel scanner -------------------------------------------------------
 * RF strength alone let 5 GHz Wi-Fi win (issue #128): a channel only
 * qualifies with analog-video confidence (line-period periodicity), and its
 * carrier must sit within 3 MHz of the channel centre (a VTX also shows on
 * neighbouring channels through the filter, and the 50 ns offset estimate
 * wraps every 20 MHz). Among centred candidates the strongest wins. */
_Static_assert(SCAN_BUF_OFFSET >= CONTROL_BYTES, "scan endpoints must not overlap the control window");
static uint8_t *const s_scan_buf = s_control_scratch + SCAN_BUF_OFFSET;

static analog_video_t scan_confidence(void)
{
    analog_video_t v[3] = {{0}};
    for (unsigned w = 0; w < 3u; ++w) {
        if (w) vTaskDelay(pdMS_TO_TICKS(10));
        if (copy_recent_endpoints(s_scan_buf, SCAN_PAIRS))
            v[w] = analog_video_detect(s_scan_buf, SCAN_PAIRS, c5vrx_phase8_gain_lut);
    }
    analog_video_t a = v[0], b = v[1], c = v[2], t;   /* median by confidence */
    if (a.confidence > b.confidence) { t = a; a = b; b = t; }
    if (b.confidence > c.confidence) { t = b; b = c; c = t; }
    if (a.confidence > b.confidence) { t = a; a = b; b = t; }
    return b;
}

static analog_video_t channel_confidence(uint8_t scan_gain, uint8_t max_gain, int *quality)
{
    vTaskDelay(pdMS_TO_TICKS(90));
    metrics_t m = {0};
    if (copy_control_window(s_window, CONTROL_BYTES)) m = measure_window(s_window, CONTROL_BYTES);
    if (quality) *quality = strength(&m, scan_gain);
    analog_video_t video = scan_confidence();
    /* A far VTX is starved at the scan gain: look again at maximum. */
    if (video.confidence < ANALOG_VIDEO_MIN_CONFIDENCE && m.origin_pm > 500) {
        receiver_set_gain(max_gain);
        vTaskDelay(pdMS_TO_TICKS(20));
        analog_video_t far = scan_confidence();
        receiver_set_gain(scan_gain);
        if (far.confidence > video.confidence) video = far;
    }
    return video;
}

static void scan(void)
{
    if (!receiver_pause("SCAN")) return;
    const size_t original = rf_channel_index();
    const uint8_t max_gain = rf_gain_table()->max_index, scan_gain = 52u;
    size_t best = original;
    int best_quality = -1, best_video = -1, best_offset = 0x7fffffff;
    s_scan_active = true;
    s_scan_progress = 0;
    receiver_set_gain(scan_gain);   /* every channel compared at one gain */
    for (size_t ch = 0; ch < FPV_CHANNELS; ++ch) {
        s_scan_progress = (unsigned)(ch * 100u / FPV_CHANNELS);
        if (s_menu_active) menu_render();
        if (rf_set_channel(ch) != ESP_OK) continue;
        int quality = 0;
        analog_video_t v = channel_confidence(scan_gain, max_gain, &quality);
        if (v.confidence < ANALOG_VIDEO_MIN_CONFIDENCE) continue;
        int off = abs(v.offset_khz);
        printf("SCAN candidate %s %u MHz video=%d offset_khz=%d quality=%d %s\n", rf_channel()->name,
               rf_channel()->freq_mhz, v.confidence, v.offset_khz, quality, v.standard == 1 ? "PAL" : "NTSC");
        bool centred = off <= 3000, best_centred = best_offset <= 3000;
        if ((centred && !best_centred) ||
            (centred == best_centred &&
             (quality > best_quality ||
              (quality == best_quality &&
               (off / 500 < best_offset / 500 ||
                (off / 500 == best_offset / 500 && v.confidence > best_video)))))) {
            best = ch;
            best_quality = quality;
            best_video = v.confidence;
            best_offset = off;
        }
    }
    /* Confirm the winner: still video, still centred. */
    if (best_video >= 0) {
        (void)rf_set_channel(best);
        analog_video_t v = channel_confidence(scan_gain, max_gain, NULL);
        if (v.confidence < ANALOG_VIDEO_MIN_CONFIDENCE || abs(v.offset_khz) > 3000) best_video = -1;
    }
    if (best_video < 0) best = original;
    (void)rf_set_channel(best);
    s_scan_active = false;
    receiver_set_gain(max_gain);
    receiver_resume();
    printf("SCAN result=%s channel=%s\n", best_video < 0 ? "none_kept" : "found", rf_channel()->name);
    after_retune();
}

/* ---- Menu actions ---------------------------------------------------------- */
static void cycle_video_standard(void)
{
    menu_set_standard_mode(s_std_mode == STD_MODE_AUTO ? STD_MODE_NTSC :
                           s_std_mode == STD_MODE_NTSC ? STD_MODE_PAL : STD_MODE_AUTO);
    settings_save();
}

/* AFC OFF returns the synthesizer to the exact channel centre (0 kHz offset). */
/* Menu choices kept across reboots (settings prefs). */
static void prefs_save(void)
{
    const uint8_t gain = receiver_gain_mode();
    const settings_prefs_t p = {SETTINGS_PREFS_VERSION, (int8_t)profile_selected(),
                                receiver_gain_mode_auto(gain) ? gain : (uint8_t)GAIN_MODE_DEFAULT,
                                (uint8_t)phy_dfilt_mode()};
    settings_prefs_store(&p);
}

/* Menu PHY FILT: the digital RX filter mode 12 -> 0 -> 4 -> 8 (saved;
 * phy.h). A different transfer function changes the receiver noise
 * (PR #244: +0.24..+0.73 dB between modes) and the IQ the DC servo trims,
 * so the gain observation restarts on the new mode. */
static void cycle_dfilt(void)
{
    const int want = phy_dfilt_next(phy_dfilt_mode());
    const bool ok = phy_dfilt_select(want);
    receiver_dc_reset_all();
    if (ok) prefs_save();
    printf("[DFILT] mode=%d requested=%d applied=%u hw=%d (saved; boot default %d)\n", phy_dfilt_mode(), want,
           ok, phy_dfilt_hw_mode(), PHY_DFILT_MODE_DEFAULT);
}

static void toggle_afc(void)
{
    s_afc_enabled = !s_afc_enabled;
    s_afc_locked = false;
    s_cfo_khz = 0;
    if (!s_afc_enabled && rf_offset_khz() != 0) {
        rf_set_offset_khz(0);
        s_last_gain_write_us = esp_timer_get_time();
        ++s_gain_epoch;
    }
    ++s_control_epoch;
    const uint8_t off = !s_afc_enabled;
    (void)blob_store("afc_off", &off, 1);
    printf("[AFC] mode=%s offset_khz=%d\n", s_afc_enabled ? "ON" : "OFF", rf_offset_khz());
}

/* Press: the next row, in the CALIBRATE submenu while it is open. */
static void menu_next(void)
{
    s_exp_failed = false;
    if (s_exp_sub >= 0) s_exp_sub = (s_exp_sub + 1) % EXP_MENU_ITEMS;
    else if (s_cal_sub >= 0) s_cal_sub = (s_cal_sub + 1) % MENU_CAL_ITEMS;
    else s_menu_cursor = (s_menu_cursor + 1) % MENU_ITEMS;
    menu_render();
}

static void menu_action(void)
{
    if (s_exp_sub >= 0) {
        const unsigned k = (unsigned)s_exp_sub;
        uint8_t before[EXP_ITEMS], after[EXP_ITEMS];
        experimental_snapshot(before);
        s_exp_failed = false;
        if (k == EXP_BACK) s_exp_sub = -1;
        else if (k == EXP_RESET) {
            experimental_reset();
            receiver_set_gain_mode(GAIN_MODE_DEFAULT, true);
            if (!phy_dfilt_select(PHY_DFILT_MODE_DEFAULT)) s_exp_failed = true;
            if (!s_afc_enabled) toggle_afc();
            if (!transport_set_demod(TRANSPORT_DEMOD_P64)) s_exp_failed = true;
            else { const uint8_t d = TRANSPORT_DEMOD_P64; (void)blob_store("demod", &d, 1); }
            prefs_save();
        }
        else if (k == EXP_DAC) menu_dac_step();
        else if (k == EXP_GAIN) { receiver_set_gain_mode(receiver_gain_mode_next(receiver_gain_mode()), true); prefs_save(); }
        else if (k == EXP_DFILT) cycle_dfilt();
        else if (k == EXP_AFC) toggle_afc();
        else if (k == EXP_DEMOD) demod_next();
        else if (k < EXP_ITEMS) s_exp_failed = !experimental_set(k, (experimental_get(k) + 1u) % experimental_items[k].count);
        experimental_snapshot(after);
        for (unsigned i = 0; i < EXP_IQ_CORR; ++i) if (before[i] != after[i]) {
            const uint8_t e[] = {(uint8_t)i, before[i], after[i], 1};
            flightlog_event(FLOG_E_EXP, e, sizeof(e));
        }
        experimental_log_state();
        menu_render();
        return;
    }
    if (s_cal_sub >= 0) {   /* CALIBRATE submenu: QUICK = console F, FULL = console B */
        const int k = s_cal_sub;
        s_cal_sub = -1;
        if (k == MENU_CAL_QUICK) { menu_close(); filter_request(); return; }        /* runs once the menu is closed */
        if (k == MENU_CAL_FULL) { menu_close(); filter_request_full(); return; }
        menu_render();
        return;
    }
    switch (s_menu_cursor) {
    case MENU_BAND: rf_next_band(); after_retune(); break;
    case MENU_CHANNEL: rf_next_channel(); after_retune(); break;
    case MENU_SCAN: scan(); break;
    case MENU_FILTER: (void)profile_select_cycle(); prefs_save(); break;
    case MENU_AFC: toggle_afc(); break;
    case MENU_DFILT: cycle_dfilt(); break;
    case MENU_EXPERIMENTAL: s_exp_sub = 0; break;
    case MENU_CALIBRATE: s_cal_sub = MENU_CAL_QUICK; break;   /* opens the QUICK / FULL / BACK submenu */
    case MENU_OUTPUT: (void)transport_set_output((transport_output() + 1u) % OUTPUT_LEVELS); break;
#ifdef C5VRX_LCD
    case MENU_DAC_BITS: dac_bits_next(); break;
#endif
    case MENU_GAIN: receiver_set_gain_mode(receiver_gain_mode_next(receiver_gain_mode()), true); prefs_save(); break;
    case MENU_DEMOD: demod_next(); break;   /* applied by the verified restart on menu exit */
    case MENU_DAC: menu_dac_step(); break;
    case MENU_VIDEO:
        cycle_video_standard();
        break;
    default: menu_close(); return;
    }
    if (s_menu_active) menu_render();
}

static void command(int c, unsigned *menu_idle)
{
    *menu_idle = 0;
    if (menu_status_active()) {   /* a calibration owns the picture */
        printf("COMMAND %c refused=busy (a calibration or lane test owns the picture; retry when it ends)\n", c);
        return;
    }
    switch (c) {
    case 'G': receiver_set_gain_mode(receiver_gain_mode_next(receiver_gain_mode()), true); prefs_save(); break;
    /* Decoder P64 <-> U85Q: a verified restart behind the shared 5 s guard. */
    case 'm': demod_next(); break;
#ifdef C5VRX_LCD
    case 'd': dac_bits_next(); if (s_menu_active) menu_render(); break;
#endif
    /* Bench: the real gain step of every table row pair (receiver.c). */
    case 'Q': receiver_pair_sweep_toggle(); break;
    /* DAC self-test: open the menu on DAC TEST, then step like a hold. */
    case 'Z':
        if (!s_menu_active && !menu_open_checked()) break;
        if (s_exp_sub != EXP_DAC || s_cal_sub >= 0) { s_cal_sub = -1; s_menu_cursor = MENU_EXPERIMENTAL; s_exp_sub = EXP_DAC; menu_render(); }
        else menu_dac_step();
        break;
    case 'o': if (s_menu_active) menu_close(); else (void)menu_open_checked(); break;
    case 'e': menu_close(); break;  /* idempotent web-menu EXIT */
    case 'v': cycle_video_standard(); break;
    case 'f': (void)profile_select_cycle(); prefs_save(); if (s_menu_active) menu_render(); break;   /* menu FILTER hold */
    case 'n': if (s_menu_active) menu_next(); break;
    case 'x': if (s_menu_active) menu_action(); break;
    case 'c': rf_next_channel(); after_retune(); break;
    case 'b': rf_next_band(); after_retune(); break;
    case 's': scan(); break;
    case 'A': {
        profile_set_live_mode(!profile_live_mode());
        printf("PROFILE live switch %s (saved): %s\n", profile_live_mode() ? "ON" : "OFF",
               profile_live_mode() ? "widen after one reliable result, narrow after two agreeing results"
                                   : "report only, PENDING for the next boot or CALIBRATE");
        break;
    }
    case 'g': {
        unsigned next = (transport_output() + 1u) % OUTPUT_LEVELS;
        bool ok = transport_set_output(next);
        printf("OUTPUT manual %s result=%s wait_ms=%lu%s\n", transport_output_name(),
               ok ? "applied" : "refused", (unsigned long)transport_switch_wait_ms(),
               transport_output() == 9 ? " (low swing: halves noise & clipping; requires sensitive sync)" :
               (transport_output() ? " WARNING: may clip wide VTX; verify sync/white into 75 ohm" : ""));
        if (s_menu_active) menu_render();
        break;
    }
    default: break;
    }
}

static uint32_t afc_context(void)
{
    const uint32_t parts[5] = {(uint32_t)rf_channel_index(), s_control_epoch, (uint32_t)rf_offset_khz(),
                               rf_table_generation(), phy_generation()};
    uint32_t h = 2166136261u;
    for (unsigned k = 0; k < 5u; ++k) h = (h ^ parts[k]) * 16777619u;
    return h;
}

/* End-to-end video quality, one-second periods (video_quality.h). */
static vq_acc_t s_vq_acc;
static vq_t s_vq;
static uint32_t s_vq_periods;

/* copy_snapshot() under the same scheduler hold as copy_control_window():
 * a preempted copy fails its overrun check. */
/* Picture measure totals and its CPU cost per window (T PICTURE). */
static struct { picture_state_t state; uint32_t found, expected, windows, us_max; uint64_t us_sum; } s_pic;

static bool copy_afc_window(uint8_t *dst)
{
    vTaskSuspendAll();
    const bool ok = copy_snapshot(dst);
    (void)xTaskResumeAll();
    return ok;
}

static void control_task(void *arg)
{
    (void)arg;
    afc2_ctrl_t afc = {.target_khz = AFC_PORCH_TARGET_KHZ, .sign = s_afc_sign, .sign_state = s_afc_sign_state};
    uint8_t seen_sign_state = afc.sign_state;
    bool own_write = false;
    uint8_t afc_lost = 0;
    unsigned settle = RETUNE_SETTLE_TICKS, btn = 0, boot_grace = 20, menu_idle = 0, menu_refresh = 0;
    bool long_fired = false, was_locked = false;
    uint32_t seen_phy = phy_generation(), seen_control = s_control_epoch;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        int c;
        experimental_poll();
        for (unsigned k = 0; k < 8 && xQueueReceive(s_commands, &c, 0) == pdTRUE; ++k) command(c, &menu_idle);

        /* BOOT button: short = next channel (menu: next item); 0.6 s = menu
         * (menu: do the selected item). */
        /* A calibration status screen or the boot logo owns the picture: no
         * menu, no idle exit. */
        if (menu_status_active() || menu_splash_active()) {
            btn = 0; long_fired = false; menu_idle = 0; continue;
        }
        if (boot_grace) { --boot_grace; btn = 0; long_fired = false; }
        else if (gpio_get_level(BOOT_BUTTON) == 0) {
            ++btn;
            menu_idle = 0;
            if (btn >= LONG_TICKS && !long_fired) {
                long_fired = true;
                if (!s_menu_active) (void)menu_open_checked();
                else menu_action();
            }
        } else if (btn) {
            if (!long_fired && btn >= 2) {
                if (s_menu_active) menu_next();
                else { rf_next_channel(); after_retune(); }
            }
            btn = 0;
            long_fired = false;
        }
        if (s_menu_active) {
            /* The dBm on the CHANNEL page: the gain observer stands still in
             * the menu, so its control windows feed the level, rolled once a
             * second (20 ticks). */
            static uint8_t level_ticks;
            if (copy_control_window(s_window, CONTROL_BYTES)) receiver_level_feed(s_window, CONTROL_BYTES);
            if (++level_ticks >= 20u) { level_ticks = 0; receiver_level_roll(false, false); }
            /* A multimeter needs longer than 30 s; the page is only reached
             * by pressing through the menu. */
            const unsigned idle_limit = (s_exp_sub == EXP_DAC || s_menu_cursor == MENU_DAC) ? DAC_IDLE_TICKS : MENU_IDLE_TICKS;
            if (++menu_idle >= idle_limit) {
                menu_close();
                printf("[MENU] closed after %u s idle\n", idle_limit * TICK_MS / 1000u);
            }
            else if (++menu_refresh >= 5) { menu_refresh = 0; menu_render(); }
        } else menu_idle = menu_refresh = 0;

        transport_poll_faults();
        if (phy_generation() != seen_phy || s_control_epoch != seen_control) {
            seen_phy = phy_generation();
            seen_control = s_control_epoch;
            s_cfo_khz = 0;
            afc2_ctrl_invalidate(&afc);
            settle = own_write ? 2u : RETUNE_SETTLE_TICKS;
        }
        uint32_t gain_epoch = s_gain_epoch, ctx = afc_context();
        if (s_scan_active) {
            afc2_ctrl_invalidate(&afc);
            memset(&s_afc_evidence, 0, sizeof(s_afc_evidence));
            s_afc_evidence.window="scan"; s_afc_evidence.held="scan";
            s_afc_evidence.cn_x10=CN_UNKNOWN_X10;
            diag_window(false, false, false);
            continue;
        }
#ifdef C5VRX_LCD
        /* LCD build: the AFC window costs ~17 ms of CPU per tick (afc2_measure,
         * 80 % of the tick; board 2026-10-30) that the frame grabber needs.
         * Locked, the AFC only watches for slow VTX drift (the lock is
         * sticky), so it measures every AFC_LOCKED_PERIOD_US; unlocked or
         * settling, every tick. 300 ms keeps the menu's VIDEO (a confirmed
         * sync within 500 ms) steady. */
        {
            static int64_t measured_us;
            const int64_t t = esp_timer_get_time();
            if (s_afc_enabled && s_afc_locked && settle == 0 && t - measured_us < AFC_LOCKED_PERIOD_US) continue;
            measured_us = t;
        }
#endif
        const bool copied = copy_control_window(s_window, CONTROL_BYTES);
        metrics_t m = {0};
        if (copied) {
            m = measure_window(s_window, CONTROL_BYTES);
            s_signal_strength = (s_signal_strength * 3 + strength(&m, s_current_gain) + 2) / 4;
        }

        /* AFC: burst/line-timing-confirmed sync/porch levels, never the
         * scene-biased FM slope; acquisition only, sticky once locked. */
        if (afc2_ctrl_sync(&afc, ctx, own_write)) { s_afc_locked = false; afc_lost = 0; }
        own_write = false;
        int64_t now = esp_timer_get_time();
        /* Preserve the receiver-settling gate independently of the retired
         * kp ladder: AFC must never use samples spanning a gain/context write. */
        const bool receiver_ok = copied && settle == 0 && ctx == afc_context() && !phy_busy() &&
                                 now - s_last_gain_write_us >= 5000;
        /* The AFC window holds 3.2 lines, not the 1.6 of one descriptor:
         * without a detected burst a sync counts only with a second one a
         * line earlier, which one descriptor contains in ~43 % of windows
         * (board IQ replay 2026-10-26: 45 % vs 30/30 snapshots). It ends
         * where the active descriptor began, so it covers s_window. */
        const bool afc_copied = receiver_ok && copy_afc_window(s_afc_window);
        bool window_ok = afc_copied && afc2_envelope_stationary(s_afc_window, AFC_WINDOW_BYTES);
        /* Picture measure (picture_sync.h): every live window, also those the
         * AFC leaves out (a gain write, settle, fading): the goggles see those.
         * Not in the LCD build: it rates the DAC decoder's output, which no
         * display shows there, at ~1.7 ms per window. */
#ifdef C5VRX_LCD
        if (false) {
#else
        if (copied && !s_menu_active && (afc_copied || copy_afc_window(s_afc_window))) {
#endif
            unsigned bits;
            const uint16_t *lut = transport_decoder_lut(&bits);
            if (lut) {
                const int64_t t0 = esp_timer_get_time();
                const picture_sync_t ps = picture_sync_window(&s_pic.state, s_afc_window, AFC_WINDOW_BYTES, lut, bits);
                const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
                if (ps.expected) diag_picture(ps.found, ps.expected);
                s_pic.found += ps.found; s_pic.expected += ps.expected; ++s_pic.windows;
                s_pic.us_sum += us;
                if (us > s_pic.us_max) s_pic.us_max = us;
            }
        }
        const char *afc_window_reason = !copied ? "copy_missing" : settle ? "settle" :
            ctx != afc_context() ? "context" : phy_busy() ? "phy_busy" : !receiver_ok ? "gain_settle" :
            !afc_copied ? "afc_copy_missing" : !window_ok ? "nonstationary" : "measured";
        if (settle) --settle;
        afc2_result_t r = {0};
        if (window_ok) {
            r = afc2_measure(s_afc_window, AFC_WINDOW_BYTES, c5vrx_phase8_gain_lut);
            afc2_ctrl_observe_at(&afc, &r, (uint64_t)now);
            vq_add(&s_vq_acc, &r);
        } else afc2_ctrl_miss(&afc);   /* settle and context changes reset via afc2_ctrl_sync */
        if (vq_close(&s_vq_acc, now, &s_vq)) ++s_vq_periods;
        if (copied) s_cfo_khz = r.porch_pairs ? r.porch_khz : 0;
        /* Burst or line timing confirmed the syncs (afc2_measure). */
        bool valid = window_ok && r.lines && r.standard && r.porch_pairs && r.sync_pairs;
        const bool qualified=valid && r.polarity && gain_epoch==s_gain_epoch && ctx==afc_context();
        if (s_afc_evidence.context != ctx) s_afc_evidence.qualified_us=0;
        if (qualified) {
            s_afc_evidence.qualified_us=(uint64_t)now;
            s_afc_evidence.qualified_gain_epoch=gain_epoch;
            s_afc_evidence.raw_porch_khz=r.porch_khz;
            s_afc_evidence.residual_khz=r.porch_khz-afc.target_khz;
        }
        if (valid) { s_last_video_us = now; standard_vote(r.standard); profile_observe(&r); }
        if (!s_afc_enabled && rf_offset_khz() != 0) {
            rf_set_offset_khz(0);
            s_last_gain_write_us = esp_timer_get_time();
            ++s_gain_epoch;
        }
        bool was = s_afc_locked;
        if (!s_afc_enabled) { s_afc_locked = false; afc_lost = 0; }
        else if (copied) s_afc_locked = afc2_native_lock(s_afc_locked, valid, afc2_ctrl_can_lock(&afc, true), afc.n, &afc_lost);
        if (was && !s_afc_locked) afc2_ctrl_reacquire(&afc);
        /* Re-acquisition retunes only on a strong carrier: at the range edge
         * the porch estimate is pulled toward the receiver centre and would
         * walk the offset the wrong way (cn_estimate.h, afc_bias.py). A weak
         * window still feeds the estimate history, it just cannot step. */
        const int cn_x10 = receiver_obs_cn_x10();
        const char *afc_held = !s_afc_enabled ? "afc_off" : s_afc_locked ? "sticky_lock" : !valid ? "window" : s_menu_active ? "menu" : "actuator";
        if (s_afc_enabled && !s_afc_locked && valid && !s_menu_active &&
            phy_try_actuator(phy_generation())) {
            int32_t step = 0;
            const bool strong = receiver_obs_cn_x10() >= AFC_MIN_CN_X10;   /* noise-referenced power C/N */
            if (afc2_ctrl_decide(&afc, gain_epoch == s_gain_epoch && strong, &step)) {
                rf_set_offset_khz(rf_offset_khz() + (int)step);
                s_last_gain_write_us = esp_timer_get_time();   /* restore re-forces gain */
                ++s_gain_epoch;
                ++s_afc_corrections;
                own_write = true;
                afc_held="stepped";
                printf("[AFC] step_khz=%ld offset_khz=%d cn_est_x10=%d coherence=%d gain=%u\n", (long)step,
                       rf_offset_khz(), cn_x10, s_obs_coherence, s_current_gain);
            } else if (!strong && afc2_ctrl_stable(&afc, &step) &&
                       (step > AFC2_CTRL_DEADBAND_KHZ || step < -AFC2_CTRL_DEADBAND_KHZ)) {
                ++s_afc_weak_skips;    /* would have stepped on a weak carrier */
            }
            if (!own_write) {
                int32_t median=0;
                afc_held=gain_epoch != s_gain_epoch ? "gain_changed" : !strong ? "weak_or_unknown_cn" :
                    afc.n<AFC2_CTRL_SAMPLES ? "collecting" : !afc2_ctrl_stable(&afc,&median) ? "unstable" :
                    (median>=-AFC2_CTRL_DEADBAND_KHZ && median<=AFC2_CTRL_DEADBAND_KHZ) ? "deadband" : "correction_budget";
            }
            phy_end_actuator();
        }
        s_afc_evidence.history=afc2_ctrl_evidence(&afc,(uint64_t)now);
        s_afc_evidence.sampled_us=(uint64_t)now;
        s_afc_evidence.context=ctx;
        s_afc_evidence.cn_x10=cn_x10;
        s_afc_evidence.latest_valid=qualified;
        s_afc_evidence.sticky_lock=s_afc_locked;
        s_afc_evidence.window=qualified ? "valid" : valid ? "stale_gain_or_context" : window_ok ? "no_confirmed_sync" : afc_window_reason;
        s_afc_evidence.held=afc_held;
        if (afc.sign_state != seen_sign_state) {
            seen_sign_state = afc.sign_state;
            s_afc_sign_state = afc.sign_state;
            printf("[AFC] offset sign %s: %+d\n", afc.sign_state == AFC2_SIGN_FLIPPED ? "FLIPPED" : "confirmed",
                   afc.sign < 0 ? -1 : 1);
            int8_t sign = afc.sign < 0 ? -1 : 1;
            if (!s_afc_sign_saved || sign != s_afc_sign) {
                s_afc_sign = sign;
                s_afc_sign_saved = blob_store("afc_sign", &sign, 1);
            }
        }
        diag_window(window_ok, valid, own_write);   /* own_write: an AFC step this window */
        if (s_afc_locked && !was_locked)
            printf("[VIDEO] locked %s %u MHz %s offset_khz=%d gain=%u\n", rf_channel()->name,
                   rf_channel()->freq_mhz, r.standard == 2 ? "PAL" : "NTSC", rf_offset_khz(), s_current_gain);
        was_locked = s_afc_locked;
    }
}

void control_start(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    if (!blob_load("last_std", &s_last_std, 1)) s_last_std = 0xFF;
    uint8_t afc_off = 0;
    if (blob_load("afc_off", &afc_off, 1)) s_afc_enabled = !afc_off;
    profile_set_live_mode(experimental_get(EXP_AUTO_PROFILE) != 0);
    experimental_log_state();
    int8_t sign;
    if (blob_load("afc_sign", &sign, 1) && (sign == 1 || sign == -1)) {
        s_afc_sign = sign;
        s_afc_sign_state = AFC2_SIGN_CONFIRMED;
        s_afc_sign_saved = true;
    }
    s_commands = xQueueCreate(16, sizeof(int));
    ESP_ERROR_CHECK(s_commands ? ESP_OK : ESP_ERR_NO_MEM);
    /* IDF task stack depths are bytes. Reserve before driver allocations,
     * then supply that same 6144-byte stack and TCB without another malloc.
     * LCD build: below the frame grabber (lcd.c), which rests 15 ms after
     * every picture. At priority 3 each 50 ms tick's ~10 ms of work stalled
     * the grabber far beyond the 0.8 ms ring, on the same lines every time
     * (50 ms = 2.5 PAL fields; board 2026-10-30: 158 of 177 grabs stalled
     * >= 5 ms, up to 90 ms). It now runs between pictures. */
#ifdef C5VRX_LCD
    const UBaseType_t prio = 1;
#else
    const UBaseType_t prio = 3;
#endif
    ESP_ERROR_CHECK(control_prepare() &&
        xTaskCreateStatic(control_task,"control",CONTROL_STACK_BYTES,NULL,prio,
            s_control_memory->stack,&s_control_memory->task)!=NULL ? ESP_OK:ESP_ERR_NO_MEM);
}

/* Status beat for the web menu (web/receiver-console.js): one short line
 * every 2 s from the console task, so the menu follows the receiver
 * without a status request. Read-only. */
void control_print_beat(void)
{
    if (experimental_mask()) printf("EXP active=0x%08lx reboot=%u\n", (unsigned long)experimental_mask(), experimental_reboot_needed());
    /* video: confirmed sync within 0.5 s (the web menu's VIDEO/SEARCH);
     * afc_locked alone said SEARCH over a good picture without a lock. */
    const int64_t video_at = s_last_video_us;
    printf("STATUS channel=%s freq=%u afc_locked=%u video=%u standard=%s detected=%s signal=%d menu=%u mode=%s demod=%s "
           "output=%s live=%u busy=%u fw=%s\n",
           rf_channel()->name, rf_frequency_mhz(), s_afc_locked,
           video_at && esp_timer_get_time() - video_at < 500000,
           s_std_mode == STD_MODE_AUTO ? "AUTO" : s_std_mode == STD_MODE_PAL ? "PAL" : "NTSC",
           s_detected_std_valid ? (s_detected_std == VIDEO_STD_PAL ? "PAL" : "NTSC") : "none",
           s_signal_strength, s_menu_active, profile_name(profile_active()), transport_program_name(), transport_output_name(),
           profile_live_mode(), menu_status_active(), firmware_id());
}

void control_flog_afc(int16_t *offset_khz, int16_t *residual_khz, uint32_t *corrections)
{
    *offset_khz = (int16_t)rf_offset_khz();
    *residual_khz = (int16_t)s_afc_evidence.residual_khz;
    *corrections = s_afc_corrections;
}

/* End-to-end video quality of the last second (video_quality.h). */
void control_video_quality(vq_t *out) { vTaskSuspendAll(); *out = s_vq; (void)xTaskResumeAll(); }

void control_print(void)
{
    vTaskSuspendAll();
    const afc_evidence_snapshot_t afc_snapshot=s_afc_evidence;
    const uint64_t afc_now=(uint64_t)esp_timer_get_time();
    const bool afc_context_current=afc_snapshot.context==afc_context();
    const bool afc_gain_current=afc_snapshot.qualified_us && afc_snapshot.qualified_gain_epoch==s_gain_epoch;
    (void)xTaskResumeAll();
    const afc2_evidence_t evidence=afc_snapshot.history;
    printf("AFC_EVIDENCE latest_valid=%u context_current=%u qualified_gain_current=%u sticky_lock=%u target_khz=%d "
           "last_qualified_raw_porch_khz=%ld last_qualified_residual_khz=%ld qualified_age_ms=%lld sample_age_ms=%lld "
           "n=%u timestamped=%u stable=%u median_residual_khz=%ld mad_khz=%ld oldest_age_ms=%lld newest_age_ms=%lld "
           "cn_x10=%d window=%s held=%s reference=porch_not_rf_truth\n",
           afc_snapshot.latest_valid, afc_context_current, afc_gain_current, afc_snapshot.sticky_lock, AFC_PORCH_TARGET_KHZ,
           (long)afc_snapshot.raw_porch_khz, (long)afc_snapshot.residual_khz,
           (long long)afc2_evidence_age_ms(afc_context_current && afc_gain_current ? afc_snapshot.qualified_us : 0,afc_now),
           (long long)afc2_evidence_age_ms(afc_snapshot.sampled_us,afc_now),
           evidence.n, evidence.timestamped, evidence.stable, (long)evidence.median_khz, (long)evidence.mad_khz,
           (long long)afc2_evidence_age_ms(evidence.oldest_us,afc_now),
           (long long)afc2_evidence_age_ms(evidence.newest_us,afc_now),
           afc_snapshot.cn_x10, afc_snapshot.window, afc_snapshot.held);
    printf("CONTROL channel=%s freq=%u offset_khz=%d cfo_khz=%d afc_locked=%u afc_corrections=%u "
           "afc_weak_skips=%u afc_sign=%+d(%s) standard=%s detected=%s signal=%d menu=%u deviation_x100=%d mode=%s "
           "demod=%s busy=%u\n",
           rf_channel()->name, rf_frequency_mhz(), rf_offset_khz(), s_cfo_khz, s_afc_locked,
           s_afc_corrections, s_afc_weak_skips, s_afc_sign,
           s_afc_sign_state == AFC2_SIGN_FLIPPED ? "flipped" : s_afc_sign_state == AFC2_SIGN_CONFIRMED ? "confirmed" :
           "unverified", s_std_mode == STD_MODE_AUTO ? "AUTO" : s_std_mode == STD_MODE_PAL ? "PAL" : "NTSC",
           s_detected_std_valid ? (s_detected_std == VIDEO_STD_PAL ? "PAL" : "NTSC") : "none",
           s_signal_strength, s_menu_active, profile_deviation_x100(rf_channel_index()),
           profile_name(profile_active()), transport_program_name(), menu_status_active());
    vq_t vq;
    control_video_quality(&vq);
    printf("VIDEO_Q known=%u snr_x10=%d lines_x100=%u lines=%lu windows=%lu periods=%lu (end-to-end: sync/porch "
           "spread of the demodulated video, last second)\n", vq.known, vq.snr_x10, vq.lines_x100,
           (unsigned long)vq.lines, (unsigned long)vq.windows, (unsigned long)s_vq_periods);
    printf("PICTURE found=%lu expected=%lu windows=%lu us_mean=%lu us_max=%lu (picture_sync.h: sync pulses in the "
           "decoded control windows since boot, apart from the AFC's video_pct; CPU per window)\n",
           (unsigned long)s_pic.found, (unsigned long)s_pic.expected, (unsigned long)s_pic.windows,
           (unsigned long)(s_pic.windows ? s_pic.us_sum / s_pic.windows : 0), (unsigned long)s_pic.us_max);
}

