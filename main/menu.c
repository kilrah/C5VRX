/* C5VRX-5 by Twotoz and the C5VRX contributors: on-screen menu.
 * A standalone PAL/NTSC raster (menu_raster.c) owns TX while the menu is
 * open; live video resumes with the exact boot topology on exit. The GDMA
 * scatter chain is allocated in 384-byte chunks only while the menu is open
 * (NTSC ~18 KiB, PAL ~22 KiB of internal SRAM). */
#include "receiver.h"
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rf.h"
#include "dac_check.h"
#include "menu_ui.h"
#include "phy.h"
#include "profile_logic.h"

void transport_start_chain(dma_descriptor_t *first);
void transport_stop_chain(void);

/* 32 x 12 B = 384 B, word aligned. With live tasks running the internal
 * heap has >20 KiB free but in pieces: 64-byte-aligned 1.5-KiB and 768-B
 * chunks failed after ~14 KiB (board 2026-10-18, NO_MEM at 7-8 KiB free).
 * NTSC needs 1499 nodes, PAL 1811. Internal SRAM is not cached on the C5
 * and sync_dma_c2m rounds itself, so cache-line alignment buys nothing. */
#define NODE_CHUNK  32u
#define NODE_CHUNKS ((MENU_MAX_NODES + NODE_CHUNK - 1u) / NODE_CHUNK)

static DMA_ATTR __attribute__((aligned(64))) menu_raster_t s_raster;
static dma_descriptor_t *s_chunks[NODE_CHUNKS];
static unsigned s_chunk_count, s_node_capacity, s_node_count;
volatile int s_menu_cursor;
volatile int s_exp_sub = -1;
bool s_exp_failed;
volatile int s_cal_sub = -1;   /* MENU_CAL_* while the CALIBRATE submenu is open */
/* Status screen: a full-screen message on the menu raster (calibration). */
static volatile bool s_status;
static char s_status_title[40], s_status_line1[48], s_status_line2[48];
static volatile int s_status_percent;
/* DAC TEST: the current step, and whether the whole raster is a flat
 * meter level (sync and burst overwritten) instead of the menu picture. */
static dac_step_t s_dac_step = DAC_STEP_PATTERN;
static bool s_raster_flat;
static uint8_t s_flat_code;

static bool s_interlaced = true;

/* Back from a flat meter level to the menu picture: sync, burst and blank
 * segments are rebuilt in place; the descriptor chain is unchanged. */
static void raster_restore(void)
{
    if (!s_raster_flat) return;
    menu_raster_init(&s_raster, s_video_std, s_interlaced);
    s_raster_flat = false;
    sync_dma_c2m(&s_raster, sizeof(s_raster));
}

/* Every byte the chain sends becomes one DAC code: a constant node voltage
 * for a multimeter. The display loses sync meanwhile. */
static void raster_flat(uint8_t code)
{
    if (s_raster_flat && s_flat_code == code) return;
    memset(&s_raster, code, sizeof(s_raster));
    s_raster_flat = true;
    s_flat_code = code;
    sync_dma_c2m(&s_raster, sizeof(s_raster));
}

void menu_dac_step(void)
{
    s_dac_step = dac_check_next(s_dac_step);
    const uint8_t code = dac_check_code(s_dac_step);
    if (s_dac_step == DAC_STEP_PATTERN) {
        printf("DACTEST pattern: a smooth ramp, then six bars whose contrast doubles from D4 to D9\n");
    } else {
        if (s_dac_step == DAC_STEP_BIT0) {
            printf("DACTEST meter steps: unplug the display, measure CVBS to GND; expected mV at 3.30 V:");
            for (unsigned k = 0; k < DAC_CHECK_BITS; ++k)
                printf(" %s=%d", dac_check_pins[k], dac_check_expected_mv((uint8_t)(1u << k), DAC_CHECK_VDD_MV));
            printf(" ALL=%d NONE=0 (scale by your 3V3 pin / 3.30)\n", dac_check_expected_mv(63, DAC_CHECK_VDD_MV));
        }
        printf("DACTEST step=%s code=%u expect_mv=%d (record it for tools/dac_check.py)\n",
               dac_check_step_name(s_dac_step), code, dac_check_expected_mv(code, DAC_CHECK_VDD_MV));
    }
    if (s_menu_active) menu_render();
}

/* Boot screen: -1 off, else the loading bar in percent. */
static volatile int s_splash = -1;

static void render_splash(void)
{
    raster_restore();
    menu_ui_render_splash_health(s_raster.ui, s_splash, calibration_dc_needed(NULL, 0) != 0);
    sync_dma_c2m(s_raster.ui, sizeof(s_raster.ui));
}

bool menu_splash_active(void) { return s_splash >= 0; }

/* The logo on the menu raster while the receiver tasks start (main.c).
 * RX keeps running under the raster. If the raster cannot take TX, boot
 * goes on without it. */
bool menu_splash_begin(void)
{
    s_splash = 0;
    if (menu_open()) return true;
    s_splash = -1;
    printf("SPLASH skipped (menu raster could not take TX)\n");
    return false;
}

void menu_splash_progress(int percent)
{
    if (s_splash < 0) return;
    s_splash = percent < 0 ? 0 : percent > 100 ? 100 : percent;
    render_splash();
}

void menu_splash_end(void)
{
    if (s_splash < 0) return;
    s_splash = -1;
    menu_close();
}

static void render_status(void)
{
    raster_restore();
    const fpv_channel_t *ch = rf_channel();
    const menu_status_view_t v = {
        .title = s_status_title, .line1 = s_status_line1, .line2 = s_status_line2,
        .percent = s_status_percent, .channel = ch->name, .freq_mhz = ch->freq_mhz, .exp_active = experimental_mask() != 0,
    };
    menu_ui_render_status(s_raster.ui, &v);
    sync_dma_c2m(s_raster.ui, sizeof(s_raster.ui));
}

bool menu_status_active(void) { return s_status; }

/* Show (or update) a full-screen status message; percent < 0 hides the bar.
 * The menu raster owns TX meanwhile, like the menu itself. */
void status_screen(const char *title, const char *line1, const char *line2, int percent)
{
    snprintf(s_status_title, sizeof(s_status_title), "%s", title);
    snprintf(s_status_line1, sizeof(s_status_line1), "%s", line1);
    snprintf(s_status_line2, sizeof(s_status_line2), "%s", line2);
    s_status_percent = percent;
    if (!s_status) {
        s_status = true;
        if (!s_menu_active && !menu_open()) {
            s_status = false;
            printf("STATUS screen refused (menu raster could not take TX): %s\n", title);
            return;
        }
    }
    render_status();
}

void status_screen_end(void)
{
    if (!s_status) return;
    s_status = false;
    menu_close();
}

void menu_render(void)
{
    char gain_mode[16], dc_need[24];
    if (s_splash >= 0) { render_splash(); return; }
    if (s_status) { render_status(); return; }
    if (!(s_exp_sub == EXP_DAC || (s_exp_sub < 0 && s_menu_cursor == MENU_DAC))) s_dac_step = DAC_STEP_PATTERN;   /* leaving the item ends the meter */
    if (s_dac_step != DAC_STEP_PATTERN) { raster_flat(dac_check_code(s_dac_step)); return; }
    raster_restore();
    const fpv_channel_t *ch = rf_channel();
    const unsigned index = (unsigned)rf_channel_index();
    const rx_level_t lv = receiver_level();   /* the level at the chip pin (rx_level.h) */
    const uint8_t mode = receiver_gain_mode();
    /* Range warnings take the place of the link state: they cost range
     * without any sign in the picture. */
    const bool dc_cal = calibration_dc_needed(dc_need, sizeof(dc_need)) != 0;
    const int selected = profile_selected(), active = profile_active();
    static const char *const s_class[PROFILES] = {"NORMAL", "NARROW", "NARROW", "WIDE", "XNARROW"};
    const int dev = profile_live_deviation_x100();   /* what AUTO would pick from the measured VTX */
    menu_view_t v = {
        .exp_open = s_exp_sub >= 0, .exp_cursor = s_exp_sub < 0 ? 0 : (unsigned)s_exp_sub,
        .exp_active = experimental_mask() != 0, .exp_failed = s_exp_failed || experimental_phy_failed(),
        .exp_reboot = experimental_reboot_needed(), .exp_coarse = rf_lane() != RF_LANE_ULTRAFINE,
        .cursor = s_menu_cursor, .cal_sub = s_cal_sub,
        .channel = ch->name, .freq_mhz = ch->freq_mhz,
        .band = index / 8u, .channel_in_band = index % 8u, .band_name = rf_band_name(index / 8u),
        .band_letters = "RABEFL", .band_count = FPV_BANDS,
        /* VIDEO = confirmed sync just now; it used to read the AFC lock, so
         * a picture without lock showed SEARCH (operator, 2026-10-26). */
        .signal = s_signal_strength, .afc_locked = s_afc_locked, .afc_off = !s_afc_enabled,
        .video = s_last_video_us && esp_timer_get_time() - s_last_video_us < 500000,
        .level_valid = lv.valid, .level_above_noise = lv.above_noise, .level_dbm = (lv.carrier_dbm_x10 - 5) / 10,
        .dc_cal = dc_cal, .dc_need = dc_need, .gain_fixed = !receiver_gain_mode_auto(mode),
        .scan_active = s_scan_active, .scan_progress = s_scan_progress,
        .std_mode = (int)s_std_mode, .std_out_pal = s_video_std == VIDEO_STD_PAL,
        .detected_valid = s_detected_std_valid, .detected_pal = s_detected_std == VIDEO_STD_PAL,
        .output = transport_output(), .output_name = transport_output_name(),
        .gain_mode = receiver_gain_mode_name(mode, gain_mode, sizeof(gain_mode)), .gain_auto = receiver_gain_mode_auto(mode),
        .demod = transport_demod() == TRANSPORT_DEMOD_U85Q ? "U85Q" : "P64",
        .gain = s_current_gain, .gain_max = rf_gain_table()->max_index,
        .filter_mode = selected == PROFILE_SELECT_AUTO ? "AUTO" : s_class[selected],
        .filter_active = active >= 0 && active < PROFILES ? s_class[active] : "?",
        .filter_width_khz = calibration_filter_width_khz(),
        .narrow_ok = profile_selectable(PROFILE_NARROW), .wide_ok = profile_selectable(PROFILE_WIDE),
        .dfilt_mode = phy_dfilt_mode(), .dfilt_default = phy_dfilt_mode() == PHY_DFILT_MODE_DEFAULT,
        .filter_recommended = dev > 0 ? s_class[profile_for_deviation(dev)] : NULL,
        .deviation_x100 = dev, .offset_khz = rf_offset_khz(),
        .cn_x10 = receiver_obs_cn_x10(), .firmware = esp_app_get_description()->version,
    };
    experimental_snapshot(v.exp_values);
    menu_ui_render(s_raster.ui, &v);
    sync_dma_c2m(s_raster.ui, sizeof(s_raster.ui));
}

static dma_descriptor_t *node(unsigned index) { return &s_chunks[index / NODE_CHUNK][index % NODE_CHUNK]; }

static bool count_segment(void *ctx, const uint8_t *data, unsigned length)
{
    unsigned *count = ctx;
    if (*count >= MENU_MAX_NODES || length > 4092 || ((uintptr_t)data & 3) || (length & 3)) return false;
    ++*count;
    return true;
}

static bool append_segment(void *ctx, const uint8_t *data, unsigned length)
{
    (void)ctx;
    if (s_node_count >= s_node_capacity || length > 4092 || ((uintptr_t)data & 3) || (length & 3)) return false;
    dma_descriptor_t *d = node(s_node_count++);
    memset(d, 0, sizeof(*d));
    d->dw0.size = length;
    d->dw0.length = length;
    d->dw0.owner = 1;
    d->buffer = (void *)data;
    return true;
}

static void free_nodes(void)
{
    for (unsigned k = 0; k < s_chunk_count; ++k) { heap_caps_free(s_chunks[k]); s_chunks[k] = NULL; }
    s_chunk_count = s_node_capacity = s_node_count = 0;
}

/* Count the raster, reserve exactly that (grow only), link a cycle.
 * Interlaced first; the progressive field only when its chain cannot be
 * reserved (half the descriptors, but not every display keeps it). */
static esp_err_t reserve_chain(video_standard_t standard, bool interlaced, unsigned *required)
{
    menu_raster_init(&s_raster, standard, interlaced);
    s_raster_flat = false;
    *required = 0;
    if (!menu_raster_emit(&s_raster, standard, interlaced, count_segment, required) || !*required)
        return ESP_ERR_INVALID_SIZE;
    unsigned chunks = (*required + NODE_CHUNK - 1u) / NODE_CHUNK;
    while (s_chunk_count < chunks) {
        dma_descriptor_t *c = heap_caps_aligned_alloc(4u, NODE_CHUNK * sizeof(dma_descriptor_t),
                                                      MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL);
        if (!c) return ESP_ERR_NO_MEM;
        s_chunks[s_chunk_count++] = c;
        s_node_capacity += NODE_CHUNK;
    }
    return ESP_OK;
}

static esp_err_t build_chain(video_standard_t standard)
{
    unsigned required = 0;
#ifdef C5VRX_P4LINK
    /* P4-link build (docs/P4LINK.md): always the progressive field. The
     * splash chain is held while receiver_start creates the gain tasks; on a
     * FireBeetle without PSRAM the interlaced one (when it fitted) left them
     * no room: gain_fast/gain_obs ESP_ERR_NO_MEM boot loops (2026-10-31).
     * The P4 line-doubles a progressive field. */
    bool interlaced = false;
    esp_err_t err = reserve_chain(standard, false, &required);
#else
    bool interlaced = true;
    esp_err_t err = reserve_chain(standard, true, &required);
#endif
    if (err == ESP_ERR_NO_MEM && interlaced) {
        interlaced = false;
        err = reserve_chain(standard, false, &required);
        if (err == ESP_OK) printf("[MENU] progressive fallback: interlaced chain did not fit\n");
    }
    if (err != ESP_OK) return err;
    s_interlaced = interlaced;
    s_node_count = 0;
    if (!menu_raster_emit(&s_raster, standard, interlaced, append_segment, NULL) || s_node_count != required)
        return ESP_ERR_INVALID_SIZE;
    for (unsigned k = 0; k < s_node_count; ++k) node(k)->next = node((k + 1u) % s_node_count);
    s_video_std = standard;
    menu_render();
    sync_dma_c2m(&s_raster, sizeof(s_raster));
    for (unsigned k = 0; k < s_chunk_count; ++k) sync_dma_c2m(s_chunks[k], NODE_CHUNK * sizeof(dma_descriptor_t));
    return ESP_OK;
}

/* A short flat DAC level without sync between live video and the menu
 * raster (both ways): the goggle drops its lock on the camera timing and
 * locks fresh, instead of showing the menu briefly, inverting and going
 * black (operator, 2026-10-18). Not in flight: only menu/status entry. */
#define MENU_RELOCK_GAP_MS 150u

static void entry_start(void)
{
    s_menu_active = true;
    transport_stop_tx();
    vTaskDelay(pdMS_TO_TICKS(MENU_RELOCK_GAP_MS));
    transport_start_chain(node(0));
}

bool menu_open(void)
{
    if (s_menu_active) return true;
    /* Allocate and render before touching live video: if memory is short,
     * flight video keeps running. */
    const esp_err_t err = build_chain(menu_standard());
    if (err != ESP_OK) {
        ESP_LOGE("menu", "unavailable: %s (internal free=%u dma_free=%u dma_largest=%u chunks=%u)",
                 esp_err_to_name(err), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL),
                 s_chunk_count);
        free_nodes();
        return false;
    }
    entry_start();
    return true;
}

void menu_close(void)
{
    if (!s_menu_active) return;
    transport_stop_chain();
    vTaskDelay(pdMS_TO_TICKS(MENU_RELOCK_GAP_MS));
    transport_resume_live();
    free_nodes();       /* GDMA no longer references the chain */
    s_menu_active = false;
    s_cal_sub = -1;
    s_exp_sub = -1;
    s_exp_failed = false;
    s_dac_step = DAC_STEP_PATTERN;
    ++s_control_epoch;
}

void menu_set_standard_mode(std_mode_t mode)
{
    s_std_mode = mode;
    if (!s_menu_active) return;
    video_standard_t want = menu_standard();
    if (want == s_video_std) { menu_render(); return; }
    video_standard_t previous = s_video_std;
    transport_stop_chain();
    /* PAL needs ~312 more descriptors; without them keep the old standard. */
    if (build_chain(want) != ESP_OK) ESP_ERROR_CHECK(build_chain(previous));
    transport_start_chain(node(0));
}

