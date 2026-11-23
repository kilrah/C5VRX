/* C5VRX by Twotoz/contributors: menu picture on the host.
 * Every page and status screen renders with worst-case strings: no text is
 * clipped, no code leaves the picture range (black 20 .. 63; the DAC TEST
 * ramp is the only content above white), and the pane and sidebar keep
 * their columns. With a directory argument the frames are also written as
 * PGM files (one sample per column, one UI row per line) for
 * tools/menu_preview.py. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "menu_ui.h"

static menu_ui_rows_t ui;
static const char *s_dir;
static unsigned s_frames, s_overflow_frames;

static void check(const char *name, bool dac)
{
    if (menu_ui_overflow) {
        fprintf(stderr, "%s: %u text overflow(s)\n", name, menu_ui_overflow);
        ++s_overflow_frames;
    }
    for (unsigned y = 0; y < MENU_UI_LINES; ++y)
        for (unsigned x = 0; x < MENU_UI_BYTES; ++x) {
            const uint8_t c = ui[y][x];
            assert(c >= MENU_INK_BLACK && c <= 63);
            if (!dac) assert(c <= MENU_INK_WHITE);
        }
    ++s_frames;
    if (!s_dir) return;
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.pgm", s_dir, name);
    FILE *f = fopen(path, "wb");
    assert(f);
    fprintf(f, "P5\n%u %u\n63\n", MENU_UI_BYTES, MENU_UI_LINES);
    fwrite(ui, 1, sizeof(ui), f);
    fclose(f);
}

static menu_view_t base(void)
{
    return (menu_view_t){
        .cal_sub = -1, .channel = "R1", .freq_mhz = 5658, .band = 0, .channel_in_band = 0, .band_name = "RACEBAND R",
        .band_letters = "RABEFL", .band_count = 6, .signal = 72, .video = true, .level_valid = true,
        .level_above_noise = true, .level_dbm = -71, .dc_need = "", .std_mode = 0, .std_out_pal = true,
        .detected_valid = true, .detected_pal = true, .output = 1, .output_name = "1.5x", .gain_mode = "AUTO",
        .gain_auto = true, .gain = 42, .gain_max = 78, .filter_mode = "AUTO", .filter_active = "NORMAL",
        .filter_width_khz = 14375, .narrow_ok = true, .wide_ok = true, .dfilt_mode = 12, .dfilt_default = true,
        .deviation_x100 = 73, .offset_khz = 81,
        .cn_x10 = 124, .firmware = "5.0.0-afcwin", .demod = "U85Q",
    };
}

int main(int argc, char **argv)
{
    static const char *const names[MENU_ITEMS] = {"band", "channel", "scan", "video", "filter", "output", "calibrate", "experimental", "exit"};
    s_dir = argc > 1 ? argv[1] : NULL;
    char name[64];
    for (int i = 0; i < MENU_ITEMS; ++i) {
        menu_view_t v = base();
        v.cursor = i;
        menu_ui_render(ui, &v);
        check(names[i], i == MENU_DAC);
    }
    /* Worst cases: longest band name, warnings, scanning, fixed gain, the
     * widest channel/frequency/level strings, every output level. */
    menu_view_t v = base();
    v.cursor = MENU_BAND; v.band = 4; v.band_name = "FATSHARK F"; v.channel = "F8"; v.freq_mhz = 5880;
    v.channel_in_band = 7; v.dc_cal = true; v.band_name = "FATSHARK F"; v.dc_need = "NORMAL+NARROW+WIDE";
    menu_ui_render(ui, &v); check("band_dc_warning", false);
    v.cursor = MENU_CALIBRATE; menu_ui_render(ui, &v); check("calibrate_dc", false);
    v = base(); v.cursor = MENU_SCAN; v.scan_active = true; v.scan_progress = 100; v.video = false; v.signal = 0;
    menu_ui_render(ui, &v); check("scan_active", false);
    v = base(); v.cursor = MENU_GAIN; v.gain_auto = false; v.gain_fixed = true; v.gain_mode = "MAX"; v.gain = 78;
    menu_ui_render(ui, &v); check("gain_fixed", false);
    v.gain_mode = "G24"; v.gain = 24; menu_ui_render(ui, &v); check("gain_g24", false);
    v = base(); v.cursor = MENU_CHANNEL; v.level_dbm = -100; v.channel = "L8"; v.freq_mhz = 5621;
    menu_ui_render(ui, &v); check("channel_weak", false);
    v.level_valid = false; menu_ui_render(ui, &v); check("channel_nolevel", false);
    v = base(); v.cursor = MENU_VIDEO; v.std_mode = 1; v.std_out_pal = false; v.detected_valid = false;
    menu_ui_render(ui, &v); check("video_ntsc_novideo", false);
    static const char *const outputs[10] = {"1.0x", "1.5x", "2.0x", "2.5x", "3.0x", "3.5x", "4.0x", "4.5x", "5.0x",
                                            "0.5x"};
    for (unsigned k = 0; k < 10; ++k) {
        v = base(); v.cursor = MENU_OUTPUT; v.output = k; v.output_name = outputs[k];
        snprintf(name, sizeof(name), "output_%u", k);
        menu_ui_render(ui, &v); check(name, false);
    }
    /* FILTER: every choice, unknown width/deviation, missing classes, the
     * widest offset and C/N strings; firmware: the longest version string. */
    static const char *const modes[5] = {"AUTO", "XNARROW", "NARROW", "NORMAL", "WIDE"};
    for (unsigned k = 0; k < 5; ++k) {
        v = base(); v.cursor = MENU_FILTER; v.filter_mode = modes[k]; v.filter_active = k ? modes[k] : "NORMAL";
        v.filter_recommended = k == 1 ? "NORMAL" : NULL;
        snprintf(name, sizeof(name), "filter_%u", k);
        menu_ui_render(ui, &v); check(name, false);
    }
    v = base(); v.cursor = MENU_FILTER; v.filter_width_khz = 0; v.deviation_x100 = -1; v.video = false;
    v.narrow_ok = v.wide_ok = false; menu_ui_render(ui, &v); check("filter_unknown", false);
    v = base(); v.cursor = MENU_FILTER; v.filter_mode = "NARROW"; v.filter_active = "NARROW";
    v.filter_width_khz = 19950; v.offset_khz = -2500; v.deviation_x100 = 400; v.wide_ok = false;
    menu_ui_render(ui, &v); check("filter_worst", false);
    for (int m = 0; m <= 12; m += 4) {   /* every PHY FILT mode, with the BOOT note off the default */
        v = base(); v.cursor = MENU_DFILT; v.dfilt_mode = m; v.dfilt_default = m == 12;
        menu_ui_render(ui, &v); check(m == 12 ? "dfilt_12" : m == 0 ? "dfilt_0" : m == 4 ? "dfilt_4" : "dfilt_8", false);
    }
    v = base(); v.cursor = MENU_AFC; v.afc_off = true;
    menu_ui_render(ui, &v); check("afc_off", false);
    v.cursor = MENU_FILTER; menu_ui_render(ui, &v); check("filter_afc_off", false);
    v = base(); v.cursor = MENU_CHANNEL; v.level_dbm = -100; v.cn_x10 = -305; menu_ui_render(ui, &v); check("cn_negative", false);
    v.cn_x10 = 455; menu_ui_render(ui, &v); check("cn_high", false);
    v.cn_x10 = -990; menu_ui_render(ui, &v); check("cn_unknown", false);
    v = base(); v.cursor = MENU_EXIT; v.firmware = "5.0.0-pr.9999"; menu_ui_render(ui, &v); check("exit_fw", false);
    /* CALIBRATE submenu: every row, with the DC warning. */
    for (int k = 0; k < MENU_CAL_ITEMS; ++k) {
        v = base(); v.cursor = MENU_CALIBRATE; v.cal_sub = k; v.dc_cal = k == 0;
        snprintf(name, sizeof(name), "cal_sub_%d", k);
        menu_ui_render(ui, &v); check(name, false);
    }
    /* Scrolling: the cursor row is drawn (white pill) at every position. */
    for (int i = 0; i < MENU_ITEMS; ++i) {
        v = base(); v.cursor = i; menu_ui_render(ui, &v);
        bool pill = false;
        for (unsigned y = 0; y < MENU_UI_LINES && !pill; ++y) pill = ui[y][2] == MENU_INK_WHITE;
        assert(pill);
    }
    /* Every submenu row/value and marker: same renderer as the firmware. */
    for (unsigned i = 0; i < EXP_MENU_ITEMS; ++i) {
        const unsigned count = i < EXP_ITEMS && experimental_items[i].count ? experimental_items[i].count : 1;
        for (unsigned j = 0; j < count; ++j) {
            v = base(); v.exp_open = true; v.exp_active = true; v.exp_cursor = i;
            if (i < EXP_ITEMS) v.exp_values[i] = j;
            snprintf(name, sizeof(name), "exp_%u_%u", i, j);
            menu_ui_render(ui, &v); check(name, i == EXP_DAC);
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        v = base(); v.exp_open = true; v.exp_active = true; v.exp_cursor = EXP_IQ_LANE;
        v.exp_reboot = i == 0; v.exp_coarse = i == 1; v.exp_failed = i == 2;
        menu_ui_render(ui, &v); check("exp_status", false);
    }
    for (unsigned on = 0; on < 2; ++on) {
        v = base(); v.exp_open = true; v.exp_cursor = EXP_CLOCK_D10;
        v.exp_values[EXP_CLOCK_D10] = on; v.clock_d10 = !on; v.exp_reboot = true;
        menu_ui_render(ui, &v); check("clock_reboot", false);
        v.exp_cursor = EXP_DAC;
        menu_ui_render(ui, &v); check("clock_dac", true);
    }
    const menu_status_view_t exp_status = {.title="CALIBRATING", .line1="VTX OFF", .line2="",
                                            .percent=30, .channel="R1", .freq_mhz=5865, .exp_active=true};
    menu_ui_render_status(ui, &exp_status); check("status_exp", false);
    /* Status screens: every title/line calibration.c and profile.c show. */
    static const menu_status_view_t status[] = {
        {"CALIBRATING", "LOOKING FOR A QUIET CHANNEL", "ONE TIME ONLY, ABOUT 30 S", -1, "R1", 5658, false},
        {"CALIBRATING DC", "ROW 12 OF 40", "VTX MAY STAY ON", 30, "E8", 5945, false},
        {"FILTER NOT CALIBRATED", "NO QUIET CHANNEL FOUND", "SAFE DEFAULT, RETRY NEXT BOOT", -1, "R1", 5658, false},
        {"DC CALIBRATED", "LOWER GAINS NEED NO DC CORRECTION", "", 100, "R1", 5658, false},
        {"GAIN NOT AUTO", "FIXED G24 COSTS RANGE", "MENU > GAIN, HOLD TO CHANGE", -1, "R1", 5658, false},
        {"BENCH LANE TEST", "VIDEO STOPS FOR ABOUT 3 S", "SAME GAIN, FILTER AND CHANNEL", -1, "R1", 5658, false},
    };
    for (unsigned k = 0; k < sizeof(status) / sizeof(status[0]); ++k) {
        menu_ui_render_status(ui, &status[k]);
        snprintf(name, sizeof(name), "status_%u", k);
        check(name, false);
    }
    for (int pct = 0; pct <= 100; pct += 50) {
        menu_ui_render_splash(ui, pct);
        snprintf(name, sizeof(name), "splash_%d", pct);
        check(name, false);
    }
    assert(s_overflow_frames == 0);   /* after every frame, so all PGMs are written */
    printf("menu ui: %u frames, no overflow, codes in range\n", s_frames);
    return 0;
}
