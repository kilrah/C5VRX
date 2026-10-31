/* C5VRX-5 by Twotoz and the C5VRX contributors: menu picture (menu_ui.h).
 *
 * Layout on the 384 x 56 grid: one settings list, nine rows visible at once
 * (a row of text plus a row of air each; more items scroll), label left and
 * value right, the selected row on a white pill. A live card on the right
 * keeps the frequency, signal, level and C/N in view (FILTER: width, VTX
 * deviation and AFC; EXIT: firmware), and says in one word what a long
 * press does. One button: press = next row, hold = change it. */
#include "menu_ui.h"
#include <stdio.h>
#include <string.h>
#include "dac_check.h"
#include "menu_font.h"
#include "menu_logo.h"

unsigned menu_ui_overflow;

static uint8_t (*s_ui)[MENU_UI_BYTES];

#define W ((int)MENU_UI_WIDTH)
#define H ((int)MENU_UI_LINES)
#define TEXT_H ((int)MENU_TEXT_ROWS)
#define BIG_H ((int)MENU_DISPLAY_ROWS)

/* Settings list and live card. */
#define LIST_W 232
#define LIST_PITCH 6
#define CARD_X 248
#define CARD_R (W - 4)
#define CARD_W (CARD_R - CARD_X)

static unsigned sample(int x) { return (unsigned)x * MENU_UI_X_SCALE_NUM / MENU_UI_X_SCALE_DEN; }

/* Rendering one row at a time (render_rows): only row s_row is drawn, into
 * s_band. Clearing the whole image and drawing it again while the TX DMA
 * scanned it sent rows out black: a band of black lines crawling up the
 * picture at every refresh (P4 display, board 2026-10-31; the XIAO's
 * composite output gets the same). */
static int s_row = -1;                       /* -1: the whole image (s_ui) */
static uint8_t s_band[MENU_UI_BYTES];
static uint8_t *row_at(int y) { return s_row < 0 ? s_ui[y] : s_band; }

static void rect(int x, int y, int w, int h, uint8_t code)
{
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > W ? W : x + w, y1 = y + h > H ? H : y + h;
    if (s_row >= 0) {
        if (s_row < y0 || s_row >= y1) return;
        y0 = s_row;
        y1 = s_row + 1;
    }
    if (x1 <= x0 || y1 <= y0) return;
    const unsigned s0 = sample(x0), s1 = sample(x1);
    for (int yy = y0; yy < y1; ++yy) memset(&row_at(yy)[s0], code, s1 - s0);
}

/* Rounded box: the outer rows are pulled in, which on this grid reads as
 * a radius of about one row. */
static void pill(int x, int y, int w, int h, uint8_t code)
{
    if (h < 3) { rect(x, y, w, h, code); return; }
    const int r = h >= 7 ? 4 : 3, step = h >= 7 ? 1 : 0;   /* taller pills: a second, softer step */
    rect(x + r, y, w - 2 * r, 1, code);
    rect(x + r, y + h - 1, w - 2 * r, 1, code);
    if (step) { rect(x + 1, y + 1, w - 2, 1, code); rect(x + 1, y + h - 2, w - 2, 1, code); }
    rect(x, y + 1 + step, w, h - 2 - 2 * step, code);
}

/* ---- text ---- */

typedef enum { FONT_TEXT, FONT_BIG } font_t;
typedef enum { ALIGN_LEFT, ALIGN_CENTER, ALIGN_RIGHT } align_t;

static unsigned glyph_index(char c)
{
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    return (c >= 32 && c <= 95) ? (unsigned)(c - 32) : (unsigned)('?' - 32);
}

static int glyph_width(font_t f, char c)
{
    const unsigned i = glyph_index(c);
    return f == FONT_BIG ? s_menu_display_font[i].width : s_menu_text_font[i].width;
}

static int default_track(font_t f) { return f == FONT_BIG ? 5 : 2; }

static int text_width_track(const char *s, font_t f, int track)
{
    int w = 0;
    for (const char *p = s; p && *p; ++p) w += glyph_width(f, *p) + (p[1] ? track : 0);
    return w;
}

static int text_width(const char *s, font_t f) { return text_width_track(s, f, default_track(f)); }

/* One row mask as runs, clipped at clip_r. */
static void mask_row(uint32_t bits, int x, int y, int clip_r, uint8_t code)
{
    for (int b = 0; bits; ) {
        if (!(bits & 1u)) { bits >>= 1; ++b; continue; }
        int start = b;
        while (bits & 1u) { bits >>= 1; ++b; }
        int x0 = x + start, x1 = x + b;
        if (x1 > clip_r) x1 = clip_r;
        if (x1 > x0) rect(x0, y, x1 - x0, 1, code);
    }
}

static void glyph5(const menu_glyph5_t *g, int x, int y, int clip_r, uint8_t code)
{
    for (int r = 0; r < TEXT_H; ++r) mask_row(g->rows[r], x, y + r, clip_r, code);
}

static int draw(const char *s, int x, int y, font_t f, int track, int clip_r, uint8_t code)
{
    for (const char *p = s; p && *p && x < clip_r; ++p) {
        const unsigned i = glyph_index(*p);
        if (f == FONT_BIG)
            for (int r = 0; r < BIG_H; ++r) mask_row(s_menu_display_font[i].rows[r], x, y + r, clip_r, code);
        else
            glyph5(&s_menu_text_font[i], x, y, clip_r, code);
        x += glyph_width(f, *p) + track;
    }
    return x - track;
}

/* Text in a box [x, x + w): tracking tightens before anything is clipped;
 * a clip counts as an overflow. Returns the right edge of the ink. */
static int text_box(const char *s, int x, int y, int w, align_t a, font_t f, uint8_t code)
{
    int track = default_track(f), tw = text_width_track(s, f, track);
    while (tw > w && track > 0) tw = text_width_track(s, f, --track);
    if (tw > w) { ++menu_ui_overflow; tw = w; }
    const int left = a == ALIGN_LEFT ? x : a == ALIGN_CENTER ? x + (w - tw) / 2 : x + w - tw;
    return draw(s, left, y, f, track, x + w, code);
}

static int text(const char *s, int x, int y, uint8_t code)
{
    return text_box(s, x, y, W - x, ALIGN_LEFT, FONT_TEXT, code);
}

/* ---- components ---- */

/* A horizontal meter: grey track, white fill, two rows. */
static void meter(int x, int y, int w, int percent)
{
    percent = percent < 0 ? 0 : percent > 100 ? 100 : percent;
    rect(x, y, w, 2, MENU_INK_TRACK);
    if (percent) rect(x, y, w * percent / 100, 2, MENU_INK_WHITE);
}

/* Signal bars: five, one row taller each, bottom aligned at y + 4. */
static void signal_bars(int x, int y, int signal)
{
    const int bars = signal <= 0 ? 0 : signal >= 100 ? 5 : (signal + 19) / 20;
    for (int i = 0; i < 5; ++i) rect(x + i * 5, y + 4 - i, 3, i + 1, i < bars ? MENU_INK_WHITE : MENU_INK_TRACK);
}

/* Small inverted badge (warnings, HOLD key). Returns its right edge. */
static int badge(const char *s, int x, int y, uint8_t fill, uint8_t ink)
{
    const int w = text_width(s, FONT_TEXT) + 10;
    pill(x, y, w, 7, fill);
    text_box(s, x + 5, y + 1, w - 10, ALIGN_LEFT, FONT_TEXT, ink);
    return x + w;
}

/* ---- settings list ---- */

static const char *const s_items[MENU_ITEMS] = {
    "BAND", "CHANNEL", "SCAN", "VIDEO", "FILTER", "OUTPUT", "CALIBRATE", "EXPERIMENTAL", "EXIT",
};

/* "14.4M" (MHz; the FILTER card spells it out), or "--" while the active
 * filter has no measured width. */
static const char *width_text(unsigned khz, char *buf, size_t size)
{
    if (!khz) return "--";
    snprintf(buf, size, "%u.%uM", (khz + 50u) / 1000u, (khz + 50u) % 1000u / 100u);
    return buf;
}

/* The value a row shows: main text plus an optional grey note (the live
 * state behind an AUTO setting). Actions show none. */
static void row_value(const menu_view_t *v, int item, const char **value, const char **note)
{
    static char band[2], width[16];
    *value = *note = NULL;
    switch (item) {
    case MENU_BAND:
        band[0] = v->band < v->band_count ? v->band_letters[v->band] : '?';
        band[1] = 0;
        *value = band;
        break;
    case MENU_CHANNEL: *value = v->channel; break;
    case MENU_VIDEO:
        *value = v->std_mode == 1 ? "NTSC" : v->std_mode == 2 ? "PAL" : "AUTO";
        if (v->std_mode == 0) *note = v->detected_valid ? (v->detected_pal ? "PAL" : "NTSC") : "--";
        break;
    case MENU_FILTER:
        *value = v->filter_mode;
        *note = width_text(v->filter_width_khz, width, sizeof(width));
        break;
    case MENU_AFC: *value = v->afc_off ? "OFF" : "ON"; break;
    case MENU_DFILT: {
        static char mode[8];
        snprintf(mode, sizeof(mode), "MODE %d", v->dfilt_mode);
        *value = mode;
        if (!v->dfilt_default) *note = "LAB";      /* the maps are measured in 12 */
        break;
    }
    case MENU_EXPERIMENTAL: if (v->exp_active) *value = "EXP"; break;
    case MENU_CALIBRATE: if (v->dc_cal) *value = "DC!"; break;
    case MENU_OUTPUT: *value = v->output_name; break;
    case MENU_GAIN: *value = v->gain_mode; break;
    case MENU_DEMOD: *value = v->demod; break;
    default: break;
    }
}

/* Nine rows fit; with more items the list scrolls so the cursor stays in
 * view, and a thin bar beside it shows where the window is. */
static int list_first(int cursor)
{
    const int last = MENU_ITEMS - MENU_VISIBLE_ROWS;
    const int first = cursor - (MENU_VISIBLE_ROWS - 1);
    return last <= 0 || first <= 0 ? 0 : first > last ? last : first;
}

static void list(const menu_view_t *v)
{
    const int first = list_first(v->cursor);
    const int rows = MENU_ITEMS < MENU_VISIBLE_ROWS ? MENU_ITEMS : MENU_VISIBLE_ROWS;
    if (MENU_ITEMS > MENU_VISIBLE_ROWS) {
        const int track = rows * LIST_PITCH - 1, thumb = track * rows / MENU_ITEMS;
        rect(LIST_W + 4, 0, 2, track, MENU_INK_TRACK);
        rect(LIST_W + 4, (track - thumb) * first / (MENU_ITEMS > rows ? MENU_ITEMS - rows : 1), 2, thumb, MENU_INK_GREY);
    }
    for (int i = first; i < first + rows; ++i) {
        const int y = 1 + (i - first) * LIST_PITCH;
        const bool on = i == v->cursor;
        if (on) pill(0, y - 1, LIST_W, 7, MENU_INK_WHITE);
        const char *value, *note;
        row_value(v, i, &value, &note);
        int right = LIST_W - 8;
        if (value) right = text_box(value, 0, y, right, ALIGN_RIGHT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_WHITE);
        if (note) {
            const int nw = text_width(note, FONT_TEXT), vw = text_width(value, FONT_TEXT);
            text_box(note, right - vw - 8 - nw, y, nw, ALIGN_LEFT, FONT_TEXT, MENU_INK_DIM);
        }
        text_box(s_items[i], 9, y, i == MENU_EXPERIMENTAL ? 160 : 100, ALIGN_LEFT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_GREY);
    }
}

/* Submenu uses eight rows and a title; values and labels own separate
 * boxes, including the widest PHASE REPAIR / LEVEL RELEASE labels. */
static void exp_list(const menu_view_t *v)
{
    text_box("EXPERIMENTAL", 9, 1, LIST_W - 18, ALIGN_LEFT, FONT_TEXT, MENU_INK_DIM);
    const unsigned rows = 8;
    const unsigned first = v->exp_cursor >= rows ? v->exp_cursor - rows + 1 : 0;
    for (unsigned i = first; i < first + rows && i < EXP_MENU_ITEMS; ++i) {
        const int y = 8 + (int)(i - first) * LIST_PITCH;
        const bool on = i == v->exp_cursor;
        if (on) pill(0, y - 1, LIST_W, 7, MENU_INK_WHITE);
        const char *label = i < EXP_ITEMS ? experimental_items[i].name :
                            i == EXP_DAC ? "DAC TEST" : i == EXP_RESET ? "RESET ALL" : "BACK";
        const char *value = NULL;
        char mode[12];
        if (i == EXP_GAIN) value = v->gain_mode;
        else if (i == EXP_DFILT) { snprintf(mode, sizeof(mode), "%d", v->dfilt_mode); value = mode; }
        else if (i == EXP_AFC) value = v->afc_off ? "OFF" : "ON";
        else if (i == EXP_DEMOD) value = v->demod;
        else if (i < EXP_ITEMS) {
            const unsigned k = v->exp_values[i];
            value = k < experimental_items[i].count ? experimental_items[i].values[k] : "?";
        }
        text_box(label, 9, y, 138, ALIGN_LEFT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_GREY);
        if (value) text_box(value, 148, y, LIST_W - 156, ALIGN_RIGHT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_WHITE);
    }
}

/* ---- live card ---- */

/* Link state (or a range warning in its place: both cost range without
 * any sign in the picture) beside the signal bars. */
static void card_status(const menu_view_t *v)
{
    signal_bars(CARD_X, 1, v->signal);
    if (v->exp_active) text_box("EXP", CARD_X, 7, CARD_W, ALIGN_RIGHT, FONT_TEXT, MENU_INK_WHITE);
    if (v->dc_cal || v->gain_fixed) badge(v->dc_cal ? "DC CAL!" : "GAIN!", CARD_X + 31, 0, MENU_INK_WHITE, MENU_INK_BLACK);
    else text(v->video ? "VIDEO" : "SEARCH", CARD_X + 32, 1, v->video ? MENU_INK_WHITE : MENU_INK_GREY);
}

static void card_tuning(const menu_view_t *v)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", v->freq_mhz);
    text_box(buf, CARD_X, 12, CARD_W, ALIGN_LEFT, FONT_BIG, MENU_INK_WHITE);
    int x = text(v->channel, CARD_X, 26, MENU_INK_WHITE) + 8;
    if (v->cn_x10 > -990) {   /* C/N (dB): the margin to the picture's edge, where the band name was */
        const int cn = v->cn_x10 > 999 ? 99 : (v->cn_x10 + (v->cn_x10 >= 0 ? 5 : -5)) / 10;
        snprintf(buf, sizeof(buf), "C/N %d", cn);
        text_box(buf, x, 26, CARD_R - x, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
    } else {
        char band[16];   /* "RACEBAND R" -> "RACEBAND": the letter is in the channel */
        snprintf(band, sizeof(band), "%s", v->band_name ? v->band_name : "");
        char *space = strchr(band, ' ');
        if (space) *space = 0;
        text_box(band, x, 26, CARD_R - x, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    }
    if (v->level_valid && v->level_above_noise) snprintf(buf, sizeof(buf), "%d DBM", v->level_dbm);
    else snprintf(buf, sizeof(buf), v->level_valid ? "NOISE" : "NO LEVEL");
    meter(CARD_X, 35, CARD_W, v->signal);
    text_box(buf, CARD_X, 39, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
}

/* FILTER: the measured width large, the class and what decides it, and
 * the VTX facts that matter for the choice (deviation, AFC). */
static void card_filter(const menu_view_t *v)
{
    char buf[32];
    if (v->filter_width_khz) snprintf(buf, sizeof(buf), "%u.%u", (v->filter_width_khz + 50u) / 1000u,
                                      (v->filter_width_khz + 50u) % 1000u / 100u);
    else snprintf(buf, sizeof(buf), "--");
    const int x = text_box(buf, CARD_X, 12, CARD_W - 24, ALIGN_LEFT, FONT_BIG, MENU_INK_WHITE);
    text("MHZ", x + 4, 18, MENU_INK_GREY);
    /* AUTO: the class it chose. A fixed choice (shown in the list) gets
     * what AUTO would pick for the measured VTX instead. A class missing
     * from the hold cycle has no calibrated filter yet (CALIBRATE > FULL). */
    if (!strcmp(v->filter_mode, "AUTO")) {
        text_box(v->filter_active, CARD_X, 26, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
    } else {
        snprintf(buf, sizeof(buf), "REC %s", v->filter_recommended ? v->filter_recommended : "--");
        const bool same = v->filter_recommended && !strcmp(v->filter_recommended, v->filter_mode);
        text_box(buf, CARD_X, 26, CARD_W, ALIGN_LEFT, FONT_TEXT, same ? MENU_INK_GREY : MENU_INK_WHITE);
    }
    if (v->deviation_x100 >= 0) snprintf(buf, sizeof(buf), "DEV %d.%02d", v->deviation_x100 / 100,
                                         v->deviation_x100 % 100);
    else snprintf(buf, sizeof(buf), "DEV --");
    text_box(buf, CARD_X, 33, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    if (v->afc_off) snprintf(buf, sizeof(buf), "AFC OFF");
    else if (v->afc_locked) snprintf(buf, sizeof(buf), "AFC %+dK", v->offset_khz);
    else snprintf(buf, sizeof(buf), "AFC NO LOCK");
    text_box(buf, CARD_X, 40, CARD_W, ALIGN_LEFT, FONT_TEXT, (!v->afc_off && v->afc_locked) ? MENU_INK_WHITE : MENU_INK_GREY);
}

/* CALIBRATE submenu: the title, then QUICK / FULL / BACK in the list's
 * style; the card says what the selected one does and how long it takes. */
static const char *const s_cal_items[MENU_CAL_ITEMS] = {"QUICK", "FULL", "BACK"};

static void cal_list(const menu_view_t *v)
{
    text_box("CALIBRATE", 9, 1, LIST_W - 18, ALIGN_LEFT, FONT_TEXT, MENU_INK_DIM);
    rect(9, 8, LIST_W - 18, 1, MENU_INK_RULE);
    for (int i = 0; i < MENU_CAL_ITEMS; ++i) {
        const int y = 1 + (i + 2) * LIST_PITCH;
        const bool on = i == v->cal_sub;
        if (on) pill(0, y - 1, LIST_W, 7, MENU_INK_WHITE);
        text_box(s_cal_items[i], 9, y, 100, ALIGN_LEFT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_GREY);
        if (i == MENU_CAL_QUICK && v->dc_cal)
            text_box("DC!", 0, y, LIST_W - 8, ALIGN_RIGHT, FONT_TEXT, on ? MENU_INK_BLACK : MENU_INK_WHITE);
    }
}

static void card_cal(const menu_view_t *v)
{
    static const char *const lines[MENU_CAL_ITEMS][3] = {
        {"THIS FILTER", "AND DC ROWS", "ABOUT 30 S"},
        {"ALL FILTERS", "VTX OFF", "1-2 MIN"},
        {"MAIN MENU", "", ""},
    };
    const int k = v->cal_sub < 0 || v->cal_sub >= MENU_CAL_ITEMS ? MENU_CAL_BACK : v->cal_sub;
    text_box(lines[k][0], CARD_X, 14, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
    text_box(lines[k][1], CARD_X, 22, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    text_box(lines[k][2], CARD_X, 33, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_DIM);
}

/* EXIT: which build is running. */
static void card_info(const menu_view_t *v)
{
    text_box("FIRMWARE", CARD_X, 14, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    text_box(v->firmware ? v->firmware : "?", CARD_X, 22, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
    text_box("C5VRX-5", CARD_X, 35, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_DIM);
}

static void card_scan(const menu_view_t *v)
{
    char buf[8];   /* the number in the display font, its % in text size */
    snprintf(buf, sizeof(buf), "%u", v->scan_progress > 100u ? 100u : v->scan_progress);
    const int x = text_box(buf, CARD_X, 12, CARD_W - 16, ALIGN_LEFT, FONT_BIG, MENU_INK_WHITE);
    text("%", x + 4, 18, MENU_INK_GREY);
    text_box("SCANNING", CARD_X, 26, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    meter(CARD_X, 35, CARD_W, (int)v->scan_progress);
}

/* DAC TEST: a ramp over every level from blanking to the top code, then one
 * bar per bit whose halves differ in that bit alone. Measurement codes,
 * not the UI palette. */
static void card_dac(void)
{
    const unsigned x0 = sample(CARD_X), x1 = sample(CARD_R);
    for (unsigned y = 11; y < 21; ++y) {
        if (s_row >= 0 && (int)y != s_row) continue;
        uint8_t *r = row_at((int)y);
        for (unsigned x = x0; x < x1; ++x) r[x] = dac_check_ramp_code(x - x0, x1 - x0);
    }
    const int pitch = CARD_W / DAC_CHECK_BITS, half = (pitch - 4) / 2;
    for (unsigned k = 0; k < DAC_CHECK_BITS; ++k) {
        const int x = CARD_X + (int)k * pitch;
        rect(x, 24, half, 9, dac_check_bar_base(k));
        rect(x + half, 24, half, 9, dac_check_bar_top(k));
        text_box(dac_check_pins[k] + 1, x, 36, 2 * half, ALIGN_CENTER, FONT_TEXT, MENU_INK_GREY);   /* "4" of D4 */
    }
}

/* What a long press does, one word. */
static const char *hold_action(const menu_view_t *v)
{
    if (v->exp_open) return v->exp_cursor == EXP_BACK ? "BACK" : v->exp_cursor == EXP_RESET ? "RESET" :
                            v->exp_cursor == EXP_DAC ? "STEP" : "NEXT";
    if (v->cal_sub >= 0) return v->cal_sub == MENU_CAL_BACK ? "BACK" : "START";
    switch (v->cursor) {
    case MENU_BAND: case MENU_CHANNEL: case MENU_FILTER: case MENU_DFILT: case MENU_OUTPUT: case MENU_GAIN: case MENU_DEMOD: return "NEXT";
    case MENU_SCAN: return v->scan_active ? NULL : "SCAN";
    case MENU_VIDEO: case MENU_AFC: return "CHANGE";
    case MENU_CALIBRATE: case MENU_EXPERIMENTAL: return "OPEN";
    case MENU_DAC: return "STEP";
    default: return "EXIT";
    }
}

/* draw() once per row: each row is drawn complete into s_band and copied over
 * the shown one, so the scan sees a row either old or new, never cleared.
 * Text overflow counts once (the first row's pass counts every text). */
static void render_rows(menu_ui_rows_t ui, void (*draw)(const void *), const void *view)
{
    s_ui = ui;
    menu_ui_overflow = 0;
    unsigned overflow = 0;
    for (int y = 0; y < H; ++y) {
        s_row = y;
        memset(s_band, MENU_INK_BLACK, sizeof(s_band));
        draw(view);
        if (y == 0) overflow = menu_ui_overflow;
        memcpy(ui[y], s_band, sizeof(s_band));
    }
    s_row = -1;
    menu_ui_overflow = overflow;
}

static void draw_menu(const void *view)
{
    const menu_view_t *v = view;
    if (v->exp_open) exp_list(v);
    else if (v->cal_sub >= 0) cal_list(v);
    else list(v);
    pill(CARD_X - 7, 0, W - CARD_X + 7, H, MENU_INK_PANEL);
    card_status(v);
    if (v->exp_open && v->exp_failed) {
        text_box("NOT APPLIED", CARD_X, 16, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
        text_box("RETRY HOLD", CARD_X, 28, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    }
    else if (v->exp_open && v->exp_cursor == EXP_IQ_LANE) {
        text_box(v->exp_reboot ? "REBOOT" : "BOOT SETTING", CARD_X, 14, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
        if (v->exp_reboot) text_box("NEEDED", CARD_X, 22, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
        text_box(v->exp_coarse ? "DC/PHASE OFF" : "DC/PHASE ON", CARD_X, 33, CARD_W, ALIGN_LEFT, FONT_TEXT, MENU_INK_GREY);
    }
    else if (v->exp_open && v->exp_cursor == EXP_DAC) card_dac();
    else if (v->cal_sub >= 0) card_cal(v);
    else if (v->cursor == MENU_DAC) card_dac();
    else if (v->scan_active) card_scan(v);
    else if (v->cursor == MENU_FILTER) card_filter(v);
    else if (v->cursor == MENU_EXIT) card_info(v);
    else card_tuning(v);
    const char *hold = hold_action(v);
    if (hold) {
        const int x = badge("HOLD", CARD_X, 48, MENU_INK_WHITE, MENU_INK_BLACK);
        text_box(hold, x + 6, 49, CARD_R - x - 6, ALIGN_LEFT, FONT_TEXT, MENU_INK_WHITE);
    }
}

void menu_ui_render(menu_ui_rows_t ui, const menu_view_t *v) { render_rows(ui, draw_menu, v); }

/* Full-screen status (calibration and the like): the title in the display
 * font when it fits, two lines and a progress bar. */
static void draw_status(const void *view)
{
    const menu_status_view_t *v = view;
    char buf[24];
    const int x = 12, w = W - 24;
    text(v->exp_active ? "C5VRX EXP" : "C5VRX", x, 1, MENU_INK_WHITE);
    snprintf(buf, sizeof(buf), "%s  %u", v->channel ? v->channel : "", v->freq_mhz);
    text_box(buf, x, 1, w, ALIGN_RIGHT, FONT_TEXT, MENU_INK_GREY);
    rect(x, 9, w, 1, MENU_INK_RULE);
    if (text_width(v->title, FONT_BIG) <= w) {
        text_box(v->title, x, 14, w, ALIGN_CENTER, FONT_BIG, MENU_INK_WHITE);
    } else {   /* too long for the display font: a white title pill */
        const int tw = text_width(v->title, FONT_TEXT) + 16, tx = (W - tw) / 2;
        pill(tx, 16, tw, 7, MENU_INK_WHITE);
        text_box(v->title, tx + 8, 17, tw - 16, ALIGN_LEFT, FONT_TEXT, MENU_INK_BLACK);
    }
    text_box(v->line1, x, 30, w, ALIGN_CENTER, FONT_TEXT, MENU_INK_WHITE);
    text_box(v->line2, x, 37, w, ALIGN_CENTER, FONT_TEXT, MENU_INK_GREY);
    if (v->percent >= 0) {
        const int bw = 240, bx = (W - bw) / 2;
        meter(bx, 47, bw, v->percent);
        snprintf(buf, sizeof(buf), "%d%%", v->percent > 100 ? 100 : v->percent);
        text(buf, bx + bw + 8, 46, MENU_INK_WHITE);
    }
}

void menu_ui_render_status(menu_ui_rows_t ui, const menu_status_view_t *v) { render_rows(ui, draw_status, v); }

void menu_ui_render_splash(menu_ui_rows_t ui, int percent)
{
    menu_ui_render_splash_health(ui, percent, false);
}

typedef struct { int percent; bool needs_calibration; } splash_view_t;

static void draw_splash(const void *view)
{
    const splash_view_t *v = view;
    const int percent = v->percent;
    const bool needs_calibration = v->needs_calibration;
    const int x0 = (W - (int)MENU_LOGO_WIDTH) / 2, y0 = (H - (int)MENU_LOGO_ROWS) / 2 - 3;
    for (int y = 0; y < (int)MENU_LOGO_ROWS; ++y)
        for (int x = 0; x < (int)MENU_LOGO_WIDTH; ++x)
            if (s_menu_logo[y][x] != MENU_INK_BLACK) rect(x0 + x, y0 + y, 1, 1, s_menu_logo[y][x]);
    const int bw = 120;
    meter((W - bw) / 2, y0 + (int)MENU_LOGO_ROWS + 8, bw, percent);
    if (needs_calibration)
        text_box("CALIBRATION RECOMMENDED", 12, H - 7, W - 24, ALIGN_CENTER, FONT_TEXT, MENU_INK_WHITE);
}

void menu_ui_render_splash_health(menu_ui_rows_t ui, int percent, bool needs_calibration)
{
    const splash_view_t v = {percent, needs_calibration};
    render_rows(ui, draw_splash, &v);
}
