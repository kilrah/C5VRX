/* C5VRX P4 display by Twotoz and the C5VRX contributors: settings and status
 * column (224 x 600, GT911 touch) right of the 4:3 picture (docs/P4LINK.md).
 *
 * Drawn straight into the RGB565 frame buffers with a 5x7 font, after the
 * settings column of the board owner's JC1060P470C camera example
 * (video_lcd_display/main/ui_panel.c). Receiver settings are commands to
 * the C5 (link.c), which keeps them in its own NVS; the display settings
 * are kept here. */
#include "ui.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "cvbs.h"
#include "display.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "link.h"
#include "nvs.h"
#include "uvc.h"

static const char *TAG = "ui";

#define STATUS_H        150
#define ROW_H           64
#define ROWS            7
#define BTN_W           60
#define TOUCH_POLL_MS   30
#define REPEAT_DELAY_MS 400
#define REPEAT_MS       150
#define SAVE_DELAY_US   1000000

#define RGB(r, g, b)    (uint16_t)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define COL_BG          RGB(24, 26, 30)
#define COL_LINE        RGB(60, 64, 72)
#define COL_BTN         RGB(70, 76, 90)
#define COL_BTN_HI      RGB(40, 110, 170)
#define COL_TEXT        RGB(255, 255, 255)
#define COL_LABEL       RGB(170, 176, 190)
#define COL_OK          RGB(60, 190, 90)
#define COL_WARN        RGB(230, 190, 40)
#define COL_BAD         RGB(220, 60, 50)

/* ---------------- display settings (P4 NVS) ---------------- */

enum { S_BACKLIGHT, S_CONTRAST, S_BRIGHT, S_SAT, S_SWAP, S_EDGE, S_COUNT };
static struct { const char *key; int value, min, max, step; } s_set[S_COUNT] = {
    [S_BACKLIGHT] = {"backlight", 100, 10, 100, 10},
    [S_CONTRAST]  = {"contrast", 100, 50, 200, 10},
    [S_BRIGHT]    = {"bright", 0, -64, 64, 8},
    [S_SAT]       = {"sat", 30, 0, 200, 10},     /* 30 %: the FPV link's chroma is noisy (board 2026-10-31) */
    [S_SWAP]      = {"swap", 0, 0, 1, 1},
    [S_EDGE]      = {"edge", 0, 0, 1, 1},
};
static int64_t s_dirty_since;

static void apply(int id)
{
    switch (id) {
    case S_BACKLIGHT: display_set_backlight(s_set[S_BACKLIGHT].value); break;
    case S_CONTRAST: case S_BRIGHT: cvbs_set_picture(s_set[S_CONTRAST].value, s_set[S_BRIGHT].value); break;
    case S_SAT: cvbs_set_saturation(s_set[S_SAT].value); break;
    case S_SWAP: cvbs_set_field_swap(s_set[S_SWAP].value); break;
    case S_EDGE: cvbs_set_sample_edge_neg(s_set[S_EDGE].value); break;
    default: break;
    }
}

static void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open("p4ui", NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < S_COUNT; ++i) {
            int32_t v;
            if (nvs_get_i32(h, s_set[i].key, &v) == ESP_OK && v >= s_set[i].min && v <= s_set[i].max) s_set[i].value = v;
        }
        nvs_close(h);
    }
    for (int i = 0; i < S_COUNT; ++i) apply(i);
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open("p4ui", NVS_READWRITE, &h) != ESP_OK) return;
    for (int i = 0; i < S_COUNT; ++i) (void)nvs_set_i32(h, s_set[i].key, s_set[i].value);
    (void)nvs_commit(h);
    nvs_close(h);
}

static volatile uint32_t s_version = 1;   /* bumps on every local change */

static void change(int id, int dir)
{
    int v = s_set[id].max == 1 && s_set[id].min == 0 ? !s_set[id].value : s_set[id].value + dir * s_set[id].step;
    if (v < s_set[id].min) v = s_set[id].min;
    if (v > s_set[id].max) v = s_set[id].max;
    if (v == s_set[id].value) return;
    s_set[id].value = v;
    apply(id);
    s_dirty_since = esp_timer_get_time();
    ++s_version;
}

/* ---------------- font and drawing ---------------- */

static const uint8_t *glyph(char c)
{
    static const uint8_t digits[10][7] = {
        {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
        {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},
        {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
        {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
        {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    };
    static const uint8_t letters[26][7] = {
        {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
        {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
        {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
        {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}, {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
        {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C},
        {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
        {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11},
        {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
        {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
        {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}, {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
        {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}, {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
        {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
    };
    static const struct { char c; uint8_t g[7]; } other[] = {
        {'+', {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}}, {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
        {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}}, {'/', {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10}},
        {'%', {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03}}, {':', {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}},
        {'<', {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02}}, {'>', {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08}},
        {'_', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}}, {'=', {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00}},
    };
    c = (char)toupper((unsigned char)c);
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    for (size_t k = 0; k < sizeof(other) / sizeof(other[0]); ++k)
        if (other[k].c == c) return other[k].g;
    return NULL;
}

static void fill(uint16_t *fb, int x, int y, int w, int h, uint16_t col)
{
    for (int j = y; j < y + h; ++j) {
        uint16_t *p = fb + (size_t)j * LCD_H_RES + x;
        for (int i = 0; i < w; ++i) p[i] = col;
    }
}

static int text_w(const char *s, int scale) { int n = (int)strlen(s); return n ? n * 6 * scale - scale : 0; }

static void text(uint16_t *fb, int x, int y, const char *s, int scale, uint16_t col)
{
    for (; *s; ++s, x += 6 * scale) {
        const uint8_t *g = glyph(*s);
        if (!g) continue;
        for (int gy = 0; gy < 7; ++gy)
            for (int gx = 0; gx < 5; ++gx)
                if (g[gy] & (0x10 >> gx)) fill(fb, x + gx * scale, y + gy * scale, scale, scale, col);
    }
}

static void text_c(uint16_t *fb, int cx, int y, const char *s, int scale, uint16_t col)
{
    /* Shrink a long value to fit the column. */
    while (scale > 1 && text_w(s, scale) > PANEL_W - 8) --scale;
    text(fb, cx - text_w(s, scale) / 2, y, s, scale, col);
}

/* ---------------- pages ---------------- */

enum { PAGE_RX, PAGE_DISPLAY };
static int s_page;

typedef enum { ROW_NONE, ROW_STEP, ROW_TAP, ROW_PAIR } row_kind_t;
typedef struct {
    row_kind_t kind;
    char label[20], value[20];      /* STEP/TAP; PAIR: the two button texts */
    bool hi[2];
} row_t;

static void rows_for(const c5_status_t *st, row_t r[ROWS])
{
    memset(r, 0, sizeof(row_t) * ROWS);
    if (s_page == PAGE_RX && st->menu) {
        /* The C5's own menu shows in the picture: drive it from here. */
        r[0] = (row_t){.kind = ROW_TAP, .label = "C5 MENU OPEN", .value = "NEXT"};
        r[2] = (row_t){.kind = ROW_TAP, .value = "SELECT"};
        r[6] = (row_t){.kind = ROW_TAP, .value = "EXIT MENU"};
        return;
    }
    if (s_page == PAGE_RX) {
        char band = '?';
        int mhz = st->freq;
        if (!link_channel(st->idx, &band, &mhz)) band = st->ch[0];
        r[0] = (row_t){.kind = ROW_STEP};
        strcpy(r[0].label, "CHANNEL");
        snprintf(r[0].value, sizeof(r[0].value), "%s", st->online ? st->ch : "--");
        r[1] = (row_t){.kind = ROW_STEP};
        strcpy(r[1].label, "BAND");
        snprintf(r[1].value, sizeof(r[1].value), "%c", st->online ? band : '-');
        r[2] = (row_t){.kind = ROW_TAP};
        strcpy(r[2].label, "GAIN");
        snprintf(r[2].value, sizeof(r[2].value), "%s", st->online ? st->gain_mode : "--");
        r[3] = (row_t){.kind = ROW_TAP};
        strcpy(r[3].label, "FILTER");
        snprintf(r[3].value, sizeof(r[3].value), "%s", st->online ? st->profile : "--");
        r[4] = (row_t){.kind = ROW_TAP};
        strcpy(r[4].label, "VIDEO STANDARD");
        snprintf(r[4].value, sizeof(r[4].value), "%s", st->online ? st->std : "--");
        r[5] = (row_t){.kind = ROW_PAIR, .hi = {st->scan, st->afc}};
        strcpy(r[5].label, st->scan ? "SCANNING" : "SCAN");
        strcpy(r[5].value, st->afc ? "AFC ON" : "AFC OFF");
        r[6] = (row_t){.kind = ROW_PAIR, .label = "C5 MENU", .value = "MORE >"};
    } else {
        r[0] = (row_t){.kind = ROW_STEP};
        strcpy(r[0].label, "BACKLIGHT");
        snprintf(r[0].value, sizeof(r[0].value), "%d%%", s_set[S_BACKLIGHT].value);
        r[1] = (row_t){.kind = ROW_STEP};
        strcpy(r[1].label, "CONTRAST");
        snprintf(r[1].value, sizeof(r[1].value), "%d%%", s_set[S_CONTRAST].value);
        r[2] = (row_t){.kind = ROW_STEP};
        strcpy(r[2].label, "BRIGHTNESS");
        snprintf(r[2].value, sizeof(r[2].value), "%+d", s_set[S_BRIGHT].value);
        r[3] = (row_t){.kind = ROW_STEP};
        strcpy(r[3].label, "SATURATION");
        if (s_set[S_SAT].value) snprintf(r[3].value, sizeof(r[3].value), "%d%%", s_set[S_SAT].value);
        else strcpy(r[3].value, "B/W");
        r[4] = (row_t){.kind = ROW_PAIR, .label = "CAL QUICK", .value = "CAL FULL"};
        r[5] = (row_t){.kind = ROW_PAIR, .hi = {s_set[S_EDGE].value, s_set[S_SWAP].value}};
        strcpy(r[5].label, s_set[S_EDGE].value ? "CLK FALL" : "CLK RISE");
        strcpy(r[5].value, s_set[S_SWAP].value ? "FIELDS SWAP" : "FIELDS STD");
        r[6] = (row_t){.kind = ROW_PAIR, .value = "< BACK"};
        snprintf(r[6].label, sizeof(r[6].label), "%s", st->online ? st->demod : "--");
    }
}

static void draw_status(uint16_t *fb, const c5_status_t *st)
{
    const int x0 = PANEL_X0 + 8, right = LCD_H_RES - 8;
    char buf[32];
    if (!st->online) {
        text_c(fb, PANEL_X0 + PANEL_W / 2, 20, "NO C5", 4, COL_BAD);
        text_c(fb, PANEL_X0 + PANEL_W / 2, 64, "CHECK UART LINK", 2, COL_LABEL);
    } else {
        text(fb, x0, 8, st->ch, 5, COL_TEXT);
        snprintf(buf, sizeof(buf), "%d", st->freq);
        text(fb, right - text_w(buf, 3), 10, buf, 3, COL_TEXT);
        text(fb, right - text_w("MHZ", 2), 36, "MHZ", 2, COL_LABEL);
        if (st->busy) text(fb, x0, 54, "BUSY", 2, COL_WARN);
        else if (st->video) {
            snprintf(buf, sizeof(buf), "VIDEO %s", st->det);
            text(fb, x0, 54, buf, 2, COL_OK);
        } else text(fb, x0, 54, st->scan ? "SCANNING" : "NO VIDEO", 2, st->scan ? COL_WARN : COL_BAD);
        /* signal strength bar */
        int s = st->sig < 0 ? 0 : st->sig > 100 ? 100 : st->sig;
        fill(fb, x0, 76, PANEL_W - 16, 10, COL_LINE);
        fill(fb, x0, 76, (PANEL_W - 16) * s / 100, 10, s > 50 ? COL_OK : s > 20 ? COL_WARN : COL_BAD);
        if (st->cn_x10 > -900 && st->cn_x10 < 9000) snprintf(buf, sizeof(buf), "CN %d.%d G%d", st->cn_x10 / 10, abs(st->cn_x10 % 10), st->gain);
        else snprintf(buf, sizeof(buf), "CN -- G%d", st->gain);
        text(fb, x0, 94, buf, 2, COL_LABEL);
        snprintf(buf, sizeof(buf), "AFC %s %+d", st->afc_lock ? "LOCK" : st->afc ? "SRCH" : "OFF", st->cfo_khz);
        text(fb, x0, 112, buf, 2, COL_LABEL);
    }
    cvbs_stats_t cs;
    cvbs_get_stats(&cs);
    if (uvc_streaming()) snprintf(buf, sizeof(buf), "%u FPS USB %u", (unsigned)cs.frames_ps, (unsigned)uvc_fps());
    else snprintf(buf, sizeof(buf), "%u FPS %s", (unsigned)cs.frames_ps, cs.locked ? (cs.pal ? "PAL" : "NTSC") : "NO SYNC");
    text(fb, x0, 130, buf, 2, COL_LABEL);
}

static void draw_panel(uint16_t *fb, const c5_status_t *st)
{
    fill(fb, PANEL_X0, 0, PANEL_W, LCD_V_RES, COL_BG);
    fill(fb, PANEL_X0, 0, 1, LCD_V_RES, COL_LINE);
    draw_status(fb, st);
    row_t r[ROWS];
    rows_for(st, r);
    const int cx = PANEL_X0 + PANEL_W / 2;
    for (int i = 0; i < ROWS; ++i) {
        const int y0 = STATUS_H + i * ROW_H;
        fill(fb, PANEL_X0, y0, PANEL_W, 1, COL_LINE);
        switch (r[i].kind) {
        case ROW_STEP:
            text_c(fb, cx, y0 + 5, r[i].label, 2, COL_LABEL);
            fill(fb, PANEL_X0 + 6, y0 + 24, BTN_W, 36, COL_BTN);
            fill(fb, LCD_H_RES - 6 - BTN_W, y0 + 24, BTN_W, 36, COL_BTN);
            text_c(fb, PANEL_X0 + 6 + BTN_W / 2, y0 + 32, "-", 3, COL_TEXT);
            text_c(fb, LCD_H_RES - 6 - BTN_W / 2, y0 + 32, "+", 3, COL_TEXT);
            text_c(fb, cx, y0 + 32, r[i].value, 3, COL_TEXT);
            break;
        case ROW_TAP:
            if (r[i].label[0]) text_c(fb, cx, y0 + 5, r[i].label, 2, COL_LABEL);
            fill(fb, PANEL_X0 + 6, y0 + 24, PANEL_W - 12, 36, COL_BTN);
            text_c(fb, cx, y0 + 32, r[i].value, 3, COL_TEXT);
            break;
        case ROW_PAIR: {
            const int w = (PANEL_W - 18) / 2;
            fill(fb, PANEL_X0 + 6, y0 + 8, w, 50, r[i].hi[0] ? COL_BTN_HI : COL_BTN);
            fill(fb, PANEL_X0 + 12 + w, y0 + 8, w, 50, r[i].hi[1] ? COL_BTN_HI : COL_BTN);
            for (int b = 0; b < 2; ++b) {
                const char *t = b ? r[i].value : r[i].label;
                const int sc = text_w(t, 2) <= w - 6 ? 2 : 1;
                text(fb, PANEL_X0 + 6 + b * (w + 6) + (w - text_w(t, sc)) / 2, y0 + 33 - 7 * sc / 2, t, sc, COL_TEXT);
            }
            break;
        }
        default: break;
        }
    }
}

/* Short notices over the picture: link, data clock and sync state. */
static void draw_overlay(uint16_t *fb, const c5_status_t *st)
{
    cvbs_stats_t cs;
    cvbs_get_stats(&cs);
    const char *msg = !st->online ? "NO C5 LINK" : !cs.clock ? "NO VIDEO DATA" : st->scan ? "SCANNING" :
                      !cs.locked ? "NO SYNC" : NULL;
    if (!msg) return;
    const int w = text_w(msg, 3) + 16;
    display_invalidate(fb, VIDEO_H - 50, VIDEO_H - 13, (VIDEO_W - w) / 2, (VIDEO_W + w) / 2);
    fill(fb, (VIDEO_W - w) / 2, VIDEO_H - 50, w, 37, RGB(0, 0, 0));
    text(fb, (VIDEO_W - w) / 2 + 8, VIDEO_H - 42, msg, 3, COL_WARN);
}

static uint32_t s_drawn[2];

static uint32_t current_version(void)
{
    c5_status_t st;
    link_status(&st);
    return s_version * 7919u + st.seq * 31u + st.online;
}

bool ui_dirty(int fb_index) { return s_drawn[fb_index & 1] != current_version(); }

void ui_draw(uint16_t *fb, int fb_index)
{
    c5_status_t st;
    link_status(&st);
    draw_overlay(fb, &st);
    const uint32_t v = current_version();
    if (s_drawn[fb_index & 1] == v) return;
    draw_panel(fb, &st);
    s_drawn[fb_index & 1] = v;
}

/* ---------------- touch ---------------- */

static esp_lcd_touch_handle_t s_touch;

static void press(int x, int y, bool repeat)
{
    if (x < PANEL_X0 || y < STATUS_H) return;
    const int row = (y - STATUS_H) / ROW_H;
    if (row >= ROWS) return;
    c5_status_t st;
    link_status(&st);
    row_t r[ROWS];
    rows_for(&st, r);
    const bool left = x < PANEL_X0 + PANEL_W / 2;
    const bool minus = x < PANEL_X0 + 6 + BTN_W + 10, plus = x > LCD_H_RES - 6 - BTN_W - 10;
    if (r[row].kind != ROW_STEP && repeat) return;
    if (s_page == PAGE_RX && st.menu) {
        switch (row) {
        case 0: case 1: link_key('n'); break;
        case 2: case 3: link_key('x'); break;
        case 6: link_key('e'); break;
        default: break;
        }
        return;
    }
    if (s_page == PAGE_RX) {
        switch (row) {
        case 0:   /* channel within the band */
            if (minus) link_tune(st.idx / 8 * 8 + (st.idx % 8 + 7) % 8);
            else if (plus) link_tune(st.idx / 8 * 8 + (st.idx % 8 + 1) % 8);
            break;
        case 1:   /* band, same channel number */
            if (minus) link_tune((st.idx + LINK_CHANNELS - 8) % LINK_CHANNELS);
            else if (plus) link_tune((st.idx + 8) % LINK_CHANNELS);
            break;
        case 2: link_key('G'); break;
        case 3: link_key('f'); break;
        case 4: link_key('v'); break;
        case 5: if (left) link_key('s'); else link_afc_toggle(); break;
        case 6: if (left) link_key('o'); else { s_page = PAGE_DISPLAY; ++s_version; } break;
        default: break;
        }
    } else {
        switch (row) {
        case 0: case 1: case 2: case 3:
            if (minus || plus)
                change(row == 0 ? S_BACKLIGHT : row == 1 ? S_CONTRAST : row == 2 ? S_BRIGHT : S_SAT, minus ? -1 : 1);
            break;
        case 4: link_key(left ? 'F' : 'B'); break;
        case 5: change(left ? S_EDGE : S_SWAP, 1); break;
        case 6: if (left) link_key('m'); else { s_page = PAGE_RX; ++s_version; } break;   /* decoder P64 <-> U85Q */
        default: break;
        }
    }
}

static void touch_task(void *arg)
{
    (void)arg;
    bool was = false;
    int64_t since = 0, last = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));
        const int64_t now = esp_timer_get_time();
        if (s_dirty_since && now - s_dirty_since > SAVE_DELAY_US) { s_dirty_since = 0; settings_save(); }
        esp_lcd_touch_point_data_t pt;
        uint8_t cnt = 0;
        const bool pressed = esp_lcd_touch_read_data(s_touch) == ESP_OK &&
                             esp_lcd_touch_get_data(s_touch, &pt, &cnt, 1) == ESP_OK && cnt > 0;
        if (pressed) {
            if (!was) { since = last = now; press(pt.x, pt.y, false); }
            else if (now - since > REPEAT_DELAY_MS * 1000 && now - last > REPEAT_MS * 1000) { last = now; press(pt.x, pt.y, true); }
        }
        was = pressed;
    }
}

esp_err_t ui_start(void)
{
    settings_load();
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bcfg = {
        .i2c_port = 0,
        .sda_io_num = TOUCH_SDA_GPIO,
        .scl_io_num = TOUCH_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bcfg, &bus), TAG, "I2C");
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io_cfg.scl_speed_hz = 100000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &io), TAG, "touch io");
    const esp_lcd_touch_config_t tcfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_gt911(io, &tcfg, &s_touch), TAG, "GT911");
    return xTaskCreatePinnedToCore(touch_task, "touch", 4096, NULL, 3, NULL, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
