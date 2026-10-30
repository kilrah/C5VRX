/* C5VRX-5 by Twotoz and the C5VRX contributors: picture and menu on the
 * Waveshare ESP32-C5-LCD-1.47 (320 x 172 ST7789, SPI), docs/LCD147.md.
 *
 * This board has no DAC. Two tasks below the gain observer and sentinel:
 * - grab (priority 2): the frame grabber (grab.c) demodulates the raw IQ
 *   ring in software into a 228 x 172 (4:3) grey frame, two fields per frame. It
 *   only reads the ring; TX and its verified restarts run as on the DAC
 *   build, without pins. It rests GRAB_REST_MS after every picture, when the
 *   control task (priority 1 in this build, control.c) does its 50 ms tick,
 *   and while the menu is open.
 * - flush (priority 3): sends the grey frame, or the menu's UI rows while
 *   the menu is open, to the panel in 4-row strips, two strip buffers in
 *   turn, so one is converted while the other is on the SPI DMA. It sleeps
 *   while the DMA runs; its work between is tens of microseconds.
 * Pins (Waveshare): SCLK 7, MOSI 6, CS 23, DC 24, RST 26, backlight 10;
 * RGB LED 8 is held low. The IQ loopback pins (rf.h) are unchanged; 4 and 5
 * are the microSD socket's CS and MISO, so leave the socket empty. */
#include "receiver.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_timer.h"
#include "grab.h"
#include "menu_font.h"
#include "rf.h"
#include "sdkconfig.h"

#define LCD_W 320
#define LCD_H 172
#define LCD_GAP_Y 34            /* the 172 visible columns of the 240-column controller */
#define PIN_SCLK 7
#define PIN_MOSI 6
#define PIN_CS 23
#define PIN_DC 24
#define PIN_RST 26
#define PIN_BACKLIGHT 10
#define PIN_RGB_LED 8
#define PIN_BOOT 28             /* the menu button (control.c BOOT_BUTTON), active low */
#define STRIP_ROWS 4             /* 2.5 KiB per buffer: SRAM is tight at the boot peak */
#define GRAB_TIMEOUT_MS 70      /* ~3.5 PAL fields: the rows the first, partial field missed get the third */
#define ROW_KEEP_PICTURES 1     /* a missed row shows the previous picture's row this long, then is interpolated */
#define ROW_BLEND_GAP 8         /* ... but only between fresh rows at most this far apart */
#define GRAB_REST_MS 10         /* after every picture: the tasks below the grabber (control: ~1 ms per tick, ~20 ms per AFC window) */
#define NO_VIDEO_HOLD_US 1000000 /* the last picture stays this long without a captured row, then NO VIDEO */
#define MENU_TOP 2              /* 56 UI rows x 3 = 168 of 172 lines */
#define MENU_ROW_LINES 3

#define IMG_X ((LCD_W - GRAB_W) / 2)   /* 4:3 picture centred, black bars left and right */
_Static_assert(GRAB_W <= LCD_W && GRAB_H == LCD_H, "the grabber's frame fits the panel");

static uint8_t s_img[GRAB_H][GRAB_W];
static uint16_t *s_strip[2];
static unsigned s_strip_next;
static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t s_panel;
static TaskHandle_t s_flush_task, s_grab_task;
static volatile bool s_picture;          /* s_img holds something to show */
static volatile bool s_quiet;            /* bench (console J): no SPI traffic, no grabbing */

void lcd_quiet_toggle(void)
{
    s_quiet = !s_quiet;
    printf("LCD quiet=%u (no panel SPI and no grabbing while 1)\n", s_quiet);
}
static uint16_t s_menu_lut[64];          /* DAC code -> panel RGB565 */
static uint16_t s_menu_x[LCD_W];         /* panel column -> logical menu pixel */

/* Auto levels (as the T-Embed receiver of the UART link): black at the
 * darkest 1 %, white at the brightest 0.5 %, followed slowly, at most 1.6x.
 * Many cameras' white sits below nominal. Q8 fixed point. */
static int s_level_lo, s_level_gain = 256;
static uint8_t s_row_age[GRAB_H];       /* pictures since the row was last captured (console V) */

static struct {
    uint32_t grabs, complete, rows, late, substituted, nosync, wide, vslips, vmisses, cpu_us, flushes;
    int grab_ms, acquire_ms, max_stall_us;
    uint32_t stalls_1ms, stalls_5ms, stalls_20ms;   /* grabs whose longest stall reached that */
    uint32_t stalls_3ms_write, stalls_3ms_nowrite;  /* ... 3 ms or more, with or without a gain write in the grab */
    bool pal, tb_bad;
    float line_us, jitter_ns, carrier_mhz;
    const char *error;
    int64_t t0;
} s_stats;

static inline uint16_t rgb565_be(unsigned r, unsigned g, unsigned b)
{
    const uint16_t c = (uint16_t)(((r & 0xf8u) << 8) | ((g & 0xfcu) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));   /* the panel takes the high byte first */
}

static inline uint16_t grey565(unsigned v) { return rgb565_be(v, v, v); }

/* ---- the panel ---- */

static uint16_t s_image_lut[256];        /* grey -> panel RGB565, auto levels applied */

static void fill_image(uint16_t *dst, int y0, int rows)
{
    for (int y = y0; y < y0 + rows; ++y) {
        const uint8_t *src = s_img[y];
        for (int x = 0; x < IMG_X; ++x) *dst++ = 0;
        for (int x = 0; x < GRAB_W; ++x) *dst++ = s_image_lut[src[x]];
        for (int x = IMG_X + GRAB_W; x < LCD_W; ++x) *dst++ = 0;
    }
}

static void fill_menu(uint16_t *dst, int y0, int rows, const uint8_t (*ui)[MENU_UI_BYTES])
{
    for (int y = y0; y < y0 + rows; ++y) {
        const int r = (y - MENU_TOP) / MENU_ROW_LINES;
        if (y < MENU_TOP || r >= (int)MENU_UI_LINES) {
            for (int x = 0; x < LCD_W; ++x) *dst++ = s_menu_lut[0];
            continue;
        }
        const uint8_t *row = ui[r];
        for (int x = 0; x < LCD_W; ++x) *dst++ = s_menu_lut[row[s_menu_x[x]] & 63u];
    }
}

/* One full screen. draw_bitmap queues the strip's DMA and returns; the next
 * call first waits (asleep) for the previous strip, so a buffer is only
 * refilled after its transfer ended. */
static void flush(const uint8_t (*ui)[MENU_UI_BYTES])
{
    if (!ui) {
        const int lo = s_level_lo, gain = s_level_gain;
        for (int v = 0; v < 256; ++v) {
            const int g = ((v - lo) * gain) >> 8;
            s_image_lut[v] = grey565(g < 0 ? 0u : g > 255 ? 255u : (unsigned)g);
        }
    }
    for (int y = 0; y < LCD_H; y += STRIP_ROWS) {
        const int rows = LCD_H - y < STRIP_ROWS ? LCD_H - y : STRIP_ROWS;
        uint16_t *strip = s_strip[s_strip_next];
        s_strip_next ^= 1u;
        if (ui) {
            /* Never a half-redrawn menu: retry while the rows change. */
            for (unsigned tries = 0; tries < 8u; ++tries) {
                const uint32_t seq = menu_lcd_seq();
                if (seq & 1u) { vTaskDelay(1); continue; }
                fill_menu(strip, y, rows, ui);
                if (menu_lcd_seq() == seq) break;
            }
        } else fill_image(strip, y, rows);
        (void)esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + rows, strip);
    }
    ++s_stats.flushes;
}

static bool panel_init(void)
{
    const esp_lcd_panel_io_spi_config_t io = {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = PIN_CS,
        .pclk_hz = CONFIG_C5VRX_LCD_SPI_MHZ * 1000000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4,
    };
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io, &s_io) != ESP_OK) return false;
    const esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_st7789(s_io, &dev, &s_panel) != ESP_OK) return false;
    /* Landscape as Waveshare's examples (rotation 3: row/column exchange,
     * rows mirrored); IPS panel, so the colours are inverted. */
#ifdef CONFIG_C5VRX_LCD_FLIP
    const bool flip = true;
#else
    const bool flip = false;
#endif
    return esp_lcd_panel_reset(s_panel) == ESP_OK && esp_lcd_panel_init(s_panel) == ESP_OK &&
           esp_lcd_panel_invert_color(s_panel, true) == ESP_OK && esp_lcd_panel_swap_xy(s_panel, true) == ESP_OK &&
           esp_lcd_panel_mirror(s_panel, flip, !flip) == ESP_OK && esp_lcd_panel_set_gap(s_panel, 0, LCD_GAP_Y) == ESP_OK &&
           esp_lcd_panel_disp_on_off(s_panel, true) == ESP_OK;
}

static void flush_task(void *arg)
{
    (void)arg;
    if (!panel_init()) {
        printf("LCD panel init failed\n");
        vTaskDelete(NULL);
    }
    memset(s_img, 0, sizeof(s_img));
    flush(NULL);                                  /* black before the backlight */
    gpio_set_level(PIN_BACKLIGHT, 1);
    for (;;) {
        /* A new frame wakes this task; the menu redraws at 20 Hz. */
        const bool woken = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)) != 0;
        const uint8_t (*ui)[MENU_UI_BYTES] = menu_lcd_rows();
        if (s_quiet) continue;
        if (ui) flush(ui);
        else if (woken && s_picture) flush(NULL);
    }
}

/* ---- the grey frame ---- */

/* Text in the menu's 5-row font, each font pixel sx x sy panel pixels (the
 * font is drawn for the composite menu's tall pixels). */
static int text_width(const char *s, int sx)
{
    int w = 0;
    for (const char *p = s; *p; ++p) {
        char c = *p >= 'a' && *p <= 'z' ? (char)(*p - 'a' + 'A') : *p;
        w += (s_menu_text_font[c >= 32 && c <= 95 ? c - 32 : '?' - 32].width + 2) * sx;
    }
    return w > 0 ? w - 2 * sx : 0;
}

static void draw_text(const char *s, int y, int sx, int sy, uint8_t level)
{
    int x = (GRAB_W - text_width(s, sx)) / 2;
    for (const char *p = s; *p; ++p) {
        char c = *p >= 'a' && *p <= 'z' ? (char)(*p - 'a' + 'A') : *p;
        const menu_glyph5_t *g = &s_menu_text_font[c >= 32 && c <= 95 ? c - 32 : '?' - 32];
        for (int r = 0; r < (int)MENU_TEXT_ROWS; ++r)
            for (int b = 0; b < g->width; ++b) {
                if (!(g->rows[r] >> b & 1u)) continue;
                for (int yy = y + r * sy; yy < y + (r + 1) * sy && yy < LCD_H; ++yy)
                    for (int xx = x + b * sx; xx < x + (b + 1) * sx; ++xx)
                        if (xx >= 0 && xx < GRAB_W && yy >= 0) s_img[yy][xx] = level;
            }
        x += (g->width + 2) * sx;
    }
}

static void draw_no_video(void)
{
    char line[32];
    const fpv_channel_t *ch = rf_channel();
    memset(s_img, 0, sizeof(s_img));
    draw_text("NO VIDEO", 44, 2, 8, 255);
    snprintf(line, sizeof(line), "%s  %u MHZ", ch->name, ch->freq_mhz);
    draw_text(line, 104, 1, 4, 150);
    s_level_lo = 0;
    s_level_gain = 256;
}

static void follow_levels(void)
{
    uint16_t hist[256] = {0};
    unsigned n = 0;
    for (int y = 0; y < GRAB_H; y += 2)
        for (int x = 0; x < GRAB_W; x += 4, ++n) ++hist[s_img[y][x]];
    unsigned lo = 0, hi = 255, sum = 0;
    while (lo < 255 && (sum += hist[lo]) < n / 100u) ++lo;
    sum = 0;
    while (hi > 0 && (sum += hist[hi]) < n / 200u) --hi;
    if (hi <= lo + 16u) return;                   /* flat picture: keep the levels */
    int gain = (int)(255u * 256u / (hi - lo));
    if (gain > 410) gain = 410;                   /* 1.6x */
    if (gain < 256) gain = 256;
    s_level_lo += ((int)lo - s_level_lo) / 8;
    s_level_gain += (gain - s_level_gain) / 8;
}

/* The grabber also rests while the button is held: the control task, below
 * it in this build, counts its 50 ms ticks to time a press (short = next,
 * 0.6 s = menu) and must run at its full rate meanwhile. */
static bool grab_abort(void) { return s_menu_active || s_quiet || gpio_get_level(PIN_BOOT) == 0; }

/* A row missed for more than ROW_KEEP_PICTURES pictures (its lines left the
 * ring while the receiver tasks held the CPU, several grabs running) combs
 * against a moving picture: blend it from the nearest fresh rows above and
 * below. A row missed once keeps the previous picture's row. */
static void refresh_stale_rows(void)
{
    int above = -1;
    for (int y = 0; y < GRAB_H; ++y) {
        if (s_row_age[y] == 0) { above = y; continue; }
        if (s_row_age[y] <= ROW_KEEP_PICTURES) continue;
        int below = y + 1;
        while (below < GRAB_H && s_row_age[below] != 0) ++below;
        /* Across a wide band (a long stall) a blend smears: keep the row. */
        if (above < 0 || below >= GRAB_H || below - above > ROW_BLEND_GAP) continue;
        const int wa = below - y, wb = y - above, w = wa + wb;   /* the nearer row weighs more */
        for (int x = 0; x < GRAB_W; ++x)
            s_img[y][x] = (uint8_t)((s_img[above][x] * wa + s_img[below][x] * wb + w / 2) / w);
    }
}

static void grab_task(void *arg)
{
    (void)arg;
    grab_init();
    grab_set_abort(grab_abort);
    uint32_t epoch = s_control_epoch;
    int64_t video_at = esp_timer_get_time(), no_video_at = 0;   /* last captured row, last NO VIDEO draw */
    for (;;) {
        if (grab_abort() || !transport_live()) {
            video_at = esp_timer_get_time();   /* the hold starts again after the menu or a restart */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (epoch != s_control_epoch) {   /* retune, scan, calibration: the old timing is gone */
            epoch = s_control_epoch;
            grab_unlock();
        }
        grab_info_t info;
        const int64_t grab_t0 = esp_timer_get_time();
        const int rows = grab_frame(&s_img[0][0], &info, GRAB_TIMEOUT_MS);
        if (info.max_stall_us >= 3000) {
            if (s_last_gain_write_us >= grab_t0) ++s_stats.stalls_3ms_write;
            else ++s_stats.stalls_3ms_nowrite;
        }
        ++s_stats.grabs;
        s_stats.rows += (uint32_t)rows;
        s_stats.late += (uint32_t)info.late;
        s_stats.substituted += (uint32_t)info.substituted;
        s_stats.nosync += (uint32_t)info.nosync;
        s_stats.wide += (uint32_t)info.wide_hits;
        s_stats.vslips += (uint32_t)info.vslips;
        s_stats.vmisses += (uint32_t)info.vmisses;
        s_stats.cpu_us += (uint32_t)info.cpu_us;
        s_stats.grab_ms = info.grab_ms;
        s_stats.acquire_ms = info.acquire_ms;
        if (info.max_stall_us > s_stats.max_stall_us) s_stats.max_stall_us = info.max_stall_us;
        s_stats.stalls_1ms += info.max_stall_us >= 1000;
        s_stats.stalls_5ms += info.max_stall_us >= 5000;
        s_stats.stalls_20ms += info.max_stall_us >= 20000;
        s_stats.pal = info.pal;
        s_stats.tb_bad = info.tb_bad;
        s_stats.line_us = info.line_us;
        s_stats.jitter_ns = info.jitter_ns;
        s_stats.carrier_mhz = info.carrier_mhz;
        s_stats.error = info.error;
        if (s_menu_active) continue;
        for (int y = 0; y < GRAB_H; ++y)
            s_row_age[y] = grab_row_done(y) ? 0u : s_row_age[y] < 255u ? (uint8_t)(s_row_age[y] + 1u) : 255u;
        if (rows > 0) {
            if (!info.error) ++s_stats.complete;
            video_at = esp_timer_get_time();
            refresh_stale_rows();
            if (rows >= GRAB_H / 2) follow_levels();
            s_picture = true;
            xTaskNotifyGive(s_flush_task);
        } else if (esp_timer_get_time() - video_at > NO_VIDEO_HOLD_US && esp_timer_get_time() - no_video_at > 500000) {
            /* Short dropouts keep the last picture on the panel. */
            no_video_at = esp_timer_get_time();   /* redrawn twice a second: the channel may change */
            draw_no_video();
            s_picture = true;
            xTaskNotifyGive(s_flush_task);
        }
        vTaskDelay(pdMS_TO_TICKS(GRAB_REST_MS));   /* control, calibration, flight log and IDLE run now */
    }
}

/* ---- start and status ---- */

void lcd_start(void)
{
    for (unsigned k = 0; k < 64u; ++k) {
        const unsigned v = k <= DAC_IDLE_CODE ? 0u : k >= 60u ? 255u : (k - DAC_IDLE_CODE) * 255u / (60u - DAC_IDLE_CODE);
        s_menu_lut[k] = grey565(v);
    }
    for (unsigned x = 0; x < LCD_W; ++x) s_menu_x[x] = (uint16_t)(x * MENU_UI_WIDTH / LCD_W);
    const gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_BACKLIGHT) | (1ULL << PIN_RGB_LED),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(PIN_BACKLIGHT, 0);
    gpio_set_level(PIN_RGB_LED, 0);   /* a floating WS2812 input latches noise */
    const spi_bus_config_t bus = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * STRIP_ROWS * (int)sizeof(uint16_t),
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    for (unsigned k = 0; k < 2u && err == ESP_OK; ++k) {
        s_strip[k] = heap_caps_malloc(LCD_W * STRIP_ROWS * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_strip[k]) err = ESP_ERR_NO_MEM;
    }
    if (err == ESP_OK && xTaskCreate(flush_task, "lcd_flush", 2560, NULL, 3, &s_flush_task) != pdPASS) err = ESP_ERR_NO_MEM;
    if (err == ESP_OK && xTaskCreate(grab_task, "lcd_grab", 3072, NULL, 2, &s_grab_task) != pdPASS) err = ESP_ERR_NO_MEM;
    if (err != ESP_OK) printf("LCD unavailable: %s (USB console remains available)\n", esp_err_to_name(err));
    else printf("LCD started heap_internal=%u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    s_stats.t0 = esp_timer_get_time();
}

/* console V: the shown grey frame for tools (one line per row: its age in
 * pictures, then the pixels in hex), between LCDFRAME and LCDFRAME_END. */
void lcd_dump(void)
{
    static char line[2 * GRAB_W + 16];
    printf("LCDFRAME w=%d h=%d levels=%d/%d\n", GRAB_W, GRAB_H, s_level_lo, s_level_gain);
    for (int y = 0; y < GRAB_H; ++y) {
        int n = snprintf(line, sizeof(line), "%02x ", s_row_age[y]);
        for (int x = 0; x < GRAB_W; ++x) {
            static const char hex[] = "0123456789abcdef";
            line[n++] = hex[s_img[y][x] >> 4];
            line[n++] = hex[s_img[y][x] & 15];
        }
        line[n] = 0;
        printf("%s\n", line);
        vTaskDelay(2);   /* stdout does not block: let the USB FIFO drain */
    }
    printf("LCDFRAME_END\n");
}

/* console T: grabber and display since the last T. */
void lcd_print(void)
{
    const int64_t now = esp_timer_get_time();
    const double s = (double)(now - s_stats.t0) / 1e6;
    printf("LCD grabs=%lu complete=%lu fps=%.1f flushes=%lu rows=%lu late=%lu sub=%lu nosync=%lu wide=%lu "
           "vslips=%lu vmisses=%lu cpu_us_per_row=%lu grab_ms=%d acquire_ms=%d std=%s line_us=%.3f jitter_ns=%.0f "
           "carrier_mhz=%.2f tb_bad=%u levels=%d/%.2f error=%s stack_free=%u/%u max_stall_us=%d stalls>=1/5/20ms=%lu/%lu/%lu\n",
           (unsigned long)s_stats.grabs, (unsigned long)s_stats.complete, s > 0 ? s_stats.complete / s : 0.0,
           (unsigned long)s_stats.flushes, (unsigned long)s_stats.rows, (unsigned long)s_stats.late,
           (unsigned long)s_stats.substituted, (unsigned long)s_stats.nosync, (unsigned long)s_stats.wide,
           (unsigned long)s_stats.vslips, (unsigned long)s_stats.vmisses,
           (unsigned long)(s_stats.rows ? s_stats.cpu_us / s_stats.rows : 0u), s_stats.grab_ms, s_stats.acquire_ms,
           s_stats.pal ? "PAL" : "NTSC", s_stats.line_us, s_stats.jitter_ns, s_stats.carrier_mhz, s_stats.tb_bad,
           s_level_lo, s_level_gain / 256.0, s_stats.error ? s_stats.error : "none",
           s_grab_task ? (unsigned)uxTaskGetStackHighWaterMark(s_grab_task) : 0u,
           s_flush_task ? (unsigned)uxTaskGetStackHighWaterMark(s_flush_task) : 0u, s_stats.max_stall_us,
           (unsigned long)s_stats.stalls_1ms, (unsigned long)s_stats.stalls_5ms, (unsigned long)s_stats.stalls_20ms);
    uint32_t writes, write_sum_us, write_max_us;
    receiver_write_time(&writes, &write_sum_us, &write_max_us);
    printf("LCD_GAINW writes=%lu write_us_mean=%lu write_us_max=%lu stalls>=3ms_with_write=%lu without=%lu\n",
           (unsigned long)writes, (unsigned long)(writes ? write_sum_us / writes : 0u), (unsigned long)write_max_us,
           (unsigned long)s_stats.stalls_3ms_write, (unsigned long)s_stats.stalls_3ms_nowrite);
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.t0 = now;
#if configGENERATE_RUN_TIME_STATS
    /* CPU share per task since the previous T: the grabber gets what the
     * receiver tasks leave. Matched by task number across calls. */
    static TaskStatus_t tasks[24];
    static struct { UBaseType_t number; configRUN_TIME_COUNTER_TYPE time; } prev[24];
    static configRUN_TIME_COUNTER_TYPE prev_total;
    configRUN_TIME_COUNTER_TYPE total;
    const UBaseType_t n = uxTaskGetSystemState(tasks, 24, &total);
    const configRUN_TIME_COUNTER_TYPE span = total - prev_total;
    printf("LCD_CPU span_ms=%lu", (unsigned long)(span / 1000u));
    for (UBaseType_t k = 0; k < n; ++k) {
        configRUN_TIME_COUNTER_TYPE before = 0;
        for (unsigned j = 0; j < 24u; ++j)
            if (prev[j].number == tasks[k].xTaskNumber) { before = prev[j].time; break; }
        const uint64_t used = (uint64_t)(configRUN_TIME_COUNTER_TYPE)(tasks[k].ulRunTimeCounter - before);
        const unsigned permille = span ? (unsigned)(used * 1000u / span) : 0u;
        if (permille >= 5u) printf(" %s=%u.%u%%", tasks[k].pcTaskName, permille / 10u, permille % 10u);   /* 0.5 % and above */
    }
    printf("\n");
    for (UBaseType_t k = 0; k < n && k < 24u; ++k) { prev[k].number = tasks[k].xTaskNumber; prev[k].time = tasks[k].ulRunTimeCounter; }
    prev_total = total;
#endif
}
