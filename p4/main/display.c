/* C5VRX P4 display by Twotoz and the C5VRX contributors: panel and render
 * task (docs/P4LINK.md).
 *
 * The JD9165 init sequence, DSI bus, DPI timing, MIPI LDO and backlight
 * settings are the Guition JC1060P470C vendor example's
 * (video_lcd_display/main/app_lcd.c and main.c, Espressif example code,
 * CC0-1.0), unchanged.
 *
 * Two DPI frame buffers in PSRAM: the PPA scales the decoder's RGB565 frame
 * into the hidden one (and into the UVC frame when a host streams), the CPU
 * adds the overlays and the settings column, the panel gets it, and the task
 * waits for the panel's refresh to finish before touching the other one, so
 * the picture never tears. The CPU conversion this replaces took 77-98 ms a
 * picture (11 per second, board 2026-10-31). */
#include "display.h"
#include <string.h>
#include "board.h"
#include "cvbs.h"
#include "driver/ledc.h"
#include "driver/ppa.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "esp_check.h"
#include "esp_lcd_jd9165.h"
#include "hal/axi_icm_ll.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ui.h"
#include "uvc.h"

static const char *TAG = "display";

static const jd9165_lcd_init_cmd_t s_init_cmds[] = {
    {0x30, (uint8_t[]){0x00}, 1, 0},
    {0xF7, (uint8_t[]){0x49, 0x61, 0x02, 0x00}, 4, 0},
    {0x30, (uint8_t[]){0x01}, 1, 0},
    {0x04, (uint8_t[]){0x0C}, 1, 0},
    {0x05, (uint8_t[]){0x08}, 1, 0},
    {0x0B, (uint8_t[]){0x11}, 1, 0},
    {0x20, (uint8_t[]){0x04}, 1, 0},
    {0x1F, (uint8_t[]){0x05}, 1, 0},
    {0x23, (uint8_t[]){0x38}, 1, 0},
    {0x28, (uint8_t[]){0x18}, 1, 0},
    {0x29, (uint8_t[]){0x29}, 1, 0},
    {0x2A, (uint8_t[]){0x01}, 1, 0},
    {0x2B, (uint8_t[]){0x29}, 1, 0},
    {0x2C, (uint8_t[]){0x01}, 1, 0},
    {0x30, (uint8_t[]){0x02}, 1, 0},
    {0x00, (uint8_t[]){0x05}, 1, 0},
    {0x01, (uint8_t[]){0x22}, 1, 0},
    {0x02, (uint8_t[]){0x08}, 1, 0},
    {0x03, (uint8_t[]){0x12}, 1, 0},
    {0x04, (uint8_t[]){0x16}, 1, 0},
    {0x05, (uint8_t[]){0x64}, 1, 0},
    {0x06, (uint8_t[]){0x00}, 1, 0},
    {0x07, (uint8_t[]){0x00}, 1, 0},
    {0x08, (uint8_t[]){0x78}, 1, 0},
    {0x09, (uint8_t[]){0x00}, 1, 0},
    {0x0A, (uint8_t[]){0x04}, 1, 0},
    {0x0B, (uint8_t[]){0x16, 0x17, 0x0B, 0x0D, 0x0D, 0x0D, 0x11, 0x10, 0x07, 0x07, 0x09}, 11, 0},
    {0x0C, (uint8_t[]){0x09, 0x1E, 0x1E, 0x1C, 0x1C, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x0D, (uint8_t[]){0x0A, 0x05, 0x0B, 0x0D, 0x0D, 0x0D, 0x11, 0x10, 0x06, 0x06, 0x08}, 11, 0},
    {0x0E, (uint8_t[]){0x08, 0x1F, 0x1F, 0x1D, 0x1D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x0F, (uint8_t[]){0x0A, 0x05, 0x0D, 0x0B, 0x0D, 0x0D, 0x11, 0x10, 0x1D, 0x1D, 0x1F}, 11, 0},
    {0x10, (uint8_t[]){0x1F, 0x08, 0x08, 0x06, 0x06, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x11, (uint8_t[]){0x16, 0x17, 0x0D, 0x0B, 0x0D, 0x0D, 0x11, 0x10, 0x1C, 0x1C, 0x1E}, 11, 0},
    {0x12, (uint8_t[]){0x1E, 0x09, 0x09, 0x07, 0x07, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x13, (uint8_t[]){0x00, 0x00, 0x00, 0x00}, 4, 0},
    {0x14, (uint8_t[]){0x00, 0x00, 0x41, 0x41}, 4, 0},
    {0x15, (uint8_t[]){0x00, 0x00, 0x00, 0x00}, 4, 0},
    {0x17, (uint8_t[]){0x00}, 1, 0},
    {0x18, (uint8_t[]){0x85}, 1, 0},
    {0x19, (uint8_t[]){0x06, 0x09}, 2, 0},
    {0x1A, (uint8_t[]){0x05, 0x08}, 2, 0},
    {0x1B, (uint8_t[]){0x0A, 0x04}, 2, 0},
    {0x26, (uint8_t[]){0x00}, 1, 0},
    {0x27, (uint8_t[]){0x00}, 1, 0},
    {0x30, (uint8_t[]){0x06}, 1, 0},
    {0x12, (uint8_t[]){0x3F, 0x25, 0x27, 0x35, 0x1D, 0x1B, 0x1B, 0x1A, 0x18, 0x0A, 0x2A, 0x21, 0x19, 0x30}, 14, 0},
    {0x13, (uint8_t[]){0x3F, 0x26, 0x27, 0x35, 0x1E, 0x1C, 0x1C, 0x1A, 0x18, 0x0B, 0x2A, 0x21, 0x19, 0x30}, 14, 0},
    {0x30, (uint8_t[]){0x0A}, 1, 0},
    {0x02, (uint8_t[]){0x4F}, 1, 0},
    {0x0B, (uint8_t[]){0x40}, 1, 0},
    {0x30, (uint8_t[]){0x0D}, 1, 0},
    {0x0D, (uint8_t[]){0x04}, 1, 0},
    {0x10, (uint8_t[]){0x05}, 1, 0},
    {0x11, (uint8_t[]){0x0C}, 1, 0},
    {0x12, (uint8_t[]){0x05}, 1, 0},
    {0x13, (uint8_t[]){0x0C}, 1, 0},
    {0x30, (uint8_t[]){0x00}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},     /* RGB565 */
    {0x11, (uint8_t[]){0x00}, 1, 120},   /* sleep out */
    {0x29, (uint8_t[]){0x00}, 1, 20},    /* display on */
};

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_fb[2];
static SemaphoreHandle_t s_refresh;
static volatile uint32_t s_fps;
static ppa_client_handle_t s_ppa, s_ppa_uvc;
static uint32_t s_ppa_errors;

#define BL_LEDC_CH      LEDC_CHANNEL_0
#define BL_LEDC_TIMER   LEDC_TIMER_1

/* Panel frame timing, from the DMA's end-of-frame interrupt (which also
 * restarts the DMA): frames that ended more than 1 ms late, and the longest. */
static volatile uint32_t s_late_frames, s_max_frame_us;
static IRAM_ATTR bool on_refresh_done(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *ctx)
{
    static int64_t last;
    const int64_t now = esp_timer_get_time();
    if (last) {
        const uint32_t d = (uint32_t)(now - last);
        if (d > s_max_frame_us) s_max_frame_us = d;
        if (d > 16400u + 1000u) ++s_late_frames;
    }
    last = now;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_refresh, &woken);
    return woken == pdTRUE;
}

static esp_err_t panel_init(void)
{
    esp_ldo_channel_handle_t ldo = NULL;
    const esp_ldo_channel_config_t lcfg = {.chan_id = LCD_MIPI_LDO_CHAN, .voltage_mv = LCD_MIPI_LDO_MV};
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&lcfg, &ldo), TAG, "MIPI LDO");

    esp_lcd_dsi_bus_handle_t bus;
    const esp_lcd_dsi_bus_config_t bus_cfg = JD9165_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_cfg, &bus), TAG, "DSI bus");
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_dbi_io_config_t dbi_cfg = {.virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8};
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(bus, &dbi_cfg, &io), TAG, "DBI io");
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 52,
        .virtual_channel = 0,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 2,
        .video_timing = {
            .h_size = LCD_H_RES,
            .v_size = LCD_V_RES,
            .hsync_back_porch = 136,
            .hsync_pulse_width = 24,
            .hsync_front_porch = 160,
            .vsync_back_porch = 21,
            .vsync_pulse_width = 2,
            .vsync_front_porch = 12,
        },
        .flags.use_dma2d = true,
    };
    jd9165_vendor_config_t vendor = {
        .init_cmds = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .mipi_config = {.dsi_bus = bus, .dpi_config = &dpi_cfg},
    };
    const esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_jd9165(io, &dev_cfg, &s_panel), TAG, "JD9165");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    void *fb0, *fb1;
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &fb0, &fb1), TAG, "frame buffers");
    s_fb[0] = fb0;
    s_fb[1] = fb1;
    s_refresh = xSemaphoreCreateBinary();
    const esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_refresh_done = on_refresh_done};
    return esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL);
}

static void backlight_init(void)
{
    const ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t c = {
        .gpio_num = LCD_BL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BL_LEDC_CH,
        .timer_sel = BL_LEDC_TIMER,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

void display_set_backlight(int percent)
{
    if (percent < 5) percent = 5;
    if (percent > 100) percent = 100;
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CH, 1023u * (unsigned)percent / 100u);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CH);
}

uint32_t display_fps(void) { return s_fps; }
static volatile uint32_t s_draw_us;
static volatile uint32_t s_panel_us, s_uvc_ppa_us;   /* the last pass of each */
static int64_t s_uvc_t0;
uint32_t display_panel_ppa_us(void) { return s_panel_us; }
uint32_t display_uvc_ppa_us(void) { return s_uvc_ppa_us; }
uint32_t display_draw_us(void) { return s_draw_us; }
uint32_t display_ppa_errors(void) { return s_ppa_errors; }

uint32_t display_late_frames(uint32_t *max_us)
{
    *max_us = s_max_frame_us;
    s_max_frame_us = 0;
    return s_late_frames;
}

/* The PPA converts a frame (512 x 576/480 YUV420, BT.601 full range) to
 * RGB565 and scales it by exact 1/16 steps: 25/16 across (800 columns),
 * NTSC 20/16 down (600 rows), PAL 17/16 down from the middle 564 lines (599
 * rows; YUV420 needs even blocks, 6 lines of overscan go). */
static esp_err_t scale_frame(const cvbs_frame_t *f, void *out, uint32_t out_w, uint32_t out_h, size_t out_size,
                             ppa_client_handle_t client, ppa_trans_mode_t mode)
{
    /* 576 PAL, 480 NTSC; a progressive field 288 / 240: 34/16 from the
     * middle 282 rows, 40/16 (600 rows). */
    const int h = f->height;
    const uint32_t bh = h == 576 ? 564 : h == 288 ? 282 : (uint32_t)h, by = h == 576 ? 6 : h == 288 ? 2 : 0;
    const float sy = h == 576 ? 17.0f / 16.0f : h == 480 ? 20.0f / 16.0f : h == 288 ? 34.0f / 16.0f : 40.0f / 16.0f;
    const ppa_srm_oper_config_t c = {
        .in = {.buffer = f->yuv, .pic_w = CVBS_W, .pic_h = (uint32_t)h, .block_w = CVBS_W,
               .block_h = bh, .block_offset_x = 0, .block_offset_y = by,
               .srm_cm = PPA_SRM_COLOR_MODE_YUV420, .yuv_range = PPA_COLOR_RANGE_FULL,
               .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601},
        .out = {.buffer = out, .buffer_size = out_size, .pic_w = out_w, .pic_h = out_h,
                .block_offset_x = 0, .block_offset_y = 0, .srm_cm = PPA_SRM_COLOR_MODE_RGB565},
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 25.0f / 16.0f,
        .scale_y = sy,
        .mode = mode,
    };
    return ppa_do_scale_rotate_mirror(client, &c);
}

static size_t s_line = 64;   /* PSRAM cache line (L1/L2), from the heap's alignment */

/* Drop the CPU's cached copy of rows y0..y1-1, bytes x0..x1-1 of fb (pixels
 * x0/2..), widened to whole cache lines, one row at a time: the PPA wrote
 * them, and a stale line the CPU partly overwrites would be written back
 * over the picture. Never the whole buffer at once: an invalidate runs as
 * one critical section (IDF 5.5 chunks only write-backs), and 1.2 MB of it
 * delayed the panel DMA's end-of-frame interrupt past the blanking: the
 * panel showed a white frame now and then (board 2026-10-31). */
void display_invalidate(uint16_t *fb, int y0, int y1, int x0, int x1)
{
    for (int y = y0; y < y1; ++y) {
        const uintptr_t row = (uintptr_t)(fb + (size_t)y * LCD_H_RES);
        const uintptr_t a = (row + (uintptr_t)x0 * 2u) / s_line * s_line;
        const uintptr_t b = (row + (uintptr_t)x1 * 2u + s_line - 1u) / s_line * s_line;
        (void)esp_cache_msync((void *)a, b - a, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    }
}

/* The picture into the left 800 x 600 of frame buffer fb (black without a
 * frame yet), and into the UVC frame when a host streams. Where a cache line
 * spans the picture's last columns and the column's first (byte 1600 of a
 * row is not a line boundary for 128-byte lines), the CPU's copy goes: the
 * column's redraw would otherwise write stale picture bytes back. */
static bool uvc_ppa_done(ppa_client_handle_t c, ppa_event_data_t *e, void *user)
{
    (void)c;
    (void)e;
    (void)user;
    s_uvc_ppa_us = (uint32_t)(esp_timer_get_time() - s_uvc_t0);
    return uvc_frame_end_isr();
}

static void draw_video(int fb_index, const cvbs_frame_t *f)
{
    uint16_t *fb = s_fb[fb_index];
    const size_t fb_bytes = LCD_H_RES * LCD_V_RES * 2;
    if (!f) {
        for (int y = 0; y < VIDEO_H; ++y) memset(fb + (size_t)y * LCD_H_RES, 0, VIDEO_W * 2);
        (void)esp_cache_msync(fb, fb_bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        return;
    }
    const int64_t t0 = esp_timer_get_time();
    if (scale_frame(f, fb, LCD_H_RES, LCD_V_RES, fb_bytes, s_ppa, PPA_TRANS_MODE_BLOCKING) != ESP_OK) ++s_ppa_errors;
    s_panel_us = (uint32_t)(esp_timer_get_time() - t0);
    if ((VIDEO_W * 2) % s_line) display_invalidate(fb, 0, VIDEO_H, VIDEO_W - 1, VIDEO_W);
    /* The UVC picture: the panel's video area copied by the PPA's blend
     * engine, which runs beside the scale engine, while the CPU draws the
     * column (it ends, uvc_ppa_done, long before this buffer is drawn
     * again). A second scale pass for UVC took 18 ms on the same engine as
     * the panel's 16-23: ~40 ms a picture, the panel fell to 21 fps and the
     * picture flashed white (board 2026-10-31). The foreground is a
     * don't-care read (A4, alpha fixed at 0): the output is the background. */
    uint8_t *uvc = uvc_frame_begin();
    s_uvc_t0 = esp_timer_get_time();
    if (uvc) {
        const ppa_blend_oper_config_t b = {
            .in_bg = {.buffer = fb, .pic_w = LCD_H_RES, .pic_h = LCD_V_RES, .block_w = VIDEO_W, .block_h = VIDEO_H,
                      .blend_cm = PPA_BLEND_COLOR_MODE_RGB565},
            .in_fg = {.buffer = fb, .pic_w = VIDEO_W, .pic_h = VIDEO_H, .block_w = VIDEO_W, .block_h = VIDEO_H,
                      .blend_cm = PPA_BLEND_COLOR_MODE_A4},
            .out = {.buffer = uvc, .buffer_size = UVC_FRAME_BYTES, .pic_w = VIDEO_W, .pic_h = VIDEO_H,
                    .blend_cm = PPA_BLEND_COLOR_MODE_RGB565},
            .bg_alpha_update_mode = PPA_ALPHA_FIX_VALUE,
            .bg_alpha_fix_val = 255,
            .fg_alpha_update_mode = PPA_ALPHA_FIX_VALUE,
            .fg_alpha_fix_val = 0,
            .mode = PPA_TRANS_MODE_NON_BLOCKING,
        };
        if (ppa_do_blend(s_ppa_uvc, &b) != ESP_OK) {
            uvc_frame_abort();
            ++s_ppa_errors;
        }
    }
}

static void render_task(void *arg)
{
    (void)arg;
    cvbs_frame_t frame;
    bool have = false;
    int back = 1;
    uint32_t frames = 0;
    int64_t fps_t0 = esp_timer_get_time(), last_draw = 0;
    for (;;) {
        bool fresh = false;
        for (int k = 0; k < 8 && !fresh; ++k) {
            if (cvbs_acquire(&frame)) fresh = have = true;
            else vTaskDelay(pdMS_TO_TICKS(5));
        }
        const int64_t now = esp_timer_get_time();
        if (!fresh && now - last_draw < 100000 && !ui_dirty(back)) continue;
        last_draw = now;
        const int64_t d0 = esp_timer_get_time();
        draw_video(back, have ? &frame : NULL);
        ui_draw(s_fb[back], back);
        s_draw_us = (uint32_t)(esp_timer_get_time() - d0);
        (void)xSemaphoreTake(s_refresh, 0);
        (void)esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_H_RES, LCD_V_RES, s_fb[back]);
        /* The panel switches buffers at the end of its current refresh. */
        (void)xSemaphoreTake(s_refresh, pdMS_TO_TICKS(40));
        back ^= 1;
        ++frames;
        if (now - fps_t0 >= 1000000) {
            s_fps = frames;
            frames = 0;
            fps_t0 = now;
        }
    }
}

esp_err_t display_start(void)
{
    backlight_init();
    ESP_RETURN_ON_ERROR(panel_init(), TAG, "panel");
    (void)esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_line);
    if (s_line < 4) s_line = 64;
    /* The panel's scanout (DW-GDMA) first on the AXI bus (all masters default
     * to priority 0). Alone it did not stop the white frames; spreading the
     * decoder's frame write-back did (cvbs.c). Rate-limiting the PPA's DMA2D
     * or 32-byte PPA bursts (58 late frames in 90 s) did not help. */
#ifndef P4_AXI_QOS
#define P4_AXI_QOS 1
#endif
#ifndef P4_PPA_BURST
#define P4_PPA_BURST PPA_DATA_BURST_LENGTH_128
#endif
    if (P4_AXI_QOS) {
        axi_icm_ll_set_dw_gdma_qos_arbiter_prio(0, 15, 15);
        axi_icm_ll_set_dw_gdma_qos_arbiter_prio(1, 15, 15);
    }
    const ppa_client_config_t pcfg = {.oper_type = PPA_OPERATION_SRM, .data_burst_length = P4_PPA_BURST};
    ESP_RETURN_ON_ERROR(ppa_register_client(&pcfg, &s_ppa), TAG, "PPA");
    const ppa_client_config_t ucfg = {.oper_type = PPA_OPERATION_BLEND, .max_pending_trans_num = 1,
                                      .data_burst_length = P4_PPA_BURST};
    ESP_RETURN_ON_ERROR(ppa_register_client(&ucfg, &s_ppa_uvc), TAG, "PPA UVC");
    const ppa_event_callbacks_t ucbs = {.on_trans_done = uvc_ppa_done};
    ESP_RETURN_ON_ERROR(ppa_client_register_event_callbacks(s_ppa_uvc, &ucbs), TAG, "PPA UVC cb");
    for (int k = 0; k < 2; ++k) {
        memset(s_fb[k], 0, LCD_H_RES * LCD_V_RES * 2);
        (void)esp_cache_msync(s_fb[k], LCD_H_RES * LCD_V_RES * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
    return xTaskCreatePinnedToCore(render_task, "render", 6144, NULL, 5, NULL, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
