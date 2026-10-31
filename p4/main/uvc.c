/* C5VRX P4 display by Twotoz and the C5VRX contributors: UVC webcam on the
 * P4's high-speed USB port (espressif/usb_device_uvc, MJPEG).
 *
 * The render task's PPA scales the picture into one of three 800x600 RGB565
 * buffers; an encoder task turns each into a JPEG at once (hardware
 * encoder, three JPEG buffers), and the UVC task, at its own pace
 * (sdkconfig rate), sends the newest JPEG. Without a new picture the last
 * JPEG goes again, so the host always sees a steady stream.
 *
 * Encoding inside fb_get (with a wait for a new picture) made it serial with
 * the transfer (~23 ms for 90 KB on 512-byte isochronous packets): 18 fps
 * (host measured, 2026-10-31). */
#include "uvc.h"
#include <string.h>
#include "board.h"
#include "driver/jpeg_encode.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_device_uvc.h"

static const char *TAG = "uvc";

#define UVC_W           VIDEO_W
#define UVC_H           VIDEO_H
#define JPEG_MAX        (512 * 1024)
#define JPEG_QUALITY    80

static jpeg_encoder_handle_t s_jpeg;
static uint8_t *s_y[3];
static int s_fill = -1, s_ready = -1, s_enc = -1;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t *s_jpeg_buf[3];
static size_t s_jpeg_cap;
static uint32_t s_jpeg_len[3];
static int s_jlatest = -1, s_jbusy = -1;   /* newest JPEG; one the UVC task copies */
static TaskHandle_t s_enc_task;
static volatile uint32_t s_jpeg_us;
uint32_t uvc_jpeg_us(void) { return s_jpeg_us; }
static uvc_fb_t s_fb;
static volatile bool s_streaming;
static uint32_t s_sent, s_fps;
static int64_t s_fps_t0;

bool uvc_streaming(void) { return s_streaming; }
uint32_t uvc_fps(void) { return s_streaming ? s_fps : 0; }

uint8_t *uvc_frame_begin(void)
{
    if (!s_streaming) return NULL;
    portENTER_CRITICAL(&s_lock);
    if (s_fill >= 0) {   /* the last one still being scaled */
        portEXIT_CRITICAL(&s_lock);
        return NULL;
    }
    for (int k = 0; k < 3; ++k)
        if (k != s_ready && k != s_enc) { s_fill = k; break; }
    portEXIT_CRITICAL(&s_lock);
    return s_fill >= 0 ? s_y[s_fill] : NULL;
}

void uvc_frame_abort(void)
{
    portENTER_CRITICAL(&s_lock);
    s_fill = -1;
    portEXIT_CRITICAL(&s_lock);
}

bool uvc_frame_end_isr(void)
{
    portENTER_CRITICAL_ISR(&s_lock);
    s_ready = s_fill;
    s_fill = -1;
    portEXIT_CRITICAL_ISR(&s_lock);
    BaseType_t woken = pdFALSE;
    if (s_enc_task) vTaskNotifyGiveFromISR(s_enc_task, &woken);
    return woken == pdTRUE;
}

static bool encode(const uint8_t *y, int j)
{
    const jpeg_encode_cfg_t cfg = {
        .width = UVC_W,
        .height = UVC_H,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = JPEG_QUALITY,
    };
    uint32_t len = 0;   /* the PPA wrote y by DMA: nothing in the cache to write back */
    const esp_err_t err = jpeg_encoder_process(s_jpeg, &cfg, y, UVC_FRAME_BYTES, s_jpeg_buf[j], s_jpeg_cap, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "encode: %s", esp_err_to_name(err));
        return false;
    }
    s_jpeg_len[j] = len;
    return true;
}

static void enc_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int j = -1;
        portENTER_CRITICAL(&s_lock);
        if (s_ready >= 0) { s_enc = s_ready; s_ready = -1; }
        for (int k = 0; k < 3; ++k)
            if (k != s_jlatest && k != s_jbusy) { j = k; break; }
        portEXIT_CRITICAL(&s_lock);
        if (s_enc < 0) continue;
        const int64_t t0 = esp_timer_get_time();
        const bool ok = encode(s_y[s_enc], j);
        s_jpeg_us = (uint32_t)(esp_timer_get_time() - t0);
        portENTER_CRITICAL(&s_lock);
        s_enc = -1;
        if (ok) s_jlatest = j;
        portEXIT_CRITICAL(&s_lock);
    }
}

static uvc_fb_t *fb_get(void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    const int j = s_jlatest;
    s_jbusy = j;
    portEXIT_CRITICAL(&s_lock);
    const int64_t now = esp_timer_get_time();
    s_fb.buf = s_jpeg_buf[j];
    s_fb.len = s_jpeg_len[j];
    s_fb.width = UVC_W;
    s_fb.height = UVC_H;
    s_fb.format = UVC_FORMAT_JPEG;
    s_fb.timestamp.tv_sec = now / 1000000;
    s_fb.timestamp.tv_usec = now % 1000000;
    ++s_sent;
    if (now - s_fps_t0 >= 1000000) { s_fps = s_sent; s_sent = 0; s_fps_t0 = now; }
    return &s_fb;
}

static void fb_return(uvc_fb_t *fb, void *ctx)
{
    (void)fb;
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_jbusy = -1;
    portEXIT_CRITICAL(&s_lock);
}

static esp_err_t on_start(uvc_format_t format, int width, int height, int rate, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "host streams %dx%d at %d fps", width, height, rate);
    if (format != UVC_FORMAT_JPEG || width != UVC_W || height != UVC_H) return ESP_ERR_NOT_SUPPORTED;
    s_streaming = true;
    return ESP_OK;
}

static void on_stop(void *ctx)
{
    (void)ctx;
    s_streaming = false;
    ESP_LOGI(TAG, "host stopped streaming");
}

esp_err_t uvc_start(void)
{
    const jpeg_encode_engine_cfg_t ecfg = {.timeout_ms = 100};
    esp_err_t err = jpeg_new_encoder_engine(&ecfg, &s_jpeg);
    if (err != ESP_OK) return err;
    const jpeg_encode_memory_alloc_cfg_t mcfg = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    for (int k = 0; k < 3; ++k) s_jpeg_buf[k] = jpeg_alloc_encoder_mem(JPEG_MAX, &mcfg, &s_jpeg_cap);
    for (int k = 0; k < 3; ++k) s_y[k] = heap_caps_aligned_calloc(128, 1, UVC_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    uint8_t *xfer = heap_caps_malloc(JPEG_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_jpeg_buf[0] || !s_jpeg_buf[1] || !s_jpeg_buf[2] || !s_y[0] || !s_y[1] || !s_y[2] || !xfer) return ESP_ERR_NO_MEM;
    /* A black picture until the first real one (written back: the encoder
     * reads memory). */
    (void)esp_cache_msync(s_y[0], UVC_FRAME_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    if (!encode(s_y[0], 0)) return ESP_FAIL;
    s_jlatest = 0;
    if (xTaskCreate(enc_task, "uvc_enc", 3072, NULL, 4, &s_enc_task) != pdPASS) return ESP_ERR_NO_MEM;
    uvc_device_config_t cfg = {
        .uvc_buffer = xfer,
        .uvc_buffer_size = JPEG_MAX,
        .start_cb = on_start,
        .fb_get_cb = fb_get,
        .fb_return_cb = fb_return,
        .stop_cb = on_stop,
        .cb_ctx = NULL,
    };
    err = uvc_device_config(0, &cfg);
    if (err == ESP_OK) err = uvc_device_init();
    if (err == ESP_OK) ESP_LOGI(TAG, "UVC %dx%d MJPEG ready", UVC_W, UVC_H);
    return err;
}
