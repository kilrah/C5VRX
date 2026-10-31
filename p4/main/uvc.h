/* C5VRX P4 display by Twotoz and the C5VRX contributors: the picture as a
 * USB webcam (UVC, MJPEG 800x600, high-speed port). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t uvc_start(void);
bool uvc_streaming(void);
#define UVC_FRAME_BYTES (800 * 600 * 2)
/* An 800x600 RGB565 buffer to fill while a host streams, else NULL (also
 * while the last one is still being filled); the render task's PPA fills it
 * with the shown picture, and its done interrupt calls uvc_frame_end_isr()
 * (true: a higher-priority task woke). uvc_frame_abort() if the fill failed. */
uint8_t *uvc_frame_begin(void);
bool uvc_frame_end_isr(void);
void uvc_frame_abort(void);
uint32_t uvc_fps(void);
uint32_t uvc_jpeg_us(void);   /* the last encode */
