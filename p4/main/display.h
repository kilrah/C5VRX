/* C5VRX P4 display by Twotoz and the C5VRX contributors: the JD9165 panel,
 * its backlight and the render task (picture left, settings column right). */
#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t display_start(void);
void display_set_backlight(int percent);
/* Frames rendered to the panel per second (last second). */
uint32_t display_fps(void);
uint32_t display_panel_ppa_us(void);   /* the last panel / UVC PPA pass */
uint32_t display_uvc_ppa_us(void);
uint32_t display_draw_us(void);   /* the last picture's draw time */
uint32_t display_ppa_errors(void);
/* Panel frames that ended > 1 ms late (since boot), and the longest frame
 * since the last call (us). */
uint32_t display_late_frames(uint32_t *max_us);
/* Drop the CPU's cached copy of a picture area (rows y0..y1-1, pixels
 * x0..x1-1) before drawing over it: the PPA wrote it. */
void display_invalidate(uint16_t *fb, int y0, int y1, int x0, int x1);
