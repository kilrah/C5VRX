/* C5VRX P4 display by Twotoz and the C5VRX contributors: the touch settings
 * and status column right of the picture, and the picture's overlays. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t ui_start(void);
/* The column in frame buffer fb_index is out of date. */
bool ui_dirty(int fb_index);
/* Overlays on the picture and, if out of date, the column. */
void ui_draw(uint16_t *fb, int fb_index);
