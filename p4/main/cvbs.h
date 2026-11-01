/* C5VRX P4 display by Twotoz and the C5VRX contributors: capture of the
 * C5's six-bit DAC codes (40 MHz bus, unique codes at 20 MS/s; from raw IQ
 * through the P4's own BitScrambler decoder when the C5 sends RAW) and a
 * composite decoder (PAL and NTSC colour) into woven YUV420 frames at
 * source resolution (512 pixels of the active line); the PPA converts and
 * scales them. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define CVBS_W          512     /* pixels per frame row: 51.2 us of the active line */
#define CVBS_ROW_BYTES  (CVBS_W * 3 / 2)   /* YUV420 as the P4's PPA reads it */
/* Packing per pixel pair, 3 bytes: the row's chroma (Cb on even rows, Cr on
 * odd rows) and two Y. */
#define YUV_C           0
#define YUV_Y0          1
#define YUV_Y1          2
#define CVBS_H_MAX      576

typedef struct {
    const uint8_t *yuv;     /* CVBS_W x height, YUV420, written back from the cache */
    int height;             /* 576 PAL, 480 NTSC */
    bool pal;
    uint32_t seq;
} cvbs_frame_t;

typedef struct {
    bool locked;            /* line clock on hsyncs */
    bool pal;
    float line_us;
    uint32_t lines_ps, hsync_ps, fields_ps, frames_ps;
    uint32_t overruns, restarts;
    float sync_level, blank_level;   /* six-bit code units */
    bool clock;             /* bus data arriving */
    bool raw_iq;            /* the bus carries raw IQ, the P4 demodulates */
} cvbs_stats_t;

esp_err_t cvbs_start(void);
/* The newest complete frame, held until cvbs_release(); false if none
 * arrived since the last acquire. */
bool cvbs_acquire(cvbs_frame_t *f);
void cvbs_release(void);
void cvbs_get_stats(cvbs_stats_t *s);   /* rates over the last second */

/* Picture settings (ui.c, kept in NVS there). */
void cvbs_set_picture(int contrast_pct, int brightness);
void cvbs_set_field_swap(bool swap);
void cvbs_set_saturation(int pct);   /* 0 = monochrome, 100 = nominal */
/* Sample on the bus clock's falling edge instead of the rising one (the C5
 * changes data on the falling edge); restarts the capture. */
void cvbs_set_sample_edge_neg(bool neg);
/* Bench: one CVBS line on the console (main.c every 2 s). */
void cvbs_debug_print(void);
