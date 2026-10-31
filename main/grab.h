/* C5VRX-5 by Twotoz and the C5VRX contributors: composite frame grabber on
 * the raw IQ ring, for the LCD build (lcd.c).
 *
 * Adapted from KonradIT's UART video link (branch tembed-link, commit
 * d7bc24d, experiments/video-link/uart/components/video_link_uart/grab.c),
 * which streamed 224 x 168 frames from a XIAO ESP32-C5 to a LilyGo T-Embed.
 * Changes here: 228 x 172 (4:3) for the Waveshare panel,
 * the C5VRX-5 ring accessors, no gain statistics (Direct Gain owns the gain)
 * and no host-simulator cost hooks.
 *
 * FM-demodulates the 40 MS/s Q4/I4 ring in software (phase step between
 * samples from a 256-entry angle table), locks onto the horizontal and
 * vertical sync of the received PAL or NTSC signal with a flywheel on both,
 * and picks GRAB_H interlaced lines of GRAB_W luma pixels out of the active
 * picture: half of the rows from each of two consecutive fields. It only
 * reads the ring; the CPU never paces IQ. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define GRAB_W 228   /* 4:3 at the panel's 172 lines */
#define GRAB_H 172

typedef struct {
    int      rows;          /* rows captured (GRAB_H when complete) */
    int      fields;        /* fields visited */
    bool     pal;           /* standard detected from the line period */
    float    line_us;       /* tracked line period */
    int      acquire_ms;    /* time spent finding the vertical interval (0 when already locked) */
    int      grab_ms;       /* total time of the call */
    int      late;          /* rows whose samples were overwritten before use */
    int      substituted;   /* rows taken from the next line of the field because theirs had gone */
    int      given_up;      /* rows left out after one failed retry each (the display keeps the old row) */
    int      nosync;        /* rows without a horizontal sync where predicted */
    float    jitter_ns;     /* RMS difference between predicted and measured sync edges */
    int      cpu_us;        /* processing time of the captured rows, from the line being in the ring */
    float    carrier_mhz;   /* centre of the video swing relative to the tuned frequency */
    int      wide_hits;     /* rows found only by the fallback search over a whole line */
    bool     tb_bad;        /* the timebase disagreed with the GDMA: descriptor-granular ring checks */
    int      vchecks;       /* fields whose vertical interval was checked */
    int      vslips;        /* ... and found off the prediction (grid corrected) */
    int      vmisses;       /* ... and not found at all */
    int      max_stall_us;  /* longest time the grab did not run while waiting for a line (preempted) */
    const char *error;      /* NULL on success */
} grab_info_t;

/** Build the tables. Call once. */
void grab_init(void);

/** Between grabs: re-estimate the I/Q DC offset from the ring and rebuild
 * the angle table around it when it moved (about 1 ms when it does). */
void grab_track_dc(void);
void grab_dc(float *i, float *q);   /* the estimate, cells */
bool grab_dc_toggle(void);          /* bench: compensation on/off, returns on */

/** Forget the lock (after a channel change, for example). */
void grab_unlock(void);

/** Checked while the grab waits for a line: true ends the grab at once
 * (the menu takes the display). */
void grab_set_abort(bool (*abort_fn)(void));

/**
 * Grab one frame into img (GRAB_H rows of GRAB_W bytes, 0 = blanking level,
 * 255 = nominal white). Starts with whatever field is on air, so a complete
 * frame takes two fields. Busy-waits on the ring: call from a task below
 * every receiver task. Returns the number of rows captured; rows not
 * captured are left untouched.
 */
int grab_frame(uint8_t *img, grab_info_t *info, int timeout_ms);

/** After grab_frame: was row y captured by it? */
bool grab_row_done(int y);
