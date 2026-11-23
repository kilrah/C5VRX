/* C5VRX-5 by Twotoz and the C5VRX contributors: menu picture.
 *
 * Draws the menu and status screens into the menu raster's UI rows from a
 * plain snapshot of receiver state, so the layout builds and is tested on
 * the host (test/test_menu_ui.c) exactly as the firmware draws it.
 *
 * Black and white: black (blanking, code 20) paper, white (60) ink, two
 * greys for hierarchy. The 384 x 56 grid has pixels about six times
 * taller than wide, so strokes are three pixels wide per row of height
 * and the fonts (menu_font.h) are drawn for that aspect. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "menu_raster.h"
#include "experimental.h"

enum { MENU_BAND, MENU_CHANNEL, MENU_SCAN, MENU_VIDEO, MENU_FILTER, MENU_OUTPUT,
       MENU_CALIBRATE, MENU_EXPERIMENTAL, MENU_EXIT, MENU_ITEMS,
       MENU_DFILT, MENU_AFC, MENU_GAIN, MENU_DEMOD, MENU_DAC }; /* legacy action IDs */
#define MENU_VISIBLE_ROWS 9   /* the list scrolls when it has more items */
/* CALIBRATE submenu (hold on CALIBRATE): QUICK = the active filter and its
 * DC rows (console F), FULL = every filter class (console B), BACK. */
enum { MENU_CAL_QUICK, MENU_CAL_FULL, MENU_CAL_BACK, MENU_CAL_ITEMS };

/* DAC codes (blanking 20, sync 0 never appears in the UI rows). */
enum {
    MENU_INK_BLACK = 20,
    MENU_INK_PANEL = 23,    /* the live card's surface */
    MENU_INK_TRACK = 28,    /* empty meter track and signal bars */
    MENU_INK_RULE = 24,     /* hairlines */
    MENU_INK_DIM = 37,      /* notes, units */
    MENU_INK_GREY = 44,     /* secondary text */
    MENU_INK_WHITE = 60,
};

typedef struct {
    bool exp_open, exp_active, exp_failed, exp_reboot, exp_coarse;
    bool clock_d10; /* active boot wiring, independent of requested setting */
    unsigned exp_cursor;
    uint8_t exp_values[EXP_ITEMS];
    int cursor;                     /* MENU_* */
    int cal_sub;                    /* MENU_CAL_* while the CALIBRATE submenu is open, else -1 */
    /* Tuning. */
    const char *channel;            /* "R1" */
    unsigned freq_mhz;
    unsigned band;                  /* 0..5, the order of band_letters */
    unsigned channel_in_band;       /* 0..7 */
    const char *band_name;          /* "RACEBAND R" */
    const char *band_letters;       /* one letter per band, "RABEFL" */
    unsigned band_count;
    /* Link. */
    int signal;                     /* 0..100 */
    bool video;                     /* confirmed video sync within the last 0.5 s */
    bool afc_locked;                /* AFC centred and locked (FILTER card) */
    bool afc_off;                   /* AFC disabled (returns tuner to 0 kHz offset) */
    bool level_valid, level_above_noise;
    int level_dbm;
    /* Range warnings, shown in place of the link state. */
    bool dc_cal;
    const char *dc_need;            /* "NORMAL+WIDE" */
    bool gain_fixed;
    /* Scan. */
    bool scan_active;
    unsigned scan_progress;         /* 0..100 */
    /* Video standard. */
    int std_mode;                   /* 0 AUTO, 1 NTSC, 2 PAL (std_mode_t) */
    bool std_out_pal;               /* standard the menu itself emits */
    bool detected_valid, detected_pal;
    /* Output level. */
    unsigned output;                /* transport_output() index */
    const char *output_name;        /* "1.5x" */
    /* Gain. */
    const char *gain_mode;          /* "AUTO", "MAX", "G42" */
    const char *demod;              /* selected decoder, short: "P64", "U85Q" */
    bool gain_auto;
    unsigned gain;                  /* active gain index */
    unsigned gain_max;              /* table maximum */
    /* Analog filter (menu FILTER). */
    const char *filter_mode;        /* "AUTO", "XNARROW", "NARROW", "NORMAL", "WIDE" */
    const char *filter_active;      /* active class: "XNARROW", "NARROW", "NORMAL", "WIDE" */
    const char *filter_recommended; /* what AUTO picks for the measured VTX deviation, NULL unknown */
    unsigned filter_width_khz;      /* measured -3 dB width of the active filter, 0 unknown */
    bool narrow_ok, wide_ok;        /* class calibrated with its own DC table */
    int dfilt_mode;                 /* digital RX filter mode 0/4/8/12 (menu PHY FILT) */
    bool dfilt_default;             /* the default 12 (the maps are measured in it) */
    /* VTX and link quality. */
    int deviation_x100;             /* measured this session, < 0 unknown */
    int offset_khz;                 /* AFC tuning offset */
    int cn_x10;                     /* C/N estimate x10, <= -990 unknown */
    const char *firmware;           /* "5.0.123" */
} menu_view_t;

typedef struct {
    const char *title, *line1, *line2;
    int percent;                    /* < 0: no bar */
    const char *channel;
    unsigned freq_mhz;
    bool exp_active;
} menu_status_view_t;

typedef uint8_t menu_ui_rows_t[MENU_UI_LINES][MENU_UI_BYTES];

void menu_ui_render(menu_ui_rows_t ui, const menu_view_t *v);
void menu_ui_render_status(menu_ui_rows_t ui, const menu_status_view_t *v);
/* Boot screen: the C5VRX logo (menu_logo.h) over a loading bar, 0..100. */
void menu_ui_render_splash(menu_ui_rows_t ui, int percent);
void menu_ui_render_splash_health(menu_ui_rows_t ui, int percent, bool needs_calibration);

/* Text that did not fit its box (clipped or shortened), since the last
 * render. The host test requires zero for every page. */
extern unsigned menu_ui_overflow;
