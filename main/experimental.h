/* C5VRX-5 by Twotoz and the C5VRX contributors: field A/B controls.
 * IDs are flight-log wire identities; append, never reorder. Index zero in
 * each value list is the baseline. Legacy choices keep their own NVS keys. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

enum {
    EXP_GAIN, EXP_DFILT, EXP_AFC, EXP_DEMOD, EXP_IQ_CORR, EXP_PHASE_REPAIR,
    EXP_AUTO_PROFILE, EXP_FAST_GAIN, EXP_FOLD_RADIUS, EXP_FOLD_MEMORY,
    EXP_LEVEL_TARGET, EXP_LEVEL_RELEASE, EXP_GAIN_RATE, EXP_GAIN_CAP,
    EXP_LOG_CADENCE, EXP_LNA_BIAS, EXP_CURRENT, EXP_LOOP_CAP, EXP_SPUR0, EXP_IQ_LANE, EXP_CLOCK_D10,
    EXP_OUTPUT_AUTO, EXP_STRONG_LANE, EXP_LEVEL_TOP, EXP_LOW_DC, EXP_ITEMS,
    EXP_DAC = EXP_ITEMS, EXP_RESET, EXP_BACK, EXP_MENU_ITEMS
};
/* 2 (2026-11-14): AUTO OUTPUT's values swapped to OFF/ON, OFF the baseline
 * (operator: every applied word is a verified ~5 ms restart, unproven on a
 * goggle). experimental_load migrates a version-1 blob: the other items
 * keep their values, AUTO OUTPUT starts OFF. */
#define EXP_VERSION 2u
#define EXP_VERSION_V1 1u

typedef struct { const char *name; uint8_t count; const char *values[8]; } experimental_item_t;
static const experimental_item_t experimental_items[EXP_ITEMS] = {
    {"GAIN", 0, {0}}, {"PHY FILT", 0, {0}}, {"AFC", 0, {0}}, {"DEMOD", 0, {0}},
    {"IQ CORR", 2, {"ON", "OFF"}}, {"PHASE REPAIR", 2, {"ON", "OFF"}},
    {"AUTO PROFILE", 2, {"REPORT", "LIVE"}}, {"FAST GAIN", 2, {"ON", "OFF"}},
    {"FOLD RADIUS", 2, {"OFF", "ON"}}, {"FOLD MEMORY", 3, {"OFF", "300 MS", "1 S"}},
    {"LEVEL TARGET", 3, {"0 DB", "-3 DB", "+3 DB"}},
    {"LEVEL RELEASE", 3, {"NORMAL", "SLOW", "FAST"}},
    {"GAIN RATE", 3, {"0 MS", "5 MS", "20 MS"}},
    {"GAIN CAP", 3, {"G83", "G79", "G75"}},
    {"LOG CADENCE", 3, {"1 S", "5 S", "OFF"}},
    {"LNA BIAS", 2, {"STOCK", "INIT"}},
    {"CURRENT", 4, {"STOCK", "0", "15", "255"}},
    {"LOOP CAP", 8, {"STOCK", "BIN 0", "BIN 1", "BIN 2", "BIN 3", "BIN 4", "BIN 5", "BIN 6"}},
    {"SPUR 0", 3, {"STOCK", "+512", "-512"}},
    {"IQ LANE", 3, {"ULTRA", "FINE", "COARSE"}},
    {"Clock Pin D10", 2, {"OFF", "ON"}},
    {"AUTO OUTPUT", 2, {"OFF", "ON"}},     /* output_auto.h: the video level from the measured sync and white (v2 order) */
    {"STRONG LANE", 2, {"ON", "OFF"}},     /* strong_lane.h: a coarser IQ lane when the lowest gain rows still fold */
    /* agc.h LEVEL_TOP_NOISE_MAX_X16: the most receiver noise (cells^2) the top
     * gain row may carry; behind an external LNA it sets the highest row. */
    {"LEVEL TOP", 4, {"9", "12", "16", "6"}},
    {"LOW DC", 2, {"ON", "OFF"}},          /* calibration_dc.c: DC hold and servo on the rows under the DC table */
};
typedef struct { uint8_t version, value[EXP_ITEMS]; } experimental_blob_t;
/* Reject the entire blob on invalid data; never load half an experiment. */
static inline bool experimental_blob_valid(const experimental_blob_t *b)
{
    if (b->version != EXP_VERSION) return false;
    for (unsigned i = 0; i < EXP_ITEMS; ++i)
        if (b->value[i] >= (experimental_items[i].count ? experimental_items[i].count : 1u)) return false;
    return true;
}
uint8_t experimental_get(unsigned item);
void experimental_load(void);             /* NVS ready, before rf restore/tasks */
bool experimental_set(unsigned item, uint8_t value);
void experimental_reset(void);            /* local controls; caller resets legacy choices */
uint32_t experimental_mask(void);
void experimental_snapshot(uint8_t values[EXP_ITEMS]); /* menu: requested lane */
void experimental_effective_snapshot(uint8_t values[EXP_ITEMS]); /* log: actual boot lane */
void experimental_log_state(void);        /* after legacy choices restore */
/* IQ correction: caller owns the PHY transaction; after vendor restore. */
bool experimental_phy_restore(void);
unsigned experimental_log_interval(void); /* seconds; zero disables flash logging */

/* Bounded patch engine shared with the runtime restore path. */
bool experimental_phy_option(unsigned item, uint8_t value);
void experimental_phy_prepare_restore(void);
void experimental_fault(unsigned item);
void experimental_poll(void); /* control task: persist fallback outside PHY ownership */
bool experimental_phy_failed(void);
unsigned experimental_boot_lane(void);
bool experimental_reboot_needed(void);

/* Noise/calibration records predate experimental transfer controls. Never
 * treat a baseline measurement as exact under a different transfer. */
bool experimental_transfer_active(void);

uint32_t experimental_revision(void);

/* Latched at boot: clock and DAC wiring change together, never live. */
bool experimental_clock_d10(void);
