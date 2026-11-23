/* C5VRX-5 by Twotoz and the C5VRX contributors: absolute received level at
 * the chip pin, in dBm.
 *
 * Calibration (board 2026-10-19, docs/FINDINGS.md, research/
 * wifi_reference_snr.py): the receiver was tuned to a busy Wi-Fi channel
 * and the vendor AGC's per-packet RSSI was interleaved with our IQ at G83.
 * Packets sat where the PHY's noise floor (-168 dBm/Hz, NF ~6 dB) puts
 * them, within 1-2 dB. So the thermal noise at G83, 4.76 cells^2 (4.93
 * measured minus the 1/6 cells^2 of 4-bit quantisation), is -96.7 dBm in
 * the 13.6 MHz noise bandwidth of the default filter.
 *
 * Every observer probe gives its DC-removed mean power (cells^2 x16) at a
 * gain row; the gain model of Direct Gain (6 dB per RF stage and baseband
 * doubling, 0.83 dB per fine step) refers it to the pin. The model is
 * checked over G54-G83 (one RF stage, baseband only: the board's noise fell
 * 10.5 dB from G82 to G71, model 11.2): about +-1 dB there, a few dB below
 * G54 where RF stages change. Probes that clip are left out and counted.
 *
 * With a VTX at a known distance and power, free space says what should
 * arrive: 25 mW (14 dBm), 2 dBi each side, 10 m at 5.8 GHz is ~-50 dBm.
 * The difference is what is lost before the chip.
 *
 * The noise comes first from C/N-AUTO's noise map (cn_auto.h) of this
 * channel and filter: the highest row that measured real noise, referred to
 * the pin like the probes (rx_level_noise_from_map). That works with and
 * without an external LNA: bare, the top row measures and the reference is
 * the calibration's (on the board 1.7 dB above it at 5865 MHz, the edge of
 * the C5's documented range); behind an LNA the top rows clip and the
 * highest clean row carries the LNA's gain and noise. The older reference,
 * learned with no carrier at the table maximum within -3..+4 dB of the
 * calibration, never learned behind an LNA (2026-11-14: VTX off read as a
 * -79.5 dBm carrier, noise 17 dB over the calibration at G65-G67); it stays
 * the fallback while the map has no row, then the calibration. The carrier
 * is the pin level minus that noise. */
#pragma once
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cn_auto.h"

#define RX_LEVEL_NOISE_DBM      (-96.7f)   /* thermal noise at G83, default filter */
#define RX_LEVEL_NOISE_CELLS2   4.76f
#define RX_LEVEL_QUANT_CELLS2   (1.0f / 6.0f)   /* 4-bit quantisation, both axes */
#define RX_LEVEL_EXACT_FROM     54u        /* rows with one RF stage: the checked model */
#define RX_LEVEL_ROWS           84u

typedef struct {
    bool valid;            /* enough unclipped probes this second */
    bool exact;            /* every probe at G54 or above */
    bool above_noise;      /* the carrier is at least 1 dB over the noise */
    int input_dbm_x10;     /* carrier plus noise at the pin */
    int noise_dbm_x10;     /* the noise subtracted (learned, else calibration) */
    float rel;             /* pin level in units of the G83 calibration noise */
    int carrier_dbm_x10;   /* carrier alone (noise subtracted) */
    int cn_x10;            /* carrier over the noise, default-filter bandwidth */
    uint16_t clipped_pm;   /* probes left out for clipping */
    uint32_t probes;
    uint8_t gain_min, gain_max;
} rx_level_t;

/* sum_x16[g]: summed probe power (cells^2 x16) at row g, n[g] probes, clipped
 * probes; model_q8: Direct Gain's model gain per row (dB Q8). */
static inline rx_level_t rx_level_compute(const uint32_t sum_x16[RX_LEVEL_ROWS], const uint32_t n[RX_LEVEL_ROWS],
                                          uint32_t clipped, const int16_t model_q8[RX_LEVEL_ROWS],
                                          unsigned max_index, float noise_rel)
{
    rx_level_t r = {.gain_min = 255u};
    double rel = 0.0;   /* pin power in units of the G83 thermal noise */
    uint32_t total = 0;
    for (unsigned g = 0; g < RX_LEVEL_ROWS && g <= max_index; ++g) {
        if (!n[g]) continue;
        const double mean = (double)sum_x16[g] / 16.0 / n[g] - RX_LEVEL_QUANT_CELLS2;
        const double to_pin = pow(10.0, (double)(model_q8[max_index] - model_q8[g]) / 2560.0);
        rel += (mean > 0.0 ? mean : 0.0) * to_pin * n[g];
        total += n[g];
        if (g < r.gain_min) r.gain_min = (uint8_t)g;
        if (g > r.gain_max) r.gain_max = (uint8_t)g;
    }
    r.probes = total;
    r.clipped_pm = (uint16_t)(total + clipped ? clipped * 1000u / (total + clipped) : 0u);
    if (total < 50u) { r.gain_min = 0; return r; }
    rel /= (double)total * RX_LEVEL_NOISE_CELLS2;
    if (!(noise_rel > 0.05f)) noise_rel = 1.0f;
    r.rel = (float)rel;
    r.noise_dbm_x10 = (int)lround(10.0 * (RX_LEVEL_NOISE_DBM + 10.0 * log10(noise_rel)));
    r.valid = true;
    r.exact = r.gain_min >= RX_LEVEL_EXACT_FROM;
    r.input_dbm_x10 = (int)lround(10.0 * (RX_LEVEL_NOISE_DBM + 10.0 * log10(rel > 1e-6 ? rel : 1e-6)));
    const double carrier = rel - noise_rel;
    r.above_noise = carrier > 0.26 * noise_rel;   /* 1 dB */
    r.carrier_dbm_x10 = r.above_noise ? (int)lround(10.0 * (RX_LEVEL_NOISE_DBM + 10.0 * log10(carrier))) : r.input_dbm_x10;
    r.cn_x10 = r.above_noise ? (int)lround(100.0 * log10(carrier / noise_rel)) : -990;
    return r;
}

/* The receiver's noise at the pin, in units of the G83 calibration noise,
 * from a C/N-AUTO noise map: the highest row up to `max_index` with its own
 * measurement of real noise (not quantisation-limited), its thermal share
 * referred to the pin with the gain model, as rx_level_compute refers the
 * probes. The highest row has the largest thermal share and the smallest
 * model distance to the calibration. 0 when the map has no such row;
 * *row_out is the row used (255 = none). */
/* A row whose noise reads over this was learned while the noise itself
 * clipped the 4-bit window (behind an LNA: G72 read 19.8 cells^2, 0.8 dB
 * under the G66 row referred up, board 2026-11-14). Such a row is used only
 * when the map has no other: then the lowest, the least clipped.
 * clip_floor (optional, cells^2 per row, 0 = none): the power of no-carrier
 * windows that clipped, which C/N-AUTO does not learn: a lower bound of the
 * row's noise, used like a clipped map row. Without it a gain model parked
 * on a clipping row never got a reference: every window there clipped, the
 * map stayed empty and LEVEL kept the bare-board default (board 2026-11-14,
 * G72, 17701 folds, no move). */
#define RX_LEVEL_MAP_CLEAN_CELLS2 12.0f
static inline float rx_level_noise_from_map_floor(const cna_noise_map_t *m, const float *clip_floor,
                                                  const int16_t model_q8[RX_LEVEL_ROWS], unsigned max_index,
                                                  uint8_t *row_out)
{
    *row_out = 255u;
    int clean = -1, clipped = -1;
    float clipped_v = 0.0f;
    for (int g = (int)(max_index < RX_LEVEL_ROWS ? max_index : RX_LEVEL_ROWS - 1u); g >= 0; --g) {
        const bool own = m->n[g] >= CNA_MIN_SAMPLES && m->noise[g] >= CNA_QLIM_CELLS2 &&
                         m->noise[g] - RX_LEVEL_QUANT_CELLS2 > 0.0f;
        if (own && m->noise[g] <= RX_LEVEL_MAP_CLEAN_CELLS2) { clean = g; break; }
        /* A clipped map row, or any row with a fresh clipped-window bound
         * (also under 12 cells^2: a row whose few clipped peaks the learning
         * refused). The lowest one is kept: the least clipped. */
        const float fl = clip_floor ? clip_floor[g] : 0.0f;
        const float v = (own ? m->noise[g] : 0.0f) > fl ? m->noise[g] : fl;
        if (((own && m->noise[g] > RX_LEVEL_MAP_CLEAN_CELLS2) || fl > 0.0f) && v - RX_LEVEL_QUANT_CELLS2 > 0.0f) {
            clipped = g; clipped_v = v;
        }
    }
    const int g = clean >= 0 ? clean : clipped;
    if (g < 0) return 0.0f;
    const double thermal = (double)(clean >= 0 ? m->noise[g] : clipped_v) - RX_LEVEL_QUANT_CELLS2;
    const double to_pin = pow(10.0, (double)(model_q8[max_index] - model_q8[g]) / 2560.0);
    *row_out = (uint8_t)g;
    return (float)(thermal * to_pin / RX_LEVEL_NOISE_CELLS2);
}
static inline float rx_level_noise_from_map(const cna_noise_map_t *m, const int16_t model_q8[RX_LEVEL_ROWS],
                                            unsigned max_index, uint8_t *row_out)
{
    return rx_level_noise_from_map_floor(m, NULL, model_q8, max_index, row_out);
}

/* A carrier is shown only on evidence: 1 dB over the noise reference is
 * within what the noise itself moves (temperature, a Wi-Fi burst, a
 * reference learned at another moment), and that excess read as a carrier
 * of noise - 6 dB, "-100 dBm" with no VTX (operator, 2026-10-20). Without
 * video the carrier must reach the noise (C/N >= 0 dB, total 3 dB over);
 * with video (sync seen this second) 1 dB is enough, so the meter keeps
 * working at the range edge. */
#define RX_LEVEL_NO_VIDEO_MIN_CN_X10 0
static inline rx_level_t rx_level_gate(rx_level_t r, bool video)
{
    if (r.valid && r.above_noise && !video && r.cn_x10 < RX_LEVEL_NO_VIDEO_MIN_CN_X10) {
        r.above_noise = false;
        r.carrier_dbm_x10 = r.input_dbm_x10;
        r.cn_x10 = -990;
    }
    return r;
}
