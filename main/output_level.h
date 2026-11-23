/* C5VRX by Twotoz and contributors: issue #200 manual DAC6 scaling.
 * 1x is bit-exact. High gains can clip; output impedance is not calibrated.
 * No phase/token bits may change when scaling DAC output. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#define OUTPUT_LEVELS 10
/* AUTO OUTPUT (output_auto.h): an affine output word instead of a manual
 * level. The measured sync tip (source DAC code) goes to OUTPUT_AFFINE_TIP and
 * everything around it is multiplied by gain_q3 / 8, saturated to the DAC.
 * Like the manual levels it rewrites DAC bits 0..5 only.
 * The tip sits on code 16, not near code 0: at the range edge the receiver
 * noise and the FM clicks reach far under the tip, and a DAC that cuts them
 * on one side lifts the low-passed tip towards the porch. Host model
 * (research/output_tip_headroom.py, U80P64-R1, NTSC deviation 0.7, C/N 2 dB):
 * lines kept 0.92 at 1.0x, 0.51 with the tip on code 3, 0.91 on code 16. */
#define OUTPUT_AFFINE      0x8000u
#define OUTPUT_AFFINE_TIP  16
static inline unsigned output_affine(unsigned gain_q3, unsigned src_tip)
{
    return OUTPUT_AFFINE | (gain_q3 & 63u) << 6 | (src_tip & 63u);
}
static inline bool output_is_affine(unsigned word) { return (word & OUTPUT_AFFINE) != 0u; }
static inline unsigned output_affine_gain_q3(unsigned word) { return (word >> 6) & 63u; }
static inline unsigned output_affine_tip(unsigned word) { return word & 63u; }

static inline uint16_t output_entry(uint16_t value, unsigned level, bool *clipped)
{
    /* Keep the existing 1.0..2.5x maps; extend their half-step contrast
     * curve through 5.0x, plus 0.5x half-scale. Saturation is always limited to DAC6 bits. */
    static const int pivot[OUTPUT_LEVELS] = {21, 13, 16, 19, 22, 25, 28, 31, 34, 21};
    static const int scale[OUTPUT_LEVELS] = { 2,  3,  4,  5,  6,  7,  8,  9, 10,  1};
    int code;
    if (output_is_affine(level)) {
        const int x8 = 8 * OUTPUT_AFFINE_TIP +
                       (int)output_affine_gain_q3(level) * ((int)(value & 63u) - (int)output_affine_tip(level));
        code = x8 + 4 >= 0 ? (x8 + 4) / 8 : -((-(x8 + 4) + 7) / 8);   /* floor: nearest code, ties up */
    } else {
        if (level >= OUTPUT_LEVELS) level = 0;
        int twice = 2 * pivot[level] + scale[level] * ((int)(value & 63u) - 21);
        /* Round to nearest code, including negative values before saturation. */
        code = twice >= 0 ? (twice + 1) / 2 : twice / 2;
    }
    if (clipped) *clipped = code < 0 || code > 63;
    if (code < 0) code = 0;
    if (code > 63) code = 63;
    return (uint16_t)((value & 0xffc0u) | (unsigned)code);
}
