/* C5VRX P4 display by Twotoz and the C5VRX contributors: ring -> history with
 * the P4's PIE vector unit (cvbs.c fill_history).
 *
 * void cvbs_fill64(const uint8_t *ring, uint8_t *raw, uint8_t *hist, int n, const uint8_t *mask)
 *
 * n groups of 64 ring bytes (16 words of codes D,D,E,E or D,E,E,F): the even
 * bytes (bytes 0 and 2 of each word, the unique 20 MS/s codes) & *mask (0x3F)
 * to raw (32 a group), and the sums of their pairs to hist (16 a group, the
 * 10 MS/s samples). All pointers 16-byte aligned. In C on the CPU this took
 * 55 % of core 0 at 40 MB/s (board 2026-10-31). */
    .text
    .align 4
    .global cvbs_fill64
    .type cvbs_fill64, @function
cvbs_fill64:
    beqz a3, 2f
    esp.vldbc.8.ip q7, a4, 0
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
    esp.vld.128.ip q2, a0, 16
    esp.vld.128.ip q3, a0, 16
    esp.vunzip.8 q0, q1                /* q0: even bytes of words 0-7 */
    esp.vunzip.8 q2, q3                /* q2: even bytes of words 8-15 */
    esp.andq q0, q0, q7
    esp.andq q2, q2, q7
    esp.vst.128.ip q0, a1, 16
    esp.vst.128.ip q2, a1, 16
    esp.vunzip.8 q0, q2                /* q0: first code of each pair, q2: second */
    esp.vadd.u8 q0, q0, q2
    esp.vst.128.ip q0, a2, 16
    addi a3, a3, -1
    bnez a3, 1b
2:
    ret
    .size cvbs_fill64, .-cvbs_fill64
