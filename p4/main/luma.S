/* C5VRX P4 display by Twotoz and the C5VRX contributors: a row's Y with the
 * P4's PIE vector unit (cvbs.c). After zerowidth's p4usb `luma_row` (fork
 * zerowidth/C5VRX, branch p4usb, commit 3dfac08, p4usb/main/luma.S): shifted
 * vectors for the neighbours, s8 multiply-accumulate, a saturating shift.
 *
 * void cvbs_luma16(const int8_t *x, uint8_t *y, int n, const int8_t *k, int shift)
 *
 * n blocks of 16: y[i] = clamp((w0 x'[i-1] + w1 x'[i] + w2 x'[i+1]) >> shift,
 * -128..127) ^ 0x80, with x' = x - offset. k: five 16-byte vectors: offset,
 * w0, w1, w2, 0x80s. x 16-byte aligned, readable from x - 16 to the end of
 * block n (one block past the last output); y and k 16-byte aligned. A
 * scalar loop cost ~23 cycles a pixel on the board (2026-10-31). */
    .text
    .align 4
    .global cvbs_luma16
    .type cvbs_luma16, @function
cvbs_luma16:
    beqz a2, 2f
    esp.vld.128.ip q7, a3, 16          /* offset */
    esp.vld.128.ip q4, a3, 16          /* w0: x[i-1] */
    esp.vld.128.ip q5, a3, 16          /* w1: x[i] */
    esp.vld.128.ip q6, a3, 16          /* w2: x[i+1] */
    addi a5, a0, -16
    esp.vld.128.ip q0, a5, 16          /* block before */
    esp.vsub.s8 q0, q0, q7
    esp.vld.128.ip q1, a0, 16          /* this block */
    esp.vsub.s8 q1, q1, q7
    li t3, 15
    li t4, 1
1:
    esp.vld.128.ip q2, a0, 16          /* next block */
    esp.vsub.s8 q2, q2, q7
    esp.zero.qacc
    esp.vmulas.s8.qacc q1, q5
    esp.movx.w.sar.bytes t3
    esp.src.q q3, q0, q1               /* x[i-1 .. i+14] */
    esp.vmulas.s8.qacc q3, q4
    esp.movx.w.sar.bytes t4
    esp.src.q q3, q1, q2               /* x[i+1 .. i+16] */
    esp.vmulas.s8.qacc q3, q6
    esp.srcmb.s8.qacc q3, a4, 1        /* >> shift, clamped to -128..127 */
    esp.vld.128.ip q0, a3, 0           /* 0x80s (a3 points past w2) */
    esp.xorq q3, q3, q0
    esp.vst.128.ip q3, a1, 16
    esp.orq q0, q1, q1
    esp.orq q1, q2, q2
    addi a2, a2, -1
    bnez a2, 1b
2:
    ret
    .size cvbs_luma16, .-cvbs_luma16
