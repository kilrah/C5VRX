/* C5VRX P4 display by Twotoz and the C5VRX contributors: chroma correlation
 * with the P4's PIE vector unit (cvbs.c). After zerowidth's p4usb
 * `chroma_mix` (fork zerowidth/C5VRX, branch p4usb, commit 3dfac08,
 * p4usb/main/chroma.S), which demodulates its chroma the same way.
 *
 * void cvbs_corr16(const int8_t *x, const int8_t *tab, int32_t *out, int n)
 *
 * For each of n 16-byte blocks of x: out[0] = sum x * tab[0..15] (sin) and
 * out[1] = sum x * tab[16..31] (cos); tab advances 32 bytes and out two
 * words per block. x and tab 16-byte aligned. The scalar loop cost ~17
 * cycles per multiply-accumulate on the board (2026-10-31). */
    .text
    .align 4
    .global cvbs_corr16
    .type cvbs_corr16, @function
cvbs_corr16:
    beqz a3, 2f
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a1, 16
    esp.zero.xacc
    esp.vmulas.s8.xacc q0, q1
    esp.movx.r.xacc.l a4
    sw a4, 0(a2)
    esp.zero.xacc
    esp.vmulas.s8.xacc q0, q2
    esp.movx.r.xacc.l a4
    sw a4, 4(a2)
    addi a2, a2, 8
    addi a3, a3, -1
    bnez a3, 1b
2:
    ret
    .size cvbs_corr16, .-cvbs_corr16
