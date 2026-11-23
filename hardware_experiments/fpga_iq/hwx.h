/* C5VRX by Twotoz and the C5VRX contributors: hardware experiment FPGA_IQ.
 *
 * Built only with -DC5VRX_HWX=fpga_iq (CI job build-hwx); the production
 * firmware never includes this file. It extends the Clock Pin D10 / FPGA
 * capture work of PR #257 and LAP87's bench report (issue #256): instead of
 * the decoder's six-bit composite codes, the FPGA receives the raw MODEM_DIAG
 * Q4/I4 bits and owns demodulation.
 *
 * Source format/tap: MODEM_DIAG ULTRAFINE lane {9,6,5,4}, GPIO matrix,
 * straight to pads; no CPU, DMA or lookup in the FPGA path. The receiver
 * (gain, DC, AFC, calibration) keeps running on its RX40/raw32K loopback of
 * the same eight pads. No DAC and no clock output: PARLIO TX runs on no pads
 * (menu, video and DAC TEST have no electrical output in this build).
 *
 * Wiring: XIAO ESP32-C5 -> Tang Primer 25K Dock, PMOD J4 (operator table
 * 2026-11-16, tang_primer_25k_dock.cst); the pre-2026-10-25 IQ pin order.
 * J4 pins 1/2 +3V3 (not connected), 3/4 GND, 5..12 signals.
 *
 *   iq[n]  signal   XIAO pad       GPIO  J4 pin  ball
 *   0      Q bit 0  D0             1     5       C11
 *   1      Q bit 1  D1             0     6       C10
 *   2      Q bit 2  D2             25    7       B11
 *   3      Q sign   D3             7     8       B10
 *   4      I bit 0  D10            10    9       D11
 *   5      I bit 1  MTDO back pad  5     10      D10
 *   6      I bit 2  MTDI back pad  3     11      G11
 *   7      I sign   MTCK back pad  4     12      G10
 *   -      GND      GND            -     3/4     -
 *
 * Menu on the IQ pads (HWX_MENU_ON_IQ): while the menu or a status screen
 * is shown and nothing measures IQ, the eight pads carry PARLIO TX instead:
 * iq[5:0] = the menu raster's six-bit composite codes (sync 0, black 20,
 * white 60; the DAC's bit order) at TX40, iq[7:6] = 0. Raw IQ never holds
 * I sign and I bit 2 at 0 for a whole line (64 us), so the FPGA can tell the
 * two apart without a flag pin. A scan, calibration or other receiver pause
 * and the boot splash put IQ back on the pads.
 *
 * MTMS (GPIO2) is not connected. 3.3 V LVCMOS inputs on the FPGA, no
 * pull-ups/downs (GPIO7/25 are strapping pads), common ground. Data can
 * pause during receiver restarts; the FPGA must reacquire. */
#pragma once

#define HWX_NAME "FPGA_IQ"
/* Shared MODEM_DIAG output / PARLIO RX input order: Q low..high, I low..high. */
#define HWX_IQ_GPIO_NUMS {1, 0, 25, 7, 10, 5, 3, 4}
/* No resistor DAC: PARLIO TX drives no pads and Clock Pin D10 is refused. */
#define HWX_NO_DAC 1
/* The menu raster on the IQ pads while the menu is shown (see above). */
#define HWX_MENU_ON_IQ 1
