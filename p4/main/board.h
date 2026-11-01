/* C5VRX P4 display by Twotoz and the C5VRX contributors (docs/P4LINK.md):
 * Guition JC1060P470C (ESP32-P4, JD9165 1024x600 MIPI-DSI, GT911 touch).
 * Pins from the board's schematic (JC1060P470C_I_W_Y-V1.0) and the vendor's
 * working display example; the link pins are on the 26-pin header JP1. */
#pragma once

/* ---- C5 link (C5 side: docs/P4LINK.md) ---- */
/* The C5's eight TX bits, LSB first: JP1 pins 9, 11, 13, 19, 21, 10, 12, 14
 * (C5 GPIO2..7, 26, 27). Decoder RAW: the C5's raw ring byte, Q0..Q3 I0..I3
 * at 40 MS/s; P64/U85Q and the C5 menu: its six-bit codes on bits 0..5. */
#define LINK_DATA_GPIOS     {2, 3, 4, 32, 33, 47, 46, 45}
#define LINK_DATA_BITS      8
#define LINK_CLK_GPIO       20      /* JP1 pin 17, the C5's PARLIO TX clock (GPIO8) */
#define LINK_CLK_HZ         40000000u
/* UART0's default pins (not on JP1), driven by link.c's UART1: the boot
 * log (console UART0) goes out on GPIO37 until then; the C5 ignores it. */
#define LINK_UART_RX_GPIO   38      /* from C5 GPIO11 */
#define LINK_UART_TX_GPIO   37      /* to C5 GPIO12 */
/* GPIO39..48 are powered by the on-chip LDO VO4 (also the TF card's
 * supply): it is set to 3.3 V before those pins are used. */
#define LINK_LDO_CHAN       4
#define LINK_LDO_MV         3300

/* ---- display ---- */
#define LCD_H_RES           1024
#define LCD_V_RES           600
#define LCD_RST_GPIO        5       /* as the vendor example */
#define LCD_BL_GPIO         23
#define LCD_MIPI_LDO_CHAN   3
#define LCD_MIPI_LDO_MV     2500
#define TOUCH_SDA_GPIO      7
#define TOUCH_SCL_GPIO      8

/* ---- screen layout: 4:3 picture on the left, settings column right ---- */
#define VIDEO_W             800
#define VIDEO_H             600
#define PANEL_X0            VIDEO_W
#define PANEL_W             (LCD_H_RES - VIDEO_W)
