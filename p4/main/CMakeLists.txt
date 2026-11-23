# Board (main/Kconfig.projbuild): the JC1060P470C shows the picture on its LCD
# and as a USB webcam; the Waveshare build streams it to a Meta Quest.
set(srcs "main.c" "cvbs.c" "corr.S" "luma.S" "fill.S" "link.c")
# Requirements cannot depend on Kconfig (ESP-IDF resolves them first): all of them.
set(reqs esp_driver_parlio esp_driver_uart esp_driver_gpio esp_driver_bitscrambler esp_mm esp_timer
         esp_hw_support nvs_flash esp_driver_ledc esp_driver_i2c esp_driver_jpeg esp_driver_ppa esp_lcd
         usb esp_eth esp_netif esp_event lwip)
if(CONFIG_C5VRX_P4_QUEST)
    list(APPEND srcs "stream.c" "ncm_host.c")
else()
    list(APPEND srcs "display.c" "ui.c" "uvc.c")
endif()
idf_component_register(SRCS ${srcs}
                       PRIV_INCLUDE_DIRS "."
                       PRIV_REQUIRES ${reqs})
# Raw IQ: the C5's U80P64-R1 in the P4's PARLIO RX path (cvbs.c).
target_bitscrambler_add_src("p64_rx.bsasm")
# The per-sample loops of the decoder need real optimisation whatever the
# project level.
set_source_files_properties(cvbs.c PROPERTIES COMPILE_OPTIONS "-O2")
if(NOT CONFIG_C5VRX_P4_QUEST)
    set_source_files_properties(display.c PROPERTIES COMPILE_OPTIONS "-O2")
endif()
