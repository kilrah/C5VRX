idf_component_register(SRCS "main.c" "cvbs.c" "display.c" "link.c" "ui.c" "uvc.c"
                       PRIV_INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_parlio esp_driver_uart esp_driver_ledc esp_driver_i2c esp_driver_jpeg esp_driver_ppa
                                     esp_driver_gpio esp_lcd esp_mm esp_timer esp_hw_support nvs_flash)
# The per-sample loops of the decoder need real optimisation whatever the
# project level.
set_source_files_properties(cvbs.c display.c PROPERTIES COMPILE_OPTIONS "-O2")
