set(srcs
    "main.c" "rf.c" "phy.c" "arc_phy.c" "transport.c" "gain.c" "agc.c" "receiver.c"
    "control.c" "calibration.c" "calibration_sphase.c" "calibration_filter.c" "calibration_dc.c"
    "calibration_bench.c" "menu.c" "menu_ui.c" "menu_raster.c" "analog_video_detect.c"
    "experimental.c" "experimental_phy.c" "settings.c" "console.c" "profile.c" "diag.c" "flightlog.c" "sense_lab.c")
# FireBeetle 2 ESP32-C5 as the tuner of an ESP32-P4 display (docs/P4LINK.md):
# DAC codes plus clock on GPIO2..8, UART status/commands on GPIO11/12.
if(CONFIG_C5VRX_BOARD_FIREBEETLE_P4LINK)
    list(APPEND srcs "p4link.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_phy nvs_flash esp_event esp_netif esp_driver_parlio
             esp_driver_gpio esp_driver_bitscrambler esp_driver_usb_serial_jtag
             esp_driver_tsens esp_timer esp_mm hal heap esp_hw_support esp_app_format esp_partition
             esp_driver_uart)

if(CONFIG_C5VRX_BOARD_FIREBEETLE_P4LINK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE C5VRX_P4LINK=1)
endif()

target_bitscrambler_add_src("p64.bsasm")
target_bitscrambler_add_src("u85q.bsasm")

# The AFC decoder must not inherit the application's debug-oriented -Og.
set_source_files_properties(control.c PROPERTIES COMPILE_OPTIONS "-O2")

# Firmware identity in every log (FIRMWARE line, fw= fields): CI passes the
# exact head commit; local builds ask git at configure time and mark a dirty
# tree. The ELF SHA-256 printed beside it identifies the binary itself.
if(NOT C5VRX5_COMMIT)
    execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
                    OUTPUT_VARIABLE C5VRX5_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE git_rc)
    if(git_rc EQUAL 0 AND C5VRX5_COMMIT)
        execute_process(COMMAND git status --porcelain --untracked-files=no WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
                        OUTPUT_VARIABLE git_dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(git_dirty)
            set(C5VRX5_COMMIT "${C5VRX5_COMMIT}-dirty")
        endif()
    else()
        set(C5VRX5_COMMIT "unknown")
    endif()
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE C5VRX5_COMMIT="${C5VRX5_COMMIT}")

# Filter capacitor, digital RX mode and DC DAC access use PHY internals recovered from this
# exact ESP-IDF 6.0.2 libphy; any other PHY binary builds with them refused.
set(C5_PHY_ARCHIVE "$ENV{IDF_PATH}/components/esp_phy/lib/esp32c5/libphy.a")
if(EXISTS "${C5_PHY_ARCHIVE}")
    file(SHA256 "${C5_PHY_ARCHIVE}" C5_PHY_SHA256)
    if(C5_PHY_SHA256 STREQUAL "dbf33c418c8d408d4005c849d12a1432deea82e2e5e57de3c8ddf914d104fffb")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE C5VRX_PHY_PINNED=1)
    else()
        message(WARNING "Unverified C5 PHY: analog/digital filter and DC correction disabled (${C5_PHY_SHA256})")
    endif()
endif()
