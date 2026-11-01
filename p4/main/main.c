/* C5VRX P4 display by Twotoz and the C5VRX contributors: ESP32-P4 picture,
 * touch settings and USB webcam for the C5VRX receiver on a FireBeetle 2
 * ESP32-C5 (docs/P4LINK.md).
 * https://github.com/Twotoz/C5VRX - https://twotoz.github.io/C5VRX/ */
#include "board.h"
#include "cvbs.h"
#include "display.h"
#include "esp_ldo_regulator.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "link.h"
#include "nvs_flash.h"
#include "ui.h"
#include "uvc.h"

static const char *TAG = "c5vrx_p4";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    /* The link's GPIO45..47 are powered by LDO VO4: 3.3 V first. */
    esp_ldo_channel_handle_t ldo = NULL;
    const esp_ldo_channel_config_t lcfg = {.chan_id = LINK_LDO_CHAN, .voltage_mv = LINK_LDO_MV};
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&lcfg, &ldo));
    /* The capture first: its 94.5 KB ring needs one block of internal DMA
     * memory, which the later drivers' allocations break up (with the colour
     * history in .bss the largest block left was 108 KB: ESP_ERR_NO_MEM). */
    const esp_err_t cvbs_err = cvbs_start();
    if (cvbs_err != ESP_OK) ESP_LOGE(TAG, "capture: %s", esp_err_to_name(cvbs_err));

    ESP_ERROR_CHECK(display_start());
    if ((err = ui_start()) != ESP_OK) ESP_LOGE(TAG, "touch: %s (no settings column input)", esp_err_to_name(err));
    link_start();
    if ((err = uvc_start()) != ESP_OK) ESP_LOGE(TAG, "UVC: %s", esp_err_to_name(err));
    ESP_LOGI(TAG, "ready");
    for (;;) {   /* bench: decoder state on the console */
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t max_frame_us;
        const uint32_t late = display_late_frames(&max_frame_us);
        printf("PANEL late_frames=%lu max_frame_us=%lu\n", (unsigned long)late, (unsigned long)max_frame_us);
        printf("BOOT capture=%s heap_internal=%u largest=%u\n", esp_err_to_name(cvbs_err),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        cvbs_debug_print();
        link_debug_print();
        printf("RENDER fps=%lu draw_ms=%lu uvc_fps=%lu ppa_errors=%lu panel_ppa_us=%lu uvc_ppa_us=%lu jpeg_us=%lu\n",
               (unsigned long)display_fps(), (unsigned long)display_draw_us() / 1000, (unsigned long)uvc_fps(),
               (unsigned long)display_ppa_errors(), (unsigned long)display_panel_ppa_us(),
               (unsigned long)display_uvc_ppa_us(), (unsigned long)uvc_jpeg_us());
    }
}
