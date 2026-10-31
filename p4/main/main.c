/* C5VRX P4 display by Twotoz and the C5VRX contributors: ESP32-P4 picture,
 * touch settings and USB webcam for the C5VRX receiver on a Waveshare
 * ESP32-C5-LCD-1.47 (docs/P4LINK.md).
 * https://github.com/Twotoz/C5VRX - https://twotoz.github.io/C5VRX/ */
#include "board.h"
#include "cvbs.h"
#include "display.h"
#include "esp_ldo_regulator.h"
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

    ESP_ERROR_CHECK(display_start());
    if ((err = ui_start()) != ESP_OK) ESP_LOGE(TAG, "touch: %s (no settings column input)", esp_err_to_name(err));
    link_start();
    if ((err = cvbs_start()) != ESP_OK) ESP_LOGE(TAG, "capture: %s", esp_err_to_name(err));
    if ((err = uvc_start()) != ESP_OK) ESP_LOGE(TAG, "UVC: %s", esp_err_to_name(err));
    ESP_LOGI(TAG, "ready");
    for (;;) {   /* bench: decoder state on the console */
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t max_frame_us;
        const uint32_t late = display_late_frames(&max_frame_us);
        printf("PANEL late_frames=%lu max_frame_us=%lu\n", (unsigned long)late, (unsigned long)max_frame_us);
        cvbs_debug_print();
        printf("RENDER fps=%lu draw_ms=%lu uvc_fps=%lu ppa_errors=%lu\n", (unsigned long)display_fps(), (unsigned long)display_draw_us() / 1000, (unsigned long)uvc_fps(), (unsigned long)display_ppa_errors());
    }
}
