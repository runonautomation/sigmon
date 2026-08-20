/*
 * ESP32-C3 one-hot RF switch controller.
 *
 * Control lines A/B/C select one of three switch ports (001 / 010 / 100).
 * The port and the switch time can be set from the web UI or the serial console.
 */
#include "console_cmds.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "switch_ctl.h"
#include "web.h"
#include "wifi_mgr.h"

static const char *TAG = "app";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Outputs first: the switch settles into its stored state before radio setup. */
    ESP_ERROR_CHECK(switch_ctl_init());
    ESP_ERROR_CHECK(wifi_mgr_init());
    ESP_ERROR_CHECK(web_start());
    ESP_ERROR_CHECK(console_start());

    ESP_LOGI(TAG, "ready -- type 'help' for serial commands");
}
