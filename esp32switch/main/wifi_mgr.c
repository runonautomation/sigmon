#include "wifi_mgr.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"

static const char *TAG = "wifi";

#define NVS_NS       "wificfg"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASS "pass"

#define RETRY_DELAY_MS 5000

static esp_netif_t      *s_sta_netif;
static esp_netif_t      *s_ap_netif;
static esp_timer_handle_t s_retry_timer;
static bool              s_connected;
static char              s_ssid[33];
static char              s_pass[65];

static void apply_sta_config(void)
{
    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, s_ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, s_pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
}

static void retry_cb(void *arg)
{
    (void)arg;
    if (s_ssid[0] && !s_connected) {
        esp_wifi_connect();
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        if (s_ssid[0]) {
            ESP_LOGW(TAG, "disconnected from \"%s\", retrying in %dms", s_ssid, RETRY_DELAY_MS);
            /* Retry off the event task so the loop stays responsive. */
            esp_timer_stop(s_retry_timer);
            esp_timer_start_once(s_retry_timer, (uint64_t)RETRY_DELAY_MS * 1000);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        s_connected = true;
        ESP_LOGI(TAG, "connected to \"%s\", ip " IPSTR, s_ssid, IP2STR(&event->ip_info.ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "client joined the SoftAP");
    }
}

static void load_credentials(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = sizeof(s_ssid);
    if (nvs_get_str(h, NVS_KEY_SSID, s_ssid, &len) != ESP_OK) {
        s_ssid[0] = '\0';
    }
    len = sizeof(s_pass);
    if (nvs_get_str(h, NVS_KEY_PASS, s_pass, &len) != ESP_OK) {
        s_pass[0] = '\0';
    }
    nvs_close(h);
}

static void store_credentials(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_str(h, NVS_KEY_SSID, s_ssid);
    nvs_set_str(h, NVS_KEY_PASS, s_pass);
    nvs_commit(h);
    nvs_close(h);
}

esp_err_t wifi_mgr_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_ap_netif  = esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();

    const esp_timer_create_args_t retry_args = {
        .callback = retry_cb,
        .name     = "wifi_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_retry_timer));

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                       &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                       &on_wifi_event, NULL, NULL));

    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, WIFI_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len       = strlen(WIFI_AP_SSID);
    ap.ap.channel        = 1;
    ap.ap.max_connection = 4;
    if (strlen(WIFI_AP_PASS) >= 8) {
        strlcpy((char *)ap.ap.password, WIFI_AP_PASS, sizeof(ap.ap.password));
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    load_credentials();
    apply_sta_config();
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP \"%s\" up at 192.168.4.1", WIFI_AP_SSID);
    if (s_ssid[0]) {
        ESP_LOGI(TAG, "joining \"%s\"", s_ssid);
        esp_wifi_connect();
    } else {
        ESP_LOGI(TAG, "no stored network -- use the console: wifi <ssid> [password]");
    }
    return ESP_OK;
}

esp_err_t wifi_mgr_set_credentials(const char *ssid, const char *pass)
{
    if (ssid == NULL || strlen(ssid) > 32 || (pass != NULL && strlen(pass) > 63)) {
        return ESP_ERR_INVALID_ARG;
    }
    strlcpy(s_ssid, ssid, sizeof(s_ssid));
    strlcpy(s_pass, pass ? pass : "", sizeof(s_pass));
    store_credentials();

    esp_wifi_disconnect();
    s_connected = false;
    apply_sta_config();
    if (s_ssid[0]) {
        ESP_LOGI(TAG, "joining \"%s\"", s_ssid);
        return esp_wifi_connect();
    }
    ESP_LOGI(TAG, "station credentials cleared");
    return ESP_OK;
}

bool wifi_mgr_sta_connected(void)
{
    return s_connected;
}

void wifi_mgr_sta_ssid(char *buf, size_t len)
{
    strlcpy(buf, s_ssid, len);
}

static void netif_ip(esp_netif_t *netif, char *buf, size_t len)
{
    esp_netif_ip_info_t info;
    if (netif != NULL && esp_netif_get_ip_info(netif, &info) == ESP_OK) {
        snprintf(buf, len, IPSTR, IP2STR(&info.ip));
    } else {
        strlcpy(buf, "0.0.0.0", len);
    }
}

void wifi_mgr_sta_ip(char *buf, size_t len)
{
    if (!s_connected) {
        strlcpy(buf, "0.0.0.0", len);
        return;
    }
    netif_ip(s_sta_netif, buf, len);
}

void wifi_mgr_ap_ip(char *buf, size_t len)
{
    netif_ip(s_ap_netif, buf, len);
}
