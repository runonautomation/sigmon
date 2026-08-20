/*
 * WiFi in AP+STA mode: the SoftAP is always up so the web UI stays reachable,
 * and the station joins your network when credentials have been stored.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "ESP32-Switch"
#endif
/* Must be >= 8 characters, or the AP falls back to open. */
#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS "switch1234"
#endif

esp_err_t wifi_mgr_init(void);

/* Store credentials in NVS and (re)connect. Empty ssid clears them. */
esp_err_t wifi_mgr_set_credentials(const char *ssid, const char *pass);

bool wifi_mgr_sta_connected(void);
void wifi_mgr_sta_ssid(char *buf, size_t len);
void wifi_mgr_sta_ip(char *buf, size_t len);
void wifi_mgr_ap_ip(char *buf, size_t len);
