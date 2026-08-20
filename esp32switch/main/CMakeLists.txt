idf_component_register(
    SRCS
        "main.c"
        "switch_ctl.c"
        "wifi_mgr.c"
        "web.c"
        "state_json.c"
        "console_cmds.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
        esp_hw_support
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        nvs_flash
        console
    EMBED_FILES "index.html"
)
