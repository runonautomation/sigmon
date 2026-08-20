#include "web.h"

#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "state_json.h"
#include "switch_ctl.h"
#include "wifi_mgr.h"

static const char *TAG = "web";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* Copy one query parameter out, URL-decoding nothing beyond '+' -- the values
 * we accept are numbers, patterns and comma lists. */
static bool query_str(const char *query, const char *key, char *out, size_t len)
{
    if (httpd_query_key_value(query, key, out, len) != ESP_OK) {
        return false;
    }
    for (char *p = out; *p; p++) {
        if (*p == '+') {
            *p = ' ';
        }
    }
    return true;
}

static bool query_u64(const char *query, const char *key, uint64_t *out)
{
    char val[24];
    if (!query_str(query, key, val, sizeof(val))) {
        return false;
    }
    char *end = NULL;
    unsigned long long parsed = strtoull(val, &end, 10);
    if (end == val || *end != '\0') {
        return false;
    }
    *out = (uint64_t)parsed;
    return true;
}

static esp_err_t send_state(httpd_req_t *req)
{
    char body[STATE_JSON_MAX];
    int n = state_json(body, sizeof(body));
    if (n < 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "encode failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start - 1);
}

static esp_err_t state_get(httpd_req_t *req)
{
    return send_state(req);
}

/* /api/set?dwell_ns=<ns>&count=<n>&seq=<list>&port=<1..8|bits>&iterate=<0|1>&step=1
 * All parameters are optional and applied in that order. */
static esp_err_t set_handler(httpd_req_t *req)
{
    char query[256] = { 0 };
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no parameters");
        return ESP_FAIL;
    }

    uint64_t v;
    if (query_u64(query, "dwell_ns", &v)) {
        if (switch_ctl_set_dwell_ns(v) != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dwell out of range");
            return ESP_FAIL;
        }
    }

    if (query_u64(query, "count", &v)) {
        if (switch_ctl_set_count((uint8_t)v) != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "count must be 1..8");
            return ESP_FAIL;
        }
    }

    char text[80];
    if (query_str(query, "seq", text, sizeof(text))) {
        uint8_t ports[SW_SEQ_MAX];
        size_t len = 0;
        if (switch_ctl_parse_sequence(text, ports, sizeof(ports), &len) != ESP_OK ||
            switch_ctl_set_sequence(ports, len) != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "sequence must be 1..16 ports of 1..8");
            return ESP_FAIL;
        }
    }

    if (query_str(query, "port", text, sizeof(text))) {
        uint8_t port;
        if (!switch_ctl_parse_port(text, &port) ||
            switch_ctl_select_port(port) != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "port must be 1..8 or a 3-bit pattern");
            return ESP_FAIL;
        }
    }

    if (query_u64(query, "iterate", &v)) {
        switch_ctl_set_iterate(v != 0);
    }

    if (query_u64(query, "step", &v) && v != 0) {
        switch_ctl_step();
    }

    return send_state(req);
}

esp_err_t web_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t routes[] = {
        { .uri = "/",          .method = HTTP_GET,  .handler = root_get    },
        { .uri = "/api/state", .method = HTTP_GET,  .handler = state_get   },
        { .uri = "/api/set",   .method = HTTP_GET,  .handler = set_handler },
        { .uri = "/api/set",   .method = HTTP_POST, .handler = set_handler },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI(TAG, "web ui on port 80");
    return ESP_OK;
}
