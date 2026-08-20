/* No-op host implementations so the real switch_ctl.c links for logic tests. */
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

const char *esp_err_to_name(esp_err_t e) { (void)e; return "stub"; }
esp_err_t gpio_config(const gpio_config_t *c) { (void)c; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t g, uint32_t l) { (void)g; (void)l; return ESP_OK; }
esp_err_t gpio_reset_pin(gpio_num_t g) { (void)g; return ESP_OK; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *o) { (void)a; *o = (esp_timer_handle_t)1; return ESP_OK; }
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us) { (void)t; (void)us; return ESP_OK; }
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t, uint64_t us) { (void)t; (void)us; return ESP_OK; }
esp_err_t esp_timer_stop(esp_timer_handle_t t) { (void)t; return ESP_OK; }
int64_t esp_timer_get_time(void) { return 0; }
uint32_t esp_cpu_get_cycle_count(void) { static uint32_t c; return c += 1000; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (SemaphoreHandle_t)1; }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return (SemaphoreHandle_t)2; }
int xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) { (void)s; (void)t; return 1; }
int xSemaphoreGive(SemaphoreHandle_t s) { (void)s; return 1; }
BaseType_t xTaskCreate(TaskFunction_t f, const char *n, unsigned st, void *p, unsigned pr, TaskHandle_t *h) { (void)f;(void)n;(void)st;(void)p;(void)pr;(void)h; return pdPASS; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n, unsigned st, void *p, unsigned pr, TaskHandle_t *h, int core) { (void)f;(void)n;(void)st;(void)p;(void)pr;(void)h;(void)core; return pdPASS; }
void vTaskDelay(TickType_t t) { (void)t; }
/* NVS: report "not found" so init() keeps the compiled-in defaults. */
esp_err_t nvs_open(const char *ns, nvs_open_mode_t m, nvs_handle_t *o) { (void)ns;(void)m;(void)o; return ESP_ERR_NOT_FOUND; }
esp_err_t nvs_set_u8(nvs_handle_t h, const char *k, uint8_t v) { (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_set_u32(nvs_handle_t h, const char *k, uint32_t v) { (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_set_u64(nvs_handle_t h, const char *k, uint64_t v) { (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_set_str(nvs_handle_t h, const char *k, const char *v) { (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_set_blob(nvs_handle_t h, const char *k, const void *v, size_t l) { (void)h;(void)k;(void)v;(void)l; return ESP_OK; }
esp_err_t nvs_get_u8(nvs_handle_t h, const char *k, uint8_t *v) { (void)h;(void)k;(void)v; return ESP_ERR_NOT_FOUND; }
esp_err_t nvs_get_u32(nvs_handle_t h, const char *k, uint32_t *v) { (void)h;(void)k;(void)v; return ESP_ERR_NOT_FOUND; }
esp_err_t nvs_get_u64(nvs_handle_t h, const char *k, uint64_t *v) { (void)h;(void)k;(void)v; return ESP_ERR_NOT_FOUND; }
esp_err_t nvs_get_str(nvs_handle_t h, const char *k, char *v, size_t *l) { (void)h;(void)k;(void)v;(void)l; return ESP_ERR_NOT_FOUND; }
esp_err_t nvs_get_blob(nvs_handle_t h, const char *k, void *v, size_t *l) { (void)h;(void)k;(void)v;(void)l; return ESP_ERR_NOT_FOUND; }
esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
void nvs_close(nvs_handle_t h) { (void)h; }
esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { return ESP_OK; }
