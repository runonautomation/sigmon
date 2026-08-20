#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct esp_timer *esp_timer_handle_t;
typedef void (*esp_timer_cb_t)(void *arg);
typedef struct { esp_timer_cb_t callback; void *arg; int dispatch_method; const char *name; bool skip_unhandled_events; } esp_timer_create_args_t;
esp_err_t esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *out);
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t, uint64_t us);
esp_err_t esp_timer_stop(esp_timer_handle_t t);
int64_t esp_timer_get_time(void);
