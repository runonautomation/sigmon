#pragma once
#include "FreeRTOS.h"
#define pdPASS 1
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
typedef int BaseType_t;
BaseType_t xTaskCreate(TaskFunction_t f, const char *n, unsigned s, void *p, unsigned pr, TaskHandle_t *h);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n, unsigned s, void *p, unsigned pr, TaskHandle_t *h, int core);
void vTaskDelay(TickType_t t);
