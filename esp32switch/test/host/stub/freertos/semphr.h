#pragma once
#include "FreeRTOS.h"
typedef struct sem *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
int xSemaphoreTake(SemaphoreHandle_t s, TickType_t t);
int xSemaphoreGive(SemaphoreHandle_t s);
