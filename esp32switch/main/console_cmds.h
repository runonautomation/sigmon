#pragma once

#include "esp_err.h"

/* Starts the serial REPL: port, time, auto, pulse, status, wifi. */
esp_err_t console_start(void);
