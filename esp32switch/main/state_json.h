/*
 * One JSON encoder for the whole device state, shared by the HTTP API and the
 * serial console so both interfaces always report the same shape.
 */
#pragma once

#include <stddef.h>

/* Writes the state object into `out`. Returns the number of bytes written
 * (excluding the terminator), or a negative value on encoding failure. */
int state_json(char *out, size_t len);

/* Big enough for the whole object with a 32-character SSID. */
#define STATE_JSON_MAX 640
