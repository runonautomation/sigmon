/*
 * 3-bit RF/antenna switch control for the ESP32-C3 / ESP32-S3.
 *
 * Three control lines are driven on ports 1, 2 and 3 of the switch's control
 * header. A pattern is written the way you read it off the truth table --
 * leftmost bit is control line 1:
 *
 *      port   line1 line2 line3
 *        1      0     0     1
 *        2      0     1     0
 *        3      0     1     1
 *        4      1     0     0
 *        5      1     0     1
 *        6      1     1     0
 *        7      1     1     1
 *        8      0     0     0
 *
 * Two modes:
 *   - hold:    select one port (1..8) and stay there.
 *   - iterate: walk the sequence, holding each port for the dwell time.
 *
 * The dwell time is given in nanoseconds. esp_timer only resolves to 1us, so
 * dwells below 2ms are timed by busy-waiting on the CPU cycle counter; see
 * SW_DWELL_NS_MIN and the jitter notes in README.md.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

/* Control-line GPIOs. These are only the DEFAULT: the pins are a stored
 * setting (see switch_ctl_set_gpios and the `gpio` console command), because a
 * compiled-in guess about someone else's wiring fails silently -- the firmware
 * drives three pins that go nowhere, reports every port change, advances its
 * step counter, and the RF never moves.
 *
 *   C3 -- GPIO2/8/9 are strapping, so use 3/4/5.
 *   S3 -- the switch on this build is soldered to 1/2/3. Avoid 0/3/45/46 if
 *         you rewire: they are strapping pins, sampled at reset, so an input
 *         holding one at the wrong level changes how the part boots. GPIO3 is
 *         the JTAG source select and is the marginal one of the three in use. */
#ifdef CONFIG_IDF_TARGET_ESP32S3
#ifndef SW_GPIO_L1
#define SW_GPIO_L1 1
#endif
#ifndef SW_GPIO_L2
#define SW_GPIO_L2 2
#endif
#ifndef SW_GPIO_L3
#define SW_GPIO_L3 3
#endif
#else /* C3 and anything else */
#ifndef SW_GPIO_L1
#define SW_GPIO_L1 3
#endif
#ifndef SW_GPIO_L2
#define SW_GPIO_L2 4
#endif
#ifndef SW_GPIO_L3
#define SW_GPIO_L3 5
#endif
#endif

/* Set to 1 if the switch control inputs are active low. */
#ifndef SW_ACTIVE_LOW
#define SW_ACTIVE_LOW 0
#endif

#define SW_LINES      3
#define SW_PORT_COUNT 8
#define SW_SEQ_MAX    16

/* Dwell bounds, in nanoseconds: 1us .. 1h. */
#define SW_DWELL_NS_MIN 1000ULL
#define SW_DWELL_NS_MAX 3600000000000ULL
#define SW_DWELL_NS_DEF 1000000ULL /* 1ms */

/* At or above this, the dwell is timed by esp_timer instead of a busy-wait. */
#define SW_DWELL_NS_TIMER 2000000ULL /* 2ms */

typedef struct {
    uint8_t  port;                /* currently selected port, 1..8 */
    uint64_t dwell_ns;            /* hold time per step while iterating */
    bool     iterate;             /* walking the sequence */
    uint8_t  seq[SW_SEQ_MAX];     /* iteration order, as port numbers */
    uint8_t  seq_len;
    uint8_t  seq_idx;             /* position reached in the sequence */
    uint64_t steps;               /* total port changes since boot */
} sw_state_t;

esp_err_t switch_ctl_init(void);

/* Hold one port (1..8). Stops iteration. */
esp_err_t switch_ctl_select_port(uint8_t port);

/* Advance one place in the sequence and hold there. Stops iteration. */
esp_err_t switch_ctl_step(void);

esp_err_t switch_ctl_set_dwell_ns(uint64_t ns);

esp_err_t switch_ctl_set_iterate(bool on);

/* Replace the iteration order. len must be 1..SW_SEQ_MAX, entries 1..8. */
esp_err_t switch_ctl_set_sequence(const uint8_t *ports, size_t len);

/* Shorthand for "switch between the first n ports": sequence becomes 1..n. */
esp_err_t switch_ctl_set_count(uint8_t n);

void switch_ctl_get(sw_state_t *out);

/* Render the control pattern for `port` as "011". out must hold 4 bytes. */
void switch_ctl_pattern(uint8_t port, char out[4]);

/* Control code (0..7) for `port`, or 0 if the port is out of range. */
uint8_t switch_ctl_port_code(uint8_t port);

/* Accept a port number ("3") or a control pattern ("011"). Both name port 3. */
bool switch_ctl_parse_port(const char *text, uint8_t *port);

/* Parse "1,2,3", "001 010 111" or a range like "1-5" into a port sequence. */
esp_err_t switch_ctl_parse_sequence(const char *text, uint8_t *out, size_t cap, size_t *len);

void switch_ctl_gpios(int *l1, int *l2, int *l3);

/* Move the control lines to different pins, and remember it.
 *
 * The three must be distinct, within 0..31 (the drive path is a single
 * register write) and not reserved for flash. Rejected with
 * ESP_ERR_INVALID_ARG otherwise. The old pins are released back to inputs, so
 * a rewire does not leave the previous set driving. */
esp_err_t switch_ctl_set_gpios(int l1, int l2, int l3);

/* Is `pin` usable as a control line on this chip? */
bool switch_ctl_pin_ok(int pin);
