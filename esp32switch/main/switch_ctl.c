#include "switch_ctl.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "switch";

#define NVS_NS        "swctl"
#define NVS_KEY_PORT  "port"
#define NVS_KEY_DWELL "dwell"
#define NVS_KEY_ITER  "iter"
#define NVS_KEY_SEQ   "seq"
#define NVS_KEY_GPIO  "gpio"

#ifdef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
#define CPU_MHZ CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
#else
#define CPU_MHZ 160
#endif

/* Priority below the WiFi (23) and LWIP (18) tasks: on a single-core part the
 * busy-wait dwell must never starve the network stack, at the cost of some
 * switching jitter. On dual-core parts the task is pinned to the other core. */
#define ITER_TASK_PRIO  10
#define ITER_TASK_STACK 3072

/* Control code per port, index = port - 1. Bit 2 is line 1, bit 0 is line 3.
 *
 * Port N carries the binary of N-1: port 1 is 000, port 8 is 111. MEASURED on
 * the fitted switch, at 96, 103.6 and 105 MHz, by holding each of the eight
 * patterns and looking at what came back:
 *
 *      000 001 010 011 | 100 101 110 111
 *      ant ant ant ant | --- --- --- ---
 *
 * The previous table here was N rather than N-1 -- port 1 drove 001. That is
 * off by one against this switch, and the failure is a nasty one: three of the
 * four antennas still answer (on the wrong ports), the fourth appears dead,
 * and the port that is supposed to be the no-signal reference reads as an
 * antenna. Everything looks like a broken element rather than a mapping.
 *
 * Edit this table if your switch decodes differently, and prove it the same
 * way: dfstream.py --check-switch holds every port and reports which respond. */
static const uint8_t s_port_code[SW_PORT_COUNT] = { 0, 1, 2, 3, 4, 5, 6, 7 };

#define LINE_BIT(line) (1u << (SW_LINES - 1 - (line)))

/* Which pins the switch's control lines are actually wired to.
 *
 * Not const, and settable at runtime: the compiled-in default is a guess about
 * someone else's wiring, and getting it wrong is invisible from this end. The
 * firmware happily drives three pins that go nowhere, the console reports every
 * port change, the step counter advances -- and the RF never moves. That
 * failure costs an afternoon and looks like a broken switch, so the pins are a
 * setting rather than a rebuild. */
static gpio_num_t s_gpio[SW_LINES] = { SW_GPIO_L1, SW_GPIO_L2, SW_GPIO_L3 };

_Static_assert(SW_GPIO_L1 < 32 && SW_GPIO_L2 < 32 && SW_GPIO_L3 < 32,
               "control lines must be on GPIO 0..31 for the single-register write");

/* Pins the chip cannot use for this. Strapping pins are sampled at reset, so a
 * switch input holding one at the wrong level changes how the part boots; the
 * flash/PSRAM pins are simply not available. */
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define SW_PIN_MAX 31
static bool pin_reserved(int p) { return (p >= 26 && p <= 32); }   /* SPI flash */
#else
#define SW_PIN_MAX 31
static bool pin_reserved(int p) { return (p >= 11 && p <= 17); }   /* C3 flash */
#endif

bool switch_ctl_pin_ok(int pin)
{
    return pin >= 0 && pin <= SW_PIN_MAX && !pin_reserved(pin);
}

/* Per-port bit masks for the GPIO set/clear registers, indexed by port. Writing
 * the lines as two register stores instead of three gpio_set_level() calls both
 * shortens the dwell floor and narrows the window where the pattern is invalid
 * mid-change. */
static uint32_t s_clr_mask[SW_PORT_COUNT + 1];
static uint32_t s_set_mask[SW_PORT_COUNT + 1];

static SemaphoreHandle_t  s_lock;
static SemaphoreHandle_t  s_run_sem;    /* released when iteration is enabled */
static esp_timer_handle_t s_tick_timer; /* used for dwells >= SW_DWELL_NS_TIMER */

/* Bumped on every reconfiguration. The busy-wait loop polls it instead of
 * taking the mutex on each step. */
static volatile uint32_t s_gen;
/* True while the busy-wait loop may be driving the control lines. */
static volatile bool s_fast_active;

static sw_state_t s_state = {
    .port     = 1,
    .dwell_ns = SW_DWELL_NS_DEF,
    .iterate  = false,
    .seq      = { 1, 2, 3, 4, 5, 6, 7, 8 },
    .seq_len  = 8,
    .seq_idx  = 0,
    .steps    = 0,
};

/* ---------------------------------------------------------------- helpers */

void switch_ctl_pattern(uint8_t port, char out[4])
{
    uint8_t code = (port >= 1 && port <= SW_PORT_COUNT) ? s_port_code[port - 1] : 0;
    for (int i = 0; i < SW_LINES; i++) {
        out[i] = (code & LINE_BIT(i)) ? '1' : '0';
    }
    out[SW_LINES] = '\0';
}

static void build_masks(void)
{
    for (uint8_t port = 1; port <= SW_PORT_COUNT; port++) {
        uint8_t code = s_port_code[port - 1];
        uint32_t set = 0, clr = 0;
        for (int i = 0; i < SW_LINES; i++) {
            int level = (code & LINE_BIT(i)) ? 1 : 0;
#if SW_ACTIVE_LOW
            level = !level;
#endif
            if (level) {
                set |= 1u << s_gpio[i];
            } else {
                clr |= 1u << s_gpio[i];
            }
        }
        s_set_mask[port] = set;
        s_clr_mask[port] = clr;
    }
}

/* Write the control lines. Kept branch-light: it runs inside the dwell loop.
 * Clear before set, so a transition breaks before it makes -- an RF switch
 * should never momentarily decode two paths at once. */
static inline void drive_port(uint8_t port)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, s_clr_mask[port]);
    REG_WRITE(GPIO_OUT_W1TS_REG, s_set_mask[port]);
}

static void drive_locked(uint8_t port, bool log)
{
    drive_port(port);
    s_state.port = port;
    s_state.steps++;
    if (log) {
        char pat[4];
        switch_ctl_pattern(port, pat);
        ESP_LOGI(TAG, "port %u -> %s (L1=%c L2=%c L3=%c)", port, pat, pat[0], pat[1], pat[2]);
    }
}

static void store_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, NVS_KEY_PORT, s_state.port);
    nvs_set_u64(h, NVS_KEY_DWELL, s_state.dwell_ns);
    nvs_set_u8(h, NVS_KEY_ITER, s_state.iterate ? 1 : 0);
    nvs_set_blob(h, NVS_KEY_SEQ, s_state.seq, s_state.seq_len);
    const uint8_t pins[SW_LINES] = { (uint8_t)s_gpio[0], (uint8_t)s_gpio[1],
                                     (uint8_t)s_gpio[2] };
    nvs_set_blob(h, NVS_KEY_GPIO, pins, sizeof(pins));
    nvs_commit(h);
    nvs_close(h);
}

static void load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t u8;
    uint64_t u64;

    /* Pins first: everything below writes the lines, and writing the wrong
     * ones on the way to the right ones would glitch whatever is on them. */
    uint8_t pins[SW_LINES];
    size_t plen = sizeof(pins);
    if (nvs_get_blob(h, NVS_KEY_GPIO, pins, &plen) == ESP_OK && plen == SW_LINES) {
        bool ok = true;
        for (int i = 0; i < SW_LINES && ok; i++) {
            ok = switch_ctl_pin_ok(pins[i]);
            for (int j = i + 1; j < SW_LINES && ok; j++) {
                ok = (pins[i] != pins[j]);
            }
        }
        if (ok) {
            for (int i = 0; i < SW_LINES; i++) {
                s_gpio[i] = (gpio_num_t)pins[i];
            }
        }
    }

    if (nvs_get_u8(h, NVS_KEY_PORT, &u8) == ESP_OK && u8 >= 1 && u8 <= SW_PORT_COUNT) {
        s_state.port = u8;
    }
    if (nvs_get_u64(h, NVS_KEY_DWELL, &u64) == ESP_OK &&
        u64 >= SW_DWELL_NS_MIN && u64 <= SW_DWELL_NS_MAX) {
        s_state.dwell_ns = u64;
    }
    if (nvs_get_u8(h, NVS_KEY_ITER, &u8) == ESP_OK) {
        s_state.iterate = (u8 != 0);
    }

    uint8_t seq[SW_SEQ_MAX];
    size_t len = sizeof(seq);
    if (nvs_get_blob(h, NVS_KEY_SEQ, seq, &len) == ESP_OK && len >= 1 && len <= SW_SEQ_MAX) {
        bool ok = true;
        for (size_t i = 0; i < len; i++) {
            if (seq[i] < 1 || seq[i] > SW_PORT_COUNT) {
                ok = false;
                break;
            }
        }
        if (ok) {
            memcpy(s_state.seq, seq, len);
            s_state.seq_len = (uint8_t)len;
        }
    }
    nvs_close(h);
}

/* Cycles per dwell. Only used for dwells < SW_DWELL_NS_TIMER, so this always
 * fits in 32 bits (2ms at 240MHz is 480k cycles). */
static inline uint32_t dwell_cycles(uint64_t ns)
{
    return (uint32_t)((ns * CPU_MHZ) / 1000ULL);
}

/* Spin until the cycle counter reaches `target`. Signed comparison so a counter
 * wrap is handled, and so an already-passed target returns immediately -- that
 * is what makes the dwell a period rather than a delay: the time spent writing
 * the lines counts towards the dwell instead of being added on top of it. */
static inline void wait_until(uint32_t target)
{
    while ((int32_t)(esp_cpu_get_cycle_count() - target) < 0) {
        /* spin */
    }
}

/* Tell the busy-wait loop to bail out and wait until it is no longer touching
 * the lines, so the caller's own drive_locked() is the last word. The loop
 * clears the flag without needing the mutex, so holding it here is safe.
 * Caller holds the lock. */
static void quiesce_fast_locked(void)
{
    s_gen++;
    int64_t deadline = esp_timer_get_time() + 5000; /* 5ms > the longest fast dwell */
    while (s_fast_active && esp_timer_get_time() < deadline) {
        /* spin */
    }
}

/* ---------------------------------------------------------- iteration task */

/* Advance one place in the sequence. Caller holds the lock. */
static void advance_locked(void)
{
    if (s_state.seq_len == 0) {
        return;
    }
    s_state.seq_idx = (uint8_t)((s_state.seq_idx + 1) % s_state.seq_len);
    drive_locked(s_state.seq[s_state.seq_idx], false);
}

static void tick_cb(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_state.iterate) {
        advance_locked();
    }
    xSemaphoreGive(s_lock);
}

/* Fast path for sub-millisecond dwells. Taking the mutex on every step would
 * cost more than the dwell itself, so the loop works from a local snapshot and
 * only re-reads shared state when s_gen changes -- checked every `publish`
 * steps, which is also how often it publishes progress back. */
static void iterate_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool     running = s_state.iterate;
        uint64_t dwell   = s_state.dwell_ns;
        uint8_t  seq[SW_SEQ_MAX];
        uint8_t  seq_len = s_state.seq_len;
        uint8_t  idx     = s_state.seq_idx;
        uint32_t gen     = s_gen;
        memcpy(seq, s_state.seq, sizeof(seq));
        xSemaphoreGive(s_lock);

        if (!running || dwell >= SW_DWELL_NS_TIMER || seq_len == 0) {
            /* Idle, or esp_timer is driving the ticks -- just wait for a change. */
            xSemaphoreTake(s_run_sem, pdMS_TO_TICKS(200));
            continue;
        }

        /* Publish roughly every 50ms so the UI stays live without adding
         * per-step overhead. */
        uint32_t publish = (uint32_t)(50000000ULL / dwell);
        if (publish == 0) {
            publish = 1;
        }

        uint32_t cycles = dwell_cycles(dwell);
        uint64_t steps  = 0;
        uint32_t next   = esp_cpu_get_cycle_count();
        while (gen == s_gen) {
            s_fast_active = true;
            for (uint32_t i = 0; i < publish && gen == s_gen; i++) {
                idx = (uint8_t)((idx + 1) % seq_len);
                drive_port(seq[idx]);
                steps++;
                next += cycles;
                wait_until(next);
            }
            s_fast_active = false;

            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_state.steps += steps;
            steps = 0;
            if (gen == s_gen) { /* nothing reconfigured under us */
                s_state.port    = seq[idx];
                s_state.seq_idx = idx;
            }
            xSemaphoreGive(s_lock);
            next = esp_cpu_get_cycle_count(); /* publishing stole time; don't catch up */
        }
    }
}

/* Start or stop whichever timing source the current dwell calls for.
 * Caller holds the lock. */
static void apply_timing_locked(void)
{
    esp_timer_stop(s_tick_timer);
    if (s_state.iterate && s_state.dwell_ns >= SW_DWELL_NS_TIMER) {
        esp_timer_start_periodic(s_tick_timer, s_state.dwell_ns / 1000ULL);
    }
    if (s_state.iterate) {
        xSemaphoreGive(s_run_sem); /* wake the busy-wait task if it is its turn */
    }
}

/* ---------------------------------------------------------------- public */

esp_err_t switch_ctl_init(void)
{
    s_lock    = xSemaphoreCreateMutex();
    s_run_sem = xSemaphoreCreateBinary();
    if (s_lock == NULL || s_run_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Read the stored settings BEFORE configuring any pin: the wiring is one
     * of those settings now, and claiming the compiled-in default first would
     * drive three pins that may belong to something else. */
    load();

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_gpio[0]) | (1ULL << s_gpio[1]) | (1ULL << s_gpio[2]),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    build_masks();

    const esp_timer_create_args_t tick_args = {
        .callback = tick_cb,
        .name     = "sw_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &s_tick_timer));

    xSemaphoreTake(s_lock, portMAX_DELAY);
    drive_locked(s_state.port, true);
    s_state.steps = 0;
    apply_timing_locked();
    xSemaphoreGive(s_lock);

#if CONFIG_FREERTOS_UNICORE
    BaseType_t created = xTaskCreate(iterate_task, "sw_iter", ITER_TASK_STACK, NULL,
                                     ITER_TASK_PRIO, NULL);
#else
    /* Dual-core (S3): pin the busy-wait to APP_CPU so it never competes with
     * the WiFi/LWIP tasks on PRO_CPU. Cuts dwell jitter dramatically. */
    BaseType_t created = xTaskCreatePinnedToCore(iterate_task, "sw_iter", ITER_TASK_STACK,
                                                 NULL, ITER_TASK_PRIO, NULL, 1);
#endif
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "init: L1=GPIO%d L2=GPIO%d L3=GPIO%d active_%s, dwell %lluns, iterate %s",
             SW_GPIO_L1, SW_GPIO_L2, SW_GPIO_L3, SW_ACTIVE_LOW ? "low" : "high",
             (unsigned long long)s_state.dwell_ns, s_state.iterate ? "on" : "off");
    return ESP_OK;
}

esp_err_t switch_ctl_select_port(uint8_t port)
{
    if (port < 1 || port > SW_PORT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.iterate = false;
    quiesce_fast_locked();
    apply_timing_locked();
    drive_locked(port, true);
    /* Keep the sequence cursor in step with the manual selection. */
    for (uint8_t i = 0; i < s_state.seq_len; i++) {
        if (s_state.seq[i] == port) {
            s_state.seq_idx = i;
            break;
        }
    }
    store_locked();
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t switch_ctl_step(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.iterate = false;
    quiesce_fast_locked();
    apply_timing_locked();
    advance_locked();
    uint8_t port = s_state.port;
    store_locked();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "step -> port %u", port);
    return ESP_OK;
}

esp_err_t switch_ctl_set_dwell_ns(uint64_t ns)
{
    if (ns < SW_DWELL_NS_MIN || ns > SW_DWELL_NS_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    quiesce_fast_locked();
    s_state.dwell_ns = ns;
    apply_timing_locked();
    store_locked();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "dwell %lluns", (unsigned long long)ns);
    return ESP_OK;
}

esp_err_t switch_ctl_set_iterate(bool on)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    quiesce_fast_locked();
    s_state.iterate = on;
    if (on && s_state.seq_len > 0) {
        s_state.seq_idx = 0;
        drive_locked(s_state.seq[0], false);
    }
    apply_timing_locked();
    store_locked();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "iterate %s", on ? "on" : "off");
    return ESP_OK;
}

esp_err_t switch_ctl_set_sequence(const uint8_t *ports, size_t len)
{
    if (ports == NULL || len == 0 || len > SW_SEQ_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < len; i++) {
        if (ports[i] < 1 || ports[i] > SW_PORT_COUNT) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    quiesce_fast_locked();
    memcpy(s_state.seq, ports, len);
    s_state.seq_len = (uint8_t)len;
    s_state.seq_idx = 0;
    if (s_state.iterate) {
        drive_locked(s_state.seq[0], false);
        apply_timing_locked();
    }
    store_locked();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "sequence set, %u steps", (unsigned)len);
    return ESP_OK;
}

esp_err_t switch_ctl_set_count(uint8_t n)
{
    if (n < 1 || n > SW_PORT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t ports[SW_PORT_COUNT];
    for (uint8_t i = 0; i < n; i++) {
        ports[i] = (uint8_t)(i + 1);
    }
    return switch_ctl_set_sequence(ports, n);
}

void switch_ctl_get(sw_state_t *out)
{
    if (out == NULL) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_lock);
}

uint8_t switch_ctl_port_code(uint8_t port)
{
    return (port >= 1 && port <= SW_PORT_COUNT) ? s_port_code[port - 1] : 0;
}

bool switch_ctl_parse_port(const char *text, uint8_t *port)
{
    if (text == NULL || *text == '\0' || port == NULL) {
        return false;
    }

    /* Three characters of 0/1 is a control pattern, e.g. "011" -> port 3. */
    if (strlen(text) == SW_LINES &&
        strspn(text, "01") == SW_LINES) {
        uint8_t code = 0;
        for (int i = 0; i < SW_LINES; i++) {
            if (text[i] == '1') {
                code |= LINE_BIT(i);
            }
        }
        for (uint8_t p = 1; p <= SW_PORT_COUNT; p++) {
            if (s_port_code[p - 1] == code) {
                *port = p;
                return true;
            }
        }
        return false;
    }

    char *end = NULL;
    unsigned long v = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || v < 1 || v > SW_PORT_COUNT) {
        return false;
    }
    *port = (uint8_t)v;
    return true;
}

esp_err_t switch_ctl_parse_sequence(const char *text, uint8_t *out, size_t cap, size_t *len)
{
    if (text == NULL || out == NULL || len == NULL || cap == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t n = 0;
    const char *p = text;
    while (*p != '\0') {
        p += strspn(p, ", \t");
        if (*p == '\0') {
            break;
        }
        size_t tlen = strcspn(p, ", \t");
        char token[8];
        if (tlen + 1 > sizeof(token)) {
            return ESP_ERR_INVALID_ARG;
        }
        memcpy(token, p, tlen);
        token[tlen] = '\0';
        p += tlen;

        /* A token may be a single port or an inclusive range like "1-5". */
        uint8_t first, last;
        char *dash = strchr(token + 1, '-'); /* +1: never treat a leading '-' as a range */
        if (dash != NULL) {
            *dash = '\0';
            if (!switch_ctl_parse_port(token, &first) ||
                !switch_ctl_parse_port(dash + 1, &last)) {
                return ESP_ERR_INVALID_ARG;
            }
        } else if (!switch_ctl_parse_port(token, &first)) {
            return ESP_ERR_INVALID_ARG;
        } else {
            last = first;
        }

        int stride = (last >= first) ? 1 : -1;
        for (int p = first;; p += stride) {
            if (n >= cap) {
                return ESP_ERR_INVALID_SIZE;
            }
            out[n++] = (uint8_t)p;
            if (p == last) {
                break;
            }
        }
    }
    if (n == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *len = n;
    return ESP_OK;
}

void switch_ctl_gpios(int *l1, int *l2, int *l3)
{
    if (l1) *l1 = s_gpio[0];
    if (l2) *l2 = s_gpio[1];
    if (l3) *l3 = s_gpio[2];
}

esp_err_t switch_ctl_set_gpios(int l1, int l2, int l3)
{
    const int want[SW_LINES] = { l1, l2, l3 };
    for (int i = 0; i < SW_LINES; i++) {
        if (!switch_ctl_pin_ok(want[i])) {
            return ESP_ERR_INVALID_ARG;
        }
        for (int j = i + 1; j < SW_LINES; j++) {
            if (want[i] == want[j]) {
                return ESP_ERR_INVALID_ARG;   /* two lines on one pin */
            }
        }
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Release the old pins before claiming the new ones, so a rewire does not
     * leave the previous set driving. They go back to inputs rather than being
     * left as outputs at whatever level they happened to hold. */
    for (int i = 0; i < SW_LINES; i++) {
        gpio_reset_pin(s_gpio[i]);
    }
    for (int i = 0; i < SW_LINES; i++) {
        s_gpio[i] = (gpio_num_t)want[i];
    }
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_gpio[0]) | (1ULL << s_gpio[1]) | (1ULL << s_gpio[2]),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    build_masks();
    drive_port(s_state.port);        /* re-assert the current port on the new pins */
    store_locked();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "control lines -> L1=GPIO%d L2=GPIO%d L3=GPIO%d",
             s_gpio[0], s_gpio[1], s_gpio[2]);
    return err;
}
