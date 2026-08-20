#include "console_cmds.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_log.h"
#include "state_json.h"
#include "switch_ctl.h"
#include "wifi_mgr.h"

static bool parse_u64(const char *s, uint64_t *out)
{
    char *end = NULL;
    unsigned long long v = strtoull(s, &end, 10);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = (uint64_t)v;
    return true;
}

/* Nanoseconds are unreadable past a few thousand -- print a friendly unit too. */
static void format_ns(uint64_t ns, char *out, size_t len)
{
    if (ns >= 1000000000ULL) {
        snprintf(out, len, "%llu ns (%.3f s)", (unsigned long long)ns, (double)ns / 1e9);
    } else if (ns >= 1000000ULL) {
        snprintf(out, len, "%llu ns (%.3f ms)", (unsigned long long)ns, (double)ns / 1e6);
    } else if (ns >= 1000ULL) {
        snprintf(out, len, "%llu ns (%.3f us)", (unsigned long long)ns, (double)ns / 1e3);
    } else {
        snprintf(out, len, "%llu ns", (unsigned long long)ns);
    }
}

static int cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;

    sw_state_t st;
    switch_ctl_get(&st);

    int l1, l2, l3;
    switch_ctl_gpios(&l1, &l2, &l3);

    char sta_ip[16], ap_ip[16], ssid[33], pattern[4], dwell[48];
    wifi_mgr_sta_ip(sta_ip, sizeof(sta_ip));
    wifi_mgr_ap_ip(ap_ip, sizeof(ap_ip));
    wifi_mgr_sta_ssid(ssid, sizeof(ssid));
    switch_ctl_pattern(st.port, pattern);
    format_ns(st.dwell_ns, dwell, sizeof(dwell));

    printf("port     : %u  (pattern %s)\n", st.port, pattern);
    printf("lines    : L1=%c L2=%c L3=%c   on GPIO %d/%d/%d\n",
           pattern[0], pattern[1], pattern[2], l1, l2, l3);
    printf("mode     : %s\n", st.iterate ? "iterate" : "hold");
    printf("dwell    : %s\n", dwell);
    printf("sequence :");
    for (uint8_t i = 0; i < st.seq_len; i++) {
        printf(" %u%s", st.seq[i], i == st.seq_idx ? "*" : "");
    }
    printf("   (%u steps, * = current)\n", st.seq_len);
    printf("changes  : %llu since boot\n", (unsigned long long)st.steps);
    printf("station  : %s %s\n", ssid[0] ? ssid : "(unset)",
           wifi_mgr_sta_connected() ? sta_ip : "(not connected)");
    printf("softap   : %s @ %s\n", WIFI_AP_SSID, ap_ip);
    return 0;
}

static int cmd_port(int argc, char **argv)
{
    uint8_t port;
    if (argc != 2 || !switch_ctl_parse_port(argv[1], &port)) {
        printf("usage: port <1..%d | 3-bit pattern>   e.g. \"port 3\" or \"port 011\"\n",
               SW_PORT_COUNT);
        return 1;
    }
    if (switch_ctl_select_port(port) != ESP_OK) {
        printf("failed to select port %u\n", port);
        return 1;
    }
    char pattern[4];
    switch_ctl_pattern(port, pattern);
    printf("holding port %u (%s)\n", port, pattern);
    return 0;
}

static int cmd_dwell(int argc, char **argv)
{
    char text[48];
    if (argc == 1) {
        sw_state_t st;
        switch_ctl_get(&st);
        format_ns(st.dwell_ns, text, sizeof(text));
        printf("dwell %s\n", text);
        return 0;
    }
    uint64_t ns;
    if (argc != 2 || !parse_u64(argv[1], &ns)) {
        printf("usage: dwell [<nanoseconds>]\n");
        return 1;
    }
    if (switch_ctl_set_dwell_ns(ns) != ESP_OK) {
        printf("dwell must be %llu..%llu ns\n",
               (unsigned long long)SW_DWELL_NS_MIN, (unsigned long long)SW_DWELL_NS_MAX);
        return 1;
    }
    format_ns(ns, text, sizeof(text));
    printf("dwell %s\n", text);
    return 0;
}

static int cmd_gpio(int argc, char **argv)
{
    int l1, l2, l3;
    if (argc == 1) {
        switch_ctl_gpios(&l1, &l2, &l3);
        printf("lines L1=GPIO%d L2=GPIO%d L3=GPIO%d\n", l1, l2, l3);
        return 0;
    }
    if (argc != 4) {
        printf("usage: gpio [<L1> <L2> <L3>]   e.g. \"gpio 1 2 3\"\n");
        return 1;
    }
    char *end;
    long v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = strtol(argv[i + 1], &end, 10);
        if (*end != '\0') {
            printf("usage: gpio [<L1> <L2> <L3>]   e.g. \"gpio 1 2 3\"\n");
            return 1;
        }
    }
    if (switch_ctl_set_gpios((int)v[0], (int)v[1], (int)v[2]) != ESP_OK) {
        printf("pins must be distinct, 0..31, and not reserved for flash\n");
        return 1;
    }
    switch_ctl_gpios(&l1, &l2, &l3);
    printf("lines L1=GPIO%d L2=GPIO%d L3=GPIO%d\n", l1, l2, l3);
    return 0;
}

static int cmd_iterate(int argc, char **argv)
{
    bool on;
    if (argc == 2 && (!strcmp(argv[1], "on") || !strcmp(argv[1], "1") ||
                      !strcmp(argv[1], "start"))) {
        on = true;
    } else if (argc == 2 && (!strcmp(argv[1], "off") || !strcmp(argv[1], "0") ||
                             !strcmp(argv[1], "stop"))) {
        on = false;
    } else {
        printf("usage: iterate <on|off>\n");
        return 1;
    }
    switch_ctl_set_iterate(on);
    printf("iterate %s\n", on ? "on" : "off");
    return 0;
}

static int cmd_step(int argc, char **argv)
{
    (void)argc; (void)argv;
    switch_ctl_step();
    sw_state_t st;
    switch_ctl_get(&st);
    char pattern[4];
    switch_ctl_pattern(st.port, pattern);
    printf("port %u (%s)\n", st.port, pattern);
    return 0;
}

static int cmd_seq(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: seq <ports>   e.g. \"seq 1 2 3 6 7\", \"seq 1-5\" or \"seq 001,010,011\"\n");
        return 1;
    }
    /* Join the arguments so both space- and comma-separated forms work. */
    char joined[96] = { 0 };
    size_t off = 0;
    for (int i = 1; i < argc && off < sizeof(joined) - 2; i++) {
        off += (size_t)snprintf(joined + off, sizeof(joined) - off, "%s%s",
                                i > 1 ? "," : "", argv[i]);
    }

    uint8_t ports[SW_SEQ_MAX];
    size_t len = 0;
    if (switch_ctl_parse_sequence(joined, ports, sizeof(ports), &len) != ESP_OK) {
        printf("bad sequence: entries must be ports 1..%d or 3-bit patterns, max %d entries\n",
               SW_PORT_COUNT, SW_SEQ_MAX);
        return 1;
    }
    if (switch_ctl_set_sequence(ports, len) != ESP_OK) {
        printf("failed to set sequence\n");
        return 1;
    }
    printf("sequence:");
    for (size_t i = 0; i < len; i++) {
        printf(" %u", ports[i]);
    }
    printf("\n");
    return 0;
}

static int cmd_count(int argc, char **argv)
{
    uint64_t n;
    if (argc != 2 || !parse_u64(argv[1], &n) ||
        switch_ctl_set_count((uint8_t)n) != ESP_OK) {
        printf("usage: count <1..%d>   -- switch between the first n ports\n", SW_PORT_COUNT);
        return 1;
    }
    printf("switching between ports 1..%llu\n", (unsigned long long)n);
    return 0;
}

/* Machine-readable state on one line, same shape as GET /api/state. */
static int cmd_json(int argc, char **argv)
{
    (void)argc; (void)argv;
    char body[STATE_JSON_MAX];
    if (state_json(body, sizeof(body)) < 0) {
        printf("{\"error\":\"encode failed\"}\n");
        return 1;
    }
    printf("%s\n", body);
    return 0;
}

/* Log lines are interleaved with command output, which is awkward for a program
 * driving this port. "log off" silences them; responses still come through. */
static int cmd_log(int argc, char **argv)
{
    if (argc == 2 && (!strcmp(argv[1], "off") || !strcmp(argv[1], "0"))) {
        esp_log_level_set("*", ESP_LOG_NONE);
        printf("log off\n");
        return 0;
    }
    if (argc == 2 && (!strcmp(argv[1], "on") || !strcmp(argv[1], "1"))) {
        esp_log_level_set("*", ESP_LOG_INFO);
        printf("log on\n");
        return 0;
    }
    printf("usage: log <on|off>\n");
    return 1;
}

static int cmd_wifi(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        printf("usage: wifi <ssid> [password]   (wifi \"\" clears)\n");
        return 1;
    }
    if (wifi_mgr_set_credentials(argv[1], argc == 3 ? argv[2] : "") != ESP_OK) {
        printf("ssid must be <= 32 chars, password <= 63\n");
        return 1;
    }
    printf("stored; joining \"%s\"\n", argv[1]);
    return 0;
}

esp_err_t console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt             = "switch>";
    repl_config.max_cmdline_length = 128;

    esp_err_t err;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    err = esp_console_new_repl_usb_serial_jtag(&hw, &repl_config, &repl);
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    err = esp_console_new_repl_uart(&hw, &repl_config, &repl);
#endif
    if (err != ESP_OK) {
        return err;
    }

    static const esp_console_cmd_t cmds[] = {
        { .command = "status",  .help = "Show port, dwell, sequence and network state",
          .func = cmd_status },
        { .command = "port",    .help = "port <1..8|bits> -- hold one port, e.g. 3 or 011",
          .func = cmd_port },
        { .command = "dwell",   .help = "dwell [<ns>] -- get or set the dwell time in nanoseconds",
          .func = cmd_dwell },
        { .command = "iterate", .help = "iterate <on|off> -- walk the sequence, dwell per step",
          .func = cmd_iterate },
        { .command = "step",    .help = "step -- advance one place in the sequence",
          .func = cmd_step },
        { .command = "count",   .help = "count <1..8> -- switch between the first n ports",
          .func = cmd_count },
        { .command = "seq",     .help = "seq <ports> -- set the iteration order, e.g. 1 2 3 6 7 or 1-5",
          .func = cmd_seq },
        { .command = "gpio",    .help = "gpio [<L1> <L2> <L3>] -- which pins the control lines are wired to",
          .func = cmd_gpio },
        { .command = "json",    .help = "json -- full state on one line, for programs",
          .func = cmd_json },
        { .command = "log",     .help = "log <on|off> -- silence log lines while scripting",
          .func = cmd_log },
        { .command = "wifi",    .help = "wifi <ssid> [password] -- store credentials and join",
          .func = cmd_wifi },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_register_help_command());

    return esp_console_start_repl(repl);
}
