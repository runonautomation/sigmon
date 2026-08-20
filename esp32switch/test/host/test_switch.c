#include <stdio.h>
#include <string.h>
#include "switch_ctl.h"

static int fails = 0;

static void chk(int cond, const char *what) {
    printf("%s  %s\n", cond ? "ok  " : "FAIL", what);
    if (!cond) fails++;
}

static void chk_port(const char *text, int expect) {
    uint8_t p = 0;
    bool ok = switch_ctl_parse_port(text, &p);
    char msg[96];
    if (expect < 0) {
        snprintf(msg, sizeof(msg), "parse_port(\"%s\") rejected", text);
        chk(!ok, msg);
    } else {
        snprintf(msg, sizeof(msg), "parse_port(\"%s\") == %d (got %u)", text, expect, p);
        chk(ok && p == (uint8_t)expect, msg);
    }
}

static void chk_seq(const char *text, const char *expect) {
    uint8_t out[SW_SEQ_MAX];
    size_t len = 0;
    char got[96] = "";
    esp_err_t err = switch_ctl_parse_sequence(text, out, sizeof(out), &len);
    if (err == ESP_OK) {
        for (size_t i = 0; i < len; i++) {
            snprintf(got + strlen(got), sizeof(got) - strlen(got), i ? ",%u" : "%u", out[i]);
        }
    } else {
        snprintf(got, sizeof(got), "ERR");
    }
    char msg[192];
    snprintf(msg, sizeof(msg), "parse_sequence(\"%s\") -> %s (want %s)", text, got, expect);
    chk(strcmp(got, expect) == 0, msg);
}

static void chk_pattern(uint8_t port, const char *expect) {
    char pat[4];
    switch_ctl_pattern(port, pat);
    char msg[64];
    snprintf(msg, sizeof(msg), "port %u pattern %s (want %s)", port, pat, expect);
    chk(strcmp(pat, expect) == 0, msg);
}

int main(void) {
    chk(switch_ctl_init() == ESP_OK, "switch_ctl_init");

    puts("\n-- truth table --");
    /* Port N carries the binary of N-1. Measured on the fitted switch: the
     * four antennas answer on 000/001/010/011 and nothing answers above. */
    chk_pattern(1, "000"); chk_pattern(2, "001"); chk_pattern(3, "010");
    chk_pattern(4, "011"); chk_pattern(5, "100"); chk_pattern(6, "101");
    chk_pattern(7, "110"); chk_pattern(8, "111");

    puts("\n-- port parsing: number or pattern --");
    chk_port("1", 1); chk_port("3", 3); chk_port("8", 8);
    chk_port("000", 1); chk_port("010", 3); chk_port("110", 7); chk_port("111", 8);
    chk_port("0", -1); chk_port("9", -1); chk_port("012", -1);
    chk_port("", -1); chk_port("3x", -1); chk_port("abc", -1);

    puts("\n-- sequences: lists, ranges, patterns --");
    chk_seq("1 2 3 6 7", "1,2,3,6,7");
    chk_seq("1,2,3,6,7", "1,2,3,6,7");
    chk_seq("1-5", "1,2,3,4,5");          /* switch between 5 ports */
    chk_seq("5-1", "5,4,3,2,1");          /* descending range */
    chk_seq("4-4", "4");
    chk_seq("000,001,010,101,110", "1,2,3,6,7");
    chk_seq("1-3,7,8", "1,2,3,7,8");
    chk_seq("  2 ,, 4  ", "2,4");
    chk_seq("1-9", "ERR");
    chk_seq("", "ERR");
    chk_seq("1 2 3 4 5 6 7 8 1 2 3 4 5 6 7 8 1", "ERR");  /* over SW_SEQ_MAX */

    puts("\n-- count shorthand --");
    sw_state_t st;
    chk(switch_ctl_set_count(5) == ESP_OK, "set_count(5)");
    switch_ctl_get(&st);
    chk(st.seq_len == 5 && st.seq[0] == 1 && st.seq[4] == 5, "count 5 -> ports 1..5");
    chk(switch_ctl_set_count(0) != ESP_OK, "set_count(0) rejected");
    chk(switch_ctl_set_count(9) != ESP_OK, "set_count(9) rejected");

    puts("\n-- hold / dwell / iterate --");
    chk(switch_ctl_select_port(3) == ESP_OK, "select_port(3)");
    switch_ctl_get(&st);
    chk(st.port == 3 && !st.iterate, "holding port 3, iterate off");
    chk(switch_ctl_select_port(9) != ESP_OK, "select_port(9) rejected");
    chk(switch_ctl_set_dwell_ns(500) != ESP_OK, "dwell 500ns rejected (below min)");
    chk(switch_ctl_set_dwell_ns(1000) == ESP_OK, "dwell 1us accepted (min)");
    chk(switch_ctl_set_dwell_ns(SW_DWELL_NS_MAX) == ESP_OK, "dwell 1h accepted (max)");
    chk(switch_ctl_set_dwell_ns(SW_DWELL_NS_MAX + 1) != ESP_OK, "dwell above max rejected");
    chk(switch_ctl_set_iterate(true) == ESP_OK, "iterate on");
    switch_ctl_get(&st);
    chk(st.iterate && st.port == st.seq[0], "iterating from the first sequence entry");
    switch_ctl_step();
    switch_ctl_get(&st);
    chk(!st.iterate && st.seq_idx == 1 && st.port == st.seq[1], "step advances and holds");

    puts("\n-- control-line pins --");
    /* The pins are a setting because a wrong compiled-in default is invisible:
     * the firmware drives three pins that go nowhere and reports success. */
    int l1, l2, l3;
    switch_ctl_gpios(&l1, &l2, &l3);
    chk(l1 == SW_GPIO_L1 && l2 == SW_GPIO_L2 && l3 == SW_GPIO_L3,
        "defaults to the compiled-in pins");
    chk(switch_ctl_set_gpios(1, 2, 3) == ESP_OK, "set_gpios(1,2,3)");
    switch_ctl_gpios(&l1, &l2, &l3);
    chk(l1 == 1 && l2 == 2 && l3 == 3, "reads back 1/2/3");
    chk(switch_ctl_set_gpios(7, 7, 8) != ESP_OK, "two lines on one pin rejected");
    chk(switch_ctl_set_gpios(5, 6, 32) != ESP_OK, "pin above 31 rejected");
    chk(switch_ctl_set_gpios(-1, 6, 7) != ESP_OK, "negative pin rejected");
    switch_ctl_gpios(&l1, &l2, &l3);
    chk(l1 == 1 && l2 == 2 && l3 == 3, "a rejected change leaves the pins alone");
    /* The port must survive a rewire: the lines move, the selection does not. */
    switch_ctl_select_port(4);
    chk(switch_ctl_set_gpios(4, 5, 6) == ESP_OK, "set_gpios(4,5,6)");
    switch_ctl_get(&st);
    chk(st.port == 4, "selected port survives a pin change");
    chk(switch_ctl_set_gpios(1, 2, 3) == ESP_OK, "back to 1/2/3");

    printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails, fails == 1 ? "" : "s");
    return fails != 0;
}
