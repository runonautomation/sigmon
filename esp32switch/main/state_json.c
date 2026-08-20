#include "state_json.h"

#include <stdio.h>
#include <string.h>

#include "switch_ctl.h"
#include "wifi_mgr.h"

/* Escape the few characters that would break a JSON string. An SSID is
 * arbitrary user text, so it cannot go into the response verbatim. */
static void json_escape(const char *in, char *out, size_t len)
{
    size_t o = 0;
    for (size_t i = 0; in[i] != '\0' && o + 2 < len; i++) {
        if (in[i] == '"' || in[i] == '\\') {
            out[o++] = '\\';
            out[o++] = in[i];
        } else if ((unsigned char)in[i] < 0x20) {
            out[o++] = '?'; /* control characters: not worth a \uXXXX escape */
        } else {
            out[o++] = in[i];
        }
    }
    out[o] = '\0';
}

int state_json(char *out, size_t len)
{
    sw_state_t st;
    switch_ctl_get(&st);

    int l1, l2, l3;
    switch_ctl_gpios(&l1, &l2, &l3);

    char sta_ip[16], ap_ip[16], ssid[33], ssid_json[67], pattern[4];
    wifi_mgr_sta_ip(sta_ip, sizeof(sta_ip));
    wifi_mgr_ap_ip(ap_ip, sizeof(ap_ip));
    wifi_mgr_sta_ssid(ssid, sizeof(ssid));
    json_escape(ssid, ssid_json, sizeof(ssid_json));
    switch_ctl_pattern(st.port, pattern);

    char seq[SW_SEQ_MAX * 3 + 2];
    size_t off = 0;
    seq[off++] = '[';
    for (uint8_t i = 0; i < st.seq_len && off < sizeof(seq) - 6; i++) {
        off += (size_t)snprintf(seq + off, sizeof(seq) - off, i ? ",%u" : "%u", st.seq[i]);
    }
    snprintf(seq + off, sizeof(seq) - off, "]");

    /* The port -> pattern map, so callers never duplicate the truth table. */
    char codes[SW_PORT_COUNT * 7 + 2];
    off = 0;
    codes[off++] = '[';
    for (uint8_t p = 1; p <= SW_PORT_COUNT; p++) {
        char pat[4];
        switch_ctl_pattern(p, pat);
        off += (size_t)snprintf(codes + off, sizeof(codes) - off,
                                p > 1 ? ",\"%s\"" : "\"%s\"", pat);
    }
    snprintf(codes + off, sizeof(codes) - off, "]");

    return snprintf(out, len,
                    "{\"port\":%u,\"pattern\":\"%s\",\"dwell_ns\":%llu,\"iterate\":%s,"
                    "\"seq\":%s,\"seq_idx\":%u,\"steps\":%llu,\"codes\":%s,"
                    "\"dwell_min\":%llu,\"dwell_max\":%llu,\"ports\":%u,"
                    "\"gpio\":[%d,%d,%d],"
                    "\"sta\":{\"connected\":%s,\"ssid\":\"%s\",\"ip\":\"%s\"},"
                    "\"ap\":{\"ssid\":\"%s\",\"ip\":\"%s\"}}",
                    st.port, pattern, (unsigned long long)st.dwell_ns,
                    st.iterate ? "true" : "false",
                    seq, st.seq_idx, (unsigned long long)st.steps, codes,
                    (unsigned long long)SW_DWELL_NS_MIN, (unsigned long long)SW_DWELL_NS_MAX,
                    SW_PORT_COUNT, l1, l2, l3,
                    wifi_mgr_sta_connected() ? "true" : "false", ssid_json, sta_ip,
                    WIFI_AP_SSID, ap_ip);
}
