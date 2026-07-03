/*
 * app/web — embedded HTTP config server. See web.h for the contract.
 *
 * Architecture: per-connection request buffer accumulator. On socket 7:
 *   LISTEN  ─(panel SYN)→  SYN_RECV  ─(handshake)→  ESTABLISHED
 *     ↓ read into s_req_buf until "\r\n\r\n" seen
 *     ↓ parse first line + Content-Length
 *     ↓ if POST: read more bytes to Content-Length
 *     ↓ dispatch by (method, path)
 *     ↓ send response (Connection: close)
 *     ↓ disconnect → CLOSED → reopen LISTEN
 *
 * Everything is blocking-on-this-cycle: we read what's available, parse,
 * respond, close. If the browser is slow we hand the partial frame back
 * out to web_tick on the next loop iteration.
 *
 * No JSON library — fixed schema, parsed by find-key-then-strtod / strtol /
 * parse-IP-octets. Same for build: snprintf into a static tx buf.
 *
 * IP changes don't take effect until reboot (W6100 needs reinit with the
 * new IP). The page's "Save + Reboot" button posts /api/save then
 * /api/reboot to give the operator atomic apply-and-restart.
 */

#include "web.h"

#include "app/axis_manager/axis_manager.h"
#include "app/camerad/camerad.h"
#include "app/config/config.h"
#include "app/controller_mgr/controller_mgr.h"
#include "app/log/log.h"
#include "bsp/identity/identity.h"
#include "bsp/net/net.h"
#include "bsp/sys/sys.h"
#include "bsp/time/time.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/*----------------------------------------------------------------------------
 * Externs from web_page.c — the embedded HTML page.
 *---------------------------------------------------------------------------*/
extern const char     WEB_INDEX_HTML[];
extern const size_t   WEB_INDEX_HTML_LEN;

/*----------------------------------------------------------------------------
 * Tunables
 *---------------------------------------------------------------------------*/

#define WEB_SOCKET             ((net_sock_t)7)
#define REQ_BUF_BYTES          1536u    /* fits a full request incl. small body */
#define TX_BUF_BYTES           2048u    /* fits the largest JSON response we send */

/* Boot delay before honouring a reboot request, so the response can fully
 * drain to the browser. */
#define REBOOT_DELAY_MS        200u

/*----------------------------------------------------------------------------
 * Module state
 *---------------------------------------------------------------------------*/

static bool     s_inited;
static uint8_t  s_req_buf[REQ_BUF_BYTES];
static char     s_tx_buf [TX_BUF_BYTES];
static size_t   s_req_len;              /* bytes accumulated in s_req_buf */
static bool     s_listener_up_prev;
static bool     s_reboot_pending;
static uint32_t s_reboot_at_ms;

/*----------------------------------------------------------------------------
 * HTTP send helpers
 *---------------------------------------------------------------------------*/

static void send_full(const void *buf, size_t len)
{
    /* W6100 socket has a 2 KB TX buffer. For our small payloads one
     * net_send is enough; loop defensively in case len > free buffer. */
    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = len;
    while (remaining > 0) {
        int32_t n = net_send(WEB_SOCKET, p, remaining);
        if (n <= 0) {
            LOG_WARN("web: net_send failed (rc=%ld)", (long)n);
            return;
        }
        p         += n;
        remaining -= (size_t)n;
    }
}

static void send_status_response(const char *status_line,
                                 const char *content_type,
                                 const void *body, size_t body_len)
{
    char hdr[160];
    int n = snprintf(hdr, sizeof(hdr),
                     "HTTP/1.1 %s\r\n"
                     "Content-Type: %s\r\n"
                     "Content-Length: %u\r\n"
                     "Connection: close\r\n"
                     "Cache-Control: no-store\r\n"
                     "\r\n",
                     status_line, content_type, (unsigned)body_len);
    if (n > 0) send_full(hdr, (size_t)n);
    if (body && body_len > 0) send_full(body, body_len);
}

static void send_404(void)
{
    static const char body[] = "404 not found\n";
    send_status_response("404 Not Found", "text/plain", body, sizeof(body) - 1);
}

static void send_400(const char *reason)
{
    char body[96];
    int n = snprintf(body, sizeof(body), "400 bad request: %s\n", reason ? reason : "");
    send_status_response("400 Bad Request", "text/plain", body, (size_t)n);
}

static void send_500(const char *reason)
{
    char body[96];
    int n = snprintf(body, sizeof(body), "500: %s\n", reason ? reason : "internal");
    send_status_response("500 Internal Server Error", "text/plain", body, (size_t)n);
}

static void send_json_ok(const char *json, size_t len)
{
    send_status_response("200 OK", "application/json", json, len);
}

static void send_text_ok(const char *text)
{
    send_status_response("200 OK", "text/plain", text, strlen(text));
}

/*----------------------------------------------------------------------------
 * JSON-safe float formatter.
 *
 * Why hand-rolled: (1) newlib-nano disables %f in printf by default in
 * STM32CubeIDE projects (saves ~10 KB of flash) so a vanilla snprintf
 * with %f produces empty output and breaks our JSON. (2) JSON has no
 * Inf/NaN — printf-style "inf"/"nan" would parse-fail in the browser.
 *
 * Emits `null` for non-finite values (browser-side code treats it as
 * "unset / no limit") and a 6-decimal-place fixed representation for
 * finite values. Doesn't touch %f, so works regardless of the build's
 * printf-float linker flag.
 *
 * `buf` must have at least 24 bytes. Returns a pointer to `buf` for
 * easy use with snprintf("%s", ...).
 *---------------------------------------------------------------------------*/
static const char *fmt_f32(float v, char *buf, size_t buflen)
{
    if (isnan(v) || isinf(v)) {
        return "null";
    }
    int neg = (v < 0.0f);
    if (neg) v = -v;

    /* Split integer and fractional parts. Round the fractional to 6 dp
     * and carry into the integer part if it wraps to 1.000000. */
    uint32_t whole = (uint32_t)v;
    float    frac = v - (float)whole;
    uint32_t frac_e6 = (uint32_t)(frac * 1000000.0f + 0.5f);
    if (frac_e6 >= 1000000u) {
        whole++;
        frac_e6 = 0u;
    }
    snprintf(buf, buflen, "%s%lu.%06lu",
             neg ? "-" : "",
             (unsigned long)whole, (unsigned long)frac_e6);
    return buf;
}

/*----------------------------------------------------------------------------
 * Tiny JSON helpers — fixed-schema parse + build.
 *
 * find_key returns a pointer to the FIRST character of the value (number,
 * string opening quote, true/false/null, or [ / { for nested). Returns NULL
 * if the key is missing. Doesn't validate structure — relies on the
 * generator (the embedded page) sending well-formed bodies.
 *---------------------------------------------------------------------------*/

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

static const char *find_key(const char *json, const char *key)
{
    size_t kl = strlen(key);
    const char *p = json;
    while ((p = strchr(p, '"')) != NULL) {
        if (strncmp(p + 1, key, kl) == 0 && p[1 + kl] == '"') {
            const char *q = p + 2 + kl;
            q = skip_ws(q);
            if (*q != ':') return NULL;
            q = skip_ws(q + 1);
            return q;
        }
        p++;
    }
    return NULL;
}

/* Parse a numeric value at p (may have sign, decimal). Returns true on success.
 * Doesn't consume — leaves p at the start of the number. */
static bool parse_f32(const char *p, float *out)
{
    if (!p) return false;
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) return false;
    *out = v;
    return true;
}

static bool parse_i32(const char *p, int32_t *out)
{
    if (!p) return false;
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    *out = (int32_t)v;
    return true;
}

static bool parse_u32(const char *p, uint32_t *out)
{
    if (!p) return false;
    char *end = NULL;
    unsigned long v = strtoul(p, &end, 10);
    if (end == p) return false;
    *out = (uint32_t)v;
    return true;
}

/* Parse a quoted IP "a.b.c.d" at p (p points at the opening quote). */
static bool parse_ip_quoted(const char *p, uint8_t out[4])
{
    if (!p || *p != '"') return false;
    p++;
    unsigned a, b, c, d;
    int matched = sscanf(p, "%u.%u.%u.%u", &a, &b, &c, &d);
    if (matched != 4 || a > 255 || b > 255 || c > 255 || d > 255) return false;
    out[0] = (uint8_t)a; out[1] = (uint8_t)b; out[2] = (uint8_t)c; out[3] = (uint8_t)d;
    return true;
}

/*----------------------------------------------------------------------------
 * HTTP request parsing
 *
 * After we've accumulated a full request (header end + body bytes), parse
 * just enough to dispatch: method, path, content-length, body pointer.
 *---------------------------------------------------------------------------*/

typedef enum { METHOD_GET, METHOD_POST, METHOD_OTHER } method_t;

typedef struct {
    method_t    method;
    const char *path;       /* into s_req_buf; not null-terminated */
    size_t      path_len;
    size_t      content_length;
    const char *body;       /* may be NULL for GET; into s_req_buf */
    size_t      body_len;
} request_t;

static bool path_eq(const request_t *req, const char *literal)
{
    size_t L = strlen(literal);
    if (req->path_len != L) return false;
    return memcmp(req->path, literal, L) == 0;
}

static bool parse_request_header(request_t *out, const uint8_t *buf, size_t len,
                                 size_t *header_end_off)
{
    /* Find "\r\n\r\n" (or "\n\n" tolerated). */
    size_t hdr_end = 0;
    for (size_t i = 3; i < len; i++) {
        if (buf[i-3] == '\r' && buf[i-2] == '\n' && buf[i-1] == '\r' && buf[i] == '\n') {
            hdr_end = i + 1;
            break;
        }
    }
    if (hdr_end == 0) return false;    /* headers not all in yet */
    *header_end_off = hdr_end;

    const char *p = (const char *)buf;

    /* Method */
    if (len >= 4 && memcmp(p, "GET ", 4) == 0) {
        out->method = METHOD_GET;
        p += 4;
    } else if (len >= 5 && memcmp(p, "POST ", 5) == 0) {
        out->method = METHOD_POST;
        p += 5;
    } else {
        out->method = METHOD_OTHER;
        return false;
    }

    /* Path runs to next space (start of HTTP/1.x). */
    const char *space = memchr(p, ' ', hdr_end - (size_t)(p - (const char *)buf));
    if (!space) return false;
    out->path     = p;
    out->path_len = (size_t)(space - p);

    /* Content-Length (case-insensitive find — but we accept the canonical
     * spelling that browsers use). */
    out->content_length = 0;
    const char *cl = strstr((const char *)buf, "Content-Length:");
    if (!cl) cl = strstr((const char *)buf, "content-length:");
    if (cl) {
        const char *v = cl + 15;
        while (*v == ' ' || *v == '\t') v++;
        unsigned long n = strtoul(v, NULL, 10);
        out->content_length = (size_t)n;
    }
    return true;
}

/*----------------------------------------------------------------------------
 * /api/config  — GET (return current values), POST (apply incoming)
 *---------------------------------------------------------------------------*/

static void build_config_json(void)
{
    const network_cfg_t *net = config_get_network();
    uint8_t mac[6];
    identity_get_mac(mac);

    uint32_t up_ms = time_ms();
    uint32_t up_s  = up_ms / 1000u;
    uint32_t up_h  = up_s / 3600u;
    uint32_t up_m  = (up_s / 60u) % 60u;
    uint32_t up_ss = up_s % 60u;

    /* Pre-format each float into a small local buffer; the JSON template
     * then references them via %s. This sidesteps newlib-nano's
     * (typically-disabled) %f and JSON's no-Inf/NaN rule in one go.
     * 24 bytes is enough for any finite F32 (worst case ~16 digits + sign
     * + decimal + NUL). */
    char b_vel[24], b_plo[24], b_phi[24], b_acc[24];
    char b_load[24];
    char b_aup[24], b_adn[24], b_ajk[24];
    const char *s_vel  = fmt_f32(axis_manager_get_velocity_limit(),      b_vel,  sizeof(b_vel));
    const char *s_plo  = fmt_f32(axis_manager_get_position_limit_lo(),   b_plo,  sizeof(b_plo));
    const char *s_phi  = fmt_f32(axis_manager_get_position_limit_hi(),   b_phi,  sizeof(b_phi));
    const char *s_acc  = fmt_f32(axis_manager_get_accel_limit(),         b_acc,  sizeof(b_acc));
    const char *s_load = fmt_f32(axis_manager_get_load_factor(),         b_load, sizeof(b_load));
    const char *s_aup  = fmt_f32(axis_manager_get_vel_accel_up(),        b_aup,  sizeof(b_aup));
    const char *s_adn  = fmt_f32(axis_manager_get_vel_accel_dn(),        b_adn,  sizeof(b_adn));
    const char *s_ajk  = fmt_f32(axis_manager_get_vel_accel_jerk(),      b_ajk,  sizeof(b_ajk));

    int n = snprintf(s_tx_buf, sizeof(s_tx_buf),
        "{"
          "\"status\":{"
            "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"uptime_s\":%lu,"
            "\"uptime_pretty\":\"%luh %lum %lus\""
          "},"
          "\"net\":{"
            "\"ip\":\"%u.%u.%u.%u\","
            "\"netmask\":\"%u.%u.%u.%u\","
            "\"gateway\":\"%u.%u.%u.%u\","
            "\"cmc_device_no\":%lu,"
            "\"tcp_camerad_port\":%u,"
            "\"panel_a_ip\":\"%u.%u.%u.%u\","
            "\"panel_b_port\":%u,"
            "\"panel_b_ip\":\"%u.%u.%u.%u\""
          "},"
          "\"limits\":{"
            "\"velocity\":%s,"
            "\"position_lo\":%s,"
            "\"position_hi\":%s,"
            "\"accel\":%s"
          "},"
          "\"joystick\":{"
            "\"raw_center\":%ld,"
            "\"raw_full_pos\":%ld,"
            "\"raw_full_neg\":%ld,"
            "\"raw_deadband\":%lu,"
            "\"raw_current\":%ld,"
            "\"vel_accel_up\":%s,"
            "\"vel_accel_dn\":%s,"
            "\"vel_accel_jerk\":%s"
          "},"
          "\"dynamics\":{"
            "\"load_factor\":%s"
          "},"
          "\"axis\":{"
            "\"encoder_incremental\":%u,"
            "\"is_homed\":%u,"
            "\"home_status\":%u,"
            "\"role\":%u"
          "}"
        "}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        (unsigned long)up_s,
        (unsigned long)up_h, (unsigned long)up_m, (unsigned long)up_ss,
        net->ip[0], net->ip[1], net->ip[2], net->ip[3],
        net->netmask[0], net->netmask[1], net->netmask[2], net->netmask[3],
        net->gateway[0], net->gateway[1], net->gateway[2], net->gateway[3],
        (unsigned long)net->cmc_device_no,
        (unsigned)net->tcp_camerad_port,
        net->panel_a_ip[0], net->panel_a_ip[1], net->panel_a_ip[2], net->panel_a_ip[3],
        (unsigned)net->panel_b_port,
        net->panel_b_ip[0], net->panel_b_ip[1], net->panel_b_ip[2], net->panel_b_ip[3],
        s_vel, s_plo, s_phi, s_acc,
        (long)axis_manager_get_joystick_raw_center(),
        (long)axis_manager_get_joystick_raw_full_pos(),
        (long)axis_manager_get_joystick_raw_full_neg(),
        (unsigned long)axis_manager_get_joystick_raw_deadband(),
        (long)axis_manager_get_joystick_raw(),
        s_aup, s_adn, s_ajk,
        s_load,
        (unsigned)(axis_manager_encoder_is_incremental() ? 1u : 0u),
        (unsigned)(axis_manager_is_homed() ? 1u : 0u),
        (unsigned)axis_manager_get_home_status(),
        (unsigned)axis_manager_get_axis_role());

    if (n < 0 || (size_t)n >= sizeof(s_tx_buf)) {
        send_500("json too large");
        return;
    }
    send_json_ok(s_tx_buf, (size_t)n);
}

/* Apply any present sub-objects from the incoming JSON. Missing keys are
 * left untouched — the page only POSTs the section the operator changed. */
static void apply_config_json(const char *json)
{
    /* network — IP changes don't apply until reboot. We still update RAM so
     * the GET reflects the staged value; the operator clicks Save + Reboot. */
    const char *net = find_key(json, "net");
    if (net && *net == '{') {
        network_cfg_t cfg = *config_get_network();
        const char *p;

        p = find_key(net, "ip");
        if (p) (void)parse_ip_quoted(p, cfg.ip);

        p = find_key(net, "netmask");
        if (p) (void)parse_ip_quoted(p, cfg.netmask);

        p = find_key(net, "gateway");
        if (p) (void)parse_ip_quoted(p, cfg.gateway);

        p = find_key(net, "cmc_device_no");
        if (p) {
            uint32_t v;
            if (parse_u32(p, &v) && v >= 1u && v <= 255u) cfg.cmc_device_no = v;
        }

        /* tcp_camerad_port — CAMERAD TCP listener A (advertised to panel_a_ip
         * as return_port). Takes effect after reboot. Accept the IANA
         * dynamic range; reject < 1024 (privileged) and > 65535. */
        p = find_key(net, "tcp_camerad_port");
        if (p) {
            uint32_t v;
            if (parse_u32(p, &v) && v >= 1024u && v <= 65535u) {
                cfg.tcp_camerad_port = (uint16_t)v;
            }
        }

        /* panel_a_ip — expected source IP for panel A's POLLs. 0.0.0.0
         * disables slot A (strict mode — no listener opened). */
        p = find_key(net, "panel_a_ip");
        if (p) (void)parse_ip_quoted(p, cfg.panel_a_ip);

        /* panel_b_port — listener for panel B. 0 leaves the previous value
         * (so panel B can be enabled by setting just the IP if a default
         * port is acceptable). Otherwise same validation as A. */
        p = find_key(net, "panel_b_port");
        if (p) {
            uint32_t v;
            if (parse_u32(p, &v) && (v == 0u || (v >= 1024u && v <= 65535u))) {
                cfg.panel_b_port = (uint16_t)v;
            }
        }

        /* panel_b_ip — expected source IP for panel B's POLLs. 0.0.0.0
         * disables slot B. */
        p = find_key(net, "panel_b_ip");
        if (p) (void)parse_ip_quoted(p, cfg.panel_b_ip);

        (void)config_set_network(&cfg);
    }

    /* limits — applied immediately (live). */
    const char *lim = find_key(json, "limits");
    if (lim && *lim == '{') {
        const char *p; float f;
        if ((p = find_key(lim, "velocity"))    && parse_f32(p, &f)) (void)axis_manager_set_velocity_limit(f);
        if ((p = find_key(lim, "position_lo")) && parse_f32(p, &f)) (void)axis_manager_set_position_limit_lo(f);
        if ((p = find_key(lim, "position_hi")) && parse_f32(p, &f)) (void)axis_manager_set_position_limit_hi(f);
        if ((p = find_key(lim, "accel"))       && parse_f32(p, &f)) (void)axis_manager_set_accel_limit(f);
    }

    /* joystick — applied immediately (live). */
    const char *js = find_key(json, "joystick");
    if (js && *js == '{') {
        const char *p; float f; int32_t i; uint32_t u;
        if ((p = find_key(js, "raw_center"))    && parse_i32(p, &i)) (void)axis_manager_set_joystick_raw_center(i);
        if ((p = find_key(js, "raw_full_pos")) && parse_i32(p, &i)) (void)axis_manager_set_joystick_raw_full_pos(i);
        if ((p = find_key(js, "raw_full_neg")) && parse_i32(p, &i)) (void)axis_manager_set_joystick_raw_full_neg(i);
        if ((p = find_key(js, "raw_deadband")) && parse_u32(p, &u)) (void)axis_manager_set_joystick_raw_deadband(u);
        /* vel_accel_up/dn/jerk — motor-owned (0x2300:6/7/8); axis_manager
         * caches locally + fires SDO write. Persisted on motor flash via
         * the motor-save sequencer kicked off by cmc_save_config. */
        if ((p = find_key(js, "vel_accel_up"))   && parse_f32(p, &f)) (void)axis_manager_set_vel_accel_up(f);
        if ((p = find_key(js, "vel_accel_dn"))   && parse_f32(p, &f)) (void)axis_manager_set_vel_accel_dn(f);
        if ((p = find_key(js, "vel_accel_jerk")) && parse_f32(p, &f)) (void)axis_manager_set_vel_accel_jerk(f);
    }

    /* dynamics — load_factor writes through to motor 0x2300:5 via SDO
     * (see axis_manager_set_load_factor for the SDO-write path). Applied
     * immediately on POST; rejected outside [0.3, 2.0]. */
    const char *dyn = find_key(json, "dynamics");
    if (dyn && *dyn == '{') {
        const char *p; float f;
        if ((p = find_key(dyn, "load_factor")) && parse_f32(p, &f)) (void)axis_manager_set_load_factor(f);
    }

    /* axis — currently just role (which CAMERAD MOVEMENT field this CMC
     * consumes). Applied live; persists on the next Save all to flash. */
    const char *ax = find_key(json, "axis");
    if (ax && *ax == '{') {
        const char *p; int32_t i;
        if ((p = find_key(ax, "role")) && parse_i32(p, &i) && i > 0 && i <= 0xFF) {
            (void)axis_manager_set_axis_role((uint8_t)i);
        }
    }

    send_text_ok("ok");
}

/*----------------------------------------------------------------------------
 * /api/stats  — JSON snapshot of controller_mgr counters
 *
 * Consumed by the Python robustness test harness for before/after diffs.
 * Emits the per-opcode rx_ok / tx_ok arrays as objects keyed by the
 * CAMERAD opcode (POLL=1, SELECT=2, ...). Anything zero is still emitted
 * so the test can compute deltas without worrying about missing keys.
 *---------------------------------------------------------------------------*/

static void build_stats_json(void)
{
    controller_mgr_stats_t s;
    controller_mgr_get_stats(&s);

    int n = snprintf(s_tx_buf, sizeof(s_tx_buf),
        "{"
          "\"rx_ok\":{"
            "\"POLL\":%lu,\"SELECT\":%lu,\"DESELECT\":%lu,\"GRAB\":%lu,"
            "\"KP_T1\":%lu,\"KP_T2\":%lu,\"KP_T3\":%lu,\"MOVEMENT\":%lu,"
            "\"LIMIT\":%lu,\"POSITION_REQ\":%lu"
          "},"
          "\"tx_ok\":{"
            "\"POLL\":%lu,\"SELECT\":%lu,\"DESELECT\":%lu,\"GRAB\":%lu,"
            "\"KP_T1\":%lu,\"KP_T2\":%lu,\"KP_T3\":%lu"
          "},"
          "\"errors\":{"
            "\"rx_parse_fail\":%lu,"
            "\"rx_unknown_opcode\":%lu,"
            "\"rx_truncated\":%lu,"
            "\"send_errors\":%lu"
          "},"
          "\"tcp\":{"
            "\"listener_accepts\":%lu,"
            "\"listener_closes\":%lu,"
            "\"outbound_connects\":%lu,"
            "\"outbound_failures\":%lu"
          "},"
          "\"rx_buf_high_water\":%lu"
        "}",
        (unsigned long)s.rx_ok[CAMERAD_MSG_POLL],
        (unsigned long)s.rx_ok[CAMERAD_MSG_SELECT],
        (unsigned long)s.rx_ok[CAMERAD_MSG_DESELECT],
        (unsigned long)s.rx_ok[CAMERAD_MSG_GRAB],
        (unsigned long)s.rx_ok[CAMERAD_MSG_KEYPRESS_T1],
        (unsigned long)s.rx_ok[CAMERAD_MSG_KEYPRESS_T2],
        (unsigned long)s.rx_ok[CAMERAD_MSG_KEYPRESS_T3],
        (unsigned long)s.rx_ok[CAMERAD_MSG_MOVEMENT],
        (unsigned long)s.rx_ok[CAMERAD_MSG_LIMIT],
        (unsigned long)s.rx_ok[CAMERAD_MSG_POSITION_REQ],
        (unsigned long)s.tx_ok[CAMERAD_MSG_POLL],
        (unsigned long)s.tx_ok[CAMERAD_MSG_SELECT],
        (unsigned long)s.tx_ok[CAMERAD_MSG_DESELECT],
        (unsigned long)s.tx_ok[CAMERAD_MSG_GRAB],
        (unsigned long)s.tx_ok[CAMERAD_MSG_KEYPRESS_T1],
        (unsigned long)s.tx_ok[CAMERAD_MSG_KEYPRESS_T2],
        (unsigned long)s.tx_ok[CAMERAD_MSG_KEYPRESS_T3],
        (unsigned long)s.rx_parse_fail,
        (unsigned long)s.rx_unknown_opcode,
        (unsigned long)s.rx_truncated,
        (unsigned long)s.poll_send_errors,
        (unsigned long)s.tcp_listener_accepts,
        (unsigned long)s.tcp_listener_closes,
        (unsigned long)s.tcp_outbound_connects,
        (unsigned long)s.tcp_outbound_failures,
        (unsigned long)s.rx_buf_high_water);

    if (n < 0 || (size_t)n >= sizeof(s_tx_buf)) {
        send_500("stats json too large");
        return;
    }
    send_json_ok(s_tx_buf, (size_t)n);
}

/*----------------------------------------------------------------------------
 * /api/save and /api/reboot
 *---------------------------------------------------------------------------*/

static void handle_save(void)
{
    bool ok1 = axis_manager_save_to_flash();
    bool ok2 = config_save_network_to_flash();
    if (ok1 && ok2) {
        send_text_ok("ok");
        /* Also kick off the motor-side save (disable -> SDO 0x2800:1 -> re-enable).
         * Fire-and-forget; runs over multiple ticks in axis_manager. Operator
         * sees the motor briefly disabled after the HTTP response. */
        (void)axis_manager_request_motor_save();
    } else {
        send_500("flash save failed");
    }
}

static void handle_reboot(void)
{
    /* Respond first so the browser sees the "ok" before the chip vanishes.
     * Defer the actual reset to web_tick so this function can return and
     * the TCP FIN gets out cleanly. */
    send_text_ok("ok; rebooting");
    s_reboot_at_ms   = time_ms() + REBOOT_DELAY_MS;
    s_reboot_pending = true;
    LOG_INFO("web: reboot requested via /api/reboot");
}

static void handle_home(void)
{
    /* Fire-and-forget kick to the axis_manager home sequencer. Progress is
     * observable via /api/config's axis.home_status field (MC_IF_HOME_*
     * enum) which the page polls at 2 s. */
    if (axis_manager_request_home()) send_text_ok("ok");
    else                             send_500("home busy or unavailable");
}

/*----------------------------------------------------------------------------
 * Dispatch
 *---------------------------------------------------------------------------*/

static void dispatch(const request_t *req)
{
    if (req->method == METHOD_GET && path_eq(req, "/")) {
        send_status_response("200 OK", "text/html; charset=utf-8",
                             WEB_INDEX_HTML, WEB_INDEX_HTML_LEN);
        return;
    }
    if (req->method == METHOD_GET && path_eq(req, "/api/config")) {
        build_config_json();
        return;
    }
    if (req->method == METHOD_GET && path_eq(req, "/api/stats")) {
        build_stats_json();
        return;
    }
    if (req->method == METHOD_POST && path_eq(req, "/api/config")) {
        if (!req->body) { send_400("missing body"); return; }
        /* Null-terminate body in-place — there's at least one byte beyond
         * it in s_req_buf because we cap req_len at REQ_BUF_BYTES-1. */
        ((char *)req->body)[req->body_len] = '\0';
        apply_config_json(req->body);
        return;
    }
    if (req->method == METHOD_POST && path_eq(req, "/api/save")) {
        handle_save();
        return;
    }
    if (req->method == METHOD_POST && path_eq(req, "/api/reboot")) {
        handle_reboot();
        return;
    }
    if (req->method == METHOD_POST && path_eq(req, "/api/home")) {
        handle_home();
        return;
    }
    send_404();
}

/*----------------------------------------------------------------------------
 * Connection lifecycle
 *---------------------------------------------------------------------------*/

static void reopen_listen(void)
{
    const network_cfg_t *cfg = config_get_network();
    uint16_t port = cfg->http_port ? cfg->http_port : 80;
    s_req_len = 0;
    if (!net_tcp_reopen_listen(WEB_SOCKET, port)) {
        LOG_ERROR("web: failed to (re)open listen on :%u", (unsigned)port);
    }
}

static void close_connection(void)
{
    /* Graceful close (Sn_CR_DISCON): the W6100 drains its TX buffer
     * before sending FIN, so the full response actually reaches the
     * wire. A plain net_close() (Sn_CR_CLOSE) would drop anything
     * still queued — for small responses (POST's 2-byte "ok") that
     * meant browsers saw 200 OK headers with no body and the
     * subsequent GET hung. This was the build that fixed the user-
     * visible "load failed: TypeError: failed to fetch" toast.
     *
     * Slot doesn't immediately return to LISTEN — it goes through
     * FIN_WAIT → TIME_WAIT → CLOSED in the background (~tens of ms).
     * service_connection's CLOSED/CLOSE_WAIT/OTHER branch then
     * reopens listen cleanly. */
    net_tcp_graceful_close(WEB_SOCKET);
    s_req_len = 0;
}

static void service_connection(void)
{
    net_tcp_state_t st = net_tcp_state(WEB_SOCKET);
    bool up = (st == NET_TCP_ESTABLISHED);

    s_listener_up_prev = up;

    /* Reopen on ANY non-listening, non-established state. */
    if (st == NET_TCP_CLOSED ||
        st == NET_TCP_CLOSE_WAIT ||
        st == NET_TCP_OTHER) {
        reopen_listen();
        return;
    }
    if (!up) return;

    /* Read whatever is available into the accumulator. */
    if (s_req_len < sizeof(s_req_buf) - 1u) {
        int32_t n = net_recv(WEB_SOCKET, s_req_buf + s_req_len,
                             (int32_t)(sizeof(s_req_buf) - 1u - s_req_len));
        if (n > 0) s_req_len += (size_t)n;
    }

    /* Have we got the full request yet? */
    request_t req;
    size_t hdr_end = 0;
    if (!parse_request_header(&req, s_req_buf, s_req_len, &hdr_end)) {
        /* Headers not in yet — wait for next tick. (If the buffer is full
         * with no header end, the request is malformed — give up.) */
        if (s_req_len >= sizeof(s_req_buf) - 1u) {
            send_400("headers too large");
            close_connection();
        }
        return;
    }

    size_t body_have = s_req_len - hdr_end;
    if (body_have < req.content_length) {
        /* Wait for more body bytes. */
        return;
    }

    req.body     = (req.method == METHOD_POST) ? (const char *)(s_req_buf + hdr_end) : NULL;
    req.body_len = req.content_length;

    dispatch(&req);
    close_connection();
}

/*----------------------------------------------------------------------------
 * Lifecycle
 *
 * Throttle: HTTP is configuration-grade traffic, not real-time. Polling the
 * W6100 every 1 ms cyclic tick costs ~100 us per cycle in SPI reads
 * (net_tcp_state + net_recv), which directly steals from the cia402
 * cyclic-frame rate. At 10 ms (~100 Hz), the page still feels instant to
 * a human but we get back ~9 ms of every 10 ms for the cyclic loop.
 *
 * Pending reboot is checked every tick (cheap timestamp compare) so the
 * 200 ms post-response wait isn't blocked by the throttle.
 *---------------------------------------------------------------------------*/

#define WEB_TICK_PERIOD_MS  10u

static uint32_t s_last_tick_ms;

void web_init(void)
{
    s_inited           = true;
    s_req_len          = 0;
    s_listener_up_prev = false;
    s_reboot_pending   = false;
    s_last_tick_ms     = 0u;
    reopen_listen();
    const network_cfg_t *cfg = config_get_network();
    LOG_INFO("web: ready on :%u", (unsigned)(cfg->http_port ? cfg->http_port : 80));
}

void web_tick(void)
{
    if (!s_inited) return;

    /* Reboot countdown — check every cycle (cheap). */
    if (s_reboot_pending && time_ms() >= s_reboot_at_ms) {
        LOG_INFO("web: rebooting now");
        sys_reboot();
        /* not reached */
    }

    /* Throttle the SPI-touching connection service to ~100 Hz so the
     * cyclic SPI link to the motor MCU keeps its 1 kHz cadence. */
    if (time_elapsed_ms(s_last_tick_ms) < WEB_TICK_PERIOD_MS) return;
    s_last_tick_ms = time_ms();

    service_connection();
}
