/*
 * app/log — Phase 0a: RAM ring buffer only.
 *
 * Lines are formatted into a stack buffer with vsnprintf and copied into a
 * 4 KB ring. Overflow drops the oldest line (overwrite-old) and increments a
 * dropped-line counter that a future TCP layer can surface as a "[N dropped]"
 * marker.
 *
 * Line format: "[HH:MM:SS.mmm] LVL  message\r\n",
 *   e.g. "[00:00:01.234] INFO   hello\r\n".
 * No float support (nano-newlib doesn't link %f) — use integer scaling.
 */

#include "log.h"
#include "bsp/time/time.h"
#include "bsp/net/net.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 8 KB ring (was 4 KB). Two reasons: (1) high-rate events like a fade
 * trigger a tight burst of ~15 lines (RX KEYPRESS, cmc_state, SDO
 * start/done x N, NEW_SETPOINT, state/op_mode/movement_status x several);
 * if the TCP log client briefly disconnects, we want to retain the
 * full event chain. (2) The recent "1100 lines dropped" episode showed
 * the buffer overflowing when nobody was attached. STM32G431RB has
 * 32 KB SRAM total — 8 KB for the log is comfortably 25%, still leaves
 * plenty for everything else. */
#define LOG_RING_BYTES     8192
#define LOG_LINE_MAX       256

/* Per the static W6100 socket map in Documentation/architecture.md §10.1.
 * Slot 6 — slot 5 is the OD telemetry UDP socket. */
#define LOG_TCP_SOCKET     ((net_sock_t)6)

/* Per-tick send cap so log_tick is bounded. The W6100 TX buffer is 2 KB
 * but we only need to keep up with real log volume; 512 B/tick at the
 * default tick rate is far more than the heartbeat-only case. */
#define LOG_TX_CHUNK_MAX   512

static uint8_t      s_ring[LOG_RING_BYTES];
static size_t       s_head;            /* next write index */
static size_t       s_tail;            /* oldest unread index; ==head if empty */
static bool         s_full;            /* distinguishes empty from full when head==tail */
static size_t       s_dropped_lines;
static log_level_t  s_level = LOG_LVL_INFO;

/* TCP log socket state — set up in log_tick once the network is up. */
static uint16_t     s_tcp_port    = 0;          /* 0 = not configured yet */
static bool         s_listen_open = false;     /* true once we've opened the listen slot */
static bool         s_have_client = false;     /* true while a client is ESTABLISHED */

static const char *level_str(log_level_t lvl)
{
    switch (lvl) {
        case LOG_LVL_DEBUG: return "DEBUG";
        case LOG_LVL_INFO:  return "INFO ";
        case LOG_LVL_WARN:  return "WARN ";
        case LOG_LVL_ERROR: return "ERROR";
        default:            return "?????";
    }
}

static size_t ring_used(void)
{
    if (s_full) return LOG_RING_BYTES;
    if (s_head >= s_tail) return s_head - s_tail;
    return LOG_RING_BYTES - (s_tail - s_head);
}

static size_t ring_free(void)
{
    return LOG_RING_BYTES - ring_used();
}

/* Advance the tail by n bytes, walking through any partial line boundary
 * to keep the ring line-aligned. Returns true if at least one full line
 * was dropped (caller counts it). */
static bool ring_drop_bytes(size_t n)
{
    if (n >= ring_used()) {
        s_tail = s_head;
        s_full = false;
        return true;
    }
    s_tail = (s_tail + n) % LOG_RING_BYTES;
    s_full = false;
    return true;
}

/* Drop whole lines (up to and including the next '\n') until at least
 * need_bytes are free. */
static void ring_make_room(size_t need_bytes)
{
    while (ring_free() < need_bytes && ring_used() > 0) {
        /* Find the next '\n' after the current tail. */
        size_t i = s_tail;
        size_t scanned = 0;
        size_t used = ring_used();
        bool found = false;
        while (scanned < used) {
            if (s_ring[i] == '\n') { found = true; break; }
            i = (i + 1) % LOG_RING_BYTES;
            scanned++;
        }
        /* Drop scanned+1 bytes (inclusive of the '\n') if a line was
         * found; otherwise drop the lot. */
        size_t drop = found ? (scanned + 1) : used;
        ring_drop_bytes(drop);
        s_dropped_lines++;
    }
}

static void ring_write(const uint8_t *src, size_t len)
{
    if (len == 0) return;
    if (len >= LOG_RING_BYTES) {
        /* Pathological case — line larger than the whole ring. Keep the
         * tail of the line so the most recent content survives. */
        src += (len - LOG_RING_BYTES) + 1;
        len  = LOG_RING_BYTES - 1;
    }
    if (len > ring_free()) ring_make_room(len);

    for (size_t i = 0; i < len; i++) {
        s_ring[s_head] = src[i];
        s_head = (s_head + 1) % LOG_RING_BYTES;
    }
    if (s_head == s_tail) s_full = true;
}

void log_init(void)
{
    s_head = s_tail = 0;
    s_full = false;
    s_dropped_lines = 0;
    s_level = LOG_LVL_INFO;
}

void log_set_tcp_port(uint16_t port) { s_tcp_port = port; }

/* Try to bring the listen socket up. Idempotent — safe to call every tick.
 * Returns true once the socket is in LISTEN. */
static bool ensure_listen(void)
{
    if (s_listen_open) return true;
    if (s_tcp_port == 0) return false;     /* not configured yet */
    if (!net_link_up()) return false;      /* W6100 not ready */

    if (!net_open(LOG_TCP_SOCKET, NET_PROTO_TCP, s_tcp_port, true)) {
        return false;
    }
    s_listen_open = true;
    s_have_client = false;
    return true;
}

void log_tick(void)
{
    if (!ensure_listen()) return;

    net_tcp_state_t state = net_tcp_state(LOG_TCP_SOCKET);

    /* Detect transitions in/out of ESTABLISHED. */
    if (state == NET_TCP_ESTABLISHED && !s_have_client) {
        s_have_client = true;
        /* Don't reset s_tail — newly-connected clients see whatever is
         * currently in the ring as backlog, plus a header explaining what
         * was dropped while no one was listening. */
        if (s_dropped_lines > 0) {
            LOG_INFO("[log: %lu lines dropped while no client was connected]",
                     (unsigned long)s_dropped_lines);
            s_dropped_lines = 0;
        } else {
            LOG_INFO("[log: client connected]");
        }
    } else if ((state == NET_TCP_CLOSE_WAIT || state == NET_TCP_CLOSED) && s_have_client) {
        s_have_client = false;
        s_listen_open = false;
        /* Close and re-open back into LISTEN on the same slot/port. */
        if (!net_tcp_reopen_listen(LOG_TCP_SOCKET, s_tcp_port)) {
            return;                /* will retry next tick via ensure_listen */
        }
        s_listen_open = true;
    }

    /* Only ESTABLISHED sockets actually drain the ring. */
    if (state != NET_TCP_ESTABLISHED) return;

    /* Send one contiguous chunk per tick, bounded by LOG_TX_CHUNK_MAX. */
    uint8_t  chunk[LOG_TX_CHUNK_MAX];
    size_t   n = log_peek(chunk, sizeof(chunk));
    if (n == 0) return;

    int32_t sent = net_send(LOG_TCP_SOCKET, chunk, n);
    if (sent < 0) {
        /* Peer-side fault. Force the next tick to notice via state. */
        s_have_client = false;
        s_listen_open = false;
        net_close(LOG_TCP_SOCKET);
        return;
    }
    if (sent > 0) log_consume((size_t)sent);
}

void log_set_level(log_level_t level) { s_level = level; }
log_level_t log_get_level(void)       { return s_level; }

void log_printf(log_level_t lvl, const char *fmt, ...)
{
    if (lvl < s_level) return;

    char     buf[LOG_LINE_MAX];
    int      prefix_len;
    va_list  ap;

    /* Wall-clock-ish timestamp: HH:MM:SS.mmm since boot. Past 24 h the
     * hour field just keeps counting (no day rollover) — still readable.
     * Easier on the eye than a 6-digit millisecond count, and stays
     * fixed-width as long as the device runs < ~46 days (then HH grows
     * to 4 digits and column alignment drifts — acceptable). */
    uint32_t ms_total = time_ms();
    uint32_t ms       = ms_total % 1000u;
    uint32_t s_total  = ms_total / 1000u;
    uint32_t ss       = s_total % 60u;
    uint32_t mm       = (s_total / 60u) % 60u;
    uint32_t hh       = s_total / 3600u;

    prefix_len = snprintf(buf, sizeof(buf),
                          "[%02lu:%02lu:%02lu.%03lu] %s  ",
                          (unsigned long)hh, (unsigned long)mm,
                          (unsigned long)ss, (unsigned long)ms,
                          level_str(lvl));
    if (prefix_len < 0 || (size_t)prefix_len >= sizeof(buf)) return;

    va_start(ap, fmt);
    int n = vsnprintf(buf + prefix_len, sizeof(buf) - prefix_len, fmt, ap);
    va_end(ap);

    if (n < 0) return;

    size_t total = (size_t)prefix_len + (size_t)n;
    if (total >= sizeof(buf)) {
        /* Truncated. Mark with ellipsis. Leave room for the CRLF below. */
        total = sizeof(buf) - 2;
        buf[total - 3] = '.';
        buf[total - 2] = '.';
        buf[total - 1] = '.';
    }
    /* Use CRLF so Windows terminals (telnet/PuTTY/nc) line-break properly.
     * A bare \n leaves the cursor in the same column on the next line and
     * makes everything stair-step across the screen. Strip any trailing
     * \n the caller may have included so we don't double up. */
    if (total > 0 && buf[total - 1] == '\n') total--;
    if (total + 2 > sizeof(buf)) total = sizeof(buf) - 2;
    buf[total++] = '\r';
    buf[total++] = '\n';

    ring_write((const uint8_t *)buf, total);
}

size_t log_peek(uint8_t *out, size_t max)
{
    /* Contiguous-only: copy up to the wrap boundary in a single call so
     * the caller can hand a single memory range to net_send without
     * splitting. The caller can call log_peek again after a successful
     * consume to pull the wrapped-around bytes. */
    if (!out || max == 0) return 0;
    size_t used = ring_used();
    if (used == 0) return 0;

    size_t to_end = LOG_RING_BYTES - s_tail;       /* bytes from tail to wrap */
    size_t span   = (used < to_end) ? used : to_end;
    size_t n      = (span < max)    ? span : max;

    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[s_tail + i];
    }
    return n;
}

void log_consume(size_t n)
{
    size_t used = ring_used();
    if (n > used) n = used;
    s_tail = (s_tail + n) % LOG_RING_BYTES;
    if (n > 0) s_full = false;
}

size_t log_drain(uint8_t *out, size_t max)
{
    size_t n = log_peek(out, max);
    if (n > 0) log_consume(n);
    return n;
}

size_t log_dropped_lines(void)       { return s_dropped_lines; }
bool   log_tcp_client_connected(void) { return s_have_client; }
