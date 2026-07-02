/*
 * app/log — diagnostic log via RAM ring buffer (Phase 0a) and TCP socket (Phase 0b).
 *
 * See app/log/README.md for the contract.
 */

#ifndef APP_LOG_H
#define APP_LOG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LVL_DEBUG = 0,
    LOG_LVL_INFO  = 1,
    LOG_LVL_WARN  = 2,
    LOG_LVL_ERROR = 3,
} log_level_t;

void        log_init(void);
void        log_tick(void);

void        log_set_level(log_level_t level);
log_level_t log_get_level(void);

void        log_printf(log_level_t lvl, const char *fmt, ...)
                __attribute__((format(printf, 2, 3)));

#define LOG_DEBUG(...) log_printf(LOG_LVL_DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  log_printf(LOG_LVL_INFO,  __VA_ARGS__)
#define LOG_WARN(...)  log_printf(LOG_LVL_WARN,  __VA_ARGS__)
#define LOG_ERROR(...) log_printf(LOG_LVL_ERROR, __VA_ARGS__)

/* Cache the TCP log port from config. Call from main_loop_init after
 * config_init() so the value is current before log_tick runs. */
void   log_set_tcp_port(uint16_t port);

/* Read-only debug helpers + the peek/consume pair used by log_tick to
 * stream the ring to a TCP client.
 *
 *   log_peek    — copies up to max bytes from the oldest unread position,
 *                 staying within one contiguous span (does NOT cross the
 *                 wrap boundary in a single call). Does not advance.
 *                 Returns bytes copied (0 if empty).
 *   log_consume — advances the read tail by n bytes (after a successful
 *                 send). n must be <= the most recent log_peek return.
 *   log_drain   — peek + consume, in one go. Convenient for non-network
 *                 callers (e.g. a debugger script) where partial-send
 *                 recovery isn't a concern.
 */
size_t log_peek   (uint8_t *out, size_t max);
void   log_consume(size_t n);
size_t log_drain  (uint8_t *out, size_t max);
size_t log_dropped_lines(void);
bool   log_tcp_client_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_H */
