# app/log

## Purpose
Diagnostic logging over the network. Accepts one TCP client at a time on a configurable port (default 30200), streams new log lines to it, and keeps a small RAM ring buffer so a connecting client sees recent history.

## Owns
- One TCP listen socket (port 30200 by default; settable via OD entry).
- One accepted client socket (max one connected client; later clients displace earlier).
- A RAM ring buffer (e.g. 8 KB) that stores lines emitted while no client is connected — flushed to the new client on connect.
- Log-level enum (`debug`, `info`, `warn`, `error`).

## Does NOT do
- Persist anything across reboot.
- Buffer per-module — one shared ring buffer.
- Wait for the client. If the client is slow, writes drop the oldest line (overwrite-old).

## Public API
```c
void log_init(void);
void log_tick(void);                                    /* accept, flush ring, service client */

void log_set_level(log_level_t level);
log_level_t log_get_level(void);

void log_printf(log_level_t lvl, const char *fmt, ...);  /* one line per call */
#define LOG_DEBUG(...) log_printf(LOG_LVL_DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  log_printf(LOG_LVL_INFO,  __VA_ARGS__)
#define LOG_WARN(...)  log_printf(LOG_LVL_WARN,  __VA_ARGS__)
#define LOG_ERROR(...) log_printf(LOG_LVL_ERROR, __VA_ARGS__)
```

## Dependencies
- `bsp/net` (TCP socket).
- `bsp/time` (monotonic timestamp prefix per line).

## Acceptance criteria
- `nc <ip> 30200` shows log output from that moment on, plus any buffered backlog.
- Disconnecting the client and reconnecting later resumes; missed lines beyond ring capacity are reported as a "[N lines dropped]" marker.
- A log call from any tick handler never blocks. Output during socket congestion is dropped, not queued forever.
- Log-level filtering happens at the `log_printf` entry point — sub-threshold calls cost only a level comparison.

## Notes
- Line format: `[ms] LEVEL message\n` — e.g. `[01234] INFO  controller 7 selected`.
- `vsnprintf` into a 256-byte stack buffer; truncated lines are marked with a trailing `…`.
- No `%f` — the runtime doesn't link printf-float. Use integer scaling.
- Calls into `log_printf` from inside `bsp/net` itself are forbidden (would recurse).
- The TCP socket counts against the W6100's 8-socket budget alongside controllers, HTTP, UDP poll, and OD UDP.
