# app/web

## Purpose
HTTP server on the W6100. Serves a small set of pages (network config, motor limits, status) and a JSON API for OD access.

## Owns
- One TCP listen socket on port 80.
- A single in-progress request buffer (one request at a time — no parallelism needed at this scope).
- Router table mapping `(method, path)` to handler functions.
- HTTP Basic auth check against the user/pass in `config`.
- Static pages embedded as C string literals.

## Does NOT do
- Serve arbitrary files from flash — no virtual filesystem.
- Hold any application state — reads/writes through `cmc_state`, `config`, `od`.
- Support TLS. (LAN device behind a router; auth is over plain HTTP. Documented constraint.)

## Public API
```c
void web_init(void);
void web_tick(void);
```

Internal: a handlers table — `handlers/status.c`, `handlers/network.c`, `handlers/limits.c`, `handlers/od.c`. Each owns its route(s).

## Dependencies
- `cmc_state` (reads for status pages)
- `config`    (reads & writes for network, limits, auth)
- `od`        (for `/api/od/...` endpoints)
- `bsp/net`   (TCP socket)

## Acceptance criteria
- `GET /`            → status page, 200, contains current IP, selected controller, motor state.
- `GET /network`     → form prefilled with current IP/mask/gw/node-id.
- `POST /network`    → with valid Basic auth, persists and applies on next boot.
- `GET /limits`      → form with current per-axis soft limits.
- `POST /limits`     → with valid Basic auth, persists; `motor_ctrl` reads new values immediately.
- `GET /api/od/<idx>/<sub>` → JSON `{"value": ..., "type": ...}`.
- `POST /api/od/<idx>/<sub>` → with valid auth, writes the OD entry.
- Missing/wrong Basic auth → `401 WWW-Authenticate: Basic realm="cmc"`.
- Total page load + auth round trip < 1 s on a quiet LAN.

## Acceptance non-criteria
- High request rate. One request at a time; client must wait for the previous response.
- Streaming responses. Each handler builds the full response in a static buffer.

## Notes
- HTTP/1.0 only. `Connection: close` after every response. Simpler than keep-alive on a single-socket model.
- Maximum request size: 1 KB (header + body). Larger requests get `413`.
- Auth credentials are stored hashed (SHA-256 + per-device salt) in `config`. The factory default is `admin` / `cmc` and the device refuses to enter production state until the password has been changed (flag stored in config).
