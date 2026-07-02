# bsp/net

## Purpose
Socket-like API on top of the WIZnet W6100. Hides the chip-specific calls behind a uniform interface that app modules use.

## Owns
- Init of SPI2 + chip-select GPIO for the W6100.
- Setting the W6100's MAC and IP from `config`.
- Allocation of the W6100's 8 hardware sockets (compile-time map):

  | Socket | Purpose | Layer |
  |---:|---|---|
  | 0 | UDP — CAMERAD poll listen, port 30002 | `controller_mgr` |
  | 1 | TCP — CAMERAD listen, port 30001 (matches SW050 LISTENPORT1) | `controller_mgr` |
  | 2 | TCP — controller A outbound | `controller_mgr` |
  | 3 | TCP — controller B outbound (**RESERVED**, unallocated in code today but must stay reserved for Phase B 1×S + 1×T deployment — do NOT assign to anything else, e.g. a future bootloader; see `Documentation/dual_bootloader_design.md` §5.0) | `controller_mgr` (future) |
  | 4 | UDP — OD access, port **5000** (per `Interface/NETWORK_UDP_SPEC.md`) | `od` |
  | 5 | UDP — OD telemetry stream, port **5001** | `od` |
  | 6 | TCP — log socket, port 30200 | `log` |
  | 7 | TCP — HTTP listen, port 80 | `web` |

  Per-controller TCP slots: **2** (covers a typical 1×S + 1×T deployment).

## Does NOT do
- Hold any application protocol state.
- Talk to the motor MCU — that's `bsp/motor_spi`.

## Public API
```c
void net_init(void);

typedef int net_sock_t;            /* opaque handle */

net_sock_t net_udp_open (uint16_t local_port);
net_sock_t net_tcp_listen(uint16_t local_port, uint8_t backlog);
net_sock_t net_tcp_connect(const net_addr_t *peer);   /* non-blocking */
net_sock_t net_tcp_accept (net_sock_t listen_sock, net_addr_t *peer_out);

int32_t    net_send (net_sock_t sock, const uint8_t *buf, size_t len);
int32_t    net_recv (net_sock_t sock,       uint8_t *buf, size_t maxlen, net_addr_t *peer_out);
                       /* peer_out used for UDP; ignored for TCP */
void       net_close(net_sock_t sock);

bool       net_is_connected(net_sock_t sock);
```

All calls are **non-blocking** — return `0` for "no progress", `>0` for bytes done, `<0` for error.

## Dependencies
- `Drivers/w6100` (WIZnet ioLibrary).
- `Drivers/STM32G4xx_HAL_Driver` (SPI2 HAL).

## Acceptance criteria
- `net_init` reads MAC/IP from `config` and applies them; verifiable by ARP request from a peer.
- Eight simultaneous sockets can be open without errors.
- `net_recv` on a TCP socket returns `0` immediately if no data is available (does not block).
- Closing a socket and re-opening on the same hardware slot works without a stale state.

## Notes
- W6100 ioLibrary uses callback-registered SPI functions. `net_init` registers callbacks pointing to local static functions that wrap HAL SPI2.
- Chip-select must be exclusive — no other peripheral may share SPI2. (If anything else gets put on SPI2 later, this module owns the lock.)
- Interrupt-driven RX from the W6100 is desirable but not required for Phase 1; polled is fine to start.
