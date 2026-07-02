/*
 * bsp/net — socket-like API over the W6100.
 *
 * See bsp/net/README.md for the contract and the W6100 hardware-socket map.
 *
 * Phase 0b implements the subset needed by the log module:
 *   net_init, net_link_up, net_open (TCP listen),
 *   net_tcp_state, net_send, net_recv, net_close,
 *   net_tcp_reopen_listen.
 *
 * UDP, outbound TCP connect, and IPv6 are scoped in here but stubbed
 * until Phase 1 (controller_mgr) needs them.
 */

#ifndef BSP_NET_H
#define BSP_NET_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int8_t net_sock_t;          /* 0..7 hardware socket slot; <0 invalid */
#define NET_INVALID_SOCKET ((net_sock_t)-1)

typedef enum {
    NET_PROTO_TCP,
    NET_PROTO_UDP,
} net_proto_t;

typedef struct {
    uint8_t  addr[4];
    uint16_t port;
} net_addr_t;

typedef enum {
    NET_TCP_CLOSED,
    NET_TCP_INIT,
    NET_TCP_LISTEN,
    NET_TCP_ESTABLISHED,
    NET_TCP_CLOSE_WAIT,
    NET_TCP_OTHER,
} net_tcp_state_t;

/*----------------------------------------------------------------------------
 * Lifecycle
 *
 * net_init performs: hardware reset, callback registration, chip-ID check,
 * PHY-link wait (bounded), system unlock, network info apply, socket
 * buffer init. Logs progress via app/log. Returns true on success.
 *---------------------------------------------------------------------------*/

bool net_init(const uint8_t mac[6],
              const uint8_t ip[4],
              const uint8_t netmask[4],
              const uint8_t gateway[4]);

/* True once net_init has succeeded and the PHY reports link up. Cheap
 * to call; can be polled by upper layers each tick. */
bool net_link_up(void);

/*----------------------------------------------------------------------------
 * Socket open / close
 *
 * Hardware socket numbering is allocated by the caller per the static
 * map in Documentation/architecture.md §10.1 / bsp/net/README.md.
 *
 *   - TCP listen socket: net_open(sock, NET_PROTO_TCP, port, listen=true).
 *   - TCP outbound:      net_open(sock, NET_PROTO_TCP, 0,    listen=false),
 *                        then net_tcp_connect(sock, peer). (Phase 1.)
 *   - UDP:               net_open(sock, NET_PROTO_UDP, port, listen=false).
 *
 * Returns true on success.
 *---------------------------------------------------------------------------*/

bool net_open (net_sock_t sock, net_proto_t proto, uint16_t local_port, bool do_listen);
void net_close(net_sock_t sock);
/* Graceful TCP shutdown — sends FIN and returns immediately. Use instead
 * of net_close() for sockets where pending TX bytes MUST flush to the
 * wire (e.g. HTTP responses). The W6100 drains TX + completes FIN/ACK
 * in the background; caller polls net_tcp_state() and reopens once
 * SOCK_CLOSED. No-op on non-TCP / non-ESTABLISHED sockets (those fall
 * back to a hard close). See net.c for the rationale. */
void net_tcp_graceful_close(net_sock_t sock);

/*----------------------------------------------------------------------------
 * TCP
 *---------------------------------------------------------------------------*/

net_tcp_state_t net_tcp_state(net_sock_t sock);

/* Phase 1: outbound TCP connect. Stub for Phase 0b — returns false. */
bool net_tcp_connect(net_sock_t sock, const net_addr_t *peer);

/* Close an ESTABLISHED TCP connection cleanly and re-open the same hardware
 * slot in LISTEN on the same port. Used by the log module when the client
 * disconnects, so the next nc can connect to the same socket. */
bool net_tcp_reopen_listen(net_sock_t sock, uint16_t local_port);

/*----------------------------------------------------------------------------
 * I/O
 *
 * All operations are non-blocking from the caller's perspective.
 * Return values:
 *    > 0  bytes transferred
 *      0  nothing to do (no RX data; or no TX room without going partial)
 *    < 0  error (peer closed, hardware fault, etc.)
 *---------------------------------------------------------------------------*/

int32_t net_send(net_sock_t sock, const uint8_t *buf, size_t len);
int32_t net_recv(net_sock_t sock,       uint8_t *buf, size_t maxlen);

/* UDP: stubbed in Phase 0b. */
int32_t net_sendto  (net_sock_t sock, const net_addr_t *peer,
                     const uint8_t *buf, size_t len);
int32_t net_recvfrom(net_sock_t sock,       net_addr_t *peer,
                           uint8_t *buf, size_t maxlen);

#ifdef __cplusplus
}
#endif

#endif /* BSP_NET_H */
