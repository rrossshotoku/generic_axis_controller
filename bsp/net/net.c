/*
 * bsp/net — W6100 socket-like API implementation.
 *
 * Wraps the WIZnet ioLibrary (Drivers/w6100) behind the API in net.h.
 * Static socket allocation: callers supply the hardware slot they want
 * (per the architecture's socket map). No runtime pool, no reservations.
 *
 * Reference: working bring-up patterns from
 *   networked_node/microcontroller/uc_camd_interface/Core/Src/wizchip_port.c
 * and Core/Src/app_init.c, deliberately simplified.
 */

#include "net.h"
#include "wizchip_glue.h"

#include "socket.h"             /* Drivers/w6100 */
#include "wizchip_conf.h"       /* Drivers/w6100 */

#include "app/log/log.h"
#include "bsp/time/time.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Module state
 *---------------------------------------------------------------------------*/

static bool s_initialised = false;

/*----------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/

static bool valid_sock(net_sock_t sock)
{
    return sock >= 0 && sock < _WIZCHIP_SOCK_NUM_;
}

static net_tcp_state_t map_sn_sr(uint8_t sr)
{
    switch (sr) {
        case SOCK_CLOSED:      return NET_TCP_CLOSED;
        case SOCK_INIT:        return NET_TCP_INIT;
        case SOCK_LISTEN:      return NET_TCP_LISTEN;
        case SOCK_ESTABLISHED: return NET_TCP_ESTABLISHED;
        case SOCK_CLOSE_WAIT:  return NET_TCP_CLOSE_WAIT;
        default:               return NET_TCP_OTHER;
    }
}

/* (wait_phy_link removed — net_init used to block here for up to 3 s and
 * return false on timeout, which left the chip unconfigured forever if
 * the cable wasn't plugged in at boot. Init now always configures the
 * chip's MAC/IP/sockets; PHY link is read live by net_link_up().) */

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

bool net_init(const uint8_t mac[6],
              const uint8_t ip[4],
              const uint8_t netmask[4],
              const uint8_t gateway[4])
{
    if (s_initialised) return true;

    wizchip_glue_reset_pulse();
    wizchip_glue_register_callbacks();

    /* Confirm we can talk to the chip at all. The W6100 reports
     * 0x6100 in CIDR. */
    uint16_t chip_id = getCIDR();
    if (chip_id != 0x6100) {
        LOG_ERROR("net: bad W6100 chip ID 0x%04X (expected 0x6100)", chip_id);
        return false;
    }
    LOG_INFO("net: W6100 chip ID 0x%04X", chip_id);

    /* Unlock the network registers before changing them. */
    uint8_t syslock = SYS_NET_LOCK;
    ctlwizchip(CW_SYS_UNLOCK, &syslock);

    /* Apply network info BEFORE checking PHY link. MAC / IP / netmask /
     * gateway are register writes that the W6100 accepts whether the
     * cable is plugged in or not — they take effect on the wire as soon
     * as the PHY link comes up later. Earlier code waited up to 3 s
     * here for a link and then returned false on timeout, which is
     * exactly the boot-with-cable-unplugged scenario: the chip stayed
     * unconfigured forever, plugging the cable in afterward didn't
     * recover because net_init was never called again.
     *
     * The ioLibrary expects mutable wiz_NetInfo; we copy into a local
     * because the API isn't const-correct. */
    wiz_NetInfo ni = (wiz_NetInfo){ 0 };
    memcpy(ni.mac, mac,     6);
    memcpy(ni.ip,  ip,      4);
    memcpy(ni.sn,  netmask, 4);
    memcpy(ni.gw,  gateway, 4);
    ni.dhcp = NETINFO_STATIC;
    ctlnetwork(CN_SET_NETINFO, &ni);

    /* 2 KB per socket, both RX and TX — matches the reference. */
    uint8_t memsize[_WIZCHIP_SOCK_NUM_] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    if (wizchip_init(memsize, memsize) != 0) {
        LOG_ERROR("net: wizchip_init failed (socket buffer sizing)");
        return false;
    }

    LOG_INFO("net: ip=%u.%u.%u.%u mask=%u.%u.%u.%u gw=%u.%u.%u.%u",
             ip[0], ip[1], ip[2], ip[3],
             netmask[0], netmask[1], netmask[2], netmask[3],
             gateway[0], gateway[1], gateway[2], gateway[3]);

    /* Informational only — the cable may or may not be plugged in. We
     * don't gate init on this; net_link_up() reports the live state and
     * downstream consumers (log, web, controller_mgr, od) already handle
     * the no-link case gracefully. */
    if (wizphy_getphylink() == PHY_LINK_ON) {
        LOG_INFO("net: PHY link up at boot");
    } else {
        LOG_INFO("net: PHY link down at boot (waiting for cable)");
    }

    s_initialised = true;
    return true;
}

bool net_link_up(void)
{
    if (!s_initialised) return false;
    bool up = (wizphy_getphylink() == PHY_LINK_ON);

    /* Edge-log link transitions. Lets the operator (and led_indicator,
     * via its own polling) see exactly when the cable was plugged in /
     * pulled out. Static prev tracks the last reported state — first
     * call after boot reports a transition from whatever the init log
     * line said, which is harmless. */
    static bool s_prev_link;
    static bool s_prev_valid;
    if (!s_prev_valid || up != s_prev_link) {
        if (s_prev_valid) {
            LOG_INFO("net: PHY link %s", up ? "UP" : "DOWN");
        }
        s_prev_link  = up;
        s_prev_valid = true;
    }
    return up;
}

/*----------------------------------------------------------------------------
 * Open / close
 *---------------------------------------------------------------------------*/

bool net_open(net_sock_t sock, net_proto_t proto, uint16_t local_port, bool do_listen)
{
    if (!s_initialised || !valid_sock(sock)) return false;

    uint8_t mode  = (proto == NET_PROTO_TCP) ? Sn_MR_TCP4 : Sn_MR_UDP4;
    uint8_t flags = 0;

    /* socket() returns the socket number on success, or a negative ioLibrary
     * error code (e.g. SOCKERR_SOCKMODE). It also moves the socket into
     * Sn_MR_INIT and binds local_port. */
    int8_t rc = socket((uint8_t)sock, mode, local_port, flags);
    if (rc != sock) {
        LOG_ERROR("net: socket(%d) open failed rc=%d proto=%d port=%u",
                  (int)sock, (int)rc, (int)proto, (unsigned)local_port);
        return false;
    }

    if (proto == NET_PROTO_TCP && do_listen) {
        /* Parameter is named do_listen, not listen, deliberately — the
         * WIZnet ioLibrary exposes a plain `listen(uint8_t sn)` in
         * socket.h, and naming the local `listen` would shadow it. */
        rc = (int8_t)listen((uint8_t)sock);
        if (rc != SOCK_OK) {
            LOG_ERROR("net: listen(%d) failed rc=%d", (int)sock, (int)rc);
            (void)close((uint8_t)sock);
            return false;
        }
    }

    return true;
}

void net_close(net_sock_t sock)
{
    if (!valid_sock(sock)) return;
    (void)close((uint8_t)sock);
}

/* Graceful TCP shutdown — issues Sn_CR_DISCON (FIN) and returns
 * immediately. The W6100 internally waits for the TX buffer to drain,
 * sends FIN, waits for the peer's FIN-ACK, then transitions through
 * FIN_WAIT/TIME_WAIT to CLOSED. Caller polls net_tcp_state() and
 * reopens once SOCK_CLOSED. The slot is unavailable for re-listen
 * during the FIN exchange (~tens of ms typical, longer if the peer
 * is slow).
 *
 * Use this instead of net_close() when the response data MUST flush
 * to the wire before the socket dies — Sn_CR_CLOSE (immediate hard
 * close) would drop any TX bytes not yet transmitted. This was the
 * fix for the web's "POST returned ok but the body got truncated /
 * next request hung" bug.
 *
 * The ioLibrary's disconnect() spins until SOCK_CLOSED; we deliberately
 * don't use that on the cooperative super-loop — poke Sn_CR ourselves
 * and let the W6100 progress in the background. */
void net_tcp_graceful_close(net_sock_t sock)
{
    if (!valid_sock(sock)) return;
    /* DISCON only meaningful in ESTABLISHED; any other state falls back
     * to a hard close so we don't leak the slot. */
    if (net_tcp_state(sock) != NET_TCP_ESTABLISHED) {
        (void)close((uint8_t)sock);
        return;
    }
    setSn_CR((uint8_t)sock, Sn_CR_DISCON);
    /* Wait for the command-accept ack only (microseconds, NOT the
     * FIN/ACK round-trip). */
    while (getSn_CR((uint8_t)sock)) { /* spin briefly */ }
}

/*----------------------------------------------------------------------------
 * TCP
 *---------------------------------------------------------------------------*/

net_tcp_state_t net_tcp_state(net_sock_t sock)
{
    if (!valid_sock(sock)) return NET_TCP_CLOSED;
    return map_sn_sr(getSn_SR((uint8_t)sock));
}

bool net_tcp_connect(net_sock_t sock, const net_addr_t *peer)
{
    if (!s_initialised || !valid_sock(sock) || peer == NULL) return false;
    if (peer->port == 0) return false;

    /* Address must be non-zero and not broadcast. */
    uint32_t taddr = ((uint32_t)peer->addr[0] << 24)
                   | ((uint32_t)peer->addr[1] << 16)
                   | ((uint32_t)peer->addr[2] <<  8)
                   |  (uint32_t)peer->addr[3];
    if (taddr == 0u || taddr == 0xFFFFFFFFu) return false;

    uint8_t sn = (uint8_t)sock;

    /* Socket must be in INIT state (i.e. opened as TCP, not yet
     * connecting/listening). socket() left it in SOCK_INIT after a
     * successful net_open(NET_PROTO_TCP, port, false). */
    if (getSn_SR(sn) != SOCK_INIT) {
        return false;
    }

    /* The WIZnet ioLibrary's connect() blocks until ESTABLISHED. We don't
     * want that on a cooperative super-loop — instead, poke the registers
     * directly and let the caller poll net_tcp_state(). The W6100 issues
     * SYN as soon as Sn_CR_CONNECT is processed; the socket transitions
     * INIT -> SYNSENT -> ESTABLISHED on its own. */
    uint8_t addr_copy[4] = {peer->addr[0], peer->addr[1], peer->addr[2], peer->addr[3]};
    setSn_DIPR  (sn, addr_copy);
    setSn_DPORTR(sn, peer->port);
    setSn_CR    (sn, Sn_CR_CONNECT);

    /* Wait for the command register to clear (very fast — microseconds).
     * After this, the SYN is on its way out; status will progress in the
     * background. */
    while (getSn_CR(sn)) { /* spin briefly */ }

    return true;
}

bool net_tcp_reopen_listen(net_sock_t sock, uint16_t local_port)
{
    if (!valid_sock(sock)) return false;
    (void)close((uint8_t)sock);
    return net_open(sock, NET_PROTO_TCP, local_port, true);
}

/*----------------------------------------------------------------------------
 * I/O
 *---------------------------------------------------------------------------*/

int32_t net_send(net_sock_t sock, const uint8_t *buf, size_t len)
{
    if (!valid_sock(sock) || !buf || len == 0) return 0;
    /* The ioLibrary send() blocks until either all bytes are queued in the
     * W6100 TX buffer or an error occurs. For TCP it returns the number
     * sent or a negative error. We treat any negative as a hard fault for
     * the caller to handle (typically by closing). */
    int32_t rc = send((uint8_t)sock, (uint8_t *)buf, (uint16_t)len);
    if (rc <= 0) return rc;
    return rc;
}

int32_t net_recv(net_sock_t sock, uint8_t *buf, size_t maxlen)
{
    if (!valid_sock(sock) || !buf || maxlen == 0) return 0;

    /* recv() blocks if nothing is available. Peek the RX byte count first
     * so we can return 0 (no data) without blocking. */
    uint16_t avail = getSn_RX_RSR((uint8_t)sock);
    if (avail == 0) return 0;

    uint16_t want = (avail < maxlen) ? avail : (uint16_t)maxlen;
    return (int32_t)recv((uint8_t)sock, buf, want);
}

int32_t net_sendto(net_sock_t sock, const net_addr_t *peer,
                   const uint8_t *buf, size_t len)
{
    if (!valid_sock(sock) || !peer || !buf || len == 0) return -1;

    /* WIZnet's sendto takes a non-const addr pointer, so copy locally. */
    uint8_t addr[4] = { peer->addr[0], peer->addr[1], peer->addr[2], peer->addr[3] };
    int32_t rc = sendto((uint8_t)sock, (uint8_t *)buf, (uint16_t)len, addr, peer->port);
    return rc;
}

int32_t net_recvfrom(net_sock_t sock, net_addr_t *peer,
                     uint8_t *buf, size_t maxlen)
{
    if (!valid_sock(sock) || !buf || maxlen == 0) return 0;

    /* Non-blocking: only call recvfrom if there's actually data, otherwise
     * the WIZnet recvfrom may busy-wait. UDP RSR is bytes available. */
    uint16_t avail = getSn_RX_RSR((uint8_t)sock);
    if (avail == 0) return 0;

    /* WIZnet UDP frames carry an 8-byte header (peer addr + port + datalen)
     * in the RX buffer in addition to the payload. recvfrom strips it and
     * fills out peer addr+port for us. The payload returned is bounded by
     * the smaller of caller's maxlen and one UDP datagram. */
    uint8_t  raddr[4] = {0};
    uint16_t rport    = 0;
    uint16_t want     = (maxlen > UINT16_MAX) ? UINT16_MAX : (uint16_t)maxlen;
    int32_t  n        = recvfrom((uint8_t)sock, buf, want, raddr, &rport);
    if (n > 0 && peer) {
        peer->addr[0] = raddr[0]; peer->addr[1] = raddr[1];
        peer->addr[2] = raddr[2]; peer->addr[3] = raddr[3];
        peer->port    = rport;
    }
    return n;
}
