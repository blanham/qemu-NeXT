/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/net.h"
#include "net/slirp-bootp.h"
#include "net/slirp-il.h"
#include "net/slirp-plan9.h"
#include "net/slirp-rpc-internal.h"
#include "net/slirp-stream.h"
#include "net/slirp-udp.h"
#include "net/slirp.h"
#include "qapi/error.h"
#include <libslirp.h>

static NetClientState *test_netdev;
#define SENT_PACKET_CAPACITY (128 * 1024)
static uint8_t sent_packet[SENT_PACKET_CAPACITY];
static size_t sent_packet_len;
static unsigned sent_packet_count;
static GByteArray *tcp_rpc_capture;
static uint16_t tcp_rpc_capture_source_port;
static uint16_t tcp_rpc_capture_port;
static uint32_t tcp_rpc_capture_next_seq;

#define ETHERNET_HEADER_LEN 14
#define IPV4_HEADER_LEN 20
#define UDP_HEADER_LEN 8
#define TCP_HEADER_LEN 20
#define TCP_GUEST_ADDRESS 0x0a00020fU
#define TCP_SERVICE_ADDRESS 0x0a000202U
#define TCP_SERVICE_PORT 17008

NetClientState *qemu_new_net_client(NetClientInfo *info,
                                    NetClientState *peer,
                                    const char *model, const char *name)
{
    NetClientState *nc = g_malloc0(info->size);

    nc->info = info;
    nc->model = g_strdup(model);
    nc->name = g_strdup(name);
    test_netdev = nc;
    return nc;
}

void qemu_set_info_str(NetClientState *nc, const char *fmt, ...)
{
}

NetClientState *qemu_find_netdev(const char *id)
{
    if (test_netdev && !strcmp(test_netdev->name, id)) {
        return test_netdev;
    }
    return NULL;
}

static void tracked_slirp_cleanup(Slirp *slirp);
#define slirp_cleanup tracked_slirp_cleanup
#include "../../net/slirp.c"
#undef slirp_cleanup

ssize_t qemu_send_packet(NetClientState *nc, const uint8_t *buf, int size)
{
    const uint8_t *ip;
    const uint8_t *tcp;
    size_t ip_len;
    size_t ip_total;
    size_t tcp_len;
    size_t payload_len;
    size_t overlap;
    const uint8_t *payload;
    uint32_t sequence;

    g_assert_cmpuint(size, <=, sizeof(sent_packet));
    memcpy(sent_packet, buf, size);
    sent_packet_len = size;
    sent_packet_count++;
    if (tcp_rpc_capture && size >= ETHERNET_HEADER_LEN + IPV4_HEADER_LEN +
        TCP_HEADER_LEN) {
        ip = buf + ETHERNET_HEADER_LEN;
        ip_len = (size_t)(ip[0] & 0x0f) * 4;
        if (ip_len < IPV4_HEADER_LEN ||
            size < ETHERNET_HEADER_LEN + ip_len + TCP_HEADER_LEN ||
            ip[9] != 6) {
            return size;
        }
        tcp = ip + ip_len;
        tcp_len = (size_t)(tcp[12] >> 4) * 4;
        ip_total = ((size_t)ip[2] << 8) | ip[3];
        if (tcp_len < TCP_HEADER_LEN ||
            size < ETHERNET_HEADER_LEN + ip_len + tcp_len ||
            ip_total < ip_len + tcp_len ||
            size < ETHERNET_HEADER_LEN + ip_total ||
            ((uint16_t)tcp[0] << 8 | tcp[1]) != tcp_rpc_capture_source_port ||
            ((uint16_t)tcp[2] << 8 | tcp[3]) != tcp_rpc_capture_port) {
            return size;
        }
        payload_len = ip_total - ip_len - tcp_len;
        if (!payload_len) {
            return size;
        }
        sequence = ((uint32_t)tcp[4] << 24) | ((uint32_t)tcp[5] << 16) |
                   ((uint32_t)tcp[6] << 8) | tcp[7];
        payload = tcp + tcp_len;
        if (sequence < tcp_rpc_capture_next_seq) {
            overlap = tcp_rpc_capture_next_seq - sequence;
            if (overlap >= payload_len) {
                return size;
            }
            payload_len -= overlap;
            sequence += overlap;
            payload += overlap;
        }
        g_assert_cmpuint(sequence, ==, tcp_rpc_capture_next_seq);
        g_byte_array_append(tcp_rpc_capture, payload, payload_len);
        tcp_rpc_capture_next_seq += payload_len;
    }
    return size;
}

bool eth_pad_short_frame(uint8_t *padded_pkt, size_t *padded_buflen,
                         const void *pkt, size_t pkt_size)
{
    return false;
}

void unregister_savevm(VMStateIf *obj, const char *idstr, void *opaque)
{
}

void qemu_remove_exit_notifier(Notifier *notify)
{
}

size_t qemu_get_buffer(QEMUFile *f, uint8_t *buf, size_t size)
{
    return 0;
}

void qemu_put_buffer(QEMUFile *f, const uint8_t *buf, size_t size)
{
}

int qemu_file_get_error(QEMUFile *f)
{
    return 0;
}

void qemu_chr_fe_deinit(CharFrontend *c, bool del)
{
}

#ifdef CONFIG_SLIRP_IL
typedef struct TrackingState {
    unsigned listener_removes;
    bool cleanup_called;
    bool assert_cleanup;
} TrackingState;

static TrackingState tracking;

static int tracking_listen(void *opaque, struct in_addr addr, uint16_t port,
                           const QemuSlirpILBackendCallbacks *callbacks,
                           void *callbacks_opaque, void **backend_listener)
{
    return slirp_il_backend_ops.listen(opaque, addr, port, callbacks,
                                       callbacks_opaque, backend_listener);
}

static void tracking_listener_remove(void *opaque, void *backend_listener)
{
    tracking.listener_removes++;
    slirp_il_backend_ops.listener_remove(opaque, backend_listener);
}

static const QemuSlirpILBackendOps tracking_ops = {
    .listen = tracking_listen,
    .listener_remove = tracking_listener_remove,
    .send_record = slirp_il_backend_send_record,
    .connection_close = slirp_il_backend_connection_close,
};
#endif

static void tracked_slirp_cleanup(Slirp *slirp)
{
#ifdef CONFIG_SLIRP_IL
    if (tracking.assert_cleanup) {
        g_assert_cmpuint(tracking.listener_removes, ==, 2);
    }
    tracking.cleanup_called = true;
#endif
    slirp_cleanup(slirp);
}

static struct in_addr test_addr(void)
{
    return (struct in_addr) { htonl(0x0a000204) };
}

#define BOOTP_FIXED_LEN 236
#define BOOTP_VENDOR_LEN 64
#define BOOTP_OFFSET (ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + UDP_HEADER_LEN)
#define BOOTP_VENDOR_OFFSET 236
#define BOOTP_REQUEST_LEN \
    (BOOTP_OFFSET + BOOTP_FIXED_LEN + BOOTP_VENDOR_LEN)

#if defined(CONFIG_SLIRP_PLAN9_BOOTP) || defined(CONFIG_SLIRP_UDP_SERVICE) || \
    defined(CONFIG_SLIRP_BOOTP_ROOT)
static void store_be16(uint8_t *p, uint16_t value)
{
    p[0] = value >> 8;
    p[1] = value;
}

static uint16_t ipv4_checksum(const uint8_t *header, size_t length)
{
    uint32_t sum = 0;
    size_t i;

    for (i = 0; i < length; i += 2) {
        sum += ((uint16_t)header[i] << 8) | header[i + 1];
    }
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}
#endif

#ifdef CONFIG_SLIRP_PLAN9_BOOTP
static void send_plan9_bootp_request(SlirpState *s, const char *expected)
{
    static const uint8_t mac[] = { 0x00, 0x00, 0x0f, 0x12, 0x34, 0x56 };
    uint8_t request[BOOTP_REQUEST_LEN] = { 0 };
    uint8_t *ip_header = request + ETHERNET_HEADER_LEN;
    uint8_t *udp = ip_header + IPV4_HEADER_LEN;
    uint8_t *bootp = request + BOOTP_OFFSET;
    uint8_t *vendor = bootp + BOOTP_VENDOR_OFFSET;

    memset(request, 0xff, 6);
    memcpy(request + 6, mac, sizeof(mac));
    store_be16(request + 12, 0x0800);
    ip_header[0] = 0x45;
    store_be16(ip_header + 2, sizeof(request) - ETHERNET_HEADER_LEN);
    ip_header[8] = 64;
    ip_header[9] = 17;
    memset(ip_header + 16, 0xff, sizeof(struct in_addr));
    store_be16(ip_header + 10, ipv4_checksum(ip_header, IPV4_HEADER_LEN));
    store_be16(udp, 68);
    store_be16(udp + 2, 67);
    store_be16(udp + 4, sizeof(request) - ETHERNET_HEADER_LEN -
                             IPV4_HEADER_LEN);
    bootp[0] = 1;
    bootp[1] = 1;
    bootp[2] = 6;
    memcpy(bootp + 28, mac, sizeof(mac));
    memcpy(vendor, "p9  ", 4);

    sent_packet_count = 0;
    sent_packet_len = 0;
    s->nc.info->receive(&s->nc, request, sizeof(request));
    g_assert_cmpuint(sent_packet_count, ==, 1);
    g_assert_cmpuint(sent_packet_len, >=,
                     BOOTP_OFFSET + BOOTP_VENDOR_OFFSET + strlen(expected));
    g_assert_cmpmem(sent_packet + BOOTP_OFFSET + BOOTP_VENDOR_OFFSET,
                    strlen(expected), expected, strlen(expected));
}
#endif

typedef struct UdpState {
    QemuSlirpUdpListener *listener;
    struct sockaddr_in peer;
    uint8_t data[32];
    size_t len;
    unsigned calls;
    int send_result;
} UdpState;

typedef struct StreamConnectionState {
    QemuSlirpStream *stream;
    struct sockaddr_in peer;
    uint16_t source_port;
    uint32_t guest_next_seq;
    uint32_t service_next_seq;
    uint8_t data[128];
    size_t len;
    unsigned received;
    unsigned ready;
    unsigned closed;
} StreamConnectionState;

typedef struct StreamState {
    StreamConnectionState connections[3];
    unsigned nconnections;
} StreamState;

static StreamConnectionState *stream_connection(StreamState *state,
                                                QemuSlirpStream *stream)
{
    unsigned i;

    for (i = 0; i < state->nconnections; i++) {
        if (state->connections[i].stream == stream) {
            return &state->connections[i];
        }
    }
    return NULL;
}

static void stream_connected(QemuSlirpStream *stream,
                             const struct sockaddr_in *peer, void *opaque)
{
    StreamState *state = opaque;
    StreamConnectionState *connection;

    if (!state) {
        return;
    }
    g_assert_cmpuint(state->nconnections, <,
                     G_N_ELEMENTS(state->connections));
    connection = &state->connections[state->nconnections++];
    connection->stream = stream;
    connection->peer = *peer;
    connection->source_port = ntohs(peer->sin_port);
}

static void stream_receive(QemuSlirpStream *stream, const uint8_t *data,
                           size_t len, void *opaque)
{
    StreamState *state = opaque;
    StreamConnectionState *connection;

    if (!state) {
        return;
    }
    connection = stream_connection(state, stream);
    g_assert_nonnull(connection);
    g_assert_cmpuint(len, <=, sizeof(connection->data));
    memcpy(connection->data, data, len);
    connection->len = len;
    connection->received++;
}

static void stream_can_send(QemuSlirpStream *stream, void *opaque)
{
    StreamState *state = opaque;
    StreamConnectionState *connection;

    if (!state) {
        return;
    }
    connection = stream_connection(state, stream);
    g_assert_nonnull(connection);
    connection->ready++;
}

static void stream_closed(QemuSlirpStream *stream, void *opaque)
{
    StreamState *state = opaque;
    StreamConnectionState *connection;

    if (!state) {
        return;
    }
    connection = stream_connection(state, stream);
    g_assert_nonnull(connection);
    connection->closed++;
    connection->stream = NULL;
}

static const QemuSlirpStreamOps stream_ops = {
    .connected = stream_connected,
    .receive = stream_receive,
    .can_send = stream_can_send,
    .closed = stream_closed,
};

#ifdef CONFIG_SLIRP_TCP_SERVICE
static void tcp_store_be16(uint8_t *p, uint16_t value)
{
    p[0] = value >> 8;
    p[1] = value;
}

static void tcp_store_be32(uint8_t *p, uint32_t value)
{
    p[0] = value >> 24;
    p[1] = value >> 16;
    p[2] = value >> 8;
    p[3] = value;
}

static uint32_t tcp_load_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static uint16_t tcp_checksum(const uint8_t *data, size_t length)
{
    uint32_t sum = 0;
    size_t i;

    for (i = 0; i + 1 < length; i += 2) {
        sum += ((uint16_t)data[i] << 8) | data[i + 1];
    }
    if (i < length) {
        sum += (uint16_t)data[i] << 8;
    }
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}

static uint16_t tcp_segment_checksum(const uint8_t *ip, const uint8_t *tcp,
                                     size_t tcp_len)
{
    uint32_t sum = 0;
    size_t i;

    for (i = 12; i < 20; i += 2) {
        sum += ((uint16_t)ip[i] << 8) | ip[i + 1];
    }
    sum += 6;
    sum += tcp_len;
    for (i = 0; i + 1 < tcp_len; i += 2) {
        sum += ((uint16_t)tcp[i] << 8) | tcp[i + 1];
    }
    if (i < tcp_len) {
        sum += (uint16_t)tcp[i] << 8;
    }
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}

static void build_tcp_packet(uint8_t *frame, struct in_addr source,
                             uint16_t source_port, struct in_addr destination,
                             uint16_t destination_port, uint32_t seq,
                             uint32_t ack, uint8_t flags,
                             const uint8_t *payload, size_t payload_len)
{
    uint8_t *ip = frame + ETHERNET_HEADER_LEN;
    uint8_t *tcp = ip + IPV4_HEADER_LEN;
    size_t tcp_len = TCP_HEADER_LEN + payload_len;
    size_t frame_len = ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + tcp_len;

    g_assert_cmpuint(frame_len, <=, SENT_PACKET_CAPACITY);
    memset(frame, 0, frame_len);
    frame[12] = 0x08;
    frame[13] = 0x00;
    ip[0] = 0x45;
    tcp_store_be16(ip + 2, IPV4_HEADER_LEN + tcp_len);
    ip[8] = 64;
    ip[9] = 6;
    memcpy(ip + 12, &source.s_addr, sizeof(source.s_addr));
    memcpy(ip + 16, &destination.s_addr, sizeof(destination.s_addr));
    tcp_store_be16(ip + 10, tcp_checksum(ip, IPV4_HEADER_LEN));
    tcp_store_be16(tcp, source_port);
    tcp_store_be16(tcp + 2, destination_port);
    tcp_store_be32(tcp + 4, seq);
    tcp_store_be32(tcp + 8, ack);
    tcp[12] = 5 << 4;
    tcp[13] = flags;
    tcp_store_be16(tcp + 14, 65535);
    if (payload_len) {
        memcpy(tcp + TCP_HEADER_LEN, payload, payload_len);
    }
    tcp_store_be16(tcp + 16, tcp_segment_checksum(ip, tcp, tcp_len));
}

static uint32_t tcp_frame_seq(void)
{
    return tcp_load_be32(sent_packet + ETHERNET_HEADER_LEN +
                         IPV4_HEADER_LEN + 4);
}

static uint8_t tcp_frame_flags(void)
{
    return sent_packet[ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + 13];
}

static size_t tcp_frame_payload_len(void)
{
    const uint8_t *ip = sent_packet + ETHERNET_HEADER_LEN;
    const uint8_t *tcp = ip + IPV4_HEADER_LEN;

    return ((size_t)ip[2] << 8 | ip[3]) - IPV4_HEADER_LEN -
           ((size_t)(tcp[12] >> 4) * 4);
}

static const uint8_t *tcp_frame_payload(void)
{
    return sent_packet + ETHERNET_HEADER_LEN + IPV4_HEADER_LEN +
           TCP_HEADER_LEN;
}

static void prime_tcp_guest_arp(SlirpState *s)
{
    static const uint8_t guest_mac[] = {
        0x52, 0x55, 0x0a, 0x00, 0x02, 0x0f,
    };
    uint8_t frame[ETHERNET_HEADER_LEN + 28] = { 0 };
    uint8_t *arp = frame + ETHERNET_HEADER_LEN;

    memset(frame, 0xff, 6);
    memcpy(frame + 6, guest_mac, sizeof(guest_mac));
    tcp_store_be16(frame + 12, 0x0806);
    tcp_store_be16(arp, 1);
    tcp_store_be16(arp + 2, 0x0800);
    arp[4] = 6;
    arp[5] = 4;
    tcp_store_be16(arp + 6, 1);
    memcpy(arp + 8, guest_mac, sizeof(guest_mac));
    tcp_store_be32(arp + 14, TCP_GUEST_ADDRESS);
    tcp_store_be32(arp + 24, TCP_SERVICE_ADDRESS);
    slirp_input(s->slirp, frame, sizeof(frame));
}

static uint32_t establish_tcp_guest_connection(SlirpState *s,
                                               uint16_t source_port,
                                               uint16_t destination_port)
{
    const struct in_addr guest = { htonl(TCP_GUEST_ADDRESS) };
    const struct in_addr service = { htonl(TCP_SERVICE_ADDRESS) };
    uint8_t packet[SENT_PACKET_CAPACITY];
    uint32_t service_seq;

    prime_tcp_guest_arp(s);
    sent_packet_count = 0;
    sent_packet_len = 0;
    build_tcp_packet(packet, guest, source_port, service, destination_port,
                     1000, 0, 0x02, NULL, 0);
    s->nc.info->receive(&s->nc, packet,
                        ETHERNET_HEADER_LEN + IPV4_HEADER_LEN +
                        TCP_HEADER_LEN);
    g_assert_cmpuint(sent_packet_count, >, 0);
    g_assert_cmpuint(tcp_frame_flags(), ==, 0x12);
    service_seq = tcp_frame_seq();

    build_tcp_packet(packet, guest, source_port, service, destination_port,
                     1001, service_seq + 1, 0x10, NULL, 0);
    s->nc.info->receive(&s->nc, packet,
                        ETHERNET_HEADER_LEN + IPV4_HEADER_LEN +
                        TCP_HEADER_LEN);
    return service_seq + 1;
}

static StreamConnectionState *establish_tcp_stream_connection(
    SlirpState *s, StreamState *state, uint16_t source_port)
{
    StreamConnectionState *connection;
    unsigned connected_before = state->nconnections;

    uint32_t service_next = establish_tcp_guest_connection(
        s, source_port, TCP_SERVICE_PORT);

    g_assert_cmpuint(state->nconnections, ==, connected_before + 1);
    connection = &state->connections[connected_before];
    connection->guest_next_seq = 1001;
    connection->service_next_seq = service_next;
    return connection;
}

static void send_tcp_guest_segment_to(SlirpState *s,
                                      StreamConnectionState *connection,
                                      uint16_t destination_port, uint8_t flags,
                                      const uint8_t *payload,
                                      size_t payload_len)
{
    const struct in_addr guest = { htonl(TCP_GUEST_ADDRESS) };
    const struct in_addr service = { htonl(TCP_SERVICE_ADDRESS) };
    uint8_t packet[SENT_PACKET_CAPACITY];

    build_tcp_packet(packet, guest, connection->source_port, service,
                     destination_port, connection->guest_next_seq,
                     connection->service_next_seq, flags, payload,
                     payload_len);
    s->nc.info->receive(&s->nc, packet,
                        ETHERNET_HEADER_LEN + IPV4_HEADER_LEN +
                        TCP_HEADER_LEN + payload_len);
    connection->guest_next_seq += payload_len;
    if (flags & 0x01) {
        connection->guest_next_seq++;
    }
}

static void send_tcp_guest_segment(SlirpState *s,
                                   StreamConnectionState *connection,
                                   uint8_t flags, const uint8_t *payload,
                                   size_t payload_len)
{
    send_tcp_guest_segment_to(s, connection, TCP_SERVICE_PORT, flags, payload,
                              payload_len);
}

static void send_tcp_guest_ack_to(SlirpState *s,
                                  StreamConnectionState *connection,
                                  uint16_t destination_port, uint32_t ack)
{
    const struct in_addr guest = { htonl(TCP_GUEST_ADDRESS) };
    const struct in_addr service = { htonl(TCP_SERVICE_ADDRESS) };
    uint8_t packet[SENT_PACKET_CAPACITY];

    build_tcp_packet(packet, guest, connection->source_port, service,
                     destination_port, connection->guest_next_seq, ack, 0x10,
                     NULL, 0);
    s->nc.info->receive(&s->nc, packet,
                        ETHERNET_HEADER_LEN + IPV4_HEADER_LEN +
                        TCP_HEADER_LEN);
}

static void send_tcp_guest_ack(SlirpState *s,
                               StreamConnectionState *connection,
                               uint32_t ack)
{
    send_tcp_guest_ack_to(s, connection, TCP_SERVICE_PORT, ack);
}
#endif

static void udp_datagram(QemuSlirpUdpListener *listener,
                         const struct sockaddr_in *peer,
                         const uint8_t *data, size_t len, void *opaque)
{
    UdpState *state = opaque;

    state->calls++;
    state->peer = *peer;
    g_assert_cmpuint(len, <=, sizeof(state->data));
    memcpy(state->data, data, len);
    state->len = len;
    state->send_result = qemu_slirp_udp_send(listener, peer, data, len);
}

static const QemuSlirpUdpListenerOps udp_listener_ops = {
    .datagram = udp_datagram,
};

#ifdef CONFIG_SLIRP_UDP_SERVICE
static void learn_client_arp(SlirpState *s)
{
    static const uint8_t mac[] = { 0x00, 0x00, 0x0f, 0x12, 0x34, 0x56 };
    uint8_t packet[ETHERNET_HEADER_LEN + 28] = {0};
    uint8_t *arp = packet + ETHERNET_HEADER_LEN;

    memset(packet, 0xff, 6);
    memcpy(packet + 6, mac, sizeof(mac));
    store_be16(packet + 12, 0x0806);
    store_be16(arp, 1);
    store_be16(arp + 2, 0x0800);
    arp[4] = 6;
    arp[5] = 4;
    store_be16(arp + 6, 1);
    memcpy(arp + 8, mac, sizeof(mac));
    arp[14] = 10;
    arp[15] = 0;
    arp[16] = 2;
    arp[17] = 15;
    memcpy(arp + 18, mac, sizeof(mac));
    memcpy(arp + 24, arp + 14, 4);
    s->nc.info->receive(&s->nc, packet, sizeof(packet));
    sent_packet_count = 0;
    sent_packet_len = 0;
}

static void send_udp_request_to(SlirpState *s, uint32_t destination,
                                uint16_t port, const uint8_t *payload,
                                size_t payload_len)
{
    static const uint8_t mac[] = { 0x00, 0x00, 0x0f, 0x12, 0x34, 0x56 };
    bool directed_broadcast = destination == 0x0a0002ffU;
    uint8_t packet[SENT_PACKET_CAPACITY] = {0};
    uint8_t *ip_header = packet + ETHERNET_HEADER_LEN;
    uint8_t *udp = ip_header + IPV4_HEADER_LEN;
    size_t length = ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + UDP_HEADER_LEN +
                    payload_len;

    g_assert_cmpuint(payload_len, <=,
                     SENT_PACKET_CAPACITY - ETHERNET_HEADER_LEN -
                     IPV4_HEADER_LEN - UDP_HEADER_LEN);
    if (directed_broadcast) {
        memset(packet, 0xff, 6);
    } else {
        packet[0] = 0x52;
        packet[1] = 0x55;
        packet[2] = 10;
        packet[3] = 0;
        packet[4] = 2;
        packet[5] = 2;
    }
    memcpy(packet + 6, mac, sizeof(mac));
    store_be16(packet + 12, 0x0800);
    ip_header[0] = 0x45;
    store_be16(ip_header + 2, length - ETHERNET_HEADER_LEN);
    ip_header[8] = 64;
    ip_header[9] = 17;
    ip_header[12] = 10;
    ip_header[13] = 0;
    ip_header[14] = 2;
    ip_header[15] = 15;
    ip_header[16] = destination >> 24;
    ip_header[17] = destination >> 16;
    ip_header[18] = destination >> 8;
    ip_header[19] = destination;
    store_be16(ip_header + 10,
               ipv4_checksum(ip_header, IPV4_HEADER_LEN));
    store_be16(udp, 49152);
    store_be16(udp + 2, port);
    store_be16(udp + 4, UDP_HEADER_LEN + payload_len);
    memcpy(udp + UDP_HEADER_LEN, payload, payload_len);

    s->nc.info->receive(&s->nc, packet, length);
}

static void send_udp_request(SlirpState *s, uint16_t port,
                             const uint8_t *payload, size_t payload_len)
{
    send_udp_request_to(s, 0x0a000202U, port, payload, payload_len);
}
#endif

#ifdef CONFIG_SLIRP_UDP_SERVICE
static uint16_t load_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}
#endif

#ifdef CONFIG_SLIRP_BOOTP_ROOT
static uint32_t load_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}
#endif

#ifdef CONFIG_SLIRP_BOOTP_ROOT
static void send_root_bootp_request(SlirpState *s)
{
    static const uint8_t mac[] = { 0x00, 0x00, 0x0f, 0x12, 0x34, 0x56 };
    uint8_t request[BOOTP_REQUEST_LEN] = {0};
    uint8_t *ip_header = request + ETHERNET_HEADER_LEN;
    uint8_t *udp = ip_header + IPV4_HEADER_LEN;
    uint8_t *bootp = request + BOOTP_OFFSET;
    uint8_t *vendor = bootp + BOOTP_VENDOR_OFFSET;
    const uint8_t *reply_bootp;
    const uint8_t *option;

    memset(request, 0xff, 6);
    memcpy(request + 6, mac, sizeof(mac));
    store_be16(request + 12, 0x0800);
    ip_header[0] = 0x45;
    store_be16(ip_header + 2, sizeof(request) - ETHERNET_HEADER_LEN);
    ip_header[8] = 64;
    ip_header[9] = 17;
    memset(ip_header + 16, 0xff, sizeof(struct in_addr));
    store_be16(ip_header + 10, ipv4_checksum(ip_header, IPV4_HEADER_LEN));
    store_be16(udp, 68);
    store_be16(udp + 2, 67);
    store_be16(udp + 4, sizeof(request) - ETHERNET_HEADER_LEN -
                             IPV4_HEADER_LEN);
    bootp[0] = 1;
    bootp[1] = 1;
    bootp[2] = 6;
    memcpy(bootp + 28, mac, sizeof(mac));
    memcpy(vendor, "\x63\x82\x53\x63\xff", 5);

    sent_packet_count = 0;
    sent_packet_len = 0;
    s->nc.info->receive(&s->nc, request, sizeof(request));
    g_assert_cmpuint(sent_packet_count, ==, 1);
    reply_bootp = sent_packet + BOOTP_OFFSET;
    g_assert_cmphex(load_be32(reply_bootp + 20), ==, 0x0a000202);
    option = reply_bootp + BOOTP_VENDOR_OFFSET + 4;
    while (option + 1 < sent_packet + sent_packet_len && *option != 255) {
        if (*option == 17) {
            g_assert_cmpuint(option[1], ==, 1);
            g_assert_cmpuint(option[2], ==, '/');
            return;
        }
        option += option[1] + 2;
    }
    g_assert_not_reached();
}
#endif

static void *opened(QemuSlirpILConnection *connection, void *opaque)
{
    return opaque;
}

static void recorded(QemuSlirpILConnection *connection, const uint8_t *data,
                     size_t len, void *opaque)
{
}

static void closed(QemuSlirpILConnection *connection, void *opaque)
{
}

static const QemuSlirpILListenerOps listener_ops = {
    .open = opened,
    .record = recorded,
    .close = closed,
};

static SlirpState *new_user_netdev(void)
{
    const struct in_addr net = { .s_addr = htonl(0x0a000200) };
    const struct in_addr mask = { .s_addr = htonl(0xffffff00) };
    const struct in_addr host = { .s_addr = htonl(0x0a000202) };
    const struct in_addr dns = { .s_addr = htonl(0x0a000203) };
    SlirpConfig config = {
        .version = SLIRP_CHECK_VERSION(4, 9, 0) ? 6 : 4,
        .in_enabled = true,
        .vnetwork = net,
        .vnetmask = mask,
        .vhost = host,
        .vdhcp_start = { .s_addr = htonl(0x0a00020f) },
        .vnameserver = dns,
    };
    SlirpState *s = g_new0(SlirpState, 1);

    s->nc.info = &net_slirp_info;
    s->nc.name = g_strdup("user0");
    s->vnetwork = net;
    s->vnetmask = mask;
    s->vhost = host;
    s->vnameserver = dns;
    s->slirp = slirp_new(&config, &slirp_cb, s);
    g_assert_nonnull(s->slirp);
#ifdef CONFIG_SLIRP_TCP_SERVICE
    s->stream_registry = qemu_slirp_stream_registry_new(
        true, host, &slirp_stream_backend_ops, s);
#else
    s->stream_registry = qemu_slirp_stream_registry_new(true, host, NULL,
                                                         NULL);
#endif
    s->guestfwds = qemu_slirp_guestfwd_registry_new(
        true, net, mask, host, dns, &slirp_guestfwd_backend_ops, s);
#ifdef CONFIG_SLIRP_UDP_SERVICE
    s->udp_registry = qemu_slirp_udp_registry_new(
        true, host, &slirp_udp_backend_ops, s);
#else
    s->udp_registry = qemu_slirp_udp_registry_new(true, host, NULL, NULL);
#endif
    s->rpc_registry = qemu_slirp_rpc_registry_new(s->udp_registry,
                                                   s->stream_registry);
    g_assert_nonnull(s->rpc_registry);
#ifdef CONFIG_SLIRP_BOOTP_ROOT
    s->bootp_registry = qemu_slirp_bootp_registry_new(
        true, host, &slirp_bootp_backend_ops, s);
#else
    s->bootp_registry = qemu_slirp_bootp_registry_new(
        true, host, NULL, NULL);
#endif
#ifdef CONFIG_SLIRP_PLAN9_BOOTP
    s->plan9 = qemu_slirp_plan9_registry_new(
        true, net, mask, host, dns, &slirp_plan9_backend_ops, s);
#else
    s->plan9 = qemu_slirp_plan9_registry_new(
        true, net, mask, host, dns, NULL, NULL);
#endif
#ifdef CONFIG_SLIRP_IL
    tracking = (TrackingState) {0};
    s->il_registry = qemu_slirp_il_registry_new(
        true, net, mask, host, dns, &tracking_ops, s);
#endif
    s->poll_notifier.notify = net_slirp_poll_notify;
    QTAILQ_INSERT_TAIL(&slirp_stacks, s, entry);
    test_netdev = &s->nc;
    return s;
}

static void destroy_user_netdev(SlirpState *s)
{
    net_slirp_cleanup(&s->nc);
    g_free(s->nc.name);
    g_free(s);
    test_netdev = NULL;
}

#if defined(CONFIG_SLIRP_UDP_SERVICE) && defined(CONFIG_SLIRP_TCP_SERVICE)
typedef struct RpcIntegrationState {
    unsigned calls;
    uint32_t result;
} RpcIntegrationState;

typedef struct RpcLargeIntegrationState {
    unsigned calls;
    size_t body_length;
    uint8_t pattern_seed;
} RpcLargeIntegrationState;

static uint8_t rpc_large_pattern(const RpcLargeIntegrationState *state,
                                 size_t index)
{
    return state->pattern_seed ^ (uint8_t)(index * 37 + 11);
}

static OncRpcDispatchResult rpc_integration_dispatch(
    OncRpcRequest *request, const OncRpcCall *call, void *opaque)
{
    RpcIntegrationState *state = opaque;
    uint8_t reply[32];
    OncRpcXdrWriter writer;

    state->calls++;
    onc_rpc_xdr_writer_init(&writer, reply, sizeof(reply));
    g_assert_true(onc_rpc_reply_success(&writer, call->xid));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, state->result));
    g_assert_true(onc_rpc_request_reply(request, reply,
                                        onc_rpc_xdr_writer_size(&writer)));
    return ONC_RPC_DISPATCH_REPLIED;
}

static OncRpcDispatchResult rpc_large_integration_dispatch(
    OncRpcRequest *request, const OncRpcCall *call, void *opaque)
{
    RpcLargeIntegrationState *state = opaque;
    size_t reply_length = 24 + state->body_length;
    g_autofree uint8_t *reply = g_malloc0(reply_length);
    OncRpcXdrWriter writer;

    state->calls++;
    onc_rpc_xdr_writer_init(&writer, reply, reply_length);
    g_assert_true(onc_rpc_reply_success(&writer, call->xid));
    for (size_t i = 0; i < state->body_length; i++) {
        writer.cursor[i] = rpc_large_pattern(state, i);
    }
    writer.cursor += state->body_length;
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, reply_length);
    g_assert_true(onc_rpc_request_reply(request, reply, reply_length));
    return ONC_RPC_DISPATCH_REPLIED;
}

static size_t build_rpc_call(uint8_t *buffer, size_t capacity, uint32_t xid,
                             uint32_t program, uint32_t version,
                             uint32_t procedure, const uint8_t *body,
                             size_t body_length)
{
    OncRpcXdrWriter writer;

    g_assert_cmpuint(capacity, >=, 40 + body_length);
    onc_rpc_xdr_writer_init(&writer, buffer, capacity);
    g_assert_true(onc_rpc_xdr_put_u32(&writer, xid));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_CALL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, version));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, procedure));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_opaque(&writer, body, body_length));
    return onc_rpc_xdr_writer_size(&writer);
}

static size_t build_rpc_record(uint8_t *buffer, size_t capacity,
                               const uint8_t *payload, size_t length)
{
    g_assert_cmpuint(capacity, >=, length + 4);
    tcp_store_be32(buffer, UINT32_C(0x80000000) | length);
    memcpy(buffer + 4, payload, length);
    return length + 4;
}

static uint32_t rpc_integration_reply_status(const uint8_t *data, size_t len,
                                             OncRpcXdrReader *body)
{
    OncRpcXdrReader reader;
    uint32_t xid, type, status, flavor, auth_length;

    onc_rpc_xdr_reader_init(&reader, data, len);
    g_assert_true(onc_rpc_xdr_u32(&reader, &xid));
    g_assert_true(onc_rpc_xdr_u32(&reader, &type));
    g_assert_true(onc_rpc_xdr_u32(&reader, &status));
    g_assert_cmpuint(type, ==, ONC_RPC_REPLY);
    g_assert_cmpuint(status, ==, ONC_RPC_MSG_ACCEPTED);
    g_assert_true(onc_rpc_xdr_u32(&reader, &flavor));
    g_assert_true(onc_rpc_xdr_u32(&reader, &auth_length));
    g_assert_cmpuint(flavor, ==, ONC_RPC_AUTH_NULL);
    g_assert_cmpuint(auth_length, ==, 0);
    g_assert_true(onc_rpc_xdr_u32(&reader, &status));
    *body = reader;
    return status;
}

static unsigned open_fd_count(void)
{
    GDir *dir;
    const char *name;
    unsigned count = 0;

    dir = g_dir_open("/proc/self/fd", 0, NULL);
    if (!dir) {
        return 0;
    }
    while ((name = g_dir_read_name(dir))) {
        count++;
    }
    g_dir_close(dir);
    return count;
}

static const uint8_t *udp_frame_payload(size_t *length)
{
    const uint8_t *ip = sent_packet + ETHERNET_HEADER_LEN;
    const uint8_t *udp = ip + IPV4_HEADER_LEN;

    *length = load_be16(udp + 4) - UDP_HEADER_LEN;
    return udp + UDP_HEADER_LEN;
}

static uint32_t rpc_load_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static void test_named_netdev_rpc_portmapper(void)
{
    enum { RPC_PROGRAM = 200030, RPC_VERSION = 1, RPC_PORT = 4020 };
    SlirpState *s = new_user_netdev();
    RpcIntegrationState state = { .result = 0xfeedbeef };
    OncRpcProgram program = {
        .program = RPC_PROGRAM,
        .version_low = RPC_VERSION,
        .version_high = RPC_VERSION,
        .port = RPC_PORT,
        .transports = ONC_RPC_TRANSPORT_UDP,
        .dispatch = rpc_integration_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    QemuSlirpUdpListener *udp111 = NULL;
    QemuSlirpStreamListener *tcp111 = NULL;
    StreamConnectionState connection = { .source_port = 40003 };
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    Error *err = NULL;
    uint8_t body[64], call[128], record[132];
    size_t body_len, call_len, record_len, reply_len;
    uint32_t port, service_next;
    unsigned fds_before;

    fds_before = open_fd_count();
    g_assert_cmpint(qemu_slirp_rpc_register("user0", &program,
                                            &registration, &err), ==, 0);
    g_assert_nonnull(registration);
    g_assert_null(err);
    g_assert_cmpuint(open_fd_count(), ==, fds_before);

    /* A guest UDP GETPORT request reaches the in-stack portmapper. */
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, RPC_PROGRAM));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, IPPROTO_UDP));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = build_rpc_call(call, sizeof(call), 1, 100000, 2, 3, body,
                              body_len);
    learn_client_arp(s);
    sent_packet_count = 0;
    sent_packet_len = 0;
    send_udp_request(s, 111, call, call_len);
    g_assert_cmpuint(sent_packet_count, >, 0);
    g_assert_cmphex(rpc_load_be32(sent_packet + ETHERNET_HEADER_LEN + 12), ==,
                    0x0a000202);
    g_assert_cmpuint(load_be16(sent_packet + ETHERNET_HEADER_LEN +
                               IPV4_HEADER_LEN), ==, 111);
    g_assert_cmpuint(load_be16(sent_packet + ETHERNET_HEADER_LEN +
                               IPV4_HEADER_LEN + 2), ==, 49152);
    reply_len = 0;
    {
        const uint8_t *payload = udp_frame_payload(&reply_len);

        port = rpc_integration_reply_status(payload, reply_len, &reply);
        g_assert_cmpuint(port, ==, ONC_RPC_SUCCESS);
    }
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, RPC_PORT);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    /*
     * The broadcast-enabled portmapper also handles directed broadcast
     * GETPORT and CALLIT traffic entirely inside the user-mode stack.
     */
    sent_packet_count = 0;
    sent_packet_len = 0;
    send_udp_request_to(s, 0x0a0002ffU, 111, call, call_len);
    g_assert_cmpuint(sent_packet_count, >, 0);
    reply_len = 0;
    {
        const uint8_t *payload = udp_frame_payload(&reply_len);

        port = rpc_integration_reply_status(payload, reply_len, &reply);
        g_assert_cmpuint(port, ==, ONC_RPC_SUCCESS);
    }
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, RPC_PORT);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, RPC_PROGRAM));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 7));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(&writer, NULL, 0, 0));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = build_rpc_call(call, sizeof(call), 4, 100000, 2, 5, body,
                              body_len);
    sent_packet_count = 0;
    sent_packet_len = 0;
    send_udp_request_to(s, 0x0a0002ffU, 111, call, call_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(sent_packet_count, >, 0);
    reply_len = 0;
    {
        const uint8_t *payload = udp_frame_payload(&reply_len);

        port = rpc_integration_reply_status(payload, reply_len, &reply);
        g_assert_cmpuint(port, ==, ONC_RPC_SUCCESS);
    }
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, RPC_PORT);
    {
        const uint8_t *result;
        size_t result_len;

        g_assert_true(onc_rpc_xdr_counted_opaque(&reply, &result,
                                                  &result_len, 32));
        g_assert_cmpuint(result_len, ==, 4);
        g_assert_cmphex(rpc_load_be32(result), ==, state.result);
    }
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    /* The advertised UDP endpoint is also reachable without a host socket. */
    call_len = build_rpc_call(call, sizeof(call), 2, RPC_PROGRAM,
                              RPC_VERSION, 7, NULL, 0);
    sent_packet_count = 0;
    sent_packet_len = 0;
    send_udp_request(s, RPC_PORT, call, call_len);
    g_assert_cmpuint(state.calls, ==, 2);
    reply_len = 0;
    {
        const uint8_t *payload = udp_frame_payload(&reply_len);

        port = rpc_integration_reply_status(payload, reply_len, &reply);
        g_assert_cmpuint(port, ==, ONC_RPC_SUCCESS);
    }
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmphex(port, ==, state.result);

    /* TCP GETPORT discovers the UDP-only service through portmapper. */
    service_next = establish_tcp_guest_connection(s, connection.source_port,
                                                   111);
    connection.guest_next_seq = 1001;
    connection.service_next_seq = service_next;
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, RPC_PROGRAM));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, IPPROTO_UDP));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = build_rpc_call(call, sizeof(call), 3, 100000, 2, 3, body,
                              body_len);
    record_len = build_rpc_record(record, sizeof(record), call, call_len);
    sent_packet_count = 0;
    sent_packet_len = 0;
    send_tcp_guest_segment_to(s, &connection, 111, 0x18, record, record_len);
    g_assert_cmpuint(sent_packet_count, >, 0);
    g_assert_cmpuint(tcp_frame_payload_len(), >=, 4);
    g_assert_cmphex(rpc_load_be32(tcp_frame_payload()) & 0x7fffffff, ==,
                    tcp_frame_payload_len() - 4);
    {
        const uint8_t *payload = tcp_frame_payload() + 4;
        size_t payload_len = tcp_frame_payload_len() - 4;

        port = rpc_integration_reply_status(payload, payload_len, &reply);
        g_assert_cmpuint(port, ==, ONC_RPC_SUCCESS);
    }
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, RPC_PORT);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    qemu_slirp_rpc_unregister(registration);
    g_assert_cmpuint(open_fd_count(), ==, fds_before);
    /* Both port-111 transports are free after the final registration. */
    g_assert_cmpint(qemu_slirp_udp_listen("user0", 111,
                                          &udp_listener_ops, NULL, &udp111,
                                          &err), ==, 0);
    qemu_slirp_udp_listener_remove(udp111);
    g_assert_cmpint(qemu_slirp_stream_listen("user0", 111, &stream_ops, NULL,
                                             &tcp111, &err), ==, 0);
    qemu_slirp_stream_listener_remove(tcp111);
    destroy_user_netdev(s);
    error_free(err);
}

static void test_named_netdev_rpc_large_tcp_reply(void)
{
    enum {
        RPC_PROGRAM = 200031,
        RPC_VERSION = 1,
        RPC_PORT = 4031,
        RPC_BODY_LENGTH = 128 * 1024 + 4096,
    };
    SlirpState *s = new_user_netdev();
    RpcLargeIntegrationState state = {
        .body_length = RPC_BODY_LENGTH,
        .pattern_seed = 0xa5,
    };
    OncRpcProgram program = {
        .program = RPC_PROGRAM,
        .version_low = RPC_VERSION,
        .version_high = RPC_VERSION,
        .port = RPC_PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_large_integration_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    StreamConnectionState connection = { .source_port = 40004 };
    uint8_t call[128], record[132];
    size_t call_len, record_len;
    size_t reply_length = 24 + RPC_BODY_LENGTH;
    size_t expected_length = reply_length + 4;
    size_t attempts;
    uint32_t service_start;
    OncRpcXdrReader reply;
    uint32_t status;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_rpc_register("user0", &program,
                                            &registration, &err), ==, 0);
    g_assert_nonnull(registration);
    service_start = establish_tcp_guest_connection(s, connection.source_port,
                                                   RPC_PORT);
    connection.guest_next_seq = 1001;
    connection.service_next_seq = service_start;
    tcp_rpc_capture = g_byte_array_new();
    tcp_rpc_capture_source_port = RPC_PORT;
    tcp_rpc_capture_port = connection.source_port;
    tcp_rpc_capture_next_seq = service_start;
    call_len = build_rpc_call(call, sizeof(call), 0x10203040, RPC_PROGRAM,
                              RPC_VERSION, 1, NULL, 0);
    record_len = build_rpc_record(record, sizeof(record), call, call_len);
    send_tcp_guest_segment_to(s, &connection, RPC_PORT, 0x18, record,
                              record_len);
    for (attempts = 0; attempts < 256 &&
                        tcp_rpc_capture->len < expected_length; attempts++) {
        send_tcp_guest_ack_to(s, &connection, RPC_PORT,
                              tcp_rpc_capture_next_seq);
    }
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(tcp_rpc_capture->len, ==, expected_length);
    g_assert_cmphex(tcp_load_be32(tcp_rpc_capture->data), ==,
                    UINT32_C(0x80000000) | (uint32_t)reply_length);
    g_assert_cmphex(tcp_load_be32(tcp_rpc_capture->data + 4), ==,
                    UINT32_C(0x10203040));
    status = rpc_integration_reply_status(tcp_rpc_capture->data + 4,
                                          reply_length, &reply);
    g_assert_cmpuint(status, ==, ONC_RPC_SUCCESS);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reply), ==,
                     RPC_BODY_LENGTH);
    for (size_t i = 0; i < RPC_BODY_LENGTH; i++) {
        g_assert_cmphex(reply.cursor[i], ==, rpc_large_pattern(&state, i));
    }
    reply.cursor += RPC_BODY_LENGTH;
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    g_byte_array_unref(tcp_rpc_capture);
    tcp_rpc_capture = NULL;
    qemu_slirp_rpc_unregister(registration);
    destroy_user_netdev(s);
    error_free(err);
}
#endif

#ifdef CONFIG_SLIRP_TCP_SERVICE
static void test_named_netdev_stream_packet_lifecycle(void)
{
    static const uint8_t guest_payload[] = "guest-payload";
    static const uint8_t server_reply[] = "server-reply";
    struct state {
        StreamState stream;
    } state = { 0 };
    SlirpState *s = new_user_netdev();
    QemuSlirpStreamListener *listener = NULL;
    StreamConnectionState *first;
    StreamConnectionState *second;
    StreamConnectionState *third;
    Error *err = NULL;
    uint8_t *bulk;
    size_t space;
    uint32_t ready_ack;
    uint32_t fin_ack;

    g_assert_cmpint(qemu_slirp_stream_listen(
                        "user0", TCP_SERVICE_PORT, &stream_ops,
                        &state.stream, &listener, &err), ==, 0);
    g_assert_nonnull(listener);
    first = establish_tcp_stream_connection(s, &state.stream, 40000);
    g_assert_cmpuint(state.stream.nconnections, ==, 1);
    g_assert_cmpint(first->peer.sin_family, ==, AF_INET);
    g_assert_cmphex(ntohl(first->peer.sin_addr.s_addr), ==,
                    TCP_GUEST_ADDRESS);
    g_assert_cmpuint(ntohs(first->peer.sin_port), ==, 40000);

    send_tcp_guest_segment(s, first, 0x18, guest_payload,
                           sizeof(guest_payload) - 1);
    g_assert_cmpuint(first->received, ==, 1);
    g_assert_cmpuint(first->len, ==, sizeof(guest_payload) - 1);
    g_assert_cmpmem(first->data, first->len, guest_payload,
                    sizeof(guest_payload) - 1);

    sent_packet_count = 0;
    sent_packet_len = 0;
    g_assert_cmpint(qemu_slirp_stream_send(
                        first->stream, server_reply,
                        sizeof(server_reply) - 1), ==, 0);
    g_assert_cmpuint(sent_packet_count, >, 0);
    g_assert_cmpuint(tcp_frame_payload_len(), ==, sizeof(server_reply) - 1);
    g_assert_cmpmem(tcp_frame_payload(), sizeof(server_reply) - 1,
                    server_reply, sizeof(server_reply) - 1);
    first->service_next_seq += sizeof(server_reply) - 1;

    space = qemu_slirp_stream_can_send(first->stream);
    g_assert_cmpuint(space, >, 0);
    bulk = g_malloc(space);
    memset(bulk, 0xa5, space);
    g_assert_cmpint(qemu_slirp_stream_send(first->stream, bulk, space), ==, 0);
    first->service_next_seq += space;
    g_assert_cmpuint(qemu_slirp_stream_can_send(first->stream), ==, 0);
    g_assert_cmpint(qemu_slirp_stream_send(first->stream, (const uint8_t *)"x",
                                           1), ==, -EAGAIN);
    ready_ack = tcp_frame_seq() + tcp_frame_payload_len();
    send_tcp_guest_ack(s, first, ready_ack);
    g_assert_cmpuint(first->ready, ==, 1);
    g_assert_cmpuint(qemu_slirp_stream_can_send(first->stream), >, 0);
    g_free(bulk);

    second = establish_tcp_stream_connection(s, &state.stream, 40001);
    g_assert_cmpuint(state.stream.nconnections, ==, 2);
    sent_packet_count = 0;
    sent_packet_len = 0;
    send_tcp_guest_segment(s, second, 0x19, NULL, 0);
    qemu_slirp_stream_close(second->stream);
    g_assert_cmpuint(sent_packet_count, >, 0);
    g_assert_true(tcp_frame_flags() & 0x01);
    fin_ack = tcp_frame_seq() + tcp_frame_payload_len() + 1;
    send_tcp_guest_ack(s, second, fin_ack);
    g_assert_cmpuint(second->closed, ==, 1);

    third = establish_tcp_stream_connection(s, &state.stream, 40002);
    g_assert_cmpuint(state.stream.nconnections, ==, 3);
    g_assert_cmpuint(third->closed, ==, 0);
    destroy_user_netdev(s);
    g_assert_cmpuint(first->closed, ==, 1);
    g_assert_cmpuint(third->closed, ==, 1);
    qemu_slirp_stream_listener_remove(listener);
    error_free(err);
}
#endif

static void test_named_netdev_facade(void)
{
    Error *err = NULL;
    QemuSlirpILListener *listener = NULL;
    QemuSlirpStreamListener *stream = NULL;
    QemuSlirpUdpListener *udp = NULL;
    QemuSlirpBootpRootLease *root = NULL;

    g_assert_cmpint(qemu_slirp_udp_listen("missing", 2049, NULL, NULL,
                                          &udp, &err), ==, -1);
    g_assert_nonnull(err);
    g_assert_null(udp);
    error_free(err);
    err = NULL;
    g_assert_false(qemu_slirp_bootp_root_claim("missing", "/", &root,
                                               &err));
    g_assert_nonnull(err);
    g_assert_null(root);
    error_free(err);
    err = NULL;

    g_assert_false(qemu_slirp_plan9_bootp_available("missing", &err));
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "Unrecognized netdev"));
    error_free(err);
    err = NULL;

    g_assert_cmpint(qemu_slirp_il_listen("missing", test_addr(), 17008,
                                         NULL, NULL, &listener, &err), ==, -1);
#ifdef CONFIG_SLIRP_IL
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "Unrecognized netdev"));
#else
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "IL is unavailable"));
#endif
    g_assert_null(listener);
    error_free(err);

    err = NULL;
    g_assert_cmpint(qemu_slirp_stream_listen("missing", 17008, &stream_ops,
                                             NULL, &stream, &err), ==, -1);
    g_assert_nonnull(err);
    g_assert_null(stream);
    error_free(err);
}

static void test_user_netdev_lifecycle_paths(void)
{
    SlirpState *s = new_user_netdev();
    MainLoopPoll poll = {
        .state = MAIN_LOOP_POLL_OK,
        .pollfds = g_array_new(false, false, sizeof(GPollFD)),
    };
    Error *err = NULL;
    QemuSlirpILListener *listener = NULL;
    uint8_t packet[ETH_HLEN] = {0};
    QemuSlirpPlan9BootpLease *bootp = NULL;
    QemuSlirpBootpRootLease *root = NULL;
    QemuSlirpUdpListener *udp = NULL;
    QemuSlirpStreamListener *stream = NULL;
#ifdef CONFIG_SLIRP_IL
    tracking.assert_cleanup = true;
#endif
#ifdef CONFIG_SLIRP_UDP_SERVICE
    static const uint8_t message[] = { 0x12, 0x34, 0x56, 0x78 };
    UdpState udp_state = {.send_result = -1};
#endif

    s->nc.info->receive(&s->nc, packet, sizeof(packet));
    s->poll_notifier.notify(&s->poll_notifier, &poll);
#ifdef CONFIG_SLIRP_PLAN9_BOOTP
    g_assert_true(qemu_slirp_plan9_bootp_available("user0", &err));
    g_assert_true(qemu_slirp_plan9_bootp_claim(
        "user0", test_addr(), test_addr(), &bootp, &err));
    g_assert_nonnull(bootp);
    send_plan9_bootp_request(
        s, "p9  255.255.255.0 10.0.2.4 10.0.2.4 10.0.2.2");
    qemu_slirp_plan9_bootp_release(&bootp);
    g_assert_true(qemu_slirp_plan9_bootp_claim(
        "user0", test_addr(), (struct in_addr) { 0 }, &bootp, &err));
    send_plan9_bootp_request(
        s, "p9  255.255.255.0 10.0.2.4 0.0.0.0 10.0.2.2");
#else
    g_assert_false(qemu_slirp_plan9_bootp_available("user0", &err));
    g_assert_null(err);
#endif

    {
        NetClientInfo non_user = { .type = NET_CLIENT_DRIVER_NONE };
        NetClientInfo *user_info = s->nc.info;

        s->nc.info = &non_user;
        g_assert_false(qemu_slirp_plan9_bootp_available("user0", &err));
        g_assert_nonnull(err);
        g_assert_nonnull(strstr(error_get_pretty(err), "not a user-mode"));
        error_free(err);
        err = NULL;
        g_assert_cmpint(qemu_slirp_udp_listen("user0", 2049,
                                              &udp_listener_ops, NULL, &udp,
                                              &err), ==, -1);
        g_assert_nonnull(err);
        error_free(err);
        err = NULL;
        g_assert_cmpint(qemu_slirp_stream_listen("user0", 17008,
                                                 &stream_ops, NULL, &stream,
                                                 &err), ==, -1);
        g_assert_nonnull(err);
        error_free(err);
        err = NULL;
        g_assert_false(qemu_slirp_bootp_root_claim("user0", "/", &root,
                                                   &err));
        g_assert_nonnull(err);
        error_free(err);
        err = NULL;
        s->nc.info = user_info;
    }
#ifdef CONFIG_SLIRP_TCP_SERVICE
    g_assert_cmpint(qemu_slirp_stream_listen("user0", 17008, &stream_ops,
                                             NULL, &stream, &err), ==, 0);
    g_assert_nonnull(stream);
    g_assert_cmpint(qemu_slirp_stream_listen("user0", 17008, &stream_ops,
                                             NULL, NULL, &err), ==, -1);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
#else
    g_assert_cmpint(qemu_slirp_stream_listen("user0", 17008, &stream_ops,
                                             NULL, &stream, &err), ==, -1);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "TCP stream"));
    error_free(err);
    err = NULL;
#endif
#ifdef CONFIG_SLIRP_UDP_SERVICE
    g_assert_cmpint(qemu_slirp_udp_listen("user0", 2049, &udp_listener_ops,
                                          &udp_state, &udp, &err), ==, 0);
    udp_state.listener = udp;
    learn_client_arp(s);
    send_udp_request(s, 2049, message, sizeof(message));
    g_assert_cmpuint(udp_state.calls, ==, 1);
    g_assert_cmphex(ntohl(udp_state.peer.sin_addr.s_addr), ==, 0x0a00020f);
    g_assert_cmpuint(ntohs(udp_state.peer.sin_port), ==, 49152);
    g_assert_cmpmem(udp_state.data, udp_state.len, message, sizeof(message));
    g_assert_cmpint(udp_state.send_result, ==, 0);
    g_assert_cmpuint(sent_packet_count, ==, 1);
    g_assert_cmphex(load_be32(sent_packet + ETHERNET_HEADER_LEN + 12), ==,
                    0x0a000202);
    g_assert_cmphex(load_be32(sent_packet + ETHERNET_HEADER_LEN + 16), ==,
                    0x0a00020f);
    g_assert_cmpuint(load_be16(sent_packet + ETHERNET_HEADER_LEN +
                              IPV4_HEADER_LEN), ==, 2049);
    g_assert_cmpuint(load_be16(sent_packet + ETHERNET_HEADER_LEN +
                              IPV4_HEADER_LEN + 2), ==, 49152);
#endif
#ifdef CONFIG_SLIRP_BOOTP_ROOT
    g_assert_true(qemu_slirp_bootp_root_claim("user0", "/", &root, &err));
    send_root_bootp_request(s);
#endif
#ifdef CONFIG_SLIRP_IL
    NetClientInfo non_user = { .type = NET_CLIENT_DRIVER_NONE };
    NetClientInfo *user_info = s->nc.info;
    unsigned progress;

    s->nc.info = &non_user;
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, &listener, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "not a user-mode"));
    error_free(err);
    err = NULL;
    s->nc.info = user_info;
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, &listener, &err),
                    ==, 0);
    g_assert_nonnull(listener);
    progress = qemu_slirp_il_registry_get_progress_generation(s->il_registry);
    s->nc.info->receive(&s->nc, packet, sizeof(packet));
    g_assert_cmpuint(qemu_slirp_il_registry_get_progress_generation(
                         s->il_registry), ==, progress + 1);
    s->poll_notifier.notify(&s->poll_notifier, &poll);
    g_assert_cmpuint(qemu_slirp_il_registry_get_progress_generation(
                         s->il_registry), ==, progress + 2);
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, NULL, &err),
                    ==, -1);
    g_assert_nonnull(err);
    error_free(err);
    qemu_slirp_il_listener_remove(listener);
    g_assert_cmpuint(tracking.listener_removes, ==, 1);
    listener = NULL;
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17009,
                                         &listener_ops, NULL, &listener, &err),
                    ==, 0);
    g_assert_nonnull(listener);
#else
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, &listener, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "IL is unavailable"));
    error_free(err);
#endif
    g_array_free(poll.pollfds, true);
    destroy_user_netdev(s);
#ifdef CONFIG_SLIRP_IL
    g_assert_true(tracking.cleanup_called);
    g_assert_cmpuint(tracking.listener_removes, ==, 2);
    /* Caller ref survives registry invalidation and remains removable. */
    qemu_slirp_il_listener_remove(listener);
#endif
    /* The lease remains safely releasable after its netdev is gone. */
    qemu_slirp_plan9_bootp_release(&bootp);
    g_assert_null(bootp);
    qemu_slirp_bootp_root_release(&root);
    g_assert_null(root);
    qemu_slirp_udp_listener_remove(udp);
    qemu_slirp_stream_listener_remove(stream);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
#if defined(CONFIG_SLIRP_UDP_SERVICE) && defined(CONFIG_SLIRP_TCP_SERVICE)
    g_test_add_func("/slirp-il-integration/named-netdev-rpc-portmapper",
                    test_named_netdev_rpc_portmapper);
    g_test_add_func("/slirp-il-integration/named-netdev-rpc-large-tcp",
                    test_named_netdev_rpc_large_tcp_reply);
#endif
#ifdef CONFIG_SLIRP_TCP_SERVICE
    g_test_add_func("/slirp-il-integration/named-netdev-stream-packets",
                    test_named_netdev_stream_packet_lifecycle);
#endif
    g_test_add_func("/slirp-il-integration/named-netdev-facade",
                    test_named_netdev_facade);
    g_test_add_func("/slirp-il-integration/user-netdev-lifecycle",
                    test_user_netdev_lifecycle_paths);
    return g_test_run();
}
