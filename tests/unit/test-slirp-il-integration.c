/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/net.h"
#include "net/slirp-bootp.h"
#include "net/slirp-il.h"
#include "net/slirp-plan9.h"
#include "net/slirp-udp.h"
#include "net/slirp.h"
#include "qapi/error.h"
#include <libslirp.h>

static NetClientState *test_netdev;
static uint8_t sent_packet[1024];
static size_t sent_packet_len;
static unsigned sent_packet_count;

#define ETHERNET_HEADER_LEN 14
#define IPV4_HEADER_LEN 20
#define UDP_HEADER_LEN 8

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
    g_assert_cmpuint(size, <=, sizeof(sent_packet));
    memcpy(sent_packet, buf, size);
    sent_packet_len = size;
    sent_packet_count++;
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
    g_assert_cmpuint(tracking.listener_removes, ==, 2);
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

static void send_udp_request(SlirpState *s, uint16_t port,
                             const uint8_t *payload, size_t payload_len)
{
    static const uint8_t mac[] = { 0x00, 0x00, 0x0f, 0x12, 0x34, 0x56 };
    uint8_t packet[ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + UDP_HEADER_LEN + 32]
        = {0};
    uint8_t *ip_header = packet + ETHERNET_HEADER_LEN;
    uint8_t *udp = ip_header + IPV4_HEADER_LEN;
    size_t length = ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + UDP_HEADER_LEN +
                    payload_len;

    g_assert_cmpuint(payload_len, <=, 32);
    packet[0] = 0x52;
    packet[1] = 0x55;
    packet[2] = 10;
    packet[3] = 0;
    packet[4] = 2;
    packet[5] = 2;
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
    ip_header[16] = 10;
    ip_header[17] = 0;
    ip_header[18] = 2;
    ip_header[19] = 2;
    store_be16(ip_header + 10,
               ipv4_checksum(ip_header, IPV4_HEADER_LEN));
    store_be16(udp, 49152);
    store_be16(udp + 2, port);
    store_be16(udp + 4, UDP_HEADER_LEN + payload_len);
    memcpy(udp + UDP_HEADER_LEN, payload, payload_len);

    s->nc.info->receive(&s->nc, packet, length);
}
#endif

static uint16_t load_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t load_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

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
    s->guestfwds = qemu_slirp_guestfwd_registry_new(
        true, net, mask, host, dns, &slirp_guestfwd_backend_ops, s);
#ifdef CONFIG_SLIRP_UDP_SERVICE
    s->udp_registry = qemu_slirp_udp_registry_new(
        true, host, &slirp_udp_backend_ops, s);
#else
    s->udp_registry = qemu_slirp_udp_registry_new(true, host, NULL, NULL);
#endif
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

static void test_named_netdev_facade(void)
{
    Error *err = NULL;
    QemuSlirpILListener *listener = NULL;
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
        g_assert_false(qemu_slirp_bootp_root_claim("user0", "/", &root,
                                                   &err));
        g_assert_nonnull(err);
        error_free(err);
        err = NULL;
        s->nc.info = user_info;
    }
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
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-il-integration/named-netdev-facade",
                    test_named_netdev_facade);
    g_test_add_func("/slirp-il-integration/user-netdev-lifecycle",
                    test_user_netdev_lifecycle_paths);
    return g_test_run();
}
