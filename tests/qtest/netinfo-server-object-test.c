/*
 * Guest-only NetInfo server user-creatable object tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/sockets.h"
#include "hw/netinfo/netinfo-protocol.h"
#include "net/onc-rpc.h"
#include "libqtest.h"
#include "qapi/error.h"
#include "qobject/qdict.h"

#include <poll.h>

static char *test_root;

#define NETINFO_BINDER_UDP_PORT 659
#define NETINFO_BINDER_TCP_PORT 661
#define NETINFO_DATABASE_UDP_PORT 660
#define NETINFO_DATABASE_TCP_PORT 662
#define NETINFO_CUSTOM_BINDER_UDP_PORT 7659
#define NETINFO_CUSTOM_BINDER_TCP_PORT 7661
#define NETINFO_CUSTOM_DATABASE_UDP_PORT 7660
#define NETINFO_CUSTOM_DATABASE_TCP_PORT 7662

#define NETINFO_GUEST_IP 0x0a00020fU
#define NETINFO_SLIRP_HOST_IP 0x0a000202U
#define NETINFO_GUEST_RPC_PORT 40000U
#define NETINFO_RPC_TIMEOUT_US (5 * G_TIME_SPAN_SECOND)
#define NETINFO_RPC_MAX_FRAMES 64U
#define NETINFO_UDP_PROTOCOL 17U

#define NETINFO_ETHERNET_HEADER_LEN 14
#define NETINFO_IPV4_HEADER_LEN 20
#define NETINFO_UDP_HEADER_LEN 8

static const uint8_t netinfo_guest_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00,
                                              0x01 };

static QTestState *start_vm(const char *extra)
{
    return qtest_initf("-machine none -nodefaults "
                       "-netdev user,id=nextnet %s", extra ?: "");
}

static void assert_qmp_error_contains(QDict *response, const char *needle)
{
    QDict *error;

    g_assert_nonnull(response);
    error = qdict_get_qdict(response, "error");
    g_assert_nonnull(error);
    g_assert_nonnull(strstr(qdict_get_str(error, "desc"), needle));
    qobject_unref(response);
}

static void assert_qmp_success(QDict *response)
{
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    qobject_unref(response);
}

static void put_be16(uint8_t *buf, uint16_t value)
{
    buf[0] = value >> 8;
    buf[1] = value;
}

static void put_be32(uint8_t *buf, uint32_t value)
{
    buf[0] = value >> 24;
    buf[1] = value >> 16;
    buf[2] = value >> 8;
    buf[3] = value;
}

static uint16_t get_be16(const uint8_t *buf)
{
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static uint32_t get_be32(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | buf[3];
}

static uint16_t internet_checksum(const uint8_t *buf, size_t len)
{
    uint32_t sum = 0;

    while (len >= 2) {
        sum += get_be16(buf);
        buf += 2;
        len -= 2;
    }
    if (len) {
        sum += (uint16_t)buf[0] << 8;
    }
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}

static void send_all(int fd, const uint8_t *buf, size_t len)
{
    while (len) {
        ssize_t ret = send(fd, buf, len, 0);

        if (ret < 0 && errno == EINTR) {
            continue;
        }
        g_assert_cmpint(ret, >, 0);
        buf += ret;
        len -= ret;
    }
}

static bool recv_all(int fd, uint8_t *buf, size_t len, gint64 deadline)
{
    struct pollfd pfd = {
        .fd = fd,
        .events = POLLIN,
    };

    while (len) {
        ssize_t ret;
        int ready;
        gint64 remaining = deadline - g_get_monotonic_time();
        int timeout_ms;

        if (remaining <= 0) {
            return false;
        }
        timeout_ms = remaining / 1000;
        if (remaining % 1000) {
            timeout_ms++;
        }
        timeout_ms = MIN(timeout_ms, INT_MAX);

        do {
            ready = poll(&pfd, 1, timeout_ms);
        } while (ready < 0 && errno == EINTR);
        if (ready != 1 || !(pfd.revents & POLLIN)) {
            return false;
        }
        ret = recv(fd, buf, len, 0);

        if (ret < 0 && errno == EINTR) {
            continue;
        }
        if (ret <= 0) {
            return false;
        }
        buf += ret;
        len -= ret;
    }
    return true;
}

static void send_redirector_frame(int fd, const uint8_t *frame, size_t len)
{
    uint32_t frame_len = htonl(len);

    send_all(fd, (const uint8_t *)&frame_len, sizeof(frame_len));
    send_all(fd, frame, len);
}

static bool recv_redirector_frame(int fd, uint8_t *frame, size_t capacity,
                                  gint64 deadline, size_t *frame_len_out)
{
    uint32_t frame_len;

    if (!recv_all(fd, (uint8_t *)&frame_len, sizeof(frame_len), deadline)) {
        return false;
    }
    frame_len = ntohl(frame_len);
    if (frame_len > capacity ||
        !recv_all(fd, frame, frame_len, deadline)) {
        return false;
    }
    *frame_len_out = frame_len;
    return true;
}

static int connect_redirector(const char *path)
{
    Error *local_err = NULL;
    int fd;

    for (unsigned attempt = 0; attempt < 100; attempt++) {
        fd = unix_connect(path, &local_err);
        if (fd >= 0) {
            return fd;
        }
        error_free(local_err);
        local_err = NULL;
        g_usleep(10000);
    }
    error_report_err(local_err);
    g_assert_not_reached();
    return -1;
}

static size_t build_arp_request(uint8_t *frame)
{
    memset(frame, 0, 42);
    memset(frame, 0xff, 6);
    memcpy(frame + 6, netinfo_guest_mac, sizeof(netinfo_guest_mac));
    put_be16(frame + 12, 0x0806);
    put_be16(frame + 14, 1);
    put_be16(frame + 16, 0x0800);
    frame[18] = 6;
    frame[19] = 4;
    put_be16(frame + 20, 1);
    memcpy(frame + 22, netinfo_guest_mac, sizeof(netinfo_guest_mac));
    put_be32(frame + 28, NETINFO_GUEST_IP);
    put_be32(frame + 38, NETINFO_SLIRP_HOST_IP);
    return 42;
}

static bool is_arp_reply(const uint8_t *frame, size_t len)
{
    if (len < 42 || get_be16(frame + 12) != 0x0806 ||
        memcmp(frame, netinfo_guest_mac, sizeof(netinfo_guest_mac)) ||
        get_be16(frame + 14) != 1 || get_be16(frame + 16) != 0x0800 ||
        frame[18] != 6 || frame[19] != 4 || get_be16(frame + 20) != 2 ||
        get_be32(frame + 28) != NETINFO_SLIRP_HOST_IP ||
        memcmp(frame + 32, netinfo_guest_mac, sizeof(netinfo_guest_mac)) ||
        get_be32(frame + 38) != NETINFO_GUEST_IP ||
        memcmp(frame + 6, frame + 22, sizeof(netinfo_guest_mac))) {
        return false;
    }
    return true;
}

static size_t build_binder_call(uint8_t *frame, const uint8_t *host_mac,
                                uint16_t binder_udp_port, uint32_t xid,
                                uint32_t procedure, const char *tag)
{
    uint8_t *ip = frame + NETINFO_ETHERNET_HEADER_LEN;
    uint8_t *udp = ip + NETINFO_IPV4_HEADER_LEN;
    uint8_t *rpc = udp + NETINFO_UDP_HEADER_LEN;
    size_t rpc_len;
    size_t ip_len;
    size_t tag_len = 0;

    rpc_len = 40;
    if (procedure == NIBIND_GETREGISTER) {
        g_assert_nonnull(tag);
        tag_len = strlen(tag);
        g_assert_cmpuint(tag_len, <=, UINT32_MAX);
        rpc_len += sizeof(uint32_t) +
                   QEMU_ALIGN_UP(tag_len, sizeof(uint32_t));
    } else {
        g_assert_cmpuint(procedure, ==, NIBIND_LISTREG);
    }
    ip_len = NETINFO_IPV4_HEADER_LEN + NETINFO_UDP_HEADER_LEN + rpc_len;
    memset(frame, 0, NETINFO_ETHERNET_HEADER_LEN + ip_len);

    put_be32(rpc, xid);
    put_be32(rpc + 4, ONC_RPC_CALL);
    put_be32(rpc + 8, ONC_RPC_VERSION);
    put_be32(rpc + 12, NIBIND_PROG);
    put_be32(rpc + 16, NIBIND_VERS);
    put_be32(rpc + 20, procedure);
    put_be32(rpc + 24, ONC_RPC_AUTH_NULL);
    put_be32(rpc + 28, 0);
    put_be32(rpc + 32, ONC_RPC_AUTH_NULL);
    put_be32(rpc + 36, 0);
    if (procedure == NIBIND_GETREGISTER) {
        put_be32(rpc + 40, tag_len);
        memcpy(rpc + 44, tag, tag_len);
    }
    memcpy(frame, host_mac, sizeof(netinfo_guest_mac));
    memcpy(frame + 6, netinfo_guest_mac, sizeof(netinfo_guest_mac));
    put_be16(frame + 12, 0x0800);

    ip[0] = 0x45;
    put_be16(ip + 2, ip_len);
    ip[8] = 64;
    ip[9] = NETINFO_UDP_PROTOCOL;
    put_be32(ip + 12, NETINFO_GUEST_IP);
    put_be32(ip + 16, NETINFO_SLIRP_HOST_IP);
    put_be16(ip + 10, internet_checksum(ip, NETINFO_IPV4_HEADER_LEN));

    put_be16(udp, NETINFO_GUEST_RPC_PORT);
    put_be16(udp + 2, binder_udp_port);
    put_be16(udp + 4, NETINFO_UDP_HEADER_LEN + rpc_len);
    /* UDP checksum zero means no checksum for IPv4. */
    return NETINFO_ETHERNET_HEADER_LEN + ip_len;
}

static bool is_binder_reply(const uint8_t *frame, size_t len,
                            const uint8_t *host_mac, uint16_t binder_udp_port,
                            const uint8_t **rpc, size_t *rpc_len)
{
    size_t ip_header_len;
    size_t ip_payload_len;
    size_t udp_len;
    const uint8_t *ip;
    const uint8_t *udp;
    const uint8_t *checksum_data;
    g_autofree uint8_t *checksum_buf = NULL;

    if (len < NETINFO_ETHERNET_HEADER_LEN + NETINFO_IPV4_HEADER_LEN +
        NETINFO_UDP_HEADER_LEN ||
        get_be16(frame + 12) != 0x0800 ||
        memcmp(frame + 6, host_mac, sizeof(netinfo_guest_mac)) ||
        memcmp(frame, netinfo_guest_mac, sizeof(netinfo_guest_mac))) {
        return false;
    }
    ip = frame + NETINFO_ETHERNET_HEADER_LEN;
    ip_header_len = (ip[0] & 0x0f) * 4;
    if ((ip[0] >> 4) != 4 || ip_header_len < NETINFO_IPV4_HEADER_LEN ||
        len < NETINFO_ETHERNET_HEADER_LEN + ip_header_len +
              NETINFO_UDP_HEADER_LEN ||
        get_be16(ip + 2) < ip_header_len + NETINFO_UDP_HEADER_LEN ||
        get_be16(ip + 2) > len - NETINFO_ETHERNET_HEADER_LEN ||
        internet_checksum(ip, ip_header_len) != 0 ||
        (get_be16(ip + 6) & 0x3fff) || ip[9] != NETINFO_UDP_PROTOCOL ||
        get_be32(ip + 12) != NETINFO_SLIRP_HOST_IP ||
        get_be32(ip + 16) != NETINFO_GUEST_IP) {
        return false;
    }
    ip_payload_len = get_be16(ip + 2) - ip_header_len;
    udp = ip + ip_header_len;
    udp_len = get_be16(udp + 4);
    if (udp_len < NETINFO_UDP_HEADER_LEN || udp_len != ip_payload_len ||
        len < NETINFO_ETHERNET_HEADER_LEN + ip_header_len + udp_len ||
        get_be16(udp) != binder_udp_port ||
        get_be16(udp + 2) != NETINFO_GUEST_RPC_PORT) {
        return false;
    }
    if (get_be16(udp + 6)) {
        checksum_buf = g_malloc(12 + udp_len);
        memcpy(checksum_buf, ip + 12, 8);
        checksum_buf[8] = 0;
        checksum_buf[9] = ip[9];
        put_be16(checksum_buf + 10, udp_len);
        checksum_data = udp;
        memcpy(checksum_buf + 12, checksum_data, udp_len);
        if (internet_checksum(checksum_buf, 12 + udp_len) != 0) {
            return false;
        }
    }
    *rpc = udp + NETINFO_UDP_HEADER_LEN;
    *rpc_len = udp_len - NETINFO_UDP_HEADER_LEN;
    return true;
}

static bool recv_binder_reply(int fd, uint8_t *frame, size_t capacity,
                              gint64 deadline, const uint8_t *host_mac,
                              uint16_t binder_udp_port, uint32_t xid,
                              const uint8_t **rpc, size_t *rpc_len)
{
    size_t frame_len;

    for (unsigned frame_count = 0; frame_count < NETINFO_RPC_MAX_FRAMES;
         frame_count++) {
        if (!recv_redirector_frame(fd, frame, capacity, deadline,
                                   &frame_len)) {
            return false;
        }
        if (is_binder_reply(frame, frame_len, host_mac, binder_udp_port,
                            rpc, rpc_len) && *rpc_len >= sizeof(uint32_t) &&
            get_be32(*rpc) == xid) {
            return true;
        }
    }
    return false;
}

static void assert_binder_reply_header(const uint8_t *rpc, size_t rpc_len,
                                       uint32_t xid)
{
    g_assert_cmpuint(rpc_len, >=, 24);
    g_assert_cmpuint(get_be32(rpc), ==, xid);
    g_assert_cmpuint(get_be32(rpc + 4), ==, ONC_RPC_REPLY);
    g_assert_cmpuint(get_be32(rpc + 8), ==, ONC_RPC_MSG_ACCEPTED);
    g_assert_cmpuint(get_be32(rpc + 12), ==, ONC_RPC_AUTH_NULL);
    g_assert_cmpuint(get_be32(rpc + 16), ==, 0);
    g_assert_cmpuint(get_be32(rpc + 20), ==, ONC_RPC_SUCCESS);
}

static void test_cli_creation(void)
{
    QTestState *qts = qtest_initf(
        "-machine none -nodefaults -netdev user,id=nextnet "
        "-object netinfo-server,id=ni0,netdev=nextnet");
    QDict *response;

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni0','property':'netdev'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "nextnet");
    qobject_unref(response);
    qtest_quit(qts);
}

static QDict *object_add_response_with_database_tcp_port(
    QTestState *qts, const char *id, const char *netdev, const char *config,
    const char *domain_tag, bool writable, bool custom_ports,
    uint16_t database_tcp_port)
{
    QDict *arguments = qdict_new();

    qdict_put_str(arguments, "qom-type", "netinfo-server");
    qdict_put_str(arguments, "id", id);
    if (netdev) {
        qdict_put_str(arguments, "netdev", netdev);
    }
    if (config) {
        qdict_put_str(arguments, "config", config);
    }
    if (domain_tag) {
        qdict_put_str(arguments, "domain-tag", domain_tag);
    }
    if (writable) {
        qdict_put_bool(arguments, "writable", true);
    }
    if (custom_ports) {
        qdict_put_int(arguments, "binder-udp-port",
                      NETINFO_CUSTOM_BINDER_UDP_PORT);
        qdict_put_int(arguments, "binder-tcp-port",
                      NETINFO_CUSTOM_BINDER_TCP_PORT);
        qdict_put_int(arguments, "database-udp-port",
                      NETINFO_CUSTOM_DATABASE_UDP_PORT);
        qdict_put_int(arguments, "database-tcp-port", database_tcp_port);
    }
    return qtest_qmp(qts,
                     "{'execute':'object-add','arguments':%p}", arguments);
}

static QDict *object_add_response(QTestState *qts, const char *id,
                                  const char *netdev, const char *config,
                                  const char *domain_tag, bool writable,
                                  bool custom_ports)
{
    return object_add_response_with_database_tcp_port(
        qts, id, netdev, config, domain_tag, writable, custom_ports,
        NETINFO_DATABASE_TCP_PORT);
}

static void object_add(QTestState *qts, const char *id)
{
    assert_qmp_success(object_add_response(qts, id, "nextnet", NULL, NULL,
                                           false, false));
}

static void object_del(QTestState *qts, const char *id)
{
    assert_qmp_success(qtest_qmp(qts,
        "{'execute':'object-del','arguments':{'id':%s}}", id));
}

static char *write_domain(const char *tag)
{
    g_autofree char *path = g_strdup_printf("%s/%s.json5", test_root, tag);
    g_autofree char *contents = g_strdup_printf(
        "{ version: 1, domain: { tag: '%s', name: '/' }, "
        "nodes: [{ id: 0x0, instance: 0x24, parent: null, "
        "properties: { name: ['/'] } }] }", tag);

    g_assert_true(g_file_set_contents(path, contents, -1, NULL));
    return g_steal_pointer(&path);
}

static void assert_no_proc_listener(pid_t pid, const char *protocol,
                                    uint16_t port)
{
#ifdef CONFIG_LINUX
    g_autofree char *path = g_strdup_printf("/proc/%d/net/%s", (int)pid,
                                            protocol);
    g_autofree char *contents = NULL;
    g_auto(GStrv) lines = NULL;
    g_autoptr(GError) local_err = NULL;

    g_assert_true(g_file_get_contents(path, &contents, NULL, &local_err));
    g_assert_no_error(local_err);
    lines = g_strsplit(contents, "\n", -1);
    for (size_t i = 1; lines[i]; i++) {
        char *local_address = strchr(lines[i], ':');
        char *local_port;
        const char *end;
        unsigned long value;

        if (!local_address) {
            continue;
        }
        local_address++;
        local_port = strchr(local_address, ':');
        if (!local_port) {
            continue;
        }
        local_port++;
        if (qemu_strtoul(local_port, &end, 16, &value) ||
            value > UINT16_MAX ||
            (*end != ' ' && *end != '\t')) {
            continue;
        }
        g_assert_cmpuint(value, !=, port);
    }
#else
    (void)pid;
    (void)protocol;
    (void)port;
    g_test_skip("/proc network tables are unavailable on this host");
#endif
}

static void test_zero_config_creation(void)
{
    QTestState *qts = start_vm("");
    QDict *response;

    object_add(qts, "ni0");
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni0','property':'netdev'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "nextnet");
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni0','property':'binder-udp-port'}}");
    g_assert_cmpuint(qdict_get_uint(response, "return"), ==,
                     NETINFO_BINDER_UDP_PORT);
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni0','property':'binder-tcp-port'}}");
    g_assert_cmpuint(qdict_get_uint(response, "return"), ==,
                     NETINFO_BINDER_TCP_PORT);
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni0','property':'database-udp-port'}}");
    g_assert_cmpuint(qdict_get_uint(response, "return"), ==,
                     NETINFO_DATABASE_UDP_PORT);
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni0','property':'database-tcp-port'}}");
    g_assert_cmpuint(qdict_get_uint(response, "return"), ==,
                     NETINFO_DATABASE_TCP_PORT);
    qobject_unref(response);
    object_del(qts, "ni0");
    qtest_quit(qts);
}

static void test_json5_creation(void)
{
    g_autofree char *config = write_domain("json5");
    QTestState *qts = start_vm("");

    assert_qmp_success(object_add_response(qts, "ni0", "nextnet", config,
                                           "json5", false, false));
    object_del(qts, "ni0");
    qtest_quit(qts);
    g_assert_cmpint(g_unlink(config), ==, 0);
}

static void test_missing_and_wrong_netdev(void)
{
    QTestState *qts = start_vm("");

    assert_qmp_error_contains(object_add_response(qts, "missing", NULL, NULL,
                                                  NULL, false, false),
                              "netdev");
    assert_qmp_error_contains(object_add_response(qts, "wrong", "missingnet",
                                                  NULL, NULL, false, false),
                              "netdev");
    qtest_quit(qts);
}

static void test_duplicate_object(void)
{
    QTestState *qts = start_vm("");

    object_add(qts, "ni0");
    assert_qmp_error_contains(object_add_response(qts, "ni0", "nextnet",
                                                  NULL, NULL, false, false),
                              "duplicate");
    object_del(qts, "ni0");
    qtest_quit(qts);
}

static void test_netdev_isolation(void)
{
    QTestState *qts = start_vm("-netdev user,id=othernet");

    object_add(qts, "ni0");
    assert_qmp_success(object_add_response(qts, "ni1", "othernet", NULL,
                                           NULL, false, false));
    object_del(qts, "ni1");
    object_del(qts, "ni0");
    qtest_quit(qts);
}

static void test_port_collision_unwind(void)
{
    QTestState *qts = start_vm("");
    QDict *response;

    object_add(qts, "ni0");
    assert_qmp_error_contains(object_add_response(qts, "ni1", "nextnet",
                                                  NULL, NULL, false, true),
                              "Conflicting");
    object_del(qts, "ni0");
    assert_qmp_success(object_add_response(qts, "ni1", "nextnet", NULL,
                                           NULL, false, true));
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/ni1','property':'database-tcp-port'}}");
    g_assert_cmpuint(qdict_get_uint(response, "return"), ==, 662);
    qobject_unref(response);
    object_del(qts, "ni1");
    qtest_quit(qts);
}

static void test_delete_recreate_reset(void)
{
    QTestState *qts = start_vm("");

    object_add(qts, "ni0");
    qtest_system_reset(qts);
    /* The reset must not silently drop the live RPC registrations. */
    assert_qmp_error_contains(object_add_response(qts, "ni1", "nextnet",
                                                  NULL, NULL, false, true),
                              "Conflicting");
    object_del(qts, "ni0");
    object_add(qts, "ni0");
    object_del(qts, "ni0");
    qtest_quit(qts);
}

static void test_config_domain_tag_and_writable(void)
{
    g_autofree char *config = write_domain("custom");
    QTestState *qts = start_vm("");

    assert_qmp_error_contains(object_add_response(qts, "mismatch", "nextnet",
                                                  config, "network", false,
                                                  false),
                              "domain-tag");
    assert_qmp_error_contains(object_add_response(qts, "writable", "nextnet",
                                                  NULL, NULL, true, false),
                              "writable");
    assert_qmp_success(object_add_response(qts, "tagged", "nextnet", NULL,
                                           "custom", false, false));
    object_del(qts, "tagged");
    assert_qmp_success(object_add_response(qts, "custom", "nextnet", config,
                                           "custom", false, false));
    object_del(qts, "custom");
    qtest_quit(qts);
    g_assert_cmpint(g_unlink(config), ==, 0);
}

static void test_migration_blocker(void)
{
    QTestState *qts = start_vm("");

    object_add(qts, "ni0");
    assert_qmp_error_contains(qtest_qmp(qts,
        "{'execute':'migrate','arguments':{'uri':'file:/dev/null'}}"),
                              "NetInfo server state is not migratable");
    qtest_quit(qts);

    qts = start_vm("-only-migratable");

    assert_qmp_error_contains(object_add_response(qts, "ni0", "nextnet",
                                                  NULL, NULL, false, false),
                              "NetInfo server state is not migratable");
    assert_qmp_error_contains(object_add_response(qts, "ni1", "nextnet",
                                                  NULL, NULL, false, false),
                              "NetInfo server state is not migratable");
    qtest_quit(qts);
}

static void test_no_host_listener(void)
{
    QTestState *qts = start_vm("");

    object_add(qts, "ni0");
    assert_no_proc_listener(qtest_pid(qts), "udp", NETINFO_BINDER_UDP_PORT);
    assert_no_proc_listener(qtest_pid(qts), "udp", NETINFO_DATABASE_UDP_PORT);
    assert_no_proc_listener(qtest_pid(qts), "tcp", NETINFO_BINDER_TCP_PORT);
    assert_no_proc_listener(qtest_pid(qts), "tcp", NETINFO_DATABASE_TCP_PORT);
    object_del(qts, "ni0");
    qtest_quit(qts);
}

static void test_custom_binder_registration_ports(void)
{
    g_autofree char *input_path = g_strdup_printf("%s/binder-input",
                                                  test_root);
    g_autofree char *output_path = g_strdup_printf("%s/binder-output",
                                                   test_root);
    uint8_t frame[4096];
    uint8_t host_mac[6];
    const uint8_t *rpc;
    size_t frame_len;
    size_t rpc_len;
    int input_fd;
    int output_fd;
    QTestState *qts;
    gint64 deadline;
    bool received;
    size_t offset;

    /* Inject real guest Ethernet frames so the probe exercises slirp RPC. */
    qts = qtest_initf(
        "-machine virt -nodefaults "
        "-netdev user,id=nextnet "
        "-device virtio-net-device,netdev=nextnet "
        "-chardev socket,id=binderin,path=%s,server=on,wait=off "
        "-chardev socket,id=binderout,path=%s,server=on,wait=off "
        "-object filter-redirector,id=binderinj,netdev=nextnet,"
        "queue=rx,indev=binderin "
        "-object filter-redirector,id=binderout,netdev=nextnet,"
        "queue=tx,outdev=binderout",
        input_path, output_path);
    input_fd = connect_redirector(input_path);
    output_fd = connect_redirector(output_path);
    qtest_qmp_assert_success(qts, "{ 'execute' : 'query-status'}");
    assert_qmp_success(object_add_response_with_database_tcp_port(
        qts, "ni0", "nextnet", NULL, "custom", false, true,
        NETINFO_CUSTOM_DATABASE_TCP_PORT));

    frame_len = build_arp_request(frame);
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + NETINFO_RPC_TIMEOUT_US;
    received = false;
    for (unsigned frame_count = 0; frame_count < NETINFO_RPC_MAX_FRAMES;
         frame_count++) {
        if (!recv_redirector_frame(output_fd, frame, sizeof(frame), deadline,
                                   &frame_len)) {
            break;
        }
        if (is_arp_reply(frame, frame_len)) {
            received = true;
            break;
        }
    }
    g_assert_true(received);
    memcpy(host_mac, frame + 6, sizeof(host_mac));

    frame_len = build_binder_call(frame, host_mac,
                                  NETINFO_CUSTOM_BINDER_UDP_PORT, 1,
                                  NIBIND_GETREGISTER, "custom");
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + NETINFO_RPC_TIMEOUT_US;
    g_assert_true(recv_binder_reply(output_fd, frame, sizeof(frame), deadline,
                                    host_mac, NETINFO_CUSTOM_BINDER_UDP_PORT,
                                    1, &rpc, &rpc_len));
    assert_binder_reply_header(rpc, rpc_len, 1);
    g_assert_cmpuint(rpc_len, ==, 36);
    g_assert_cmpuint(get_be32(rpc + 24), ==, NI_OK);
    g_assert_cmpuint(get_be32(rpc + 28), ==,
                     NETINFO_CUSTOM_DATABASE_UDP_PORT);
    g_assert_cmpuint(get_be32(rpc + 32), ==,
                     NETINFO_CUSTOM_DATABASE_TCP_PORT);

    frame_len = build_binder_call(frame, host_mac,
                                  NETINFO_CUSTOM_BINDER_UDP_PORT, 2,
                                  NIBIND_LISTREG, NULL);
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + NETINFO_RPC_TIMEOUT_US;
    g_assert_true(recv_binder_reply(output_fd, frame, sizeof(frame), deadline,
                                    host_mac, NETINFO_CUSTOM_BINDER_UDP_PORT,
                                    2, &rpc, &rpc_len));
    assert_binder_reply_header(rpc, rpc_len, 2);
    g_assert_cmpuint(rpc_len, ==, 52);
    g_assert_cmpuint(get_be32(rpc + 24), ==, NI_OK);
    g_assert_cmpuint(get_be32(rpc + 28), ==, 1);
    offset = 32;
    g_assert_cmpuint(get_be32(rpc + offset), ==, strlen("custom"));
    offset += sizeof(uint32_t);
    g_assert_cmpmem(rpc + offset, strlen("custom"), "custom",
                    strlen("custom"));
    offset += QEMU_ALIGN_UP(strlen("custom"), sizeof(uint32_t));
    g_assert_cmpuint(get_be32(rpc + offset), ==,
                     NETINFO_CUSTOM_DATABASE_UDP_PORT);
    offset += sizeof(uint32_t);
    g_assert_cmpuint(get_be32(rpc + offset), ==,
                     NETINFO_CUSTOM_DATABASE_TCP_PORT);
    offset += sizeof(uint32_t);
    g_assert_cmpuint(offset, ==, rpc_len);

    object_del(qts, "ni0");
    close(input_fd);
    close(output_fd);
    qtest_quit(qts);
    g_unlink(input_path);
    g_unlink(output_path);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    test_root = g_dir_make_tmp("qemu-netinfo-object-XXXXXX", NULL);
    g_assert_nonnull(test_root);

    qtest_add_func("netinfo-server-object/zero-config-creation",
                   test_zero_config_creation);
    qtest_add_func("netinfo-server-object/cli-creation", test_cli_creation);
    qtest_add_func("netinfo-server-object/json5-creation",
                   test_json5_creation);
    qtest_add_func("netinfo-server-object/missing-and-wrong-netdev",
                   test_missing_and_wrong_netdev);
    qtest_add_func("netinfo-server-object/duplicate-object",
                   test_duplicate_object);
    qtest_add_func("netinfo-server-object/netdev-isolation",
                   test_netdev_isolation);
    qtest_add_func("netinfo-server-object/port-collision-unwind",
                   test_port_collision_unwind);
    qtest_add_func("netinfo-server-object/delete-recreate-reset",
                   test_delete_recreate_reset);
    qtest_add_func("netinfo-server-object/config-domain-tag-writable",
                   test_config_domain_tag_and_writable);
    qtest_add_func("netinfo-server-object/migration-blocker",
                   test_migration_blocker);
    qtest_add_func("netinfo-server-object/no-host-listener",
                   test_no_host_listener);
    qtest_add_func("netinfo-server-object/custom-binder-registration-ports",
                   test_custom_binder_registration_ports);
    ret = g_test_run();

    g_rmdir(test_root);
    g_free(test_root);
    return ret;
}
