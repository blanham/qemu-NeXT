/*
 * Guest-only NFS server user-creatable object tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qapi/error.h"
#include "qemu/sockets.h"
#include "qobject/qdict.h"

#include <poll.h>

static char *test_root;

#define RPC_TEST_PROGRAM 200001U
#define RPC_TEST_VERSION 1U
#define RPC_TEST_PORT 4001U
#define PMAP_PROGRAM 100000U
#define PMAP_VERSION 2U
#define PMAP_PORT 111U
#define PMAP_DUMP 4U
#define UDP_PROTOCOL 17U
#define TCP_PROTOCOL 6U
#define GUEST_IP 0x0a00020fU
#define SLIRP_HOST_IP 0x0a000202U
#define GUEST_RPC_PORT 40000U
#define RPC_TEST_XID 0x12345678U
#define RPC_TEST_TIMEOUT_US (5 * G_TIME_SPAN_SECOND)
#define RPC_TEST_MAX_FRAMES 64U
#define NFS_PROGRAM 100003U
#define NFS_VERSION 3U
#define NFS_PORT 2049U
#define NFS_TCP_GUEST_PORT 40001U
#define NFS_TCP_XID 0x23456789U

#define ETHERNET_HEADER_LEN 14
#define IPV4_HEADER_LEN 20
#define UDP_HEADER_LEN 8

static const uint8_t guest_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00,
                                      0x01 };

typedef struct RpcMapping {
    uint32_t program;
    uint32_t version;
    uint32_t protocol;
    uint32_t port;
} RpcMapping;

typedef struct TcpPacket {
    uint32_t sequence;
    uint32_t acknowledgment;
    uint8_t flags;
    const uint8_t *payload;
    size_t payload_len;
} TcpPacket;

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

static uint16_t tcp_checksum(uint32_t source, uint32_t destination,
                             const uint8_t *tcp, size_t tcp_len)
{
    g_autofree uint8_t *checksum = g_malloc(12 + tcp_len);

    put_be32(checksum, source);
    put_be32(checksum + 4, destination);
    checksum[8] = 0;
    checksum[9] = TCP_PROTOCOL;
    put_be16(checksum + 10, tcp_len);
    memcpy(checksum + 12, tcp, tcp_len);
    return internet_checksum(checksum, 12 + tcp_len);
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
    memcpy(frame + 6, guest_mac, sizeof(guest_mac));
    put_be16(frame + 12, 0x0806);
    put_be16(frame + 14, 1);
    put_be16(frame + 16, 0x0800);
    frame[18] = 6;
    frame[19] = 4;
    put_be16(frame + 20, 1);
    memcpy(frame + 22, guest_mac, sizeof(guest_mac));
    put_be32(frame + 28, GUEST_IP);
    put_be32(frame + 38, SLIRP_HOST_IP);
    return 42;
}

static size_t build_pmap_dump(uint8_t *frame, const uint8_t *host_mac)
{
    uint8_t *ip = frame + ETHERNET_HEADER_LEN;
    uint8_t *udp = ip + IPV4_HEADER_LEN;
    uint8_t *rpc = udp + UDP_HEADER_LEN;
    const size_t rpc_len = 40;
    const size_t ip_len = IPV4_HEADER_LEN + UDP_HEADER_LEN + rpc_len;

    memset(frame, 0, ETHERNET_HEADER_LEN + ip_len);
    memcpy(frame, host_mac, 6);
    memcpy(frame + 6, guest_mac, sizeof(guest_mac));
    put_be16(frame + 12, 0x0800);

    ip[0] = 0x45;
    put_be16(ip + 2, ip_len);
    ip[8] = 64;
    ip[9] = UDP_PROTOCOL;
    put_be32(ip + 12, GUEST_IP);
    put_be32(ip + 16, SLIRP_HOST_IP);
    put_be16(ip + 10, internet_checksum(ip, IPV4_HEADER_LEN));

    put_be16(udp, GUEST_RPC_PORT);
    put_be16(udp + 2, PMAP_PORT);
    put_be16(udp + 4, UDP_HEADER_LEN + rpc_len);
    /* UDP checksum zero means no checksum for IPv4. */

    put_be32(rpc, RPC_TEST_XID);
    put_be32(rpc + 4, 0);              /* CALL */
    put_be32(rpc + 8, 2);              /* RPC version */
    put_be32(rpc + 12, PMAP_PROGRAM);
    put_be32(rpc + 16, PMAP_VERSION);
    put_be32(rpc + 20, PMAP_DUMP);
    put_be32(rpc + 24, 0);             /* NULL credential flavor */
    put_be32(rpc + 28, 0);             /* NULL credential length */
    put_be32(rpc + 32, 0);             /* NULL verifier flavor */
    put_be32(rpc + 36, 0);             /* NULL verifier length */
    return ETHERNET_HEADER_LEN + ip_len;
}

static bool is_arp_reply(const uint8_t *frame, size_t len)
{
    if (len < 42 || get_be16(frame + 12) != 0x0806 ||
        memcmp(frame, guest_mac, sizeof(guest_mac)) ||
        get_be16(frame + 14) != 1 || get_be16(frame + 16) != 0x0800 ||
        frame[18] != 6 || frame[19] != 4 || get_be16(frame + 20) != 2 ||
        get_be32(frame + 28) != SLIRP_HOST_IP ||
        memcmp(frame + 32, guest_mac, sizeof(guest_mac)) ||
        get_be32(frame + 38) != GUEST_IP ||
        memcmp(frame + 6, frame + 22, sizeof(guest_mac))) {
        return false;
    }
    return true;
}

static bool is_pmap_reply(const uint8_t *frame, size_t len,
                          const uint8_t *host_mac,
                          const uint8_t **rpc, size_t *rpc_len)
{
    size_t ip_header_len;
    size_t ip_payload_len;
    size_t udp_len;
    const uint8_t *ip;
    const uint8_t *udp;
    const uint8_t *checksum_data;
    g_autofree uint8_t *checksum_buf = NULL;

    if (len < ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + UDP_HEADER_LEN ||
        get_be16(frame + 12) != 0x0800 ||
        memcmp(frame + 6, host_mac, sizeof(guest_mac)) ||
        memcmp(frame, guest_mac, sizeof(guest_mac))) {
        return false;
    }
    ip = frame + ETHERNET_HEADER_LEN;
    ip_header_len = (ip[0] & 0x0f) * 4;
    if ((ip[0] >> 4) != 4 || ip_header_len < IPV4_HEADER_LEN ||
        len < ETHERNET_HEADER_LEN + ip_header_len + UDP_HEADER_LEN ||
        get_be16(ip + 2) < ip_header_len + UDP_HEADER_LEN ||
        get_be16(ip + 2) > len - ETHERNET_HEADER_LEN ||
        internet_checksum(ip, ip_header_len) != 0 ||
        (get_be16(ip + 6) & 0x3fff) || ip[9] != UDP_PROTOCOL ||
        get_be32(ip + 12) != SLIRP_HOST_IP ||
        get_be32(ip + 16) != GUEST_IP) {
        return false;
    }
    ip_payload_len = get_be16(ip + 2) - ip_header_len;
    udp = ip + ip_header_len;
    udp_len = get_be16(udp + 4);
    if (udp_len < UDP_HEADER_LEN || udp_len != ip_payload_len ||
        len < ETHERNET_HEADER_LEN + ip_header_len + udp_len ||
        get_be16(udp) != PMAP_PORT || get_be16(udp + 2) != GUEST_RPC_PORT) {
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
    *rpc = udp + UDP_HEADER_LEN;
    *rpc_len = udp_len - UDP_HEADER_LEN;
    return true;
}

static size_t parse_pmap_dump(const uint8_t *rpc, size_t len,
                              RpcMapping *mappings, size_t capacity)
{
    size_t offset = 24;
    size_t count = 0;

    g_assert_cmpuint(len, >=, offset);
    g_assert_cmpuint(get_be32(rpc), ==, RPC_TEST_XID);
    g_assert_cmpuint(get_be32(rpc + 4), ==, 1); /* REPLY */
    g_assert_cmpuint(get_be32(rpc + 8), ==, 0); /* MSG_ACCEPTED */
    g_assert_cmpuint(get_be32(rpc + 12), ==, 0); /* AUTH_NULL */
    g_assert_cmpuint(get_be32(rpc + 16), ==, 0); /* verifier length */
    g_assert_cmpuint(get_be32(rpc + 20), ==, 0); /* SUCCESS */
    while (true) {
        g_assert_cmpuint(offset + 4, <=, len);
        g_assert_cmpuint(get_be32(rpc + offset), <=, 1);
        if (!get_be32(rpc + offset)) {
            offset += 4;
            break;
        }
        g_assert_cmpuint(offset + 20, <=, len);
        g_assert_cmpuint(count, <, capacity);
        mappings[count].program = get_be32(rpc + offset + 4);
        mappings[count].version = get_be32(rpc + offset + 8);
        mappings[count].protocol = get_be32(rpc + offset + 12);
        mappings[count].port = get_be32(rpc + offset + 16);
        count++;
        offset += 20;
    }
    g_assert_cmpuint(offset, ==, len);
    return count;
}

static size_t build_pmap_success_reply(uint8_t *frame, const uint8_t *host_mac)
{
    uint8_t *ip = frame + ETHERNET_HEADER_LEN;
    uint8_t *udp = ip + IPV4_HEADER_LEN;
    uint8_t *rpc = udp + UDP_HEADER_LEN;
    const size_t rpc_len = 24;
    const size_t ip_len = IPV4_HEADER_LEN + UDP_HEADER_LEN + rpc_len;

    memset(frame, 0, ETHERNET_HEADER_LEN + ip_len);
    memcpy(frame, guest_mac, sizeof(guest_mac));
    memcpy(frame + 6, host_mac, sizeof(guest_mac));
    put_be16(frame + 12, 0x0800);

    ip[0] = 0x45;
    put_be16(ip + 2, ip_len);
    ip[8] = 64;
    ip[9] = UDP_PROTOCOL;
    put_be32(ip + 12, SLIRP_HOST_IP);
    put_be32(ip + 16, GUEST_IP);
    put_be16(ip + 10, internet_checksum(ip, IPV4_HEADER_LEN));

    put_be16(udp, PMAP_PORT);
    put_be16(udp + 2, GUEST_RPC_PORT);
    put_be16(udp + 4, UDP_HEADER_LEN + rpc_len);

    put_be32(rpc, RPC_TEST_XID);
    put_be32(rpc + 4, 1);              /* REPLY */
    put_be32(rpc + 8, 0);              /* MSG_ACCEPTED */
    put_be32(rpc + 12, 0);             /* AUTH_NULL */
    put_be32(rpc + 16, 0);             /* verifier length */
    put_be32(rpc + 20, 0);             /* SUCCESS */
    return ETHERNET_HEADER_LEN + ip_len;
}

static size_t build_tcp_frame(uint8_t *frame, const uint8_t *host_mac,
                              uint16_t source_port, uint16_t destination_port,
                              uint32_t sequence, uint32_t acknowledgment,
                              uint8_t flags, const uint8_t *payload,
                              size_t payload_len)
{
    uint8_t *ip = frame + ETHERNET_HEADER_LEN;
    uint8_t *tcp = ip + IPV4_HEADER_LEN;
    size_t tcp_len = 20 + payload_len;
    size_t ip_len = IPV4_HEADER_LEN + tcp_len;

    g_assert_cmpuint(payload_len, <=, 4096 - ETHERNET_HEADER_LEN -
                     IPV4_HEADER_LEN - 20);
    memset(frame, 0, ETHERNET_HEADER_LEN + ip_len);
    memcpy(frame, host_mac, sizeof(guest_mac));
    memcpy(frame + 6, guest_mac, sizeof(guest_mac));
    put_be16(frame + 12, 0x0800);

    ip[0] = 0x45;
    put_be16(ip + 2, ip_len);
    ip[8] = 64;
    ip[9] = TCP_PROTOCOL;
    put_be32(ip + 12, GUEST_IP);
    put_be32(ip + 16, SLIRP_HOST_IP);
    put_be16(ip + 10, internet_checksum(ip, IPV4_HEADER_LEN));

    put_be16(tcp, source_port);
    put_be16(tcp + 2, destination_port);
    put_be32(tcp + 4, sequence);
    put_be32(tcp + 8, acknowledgment);
    tcp[12] = 5 << 4;
    tcp[13] = flags;
    put_be16(tcp + 14, UINT16_MAX);
    put_be16(tcp + 16, 0);
    put_be16(tcp + 18, 0);
    if (payload_len) {
        memcpy(tcp + 20, payload, payload_len);
    }
    put_be16(tcp + 16, tcp_checksum(GUEST_IP, SLIRP_HOST_IP,
                                    tcp, tcp_len));
    return ETHERNET_HEADER_LEN + ip_len;
}

static bool parse_tcp_reply(const uint8_t *frame, size_t len,
                            const uint8_t *host_mac, TcpPacket *packet)
{
    const uint8_t *ip;
    const uint8_t *tcp;
    size_t ip_header_len;
    size_t tcp_header_len;
    size_t ip_len;

    if (len < ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + 20 ||
        get_be16(frame + 12) != 0x0800 ||
        memcmp(frame, guest_mac, sizeof(guest_mac)) ||
        memcmp(frame + 6, host_mac, sizeof(guest_mac))) {
        return false;
    }
    ip = frame + ETHERNET_HEADER_LEN;
    ip_header_len = (ip[0] & 0x0f) * 4;
    if ((ip[0] >> 4) != 4 || ip_header_len < IPV4_HEADER_LEN ||
        len < ETHERNET_HEADER_LEN + ip_header_len + 20 ||
        internet_checksum(ip, ip_header_len) != 0 || ip[9] != TCP_PROTOCOL ||
        get_be32(ip + 12) != SLIRP_HOST_IP ||
        get_be32(ip + 16) != GUEST_IP) {
        return false;
    }
    ip_len = get_be16(ip + 2);
    if (ip_len < ip_header_len + 20 ||
        ip_len > len - ETHERNET_HEADER_LEN ||
        (get_be16(ip + 6) & 0x3fff)) {
        return false;
    }
    tcp = ip + ip_header_len;
    if (get_be16(tcp) != NFS_PORT ||
        get_be16(tcp + 2) != NFS_TCP_GUEST_PORT) {
        return false;
    }
    tcp_header_len = (tcp[12] >> 4) * 4;
    if (tcp_header_len < 20 || tcp_header_len > ip_len - ip_header_len) {
        return false;
    }
    packet->sequence = get_be32(tcp + 4);
    packet->acknowledgment = get_be32(tcp + 8);
    packet->flags = tcp[13];
    packet->payload = tcp + tcp_header_len;
    packet->payload_len = ip_len - ip_header_len - tcp_header_len;
    return true;
}

static bool recv_tcp_reply(int fd, uint8_t *frame, size_t capacity,
                           gint64 deadline, const uint8_t *host_mac,
                           TcpPacket *packet)
{
    size_t frame_len;

    while (recv_redirector_frame(fd, frame, capacity, deadline, &frame_len)) {
        if (parse_tcp_reply(frame, frame_len, host_mac, packet)) {
            return true;
        }
    }
    return false;
}

static size_t build_nfs_null_record(uint8_t *record)
{
    uint8_t rpc[64];
    size_t rpc_len = 40;

    put_be32(rpc, NFS_TCP_XID);
    put_be32(rpc + 4, 0);              /* CALL */
    put_be32(rpc + 8, 2);              /* RPC version */
    put_be32(rpc + 12, NFS_PROGRAM);
    put_be32(rpc + 16, NFS_VERSION);
    put_be32(rpc + 20, 0);             /* NFS NULL */
    put_be32(rpc + 24, 0);             /* NULL credential flavor */
    put_be32(rpc + 28, 0);             /* NULL credential length */
    put_be32(rpc + 32, 0);             /* NULL verifier flavor */
    put_be32(rpc + 36, 0);             /* NULL verifier length */
    put_be32(record, UINT32_C(0x80000000) | rpc_len);
    memcpy(record + 4, rpc, rpc_len);
    return 4 + rpc_len;
}

static void assert_nfs_null_reply(const TcpPacket *packet)
{
    g_assert_cmpuint(packet->payload_len, >=, 28);
    g_assert_cmpuint(get_be32(packet->payload), ==, 0x80000000U | 24U);
    g_assert_cmpuint(get_be32(packet->payload + 4), ==, NFS_TCP_XID);
    g_assert_cmpuint(get_be32(packet->payload + 8), ==, 1); /* REPLY */
    g_assert_cmpuint(get_be32(packet->payload + 12), ==, 0); /* ACCEPTED */
    g_assert_cmpuint(get_be32(packet->payload + 16), ==, 0); /* AUTH_NULL */
    g_assert_cmpuint(get_be32(packet->payload + 20), ==, 0);
    g_assert_cmpuint(get_be32(packet->payload + 24), ==, 0); /* SUCCESS */
}

static void test_pmap_dump_rejects_noncanonical_boolean(void)
{
    if (g_test_subprocess()) {
        uint8_t rpc[48] = { 0 };
        RpcMapping mapping;

        put_be32(rpc, RPC_TEST_XID);
        put_be32(rpc + 4, 1);           /* REPLY */
        put_be32(rpc + 8, 0);           /* MSG_ACCEPTED */
        put_be32(rpc + 12, 0);          /* AUTH_NULL */
        put_be32(rpc + 16, 0);          /* verifier length */
        put_be32(rpc + 20, 0);          /* SUCCESS */
        put_be32(rpc + 24, 2);          /* invalid TRUE */
        put_be32(rpc + 28, RPC_TEST_PROGRAM);
        put_be32(rpc + 32, RPC_TEST_VERSION);
        put_be32(rpc + 36, UDP_PROTOCOL);
        put_be32(rpc + 40, RPC_TEST_PORT);
        put_be32(rpc + 44, 0);          /* FALSE */
        parse_pmap_dump(rpc, sizeof(rpc), &mapping, 1);
        return;
    }

    g_test_trap_subprocess(NULL, 0, 0);
    g_test_trap_assert_failed();
}

static void test_pmap_reply_rejects_ipv4_fragments(void)
{
    static const uint8_t host_mac[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0xdd,
                                         0xee };
    uint8_t frame[ETHERNET_HEADER_LEN + IPV4_HEADER_LEN + UDP_HEADER_LEN +
                  24];
    const uint8_t *rpc;
    size_t rpc_len;
    uint8_t *ip = frame + ETHERNET_HEADER_LEN;

    build_pmap_success_reply(frame, host_mac);
    put_be16(ip + 6, 0x2000);          /* IPv4 more-fragments flag */
    put_be16(ip + 10, 0);
    put_be16(ip + 10, internet_checksum(ip, IPV4_HEADER_LEN));
    g_assert_false(is_pmap_reply(frame, sizeof(frame), host_mac,
                                 &rpc, &rpc_len));
}

static QTestState *start_vm(const char *extra)
{
    return qtest_initf("-machine none -nodefaults "
                       "-fsdev local,id=root,path=%s,"
                       "security_model=mapped-xattr "
                       "-netdev user,id=nextnet %s",
                       test_root, extra ?: "");
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

static QDict *object_add_response(QTestState *qts, const char *id,
                                  const char *fsdev, const char *netdev,
                                  const char *root_path, bool writable)
{
    return qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'nfs-server','id':%s,'fsdev':%s,'netdev':%s,"
        "'root-path':%s,'writable':%i}}",
        id, fsdev, netdev, root_path, writable);
}

static void object_add(QTestState *qts, const char *id, bool success)
{
    QDict *response = object_add_response(qts, id, "root", "nextnet", "/",
                                          false);

    if (success) {
        assert_qmp_success(response);
    } else {
        g_assert_true(qdict_haskey(response, "error"));
        qobject_unref(response);
    }
}

static void object_del(QTestState *qts, const char *id)
{
    assert_qmp_success(qtest_qmp(qts,
        "{'execute':'object-del','arguments':{'id':%s}}", id));
}

static void test_cli_creation(void)
{
    QTestState *qts = start_vm(
        "-object nfs-server,id=nfs,fsdev=root,netdev=nextnet,writable=on");

    qtest_quit(qts);
}

static void test_defaults_delete_recreate_reset(void)
{
    QTestState *qts = start_vm("");
    QDict *response;

    object_add(qts, "nfs", true);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/nfs','property':'root-path'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "/");
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/nfs','property':'writable'}}");
    g_assert_false(qdict_get_bool(response, "return"));
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-set','arguments':{"
        "'path':'/objects/nfs','property':'root-path','value':'/changed'}}");
    assert_qmp_error_contains(response, "cannot change");
    response = qtest_qmp(qts,
        "{'execute':'migrate','arguments':{'uri':'file:/dev/null'}}");
    assert_qmp_error_contains(response, "not migratable");
    qtest_system_reset(qts);
    object_del(qts, "nfs");
    object_add(qts, "nfs", true);
    object_del(qts, "nfs");
    qtest_quit(qts);
}

static void test_validation_and_unwind(void)
{
    QTestState *qts = start_vm("-netdev socket,id=socketnet,listen=:0");
    g_autofree char *max_root = g_strnfill(255, 'a');
    g_autofree char *too_long_root = g_strnfill(256, 'a');
    QDict *response;

    max_root[0] = '/';
    too_long_root[0] = '/';

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'nfs-server','id':'missing'}}");
    assert_qmp_error_contains(response, "fsdev");
    response = object_add_response(qts, "badfs", "missing", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "fsdev");
    response = object_add_response(qts, "badnet", "root", "missing", "/",
                                   false);
    assert_qmp_error_contains(response, "netdev");
    response = object_add_response(qts, "notuser", "root", "socketnet", "/",
                                   false);
    assert_qmp_error_contains(response, "user-mode");
    response = object_add_response(qts, "relative", "root", "nextnet",
                                   "relative", false);
    assert_qmp_error_contains(response, "absolute");
    response = object_add_response(qts, "empty", "root", "nextnet", "",
                                   false);
    assert_qmp_error_contains(response, "absolute");
    response = object_add_response(qts, "too-long", "root", "nextnet",
                                   too_long_root, false);
    assert_qmp_error_contains(response, "1-255");
    response = object_add_response(qts, "unsupported-root", "root", "nextnet",
                                   "/export", false);
    assert_qmp_error_contains(response, "must be /");
    response = object_add_response(qts, "max-root", "root", "nextnet",
                                   max_root, false);
    assert_qmp_error_contains(response, "must be /");

    object_add(qts, "owner", true);
    response = object_add_response(qts, "collision", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "owns the BOOTP root lease");
    object_del(qts, "owner");
    object_add(qts, "after-unwind", true);
    object_del(qts, "after-unwind");
    qtest_quit(qts);
}

static void test_readonly_and_ipv4_disabled(void)
{
    QTestState *qts = qtest_initf(
        "-machine none -nodefaults "
        "-fsdev local,id=readonly,path=%s,security_model=mapped-xattr,"
        "readonly=on -netdev user,id=nextnet",
        test_root);
    QDict *response = object_add_response(qts, "nfs", "readonly", "nextnet",
                                          "/", true);

    assert_qmp_error_contains(response, "writable fsdev");
    qtest_quit(qts);

    qts = qtest_initf(
        "-machine none -nodefaults "
        "-fsdev local,id=root,path=%s,security_model=mapped-xattr "
        "-netdev user,id=nextnet,ipv4=off,ipv6=on",
        test_root);
    response = object_add_response(qts, "nfs", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "IPv4 is disabled");
    qtest_quit(qts);
}

static void test_migration_blocker_unwind(void)
{
    QTestState *qts = start_vm("-only-migratable");
    QDict *response;

    response = object_add_response(qts, "first", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "only-migratable");
    response = object_add_response(qts, "second", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "only-migratable");
    qtest_quit(qts);
}

static void test_no_host_listener(void)
{
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(2049),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    QTestState *qts;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(bind(fd, (struct sockaddr *)&address, sizeof(address)),
                    ==, 0);
    qts = start_vm("");
    object_add(qts, "nfs", true);
    object_del(qts, "nfs");
    qtest_quit(qts);
    close(fd);
}

static void test_shared_pmap_dump(void)
{
    g_autofree char *input_path = g_strdup_printf("%s/rpc-input", test_root);
    g_autofree char *output_path = g_strdup_printf("%s/rpc-output", test_root);
    uint8_t frame[4096];
    uint8_t host_mac[6];
    RpcMapping mappings[8];
    const uint8_t *rpc = NULL;
    size_t frame_len;
    size_t rpc_len = 0;
    size_t mapping_count;
    int input_fd;
    int output_fd;
    QTestState *qts;
    gint64 deadline;
    unsigned int frame_count;
    bool received;
    static const RpcMapping expected[] = {
        { 100005, 1, UDP_PROTOCOL, 635 },
        { 100005, 2, UDP_PROTOCOL, 635 },
        { 100005, 3, UDP_PROTOCOL, 635 },
        { 100003, 2, UDP_PROTOCOL, 2049 },
        { 100003, 2, TCP_PROTOCOL, 2049 },
        { 100003, 3, UDP_PROTOCOL, 2049 },
        { 100003, 3, TCP_PROTOCOL, 2049 },
        { RPC_TEST_PROGRAM, RPC_TEST_VERSION, UDP_PROTOCOL, RPC_TEST_PORT },
    };

    /*
     * The registry lives in QEMU's process, while this qtest is its client.
     * Ask the NFS object to add one test-only registration so this exercises
     * one real PMAP DUMP containing two registrations.
     */
    g_setenv("QEMU_NFS_TEST_RPC", "1", true);
    qts = qtest_initf(
        "-machine virt -nodefaults "
        "-fsdev local,id=root,path=%s,security_model=mapped-xattr "
        "-netdev user,id=nextnet "
        "-device virtio-net-device,netdev=nextnet "
        "-chardev socket,id=rpcin,path=%s,server=on,wait=off "
        "-chardev socket,id=rpcout,path=%s,server=on,wait=off "
        "-object filter-redirector,id=rpcinj,netdev=nextnet,"
        "queue=rx,indev=rpcin "
        "-object filter-redirector,id=rpccap,netdev=nextnet,"
        "queue=tx,outdev=rpcout",
        test_root, input_path, output_path);
    input_fd = connect_redirector(input_path);
    output_fd = connect_redirector(output_path);
    qtest_qmp_assert_success(qts, "{ 'execute' : 'query-status'}");
    object_add(qts, "nfs", true);
    g_unsetenv("QEMU_NFS_TEST_RPC");

    frame_len = build_arp_request(frame);
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + RPC_TEST_TIMEOUT_US;
    received = false;
    for (frame_count = 0; frame_count < RPC_TEST_MAX_FRAMES;
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

    frame_len = build_pmap_dump(frame, host_mac);
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + RPC_TEST_TIMEOUT_US;
    received = false;
    for (frame_count = 0; frame_count < RPC_TEST_MAX_FRAMES;
         frame_count++) {
        if (!recv_redirector_frame(output_fd, frame, sizeof(frame), deadline,
                                   &frame_len)) {
            break;
        }
        if (is_pmap_reply(frame, frame_len, host_mac, &rpc, &rpc_len)) {
            received = true;
            break;
        }
    }
    g_assert_true(received);
    mapping_count = parse_pmap_dump(rpc, rpc_len, mappings,
                                    G_N_ELEMENTS(mappings));
    g_assert_cmpuint(mapping_count, ==, G_N_ELEMENTS(expected));
    for (size_t i = 0; i < G_N_ELEMENTS(expected); i++) {
        g_assert_cmpuint(mappings[i].program, ==, expected[i].program);
        g_assert_cmpuint(mappings[i].version, ==, expected[i].version);
        g_assert_cmpuint(mappings[i].protocol, ==, expected[i].protocol);
        g_assert_cmpuint(mappings[i].port, ==, expected[i].port);
    }

    object_del(qts, "nfs");
    close(input_fd);
    close(output_fd);
    qtest_quit(qts);
    g_unlink(input_path);
    g_unlink(output_path);
}

static void test_tcp_nfs_rpc(void)
{
    g_autofree char *input_path = g_strdup_printf("%s/tcp-rpc-input",
                                                   test_root);
    g_autofree char *output_path = g_strdup_printf("%s/tcp-rpc-output",
                                                    test_root);
    uint8_t frame[4096];
    uint8_t host_mac[6];
    uint8_t record[68];
    TcpPacket packet;
    size_t frame_len;
    size_t record_len;
    uint32_t guest_sequence = 0x10000000;
    uint32_t host_sequence;
    gint64 deadline;
    int input_fd;
    int output_fd;
    QTestState *qts;

    qts = qtest_initf(
        "-machine virt -nodefaults "
        "-fsdev local,id=root,path=%s,security_model=mapped-xattr "
        "-netdev user,id=nextnet "
        "-device virtio-net-device,netdev=nextnet "
        "-chardev socket,id=rpcin,path=%s,server=on,wait=off "
        "-chardev socket,id=rpcout,path=%s,server=on,wait=off "
        "-object filter-redirector,id=rpcinj,netdev=nextnet,"
        "queue=rx,indev=rpcin "
        "-object filter-redirector,id=rpccap,netdev=nextnet,"
        "queue=tx,outdev=rpcout",
        test_root, input_path, output_path);
    input_fd = connect_redirector(input_path);
    output_fd = connect_redirector(output_path);
    object_add(qts, "nfs", true);

    frame_len = build_arp_request(frame);
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + RPC_TEST_TIMEOUT_US;
    g_assert_true(recv_redirector_frame(output_fd, frame, sizeof(frame),
                                        deadline, &frame_len));
    g_assert_true(is_arp_reply(frame, frame_len));
    memcpy(host_mac, frame + 6, sizeof(host_mac));

    frame_len = build_tcp_frame(frame, host_mac, NFS_TCP_GUEST_PORT,
                                NFS_PORT, guest_sequence, 0, 0x02, NULL, 0);
    send_redirector_frame(input_fd, frame, frame_len);
    deadline = g_get_monotonic_time() + RPC_TEST_TIMEOUT_US;
    g_assert_true(recv_tcp_reply(output_fd, frame, sizeof(frame), deadline,
                                 host_mac, &packet));
    g_assert_cmpuint(packet.flags & 0x12, ==, 0x12); /* SYN|ACK */
    g_assert_cmpuint(packet.acknowledgment, ==, guest_sequence + 1);
    host_sequence = packet.sequence;
    guest_sequence++;

    frame_len = build_tcp_frame(frame, host_mac, NFS_TCP_GUEST_PORT,
                                NFS_PORT, guest_sequence, host_sequence + 1,
                                0x10, NULL, 0);
    send_redirector_frame(input_fd, frame, frame_len);

    record_len = build_nfs_null_record(record);
    frame_len = build_tcp_frame(frame, host_mac, NFS_TCP_GUEST_PORT,
                                NFS_PORT, guest_sequence, host_sequence + 1,
                                0x18, record, record_len); /* PSH|ACK */
    send_redirector_frame(input_fd, frame, frame_len);
    guest_sequence += record_len;

    deadline = g_get_monotonic_time() + RPC_TEST_TIMEOUT_US;
    for (;;) {
        g_assert_true(recv_tcp_reply(output_fd, frame, sizeof(frame),
                                     deadline, host_mac, &packet));
        if (!packet.payload_len) {
            continue;
        }
        assert_nfs_null_reply(&packet);
        break;
    }

    frame_len = build_tcp_frame(frame, host_mac, NFS_TCP_GUEST_PORT,
                                NFS_PORT, guest_sequence,
                                packet.sequence + packet.payload_len, 0x10,
                                NULL, 0);
    send_redirector_frame(input_fd, frame, frame_len);

    object_del(qts, "nfs");
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
    test_root = g_dir_make_tmp("qemu-nfs-object-XXXXXX", NULL);
    g_assert_nonnull(test_root);

    qtest_add_func("nfs-server-object/cli-creation", test_cli_creation);
    qtest_add_func("nfs-server-object/defaults-delete-recreate-reset",
                   test_defaults_delete_recreate_reset);
    qtest_add_func("nfs-server-object/validation-unwind",
                   test_validation_and_unwind);
    qtest_add_func("nfs-server-object/readonly-ipv4-disabled",
                   test_readonly_and_ipv4_disabled);
    qtest_add_func("nfs-server-object/migration-blocker-unwind",
                   test_migration_blocker_unwind);
    qtest_add_func("nfs-server-object/no-host-listener",
                   test_no_host_listener);
    qtest_add_func("nfs-server-object/pmap-dump-rejects-noncanonical-boolean",
                   test_pmap_dump_rejects_noncanonical_boolean);
    qtest_add_func("nfs-server-object/pmap-reply-rejects-ipv4-fragments",
                   test_pmap_reply_rejects_ipv4_fragments);
    qtest_add_func("nfs-server-object/shared-pmap-dump",
                   test_shared_pmap_dump);
    qtest_add_func("nfs-server-object/tcp-nfs-rpc", test_tcp_nfs_rpc);
    ret = g_test_run();

    g_rmdir(test_root);
    g_free(test_root);
    return ret;
}
