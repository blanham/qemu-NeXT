/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-bootp-internal.h"
#include "net/slirp-udp-internal.h"
#include "qapi/error.h"

typedef struct FakeUdpListener {
    const QemuSlirpUdpBackendCallbacks *callbacks;
    void *callbacks_opaque;
    struct in_addr address;
    uint16_t port;
} FakeUdpListener;

typedef struct FakeUdpBackend {
    FakeUdpListener *listeners[4];
    size_t nlisteners;
    unsigned listens;
    unsigned removes;
    unsigned sends;
    struct sockaddr_in last_peer;
    uint8_t last_data[32];
    size_t last_len;
    int listen_result;
    int send_result;
} FakeUdpBackend;

static int fake_udp_listen(void *opaque, struct in_addr address, uint16_t port,
                           const QemuSlirpUdpBackendCallbacks *callbacks,
                           void *callbacks_opaque, void **backend_listener)
{
    FakeUdpBackend *backend = opaque;
    FakeUdpListener *listener;

    g_assert_cmpuint(backend->nlisteners, <,
                     G_N_ELEMENTS(backend->listeners));
    backend->listens++;
    if (backend->listen_result < 0) {
        return backend->listen_result;
    }
    listener = g_new0(FakeUdpListener, 1);
    listener->callbacks = callbacks;
    listener->callbacks_opaque = callbacks_opaque;
    listener->address = address;
    listener->port = port;
    backend->listeners[backend->nlisteners++] = listener;
    *backend_listener = listener;
    return 0;
}

static void fake_udp_remove(void *opaque, void *backend_listener)
{
    FakeUdpBackend *backend = opaque;
    FakeUdpListener *listener = backend_listener;

    backend->removes++;
    g_free(listener);
}

static int fake_udp_send(void *opaque, void *backend_listener,
                         const struct sockaddr_in *peer,
                         const uint8_t *data, size_t len)
{
    FakeUdpBackend *backend = opaque;

    g_assert_nonnull(backend_listener);
    g_assert_cmpuint(len, <=, sizeof(backend->last_data));
    backend->sends++;
    backend->last_peer = *peer;
    memcpy(backend->last_data, data, len);
    backend->last_len = len;
    return backend->send_result;
}

static const QemuSlirpUdpBackendOps udp_backend_ops = {
    .listen = fake_udp_listen,
    .listener_remove = fake_udp_remove,
    .send = fake_udp_send,
};

static struct in_addr ip(uint32_t value)
{
    return (struct in_addr) { htonl(value) };
}

static QemuSlirpUdpRegistry *new_udp_registry(FakeUdpBackend *backend,
                                               bool ipv4_enabled)
{
    return qemu_slirp_udp_registry_new(ipv4_enabled, ip(0x0a000202),
                                       &udp_backend_ops, backend);
}

typedef struct DatagramState {
    FakeUdpBackend *backend;
    QemuSlirpUdpListener *listener;
    struct sockaddr_in peer;
    uint8_t data[32];
    size_t len;
    unsigned calls;
    int send_result;
    bool remove;
    bool removed_inside_callback;
} DatagramState;

static void datagram(QemuSlirpUdpListener *listener,
                     const struct sockaddr_in *peer,
                     const uint8_t *data, size_t len, void *opaque)
{
    DatagramState *state = opaque;

    state->calls++;
    state->peer = *peer;
    g_assert_cmpuint(len, <=, sizeof(state->data));
    memcpy(state->data, data, len);
    state->len = len;
    state->send_result = qemu_slirp_udp_send(listener, peer, data, len);
    if (state->remove) {
        qemu_slirp_udp_listener_remove(listener);
        state->listener = NULL;
        state->removed_inside_callback = state->backend->removes != 0;
    }
}

static const QemuSlirpUdpListenerOps udp_listener_ops = {
    .datagram = datagram,
};

static void fake_udp_deliver(FakeUdpListener *listener,
                             const struct sockaddr_in *peer,
                             const uint8_t *data, size_t len)
{
    listener->callbacks->datagram(peer, data, len,
                                  listener->callbacks_opaque);
}

static void assert_udp_listen_fails(QemuSlirpUdpRegistry *registry,
                                    uint16_t port,
                                    const QemuSlirpUdpListenerOps *ops)
{
    QemuSlirpUdpListener *listener = (void *)0x1;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_udp_registry_listen(
                        registry, port, ops, NULL, &listener, &err), ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_udp_validation_and_fixed_host(void)
{
    FakeUdpBackend backend = {0};
    QemuSlirpUdpRegistry *registry = new_udp_registry(&backend, true);
    QemuSlirpUdpListener *one = NULL, *two = NULL, *three = NULL;
    DatagramState state = {.backend = &backend};
    Error *err = NULL;

    assert_udp_listen_fails(registry, 0, &udp_listener_ops);
    assert_udp_listen_fails(registry, 111, NULL);
    g_assert_cmpint(qemu_slirp_udp_registry_listen(
                        registry, 111, &udp_listener_ops, &state, &one, &err),
                    ==, 0);
    g_assert_cmphex(backend.listeners[0]->address.s_addr, ==,
                    ip(0x0a000202).s_addr);
    assert_udp_listen_fails(registry, 111, &udp_listener_ops);
    g_assert_cmpint(qemu_slirp_udp_registry_listen(
                        registry, 635, &udp_listener_ops, &state, &two, &err),
                    ==, 0);
    g_assert_cmpint(qemu_slirp_udp_registry_listen(
                        registry, 2049, &udp_listener_ops, &state, &three,
                        &err), ==, 0);
    g_assert_cmpuint(backend.listens, ==, 3);
    qemu_slirp_udp_listener_remove(one);
    qemu_slirp_udp_listener_remove(two);
    qemu_slirp_udp_listener_remove(three);
    g_assert_cmpuint(backend.removes, ==, 3);
    qemu_slirp_udp_registry_free(registry);
}

static void test_udp_ipv4_disabled_and_unavailable(void)
{
    FakeUdpBackend backend = {0};
    QemuSlirpUdpRegistry *registry = new_udp_registry(&backend, false);
    QemuSlirpUdpRegistry *unavailable = qemu_slirp_udp_registry_new(
        true, ip(0x0a000202), NULL, NULL);

    assert_udp_listen_fails(registry, 2049, &udp_listener_ops);
    assert_udp_listen_fails(unavailable, 2049, &udp_listener_ops);
    g_assert_cmpuint(backend.listens, ==, 0);
    qemu_slirp_udp_registry_free(registry);
    qemu_slirp_udp_registry_free(unavailable);
}

static void test_udp_delivery_reply_and_reentrant_remove(void)
{
    static const uint8_t payload[] = { 0x12, 0x34, 0x56, 0x78 };
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeUdpBackend backend = {0};
    DatagramState state = {.backend = &backend, .remove = true};
    QemuSlirpUdpRegistry *registry = new_udp_registry(&backend, true);
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_udp_registry_listen(
                        registry, 2049, &udp_listener_ops, &state,
                        &state.listener, &err), ==, 0);
    fake_udp_deliver(backend.listeners[0], &peer, payload, sizeof(payload));
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpmem(&state.peer, sizeof(state.peer), &peer, sizeof(peer));
    g_assert_cmpmem(state.data, state.len, payload, sizeof(payload));
    g_assert_cmpint(state.send_result, ==, 0);
    g_assert_cmpuint(backend.sends, ==, 1);
    g_assert_cmpmem(&backend.last_peer, sizeof(backend.last_peer),
                    &peer, sizeof(peer));
    g_assert_cmpmem(backend.last_data, backend.last_len,
                    payload, sizeof(payload));
    g_assert_false(state.removed_inside_callback);
    g_assert_cmpuint(backend.removes, ==, 1);
    qemu_slirp_udp_registry_free(registry);
}

static void test_udp_backend_invalidation(void)
{
    static const uint8_t payload = 0x5a;
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeUdpBackend backend = {0};
    DatagramState state = {.backend = &backend};
    QemuSlirpUdpRegistry *registry = new_udp_registry(&backend, true);
    QemuSlirpUdpListener *listener = NULL;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_udp_registry_listen(
                        registry, 2049, &udp_listener_ops, &state, &listener,
                        &err), ==, 0);
    qemu_slirp_udp_registry_invalidate(registry);
    g_assert_cmpuint(backend.removes, ==, 1);
    g_assert_cmpint(qemu_slirp_udp_send(listener, &peer, &payload, 1), ==,
                    -ENOTCONN);
    qemu_slirp_udp_listener_remove(listener);
    qemu_slirp_udp_registry_free(registry);
}

typedef struct FakeBootpBackend {
    unsigned sets;
    unsigned clears;
    struct in_addr server;
    char path[256];
    bool accept;
} FakeBootpBackend;

static bool fake_bootp_set(void *opaque,
                           const QemuSlirpBootpRootConfig *config)
{
    FakeBootpBackend *backend = opaque;

    if (!config) {
        backend->clears++;
        return true;
    }
    backend->sets++;
    backend->server = config->server;
    g_strlcpy(backend->path, config->path, sizeof(backend->path));
    return backend->accept;
}

static const QemuSlirpBootpBackendOps bootp_backend_ops = {
    .set_root = fake_bootp_set,
};

static void assert_bootp_claim_fails(QemuSlirpBootpRegistry *registry,
                                     const char *path)
{
    QemuSlirpBootpRootLease *lease = (void *)0x1;
    Error *err = NULL;

    g_assert_false(qemu_slirp_bootp_registry_claim(registry, path, &lease,
                                                    &err));
    g_assert_null(lease);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_bootp_claim_validation_and_owner(void)
{
    FakeBootpBackend backend = {.accept = true};
    QemuSlirpBootpRegistry *registry = qemu_slirp_bootp_registry_new(
        true, ip(0x0a000202), &bootp_backend_ops, &backend);
    QemuSlirpBootpRootLease *lease = NULL;
    char oversized[257];
    Error *err = NULL;

    memset(oversized, 'x', sizeof(oversized));
    oversized[0] = '/';
    oversized[256] = 0;
    assert_bootp_claim_fails(registry, NULL);
    assert_bootp_claim_fails(registry, "");
    assert_bootp_claim_fails(registry, "relative");
    assert_bootp_claim_fails(registry, oversized);
    g_assert_true(qemu_slirp_bootp_registry_claim(registry, "/", &lease,
                                                  &err));
    g_assert_cmphex(backend.server.s_addr, ==, ip(0x0a000202).s_addr);
    g_assert_cmpstr(backend.path, ==, "/");
    assert_bootp_claim_fails(registry, "/second");
    qemu_slirp_bootp_root_release(&lease);
    g_assert_null(lease);
    g_assert_cmpuint(backend.clears, ==, 1);
    qemu_slirp_bootp_registry_free(registry);
}

static void test_bootp_unwind_invalidate_and_unavailable(void)
{
    FakeBootpBackend backend = {0};
    QemuSlirpBootpRegistry *registry = qemu_slirp_bootp_registry_new(
        true, ip(0x0a000202), &bootp_backend_ops, &backend);
    QemuSlirpBootpRegistry *disabled = qemu_slirp_bootp_registry_new(
        false, ip(0x0a000202), &bootp_backend_ops, &backend);
    QemuSlirpBootpRegistry *unavailable = qemu_slirp_bootp_registry_new(
        true, ip(0x0a000202), NULL, NULL);
    QemuSlirpBootpRootLease *lease = NULL;
    Error *err = NULL;

    assert_bootp_claim_fails(registry, "/");
    backend.accept = true;
    g_assert_true(qemu_slirp_bootp_registry_claim(registry, "/", &lease,
                                                  &err));
    qemu_slirp_bootp_registry_invalidate(registry);
    g_assert_cmpuint(backend.clears, ==, 1);
    qemu_slirp_bootp_root_release(&lease);
    g_assert_null(lease);
    assert_bootp_claim_fails(disabled, "/");
    assert_bootp_claim_fails(unavailable, "/");
    qemu_slirp_bootp_registry_free(registry);
    qemu_slirp_bootp_registry_free(disabled);
    qemu_slirp_bootp_registry_free(unavailable);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-udp/validation-fixed-host",
                    test_udp_validation_and_fixed_host);
    g_test_add_func("/slirp-udp/ipv4-disabled-unavailable",
                    test_udp_ipv4_disabled_and_unavailable);
    g_test_add_func("/slirp-udp/delivery-reply-reentrant-remove",
                    test_udp_delivery_reply_and_reentrant_remove);
    g_test_add_func("/slirp-udp/backend-invalidation",
                    test_udp_backend_invalidation);
    g_test_add_func("/slirp-bootp/claim-validation-owner",
                    test_bootp_claim_validation_and_owner);
    g_test_add_func("/slirp-bootp/unwind-invalidate-unavailable",
                    test_bootp_unwind_invalidate_and_unavailable);
    return g_test_run();
}
