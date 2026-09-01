/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-stream-internal.h"
#include "qapi/error.h"

typedef struct FakeStreamConnection FakeStreamConnection;
typedef struct FakeStreamBackend {
    const QemuSlirpStreamBackendCallbacks *callbacks;
    void *callbacks_opaque;
    struct in_addr address;
    uint16_t port;
    void *listener;
    unsigned listens;
    unsigned listener_removes;
    unsigned closes;
    unsigned sends;
    size_t can_send;
    int listen_result;
    int send_result;
    bool defer_close;
    bool open_during_listen;
    bool close_during_send;
    FakeStreamConnection *connections[4];
    size_t nconnections;
} FakeStreamBackend;

struct FakeStreamConnection {
    FakeStreamBackend *backend;
    void *adapter_connection;
    bool close_requested;
    bool close_reported;
};

static FakeStreamConnection *fake_stream_open(FakeStreamBackend *backend,
                                                const struct sockaddr_in *peer);

static struct in_addr ip(uint32_t value)
{
    return (struct in_addr) { htonl(value) };
}

static void fake_stream_close(FakeStreamConnection *connection)
{
    FakeStreamBackend *backend = connection->backend;

    if (connection->close_requested) {
        return;
    }
    connection->close_requested = true;
    backend->closes++;
    if (!backend->defer_close) {
        connection->close_reported = true;
        backend->callbacks->closed(connection, backend->callbacks_opaque);
        connection->adapter_connection = NULL;
    }
}

static int fake_stream_listen(
    void *opaque, struct in_addr address, uint16_t port,
    const QemuSlirpStreamBackendCallbacks *callbacks, void *callbacks_opaque,
    void **listener)
{
    FakeStreamBackend *backend = opaque;

    if (backend->listener && backend->address.s_addr == address.s_addr &&
        backend->port == port) {
        return -EADDRINUSE;
    }
    backend->callbacks = callbacks;
    backend->callbacks_opaque = callbacks_opaque;
    backend->address = address;
    backend->port = port;
    backend->listener = backend;
    backend->listens++;
    *listener = backend;
    if (backend->listen_result < 0) {
        return backend->listen_result;
    }
    if (backend->open_during_listen) {
        struct sockaddr_in peer = {
            .sin_family = AF_INET,
            .sin_addr.s_addr = htonl(0x0a00020f),
            .sin_port = htons(49152),
        };

        fake_stream_open(backend, &peer);
    }
    return 0;
}

static void fake_stream_listener_remove(void *opaque, void *listener)
{
    FakeStreamBackend *backend = opaque;
    size_t i;

    g_assert_true(listener == backend->listener);
    backend->listener_removes++;
    backend->listener = NULL;
    for (i = 0; i < backend->nconnections; i++) {
        fake_stream_close(backend->connections[i]);
    }
}

static size_t fake_stream_can_send(void *opaque, void *backend_connection)
{
    FakeStreamBackend *backend = opaque;

    g_assert_nonnull(backend_connection);
    return backend->can_send;
}

static int fake_stream_send(void *opaque, void *backend_connection,
                            const uint8_t *data, size_t len)
{
    FakeStreamBackend *backend = opaque;

    g_assert_nonnull(backend_connection);
    backend->sends++;
    if (len) {
        g_assert_nonnull(data);
    }
    if (backend->close_during_send && len) {
        fake_stream_close(backend_connection);
    }
    return backend->send_result;
}

static void fake_stream_connection_close(void *opaque, void *backend_connection)
{
    fake_stream_close(backend_connection);
}

static const QemuSlirpStreamBackendOps stream_backend_ops = {
    .listen = fake_stream_listen,
    .listener_remove = fake_stream_listener_remove,
    .can_send = fake_stream_can_send,
    .send = fake_stream_send,
    .connection_close = fake_stream_connection_close,
};

static QemuSlirpStreamRegistry *new_registry(FakeStreamBackend *backend,
                                              bool ipv4_enabled)
{
    return qemu_slirp_stream_registry_new(ipv4_enabled, ip(0x0a000202),
                                           &stream_backend_ops, backend);
}

typedef struct StreamState {
    QemuSlirpStreamListener *listener;
    QemuSlirpStream *streams[4];
    struct sockaddr_in peers[4];
    uint8_t data[4][32];
    size_t lengths[4];
    unsigned connected;
    unsigned received;
    unsigned ready;
    unsigned closed;
    bool remove_listener_from_receive;
    bool remove_listener_from_connected;
    bool free_registry_from_receive;
    bool remove_listener_from_closed;
    bool free_registry_from_closed;
    bool close_from_closed;
    bool probe_setup_stream;
    size_t setup_can_send;
    int setup_send_result;
    QemuSlirpStreamListener **listener_slot;
    QemuSlirpStreamRegistry *registry;
} StreamState;

static void stream_connected(QemuSlirpStream *stream,
                             const struct sockaddr_in *peer, void *opaque)
{
    StreamState *state = opaque;
    static const uint8_t setup_data = 0x37;

    g_assert_cmpuint(state->connected, <, G_N_ELEMENTS(state->streams));
    state->peers[state->connected] = *peer;
    state->streams[state->connected++] = stream;
    if (!state->listener && state->listener_slot) {
        state->listener = *state->listener_slot;
    }
    if (state->probe_setup_stream) {
        state->setup_can_send = qemu_slirp_stream_can_send(stream);
        state->setup_send_result = qemu_slirp_stream_send(
            stream, &setup_data, sizeof(setup_data));
    }
    if (state->remove_listener_from_connected) {
        qemu_slirp_stream_listener_remove(state->listener);
        state->listener = NULL;
    }
}

static void stream_receive(QemuSlirpStream *stream, const uint8_t *data,
                           size_t len, void *opaque)
{
    StreamState *state = opaque;
    unsigned index = state->received++;

    g_assert_cmpuint(index, <, G_N_ELEMENTS(state->data));
    g_assert_cmpuint(len, <=, sizeof(state->data[index]));
    memcpy(state->data[index], data, len);
    state->lengths[index] = len;
    if (state->remove_listener_from_receive) {
        qemu_slirp_stream_listener_remove(state->listener);
        state->listener = NULL;
    }
    if (state->free_registry_from_receive) {
        qemu_slirp_stream_registry_free(state->registry);
        state->registry = NULL;
    }
}

static void stream_can_send(QemuSlirpStream *stream, void *opaque)
{
    StreamState *state = opaque;

    state->ready++;
}

static void stream_closed(QemuSlirpStream *stream, void *opaque)
{
    StreamState *state = opaque;
    unsigned i;

    state->closed++;
    for (i = 0; i < state->connected; i++) {
        if (state->streams[i] == stream) {
            state->streams[i] = NULL;
        }
    }
    if (state->close_from_closed) {
        qemu_slirp_stream_close(stream);
    }
    if (state->remove_listener_from_closed && state->listener) {
        qemu_slirp_stream_listener_remove(state->listener);
        state->listener = NULL;
    }
    if (state->free_registry_from_closed && state->registry) {
        qemu_slirp_stream_registry_free(state->registry);
        state->registry = NULL;
    }
}

static const QemuSlirpStreamOps stream_ops = {
    .connected = stream_connected,
    .receive = stream_receive,
    .can_send = stream_can_send,
    .closed = stream_closed,
};

static FakeStreamConnection *fake_stream_open(FakeStreamBackend *backend,
                                                const struct sockaddr_in *peer)
{
    FakeStreamConnection *connection = g_new0(FakeStreamConnection, 1);

    g_assert_cmpuint(backend->nconnections, <,
                     G_N_ELEMENTS(backend->connections));
    connection->backend = backend;
    backend->connections[backend->nconnections++] = connection;
    backend->callbacks->connected(connection, peer,
                                  backend->callbacks_opaque);
    connection->adapter_connection = connection;
    return connection;
}

static void assert_listen_fails(QemuSlirpStreamRegistry *registry,
                                uint16_t port,
                                const QemuSlirpStreamOps *ops)
{
    QemuSlirpStreamListener *listener = (void *)0x1;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, port, ops, NULL, &listener, &err), ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_stream_validation_and_collision(void)
{
    FakeStreamBackend backend = {0};
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    Error *err = NULL;

    assert_listen_fails(registry, 0, &stream_ops);
    assert_listen_fails(registry, 17008, NULL);
    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, NULL, &listener, &err),
                    ==, 0);
    g_assert_cmphex(backend.address.s_addr, ==, ip(0x0a000202).s_addr);
    assert_listen_fails(registry, 17008, &stream_ops);
    qemu_slirp_stream_listener_remove(listener);
    qemu_slirp_stream_registry_free(registry);
}

static void test_stream_ipv4_disabled_and_unavailable_backend(void)
{
    FakeStreamBackend backend = {0};
    QemuSlirpStreamRegistry *registry = new_registry(&backend, false);
    QemuSlirpStreamListener *listener = (void *)0x1;
    Error *err = NULL;

    assert_listen_fails(registry, 17008, &stream_ops);
    g_assert_cmpuint(backend.listens, ==, 0);
    qemu_slirp_stream_registry_free(registry);

    registry = qemu_slirp_stream_registry_new(true, ip(0x0a000202), NULL,
                                               NULL);
    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, NULL, &listener, &err),
                    ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    error_free(err);
    qemu_slirp_stream_registry_free(registry);
}

static void test_stream_connections_and_order(void)
{
    static const uint8_t one = 0x11, two = 0x22;
    const struct sockaddr_in peer1 = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    const struct sockaddr_in peer2 = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a000210),
        .sin_port = htons(49153),
    };
    FakeStreamBackend backend = { .can_send = 8 };
    StreamState state = {0};
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    FakeStreamConnection *connection1, *connection2;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, 0);
    state.listener = listener;
    connection1 = fake_stream_open(&backend, &peer1);
    connection2 = fake_stream_open(&backend, &peer2);
    g_assert_cmpuint(state.connected, ==, 2);
    g_assert_true(state.streams[0] != state.streams[1]);
    g_assert_cmpmem(&state.peers[0], sizeof(peer1), &peer1, sizeof(peer1));
    g_assert_cmpmem(&state.peers[1], sizeof(peer2), &peer2, sizeof(peer2));
    backend.callbacks->receive(connection1, &one, 1, backend.callbacks_opaque);
    backend.callbacks->receive(connection1, &two, 1, backend.callbacks_opaque);
    g_assert_cmpuint(state.received, ==, 2);
    g_assert_cmpuint(state.data[0][0], ==, one);
    g_assert_cmpuint(state.data[1][0], ==, two);
    g_assert_cmpuint(qemu_slirp_stream_can_send(state.streams[0]), ==, 8);
    qemu_slirp_stream_listener_remove(listener);
    g_assert_cmpuint(state.closed, ==, 2);
    qemu_slirp_stream_registry_free(registry);
    g_free(connection1);
    g_free(connection2);
    error_free(err);
}

static void test_stream_send_backpressure_and_ready(void)
{
    static const uint8_t data = 0x5a;
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeStreamBackend backend = { .can_send = 4, .send_result = 0 };
    StreamState state = {0};
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    FakeStreamConnection *connection;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, 0);
    connection = fake_stream_open(&backend, &peer);
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==, 0);
    backend.send_result = -EAGAIN;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==,
                    -EAGAIN);
    backend.send_result = 0;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==, 0);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.ready, ==, 0);
    backend.send_result = -EAGAIN;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==,
                    -EAGAIN);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.ready, ==, 1);
    backend.send_result = 0;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==, 0);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.ready, ==, 1);
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], NULL, 1), ==,
                    -EINVAL);
    backend.send_result = 1;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==,
                    -EIO);
    backend.send_result = 0;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 0), ==,
                    0);
    backend.send_result = -EAGAIN;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==,
                    -EAGAIN);
    backend.send_result = 0;
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], NULL, 0), ==, 0);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.ready, ==, 2);
    qemu_slirp_stream_listener_remove(listener);
    qemu_slirp_stream_registry_free(registry);
    g_free(connection);
    error_free(err);
}

static void test_stream_backend_failure(void)
{
    FakeStreamBackend backend = { .listen_result = -EIO };
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = (void *)0x1;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, NULL, &listener, &err),
                    ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    g_assert_cmpuint(backend.listener_removes, ==, 1);
    error_free(err);
    qemu_slirp_stream_registry_free(registry);
}

static void test_stream_listener_remove_from_connected(void)
{
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeStreamBackend backend = {0};
    StreamState state = { .remove_listener_from_connected = true };
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    FakeStreamConnection *connection;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, 0);
    state.listener = listener;
    connection = fake_stream_open(&backend, &peer);
    g_assert_cmpuint(state.connected, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_null(state.streams[0]);
    g_assert_null(state.listener);
    g_assert_cmpuint(backend.listener_removes, ==, 1);
    qemu_slirp_stream_registry_free(registry);
    g_free(connection);
    error_free(err);
}

static void test_stream_close_is_idempotent(void)
{
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeStreamBackend backend = { .defer_close = true };
    StreamState state = {0};
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    FakeStreamConnection *connection;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, 0);
    connection = fake_stream_open(&backend, &peer);
    qemu_slirp_stream_close(state.streams[0]);
    qemu_slirp_stream_close(state.streams[0]);
    g_assert_cmpuint(backend.closes, ==, 1);
    g_assert_cmpuint(state.closed, ==, 0);
    backend.defer_close = false;
    backend.callbacks->closed(connection, backend.callbacks_opaque);
    backend.callbacks->closed(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.closed, ==, 1);
    qemu_slirp_stream_listener_remove(listener);
    g_assert_cmpuint(state.closed, ==, 1);
    qemu_slirp_stream_registry_free(registry);
    g_free(connection);
    error_free(err);
}

static void test_stream_registry_free_from_receive(void)
{
    static const uint8_t data = 0x5a;
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeStreamBackend backend = {0};
    StreamState state = { .free_registry_from_receive = true };
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    FakeStreamConnection *connection;
    Error *err = NULL;

    state.registry = registry;
    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, 0);
    connection = fake_stream_open(&backend, &peer);
    backend.callbacks->receive(connection, &data, 1,
                                backend.callbacks_opaque);
    g_assert_cmpuint(state.received, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_null(state.registry);
    g_assert_null(state.streams[0]);
    qemu_slirp_stream_listener_remove(listener);
    g_free(connection);
    error_free(err);
}

static void test_stream_setup_and_closed_reentrancy(void)
{
    FakeStreamBackend backend = { .can_send = 4, .open_during_listen = true };
    StreamState state = {
        .remove_listener_from_connected = true,
        .remove_listener_from_closed = true,
        .free_registry_from_closed = true,
        .probe_setup_stream = true,
    };
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    Error *err = NULL;

    state.registry = registry;
    state.listener_slot = &listener;
    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, -1);
    g_assert_null(listener);
    g_assert_cmpuint(state.connected, ==, 1);
    g_assert_cmpuint(state.setup_can_send, ==, 4);
    g_assert_cmpint(state.setup_send_result, ==, 0);
    g_assert_cmpuint(backend.sends, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_null(state.listener);
    g_assert_null(state.registry);
    g_assert_cmpuint(backend.listener_removes, ==, 1);
    g_free(backend.connections[0]);
    error_free(err);
}

static void test_stream_reentrant_close_remove_and_free(void)
{
    static const uint8_t data = 0x5a;
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };
    FakeStreamBackend backend = { .can_send = 4, .close_during_send = true,
                                  .send_result = -EAGAIN };
    StreamState state = {0};
    QemuSlirpStreamRegistry *registry = new_registry(&backend, true);
    QemuSlirpStreamListener *listener = NULL;
    FakeStreamConnection *connection;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_registry_listen(
                        registry, 17008, &stream_ops, &state, &listener, &err),
                    ==, 0);
    state.listener = listener;
    connection = fake_stream_open(&backend, &peer);
    g_assert_cmpint(qemu_slirp_stream_send(state.streams[0], &data, 1), ==,
                    -ENOTCONN);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_cmpuint(backend.closes, ==, 1);
    qemu_slirp_stream_listener_remove(listener);
    qemu_slirp_stream_registry_free(registry);
    g_free(connection);
    error_free(err);
}

static void test_stream_unavailable(void)
{
    QemuSlirpStreamListener *listener = (void *)0x1;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_stream_listen_unavailable(&listener, &err),
                    ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "TCP"));
    error_free(err);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-stream/validation-and-collision",
                    test_stream_validation_and_collision);
    g_test_add_func("/slirp-stream/ipv4-disabled-and-unavailable-backend",
                    test_stream_ipv4_disabled_and_unavailable_backend);
    g_test_add_func("/slirp-stream/connections-and-order",
                    test_stream_connections_and_order);
    g_test_add_func("/slirp-stream/send-backpressure-and-ready",
                    test_stream_send_backpressure_and_ready);
    g_test_add_func("/slirp-stream/backend-failure",
                    test_stream_backend_failure);
    g_test_add_func("/slirp-stream/remove-from-connected",
                    test_stream_listener_remove_from_connected);
    g_test_add_func("/slirp-stream/close-idempotent",
                    test_stream_close_is_idempotent);
    g_test_add_func("/slirp-stream/registry-free-from-receive",
                    test_stream_registry_free_from_receive);
    g_test_add_func("/slirp-stream/setup-and-closed-reentrancy",
                    test_stream_setup_and_closed_reentrancy);
    g_test_add_func("/slirp-stream/reentrant-close-remove-free",
                    test_stream_reentrant_close_remove_and_free);
    g_test_add_func("/slirp-stream/unavailable", test_stream_unavailable);
    return g_test_run();
}
