/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-il-internal.h"
#include "qapi/error.h"

typedef struct FakeConnection FakeConnection;
typedef struct FakeBackend {
    const QemuSlirpILBackendCallbacks *callbacks;
    void *callbacks_opaque;
    struct in_addr addr;
    uint16_t port;
    void *listener;
    int listens, listener_removes, closes, sends;
    int listen_result, send_result;
    bool defer_close;
    bool open_during_listen;
    bool close_during_send;
    FakeConnection *connections[4];
    size_t nconnections;
} FakeBackend;

struct FakeConnection {
    FakeBackend *backend;
    void *adapter_connection;
    bool close_requested;
    bool close_reported;
};

static FakeConnection *fake_open(FakeBackend *backend);

static int fake_listen(void *opaque, struct in_addr addr, uint16_t port,
                       const QemuSlirpILBackendCallbacks *callbacks,
                       void *callbacks_opaque, void **listener)
{
    FakeBackend *backend = opaque;

    if (backend->listener && backend->addr.s_addr == addr.s_addr &&
        backend->port == port) {
        return -EADDRINUSE;
    }
    backend->callbacks = callbacks;
    backend->callbacks_opaque = callbacks_opaque;
    backend->addr = addr;
    backend->port = port;
    backend->listener = backend;
    backend->listens++;
    *listener = backend;
    if (backend->open_during_listen) {
        fake_open(backend);
    }
    return backend->listen_result;
}

static void fake_close_connection(FakeConnection *connection)
{
    FakeBackend *backend = connection->backend;

    if (connection->close_requested) {
        return;
    }
    connection->close_requested = true;
    backend->closes++;
    if (!backend->defer_close) {
        connection->close_reported = true;
        backend->callbacks->close(connection, backend->callbacks_opaque);
        connection->adapter_connection = NULL;
    }
}

static void fake_listener_remove(void *opaque, void *listener)
{
    FakeBackend *backend = opaque;
    size_t i;

    g_assert_true(listener == backend->listener);
    backend->listener_removes++;
    backend->listener = NULL;
    for (i = 0; i < backend->nconnections; i++) {
        fake_close_connection(backend->connections[i]);
    }
}

static int fake_send_record(void *opaque, void *backend_connection,
                            const uint8_t *data, size_t len)
{
    FakeBackend *backend = opaque;

    g_assert_nonnull(backend_connection);
    g_assert_nonnull(data);
    g_assert_cmpuint(len, >, 0);
    backend->sends++;
    if (backend->close_during_send) {
        fake_close_connection(backend_connection);
    }
    return backend->send_result;
}

static void fake_connection_close(void *opaque, void *backend_connection)
{
    fake_close_connection(backend_connection);
}

static const QemuSlirpILBackendOps backend_ops = {
    .listen = fake_listen,
    .listener_remove = fake_listener_remove,
    .send_record = fake_send_record,
    .connection_close = fake_connection_close,
};

static struct in_addr ip(uint32_t value)
{
    return (struct in_addr) { htonl(value) };
}

static QemuSlirpILRegistry *new_registry(FakeBackend *backend,
                                         bool ipv4_enabled)
{
    return qemu_slirp_il_registry_new(ipv4_enabled, ip(0x0a000200),
                                      ip(0xffffff00), ip(0x0a000202),
                                      ip(0x0a000203), &backend_ops, backend);
}

typedef struct CallbackState {
    QemuSlirpILListener *listener;
    QemuSlirpILConnection *connections[4];
    unsigned opened, records, ready, closed;
    uint8_t last_record;
    bool remove_listener_from_record;
    bool remove_listener_from_open;
    bool check_terminal_close;
    int terminal_send_result;
    QemuSlirpILListener **listener_slot;
} CallbackState;

static void *opened(QemuSlirpILConnection *connection, void *opaque)
{
    CallbackState *state = opaque;

    state->connections[state->opened++] = connection;
    if (state->remove_listener_from_open) {
        g_assert_nonnull(state->listener_slot);
        g_assert_nonnull(*state->listener_slot);
        state->listener = *state->listener_slot;
        qemu_slirp_il_listener_remove(state->listener);
        *state->listener_slot = NULL;
        state->listener = NULL;
    }
    return state;
}

static void record(QemuSlirpILConnection *connection, const uint8_t *data,
                   size_t len, void *opaque)
{
    CallbackState *state = opaque;

    g_assert_cmpuint(len, ==, 1);
    state->records++;
    state->last_record = data[0];
    if (state->remove_listener_from_record) {
        qemu_slirp_il_listener_remove(state->listener);
        state->listener = NULL;
    }
}

static void can_send(QemuSlirpILConnection *connection, void *opaque)
{
    CallbackState *state = opaque;

    state->ready++;
}

static void closed(QemuSlirpILConnection *connection, void *opaque)
{
    CallbackState *state = opaque;
    size_t i;

    state->closed++;
    for (i = 0; i < state->opened; i++) {
        if (state->connections[i] == connection) {
            state->connections[i] = NULL;
        }
    }
    if (state->check_terminal_close) {
        uint8_t record_byte = 1;

        state->terminal_send_result = qemu_slirp_il_send_record(
            connection, &record_byte, sizeof(record_byte));
        qemu_slirp_il_connection_close(connection);
    }
}

static const QemuSlirpILListenerOps listener_ops = {
    .open = opened,
    .record = record,
    .can_send = can_send,
    .close = closed,
};

static FakeConnection *fake_open(FakeBackend *backend)
{
    FakeConnection *connection = g_new0(FakeConnection, 1);

    g_assert_cmpuint(backend->nconnections, <,
                     G_N_ELEMENTS(backend->connections));
    connection->backend = backend;
    backend->connections[backend->nconnections++] = connection;
    connection->adapter_connection = backend->callbacks->open(
        connection, backend->callbacks_opaque);
    return connection;
}

static void assert_listen_fails(QemuSlirpILRegistry *registry,
                                struct in_addr address, uint16_t port)
{
    QemuSlirpILListener *listener = (void *)0x1;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, address, port,
                                                   &listener_ops, NULL,
                                                   &listener, &err), ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_validation(void)
{
    FakeBackend backend = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);

    assert_listen_fails(registry, ip(0x0a000200), 17008);
    assert_listen_fails(registry, ip(0x0a0002ff), 17008);
    assert_listen_fails(registry, ip(0x0a000202), 17008);
    assert_listen_fails(registry, ip(0x0a000203), 17008);
    assert_listen_fails(registry, ip(0x0b000204), 17008);
    assert_listen_fails(registry, ip(0x0a000204), 0);
    qemu_slirp_il_registry_free(registry);
}

static void test_ipv4_disabled(void)
{
    FakeBackend backend = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, false);

    assert_listen_fails(registry, ip(0x0a000204), 17008);
    g_assert_cmpint(backend.listens, ==, 0);
    qemu_slirp_il_registry_free(registry);
}

static void test_unavailable_feature(void)
{
    QemuSlirpILListener *listener = (void *)0x1;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_listen_unavailable(&listener, &err), ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "IL is unavailable"));
    error_free(err);
}

static void test_duplicate_tuple(void)
{
    FakeBackend backend = {0};
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    assert_listen_fails(registry, ip(0x0a000204), 17008);
    qemu_slirp_il_listener_remove(listener);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
}

static void test_independent_connections(void)
{
    FakeBackend backend = {0};
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *one, *two;
    Error *err = NULL;
    uint8_t first = 1, second = 2;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    one = fake_open(&backend);
    two = fake_open(&backend);
    g_assert_cmpuint(state.opened, ==, 2);
    g_assert_true(state.connections[0] != state.connections[1]);
    backend.callbacks->record(one, &first, 1, backend.callbacks_opaque);
    backend.callbacks->record(two, &second, 1, backend.callbacks_opaque);
    g_assert_cmpuint(state.records, ==, 2);
    g_assert_cmpuint(state.last_record, ==, 2);
    qemu_slirp_il_listener_remove(listener);
    g_assert_cmpuint(state.closed, ==, 2);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(one);
    g_free(two);
}

static void test_atomic_send_and_errors(void)
{
    FakeBackend backend = {0};
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *connection;
    Error *err = NULL;
    uint8_t record_byte = 1;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    connection = fake_open(&backend);
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              &record_byte, 1), ==, 0);
    backend.send_result = -EAGAIN;
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              &record_byte, 1), ==, -EAGAIN);
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              NULL, 1), ==, -EINVAL);
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              &record_byte, 0), ==, -EINVAL);
    qemu_slirp_il_listener_remove(listener);
    g_assert_null(state.connections[0]);
    g_assert_null(connection->adapter_connection);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(connection);
}

static void test_send_ready_edges(void)
{
    FakeBackend backend = {0};
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *connection;
    Error *err = NULL;
    uint8_t record_byte = 1;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    connection = fake_open(&backend);
    backend.send_result = -EAGAIN;
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              &record_byte, 1), ==, -EAGAIN);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.ready, ==, 1);
    backend.send_result = 0;
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              &record_byte, 1), ==, 0);
    backend.send_result = -EAGAIN;
    g_assert_cmpint(qemu_slirp_il_send_record(connection->adapter_connection,
                                              &record_byte, 1), ==, -EAGAIN);
    backend.callbacks->can_send(connection, backend.callbacks_opaque);
    g_assert_cmpuint(state.ready, ==, 2);
    qemu_slirp_il_listener_remove(listener);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(connection);
}

static void test_send_close_reentrancy(void)
{
    FakeBackend backend = {
        .send_result = -EAGAIN,
        .close_during_send = true,
    };
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *connection;
    Error *err = NULL;
    uint8_t record_byte = 1;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    connection = fake_open(&backend);
    g_assert_cmpint(qemu_slirp_il_send_record(state.connections[0],
                                              &record_byte, 1), ==, -ENOTCONN);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_null(state.connections[0]);
    g_assert_null(connection->adapter_connection);
    qemu_slirp_il_listener_remove(listener);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(connection);
}

static void test_remove_listener_from_callback(void)
{
    FakeBackend backend = {0};
    CallbackState state = {.remove_listener_from_record = true};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *connection;
    Error *err = NULL;
    uint8_t record_byte = 1;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    state.listener = listener;
    connection = fake_open(&backend);
    backend.callbacks->record(connection, &record_byte, 1,
                              backend.callbacks_opaque);
    g_assert_null(state.listener);
    g_assert_cmpint(backend.listener_removes, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(connection);
}

static void test_repeated_close_and_invalidate(void)
{
    FakeBackend backend = {0};
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *connection;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    connection = fake_open(&backend);
    backend.defer_close = true;
    qemu_slirp_il_connection_close(connection->adapter_connection);
    qemu_slirp_il_connection_close(connection->adapter_connection);
    g_assert_cmpint(backend.closes, ==, 1);
    qemu_slirp_il_registry_invalidate(registry);
    g_assert_cmpint(backend.listener_removes, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    qemu_slirp_il_listener_remove(listener);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(connection);
}

static void test_terminal_close_callback(void)
{
    FakeBackend backend = {0};
    CallbackState state = {.check_terminal_close = true};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    FakeConnection *connection;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    connection = fake_open(&backend);
    qemu_slirp_il_connection_close(state.connections[0]);
    g_assert_cmpint(backend.closes, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_null(state.connections[0]);
    g_assert_cmpint(state.terminal_send_result, ==, -ENOTCONN);
    qemu_slirp_il_listener_remove(listener);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(connection);
}

static void test_listen_open_reentrancy(void)
{
    FakeBackend backend = {.open_during_listen = true};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    CallbackState state = {
        .remove_listener_from_open = true,
        .listener_slot = &listener,
    };
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    g_assert_cmpuint(state.opened, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_cmpint(backend.listener_removes, ==, 1);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(backend.connections[0]);
}

static void test_listen_failure_after_open(void)
{
    FakeBackend backend = {
        .listen_result = -EIO,
        .open_during_listen = true,
    };
    CallbackState state = {0};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, -1);
    g_assert_null(listener);
    g_assert_nonnull(err);
    g_assert_cmpuint(state.opened, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_cmpint(backend.listener_removes, ==, 1);
    error_free(err);
    qemu_slirp_il_registry_free(registry);
    g_free(backend.connections[0]);
}

typedef struct BridgeState {
    void *opened_raw;
    void *record_raw;
    void *ready_raw;
    void *closed_raw;
    unsigned opened, records, ready, closed;
} BridgeState;

static void *bridge_open(void *backend_connection, void *opaque)
{
    BridgeState *state = opaque;

    state->opened_raw = backend_connection;
    state->opened++;
    return state;
}

static void bridge_record(void *backend_connection, const uint8_t *data,
                          size_t len, void *opaque)
{
    BridgeState *state = opaque;

    g_assert_cmpuint(len, ==, 1);
    g_assert_cmpuint(data[0], ==, 0x5a);
    state->record_raw = backend_connection;
    state->records++;
}

static void bridge_can_send(void *backend_connection, void *opaque)
{
    BridgeState *state = opaque;

    state->ready_raw = backend_connection;
    state->ready++;
}

static void bridge_close(void *backend_connection, void *opaque)
{
    BridgeState *state = opaque;

    state->closed_raw = backend_connection;
    state->closed++;
}

static void test_backend_bridge_raw_connection_identity(void)
{
    static const QemuSlirpILBackendCallbacks callbacks = {
        .open = bridge_open,
        .record = bridge_record,
        .can_send = bridge_can_send,
        .close = bridge_close,
    };
    BridgeState state = {0};
    QemuSlirpILBackendBridge *bridge;
    uint8_t record_byte = 0x5a;
    int raw_connection;
    void *connection_opaque;

    bridge = qemu_slirp_il_backend_bridge_new(&callbacks, &state);
    connection_opaque = qemu_slirp_il_backend_bridge_connected(
        bridge, &raw_connection);
    g_assert_nonnull(connection_opaque);
    g_assert_true(state.opened_raw == &raw_connection);
    qemu_slirp_il_backend_bridge_record(&raw_connection, &record_byte,
                                        sizeof(record_byte), connection_opaque);
    qemu_slirp_il_backend_bridge_can_send(&raw_connection, connection_opaque);
    qemu_slirp_il_backend_bridge_closed(&raw_connection, connection_opaque);
    g_assert_cmpuint(state.opened, ==, 1);
    g_assert_cmpuint(state.records, ==, 1);
    g_assert_cmpuint(state.ready, ==, 1);
    g_assert_cmpuint(state.closed, ==, 1);
    g_assert_true(state.record_raw == &raw_connection);
    g_assert_true(state.ready_raw == &raw_connection);
    g_assert_true(state.closed_raw == &raw_connection);
    qemu_slirp_il_backend_bridge_free(bridge);
}

typedef struct CleanupState {
    FakeBackend *backend;
    bool called;
} CleanupState;

static void cleanup_after_registry(void *opaque)
{
    CleanupState *state = opaque;

    g_assert_cmpint(state->backend->listener_removes, ==, 1);
    state->called = true;
}

static void test_registry_progress_and_cleanup_order(void)
{
    FakeBackend backend = {0};
    CallbackState state = {0};
    CleanupState cleanup = {.backend = &backend};
    QemuSlirpILRegistry *registry = new_registry(&backend, true);
    QemuSlirpILListener *listener = NULL;
    Error *err = NULL;

    g_assert_cmpint(qemu_slirp_il_registry_listen(registry, ip(0x0a000204),
                                                   17008, &listener_ops,
                                                   &state, &listener, &err),
                    ==, 0);
    qemu_slirp_il_registry_progress(registry);
    g_assert_cmpint(backend.listener_removes, ==, 0);
    qemu_slirp_il_registry_cleanup(registry, cleanup_after_registry, &cleanup);
    g_assert_true(cleanup.called);
    error_free(err);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-il/validation", test_validation);
    g_test_add_func("/slirp-il/ipv4-disabled", test_ipv4_disabled);
    g_test_add_func("/slirp-il/unavailable-feature", test_unavailable_feature);
    g_test_add_func("/slirp-il/duplicate-tuple", test_duplicate_tuple);
    g_test_add_func("/slirp-il/independent-connections",
                    test_independent_connections);
    g_test_add_func("/slirp-il/atomic-send-errors",
                    test_atomic_send_and_errors);
    g_test_add_func("/slirp-il/send-ready-edges", test_send_ready_edges);
    g_test_add_func("/slirp-il/send-close-reentrancy",
                    test_send_close_reentrancy);
    g_test_add_func("/slirp-il/remove-from-callback",
                    test_remove_listener_from_callback);
    g_test_add_func("/slirp-il/repeated-close-invalidate",
                    test_repeated_close_and_invalidate);
    g_test_add_func("/slirp-il/terminal-close-callback",
                    test_terminal_close_callback);
    g_test_add_func("/slirp-il/listen-open-reentrancy",
                    test_listen_open_reentrancy);
    g_test_add_func("/slirp-il/listen-failure-after-open",
                    test_listen_failure_after_open);
    g_test_add_func("/slirp-il/backend-bridge-raw-connection-identity",
                    test_backend_bridge_raw_connection_identity);
    g_test_add_func("/slirp-il/registry-progress-cleanup-order",
                    test_registry_progress_and_cleanup_order);
    return g_test_run();
}
