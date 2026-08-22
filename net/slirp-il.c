/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "net/slirp-il-internal.h"
#include "qapi/error.h"

struct QemuSlirpILRegistry {
    struct in_addr network, mask, vhost, dns;
    const QemuSlirpILBackendOps *backend;
    void *backend_opaque;
    QTAILQ_HEAD(, QemuSlirpILListener) listeners;
    QTAILQ_HEAD(, QemuSlirpILListener) deferred;
    bool valid;
    bool ipv4_enabled;
    unsigned progress_generation;
};

struct QemuSlirpILListener {
    QTAILQ_ENTRY(QemuSlirpILListener) entry;
    QemuSlirpILRegistry *registry;
    struct in_addr addr;
    uint16_t port;
    QemuSlirpILListenerOps ops;
    void *opaque;
    void *backend_listener;
    QTAILQ_HEAD(, QemuSlirpILConnection) connections;
    unsigned refs;
    unsigned callback_depth;
    bool registry_ref;
    bool caller_ref;
    bool valid;
    bool registered;
    bool deferred;
    bool remove_pending;
};

struct QemuSlirpILConnection {
    QTAILQ_ENTRY(QemuSlirpILConnection) entry;
    QemuSlirpILListener *listener;
    void *backend_connection;
    void *opaque;
    unsigned refs;
    unsigned callback_depth;
    bool linked;
    bool valid;
    bool close_requested;
    bool close_notified;
    bool send_ready;
};

struct QemuSlirpILBackendBridge {
    const QemuSlirpILBackendCallbacks *callbacks;
    void *opaque;
};

typedef struct QemuSlirpILBackendConnection {
    QemuSlirpILBackendBridge *bridge;
} QemuSlirpILBackendConnection;

static void listener_finish_remove(QemuSlirpILListener *listener);

QemuSlirpILBackendBridge *qemu_slirp_il_backend_bridge_new(
    const QemuSlirpILBackendCallbacks *callbacks, void *callbacks_opaque)
{
    QemuSlirpILBackendBridge *bridge;

    bridge = g_new(QemuSlirpILBackendBridge, 1);
    bridge->callbacks = callbacks;
    bridge->opaque = callbacks_opaque;
    return bridge;
}

void qemu_slirp_il_backend_bridge_free(QemuSlirpILBackendBridge *bridge)
{
    g_free(bridge);
}

void *qemu_slirp_il_backend_bridge_connected(
    QemuSlirpILBackendBridge *bridge, void *backend_connection)
{
    QemuSlirpILBackendConnection *connection;

    connection = g_new(QemuSlirpILBackendConnection, 1);
    connection->bridge = bridge;
    bridge->callbacks->open(backend_connection, bridge->opaque);
    return connection;
}

void qemu_slirp_il_backend_bridge_record(void *backend_connection,
                                         const uint8_t *data, size_t len,
                                         void *connection_opaque)
{
    QemuSlirpILBackendConnection *connection = connection_opaque;
    const QemuSlirpILBackendCallbacks *callbacks =
        connection->bridge->callbacks;
    void *opaque = connection->bridge->opaque;

    callbacks->record(backend_connection, data, len, opaque);
}

void qemu_slirp_il_backend_bridge_can_send(void *backend_connection,
                                           void *connection_opaque)
{
    QemuSlirpILBackendConnection *connection = connection_opaque;
    const QemuSlirpILBackendCallbacks *callbacks =
        connection->bridge->callbacks;
    void *opaque = connection->bridge->opaque;

    if (callbacks->can_send) {
        callbacks->can_send(backend_connection, opaque);
    }
}

void qemu_slirp_il_backend_bridge_closed(void *backend_connection,
                                         void *connection_opaque)
{
    QemuSlirpILBackendConnection *connection = connection_opaque;
    const QemuSlirpILBackendCallbacks *callbacks =
        connection->bridge->callbacks;
    void *opaque = connection->bridge->opaque;

    callbacks->close(backend_connection, opaque);
    g_free(connection);
}

static void listener_ref(QemuSlirpILListener *listener)
{
    assert(listener->refs);
    listener->refs++;
}

static void listener_unref(QemuSlirpILListener *listener)
{
    assert(listener->refs);
    if (!--listener->refs) {
        g_free(listener);
    }
}

static void connection_ref(QemuSlirpILConnection *connection)
{
    assert(connection->refs);
    connection->refs++;
}

static void connection_unref(QemuSlirpILConnection *connection)
{
    assert(connection->refs);
    if (!--connection->refs) {
        g_free(connection);
    }
}

static QemuSlirpILConnection *find_connection(QemuSlirpILListener *listener,
                                               void *backend_connection)
{
    QemuSlirpILConnection *connection;

    QTAILQ_FOREACH(connection, &listener->connections, entry) {
        if (connection->backend_connection == backend_connection) {
            return connection;
        }
    }
    return NULL;
}

static void connection_unlink(QemuSlirpILConnection *connection)
{
    QemuSlirpILListener *listener = connection->listener;

    if (!connection->linked) {
        return;
    }
    QTAILQ_REMOVE(&listener->connections, connection, entry);
    connection->linked = false;
    connection->valid = false;
    connection->listener = NULL;
    listener_unref(listener);
    connection_unref(connection);
}

static void connection_finished(QemuSlirpILConnection *connection)
{
    QemuSlirpILListener *listener = connection->listener;

    connection_ref(connection);
    if (!connection->close_notified) {
        connection->close_notified = true;
        connection->valid = false;
        connection_unlink(connection);
        if (listener->ops.close) {
            listener_ref(listener);
            listener->callback_depth++;
            connection->callback_depth++;
            listener->ops.close(connection, connection->opaque);
            connection->callback_depth--;
            listener->callback_depth--;
            listener_finish_remove(listener);
            listener_unref(listener);
        }
    }
    connection_unref(connection);
}

static void listener_unlink(QemuSlirpILListener *listener)
{
    QemuSlirpILRegistry *registry = listener->registry;

    if (!listener->valid) {
        return;
    }
    listener->valid = false;
    if (listener->registry_ref) {
        if (listener->deferred) {
            QTAILQ_REMOVE(&registry->deferred, listener, entry);
        } else {
            QTAILQ_REMOVE(&registry->listeners, listener, entry);
        }
        QTAILQ_INSERT_TAIL(&registry->deferred, listener, entry);
        listener->deferred = true;
    }
}

static void listener_finish_remove(QemuSlirpILListener *listener)
{
    QemuSlirpILRegistry *registry;

    if (!listener->remove_pending || listener->callback_depth) {
        return;
    }
    listener->remove_pending = false;
    registry = listener->registry;
    if (listener->deferred) {
        QTAILQ_REMOVE(&registry->deferred, listener, entry);
        listener->deferred = false;
    }
    if (listener->registered) {
        listener->registered = false;
        if (listener->backend_listener) {
            registry->backend->listener_remove(registry->backend_opaque,
                                               listener->backend_listener);
        }
    }
    while (!QTAILQ_EMPTY(&listener->connections)) {
        connection_finished(QTAILQ_FIRST(&listener->connections));
    }
    if (listener->registry_ref) {
        listener->registry_ref = false;
        listener->registry = NULL;
        listener_unref(listener);
    }
}

static void listener_remove_internal(QemuSlirpILListener *listener)
{
    if (!listener || !listener->caller_ref) {
        return;
    }
    listener_ref(listener);
    listener->caller_ref = false;
    listener_unlink(listener);
    listener->remove_pending = true;
    listener_finish_remove(listener);
    listener_unref(listener);
    listener_unref(listener);
}

static void *backend_open(void *backend_connection, void *opaque)
{
    QemuSlirpILListener *listener = opaque;
    QemuSlirpILConnection *connection;
    void *result;

    if (!listener->valid) {
        return NULL;
    }
    listener_ref(listener);
    connection = g_new0(QemuSlirpILConnection, 1);
    connection->listener = listener;
    connection->backend_connection = backend_connection;
    connection->refs = 1;
    connection->linked = true;
    connection->valid = true;
    connection->send_ready = true;
    QTAILQ_INSERT_TAIL(&listener->connections, connection, entry);
    listener_ref(listener);
    connection_ref(connection);

    listener->callback_depth++;
    connection->callback_depth++;
    connection->opaque = listener->ops.open(connection, listener->opaque);
    connection->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);
    result = connection->valid ? connection : NULL;
    connection_unref(connection);
    listener_unref(listener);
    return result;
}

static void backend_record(void *backend_connection, const uint8_t *data,
                           size_t len, void *opaque)
{
    QemuSlirpILListener *listener = opaque;
    QemuSlirpILConnection *connection;

    connection = find_connection(listener, backend_connection);
    if (!listener->valid || !connection || !connection->valid) {
        return;
    }
    listener_ref(listener);
    connection_ref(connection);
    listener->callback_depth++;
    connection->callback_depth++;
    listener->ops.record(connection, data, len, connection->opaque);
    connection->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);
    connection_unref(connection);
    listener_unref(listener);
}

static void backend_can_send(void *backend_connection, void *opaque)
{
    QemuSlirpILListener *listener = opaque;
    QemuSlirpILConnection *connection;

    connection = find_connection(listener, backend_connection);
    if (!listener->valid || !connection || !connection->valid ||
        connection->send_ready) {
        return;
    }
    listener_ref(listener);
    connection_ref(connection);
    connection->send_ready = true;
    listener->callback_depth++;
    connection->callback_depth++;
    if (listener->ops.can_send) {
        listener->ops.can_send(connection, connection->opaque);
    }
    connection->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);
    connection_unref(connection);
    listener_unref(listener);
}

static void backend_close(void *backend_connection, void *opaque)
{
    QemuSlirpILListener *listener = opaque;
    QemuSlirpILConnection *connection =
        find_connection(listener, backend_connection);

    if (connection) {
        connection_finished(connection);
    }
}

static const QemuSlirpILBackendCallbacks backend_callbacks = {
    .open = backend_open,
    .record = backend_record,
    .can_send = backend_can_send,
    .close = backend_close,
};

QemuSlirpILRegistry *qemu_slirp_il_registry_new(
    bool ipv4_enabled, struct in_addr network, struct in_addr mask,
    struct in_addr vhost, struct in_addr dns,
    const QemuSlirpILBackendOps *ops, void *backend_opaque)
{
    QemuSlirpILRegistry *registry;

    if (!ops || !ops->listen || !ops->listener_remove || !ops->send_record ||
        !ops->connection_close) {
        return NULL;
    }
    registry = g_new0(QemuSlirpILRegistry, 1);
    registry->network = network;
    registry->mask = mask;
    registry->vhost = vhost;
    registry->dns = dns;
    registry->backend = ops;
    registry->backend_opaque = backend_opaque;
    registry->valid = true;
    registry->ipv4_enabled = ipv4_enabled;
    QTAILQ_INIT(&registry->listeners);
    QTAILQ_INIT(&registry->deferred);
    return registry;
}

void qemu_slirp_il_registry_set_ipv4_enabled_for_test(
    QemuSlirpILRegistry *registry, bool enabled)
{
    registry->ipv4_enabled = enabled;
}

void qemu_slirp_il_registry_invalidate(QemuSlirpILRegistry *registry)
{
    QemuSlirpILListener *listener;

    if (!registry || !registry->valid) {
        return;
    }
    registry->valid = false;
    while (!QTAILQ_EMPTY(&registry->listeners)) {
        listener = QTAILQ_FIRST(&registry->listeners);
        listener_ref(listener);
        listener_unlink(listener);
        listener->remove_pending = true;
        listener_finish_remove(listener);
        listener_unref(listener);
    }
    qemu_slirp_il_registry_flush_deferred(registry);
}

void qemu_slirp_il_registry_free(QemuSlirpILRegistry *registry)
{
    if (!registry) {
        return;
    }
    qemu_slirp_il_registry_invalidate(registry);
    assert(QTAILQ_EMPTY(&registry->deferred));
    g_free(registry);
}

int qemu_slirp_il_registry_listen(QemuSlirpILRegistry *registry,
                                  struct in_addr guest_addr,
                                  uint16_t guest_port,
                                  const QemuSlirpILListenerOps *ops,
                                  void *opaque,
                                  QemuSlirpILListener **listener_out,
                                  Error **errp)
{
    uint32_t broadcast;
    QemuSlirpILListener *listener;

    if (listener_out) {
        *listener_out = NULL;
    }
    if (!registry || !registry->valid) {
        error_setg(errp, "SLiRP IL registry is invalid");
        return -1;
    }
    if (!listener_out) {
        error_setg(errp, "SLiRP IL listener output is NULL");
        return -1;
    }
    if (!registry->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return -1;
    }
    if (!ops || !ops->open || !ops->record || !ops->close) {
        error_setg(errp,
                   "SLiRP IL open, record, and close callbacks are required");
        return -1;
    }
    if (!guest_port) {
        error_setg(errp, "SLiRP IL port must not be zero");
        return -1;
    }
    broadcast = registry->network.s_addr | ~registry->mask.s_addr;
    if (!guest_addr.s_addr || (guest_addr.s_addr & registry->mask.s_addr) !=
                              registry->network.s_addr) {
        error_setg(errp, "SLiRP IL address is outside the network");
        return -1;
    }
    if (guest_addr.s_addr == registry->network.s_addr ||
        guest_addr.s_addr == broadcast ||
        guest_addr.s_addr == registry->vhost.s_addr ||
        guest_addr.s_addr == registry->dns.s_addr) {
        error_setg(errp,
                   "SLiRP IL address is reserved by the user-mode network");
        return -1;
    }
    QTAILQ_FOREACH(listener, &registry->listeners, entry) {
        if (listener->addr.s_addr == guest_addr.s_addr &&
            listener->port == guest_port) {
            error_setg(errp, "Conflicting SLiRP IL listener endpoint");
            return -1;
        }
    }

    listener = g_new0(QemuSlirpILListener, 1);
    listener->registry = registry;
    listener->addr = guest_addr;
    listener->port = guest_port;
    listener->ops = *ops;
    listener->opaque = opaque;
    listener->refs = 2;
    listener->registry_ref = true;
    listener->caller_ref = true;
    listener->valid = true;
    QTAILQ_INIT(&listener->connections);
    QTAILQ_INSERT_TAIL(&registry->listeners, listener, entry);
    listener->registered = true;
    *listener_out = listener;
    listener_ref(listener);
    if (registry->backend->listen(registry->backend_opaque, guest_addr,
                                  guest_port, &backend_callbacks, listener,
                                  &listener->backend_listener) < 0) {
        if (listener->valid && listener->caller_ref) {
            listener_remove_internal(listener);
        }
        *listener_out = NULL;
        error_setg(errp, "Failed to register SLiRP IL listener");
        listener_unref(listener);
        return -1;
    }
    if (!listener->valid || !listener->caller_ref || !listener->registered) {
        *listener_out = NULL;
        error_setg(errp, "SLiRP IL listener was removed during setup");
        listener_unref(listener);
        return -1;
    }
    listener_unref(listener);
    return 0;
}

void qemu_slirp_il_registry_flush_deferred(QemuSlirpILRegistry *registry)
{
    GPtrArray *listeners;
    QemuSlirpILListener *listener;
    guint i;

    if (!registry) {
        return;
    }
    listeners = g_ptr_array_new();
    QTAILQ_FOREACH(listener, &registry->deferred, entry) {
        listener_ref(listener);
        g_ptr_array_add(listeners, listener);
    }
    for (i = 0; i < listeners->len; i++) {
        listener = g_ptr_array_index(listeners, i);
        listener_finish_remove(listener);
        listener_unref(listener);
    }
    g_ptr_array_free(listeners, true);
}

void qemu_slirp_il_registry_progress(QemuSlirpILRegistry *registry)
{
    qemu_slirp_il_registry_flush_deferred(registry);
    registry->progress_generation++;
}

unsigned qemu_slirp_il_registry_get_progress_generation(
    QemuSlirpILRegistry *registry)
{
    return registry->progress_generation;
}

void qemu_slirp_il_registry_cleanup(QemuSlirpILRegistry *registry,
                                    QemuSlirpILCleanup cleanup,
                                    void *cleanup_opaque)
{
    qemu_slirp_il_registry_invalidate(registry);
    qemu_slirp_il_registry_free(registry);
    cleanup(cleanup_opaque);
}

int qemu_slirp_il_listen_unavailable(QemuSlirpILListener **listener,
                                      Error **errp)
{
    if (listener) {
        *listener = NULL;
    }
    error_setg(errp,
               "SLiRP IL is unavailable: linked libslirp lacks the public IL API");
    return -1;
}

int qemu_slirp_il_send_record(QemuSlirpILConnection *connection,
                              const uint8_t *data, size_t len)
{
    QemuSlirpILListener *listener;
    int ret;

    if (!connection || !connection->valid) {
        return -ENOTCONN;
    }
    listener = connection->listener;
    if (!listener || !listener->valid) {
        return -ENOTCONN;
    }
    if (!data || !len) {
        return -EINVAL;
    }
    listener_ref(listener);
    connection_ref(connection);
    ret = listener->registry->backend->send_record(
        listener->registry->backend_opaque, connection->backend_connection,
        data, len);
    if (!connection->valid || connection->listener != listener ||
        !listener->valid) {
        ret = -ENOTCONN;
    } else if (ret == -EAGAIN) {
        connection->send_ready = false;
    }
    connection_unref(connection);
    listener_unref(listener);
    return ret > 0 ? -EIO : ret;
}

void qemu_slirp_il_connection_close(QemuSlirpILConnection *connection)
{
    QemuSlirpILListener *listener;

    if (!connection || !connection->valid || connection->close_requested) {
        return;
    }
    listener = connection->listener;
    if (!listener) {
        return;
    }
    connection->close_requested = true;
    listener->registry->backend->connection_close(
        listener->registry->backend_opaque, connection->backend_connection);
}

void qemu_slirp_il_listener_remove(QemuSlirpILListener *listener)
{
    listener_remove_internal(listener);
}
