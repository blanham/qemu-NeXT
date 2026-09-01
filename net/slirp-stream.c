/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "net/slirp-stream-internal.h"
#include "qapi/error.h"

struct QemuSlirpStreamRegistry {
    struct in_addr vhost;
    const QemuSlirpStreamBackendOps *backend;
    void *backend_opaque;
    QTAILQ_HEAD(, QemuSlirpStreamListener) listeners;
    QTAILQ_HEAD(, QemuSlirpStreamListener) deferred;
    bool valid;
    bool ipv4_enabled;
    bool free_pending;
    unsigned refs;
};

struct QemuSlirpStreamListener {
    QTAILQ_ENTRY(QemuSlirpStreamListener) entry;
    QemuSlirpStreamRegistry *registry;
    QemuSlirpStreamOps ops;
    void *opaque;
    void *backend_listener;
    QTAILQ_HEAD(, QemuSlirpStream) streams;
    uint16_t port;
    unsigned refs;
    unsigned callback_depth;
    bool registry_ref;
    bool caller_ref;
    bool valid;
    bool registered;
    bool deferred;
    bool remove_pending;
    bool setup_pending;
};

struct QemuSlirpStream {
    QTAILQ_ENTRY(QemuSlirpStream) entry;
    QemuSlirpStreamListener *listener;
    void *backend_connection;
    unsigned refs;
    unsigned callback_depth;
    bool linked;
    bool valid;
    bool close_requested;
    bool close_notified;
    bool send_blocked;
};

static void listener_finish_remove(QemuSlirpStreamListener *listener);
static void stream_notify_closed(QemuSlirpStream *stream);

static void registry_maybe_free(QemuSlirpStreamRegistry *registry)
{
    if (registry->free_pending && !registry->refs &&
        QTAILQ_EMPTY(&registry->listeners) &&
        QTAILQ_EMPTY(&registry->deferred)) {
        g_free(registry);
    }
}

static void registry_ref(QemuSlirpStreamRegistry *registry)
{
    g_assert(registry != NULL);
    registry->refs++;
}

static void registry_unref(QemuSlirpStreamRegistry *registry)
{
    g_assert(registry != NULL);
    g_assert(registry->refs > 0);
    registry->refs--;
    registry_maybe_free(registry);
}

static void listener_ref(QemuSlirpStreamListener *listener)
{
    g_assert(listener != NULL);
    g_assert(listener->refs > 0);
    listener->refs++;
}

static void listener_unref(QemuSlirpStreamListener *listener)
{
    g_assert(listener != NULL);
    g_assert(listener->refs > 0);
    if (!--listener->refs) {
        g_free(listener);
    }
}

static void stream_ref(QemuSlirpStream *stream)
{
    g_assert(stream != NULL);
    g_assert(stream->refs > 0);
    stream->refs++;
}

static void stream_unref(QemuSlirpStream *stream)
{
    g_assert(stream != NULL);
    g_assert(stream->refs > 0);
    if (!--stream->refs) {
        g_free(stream);
    }
}

static QemuSlirpStream *find_stream(QemuSlirpStreamListener *listener,
                                    void *backend_connection)
{
    QemuSlirpStream *stream;

    QTAILQ_FOREACH(stream, &listener->streams, entry) {
        if (stream->backend_connection == backend_connection) {
            return stream;
        }
    }
    return NULL;
}

static void stream_unlink(QemuSlirpStream *stream)
{
    QemuSlirpStreamListener *listener = stream->listener;

    if (!stream->linked) {
        return;
    }
    QTAILQ_REMOVE(&listener->streams, stream, entry);
    stream->linked = false;
    stream->valid = false;
    stream->listener = NULL;
    listener_unref(listener);
    stream_unref(stream);
}

static void listener_unlink(QemuSlirpStreamListener *listener)
{
    QemuSlirpStreamRegistry *registry = listener->registry;

    if (!listener->valid) {
        return;
    }
    listener->valid = false;
    if (listener->registry_ref) {
        QTAILQ_REMOVE(&registry->listeners, listener, entry);
        QTAILQ_INSERT_TAIL(&registry->deferred, listener, entry);
        listener->deferred = true;
    }
}

static void stream_notify_closed(QemuSlirpStream *stream)
{
    QemuSlirpStreamListener *listener;
    QemuSlirpStreamRegistry *registry;

    if (!stream || stream->close_notified) {
        return;
    }
    listener = stream->listener;
    if (!listener) {
        stream->close_notified = true;
        stream->valid = false;
        return;
    }
    listener_ref(listener);
    stream_ref(stream);
    registry = listener->registry;
    if (registry) {
        registry_ref(registry);
    }

    stream->close_notified = true;
    stream->close_requested = true;
    stream->backend_connection = NULL;
    stream_unlink(stream);

    listener->callback_depth++;
    stream->callback_depth++;
    if (listener->ops.closed) {
        listener->ops.closed(stream, listener->opaque);
    }
    stream->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);

    if (registry) {
        registry_unref(registry);
    }
    listener_unref(listener);
    stream_unref(stream);
}

static void listener_finish_remove(QemuSlirpStreamListener *listener)
{
    QemuSlirpStreamRegistry *registry;
    QemuSlirpStream *stream;
    void *backend_listener;

    if (!listener || !listener->remove_pending ||
        listener->callback_depth || listener->setup_pending) {
        return;
    }
    registry = listener->registry;
    if (!registry) {
        listener->remove_pending = false;
        return;
    }

    listener_ref(listener);
    registry_ref(registry);
    listener->remove_pending = false;

    backend_listener = listener->backend_listener;
    listener->backend_listener = NULL;
    if (listener->registered) {
        listener->registered = false;
        if (backend_listener && registry->backend &&
            registry->backend->listener_remove) {
            registry->backend->listener_remove(registry->backend_opaque,
                                               backend_listener);
        }
    }

    /*
     * A conforming backend closes all connections from listener_remove().
     * The fallback also completes any backend that reports removal without
     * delivering a closed callback.
     */
    while (!QTAILQ_EMPTY(&listener->streams)) {
        stream = QTAILQ_FIRST(&listener->streams);
        stream_ref(stream);
        stream_notify_closed(stream);
        stream_unref(stream);
    }

    if (listener->deferred) {
        QTAILQ_REMOVE(&registry->deferred, listener, entry);
        listener->deferred = false;
    }
    if (listener->registry_ref) {
        listener->registry_ref = false;
        listener->registry = NULL;
        listener_unref(listener);
    }
    listener_unref(listener);
    registry_unref(registry);
}

static void listener_remove_internal(QemuSlirpStreamListener *listener)
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

static void backend_connected(void *backend_connection,
                              const struct sockaddr_in *peer, void *opaque)
{
    QemuSlirpStreamListener *listener = opaque;
    QemuSlirpStreamRegistry *registry;
    QemuSlirpStream *stream;

    if (!listener || !listener->valid || !listener->registry) {
        return;
    }
    registry = listener->registry;
    listener_ref(listener);
    registry_ref(registry);
    stream = g_new0(QemuSlirpStream, 1);
    stream->listener = listener;
    stream->backend_connection = backend_connection;
    stream->refs = 1;
    stream->linked = true;
    stream->valid = true;
    QTAILQ_INSERT_TAIL(&listener->streams, stream, entry);
    listener_ref(listener);
    stream_ref(stream);

    listener->callback_depth++;
    stream->callback_depth++;
    listener->ops.connected(stream, peer, listener->opaque);
    stream->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);

    stream_unref(stream);
    listener_unref(listener);
    registry_unref(registry);
}

static void backend_receive(void *backend_connection, const uint8_t *data,
                            size_t len, void *opaque)
{
    QemuSlirpStreamListener *listener = opaque;
    QemuSlirpStream *stream;
    QemuSlirpStreamRegistry *registry;

    if (!listener || !listener->valid) {
        return;
    }
    stream = find_stream(listener, backend_connection);
    if (!stream || !stream->valid) {
        return;
    }
    registry = listener->registry;
    listener_ref(listener);
    stream_ref(stream);
    if (registry) {
        registry_ref(registry);
    }
    listener->callback_depth++;
    stream->callback_depth++;
    listener->ops.receive(stream, data, len, listener->opaque);
    stream->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);
    if (registry) {
        registry_unref(registry);
    }
    stream_unref(stream);
    listener_unref(listener);
}

static void backend_can_send(void *backend_connection, void *opaque)
{
    QemuSlirpStreamListener *listener = opaque;
    QemuSlirpStream *stream;
    QemuSlirpStreamRegistry *registry;

    if (!listener || !listener->valid) {
        return;
    }
    stream = find_stream(listener, backend_connection);
    if (!stream || !stream->valid || !stream->send_blocked) {
        return;
    }
    registry = listener->registry;
    listener_ref(listener);
    stream_ref(stream);
    if (registry) {
        registry_ref(registry);
    }
    stream->send_blocked = false;
    listener->callback_depth++;
    stream->callback_depth++;
    if (listener->ops.can_send) {
        listener->ops.can_send(stream, listener->opaque);
    }
    stream->callback_depth--;
    listener->callback_depth--;
    listener_finish_remove(listener);
    if (registry) {
        registry_unref(registry);
    }
    stream_unref(stream);
    listener_unref(listener);
}

static void backend_closed(void *backend_connection, void *opaque)
{
    QemuSlirpStreamListener *listener = opaque;
    QemuSlirpStream *stream;

    if (!listener) {
        return;
    }
    stream = find_stream(listener, backend_connection);
    if (!stream) {
        return;
    }
    stream_notify_closed(stream);
}

static const QemuSlirpStreamBackendCallbacks backend_callbacks = {
    .connected = backend_connected,
    .receive = backend_receive,
    .can_send = backend_can_send,
    .closed = backend_closed,
};

QemuSlirpStreamRegistry *qemu_slirp_stream_registry_new(
    bool ipv4_enabled, struct in_addr vhost,
    const QemuSlirpStreamBackendOps *ops, void *backend_opaque)
{
    QemuSlirpStreamRegistry *registry = g_new0(QemuSlirpStreamRegistry, 1);

    registry->vhost = vhost;
    registry->backend = ops;
    registry->backend_opaque = backend_opaque;
    registry->valid = true;
    registry->ipv4_enabled = ipv4_enabled;
    QTAILQ_INIT(&registry->listeners);
    QTAILQ_INIT(&registry->deferred);
    return registry;
}

void qemu_slirp_stream_registry_set_ipv4_enabled_for_test(
    QemuSlirpStreamRegistry *registry, bool enabled)
{
    if (registry) {
        registry->ipv4_enabled = enabled;
    }
}

int qemu_slirp_stream_registry_listen(
    QemuSlirpStreamRegistry *registry, uint16_t port,
    const QemuSlirpStreamOps *ops, void *opaque,
    QemuSlirpStreamListener **listener_out, Error **errp)
{
    QemuSlirpStreamListener *listener;
    int ret;

    if (listener_out) {
        *listener_out = NULL;
    }
    if (!registry || !registry->valid) {
        error_setg(errp, "SLiRP stream registry is invalid");
        return -1;
    }
    if (!listener_out) {
        error_setg(errp, "SLiRP stream listener output is NULL");
        return -1;
    }
    if (!registry->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return -1;
    }
    if (!registry->backend || !registry->backend->listen ||
        !registry->backend->listener_remove ||
        !registry->backend->can_send || !registry->backend->send ||
        !registry->backend->connection_close) {
        error_setg(errp,
                   "SLiRP TCP stream services are unavailable in this libslirp");
        return -1;
    }
    if (!ops || !ops->connected || !ops->receive || !ops->closed) {
        error_setg(errp,
                   "SLiRP stream connected, receive, and closed callbacks "
                   "are required");
        return -1;
    }
    if (!port) {
        error_setg(errp, "SLiRP stream port must not be zero");
        return -1;
    }
    QTAILQ_FOREACH(listener, &registry->listeners, entry) {
        if (listener->port == port) {
            error_setg(errp, "Conflicting SLiRP stream listener endpoint");
            return -1;
        }
    }

    listener = g_new0(QemuSlirpStreamListener, 1);
    listener->registry = registry;
    listener->ops = *ops;
    listener->opaque = opaque;
    listener->port = port;
    listener->refs = 2;
    listener->registry_ref = true;
    listener->caller_ref = true;
    listener->valid = true;
    listener->registered = false;
    listener->setup_pending = true;
    QTAILQ_INIT(&listener->streams);
    QTAILQ_INSERT_TAIL(&registry->listeners, listener, entry);
    *listener_out = listener;

    listener_ref(listener);
    registry_ref(registry);
    {
        void *backend_listener = NULL;

        ret = registry->backend->listen(registry->backend_opaque,
                                        registry->vhost, port,
                                        &backend_callbacks, listener,
                                        &backend_listener);
        listener->backend_listener = backend_listener;
        listener->registered = backend_listener != NULL;
        listener->setup_pending = false;
    }
    if (ret < 0 || !listener->valid || !listener->caller_ref ||
        !listener->backend_listener) {
        if (listener->caller_ref) {
            listener_remove_internal(listener);
        } else {
            listener->remove_pending = true;
            listener_finish_remove(listener);
        }
        *listener_out = NULL;
        error_setg(errp,
                   ret < 0 ? "Failed to register SLiRP TCP stream listener"
                           : "SLiRP stream listener was removed during setup");
        registry_unref(registry);
        listener_unref(listener);
        return -1;
    }
    registry_unref(registry);
    listener_unref(listener);
    return 0;
}

void qemu_slirp_stream_registry_invalidate(QemuSlirpStreamRegistry *registry)
{
    QemuSlirpStreamListener *listener;

    if (!registry) {
        return;
    }
    registry_ref(registry);
    if (!registry->valid) {
        registry_unref(registry);
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
    registry_unref(registry);
}

void qemu_slirp_stream_registry_free(QemuSlirpStreamRegistry *registry)
{
    if (!registry) {
        return;
    }
    registry_ref(registry);
    qemu_slirp_stream_registry_invalidate(registry);
    registry->free_pending = true;
    registry_unref(registry);
}

size_t qemu_slirp_stream_can_send(QemuSlirpStream *stream)
{
    QemuSlirpStreamListener *listener;
    QemuSlirpStreamRegistry *registry;
    size_t ret;

    if (!stream || !stream->valid || stream->close_requested) {
        return 0;
    }
    listener = stream->listener;
    if (!listener || !listener->valid ||
        (!listener->registered && !listener->setup_pending)) {
        return 0;
    }
    registry = listener->registry;
    if (!registry || !registry->backend || !registry->backend->can_send) {
        return 0;
    }
    listener_ref(listener);
    stream_ref(stream);
    registry_ref(registry);
    ret = registry->backend->can_send(registry->backend_opaque,
                                      stream->backend_connection);
    if (!stream->valid || stream->listener != listener || !listener->valid) {
        ret = 0;
    }
    registry_unref(registry);
    stream_unref(stream);
    listener_unref(listener);
    return ret;
}

int qemu_slirp_stream_send(QemuSlirpStream *stream, const uint8_t *data,
                           size_t len)
{
    QemuSlirpStreamListener *listener;
    QemuSlirpStreamRegistry *registry;
    int ret;

    if (!stream || !stream->valid || stream->close_requested) {
        return -ENOTCONN;
    }
    listener = stream->listener;
    if (!listener || !listener->valid ||
        (!listener->registered && !listener->setup_pending)) {
        return -ENOTCONN;
    }
    registry = listener->registry;
    if (!registry || !registry->backend || !registry->backend->send) {
        return -ENOTCONN;
    }
    if ((data == NULL && len != 0) || len > INT_MAX) {
        return -EINVAL;
    }
    listener_ref(listener);
    stream_ref(stream);
    registry_ref(registry);
    ret = registry->backend->send(registry->backend_opaque,
                                  stream->backend_connection, data, len);
    if (!stream->valid || stream->listener != listener || !listener->valid) {
        ret = -ENOTCONN;
    } else if (ret == -EAGAIN) {
        stream->send_blocked = true;
    } else if (ret == 0 && len != 0) {
        stream->send_blocked = false;
    }
    registry_unref(registry);
    stream_unref(stream);
    listener_unref(listener);
    return ret > 0 ? -EIO : ret;
}

void qemu_slirp_stream_close(QemuSlirpStream *stream)
{
    QemuSlirpStreamListener *listener;
    QemuSlirpStreamRegistry *registry;

    if (!stream || !stream->valid || stream->close_requested) {
        return;
    }
    listener = stream->listener;
    if (!listener) {
        return;
    }
    listener_ref(listener);
    stream_ref(stream);
    registry = listener->registry;
    if (registry) {
        registry_ref(registry);
    }
    stream->close_requested = true;
    if (registry && registry->backend &&
        registry->backend->connection_close && stream->backend_connection) {
        registry->backend->connection_close(registry->backend_opaque,
                                             stream->backend_connection);
    } else {
        stream_notify_closed(stream);
    }
    if (registry) {
        registry_unref(registry);
    }
    stream_unref(stream);
    listener_unref(listener);
}

void qemu_slirp_stream_listener_remove(QemuSlirpStreamListener *listener)
{
    listener_remove_internal(listener);
}

int qemu_slirp_stream_listen_unavailable(QemuSlirpStreamListener **listener,
                                         Error **errp)
{
    if (listener) {
        *listener = NULL;
    }
    error_setg(errp,
               "SLiRP TCP stream services are unavailable: linked libslirp "
               "lacks the public TCP service API");
    return -1;
}
