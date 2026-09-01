/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "net/slirp-udp-internal.h"
#include "qapi/error.h"

struct QemuSlirpUdpRegistry {
    struct in_addr vhost;
    const QemuSlirpUdpBackendOps *backend;
    void *backend_opaque;
    QTAILQ_HEAD(, QemuSlirpUdpListener) listeners;
    QTAILQ_HEAD(, QemuSlirpUdpListener) deferred;
    bool valid;
    bool ipv4_enabled;
    bool free_pending;
};

struct QemuSlirpUdpListener {
    QTAILQ_ENTRY(QemuSlirpUdpListener) entry;
    QemuSlirpUdpRegistry *registry;
    QemuSlirpUdpListenerOps ops;
    void *opaque;
    void *backend_listener;
    uint16_t port;
    unsigned refs;
    unsigned callback_depth;
    bool registry_ref;
    bool caller_ref;
    bool valid;
    bool registered;
    bool deferred;
    bool remove_pending;
};

static void listener_ref(QemuSlirpUdpListener *listener)
{
    assert(listener->refs);
    listener->refs++;
}

static void listener_unref(QemuSlirpUdpListener *listener)
{
    assert(listener->refs);
    if (!--listener->refs) {
        g_free(listener);
    }
}

static void registry_maybe_free(QemuSlirpUdpRegistry *registry)
{
    if (registry->free_pending && QTAILQ_EMPTY(&registry->listeners) &&
        QTAILQ_EMPTY(&registry->deferred)) {
        g_free(registry);
    }
}

static void listener_unlink(QemuSlirpUdpListener *listener)
{
    QemuSlirpUdpRegistry *registry = listener->registry;

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

static void listener_finish_remove(QemuSlirpUdpListener *listener)
{
    QemuSlirpUdpRegistry *registry;

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
        registry->backend->listener_remove(registry->backend_opaque,
                                           listener->backend_listener);
        listener->backend_listener = NULL;
    }
    if (listener->registry_ref) {
        listener->registry_ref = false;
        listener->registry = NULL;
        listener_unref(listener);
    }
    if (registry) {
        registry_maybe_free(registry);
    }
}

static void listener_remove_internal(QemuSlirpUdpListener *listener)
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

static void backend_datagram(const struct sockaddr_in *peer,
                             const uint8_t *data, size_t len, void *opaque)
{
    QemuSlirpUdpListener *listener = opaque;

    if (!listener->valid) {
        return;
    }
    listener_ref(listener);
    listener->callback_depth++;
    listener->ops.datagram(listener, peer, data, len, listener->opaque);
    listener->callback_depth--;
    listener_finish_remove(listener);
    listener_unref(listener);
}

static const QemuSlirpUdpBackendCallbacks backend_callbacks = {
    .datagram = backend_datagram,
};

QemuSlirpUdpRegistry *qemu_slirp_udp_registry_new(
    bool ipv4_enabled, struct in_addr vhost,
    const QemuSlirpUdpBackendOps *ops, void *backend_opaque)
{
    QemuSlirpUdpRegistry *registry = g_new0(QemuSlirpUdpRegistry, 1);

    registry->vhost = vhost;
    registry->backend = ops;
    registry->backend_opaque = backend_opaque;
    registry->valid = true;
    registry->ipv4_enabled = ipv4_enabled;
    QTAILQ_INIT(&registry->listeners);
    QTAILQ_INIT(&registry->deferred);
    return registry;
}

void qemu_slirp_udp_registry_set_ipv4_enabled_for_test(
    QemuSlirpUdpRegistry *registry, bool enabled)
{
    registry->ipv4_enabled = enabled;
}

int qemu_slirp_udp_registry_listen_full(
    QemuSlirpUdpRegistry *registry, uint16_t port,
    QemuSlirpUdpListenFlags flags, const QemuSlirpUdpListenerOps *ops,
    void *opaque, QemuSlirpUdpListener **listener_out, Error **errp)
{
    QemuSlirpUdpListener *listener;

    if (listener_out) {
        *listener_out = NULL;
    }
    if (!registry || !registry->valid) {
        error_setg(errp, "SLiRP UDP registry is invalid");
        return -1;
    }
    if (!listener_out) {
        error_setg(errp, "SLiRP UDP listener output is NULL");
        return -1;
    }
    if ((unsigned)flags & ~(unsigned)QEMU_SLIRP_UDP_LISTEN_BROADCAST) {
        error_setg(errp, "Unknown SLiRP UDP listener flags");
        return -1;
    }
    if (!registry->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return -1;
    }
    if (!registry->backend || !registry->backend->listen ||
        !registry->backend->listener_remove || !registry->backend->send) {
        error_setg(errp, "SLiRP UDP services are unavailable in this libslirp");
        return -1;
    }
    if (!ops || !ops->datagram) {
        error_setg(errp, "SLiRP UDP datagram callback is required");
        return -1;
    }
    if (!port) {
        error_setg(errp, "SLiRP UDP port must not be zero");
        return -1;
    }
    QTAILQ_FOREACH(listener, &registry->listeners, entry) {
        if (listener->port == port) {
            error_setg(errp, "Conflicting SLiRP UDP listener endpoint");
            return -1;
        }
    }

    listener = g_new0(QemuSlirpUdpListener, 1);
    listener->registry = registry;
    listener->ops = *ops;
    listener->opaque = opaque;
    listener->port = port;
    listener->refs = 2;
    listener->registry_ref = true;
    listener->caller_ref = true;
    listener->valid = true;
    QTAILQ_INSERT_TAIL(&registry->listeners, listener, entry);
    if (registry->backend->listen(registry->backend_opaque, registry->vhost,
                                  port, flags, &backend_callbacks, listener,
                                  &listener->backend_listener) < 0) {
        listener->caller_ref = false;
        listener_unlink(listener);
        listener->remove_pending = true;
        listener_finish_remove(listener);
        listener_unref(listener);
        error_setg(errp, "Failed to register SLiRP UDP listener");
        return -1;
    }
    listener->registered = true;
    *listener_out = listener;
    return 0;
}

int qemu_slirp_udp_registry_listen(QemuSlirpUdpRegistry *registry,
                                   uint16_t port,
                                   const QemuSlirpUdpListenerOps *ops,
                                   void *opaque,
                                   QemuSlirpUdpListener **listener,
                                   Error **errp)
{
    return qemu_slirp_udp_registry_listen_full(
        registry, port, QEMU_SLIRP_UDP_LISTEN_DEFAULT, ops, opaque, listener,
        errp);
}

int qemu_slirp_udp_send(QemuSlirpUdpListener *listener,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len)
{
    QemuSlirpUdpRegistry *registry;
    int ret;

    if (!listener || !listener->valid || !listener->registered) {
        return -ENOTCONN;
    }
    if (!peer || (!data && len)) {
        return -EINVAL;
    }
    listener_ref(listener);
    registry = listener->registry;
    ret = registry->backend->send(registry->backend_opaque,
                                  listener->backend_listener, peer, data, len);
    if (!listener->valid || listener->registry != registry) {
        ret = -ENOTCONN;
    }
    listener_unref(listener);
    return ret > 0 ? -EIO : ret;
}

void qemu_slirp_udp_listener_remove(QemuSlirpUdpListener *listener)
{
    listener_remove_internal(listener);
}

void qemu_slirp_udp_registry_invalidate(QemuSlirpUdpRegistry *registry)
{
    QemuSlirpUdpListener *listener;

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
}

void qemu_slirp_udp_registry_free(QemuSlirpUdpRegistry *registry)
{
    if (!registry) {
        return;
    }
    qemu_slirp_udp_registry_invalidate(registry);
    registry->free_pending = true;
    registry_maybe_free(registry);
}

int qemu_slirp_udp_listen_unavailable(QemuSlirpUdpListener **listener,
                                      Error **errp)
{
    if (listener) {
        *listener = NULL;
    }
    error_setg(errp,
               "SLiRP UDP services are unavailable: linked libslirp lacks "
               "the public guest UDP API");
    return -1;
}

int qemu_slirp_udp_listen_full_unavailable(QemuSlirpUdpListener **listener,
                                           Error **errp)
{
    if (listener) {
        *listener = NULL;
    }
    error_setg(errp,
               "SLiRP UDP broadcast listeners are unavailable: linked "
               "libslirp lacks the public slirp_udp_listen_full API");
    return -1;
}
