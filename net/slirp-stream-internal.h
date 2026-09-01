/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_STREAM_INTERNAL_H
#define QEMU_NET_SLIRP_STREAM_INTERNAL_H

#include "net/slirp-stream.h"

typedef struct QemuSlirpStreamRegistry QemuSlirpStreamRegistry;

typedef struct QemuSlirpStreamBackendCallbacks {
    void (*connected)(void *backend_connection,
                      const struct sockaddr_in *peer, void *opaque);
    void (*receive)(void *backend_connection, const uint8_t *data,
                    size_t len, void *opaque);
    void (*can_send)(void *backend_connection, void *opaque);
    void (*closed)(void *backend_connection, void *opaque);
} QemuSlirpStreamBackendCallbacks;

/*
 * The backend must not invoke a callback after listener_remove() returns.
 * listen() stores callbacks_opaque and must return a backend listener handle
 * on success; it may invoke callbacks synchronously while setting up the
 * listener.  listener_remove() closes every accepted connection and may
 * invoke closed() synchronously.  A connection close is terminal and invokes
 * closed() at most once.
 */
typedef struct QemuSlirpStreamBackendOps {
    int (*listen)(void *opaque, struct in_addr address, uint16_t port,
                  const QemuSlirpStreamBackendCallbacks *callbacks,
                  void *callbacks_opaque, void **backend_listener);
    void (*listener_remove)(void *opaque, void *backend_listener);
    size_t (*can_send)(void *opaque, void *backend_connection);
    int (*send)(void *opaque, void *backend_connection,
                const uint8_t *data, size_t len);
    void (*connection_close)(void *opaque, void *backend_connection);
} QemuSlirpStreamBackendOps;

QemuSlirpStreamRegistry *qemu_slirp_stream_registry_new(
    bool ipv4_enabled, struct in_addr vhost,
    const QemuSlirpStreamBackendOps *ops, void *backend_opaque);
void qemu_slirp_stream_registry_set_ipv4_enabled_for_test(
    QemuSlirpStreamRegistry *registry, bool enabled);
int qemu_slirp_stream_registry_listen(
    QemuSlirpStreamRegistry *registry, uint16_t port,
    const QemuSlirpStreamOps *ops, void *opaque,
    QemuSlirpStreamListener **listener, Error **errp);
void qemu_slirp_stream_registry_invalidate(QemuSlirpStreamRegistry *registry);
void qemu_slirp_stream_registry_free(QemuSlirpStreamRegistry *registry);
int qemu_slirp_stream_listen_unavailable(QemuSlirpStreamListener **listener,
                                         Error **errp);

#endif /* QEMU_NET_SLIRP_STREAM_INTERNAL_H */
