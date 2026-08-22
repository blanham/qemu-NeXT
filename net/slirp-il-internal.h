/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_IL_INTERNAL_H
#define QEMU_NET_SLIRP_IL_INTERNAL_H

#include "net/slirp-il.h"

typedef struct QemuSlirpILRegistry QemuSlirpILRegistry;

/* The named-netdev facade is intentionally supplied by Task 2's net/slirp.c. */

typedef struct QemuSlirpILBackendCallbacks {
    void *(*open)(void *backend_connection, void *opaque);
    void (*record)(void *backend_connection, const uint8_t *data, size_t len,
                   void *opaque);
    void (*can_send)(void *backend_connection, void *opaque);
    void (*close)(void *backend_connection, void *opaque);
} QemuSlirpILBackendCallbacks;

/*
 * The backend must make no callback after listener_remove returns.  listen()
 * may invoke callbacks synchronously only after assigning backend_listener.
 * Its close operation eventually invokes callbacks.close exactly once for an
 * accepted connection; it may do so synchronously.
 */
typedef struct QemuSlirpILBackendOps {
    int (*listen)(void *opaque, struct in_addr addr, uint16_t port,
                  const QemuSlirpILBackendCallbacks *callbacks,
                  void *callbacks_opaque, void **backend_listener);
    void (*listener_remove)(void *opaque, void *backend_listener);
    int (*send_record)(void *opaque, void *backend_connection,
                       const uint8_t *data, size_t len);
    void (*connection_close)(void *opaque, void *backend_connection);
} QemuSlirpILBackendOps;

QemuSlirpILRegistry *qemu_slirp_il_registry_new(
    bool ipv4_enabled, struct in_addr network, struct in_addr mask,
    struct in_addr vhost, struct in_addr dns,
    const QemuSlirpILBackendOps *ops, void *backend_opaque);
void qemu_slirp_il_registry_set_ipv4_enabled_for_test(
    QemuSlirpILRegistry *registry, bool enabled);
void qemu_slirp_il_registry_invalidate(QemuSlirpILRegistry *registry);
void qemu_slirp_il_registry_free(QemuSlirpILRegistry *registry);
int qemu_slirp_il_registry_listen(QemuSlirpILRegistry *registry,
                                  struct in_addr guest_addr,
                                  uint16_t guest_port,
                                  const QemuSlirpILListenerOps *ops,
                                  void *opaque,
                                  QemuSlirpILListener **listener,
                                  Error **errp);
void qemu_slirp_il_registry_flush_deferred(QemuSlirpILRegistry *registry);
int qemu_slirp_il_listen_unavailable(QemuSlirpILListener **listener,
                                      Error **errp);

#endif /* QEMU_NET_SLIRP_IL_INTERNAL_H */
