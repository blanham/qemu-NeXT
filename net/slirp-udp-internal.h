/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_UDP_INTERNAL_H
#define QEMU_NET_SLIRP_UDP_INTERNAL_H

#include "net/slirp-udp.h"

typedef struct QemuSlirpUdpRegistry QemuSlirpUdpRegistry;

typedef struct QemuSlirpUdpBackendCallbacks {
    void (*datagram)(const struct sockaddr_in *peer,
                     const uint8_t *data, size_t len,
                     void *opaque);
} QemuSlirpUdpBackendCallbacks;

typedef struct QemuSlirpUdpBackendOps {
    int (*listen)(void *opaque, struct in_addr address, uint16_t port,
                  const QemuSlirpUdpBackendCallbacks *callbacks,
                  void *callbacks_opaque, void **backend_listener);
    void (*listener_remove)(void *opaque, void *backend_listener);
    int (*send)(void *opaque, void *backend_listener,
                const struct sockaddr_in *peer,
                const uint8_t *data, size_t len);
} QemuSlirpUdpBackendOps;

QemuSlirpUdpRegistry *qemu_slirp_udp_registry_new(
    bool ipv4_enabled, struct in_addr vhost,
    const QemuSlirpUdpBackendOps *ops, void *backend_opaque);
void qemu_slirp_udp_registry_set_ipv4_enabled_for_test(
    QemuSlirpUdpRegistry *registry, bool enabled);
int qemu_slirp_udp_registry_listen(QemuSlirpUdpRegistry *registry,
                                   uint16_t port,
                                   const QemuSlirpUdpListenerOps *ops,
                                   void *opaque,
                                   QemuSlirpUdpListener **listener,
                                   Error **errp);
void qemu_slirp_udp_registry_invalidate(QemuSlirpUdpRegistry *registry);
void qemu_slirp_udp_registry_free(QemuSlirpUdpRegistry *registry);
int qemu_slirp_udp_listen_unavailable(QemuSlirpUdpListener **listener,
                                      Error **errp);

#endif /* QEMU_NET_SLIRP_UDP_INTERNAL_H */
