/* SPDX-License-Identifier: NCSA
 * Copyright (c) 2011-2026 Bryce Lanham
 */
#ifndef QEMU_NET_SLIRP_GUESTFWD_INTERNAL_H
#define QEMU_NET_SLIRP_GUESTFWD_INTERNAL_H

#include "net/slirp-guestfwd.h"

typedef struct QemuSlirpGuestFwdRegistry QemuSlirpGuestFwdRegistry;
typedef ssize_t (*QemuSlirpGuestFwdIncoming)(const void *buf, size_t len,
                                             void *opaque);

typedef struct QemuSlirpGuestFwdBackendOps {
    int (*add)(void *opaque, QemuSlirpGuestFwdIncoming incoming,
               void *incoming_opaque, struct in_addr addr, uint16_t port);
    int (*remove)(void *opaque, struct in_addr addr, uint16_t port);
    size_t (*can_send)(void *opaque, struct in_addr addr, uint16_t port);
    int (*send)(void *opaque, struct in_addr addr, uint16_t port,
                const uint8_t *buf, size_t len);
} QemuSlirpGuestFwdBackendOps;

QemuSlirpGuestFwdRegistry *qemu_slirp_guestfwd_registry_new(
    bool ipv4_enabled, struct in_addr network, struct in_addr mask,
    struct in_addr vhost, struct in_addr dns,
    const QemuSlirpGuestFwdBackendOps *ops, void *backend_opaque);
/* Fault injection for otherwise-unreachable defensive checks. */
void qemu_slirp_guestfwd_registry_set_ipv4_enabled_for_test(
    QemuSlirpGuestFwdRegistry *registry, bool enabled);
void qemu_slirp_guestfwd_registry_invalidate(
    QemuSlirpGuestFwdRegistry *registry);
void qemu_slirp_guestfwd_registry_free(QemuSlirpGuestFwdRegistry *registry);
int qemu_slirp_guestfwd_registry_add(QemuSlirpGuestFwdRegistry *registry,
                                     struct in_addr addr, uint16_t port,
                                     const QemuSlirpGuestFwdOps *ops,
                                     void *opaque,
                                     QemuSlirpGuestFwd **handle,
                                     Error **errp);
size_t qemu_slirp_guestfwd_registry_can_send(QemuSlirpGuestFwd *handle);
int qemu_slirp_guestfwd_registry_send(QemuSlirpGuestFwd *handle,
                                      const uint8_t *buf, size_t len);
/* Consumes the caller-owned handle. Set the caller's pointer to NULL and do
 * not reuse it after this call. Passing NULL is safe. Removal from write
 * callbacks is supported and deferred until transport dispatch returns. */
void qemu_slirp_guestfwd_registry_remove(QemuSlirpGuestFwd *handle);
/* Flushes handles whose removal was deferred from a write callback. */
void qemu_slirp_guestfwd_registry_flush_deferred(
    QemuSlirpGuestFwdRegistry *registry);
void qemu_slirp_guestfwd_registry_notify(
    QemuSlirpGuestFwdRegistry *registry);

#endif
