/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_BOOTP_INTERNAL_H
#define QEMU_NET_SLIRP_BOOTP_INTERNAL_H

#include "net/slirp-bootp.h"

typedef struct QemuSlirpBootpRegistry QemuSlirpBootpRegistry;

typedef struct QemuSlirpBootpRootConfig {
    struct in_addr server;
    const char *path;
} QemuSlirpBootpRootConfig;

typedef struct QemuSlirpBootpBackendOps {
    bool (*set_root)(void *opaque,
                     const QemuSlirpBootpRootConfig *config);
} QemuSlirpBootpBackendOps;

QemuSlirpBootpRegistry *qemu_slirp_bootp_registry_new(
    bool ipv4_enabled, struct in_addr vhost,
    const QemuSlirpBootpBackendOps *ops, void *backend_opaque);
bool qemu_slirp_bootp_registry_claim(QemuSlirpBootpRegistry *registry,
                                     const char *root_path,
                                     QemuSlirpBootpRootLease **lease,
                                     Error **errp);
void qemu_slirp_bootp_registry_invalidate(QemuSlirpBootpRegistry *registry);
void qemu_slirp_bootp_registry_free(QemuSlirpBootpRegistry *registry);

#endif /* QEMU_NET_SLIRP_BOOTP_INTERNAL_H */
