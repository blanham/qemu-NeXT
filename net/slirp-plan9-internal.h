/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_PLAN9_INTERNAL_H
#define QEMU_NET_SLIRP_PLAN9_INTERNAL_H

#include "net/slirp-plan9.h"

typedef struct QemuSlirpPlan9Registry QemuSlirpPlan9Registry;

typedef struct QemuSlirpPlan9BackendOps {
    bool (*set_bootp)(void *opaque,
                      const QemuSlirpPlan9BootpConfig *config);
} QemuSlirpPlan9BackendOps;

QemuSlirpPlan9Registry *qemu_slirp_plan9_registry_new(
    bool ipv4_enabled, struct in_addr network, struct in_addr netmask,
    struct in_addr gateway, struct in_addr dns,
    const QemuSlirpPlan9BackendOps *ops, void *backend_opaque);
bool qemu_slirp_plan9_registry_available(QemuSlirpPlan9Registry *registry);
bool qemu_slirp_plan9_registry_claim(QemuSlirpPlan9Registry *registry,
                                     struct in_addr file_server,
                                     struct in_addr auth_server,
                                     QemuSlirpPlan9BootpLease **lease,
                                     Error **errp);
void qemu_slirp_plan9_registry_invalidate(QemuSlirpPlan9Registry *registry);
void qemu_slirp_plan9_registry_free(QemuSlirpPlan9Registry *registry);

#endif /* QEMU_NET_SLIRP_PLAN9_INTERNAL_H */
