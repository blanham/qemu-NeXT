/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-plan9-internal.h"
#include "qapi/error.h"

struct QemuSlirpPlan9Registry {
    struct in_addr network;
    struct in_addr netmask;
    struct in_addr gateway;
    struct in_addr dns;
    const QemuSlirpPlan9BackendOps *ops;
    void *backend_opaque;
    QemuSlirpPlan9BootpLease *owner;
    bool valid;
    bool ipv4_enabled;
};

struct QemuSlirpPlan9BootpLease {
    QemuSlirpPlan9Registry *registry;
};

QemuSlirpPlan9Registry *qemu_slirp_plan9_registry_new(
    bool ipv4_enabled, struct in_addr network, struct in_addr netmask,
    struct in_addr gateway, struct in_addr dns,
    const QemuSlirpPlan9BackendOps *ops, void *backend_opaque)
{
    QemuSlirpPlan9Registry *registry = g_new0(QemuSlirpPlan9Registry, 1);

    registry->network = network;
    registry->netmask = netmask;
    registry->gateway = gateway;
    registry->dns = dns;
    registry->ops = ops;
    registry->backend_opaque = backend_opaque;
    registry->valid = true;
    registry->ipv4_enabled = ipv4_enabled;
    return registry;
}

bool qemu_slirp_plan9_registry_available(QemuSlirpPlan9Registry *registry)
{
    return registry && registry->valid && registry->ipv4_enabled &&
           registry->ops && registry->ops->set_bootp;
}

static bool address_valid(QemuSlirpPlan9Registry *registry,
                          struct in_addr address, bool allow_zero,
                          Error **errp)
{
    uint32_t broadcast;

    if (!address.s_addr && allow_zero) {
        return true;
    }
    broadcast = registry->network.s_addr | ~registry->netmask.s_addr;
    if (!address.s_addr ||
        (address.s_addr & registry->netmask.s_addr) !=
        registry->network.s_addr) {
        error_setg(errp, "Plan 9 BOOTP address is outside the network");
        return false;
    }
    if (address.s_addr == registry->network.s_addr) {
        error_setg(errp, "Plan 9 BOOTP address is the network address");
        return false;
    }
    if (address.s_addr == broadcast) {
        error_setg(errp, "Plan 9 BOOTP address is the broadcast address");
        return false;
    }
    if (address.s_addr == registry->gateway.s_addr) {
        error_setg(errp, "Plan 9 BOOTP address is the user-mode host address");
        return false;
    }
    if (address.s_addr == registry->dns.s_addr) {
        error_setg(errp, "Plan 9 BOOTP address is the DNS address");
        return false;
    }
    return true;
}

bool qemu_slirp_plan9_registry_claim(QemuSlirpPlan9Registry *registry,
                                     struct in_addr file_server,
                                     struct in_addr auth_server,
                                     QemuSlirpPlan9BootpLease **lease,
                                     Error **errp)
{
    QemuSlirpPlan9BootpConfig config;
    QemuSlirpPlan9BootpLease *owner;

    if (lease) {
        *lease = NULL;
    }
    if (!registry || !registry->valid) {
        error_setg(errp, "Plan 9 BOOTP registry is invalid");
        return false;
    }
    if (!lease) {
        error_setg(errp, "Plan 9 BOOTP lease output is NULL");
        return false;
    }
    if (!registry->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return false;
    }
    if (!registry->ops || !registry->ops->set_bootp) {
        error_setg(errp, "Plan 9 BOOTP is unavailable in this libslirp");
        return false;
    }
    if (registry->owner) {
        error_setg(errp, "Another endpoint already owns Plan 9 BOOTP");
        return false;
    }
    if (!address_valid(registry, file_server, false, errp) ||
        !address_valid(registry, auth_server, true, errp)) {
        return false;
    }

    config = (QemuSlirpPlan9BootpConfig) {
        .netmask = registry->netmask,
        .file_server = file_server,
        .auth_server = auth_server,
        .gateway = registry->gateway,
    };
    owner = g_new0(QemuSlirpPlan9BootpLease, 1);
    if (!registry->ops->set_bootp(registry->backend_opaque, &config)) {
        g_free(owner);
        error_setg(errp, "Invalid Plan 9 BOOTP configuration");
        return false;
    }
    owner->registry = registry;
    registry->owner = owner;
    *lease = owner;
    return true;
}

void qemu_slirp_plan9_bootp_release(QemuSlirpPlan9BootpLease **lease)
{
    QemuSlirpPlan9BootpLease *owner;
    QemuSlirpPlan9Registry *registry;

    if (!lease) {
        return;
    }
    owner = *lease;
    if (!owner) {
        return;
    }
    *lease = NULL;
    registry = owner->registry;
    owner->registry = NULL;
    if (registry && registry->owner == owner) {
        registry->owner = NULL;
        registry->ops->set_bootp(registry->backend_opaque, NULL);
    }
    g_free(owner);
}

void qemu_slirp_plan9_registry_invalidate(QemuSlirpPlan9Registry *registry)
{
    QemuSlirpPlan9BootpLease *owner;

    if (!registry || !registry->valid) {
        return;
    }
    registry->valid = false;
    owner = registry->owner;
    registry->owner = NULL;
    if (owner) {
        owner->registry = NULL;
        registry->ops->set_bootp(registry->backend_opaque, NULL);
    }
}

void qemu_slirp_plan9_registry_free(QemuSlirpPlan9Registry *registry)
{
    if (!registry) {
        return;
    }
    qemu_slirp_plan9_registry_invalidate(registry);
    g_free(registry);
}
