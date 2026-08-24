/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-bootp-internal.h"
#include "qapi/error.h"

struct QemuSlirpBootpRegistry {
    struct in_addr vhost;
    const QemuSlirpBootpBackendOps *backend;
    void *backend_opaque;
    QemuSlirpBootpRootLease *owner;
    bool valid;
    bool ipv4_enabled;
};

struct QemuSlirpBootpRootLease {
    QemuSlirpBootpRegistry *registry;
};

QemuSlirpBootpRegistry *qemu_slirp_bootp_registry_new(
    bool ipv4_enabled, struct in_addr vhost,
    const QemuSlirpBootpBackendOps *ops, void *backend_opaque)
{
    QemuSlirpBootpRegistry *registry = g_new0(QemuSlirpBootpRegistry, 1);

    registry->vhost = vhost;
    registry->backend = ops;
    registry->backend_opaque = backend_opaque;
    registry->valid = true;
    registry->ipv4_enabled = ipv4_enabled;
    return registry;
}

bool qemu_slirp_bootp_registry_claim(QemuSlirpBootpRegistry *registry,
                                     const char *root_path,
                                     QemuSlirpBootpRootLease **lease,
                                     Error **errp)
{
    QemuSlirpBootpRootConfig config;
    QemuSlirpBootpRootLease *owner;
    size_t length;

    if (lease) {
        *lease = NULL;
    }
    if (!registry || !registry->valid) {
        error_setg(errp, "SLiRP BOOTP root registry is invalid");
        return false;
    }
    if (!lease) {
        error_setg(errp, "SLiRP BOOTP root lease output is NULL");
        return false;
    }
    if (!registry->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return false;
    }
    if (!registry->backend || !registry->backend->set_root) {
        error_setg(errp, "SLiRP BOOTP root is unavailable in this libslirp");
        return false;
    }
    if (registry->owner) {
        error_setg(errp, "Another endpoint already owns the BOOTP root lease");
        return false;
    }
    length = root_path ? strlen(root_path) : 0;
    if (!length || root_path[0] != '/' || length > 255) {
        error_setg(errp, "BOOTP root path must be an absolute 1-255 byte path");
        return false;
    }

    config = (QemuSlirpBootpRootConfig) {
        .server = registry->vhost,
        .path = root_path,
    };
    owner = g_new0(QemuSlirpBootpRootLease, 1);
    if (!registry->backend->set_root(registry->backend_opaque, &config)) {
        g_free(owner);
        error_setg(errp, "Invalid SLiRP BOOTP root configuration");
        return false;
    }
    owner->registry = registry;
    registry->owner = owner;
    *lease = owner;
    return true;
}

void qemu_slirp_bootp_root_release(QemuSlirpBootpRootLease **lease)
{
    QemuSlirpBootpRootLease *owner;
    QemuSlirpBootpRegistry *registry;

    if (!lease || !*lease) {
        return;
    }
    owner = *lease;
    *lease = NULL;
    registry = owner->registry;
    owner->registry = NULL;
    if (registry && registry->owner == owner) {
        registry->owner = NULL;
        registry->backend->set_root(registry->backend_opaque, NULL);
    }
    g_free(owner);
}

void qemu_slirp_bootp_registry_invalidate(QemuSlirpBootpRegistry *registry)
{
    QemuSlirpBootpRootLease *owner;

    if (!registry || !registry->valid) {
        return;
    }
    registry->valid = false;
    owner = registry->owner;
    registry->owner = NULL;
    if (owner) {
        owner->registry = NULL;
        registry->backend->set_root(registry->backend_opaque, NULL);
    }
}

void qemu_slirp_bootp_registry_free(QemuSlirpBootpRegistry *registry)
{
    if (!registry) {
        return;
    }
    qemu_slirp_bootp_registry_invalidate(registry);
    g_free(registry);
}
