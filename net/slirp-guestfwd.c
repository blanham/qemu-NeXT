/* SPDX-License-Identifier: NCSA
 * Copyright (c) 2011-2026 Bryce Lanham
 */
#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "net/slirp-guestfwd-internal.h"
#include "qapi/error.h"

struct QemuSlirpGuestFwdRegistry {
    struct in_addr network, mask, vhost, dns;
    const QemuSlirpGuestFwdBackendOps *backend;
    void *backend_opaque;
    QTAILQ_HEAD(, QemuSlirpGuestFwd) forwards;
    bool valid;
    bool ipv4_enabled;
    QemuSlirpGuestFwd *plan9_owner;
};

struct QemuSlirpGuestFwd {
    QTAILQ_ENTRY(QemuSlirpGuestFwd) entry;
    QemuSlirpGuestFwdRegistry *registry;
    struct in_addr addr;
    uint16_t port;
    const QemuSlirpGuestFwdOps *ops;
    void *opaque;
    unsigned refs;
    bool registry_ref, caller_ref, valid, registered;
    unsigned callback_depth;
    bool remove_pending;
};

static void fwd_ref(QemuSlirpGuestFwd *fwd)
{
    assert(fwd->refs);
    fwd->refs++;
}
static void fwd_unref(QemuSlirpGuestFwd *fwd)
{
    assert(fwd->refs);
    if (!--fwd->refs)
        g_free(fwd);
}

static ssize_t incoming(const void *buf, size_t len, void *opaque)
{
    QemuSlirpGuestFwd *fwd = opaque;
    ssize_t ret;

    if (!fwd->valid || !fwd->ops || !fwd->ops->write)
        return -ENOTCONN;
    fwd->callback_depth++;
    ret = fwd->ops->write(buf, len, fwd->opaque);
    fwd->callback_depth--;
    return ret;
}

static void detach(QemuSlirpGuestFwd *fwd)
{
    QemuSlirpGuestFwdRegistry *r = fwd->registry;
    if (!fwd->valid)
        return;
    fwd->valid = false;
    if (fwd->registered) {
        r->backend->remove(r->backend_opaque, fwd->addr, fwd->port);
        fwd->registered = false;
    }
    if (r->plan9_owner == fwd) {
        r->backend->set_plan9_bootp(r->backend_opaque, NULL);
        r->plan9_owner = NULL;
    }
    if (fwd->registry_ref) {
        QTAILQ_REMOVE(&r->forwards, fwd, entry);
        fwd->registry_ref = false;
        fwd_unref(fwd);
    }
    fwd->registry = NULL;
}

QemuSlirpGuestFwdRegistry *
qemu_slirp_guestfwd_registry_new(bool ipv4_enabled, struct in_addr network,
                                 struct in_addr mask, struct in_addr vhost,
                                 struct in_addr dns,
                                 const QemuSlirpGuestFwdBackendOps *ops,
                                 void *backend_opaque)
{
    QemuSlirpGuestFwdRegistry *r;
    if (!ops || !ops->add || !ops->remove || !ops->can_send || !ops->send ||
        !ops->set_plan9_bootp)
        return NULL;
    r = g_new0(QemuSlirpGuestFwdRegistry, 1);
    r->network = network;
    r->mask = mask;
    r->vhost = vhost;
    r->dns = dns;
    r->backend = ops;
    r->backend_opaque = backend_opaque;
    r->valid = true;
    r->ipv4_enabled = ipv4_enabled;
    QTAILQ_INIT(&r->forwards);
    return r;
}

void qemu_slirp_guestfwd_registry_set_ipv4_enabled_for_test(
    QemuSlirpGuestFwdRegistry *r, bool enabled)
{
    r->ipv4_enabled = enabled;
}

void qemu_slirp_guestfwd_registry_invalidate(QemuSlirpGuestFwdRegistry *r)
{
    if (!r || !r->valid)
        return;
    r->valid = false;
    while (!QTAILQ_EMPTY(&r->forwards))
        detach(QTAILQ_FIRST(&r->forwards));
}

void qemu_slirp_guestfwd_registry_free(QemuSlirpGuestFwdRegistry *r)
{
    if (!r)
        return;
    qemu_slirp_guestfwd_registry_invalidate(r);
    g_free(r);
}

int qemu_slirp_guestfwd_registry_add(QemuSlirpGuestFwdRegistry *r,
                                     struct in_addr addr, uint16_t port,
                                     const QemuSlirpGuestFwdOps *ops,
                                     void *opaque, QemuSlirpGuestFwd **handle,
                                     Error **errp)
{
    uint32_t broadcast;
    QemuSlirpGuestFwd *fwd;
    if (handle)
        *handle = NULL;
    if (!r || !r->valid) {
        error_setg(errp, "Guest forward registry is invalid");
        return -1;
    }
    if (!handle) {
        error_setg(errp, "Guest forward handle output is NULL");
        return -1;
    }
    if (!r->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return -1;
    }
    if (!ops || !ops->write) {
        error_setg(errp, "Guest forward write callback is required");
        return -1;
    }
    if (!port) {
        error_setg(errp, "Guest forward port must not be zero");
        return -1;
    }
    broadcast = r->network.s_addr | ~r->mask.s_addr;
    if (!addr.s_addr || (addr.s_addr & r->mask.s_addr) != r->network.s_addr) {
        error_setg(errp, "Guest forward address is outside the network");
        return -1;
    }
    if (addr.s_addr == r->network.s_addr) {
        error_setg(errp, "Guest forward address is the network address");
        return -1;
    }
    if (addr.s_addr == broadcast) {
        error_setg(errp, "Guest forward address is the broadcast address");
        return -1;
    }
    if (addr.s_addr == r->vhost.s_addr) {
        error_setg(errp, "Guest forward address is the user-mode host address");
        return -1;
    }
    if (addr.s_addr == r->dns.s_addr) {
        error_setg(errp, "Guest forward address is the DNS address");
        return -1;
    }
    fwd = g_new0(QemuSlirpGuestFwd, 1);
    fwd->registry = r;
    fwd->addr = addr;
    fwd->port = port;
    fwd->ops = ops;
    fwd->opaque = opaque;
    fwd->refs = 2;
    fwd->registry_ref = true;
    fwd->caller_ref = true;
    fwd->valid = true;
    if (r->backend->add(r->backend_opaque, incoming, fwd, addr, port) < 0) {
        error_setg(errp, "Conflicting guest forward endpoint");
        fwd->valid = false;
        fwd->refs = 1;
        fwd_unref(fwd);
        return -1;
    }
    fwd->registered = true;
    QTAILQ_INSERT_TAIL(&r->forwards, fwd, entry);
    *handle = fwd;
    return 0;
}

size_t qemu_slirp_guestfwd_registry_can_send(QemuSlirpGuestFwd *fwd)
{
    return fwd && fwd->valid && fwd->registry
               ? fwd->registry->backend->can_send(fwd->registry->backend_opaque,
                                                  fwd->addr, fwd->port)
               : 0;
}

int qemu_slirp_guestfwd_registry_send(QemuSlirpGuestFwd *fwd,
                                      const uint8_t *buf, size_t len)
{
    if (!fwd || !fwd->valid || !fwd->registry)
        return -ENOTCONN;
    if (!buf || len > INT_MAX)
        return -EINVAL;
    if (len > qemu_slirp_guestfwd_registry_can_send(fwd))
        return -EAGAIN;
    return fwd->registry->backend->send(fwd->registry->backend_opaque,
                                        fwd->addr, fwd->port, buf, len);
}

bool qemu_slirp_guestfwd_registry_set_plan9_bootp(
    QemuSlirpGuestFwd *fwd, const QemuSlirpPlan9BootpConfig *config,
    Error **errp)
{
    QemuSlirpGuestFwdRegistry *r;
    if (!fwd || !fwd->valid || !(r = fwd->registry)) {
        error_setg(errp, "Guest forward handle is no longer connected");
        return false;
    }
    if (!r->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return false;
    }
    if (config && r->plan9_owner && r->plan9_owner != fwd) {
        error_setg(errp, "Another guest forward already owns Plan 9 BOOTP");
        return false;
    }
    if (!config && r->plan9_owner != fwd) {
        error_setg(errp, "Guest forward does not own Plan 9 BOOTP");
        return false;
    }
    if (!r->backend->set_plan9_bootp(r->backend_opaque, config)) {
        error_setg(errp, "Invalid Plan 9 BOOTP configuration");
        return false;
    }
    r->plan9_owner = config ? fwd : NULL;
    return true;
}

bool qemu_slirp_guestfwd_registry_get_ipv4_config(
    QemuSlirpGuestFwd *fwd, QemuSlirpIPv4Config *config, Error **errp)
{
    QemuSlirpIPv4Config snapshot;
    QemuSlirpGuestFwdRegistry *r;

    if (!fwd || !fwd->caller_ref || !fwd->valid || !fwd->registry) {
        error_setg(errp, "Guest forward handle is no longer connected");
        return false;
    }
    r = fwd->registry;
    if (!r->ipv4_enabled) {
        error_setg(errp, "IPv4 is disabled for this user-mode network stack");
        return false;
    }
    if (!config) {
        error_setg(errp, "IPv4 configuration output is NULL");
        return false;
    }
    snapshot.network = r->network;
    snapshot.netmask = r->mask;
    snapshot.host = r->vhost;
    snapshot.dns = r->dns;
    *config = snapshot;
    return true;
}

void qemu_slirp_guestfwd_registry_remove(QemuSlirpGuestFwd *fwd)
{
    if (!fwd || !fwd->caller_ref)
        return;
    fwd->caller_ref = false;
    if (fwd->callback_depth) {
        fwd->remove_pending = true;
        fwd_unref(fwd);
        return;
    }
    detach(fwd);
    fwd_unref(fwd);
}

void qemu_slirp_guestfwd_registry_flush_deferred(
    QemuSlirpGuestFwdRegistry *r)
{
    GPtrArray *a;
    QemuSlirpGuestFwd *fwd;
    guint i;

    if (!r || !r->valid) {
        return;
    }
    a = g_ptr_array_new();
    QTAILQ_FOREACH(fwd, &r->forwards, entry) {
        fwd_ref(fwd);
        g_ptr_array_add(a, fwd);
    }
    for (i = 0; i < a->len; i++) {
        fwd = g_ptr_array_index(a, i);
        if (fwd->remove_pending && !fwd->callback_depth) {
            fwd->remove_pending = false;
            detach(fwd);
        }
        fwd_unref(fwd);
    }
    g_ptr_array_free(a, true);
}

void qemu_slirp_guestfwd_registry_notify(QemuSlirpGuestFwdRegistry *r)
{
    GPtrArray *a;
    QemuSlirpGuestFwd *fwd;
    guint i;
    if (!r || !r->valid)
        return;
    a = g_ptr_array_new();
    QTAILQ_FOREACH(fwd, &r->forwards, entry) {
        fwd_ref(fwd);
        g_ptr_array_add(a, fwd);
    }
    for (i = 0; i < a->len; i++) {
        fwd = g_ptr_array_index(a, i);
        if (fwd->valid && fwd->ops->can_send &&
            qemu_slirp_guestfwd_registry_can_send(fwd))
            fwd->ops->can_send(fwd->opaque);
        fwd_unref(fwd);
    }
    g_ptr_array_free(a, true);
}
