/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-guestfwd-internal.h"
#include "qapi/error.h"

typedef struct FakeBackend {
    QemuSlirpGuestFwdIncoming incoming;
    void *incoming_opaque;
    struct in_addr addr;
    uint16_t port;
    size_t capacity;
    int adds, removes, sends, plan9_sets;
    bool plan9_valid;
} FakeBackend;

static int fake_add(void *opaque, QemuSlirpGuestFwdIncoming cb, void *cb_opaque,
                    struct in_addr addr, uint16_t port)
{
    FakeBackend *f = opaque;
    if (f->incoming && f->addr.s_addr == addr.s_addr && f->port == port)
        return -1;
    f->incoming = cb;
    f->incoming_opaque = cb_opaque;
    f->addr = addr;
    f->port = port;
    f->adds++;
    return 0;
}
static int fake_remove(void *opaque, struct in_addr addr, uint16_t port)
{
    FakeBackend *f = opaque;
    f->removes++;
    f->incoming = NULL;
    return 0;
}
static size_t fake_can_send(void *opaque, struct in_addr addr, uint16_t port)
{
    return ((FakeBackend *)opaque)->capacity;
}
static int fake_send(void *opaque, struct in_addr addr, uint16_t port,
                     const uint8_t *buf, size_t len)
{
    ((FakeBackend *)opaque)->sends++;
    return len;
}
static bool fake_plan9(void *opaque, const QemuSlirpPlan9BootpConfig *cfg)
{
    FakeBackend *f = opaque;
    f->plan9_sets++;
    if (cfg && !cfg->netmask.s_addr)
        return false;
    f->plan9_valid = cfg != NULL;
    return true;
}
static const QemuSlirpGuestFwdBackendOps backend_ops = {
    .add = fake_add,
    .remove = fake_remove,
    .can_send = fake_can_send,
    .send = fake_send,
    .set_plan9_bootp = fake_plan9,
};
static struct in_addr ip(uint32_t n)
{
    return (struct in_addr){htonl(n)};
}
static QemuSlirpGuestFwdRegistry *
new_registry_with_ipv4(FakeBackend *f, bool ipv4_enabled,
                       const QemuSlirpGuestFwdBackendOps *ops)
{
    return qemu_slirp_guestfwd_registry_new(
        ipv4_enabled, ip(0x0a000200), ip(0xffffff00), ip(0x0a000202),
        ip(0x0a000203), ops, f);
}
static QemuSlirpGuestFwdRegistry *
new_registry_with_ops(FakeBackend *f, const QemuSlirpGuestFwdBackendOps *ops)
{
    return new_registry_with_ipv4(f, true, ops);
}
static QemuSlirpGuestFwdRegistry *new_registry(FakeBackend *f)
{
    return new_registry_with_ops(f, &backend_ops);
}
static ssize_t write_full(const void *buf, size_t len, void *opaque)
{
    return len;
}
static const QemuSlirpGuestFwdOps full_ops = {.write = write_full};
static void remove_from_notify(void *opaque)
{
    QemuSlirpGuestFwd **handle = opaque;
    qemu_slirp_guestfwd_registry_remove(*handle);
}
typedef struct SelfRemoveWrite {
    FakeBackend *backend;
    QemuSlirpGuestFwd **handle;
    int removes_during;
    QemuSlirpIPv4Config config;
    Error *config_error;
    bool config_result;
} SelfRemoveWrite;
static ssize_t remove_from_write(const void *buf, size_t len, void *opaque)
{
    SelfRemoveWrite *self = opaque;
    QemuSlirpGuestFwd *handle = *self->handle;

    qemu_slirp_guestfwd_registry_remove(handle);
    self->config_result = qemu_slirp_guestfwd_registry_get_ipv4_config(
        handle, &self->config, &self->config_error);
    self->removes_during = self->backend->removes;
    *self->handle = NULL;
    return len;
}
static void assert_fail(QemuSlirpGuestFwdRegistry *r, struct in_addr a,
                        uint16_t p, const QemuSlirpGuestFwdOps *ops)
{
    QemuSlirpGuestFwd *h = (void *)0x1;
    Error *e = NULL;
    g_assert_cmpint(
        qemu_slirp_guestfwd_registry_add(r, a, p, ops, NULL, &h, &e), ==, -1);
    g_assert_null(h);
    g_assert_nonnull(e);
    error_free(e);
}
static void test_validation(void)
{
    FakeBackend f = {0};
    QemuSlirpGuestFwdRegistry *r = new_registry(&f);
    QemuSlirpGuestFwdBackendOps missing = backend_ops;
    Error *e = NULL;

    missing.add = NULL;
    g_assert_null(new_registry_with_ops(&f, &missing));
    missing = backend_ops;
    missing.remove = NULL;
    g_assert_null(new_registry_with_ops(&f, &missing));
    missing = backend_ops;
    missing.can_send = NULL;
    g_assert_null(new_registry_with_ops(&f, &missing));
    missing = backend_ops;
    missing.send = NULL;
    g_assert_null(new_registry_with_ops(&f, &missing));
    missing = backend_ops;
    missing.set_plan9_bootp = NULL;
    g_assert_null(new_registry_with_ops(&f, &missing));

    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 1,
                                                     &full_ops, NULL, NULL, &e),
                    ==, -1);
    g_assert_nonnull(e);
    error_free(e);
    assert_fail(r, ip(0x0a000200), 1, &full_ops);
    assert_fail(r, ip(0x0a0002ff), 1, &full_ops);
    assert_fail(r, ip(0x0a000202), 1, &full_ops);
    assert_fail(r, ip(0x0a000203), 1, &full_ops);
    assert_fail(r, ip(0x0a000204), 0, &full_ops);
    assert_fail(r, ip(0x0a000204), 1, NULL);
    assert_fail(r, ip(0x0b000204), 1, &full_ops);
    qemu_slirp_guestfwd_registry_free(r);
}
static void test_callbacks_and_remove(void)
{
    FakeBackend f = {0};
    QemuSlirpGuestFwdRegistry *r = new_registry(&f);
    QemuSlirpGuestFwd *h = NULL;
    Error *e = NULL;
    uint8_t b = 1;
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9,
                                                     &full_ops, NULL, &h, &e),
                    ==, 0);
    g_assert_cmpint(f.incoming(&b, 1, f.incoming_opaque), ==, 1);
    {
        QemuSlirpGuestFwd *collision = (void *)0x1;
        Error *collision_err = NULL;
        g_assert_cmpint(
            qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9, &full_ops,
                                             NULL, &collision, &collision_err),
            ==, -1);
        g_assert_null(collision);
        g_assert_nonnull(collision_err);
        g_assert_cmpint(f.adds, ==, 1);
        error_free(collision_err);
    }
    qemu_slirp_guestfwd_registry_remove(h);
    g_assert_cmpint(f.removes, ==, 1);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
static ssize_t short_write(const void *buf, size_t len, void *opaque)
{
    return len - 1;
}
static ssize_t error_write(const void *buf, size_t len, void *opaque)
{
    return -EIO;
}
static void test_send_and_results(void)
{
    FakeBackend f = {.capacity = 8};
    QemuSlirpGuestFwdRegistry *r = new_registry(&f);
    QemuSlirpGuestFwd *h;
    Error *e = NULL;
    uint8_t b[8] = {0};
    QemuSlirpGuestFwdOps ops = {.write = write_full};
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9, &ops,
                                                     NULL, &h, &e),
                    ==, 0);
    g_assert_cmpint(f.incoming(b, 8, f.incoming_opaque), ==, 8);
    qemu_slirp_guestfwd_registry_remove(h);
    ops.write = short_write;
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9, &ops,
                                                     NULL, &h, &e),
                    ==, 0);
    g_assert_cmpint(f.incoming(b, 2, f.incoming_opaque), ==, 1);
    qemu_slirp_guestfwd_registry_remove(h);
    ops.write = error_write;
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9, &ops,
                                                     NULL, &h, &e),
                    ==, 0);
    g_assert_cmpint(f.incoming(b, 1, f.incoming_opaque), ==, -EIO);
    g_assert_cmpint(qemu_slirp_guestfwd_registry_send(h, b, 8), ==, 8);
    g_assert_cmpint(qemu_slirp_guestfwd_registry_send(h, NULL, 1), ==, -EINVAL);
    g_assert_cmpint(
        qemu_slirp_guestfwd_registry_send(h, b, (size_t)INT_MAX + 1), ==,
        -EINVAL);
    g_assert_cmpint(qemu_slirp_guestfwd_registry_send(h, b, 9), ==, -EAGAIN);
    f.capacity = 0;
    g_assert_cmpint(qemu_slirp_guestfwd_registry_send(h, b, 1), ==, -EAGAIN);
    {
        int removes = f.removes;
        qemu_slirp_guestfwd_registry_invalidate(r);
        g_assert_cmpint(f.removes, ==, removes + 1);
    }
    g_assert_cmpuint(qemu_slirp_guestfwd_registry_can_send(h), ==, 0);
    g_assert_cmpint(qemu_slirp_guestfwd_registry_send(h, b, 1), ==, -ENOTCONN);
    qemu_slirp_guestfwd_registry_remove(h);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
static void test_plan9(void)
{
    FakeBackend f = {0};
    QemuSlirpGuestFwdRegistry *r = new_registry(&f);
    QemuSlirpGuestFwd *a, *b;
    Error *e = NULL;
    QemuSlirpPlan9BootpConfig cfg = {.netmask = ip(0xffffff00)};
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9,
                                                     &full_ops, NULL, &a, &e),
                    ==, 0);
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000205), 9,
                                                     &full_ops, NULL, &b, &e),
                    ==, 0);
    g_assert_true(qemu_slirp_guestfwd_registry_set_plan9_bootp(a, &cfg, &e));
    g_assert_false(qemu_slirp_guestfwd_registry_set_plan9_bootp(b, &cfg, &e));
    error_free(e);
    e = NULL;
    g_assert_true(qemu_slirp_guestfwd_registry_set_plan9_bootp(a, NULL, &e));
    cfg.netmask.s_addr = 0;
    g_assert_false(qemu_slirp_guestfwd_registry_set_plan9_bootp(b, &cfg, &e));
    error_free(e);
    e = NULL;
    cfg.netmask = ip(0xffffff00);
    g_assert_true(qemu_slirp_guestfwd_registry_set_plan9_bootp(b, &cfg, &e));
    qemu_slirp_guestfwd_registry_remove(a);
    {
        int plan9_sets = f.plan9_sets;
        qemu_slirp_guestfwd_registry_remove(b);
        g_assert_cmpint(f.plan9_sets, ==, plan9_sets + 1);
    }
    g_assert_false(f.plan9_valid);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
static void test_notify_snapshot(void)
{
    FakeBackend f = {.capacity = 1};
    QemuSlirpGuestFwdRegistry *r = new_registry(&f);
    QemuSlirpGuestFwd *self = NULL;
    Error *e = NULL;
    /* The fake deliberately exposes only guest-endpoint add/remove,
     * capacity/send, and BOOTP operations: there is no host-listener
     * operation. Pinned libslirp Task 1 guestfwd tests at 0454d23 cover raw
     * TCP teardown; the named-netdev QOM lifecycle is covered by Task 8. The
     * thin net/slirp.c facade is intentionally exercised by that QOM qtest. */
    QemuSlirpGuestFwdOps ops = {.write = write_full,
                                .can_send = remove_from_notify};
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(r, ip(0x0a000204), 9, &ops,
                                                     &self, &self, &e),
                    ==, 0);
    qemu_slirp_guestfwd_registry_notify(r);
    g_assert_cmpint(f.removes, ==, 1);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
static void test_deferred_remove_from_write(void)
{
    /* Write callbacks may consume their handle; backend removal is deferred
     * until the transport dispatch has returned. */
    FakeBackend f = {0};
    QemuSlirpGuestFwdRegistry *r = new_registry(&f);
    QemuSlirpGuestFwd *self = NULL;
    SelfRemoveWrite callback = {
        .backend = &f,
        .handle = &self,
    };
    QemuSlirpGuestFwdOps ops = {
        .write = remove_from_write,
    };
    Error *e = NULL;
    uint8_t byte = 1;

    memset(&callback.config, 0xff, sizeof(callback.config));

    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(
                        r, ip(0x0a000204), 9, &ops, &callback, &self, &e),
                    ==, 0);
    g_assert_cmpint(f.incoming(&byte, 1, f.incoming_opaque), ==, 1);
    g_assert_cmpint(callback.removes_during, ==, 0);
    g_assert_cmpint(f.removes, ==, 0);
    g_assert_null(self);
    g_assert_false(callback.config_result);
    g_assert_nonnull(callback.config_error);
    g_assert_cmpuint(callback.config.network.s_addr, ==, UINT32_MAX);
    g_assert_cmpuint(callback.config.netmask.s_addr, ==, UINT32_MAX);
    g_assert_cmpuint(callback.config.host.s_addr, ==, UINT32_MAX);
    g_assert_cmpuint(callback.config.dns.s_addr, ==, UINT32_MAX);

    qemu_slirp_guestfwd_registry_flush_deferred(r);
    g_assert_cmpint(f.removes, ==, 1);
    qemu_slirp_guestfwd_registry_remove(self);
    error_free(callback.config_error);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
static void test_ipv4_config(void)
{
    FakeBackend f = {0};
    QemuSlirpGuestFwdRegistry *r = qemu_slirp_guestfwd_registry_new(
        true, ip(0xac141000), ip(0xfffff000), ip(0xac141002), ip(0xac141003),
        &backend_ops, &f);
    QemuSlirpGuestFwd *h = NULL;
    QemuSlirpIPv4Config cfg = {0};
    QemuSlirpIPv4Config unchanged;
    Error *e = NULL;

    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(
                        r, ip(0xac141004), 564, &full_ops, NULL, &h, &e),
                    ==, 0);
    g_assert_true(qemu_slirp_guestfwd_registry_get_ipv4_config(h, &cfg, &e));
    g_assert_cmpuint(ntohl(cfg.network.s_addr), ==, 0xac141000);
    g_assert_cmpuint(ntohl(cfg.netmask.s_addr), ==, 0xfffff000);
    g_assert_cmpuint(ntohl(cfg.host.s_addr), ==, 0xac141002);
    g_assert_cmpuint(ntohl(cfg.dns.s_addr), ==, 0xac141003);

    g_assert_false(qemu_slirp_guestfwd_registry_get_ipv4_config(h, NULL, &e));
    g_assert_nonnull(e);
    error_free(e);
    e = NULL;

    memset(&cfg, 0xa5, sizeof(cfg));
    unchanged = cfg;
    g_assert_false(
        qemu_slirp_guestfwd_registry_get_ipv4_config(NULL, &cfg, &e));
    g_assert_nonnull(e);
    g_assert_cmpmem(&cfg, sizeof(cfg), &unchanged, sizeof(unchanged));
    error_free(e);
    e = NULL;

    qemu_slirp_guestfwd_registry_invalidate(r);
    g_assert_false(qemu_slirp_guestfwd_registry_get_ipv4_config(h, &cfg, &e));
    g_assert_nonnull(e);
    g_assert_cmpmem(&cfg, sizeof(cfg), &unchanged, sizeof(unchanged));

    qemu_slirp_guestfwd_registry_remove(h);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
static void test_ipv4_disabled(void)
{
    FakeBackend f = {0};
    QemuSlirpGuestFwdRegistry *r =
        new_registry_with_ipv4(&f, false, &backend_ops);
    QemuSlirpGuestFwd *h = (void *)0x1;
    QemuSlirpIPv4Config cfg;
    QemuSlirpIPv4Config unchanged;
    QemuSlirpPlan9BootpConfig bootp = {.netmask = ip(0xffffff00)};
    Error *e = NULL;

    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(
                        r, ip(0x0a000204), 564, &full_ops, NULL, &h, &e),
                    ==, -1);
    g_assert_null(h);
    g_assert_nonnull(e);
    g_assert_nonnull(strstr(error_get_pretty(e), "IPv4 is disabled"));
    g_assert_cmpint(f.adds, ==, 0);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);

    e = NULL;
    r = new_registry(&f);
    g_assert_cmpint(qemu_slirp_guestfwd_registry_add(
                        r, ip(0x0a000204), 564, &full_ops, NULL, &h, &e),
                    ==, 0);
    qemu_slirp_guestfwd_registry_set_ipv4_enabled_for_test(r, false);
    memset(&cfg, 0xa5, sizeof(cfg));
    unchanged = cfg;
    g_assert_false(qemu_slirp_guestfwd_registry_get_ipv4_config(h, &cfg, &e));
    g_assert_nonnull(e);
    g_assert_nonnull(strstr(error_get_pretty(e), "IPv4 is disabled"));
    g_assert_cmpmem(&cfg, sizeof(cfg), &unchanged, sizeof(unchanged));
    error_free(e);
    e = NULL;

    g_assert_false(qemu_slirp_guestfwd_registry_set_plan9_bootp(h, &bootp,
                                                                &e));
    g_assert_nonnull(e);
    g_assert_nonnull(strstr(error_get_pretty(e), "IPv4 is disabled"));
    g_assert_cmpint(f.plan9_sets, ==, 0);

    qemu_slirp_guestfwd_registry_remove(h);
    error_free(e);
    qemu_slirp_guestfwd_registry_free(r);
}
int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-guestfwd/validation", test_validation);
    g_test_add_func("/slirp-guestfwd/callbacks-remove",
                    test_callbacks_and_remove);
    g_test_add_func("/slirp-guestfwd/send-results", test_send_and_results);
    g_test_add_func("/slirp-guestfwd/plan9", test_plan9);
    g_test_add_func("/slirp-guestfwd/notify-snapshot", test_notify_snapshot);
    g_test_add_func("/slirp-guestfwd/deferred-write-remove",
                    test_deferred_remove_from_write);
    g_test_add_func("/slirp-guestfwd/ipv4-config", test_ipv4_config);
    g_test_add_func("/slirp-guestfwd/ipv4-disabled", test_ipv4_disabled);
    return g_test_run();
}
