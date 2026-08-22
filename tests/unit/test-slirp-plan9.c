/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/slirp-plan9-internal.h"
#include "qapi/error.h"

typedef struct FakeBackend {
    QemuSlirpPlan9BootpConfig config;
    unsigned sets;
    bool valid;
    bool accept;
    QemuSlirpPlan9BootpLease **release_slot;
} FakeBackend;

static struct in_addr ip(uint32_t host_order)
{
    return (struct in_addr) { .s_addr = htonl(host_order) };
}

static bool fake_set_bootp(void *opaque,
                           const QemuSlirpPlan9BootpConfig *config)
{
    FakeBackend *fake = opaque;

    fake->sets++;
    fake->valid = config != NULL;
    if (config) {
        fake->config = *config;
    }
    if (!config && fake->release_slot) {
        qemu_slirp_plan9_bootp_release(fake->release_slot);
    }
    return fake->accept;
}

static const QemuSlirpPlan9BackendOps backend_ops = {
    .set_bootp = fake_set_bootp,
};

static void test_claim_custom_network(void)
{
    FakeBackend fake = { .accept = true };
    QemuSlirpPlan9Registry *registry = qemu_slirp_plan9_registry_new(
        true, ip(0xac141000), ip(0xfffff000), ip(0xac141002),
        ip(0xac141003), &backend_ops, &fake);
    QemuSlirpPlan9BootpLease *lease = NULL;
    Error *err = NULL;

    g_assert_nonnull(registry);
    g_assert_true(qemu_slirp_plan9_registry_claim(
        registry, ip(0xac141064), ip(0xac141065), &lease, &err));
    g_assert_null(err);
    g_assert_nonnull(lease);
    g_assert_true(fake.valid);
    g_assert_cmpuint(ntohl(fake.config.netmask.s_addr), ==, 0xfffff000);
    g_assert_cmpuint(ntohl(fake.config.file_server.s_addr), ==, 0xac141064);
    g_assert_cmpuint(ntohl(fake.config.auth_server.s_addr), ==, 0xac141065);
    g_assert_cmpuint(ntohl(fake.config.gateway.s_addr), ==, 0xac141002);

    qemu_slirp_plan9_bootp_release(&lease);
    g_assert_null(lease);
    g_assert_false(fake.valid);
    qemu_slirp_plan9_registry_free(registry);
}

static QemuSlirpPlan9Registry *new_registry(FakeBackend *fake)
{
    return qemu_slirp_plan9_registry_new(
        true, ip(0x0a000200), ip(0xffffff00), ip(0x0a000202),
        ip(0x0a000203), &backend_ops, fake);
}

static void test_collision_release_reclaim(void)
{
    FakeBackend fake = { .accept = true };
    QemuSlirpPlan9Registry *registry = new_registry(&fake);
    QemuSlirpPlan9BootpLease *first = NULL;
    QemuSlirpPlan9BootpLease *second = (void *)0x1;
    Error *err = NULL;

    g_assert_true(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000264), ip(0), &first, &err));
    g_assert_false(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000265), ip(0), &second, &err));
    g_assert_null(second);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "already owns"));
    error_free(err);
    err = NULL;

    qemu_slirp_plan9_bootp_release(&first);
    g_assert_true(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000265), ip(0), &second, &err));
    qemu_slirp_plan9_bootp_release(&second);
    qemu_slirp_plan9_bootp_release(&second);
    qemu_slirp_plan9_registry_free(registry);
}

static void assert_invalid(QemuSlirpPlan9Registry *registry,
                           uint32_t file, uint32_t auth)
{
    QemuSlirpPlan9BootpLease *lease = (void *)0x1;
    Error *err = NULL;

    g_assert_false(qemu_slirp_plan9_registry_claim(
        registry, ip(file), ip(auth), &lease, &err));
    g_assert_null(lease);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_address_validation(void)
{
    FakeBackend fake = { .accept = true };
    QemuSlirpPlan9Registry *registry = new_registry(&fake);

    assert_invalid(registry, 0, 0);
    assert_invalid(registry, 0xc0000201, 0);
    assert_invalid(registry, 0x0a000200, 0);
    assert_invalid(registry, 0x0a0002ff, 0);
    assert_invalid(registry, 0x0a000202, 0);
    assert_invalid(registry, 0x0a000203, 0);
    assert_invalid(registry, 0x0a000264, 0xc0000201);
    assert_invalid(registry, 0x0a000264, 0x0a000200);
    assert_invalid(registry, 0x0a000264, 0x0a0002ff);
    assert_invalid(registry, 0x0a000264, 0x0a000202);
    assert_invalid(registry, 0x0a000264, 0x0a000203);
    g_assert_cmpuint(fake.sets, ==, 0);
    qemu_slirp_plan9_registry_free(registry);
}

static void test_invalidate_then_release(void)
{
    FakeBackend fake = { .accept = true };
    QemuSlirpPlan9Registry *registry = new_registry(&fake);
    QemuSlirpPlan9BootpLease *lease = NULL;
    Error *err = NULL;

    g_assert_true(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000264), ip(0), &lease, &err));
    qemu_slirp_plan9_registry_invalidate(registry);
    g_assert_false(fake.valid);
    g_assert_cmpuint(fake.sets, ==, 2);
    qemu_slirp_plan9_registry_free(registry);
    qemu_slirp_plan9_bootp_release(&lease);
    g_assert_null(lease);
    g_assert_cmpuint(fake.sets, ==, 2);
}

static void test_unavailable_and_backend_rejection(void)
{
    FakeBackend fake = { .accept = false };
    QemuSlirpPlan9Registry *registry = new_registry(&fake);
    QemuSlirpPlan9BootpLease *lease = NULL;
    Error *err = NULL;

    g_assert_true(qemu_slirp_plan9_registry_available(registry));
    g_assert_false(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000264), ip(0), &lease, &err));
    g_assert_null(lease);
    g_assert_nonnull(err);
    error_free(err);
    qemu_slirp_plan9_registry_free(registry);

    registry = qemu_slirp_plan9_registry_new(
        true, ip(0x0a000200), ip(0xffffff00), ip(0x0a000202),
        ip(0x0a000203), NULL, NULL);
    g_assert_false(qemu_slirp_plan9_registry_available(registry));
    qemu_slirp_plan9_registry_free(registry);
}

static void test_ipv4_disabled(void)
{
    FakeBackend fake = { .accept = true };
    QemuSlirpPlan9Registry *registry = qemu_slirp_plan9_registry_new(
        false, ip(0), ip(0), ip(0), ip(0), &backend_ops, &fake);
    QemuSlirpPlan9BootpLease *lease = (void *)0x1;
    Error *err = NULL;

    g_assert_false(qemu_slirp_plan9_registry_available(registry));
    g_assert_false(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000264), ip(0), &lease, &err));
    g_assert_null(lease);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "IPv4 is disabled"));
    g_assert_cmpuint(fake.sets, ==, 0);
    error_free(err);
    qemu_slirp_plan9_registry_free(registry);
}

static void test_release_callback_safe(void)
{
    FakeBackend fake = { .accept = true };
    QemuSlirpPlan9Registry *registry = new_registry(&fake);
    QemuSlirpPlan9BootpLease *lease = NULL;
    Error *err = NULL;

    g_assert_true(qemu_slirp_plan9_registry_claim(
        registry, ip(0x0a000264), ip(0), &lease, &err));
    fake.release_slot = &lease;
    qemu_slirp_plan9_bootp_release(&lease);
    g_assert_null(lease);
    g_assert_cmpuint(fake.sets, ==, 2);
    qemu_slirp_plan9_registry_free(registry);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-plan9/claim-custom-network",
                    test_claim_custom_network);
    g_test_add_func("/slirp-plan9/collision-release-reclaim",
                    test_collision_release_reclaim);
    g_test_add_func("/slirp-plan9/address-validation",
                    test_address_validation);
    g_test_add_func("/slirp-plan9/invalidate-release",
                    test_invalidate_then_release);
    g_test_add_func("/slirp-plan9/unavailable-backend-rejection",
                    test_unavailable_and_backend_rejection);
    g_test_add_func("/slirp-plan9/ipv4-disabled", test_ipv4_disabled);
    g_test_add_func("/slirp-plan9/release-callback-safe",
                    test_release_callback_safe);
    return g_test_run();
}
