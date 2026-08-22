/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "net/net.h"
#include "net/slirp-il.h"
#include "net/slirp.h"
#include "qapi/error.h"

static NetClientState *test_netdev;

NetClientState *qemu_new_net_client(NetClientInfo *info,
                                    NetClientState *peer,
                                    const char *model, const char *name)
{
    NetClientState *nc = g_malloc0(info->size);

    nc->info = info;
    nc->model = g_strdup(model);
    nc->name = g_strdup(name);
    test_netdev = nc;
    return nc;
}

void qemu_set_info_str(NetClientState *nc, const char *fmt, ...)
{
}

NetClientState *qemu_find_netdev(const char *id)
{
    if (test_netdev && !strcmp(test_netdev->name, id)) {
        return test_netdev;
    }
    return NULL;
}

#include "../../net/slirp.c"

ssize_t qemu_send_packet(NetClientState *nc, const uint8_t *buf, int size)
{
    return size;
}

bool eth_pad_short_frame(uint8_t *padded_pkt, size_t *padded_buflen,
                         const void *pkt, size_t pkt_size)
{
    return false;
}

void unregister_savevm(VMStateIf *obj, const char *idstr, void *opaque)
{
}

void qemu_remove_exit_notifier(Notifier *notify)
{
}

size_t qemu_get_buffer(QEMUFile *f, uint8_t *buf, size_t size)
{
    return 0;
}

void qemu_put_buffer(QEMUFile *f, const uint8_t *buf, size_t size)
{
}

int qemu_file_get_error(QEMUFile *f)
{
    return 0;
}

void qemu_chr_fe_deinit(CharFrontend *c, bool del)
{
}

static struct in_addr test_addr(void)
{
    return (struct in_addr) { htonl(0x0a000204) };
}

static void *opened(QemuSlirpILConnection *connection, void *opaque)
{
    return opaque;
}

static void recorded(QemuSlirpILConnection *connection, const uint8_t *data,
                     size_t len, void *opaque)
{
}

static void closed(QemuSlirpILConnection *connection, void *opaque)
{
}

static const QemuSlirpILListenerOps listener_ops = {
    .open = opened,
    .record = recorded,
    .close = closed,
};

static SlirpState *new_user_netdev(void)
{
    const struct in_addr net = { .s_addr = htonl(0x0a000200) };
    const struct in_addr mask = { .s_addr = htonl(0xffffff00) };
    const struct in_addr host = { .s_addr = htonl(0x0a000202) };
    const struct in_addr dns = { .s_addr = htonl(0x0a000203) };
    SlirpConfig config = {
        .version = SLIRP_CHECK_VERSION(4, 9, 0) ? 6 : 4,
        .in_enabled = true,
        .vnetwork = net,
        .vnetmask = mask,
        .vhost = host,
    };
    SlirpState *s = g_new0(SlirpState, 1);

    s->nc.info = &net_slirp_info;
    s->nc.name = g_strdup("user0");
    s->vnetwork = net;
    s->vnetmask = mask;
    s->vhost = host;
    s->vnameserver = dns;
    s->slirp = slirp_new(&config, &slirp_cb, s);
    g_assert_nonnull(s->slirp);
    s->guestfwds = qemu_slirp_guestfwd_registry_new(
        true, net, mask, host, dns, &slirp_guestfwd_backend_ops, s);
#ifdef CONFIG_SLIRP_IL
    s->il_registry = qemu_slirp_il_registry_new(
        true, net, mask, host, dns, &slirp_il_backend_ops, s);
#endif
    s->poll_notifier.notify = net_slirp_poll_notify;
    QTAILQ_INSERT_TAIL(&slirp_stacks, s, entry);
    test_netdev = &s->nc;
    return s;
}

static void destroy_user_netdev(SlirpState *s)
{
    net_slirp_cleanup(&s->nc);
    g_free(s->nc.name);
    g_free(s);
    test_netdev = NULL;
}

static void test_named_netdev_facade(void)
{
    Error *err = NULL;
    QemuSlirpILListener *listener = NULL;

    g_assert_cmpint(qemu_slirp_il_listen("missing", test_addr(), 17008,
                                         NULL, NULL, &listener, &err), ==, -1);
#ifdef CONFIG_SLIRP_IL
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "Unrecognized netdev"));
#else
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "IL is unavailable"));
#endif
    g_assert_null(listener);
    error_free(err);
}

static void test_user_netdev_lifecycle_paths(void)
{
    SlirpState *s = new_user_netdev();
    MainLoopPoll poll = {
        .state = MAIN_LOOP_POLL_OK,
        .pollfds = g_array_new(false, false, sizeof(GPollFD)),
    };
    Error *err = NULL;
    QemuSlirpILListener *listener = NULL;
    uint8_t packet[ETH_HLEN] = {0};

    s->nc.info->receive(&s->nc, packet, sizeof(packet));
    s->poll_notifier.notify(&s->poll_notifier, &poll);
#ifdef CONFIG_SLIRP_IL
    NetClientInfo non_user = { .type = NET_CLIENT_DRIVER_NONE };
    NetClientInfo *user_info = s->nc.info;

    s->nc.info = &non_user;
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, &listener, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "not a user-mode"));
    error_free(err);
    err = NULL;
    s->nc.info = user_info;
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, &listener, &err),
                    ==, 0);
    g_assert_nonnull(listener);
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, NULL, &err),
                    ==, -1);
    g_assert_nonnull(err);
    error_free(err);
#else
    g_assert_cmpint(qemu_slirp_il_listen("user0", test_addr(), 17008,
                                         &listener_ops, NULL, &listener, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err), "IL is unavailable"));
    error_free(err);
#endif
    g_array_free(poll.pollfds, true);
    destroy_user_netdev(s);
#ifdef CONFIG_SLIRP_IL
    /* Cleanup removed the backend listener before slirp died. */
    qemu_slirp_il_listener_remove(listener);
#endif
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/slirp-il-integration/named-netdev-facade",
                    test_named_netdev_facade);
    g_test_add_func("/slirp-il-integration/user-netdev-lifecycle",
                    test_user_netdev_lifecycle_paths);
    return g_test_run();
}
