/*
 * SLiRP adapter stubs for directly linked 9P1 protocol server tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "net/slirp-guestfwd.h"
#include "net/slirp-il.h"
#include "net/slirp-plan9.h"
#include "qapi/error.h"
#include "plan9-9p1-slirp-stub.h"

struct QemuSlirpILListener {
    struct in_addr address;
    uint16_t port;
    QemuSlirpILListenerOps ops;
    void *opaque;
    GPtrArray *connections;
};

struct QemuSlirpILConnection {
    QemuSlirpILListener *listener;
    void *opaque;
    GPtrArray *sent_records;
    unsigned int close_requests;
    bool close_requested;
    bool closed;
};

struct QemuSlirpPlan9BootpLease {
    bool claimed;
};

struct Plan9P1SlirpStubConnection {
    QemuSlirpILConnection il;
};

static bool fake_enabled;
static GPtrArray *fake_listeners;
static struct in_addr fake_bootp_file;
static struct in_addr fake_bootp_auth;
static bool fake_deferred_close;
static bool fake_duplicate_close;

static void plan9p1_slirp_stub_deliver_close_internal(
    QemuSlirpILConnection *connection)
{
    if (!connection || connection->closed) {
        return;
    }
    connection->closed = true;
    connection->listener->ops.close(connection, connection->opaque);
}

void plan9p1_slirp_stub_enable(void)
{
    g_assert_null(fake_listeners);
    fake_enabled = true;
    fake_listeners = g_ptr_array_new();
    fake_bootp_file.s_addr = 0;
    fake_bootp_auth.s_addr = 0;
    fake_deferred_close = false;
    fake_duplicate_close = false;
}

void plan9p1_slirp_stub_disable(void)
{
    g_assert_nonnull(fake_listeners);
    g_assert_cmpuint(fake_listeners->len, ==, 0);
    g_ptr_array_unref(fake_listeners);
    fake_listeners = NULL;
    fake_enabled = false;
}

void plan9p1_slirp_stub_set_deferred_close(bool enabled)
{
    fake_deferred_close = enabled;
}

bool plan9p1_slirp_stub_had_duplicate_close(void)
{
    return fake_duplicate_close;
}

unsigned int plan9p1_slirp_stub_listener_count(void)
{
    return fake_listeners ? fake_listeners->len : 0;
}

uint16_t plan9p1_slirp_stub_listener_port(unsigned int index)
{
    QemuSlirpILListener *listener = g_ptr_array_index(fake_listeners, index);

    return listener->port;
}

uint32_t plan9p1_slirp_stub_listener_address(unsigned int index)
{
    QemuSlirpILListener *listener = g_ptr_array_index(fake_listeners, index);

    return listener->address.s_addr;
}

uint32_t plan9p1_slirp_stub_bootp_file_address(void)
{
    return fake_bootp_file.s_addr;
}

uint32_t plan9p1_slirp_stub_bootp_auth_address(void)
{
    return fake_bootp_auth.s_addr;
}

Plan9P1SlirpStubConnection *plan9p1_slirp_stub_open(uint16_t port)
{
    Plan9P1SlirpStubConnection *stub;
    QemuSlirpILListener *listener = NULL;

    for (unsigned int i = 0; i < fake_listeners->len; i++) {
        QemuSlirpILListener *candidate = g_ptr_array_index(fake_listeners, i);

        if (candidate->port == port) {
            listener = candidate;
            break;
        }
    }
    g_assert_nonnull(listener);
    stub = g_new0(Plan9P1SlirpStubConnection, 1);
    stub->il.listener = listener;
    stub->il.sent_records = g_ptr_array_new_with_free_func(
        (GDestroyNotify)g_bytes_unref);
    g_ptr_array_add(listener->connections, stub);
    stub->il.opaque = listener->ops.open(&stub->il, listener->opaque);
    return stub;
}

bool plan9p1_slirp_stub_connection_accepted(
    const Plan9P1SlirpStubConnection *connection)
{
    return connection->il.opaque != NULL && !connection->il.closed;
}

bool plan9p1_slirp_stub_connection_closed(
    const Plan9P1SlirpStubConnection *connection)
{
    return connection->il.closed;
}

unsigned int plan9p1_slirp_stub_connection_close_requests(
    const Plan9P1SlirpStubConnection *connection)
{
    return connection->il.close_requests;
}

void plan9p1_slirp_stub_close(Plan9P1SlirpStubConnection *connection)
{
    qemu_slirp_il_connection_close(&connection->il);
}

void plan9p1_slirp_stub_deliver_close(Plan9P1SlirpStubConnection *connection)
{
    plan9p1_slirp_stub_deliver_close_internal(&connection->il);
}

void plan9p1_slirp_stub_deliver_record(Plan9P1SlirpStubConnection *connection,
                                      const uint8_t *data, size_t len)
{
    if (!connection->il.closed) {
        connection->il.listener->ops.record(&connection->il, data, len,
                                             connection->il.opaque);
    }
}

GBytes *plan9p1_slirp_stub_pop_sent_record(
    Plan9P1SlirpStubConnection *connection)
{
    GBytes *record;

    if (!connection->il.sent_records->len) {
        return NULL;
    }
    record = g_ptr_array_steal_index(connection->il.sent_records, 0);
    return record;
}

bool qemu_slirp_il_available(const char *netdev_id, Error **errp)
{
    if (fake_enabled) {
        return true;
    }
    error_setg(errp, "SLiRP IL is unavailable in this unit test");
    return false;
}

int qemu_slirp_il_listen(const char *netdev_id, struct in_addr guest_addr,
                         uint16_t guest_port,
                         const QemuSlirpILListenerOps *ops, void *opaque,
                         QemuSlirpILListener **listener, Error **errp)
{
    QemuSlirpILListener *created;

    if (listener) {
        *listener = NULL;
    }
    if (fake_enabled) {
        for (unsigned int i = 0; i < fake_listeners->len; i++) {
            QemuSlirpILListener *existing =
                g_ptr_array_index(fake_listeners, i);

            if (existing->port == guest_port) {
                error_setg(errp, "injected IL listener collision");
                return -1;
            }
        }
        created = g_new0(QemuSlirpILListener, 1);
        created->address = guest_addr;
        created->port = guest_port;
        created->ops = *ops;
        created->opaque = opaque;
        created->connections = g_ptr_array_new();
        g_ptr_array_add(fake_listeners, created);
        *listener = created;
        return 0;
    }
    error_setg(errp, "SLiRP IL is unavailable in this unit test");
    return -1;
}

int qemu_slirp_il_send_record(QemuSlirpILConnection *connection,
                              const uint8_t *data, size_t len)
{
    if (!connection || connection->closed || connection->close_requested) {
        return -ENOTCONN;
    }
    g_ptr_array_add(connection->sent_records, g_bytes_new(data, len));
    return 0;
}

void qemu_slirp_il_connection_close(QemuSlirpILConnection *connection)
{
    if (!connection || connection->closed) {
        return;
    }
    if (connection->close_requested) {
        fake_duplicate_close = true;
        if (fake_deferred_close) {
            /* Break a buggy close-retry loop so its test fails, not hangs. */
            plan9p1_slirp_stub_deliver_close_internal(connection);
        }
        return;
    }
    connection->close_requested = true;
    connection->close_requests++;
    if (!fake_deferred_close) {
        plan9p1_slirp_stub_deliver_close_internal(connection);
    }
}

void qemu_slirp_il_listener_remove(QemuSlirpILListener *listener)
{
    if (!listener) {
        return;
    }
    g_ptr_array_remove(fake_listeners, listener);
    while (listener->connections->len) {
        Plan9P1SlirpStubConnection *connection =
            g_ptr_array_index(listener->connections, 0);

        g_ptr_array_remove_index(listener->connections, 0);
        plan9p1_slirp_stub_deliver_close_internal(&connection->il);
        g_ptr_array_unref(connection->il.sent_records);
        g_free(connection);
    }
    g_ptr_array_unref(listener->connections);
    g_free(listener);
}

int qemu_slirp_guestfwd_add(const char *netdev_id,
                            struct in_addr guest_addr,
                            uint16_t guest_port,
                            const QemuSlirpGuestFwdOps *ops,
                            void *opaque,
                            QemuSlirpGuestFwd **handle,
                            Error **errp)
{
    if (handle) {
        *handle = NULL;
    }
    error_setg(errp, "SLiRP is unavailable in this unit test");
    return -1;
}

size_t qemu_slirp_guestfwd_can_send(QemuSlirpGuestFwd *handle)
{
    return 0;
}

int qemu_slirp_guestfwd_send(QemuSlirpGuestFwd *handle,
                             const uint8_t *buf, size_t len)
{
    return -ENOTCONN;
}

bool qemu_slirp_plan9_bootp_available(const char *netdev_id, Error **errp)
{
    return fake_enabled;
}

bool qemu_slirp_plan9_bootp_claim(const char *netdev_id,
                                  struct in_addr file_server,
                                  struct in_addr auth_server,
                                  QemuSlirpPlan9BootpLease **lease,
                                  Error **errp)
{
    if (lease) {
        *lease = NULL;
    }
    if (fake_enabled) {
        fake_bootp_file = file_server;
        fake_bootp_auth = auth_server;
        *lease = g_new0(QemuSlirpPlan9BootpLease, 1);
        (*lease)->claimed = true;
        return true;
    }
    error_setg(errp, "SLiRP is unavailable in this unit test");
    return false;
}

void qemu_slirp_plan9_bootp_release(QemuSlirpPlan9BootpLease **lease)
{
    if (lease) {
        g_free(*lease);
        *lease = NULL;
    }
}

void qemu_slirp_guestfwd_remove(QemuSlirpGuestFwd *handle)
{
}
