/*
 * SLiRP adapter stubs for directly linked 9P1 protocol server tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "net/slirp-guestfwd.h"
#include "net/slirp-plan9.h"
#include "qapi/error.h"

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
    return false;
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
    error_setg(errp, "SLiRP is unavailable in this unit test");
    return false;
}

void qemu_slirp_plan9_bootp_release(QemuSlirpPlan9BootpLease **lease)
{
    if (lease) {
        *lease = NULL;
    }
}

void qemu_slirp_guestfwd_remove(QemuSlirpGuestFwd *handle)
{
}
