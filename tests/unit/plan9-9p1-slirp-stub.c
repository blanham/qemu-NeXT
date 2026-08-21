/*
 * SLiRP adapter stubs for directly linked 9P1 protocol server tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "net/slirp-guestfwd.h"
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

bool qemu_slirp_guestfwd_set_plan9_bootp(
    QemuSlirpGuestFwd *handle,
    const QemuSlirpPlan9BootpConfig *config,
    Error **errp)
{
    error_setg(errp, "SLiRP is unavailable in this unit test");
    return false;
}

bool qemu_slirp_guestfwd_get_ipv4_config(QemuSlirpGuestFwd *handle,
                                         QemuSlirpIPv4Config *config,
                                         Error **errp)
{
    error_setg(errp, "SLiRP is unavailable in this unit test");
    return false;
}

void qemu_slirp_guestfwd_remove(QemuSlirpGuestFwd *handle)
{
}
