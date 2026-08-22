/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_BOOTP_H
#define QEMU_NET_SLIRP_BOOTP_H

typedef struct Error Error;
typedef struct QemuSlirpBootpRootLease QemuSlirpBootpRootLease;

bool qemu_slirp_bootp_root_claim(const char *netdev_id, const char *root_path,
                                 QemuSlirpBootpRootLease **lease,
                                 Error **errp);

/* Safe for NULL, an already-cleared pointer, or an invalidated netdev. */
void qemu_slirp_bootp_root_release(QemuSlirpBootpRootLease **lease);

#endif /* QEMU_NET_SLIRP_BOOTP_H */
