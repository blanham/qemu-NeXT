/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_PLAN9_H
#define QEMU_NET_SLIRP_PLAN9_H

typedef struct Error Error;
typedef struct QemuSlirpPlan9BootpLease QemuSlirpPlan9BootpLease;

/* QEMU-owned equivalent of libslirp's Plan 9 BOOTP configuration. */
typedef struct QemuSlirpPlan9BootpConfig {
    struct in_addr netmask;
    struct in_addr file_server;
    struct in_addr auth_server;
    struct in_addr gateway;
} QemuSlirpPlan9BootpConfig;

/* Main-loop-only facade for one named user-mode network stack. */
bool qemu_slirp_plan9_bootp_available(const char *netdev_id, Error **errp);
bool qemu_slirp_plan9_bootp_claim(const char *netdev_id,
                                  struct in_addr file_server,
                                  struct in_addr auth_server,
                                  QemuSlirpPlan9BootpLease **lease,
                                  Error **errp);

/*
 * Consumes and clears *lease. Passing NULL or an already-cleared pointer is
 * safe, including after the owning netdev has been destroyed.
 */
void qemu_slirp_plan9_bootp_release(QemuSlirpPlan9BootpLease **lease);

#endif /* QEMU_NET_SLIRP_PLAN9_H */
