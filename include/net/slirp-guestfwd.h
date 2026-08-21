/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2011-2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the names of the University of Illinois/NCSA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QEMU_NET_SLIRP_GUESTFWD_H
#define QEMU_NET_SLIRP_GUESTFWD_H

typedef struct Error Error;
typedef struct QemuSlirpGuestFwd QemuSlirpGuestFwd;

typedef struct QemuSlirpGuestFwdOps {
    ssize_t (*write)(const void *buf, size_t len, void *opaque);
    void (*can_send)(void *opaque);
} QemuSlirpGuestFwdOps;

/* QEMU-owned equivalent of libslirp's Plan 9 BOOTP configuration. */
typedef struct QemuSlirpPlan9BootpConfig {
    struct in_addr netmask;
    struct in_addr file_server;
    struct in_addr auth_server;
    struct in_addr gateway;
} QemuSlirpPlan9BootpConfig;

/* IPv4 addresses are stored in network byte order, as required by in_addr. */
typedef struct QemuSlirpIPv4Config {
    struct in_addr network;
    struct in_addr netmask;
    struct in_addr host;
    struct in_addr dns;
} QemuSlirpIPv4Config;

int qemu_slirp_guestfwd_add(const char *netdev_id,
                            struct in_addr guest_addr,
                            uint16_t guest_port,
                            const QemuSlirpGuestFwdOps *ops,
                            void *opaque,
                            QemuSlirpGuestFwd **handle,
                            Error **errp);
size_t qemu_slirp_guestfwd_can_send(QemuSlirpGuestFwd *handle);
int qemu_slirp_guestfwd_send(QemuSlirpGuestFwd *handle,
                             const uint8_t *buf, size_t len);
bool qemu_slirp_guestfwd_set_plan9_bootp(
    QemuSlirpGuestFwd *handle,
    const QemuSlirpPlan9BootpConfig *config,
    Error **errp);
/* Main-loop-only snapshot. On failure, config is left unchanged. */
bool qemu_slirp_guestfwd_get_ipv4_config(QemuSlirpGuestFwd *handle,
                                         QemuSlirpIPv4Config *config,
                                         Error **errp);

/* Consumes the caller-owned handle. Set the caller's pointer to NULL and do
 * not reuse it after this call. Passing NULL is safe. Removal from write
 * callbacks is supported and deferred until transport dispatch returns. */
void qemu_slirp_guestfwd_remove(QemuSlirpGuestFwd *handle);

#endif /* QEMU_NET_SLIRP_GUESTFWD_H */
