/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_UDP_H
#define QEMU_NET_SLIRP_UDP_H

typedef struct Error Error;
typedef struct QemuSlirpUdpListener QemuSlirpUdpListener;

typedef enum QemuSlirpUdpListenFlags {
    QEMU_SLIRP_UDP_LISTEN_DEFAULT = 0,
    QEMU_SLIRP_UDP_LISTEN_BROADCAST = 1U << 0,
} QemuSlirpUdpListenFlags;

typedef struct QemuSlirpUdpListenerOps {
    void (*datagram)(QemuSlirpUdpListener *listener,
                     const struct sockaddr_in *peer,
                     const uint8_t *data, size_t len,
                     void *opaque);
} QemuSlirpUdpListenerOps;

/* The endpoint address is the named stack's IPv4 virtual-host address. */
int qemu_slirp_udp_listen_full(const char *netdev_id, uint16_t port,
                               QemuSlirpUdpListenFlags flags,
                               const QemuSlirpUdpListenerOps *ops, void *opaque,
                               QemuSlirpUdpListener **listener, Error **errp);
int qemu_slirp_udp_listen(const char *netdev_id, uint16_t port,
                          const QemuSlirpUdpListenerOps *ops, void *opaque,
                          QemuSlirpUdpListener **listener, Error **errp);
int qemu_slirp_udp_send(QemuSlirpUdpListener *listener,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len);

/* Consumes the caller-owned handle. Passing NULL is safe. */
void qemu_slirp_udp_listener_remove(QemuSlirpUdpListener *listener);

#endif /* QEMU_NET_SLIRP_UDP_H */
