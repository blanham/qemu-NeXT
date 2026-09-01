/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_STREAM_H
#define QEMU_NET_SLIRP_STREAM_H

typedef struct Error Error;
typedef struct QemuSlirpStreamListener QemuSlirpStreamListener;
typedef struct QemuSlirpStream QemuSlirpStream;

typedef struct QemuSlirpStreamOps {
    void (*connected)(QemuSlirpStream *stream,
                      const struct sockaddr_in *peer, void *opaque);
    void (*receive)(QemuSlirpStream *stream, const uint8_t *data,
                    size_t len, void *opaque);
    void (*can_send)(QemuSlirpStream *stream, void *opaque);
    void (*closed)(QemuSlirpStream *stream, void *opaque);
} QemuSlirpStreamOps;

/* The endpoint is the named stack's IPv4 virtual-host address. */
int qemu_slirp_stream_listen(const char *netdev_id, uint16_t port,
                             const QemuSlirpStreamOps *ops, void *opaque,
                             QemuSlirpStreamListener **listener,
                             Error **errp);

/* Return the number of bytes that can be accepted without backpressure. */
size_t qemu_slirp_stream_can_send(QemuSlirpStream *stream);

/*
 * Queue one complete byte stream chunk atomically.  Return zero when the
 * complete chunk is accepted, -EAGAIN when output is full, -EINVAL for an
 * invalid buffer/length, or -ENOTCONN after the terminal close callback.
 */
int qemu_slirp_stream_send(QemuSlirpStream *stream, const uint8_t *data,
                           size_t len);

/*
 * Idempotent while valid; may be called from any stream callback.  The stream
 * handle becomes invalid after the closed callback returns.
 */
void qemu_slirp_stream_close(QemuSlirpStream *stream);

/* Consumes the caller-owned listener handle. Passing NULL is safe. */
void qemu_slirp_stream_listener_remove(QemuSlirpStreamListener *listener);

#endif /* QEMU_NET_SLIRP_STREAM_H */
