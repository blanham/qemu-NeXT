/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_IL_H
#define QEMU_NET_SLIRP_IL_H

typedef struct Error Error;
typedef struct QemuSlirpILListener QemuSlirpILListener;
typedef struct QemuSlirpILConnection QemuSlirpILConnection;

/*
 * All callbacks run in the user-network main-loop context.  An open callback
 * returns the connection-private value passed to later callbacks.  An open
 * callback publishes a connection handle that remains valid through its
 * terminal close callback; callers may store it and send or close while it
 * remains valid.  The close callback is the terminal ownership notification:
 * it must clear every stored reference before returning, and no operation may
 * use the handle afterwards.  A handle may be closed from any callback.
 * Closing a listener consumes its caller-owned handle, prevents future opens,
 * and closes its connections.
 */
typedef struct QemuSlirpILListenerOps {
    void *(*open)(QemuSlirpILConnection *connection, void *opaque);
    void (*record)(QemuSlirpILConnection *connection, const uint8_t *data,
                   size_t len, void *connection_opaque);
    void (*can_send)(QemuSlirpILConnection *connection,
                     void *connection_opaque);
    void (*close)(QemuSlirpILConnection *connection,
                  void *connection_opaque);
} QemuSlirpILListenerOps;

int qemu_slirp_il_listen(const char *netdev_id, struct in_addr guest_addr,
                         uint16_t guest_port,
                         const QemuSlirpILListenerOps *ops, void *opaque,
                         QemuSlirpILListener **listener, Error **errp);

/*
 * Atomic: returns zero only when the complete record was accepted.  It
 * returns -ENOTCONN during the terminal close callback or if the backend
 * closes the connection while accepting the record.
 */
int qemu_slirp_il_send_record(QemuSlirpILConnection *connection,
                              const uint8_t *data, size_t len);

/* Idempotent while valid, including within a close callback. */
void qemu_slirp_il_connection_close(QemuSlirpILConnection *connection);

/* Consumes the caller-owned listener handle.  Passing NULL is safe. */
void qemu_slirp_il_listener_remove(QemuSlirpILListener *listener);

#endif /* QEMU_NET_SLIRP_IL_H */
