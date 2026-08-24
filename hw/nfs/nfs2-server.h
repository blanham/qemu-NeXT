/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_NFS_NFS2_SERVER_H
#define HW_NFS_NFS2_SERVER_H

#include <netinet/in.h>

#include "qapi/error.h"

typedef enum Nfs2Service {
    NFS2_SERVICE_PORTMAP,
    NFS2_SERVICE_MOUNT,
    NFS2_SERVICE_NFS,
} Nfs2Service;

typedef struct Nfs2TransportOps {
    int (*send)(Nfs2Service service, const struct sockaddr_in *peer,
                const uint8_t *data, size_t len, void *opaque);
    int64_t (*clock_ms)(void *opaque);
    void (*request_ref)(void *opaque);
    void (*request_unref)(void *opaque);
} Nfs2TransportOps;

typedef struct Nfs2Server Nfs2Server;

Nfs2Server *nfs2_server_new(const char *fsdev_id, bool writable,
                            const Nfs2TransportOps *transport,
                            void *transport_opaque, Error **errp);
int nfs2_server_receive(Nfs2Server *server, Nfs2Service service,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len, Error **errp);
bool nfs2_server_busy(const Nfs2Server *server);
void nfs2_server_begin_close(Nfs2Server *server);
void nfs2_server_reset(Nfs2Server *server);
void nfs2_server_free(Nfs2Server *server);

#endif
