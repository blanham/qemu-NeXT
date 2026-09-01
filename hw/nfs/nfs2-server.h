/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_NFS_NFS2_SERVER_H
#define HW_NFS_NFS2_SERVER_H

#include "net/onc-rpc.h"
#include "qapi/error.h"

typedef struct Nfs2TransportOps {
    int64_t (*clock_ms)(void *opaque);
    void (*request_ref)(void *opaque);
    void (*request_unref)(void *opaque);
} Nfs2TransportOps;

typedef struct Nfs2Server Nfs2Server;

Nfs2Server *nfs2_server_new(const char *fsdev_id, const char *netdev_id,
                            bool writable,
                            const Nfs2TransportOps *transport,
                            void *transport_opaque, Error **errp);
int nfs2_server_receive(Nfs2Server *server, OncRpcRequest *request,
                        Error **errp);
bool nfs2_server_busy(const Nfs2Server *server);
void nfs2_server_begin_close(Nfs2Server *server);
void nfs2_server_reset(Nfs2Server *server);
void nfs2_server_free(Nfs2Server *server);

#endif
