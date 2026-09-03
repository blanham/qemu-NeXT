/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_NETINFO_NETINFO_SERVER_H
#define HW_NETINFO_NETINFO_SERVER_H

#include <stdint.h>

#include "hw/netinfo/netinfo-db.h"
#include "net/onc-rpc.h"
#include "qapi/error.h"

typedef struct NetInfoServer NetInfoServer;

/*
 * Construct the read-only binder and database services for one user-mode
 * network.  Ownership of @db is transferred to the server on success and on
 * failure.  @local_addr is the NetInfo wire-format IPv4 address accepted by
 * NI_BIND; the default constructor uses the built-in 10.0.2.2 address.
 */
NetInfoServer *netinfo_server_new(NetInfoDb *db, const char *netdev_id,
                                  Error **errp);
NetInfoServer *netinfo_server_new_with_local_addr(NetInfoDb *db,
                                                  const char *netdev_id,
                                                  uint32_t local_addr,
                                                  Error **errp);
void netinfo_server_free(NetInfoServer *server);

#endif /* HW_NETINFO_NETINFO_SERVER_H */
