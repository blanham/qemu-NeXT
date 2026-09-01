/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_SLIRP_RPC_INTERNAL_H
#define QEMU_NET_SLIRP_RPC_INTERNAL_H

#include "net/onc-rpc.h"
#include "net/slirp-stream-internal.h"
#include "net/slirp-udp-internal.h"

typedef struct QemuSlirpRpcRegistry QemuSlirpRpcRegistry;
typedef struct QemuSlirpRpcRegistration QemuSlirpRpcRegistration;

QemuSlirpRpcRegistry *qemu_slirp_rpc_registry_new(
    QemuSlirpUdpRegistry *udp_registry,
    QemuSlirpStreamRegistry *stream_registry);
int qemu_slirp_rpc_registry_register(
    QemuSlirpRpcRegistry *registry, const OncRpcProgram *program,
    QemuSlirpRpcRegistration **registration, Error **errp);
void qemu_slirp_rpc_registry_unregister(QemuSlirpRpcRegistration *registration);
void qemu_slirp_rpc_registry_invalidate(QemuSlirpRpcRegistry *registry);
void qemu_slirp_rpc_registry_free(QemuSlirpRpcRegistry *registry);

#endif /* QEMU_NET_SLIRP_RPC_INTERNAL_H */
