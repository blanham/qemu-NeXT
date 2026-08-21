/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * QEMU 9p filesystem backend lifecycle
 *
 * Copyright IBM, Corp. 2010
 *
 * Authors:
 *  Anthony Liguori <aliguori@us.ibm.com>
 *  Wei Liu <wei.liu2@citrix.com>
 *  Greg Kurz <groug@kaod.org>
 *  Pradeep Jagadeesh <pradeep.jagadeesh@huawei.com>
 *  Christian Schoenebeck <qemu_oss@crudebyte.com>
 *  Antonios Motakis <antonios.motakis@huawei.com>
 */

#ifndef QEMU_9P_BACKEND_H
#define QEMU_9P_BACKEND_H

#include "fsdev/file-op-9p.h"
#include "qemu/qht.h"

typedef enum AffixType_t {
    AffixType_Prefix,
    AffixType_Suffix,
} AffixType_t;

typedef struct VariLenAffix {
    AffixType_t type;
    uint64_t value;
    int bits;
} VariLenAffix;

typedef struct QpdEntry {
    dev_t dev;
    int prefix_bits;
} QpdEntry;

typedef struct QppEntry {
    dev_t dev;
    uint16_t ino_prefix;
    uint32_t qp_affix_index;
    VariLenAffix qp_affix;
} QppEntry;

typedef struct QpfEntry {
    dev_t dev;
    ino_t ino;
    uint64_t path;
} QpfEntry;

/*
 * Transport-independent ownership for a configured fsdev export.  Protocol,
 * request, fid, and open state remain owned by the transport server.
 */
typedef struct V9fsBackend {
    FileOperations *ops;
    FsContext ctx;
    struct stat root_st;
    dev_t dev_id;

    struct qht qpd_table;
    struct qht qpp_table;
    struct qht qpf_table;
    uint64_t qp_ndevices;
    uint16_t qp_affix_next;
    uint64_t qp_fullpath_next;

    FsThrottle throttle;
    bool initialized;
    bool ops_cleanup_needed;
    bool throttle_initialized;
} V9fsBackend;

int v9fs_backend_init(V9fsBackend *backend, const char *fsdev_id, Error **errp);
void v9fs_backend_cleanup(V9fsBackend *backend);

#endif
