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

#include "qemu/osdep.h"

#include "9p-backend.h"
#include "fsdev/qemu-fsdev.h"
#include "qapi/error.h"

static bool qpd_cmp_func(const void *obj, const void *userp)
{
    const QpdEntry *e1 = obj;
    const QpdEntry *e2 = userp;

    return e1->dev == e2->dev;
}

static bool qpp_cmp_func(const void *obj, const void *userp)
{
    const QppEntry *e1 = obj;
    const QppEntry *e2 = userp;

    return e1->dev == e2->dev && e1->ino_prefix == e2->ino_prefix;
}

static bool qpf_cmp_func(const void *obj, const void *userp)
{
    const QpfEntry *e1 = obj;
    const QpfEntry *e2 = userp;

    return e1->dev == e2->dev && e1->ino == e2->ino;
}

static void qp_table_remove(void *p, uint32_t h, void *up)
{
    g_free(p);
}

static void qp_table_destroy(struct qht *ht)
{
    if (!ht->map) {
        return;
    }
    qht_iter(ht, qp_table_remove, NULL);
    qht_destroy(ht);
}

static void backend_device_maps_init(V9fsBackend *backend)
{
    qht_init(&backend->qpd_table, qpd_cmp_func, 1, QHT_MODE_AUTO_RESIZE);
    qht_init(&backend->qpf_table, qpf_cmp_func, 1 << 16, QHT_MODE_AUTO_RESIZE);
    qht_init(&backend->qpp_table, qpp_cmp_func, 1, QHT_MODE_AUTO_RESIZE);
    backend->qp_ndevices = 0;
    backend->qp_affix_next = 1;
    backend->qp_fullpath_next = 1;
}

int v9fs_backend_init(V9fsBackend *backend, const char *fsdev_id, Error **errp)
{
    ERRP_GUARD();
    FsDriverEntry *fse;
    V9fsPath path = { 0 };
    int ret = -1;

    if (backend->initialized || backend->ops_cleanup_needed ||
        backend->ctx.fs_root) {
        error_setg(errp, "9pfs backend is already initialized");
        return -1;
    }

    fse = get_fsdev_fsentry(fsdev_id);
    if (!fse) {
        error_setg(errp, "9pfs device couldn't find fsdev with the id = %s",
                   fsdev_id ? fsdev_id : "NULL");
        return -1;
    }

    backend->ops = fse->ops;
    backend->ctx.export_flags = fse->export_flags;
    backend->ctx.fs_root = g_strdup(fse->path);
    backend->ctx.exops.get_st_gen = NULL;
    backend->ctx.uid = -1;
    backend->ctx.fmode = fse->fmode;
    backend->ctx.dmode = fse->dmode;
    backend->ctx.xattr_fid_limit = fse->max_xattr;
    backend->ctx.xattr_fid_count = 0;
    backend->throttle.cfg = fse->fst.cfg;
    backend->ctx.fst = &backend->throttle;

    backend->ops_cleanup_needed = true;
    if (backend->ops->init(&backend->ctx, errp) < 0) {
        error_prepend(errp, "cannot initialize fsdev '%s': ", fsdev_id);
        goto out;
    }

    if (backend->ops->name_to_path(&backend->ctx, NULL, "/", &path) < 0) {
        error_setg_errno(errp, errno, "error in converting name to path");
        goto out;
    }
    if (backend->ops->lstat(&backend->ctx, &path, &backend->root_st)) {
        error_setg(errp, "share path %s does not exist", fse->path);
        goto out;
    }
    if (!S_ISDIR(backend->root_st.st_mode)) {
        error_setg(errp, "share path %s is not a directory", fse->path);
        goto out;
    }

    backend->dev_id = backend->root_st.st_dev;
    backend_device_maps_init(backend);
    fsdev_throttle_init(backend->ctx.fst);
    backend->throttle_initialized = true;
    backend->initialized = true;
    ret = 0;

out:
    g_free(path.data);
    if (ret < 0) {
        v9fs_backend_cleanup(backend);
    }
    return ret;
}

void v9fs_backend_cleanup(V9fsBackend *backend)
{
    if (backend->ops_cleanup_needed && backend->ops->cleanup) {
        backend->ops->cleanup(&backend->ctx);
    }
    backend->ops_cleanup_needed = false;
    if (backend->throttle_initialized) {
        fsdev_throttle_cleanup(backend->ctx.fst);
        backend->throttle_initialized = false;
    }
    qp_table_destroy(&backend->qpd_table);
    qp_table_destroy(&backend->qpp_table);
    qp_table_destroy(&backend->qpf_table);
    g_free(backend->ctx.fs_root);

    memset(&backend->ctx, 0, sizeof(backend->ctx));
    memset(&backend->root_st, 0, sizeof(backend->root_st));
    memset(&backend->throttle, 0, sizeof(backend->throttle));
    backend->dev_id = 0;
    backend->ops = NULL;
    backend->initialized = false;
}
