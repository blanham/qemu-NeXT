/*
 * 9p backend
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Aneesh Kumar K.V <aneesh.kumar@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

/*
 * Not so fast! You might want to read the 9p developer docs first:
 * https://wiki.qemu.org/Documentation/9p
 */

#include "qemu/osdep.h"
#include "fsdev/qemu-fsdev.h"
#include "qemu/thread.h"
#include "qemu/main-loop.h"
#include "coth.h"

static ssize_t __readlink(V9fsState *s, V9fsPath *path, V9fsString *buf)
{
    ssize_t len, maxlen = PATH_MAX;

    buf->data = g_malloc(PATH_MAX);
    for (;;) {
        len = s->backend.ops->readlink(&s->backend.ctx, path, buf->data,
                                       maxlen);
        if (len < 0) {
            g_free(buf->data);
            buf->data = NULL;
            buf->size = 0;
            break;
        } else if (len == maxlen) {
            /*
             * We dodn't have space to put the NULL or we have more
             * to read. Increase the size and try again
             */
            maxlen *= 2;
            g_free(buf->data);
            buf->data = g_malloc(maxlen);
            continue;
        }
        /*
         * Null terminate the readlink output
         */
        buf->data[len] = '\0';
        buf->size = len;
        break;
    }
    return len;
}

int coroutine_fn v9fs_co_readlink(V9fsPDU *pdu, V9fsPath *path, V9fsString *buf)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = __readlink(s, path, buf);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_statfs(V9fsPDU *pdu, V9fsPath *path,
                                struct statfs *stbuf)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->statfs(&s->backend.ctx, path, stbuf);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_chmod(V9fsPDU *pdu, V9fsPath *path, mode_t mode)
{
    int err;
    FsCred cred;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    cred_init(&cred);
    cred.fc_mode = mode;
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->chmod(&s->backend.ctx, path, &cred);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_utimensat(V9fsPDU *pdu, V9fsPath *path,
                                   struct timespec times[2])
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->utimensat(&s->backend.ctx, path, times);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_futimens(V9fsPDU *pdu, V9fsFidState *fidp,
                                  struct timespec times[2])
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->futimens(&s->backend.ctx, fidp->fid_type,
                                           &fidp->fs, times);
            if (err < 0) {
                err = -errno;
            }
        });
    return err;
}

int coroutine_fn v9fs_co_chown(V9fsPDU *pdu, V9fsPath *path, uid_t uid,
                               gid_t gid)
{
    int err;
    FsCred cred;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    cred_init(&cred);
    cred.fc_uid = uid;
    cred.fc_gid = gid;
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->chown(&s->backend.ctx, path, &cred);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_truncate(V9fsPDU *pdu, V9fsPath *path, off_t size)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->truncate(&s->backend.ctx, path, size);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_ftruncate(V9fsPDU *pdu, V9fsFidState *fidp, off_t size)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->ftruncate(&s->backend.ctx, fidp->fid_type,
                                            &fidp->fs, size);
            if (err < 0) {
                err = -errno;
            }
        });
    return err;
}

int coroutine_fn v9fs_co_mknod(V9fsPDU *pdu, V9fsFidState *fidp,
                               V9fsString *name, uid_t uid, gid_t gid,
                               dev_t dev, mode_t mode, struct stat *stbuf)
{
    int err;
    V9fsPath path;
    FsCred cred;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    cred_init(&cred);
    cred.fc_uid  = uid;
    cred.fc_gid  = gid;
    cred.fc_mode = mode;
    cred.fc_rdev = dev;
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->mknod(&s->backend.ctx, &fidp->path,
                                        name->data, &cred);
            if (err < 0) {
                err = -errno;
            } else {
                v9fs_path_init(&path);
                err = v9fs_name_to_path(s, &fidp->path, name->data, &path);
                if (!err) {
                    err = s->backend.ops->lstat(&s->backend.ctx, &path, stbuf);
                    if (err < 0) {
                        err = -errno;
                    }
                }
                v9fs_path_free(&path);
            }
        });
    v9fs_path_unlock(s);
    return err;
}

/* Only works with path name based fid */
int coroutine_fn v9fs_co_remove(V9fsPDU *pdu, V9fsPath *path)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->remove(&s->backend.ctx, path->data);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

int coroutine_fn v9fs_co_unlinkat(V9fsPDU *pdu, V9fsPath *path,
                                  V9fsString *name, int flags)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->unlinkat(&s->backend.ctx, path,
                                           name->data, flags);
            if (err < 0) {
                err = -errno;
            }
        });
    v9fs_path_unlock(s);
    return err;
}

/* Only work with path name based fid */
int coroutine_fn v9fs_co_rename(V9fsPDU *pdu, V9fsPath *oldpath,
                                V9fsPath *newpath)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->rename(&s->backend.ctx, oldpath->data,
                                         newpath->data);
            if (err < 0) {
                err = -errno;
            }
        });
    return err;
}

int coroutine_fn v9fs_co_renameat(V9fsPDU *pdu, V9fsPath *olddirpath,
                                  V9fsString *oldname, V9fsPath *newdirpath,
                                  V9fsString *newname)
{
    int err;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->renameat(&s->backend.ctx, olddirpath,
                                           oldname->data, newdirpath,
                                           newname->data);
            if (err < 0) {
                err = -errno;
            }
        });
    return err;
}

int coroutine_fn v9fs_co_symlink(V9fsPDU *pdu, V9fsFidState *dfidp,
                                 V9fsString *name, const char *oldpath,
                                 gid_t gid, struct stat *stbuf)
{
    int err;
    FsCred cred;
    V9fsPath path;
    V9fsState *s = pdu->s;

    if (v9fs_request_cancelled(pdu)) {
        return -EINTR;
    }
    cred_init(&cred);
    cred.fc_uid = dfidp->uid;
    cred.fc_gid = gid;
    cred.fc_mode = 0777;
    v9fs_path_read_lock(s);
    v9fs_co_run_in_worker(
        {
            err = s->backend.ops->symlink(&s->backend.ctx, oldpath,
                                          &dfidp->path, name->data, &cred);
            if (err < 0) {
                err = -errno;
            } else {
                v9fs_path_init(&path);
                err = v9fs_name_to_path(s, &dfidp->path, name->data, &path);
                if (!err) {
                    err = s->backend.ops->lstat(&s->backend.ctx, &path, stbuf);
                    if (err < 0) {
                        err = -errno;
                    }
                }
                v9fs_path_free(&path);
            }
        });
    v9fs_path_unlock(s);
    return err;
}

/*
 * For path name based fid we don't block. So we can
 * directly call the fs driver ops.
 */
int coroutine_fn v9fs_co_name_to_path(V9fsPDU *pdu, V9fsPath *dirpath,
                                      const char *name, V9fsPath *path)
{
    int err;
    V9fsState *s = pdu->s;

    if (s->backend.ctx.export_flags & V9FS_PATHNAME_FSCONTEXT) {
        err = s->backend.ops->name_to_path(&s->backend.ctx, dirpath, name,
                                           path);
        if (err < 0) {
            err = -errno;
        }
    } else {
        if (v9fs_request_cancelled(pdu)) {
            return -EINTR;
        }
        v9fs_co_run_in_worker(
            {
                err = s->backend.ops->name_to_path(&s->backend.ctx, dirpath,
                                                   name, path);
                if (err < 0) {
                    err = -errno;
                }
            });
    }
    return err;
}
