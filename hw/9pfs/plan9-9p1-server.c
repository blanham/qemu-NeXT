// SPDX-License-Identifier: NCSA
/*
 * Copyright (c) 2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the names of the University of Illinois/NCSA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qemu/osdep.h"

#include "hw/9pfs/plan9-9p1-server.h"

#include "hw/9pfs/9p.h"
#include "hw/9pfs/plan9-9p1-codec.h"
#include "fsdev/qemu-fsdev-throttle.h"
#include "block/thread-pool.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/aio.h"
#include "qemu/coroutine.h"
#include "qemu/main-loop.h"
#include "qom/object_interfaces.h"

#ifdef CONFIG_SLIRP
#include "net/slirp-guestfwd.h"
#endif

#define PLAN9P1_DEFAULT_QUEUE_BYTES (1024 * 1024)
#define PLAN9P1_MAX_REQUESTS 128
#define PLAN9P1_MAX_DIR_CACHE (16 * 1024 * 1024)
#define PLAN9P1_DEFAULT_DIR_CACHE_BYTES (64 * 1024 * 1024)
#define PLAN9P1_DEFAULT_QID_ENTRIES 65536
#define PLAN9P1_DMDIR UINT32_C(0x80000000)

typedef enum Plan9P1OpenKind {
    PLAN9P1_OPEN_NONE,
    PLAN9P1_OPEN_FILE,
    PLAN9P1_OPEN_DIR,
} Plan9P1OpenKind;

typedef struct Plan9P1Fid {
    struct Plan9P1Server *server;
    uint16_t fid;
    V9fsPath path;
    V9fsFidOpenState fs;
    Plan9P1Qid qid;
    Plan9P1OpenKind open_kind;
    GByteArray *dir_cache;
    GArray *dir_qids;
    GPtrArray *components;
    uint8_t name[PLAN9P1_NAMELEN];
} Plan9P1Fid;

typedef struct Plan9P1QidIdentity {
    dev_t dev;
    ino_t ino;
    size_t refs;
    uint8_t device;
    bool committed;
} Plan9P1QidIdentity;

typedef enum Plan9P1RequestKind {
    PLAN9P1_REQUEST_MESSAGE,
    PLAN9P1_REQUEST_ERROR,
    PLAN9P1_REQUEST_CLEANUP,
} Plan9P1RequestKind;

typedef struct Plan9P1Request {
    Plan9P1RequestKind kind;
    Plan9P1Fcall tx;
    struct Plan9P1Server *server;
    uint8_t *data;
    size_t charge;
    uint8_t reply[PLAN9P1_MAX_FRAME];
    size_t reply_len;
    bool cancelled;
} Plan9P1Request;

typedef struct Plan9P1Reply {
    uint8_t *data;
    size_t len;
    size_t delivered;
    uint16_t tag;
} Plan9P1Reply;

typedef struct Plan9P1DeferredInput {
    uint8_t *data;
    size_t len;
} Plan9P1DeferredInput;

typedef enum Plan9P1BackendOp {
    PLAN9P1_BACKEND_NAME_TO_PATH,
    PLAN9P1_BACKEND_LSTAT,
    PLAN9P1_BACKEND_OPEN,
    PLAN9P1_BACKEND_OPENDIR,
    PLAN9P1_BACKEND_PREADV,
    PLAN9P1_BACKEND_REWINDDIR,
    PLAN9P1_BACKEND_READDIR,
    PLAN9P1_BACKEND_CLOSE,
    PLAN9P1_BACKEND_CLOSEDIR,
} Plan9P1BackendOp;

typedef struct Plan9P1BackendWork {
    Plan9P1BackendOp op;
    V9fsBackend *backend;
    V9fsPath *dirpath;
    V9fsPath *path;
    const char *name;
    struct stat *st;
    V9fsFidOpenState *fs;
    struct iovec *iov;
    int iovcnt;
    int flags;
    off_t offset;
    char dirent_name[NAME_MAX + 1];
    bool eof;
} Plan9P1BackendWork;

struct Plan9P1Server {
    Object parent_obj;
    V9fsBackend backend_storage;
    V9fsBackend *backend;
    Plan9P1TransportOps transport_ops;
    void *transport_opaque;
    Plan9P1Stream stream;
    GHashTable *fids;
    GHashTable *qid_paths;
    dev_t devices[128];
    bool device_used[128];
    bool device_committed[128];
    size_t device_identities[128];
    unsigned int max_devices;
    GQueue requests;
    GQueue replies;
    GQueue deferred_inputs;
    Plan9P1Request *active;
    size_t queued_bytes;
    size_t max_queued_bytes;
    size_t dir_cache_bytes;
    size_t max_dir_cache_bytes;
    size_t max_qid_entries;
    unsigned int pending;
    unsigned int callback_depth;
    size_t deferred_input_bytes;
    bool started;
    bool resetting;
    bool closing;
    bool connection_failed;
    bool deferred_reset;
    bool deferred_close;
    bool deferred_connection_failure;
    bool flushing;
#ifdef CONFIG_SLIRP
    QemuSlirpGuestFwd *guestfwd;
#endif
    char *fsdev_id;
    char *netdev_id;
    char *guest_address;
    uint16_t port;
    bool completed;
};

static void server_kick(Plan9P1Server *server);
static void server_flush(Plan9P1Server *server);
static void server_start_cleanup(Plan9P1Server *server);
static void server_apply_deferred(Plan9P1Server *server);
static void server_transport_failed(Plan9P1Server *server);
static int server_receive_internal(Plan9P1Server *server,
                                   const uint8_t *buf, size_t len,
                                   Error **errp);
static void path_init(V9fsPath *path);
static void path_free(V9fsPath *path);
static void path_copy(V9fsPath *dst, const V9fsPath *src);
static void qid_release(Plan9P1Server *server, uint32_t path);

static int backend_worker(void *opaque)
{
    Plan9P1BackendWork *work = opaque;
    FileOperations *ops = work->backend->ops;
    FsContext *ctx = &work->backend->ctx;
    int ret;

    errno = 0;
    switch (work->op) {
    case PLAN9P1_BACKEND_NAME_TO_PATH:
        ret = ops->name_to_path(ctx, work->dirpath, work->name, work->path);
        break;
    case PLAN9P1_BACKEND_LSTAT:
        ret = ops->lstat(ctx, work->path, work->st);
        break;
    case PLAN9P1_BACKEND_OPEN:
        ret = ops->open(ctx, work->path, work->flags, work->fs);
        break;
    case PLAN9P1_BACKEND_OPENDIR:
        ret = ops->opendir(ctx, work->path, work->fs);
        break;
    case PLAN9P1_BACKEND_PREADV:
        ret = ops->preadv(ctx, work->fs, work->iov, work->iovcnt,
                          work->offset);
        break;
    case PLAN9P1_BACKEND_REWINDDIR:
        ops->rewinddir(ctx, work->fs);
        ret = 0;
        break;
    case PLAN9P1_BACKEND_READDIR: {
        struct dirent *entry = ops->readdir(ctx, work->fs);

        if (!entry) {
            work->eof = errno == 0;
            ret = errno ? -errno : 0;
        } else {
            g_strlcpy(work->dirent_name, entry->d_name,
                      sizeof(work->dirent_name));
            ret = 1;
        }
        break;
    }
    case PLAN9P1_BACKEND_CLOSE:
        ret = ops->close ? ops->close(ctx, work->fs) : 0;
        break;
    case PLAN9P1_BACKEND_CLOSEDIR:
        ret = ops->closedir ? ops->closedir(ctx, work->fs) : 0;
        break;
    default:
        g_assert_not_reached();
    }
    if (ret < 0 && work->op != PLAN9P1_BACKEND_READDIR) {
        ret = -errno;
    }
    return ret;
}

static int coroutine_fn run_backend(Plan9P1BackendWork *work)
{
    return thread_pool_submit_co(backend_worker, work);
}

static int coroutine_fn co_name_to_path(Plan9P1Server *server,
                                        V9fsPath *dirpath, const char *name,
                                        V9fsPath *path)
{
    V9fsPath dircopy;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_NAME_TO_PATH,
        .backend = server->backend,
        .name = name,
        .path = path,
    };
    int ret;

    path_init(&dircopy);
    if (dirpath) {
        path_copy(&dircopy, dirpath);
        work.dirpath = &dircopy;
    }
    ret = run_backend(&work);
    path_free(&dircopy);
    return ret;
}

static int coroutine_fn co_lstat(Plan9P1Server *server, V9fsPath *path,
                                 struct stat *st)
{
    V9fsPath pathcopy;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_LSTAT,
        .backend = server->backend,
        .st = st,
    };
    int ret;

    path_init(&pathcopy);
    path_copy(&pathcopy, path);
    work.path = &pathcopy;
    ret = run_backend(&work);
    path_free(&pathcopy);
    return ret;
}

static int coroutine_fn co_open(Plan9P1Server *server, V9fsPath *path,
                                int flags, V9fsFidOpenState *fs)
{
    V9fsPath pathcopy;
    V9fsFidOpenState candidate = { 0 };
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_OPEN,
        .backend = server->backend,
        .flags = flags,
        .fs = &candidate,
    };
    int ret;

    path_init(&pathcopy);
    path_copy(&pathcopy, path);
    work.path = &pathcopy;
    ret = run_backend(&work);
    path_free(&pathcopy);
    if (ret >= 0) {
        *fs = candidate;
    }
    return ret;
}

static int coroutine_fn co_opendir(Plan9P1Server *server, V9fsPath *path,
                                   V9fsFidOpenState *fs)
{
    V9fsPath pathcopy;
    V9fsFidOpenState candidate = { 0 };
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_OPENDIR,
        .backend = server->backend,
        .fs = &candidate,
    };
    int ret;

    path_init(&pathcopy);
    path_copy(&pathcopy, path);
    work.path = &pathcopy;
    ret = run_backend(&work);
    path_free(&pathcopy);
    if (ret >= 0) {
        *fs = candidate;
    }
    return ret;
}

static int coroutine_fn co_preadv(Plan9P1Server *server,
                                  V9fsFidOpenState *fs,
                                  struct iovec *iov, int iovcnt,
                                  off_t offset)
{
    V9fsFidOpenState candidate = *fs;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_PREADV,
        .backend = server->backend,
        .fs = &candidate,
        .iov = iov,
        .iovcnt = iovcnt,
        .offset = offset,
    };
    return run_backend(&work);
}

static void coroutine_fn co_rewinddir(Plan9P1Server *server,
                                      V9fsFidOpenState *fs)
{
    V9fsFidOpenState candidate = *fs;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_REWINDDIR,
        .backend = server->backend,
        .fs = &candidate,
    };
    run_backend(&work);
}

static int coroutine_fn co_readdir(Plan9P1Server *server,
                                   V9fsFidOpenState *fs,
                                   char name[NAME_MAX + 1])
{
    V9fsFidOpenState candidate = *fs;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_READDIR,
        .backend = server->backend,
        .fs = &candidate,
    };
    int ret = run_backend(&work);

    if (ret > 0) {
        g_strlcpy(name, work.dirent_name, NAME_MAX + 1);
    }
    return ret;
}

static int coroutine_fn co_close(Plan9P1Server *server,
                                 V9fsFidOpenState *fs, bool directory)
{
    V9fsFidOpenState candidate = *fs;
    Plan9P1BackendWork work = {
        .op = directory ? PLAN9P1_BACKEND_CLOSEDIR : PLAN9P1_BACKEND_CLOSE,
        .backend = server->backend,
        .fs = &candidate,
    };
    return run_backend(&work);
}

static void path_init(V9fsPath *path)
{
    path->data = NULL;
    path->size = 0;
}

static void path_free(V9fsPath *path)
{
    g_free(path->data);
    path_init(path);
}

static void path_copy(V9fsPath *dst, const V9fsPath *src)
{
    path_free(dst);
    dst->data = g_memdup2(src->data, src->size);
    dst->size = src->size;
}

static gpointer fid_key(uint16_t fid)
{
    return GUINT_TO_POINTER((unsigned int)fid + 1);
}

static void fid_free(gpointer opaque)
{
    Plan9P1Fid *fid = opaque;
    unsigned int i;

    path_free(&fid->path);
    if (fid->dir_cache) {
        assert(fid->server->dir_cache_bytes >= fid->dir_cache->len);
        fid->server->dir_cache_bytes -= fid->dir_cache->len;
        g_byte_array_unref(fid->dir_cache);
    }
    if (fid->dir_qids) {
        for (i = 0; i < fid->dir_qids->len; i++) {
            qid_release(fid->server,
                        g_array_index(fid->dir_qids, uint32_t, i));
        }
        g_array_unref(fid->dir_qids);
    }
    qid_release(fid->server, fid->qid.path);
    g_ptr_array_unref(fid->components);
    g_free(fid);
}

static void qid_identity_free(gpointer opaque)
{
    g_free(opaque);
}

static void request_free(Plan9P1Request *request)
{
    g_free(request->data);
    g_free(request);
}

static void reply_free(Plan9P1Reply *reply)
{
    g_free(reply->data);
    g_free(reply);
}

static void deferred_input_free(Plan9P1DeferredInput *input)
{
    g_free(input->data);
    g_free(input);
}

static void plan9p1_server_instance_finalize(Object *obj)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    /* Finalization must never launch asynchronous cleanup work. */
    assert(server->pending == 0);
    assert(!server->active);
    assert(g_queue_is_empty(&server->requests));
    assert(g_queue_is_empty(&server->replies));
    assert(g_queue_is_empty(&server->deferred_inputs));
    assert(g_hash_table_size(server->fids) == 0);
#ifdef CONFIG_SLIRP
    if (server->guestfwd) {
        QemuSlirpGuestFwd *guestfwd = server->guestfwd;

        server->guestfwd = NULL;
        qemu_slirp_guestfwd_remove(guestfwd);
    }
#endif
    server->closing = true;
    g_hash_table_unref(server->fids);
    g_hash_table_unref(server->qid_paths);
    v9fs_backend_cleanup(&server->backend_storage);
    server->backend = NULL;
    g_free(server->fsdev_id);
    g_free(server->netdev_id);
    g_free(server->guest_address);
}

static void owner_ref(Plan9P1Server *server)
{
    object_ref(OBJECT(server));
}

static void owner_unref(Plan9P1Server *server)
{
    object_unref(OBJECT(server));
}

static Plan9P1Fid *find_fid(Plan9P1Server *server, uint16_t fid)
{
    return g_hash_table_lookup(server->fids, fid_key(fid));
}

static void fixed_string(uint8_t dst[PLAN9P1_NAMELEN], const char *src)
{
    size_t length = MIN(strlen(src), (size_t)PLAN9P1_NAMELEN);

    memset(dst, 0, PLAN9P1_NAMELEN);
    memcpy(dst, src, length);
}

static char *decode_name(const uint8_t name[PLAN9P1_NAMELEN])
{
    size_t length = strnlen((const char *)name, PLAN9P1_NAMELEN);
    char *decoded = g_strndup((const char *)name, length);

    if (!decoded[0] || strchr(decoded, '/')) {
        g_free(decoded);
        errno = EINVAL;
        return NULL;
    }
    return decoded;
}

static uint32_t stat_mtime(const struct stat *st)
{
#if defined(CONFIG_DARWIN) || defined(CONFIG_FREEBSD)
    return st->st_mtimespec.tv_sec;
#else
    return st->st_mtim.tv_sec;
#endif
}

static int qid_from_stat(Plan9P1Server *server, const struct stat *st,
                         Plan9P1Qid *qid)
{
    Plan9P1QidIdentity *identity;
    unsigned int device;
    uint32_t path;
    bool new_device = false;

    for (device = 1; device <= server->max_devices; device++) {
        if (server->device_used[device] &&
            server->devices[device] == st->st_dev) {
            break;
        }
    }
    if (device > server->max_devices) {
        for (device = 1; device <= server->max_devices; device++) {
            if (!server->device_used[device]) {
                server->device_used[device] = true;
                server->devices[device] = st->st_dev;
                new_device = true;
                break;
            }
        }
        if (device > server->max_devices) {
            return -ENOSPC;
        }
    }

    path = ((uint32_t)device << 24) | ((uint64_t)st->st_ino & 0xffffff);
    if (S_ISDIR(st->st_mode)) {
        path |= PLAN9P1_DMDIR;
    }
    identity = g_hash_table_lookup(server->qid_paths, GUINT_TO_POINTER(path));
    if (identity && (identity->dev != st->st_dev ||
                     identity->ino != st->st_ino)) {
        if (new_device) {
            server->device_used[device] = false;
        }
        return -EOVERFLOW;
    }
    if (!identity) {
        if (g_hash_table_size(server->qid_paths) >=
            server->max_qid_entries) {
            if (new_device) {
                server->device_used[device] = false;
            }
            return -ENOSPC;
        }
        identity = g_new(Plan9P1QidIdentity, 1);
        identity->dev = st->st_dev;
        identity->ino = st->st_ino;
        identity->refs = 0;
        identity->device = device;
        identity->committed = false;
        g_hash_table_insert(server->qid_paths, GUINT_TO_POINTER(path),
                            identity);
        server->device_identities[device]++;
    }
    identity->refs++;
    qid->path = path;
    qid->vers = stat_mtime(st);
    return 0;
}

static void qid_ref(Plan9P1Server *server, uint32_t path)
{
    Plan9P1QidIdentity *identity = g_hash_table_lookup(
        server->qid_paths, GUINT_TO_POINTER(path));

    assert(identity);
    identity->refs++;
}

static void qid_commit(Plan9P1Server *server, uint32_t path)
{
    Plan9P1QidIdentity *identity = g_hash_table_lookup(
        server->qid_paths, GUINT_TO_POINTER(path));

    assert(identity);
    identity->committed = true;
    server->device_committed[identity->device] = true;
}

static void qid_release(Plan9P1Server *server, uint32_t path)
{
    Plan9P1QidIdentity *identity;

    if (!path) {
        return;
    }
    identity = g_hash_table_lookup(server->qid_paths,
                                   GUINT_TO_POINTER(path));
    assert(identity && identity->refs);
    identity->refs--;
    if (!identity->refs && !identity->committed) {
        unsigned int device = identity->device;

        assert(server->device_identities[device]);
        server->device_identities[device]--;
        g_hash_table_remove(server->qid_paths, GUINT_TO_POINTER(path));
        if (!server->device_identities[device] &&
            !server->device_committed[device]) {
            server->device_used[device] = false;
            server->devices[device] = 0;
        }
    }
}

static int fill_dir(Plan9P1Server *server, const char *name,
                    const struct stat *st, Plan9P1Dir *dir)
{
    char uid[PLAN9P1_NAMELEN];
    char gid[PLAN9P1_NAMELEN];
    int ret;

    memset(dir, 0, sizeof(*dir));
    ret = qid_from_stat(server, st, &dir->qid);
    if (ret < 0) {
        return ret;
    }
    fixed_string(dir->name, name);
    snprintf(uid, sizeof(uid), "%ju", (uintmax_t)st->st_uid);
    snprintf(gid, sizeof(gid), "%ju", (uintmax_t)st->st_gid);
    fixed_string(dir->uid, uid);
    fixed_string(dir->gid, gid);
    dir->mode = st->st_mode & 0777;
    if (S_ISDIR(st->st_mode)) {
        dir->mode |= PLAN9P1_DMDIR;
    }
    dir->atime = st->st_atime;
    dir->mtime = stat_mtime(st);
    dir->length = (uint32_t)(uint64_t)st->st_size;
    dir->dev = (dir->qid.path >> 24) & 0x7f;
    return 0;
}

static void reset_qids(Plan9P1Server *server)
{
    memset(server->device_used, 0, sizeof(server->device_used));
    memset(server->device_committed, 0, sizeof(server->device_committed));
    memset(server->device_identities, 0, sizeof(server->device_identities));
    memset(server->devices, 0, sizeof(server->devices));
    g_hash_table_remove_all(server->qid_paths);
}

static int coroutine_fn close_fid(Plan9P1Server *server, Plan9P1Fid *fid)
{
    int ret = 0;

    if (fid->open_kind == PLAN9P1_OPEN_FILE) {
        ret = co_close(server, &fid->fs, false);
    } else if (fid->open_kind == PLAN9P1_OPEN_DIR) {
        ret = co_close(server, &fid->fs, true);
    }
    fid->open_kind = PLAN9P1_OPEN_NONE;
    return ret;
}

static void coroutine_fn cleanup_fids(Plan9P1Server *server)
{
    GList *fids = g_hash_table_get_values(server->fids);
    GList *link;

    for (link = fids; link; link = link->next) {
        close_fid(server, link->data);
    }
    g_list_free(fids);
    g_hash_table_remove_all(server->fids);
    reset_qids(server);
}

static int encode_fcall(Plan9P1Request *request, const Plan9P1Fcall *reply)
{
    Error *err = NULL;
    ssize_t length;

    length = plan9p1_encode(request->reply, sizeof(request->reply), reply,
                            &err);
    if (length < 0) {
        error_report_err(err);
        return -EIO;
    }
    request->reply_len = length;
    return 0;
}

static void encode_error(Plan9P1Request *request, uint16_t tag,
                         int error, const char *message)
{
    Plan9P1Fcall reply = {
        .type = PLAN9P1_RERROR,
        .tag = tag,
    };
    const char *text = message;

    if (!text) {
        text = g_strerror(error > 0 ? error : EIO);
    }
    memcpy(reply.ename, text, MIN(strlen(text), sizeof(reply.ename)));
    encode_fcall(request, &reply);
}

static int coroutine_fn stat_path(Plan9P1Server *server, V9fsPath *path,
                                  struct stat *st)
{
    return co_lstat(server, path, st);
}

static Plan9P1Fid *new_fid(Plan9P1Server *server, uint16_t number,
                          const V9fsPath *path,
                          const Plan9P1Qid *qid, const char *name)
{
    Plan9P1Fid *fid = g_new0(Plan9P1Fid, 1);

    fid->server = server;
    fid->fid = number;
    path_init(&fid->path);
    path_copy(&fid->path, path);
    fid->qid = *qid;
    fid->components = g_ptr_array_new_with_free_func(g_free);
    if (name[0] && strcmp(name, "/")) {
        g_ptr_array_add(fid->components, g_strdup(name));
    }
    fixed_string(fid->name, name);
    return fid;
}

static int coroutine_fn attach_fid(Plan9P1Request *request,
                                   Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    V9fsPath path;
    struct stat st;
    Plan9P1Qid qid;
    int ret;

    if (find_fid(server, request->tx.fid)) {
        return -EEXIST;
    }
    path_init(&path);
    ret = co_name_to_path(server, NULL, "/", &path);
    if (ret < 0) {
        goto out;
    }
    ret = stat_path(server, &path, &st);
    if (ret < 0) {
        goto out;
    }
    ret = qid_from_stat(server, &st, &qid);
    if (ret < 0) {
        goto out;
    }
    g_hash_table_insert(server->fids, fid_key(request->tx.fid),
                        new_fid(server, request->tx.fid, &path, &qid, "/"));
    qid_commit(server, qid.path);
    reply->type = PLAN9P1_RATTACH;
    reply->fid = request->tx.fid;
    reply->qid = qid;
out:
    path_free(&path);
    return ret;
}

static int clone_fid(Plan9P1Request *request, bool walk,
                     Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *source = find_fid(server, request->tx.fid);
    Plan9P1Fid *clone;

    if (!source) {
        return -ENOENT;
    }
    if (source->open_kind != PLAN9P1_OPEN_NONE) {
        return -EBUSY;
    }
    if (find_fid(server, request->tx.newfid)) {
        return -EEXIST;
    }
    qid_ref(server, source->qid.path);
    clone = new_fid(server, request->tx.newfid,
                    &source->path, &source->qid,
                    "");
    for (unsigned int i = 0; i < source->components->len; i++) {
        g_ptr_array_add(clone->components,
                        g_strdup(source->components->pdata[i]));
    }
    memcpy(clone->name, source->name, sizeof(clone->name));
    g_hash_table_insert(server->fids, fid_key(clone->fid), clone);
    reply->type = walk ? PLAN9P1_RCLWALK : PLAN9P1_RCLONE;
    reply->fid = walk ? clone->fid : request->tx.fid;
    reply->qid = clone->qid;
    return 0;
}

static int coroutine_fn walk_fid(Plan9P1Request *request, Plan9P1Fid *fid,
                                 Plan9P1Qid *qid)
{
    Plan9P1Server *server = request->server;
    g_autofree char *name = decode_name(request->tx.name);
    V9fsPath path;
    struct stat st;
    int ret;

    if (!name) {
        return -errno;
    }
    if (!fid || fid->open_kind != PLAN9P1_OPEN_NONE) {
        return fid ? -EBUSY : -ENOENT;
    }
    path_init(&path);
    ret = co_name_to_path(server, &fid->path, name, &path);
    if (ret < 0) {
        goto out;
    }
    ret = stat_path(server, &path, &st);
    if (ret < 0) {
        goto out;
    }
    ret = qid_from_stat(server, &st, qid);
    if (ret < 0) {
        goto out;
    }
    path_copy(&fid->path, &path);
    qid_release(server, fid->qid.path);
    fid->qid = *qid;
    qid_commit(server, qid->path);
    if (!strcmp(name, "..")) {
        if (fid->components->len) {
            g_ptr_array_remove_index(fid->components,
                                     fid->components->len - 1);
        }
    } else if (strcmp(name, ".")) {
        g_ptr_array_add(fid->components, g_strdup(name));
    }
    if (fid->components->len) {
        fixed_string(fid->name,
                     g_ptr_array_index(fid->components,
                                       fid->components->len - 1));
    } else {
        fixed_string(fid->name, "/");
    }
out:
    path_free(&path);
    return ret;
}

static int coroutine_fn open_fid(Plan9P1Request *request,
                                 Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    struct stat st;
    unsigned int mode = request->tx.mode;
    int ret;

    if (!fid) {
        return -ENOENT;
    }
    if (fid->open_kind != PLAN9P1_OPEN_NONE) {
        return -EBUSY;
    }
    if ((mode & 3) == 1 || (mode & 3) == 2 || (mode & 0x50)) {
        return -EROFS;
    }
    ret = stat_path(server, &fid->path, &st);
    if (ret < 0) {
        return ret;
    }
    if (S_ISDIR(st.st_mode)) {
        if ((mode & 3) != 0) {
            return -EISDIR;
        }
        ret = co_opendir(server, &fid->path, &fid->fs);
        if (ret < 0) {
            return ret;
        }
        fid->open_kind = PLAN9P1_OPEN_DIR;
    } else {
        ret = co_open(server, &fid->path, O_RDONLY, &fid->fs);
        if (ret < 0) {
            return ret;
        }
        fid->open_kind = PLAN9P1_OPEN_FILE;
    }
    reply->type = PLAN9P1_ROPEN;
    reply->fid = fid->fid;
    reply->qid = fid->qid;
    return 0;
}

static int append_dir_record(Plan9P1Server *server, Plan9P1Fid *fid,
                             GByteArray *cache, GArray *qids,
                             const char *name,
                             const struct stat *st, size_t allowance)
{
    Plan9P1Fcall stat_reply = {
        .type = PLAN9P1_RSTAT,
        .fid = fid->fid,
    };
    uint8_t encoded[121];
    Error *err = NULL;
    ssize_t length;
    int ret;

    if (cache->len > allowance ||
        PLAN9P1_DIRLEN > allowance - cache->len) {
        return -ENOSPC;
    }
    ret = fill_dir(server, name, st, &stat_reply.dir);
    if (ret < 0) {
        return ret;
    }
    length = plan9p1_encode(encoded, sizeof(encoded), &stat_reply, &err);
    if (length != sizeof(encoded)) {
        error_free(err);
        qid_release(server, stat_reply.dir.qid.path);
        return -EIO;
    }
    if (cache->len > PLAN9P1_MAX_DIR_CACHE - PLAN9P1_DIRLEN) {
        qid_release(server, stat_reply.dir.qid.path);
        return -EOVERFLOW;
    }
    g_byte_array_append(cache, encoded + 5, PLAN9P1_DIRLEN);
    g_array_append_val(qids, stat_reply.dir.qid.path);
    return 0;
}

static int coroutine_fn build_dir_cache(Plan9P1Server *server,
                                        Plan9P1Fid *fid,
                                        Plan9P1Request *request)
{
    char name[NAME_MAX + 1];
    V9fsPath path;
    GByteArray *candidate = g_byte_array_new();
    GArray *candidate_qids = g_array_new(false, false, sizeof(uint32_t));
    struct stat st;
    unsigned int i;
    size_t allowance = server->max_dir_cache_bytes -
                       server->dir_cache_bytes;
    int ret = 0;

    co_rewinddir(server, &fid->fs);
    for (;;) {
        if (request->cancelled) {
            ret = -EINTR;
            break;
        }
        ret = co_readdir(server, &fid->fs, name);
        if (request->cancelled) {
            ret = -EINTR;
            break;
        }
        if (ret <= 0) {
            break;
        }
        if (!strcmp(name, ".") || !strcmp(name, "..")) {
            continue;
        }
        if (candidate->len > allowance ||
            PLAN9P1_DIRLEN > allowance - candidate->len) {
            ret = -ENOSPC;
            break;
        }
        path_init(&path);
        ret = co_name_to_path(server, &fid->path, name, &path);
        if (ret >= 0 && request->cancelled) {
            ret = -EINTR;
        } else if (ret >= 0) {
            ret = stat_path(server, &path, &st);
        }
        if (ret >= 0 && request->cancelled) {
            ret = -EINTR;
        } else if (ret >= 0) {
            ret = append_dir_record(server, fid, candidate, candidate_qids,
                                    name, &st, allowance);
        }
        path_free(&path);
        if (ret < 0) {
            break;
        }
    }
    if (ret == 0) {
        server->dir_cache_bytes += candidate->len;
        fid->dir_cache = candidate;
        fid->dir_qids = candidate_qids;
        for (i = 0; i < candidate_qids->len; i++) {
            qid_commit(server,
                       g_array_index(candidate_qids, uint32_t, i));
        }
        candidate = NULL;
        candidate_qids = NULL;
    }
    if (candidate) {
        g_byte_array_unref(candidate);
    }
    if (candidate_qids) {
        for (i = 0; i < candidate_qids->len; i++) {
            qid_release(server,
                        g_array_index(candidate_qids, uint32_t, i));
        }
        g_array_unref(candidate_qids);
    }
    return ret;
}

static int coroutine_fn read_fid(Plan9P1Request *request,
                                 Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    size_t count;

    if (!fid) {
        return -ENOENT;
    }
    if (fid->open_kind == PLAN9P1_OPEN_NONE) {
        return -EBADF;
    }
    reply->type = PLAN9P1_RREAD;
    reply->fid = fid->fid;
    if (fid->open_kind == PLAN9P1_OPEN_DIR) {
        int ret;

        if (request->tx.offset % PLAN9P1_DIRLEN ||
            request->tx.count % PLAN9P1_DIRLEN) {
            return -EINVAL;
        }
        if (!fid->dir_cache) {
            ret = build_dir_cache(server, fid, request);
            if (ret < 0) {
                return ret;
            }
        }
        if (request->tx.offset >= fid->dir_cache->len) {
            return 0;
        }
        count = MIN((size_t)request->tx.count,
                    fid->dir_cache->len - (size_t)request->tx.offset);
        count -= count % PLAN9P1_DIRLEN;
        reply->data = fid->dir_cache->data + request->tx.offset;
        reply->count = count;
        return 0;
    }

    if (request->tx.offset > INT64_MAX ||
        request->tx.offset > INT64_MAX - request->tx.count ||
        (uint64_t)(off_t)request->tx.offset != request->tx.offset) {
        return -EOVERFLOW;
    }
    request->data = g_malloc(request->tx.count ? request->tx.count : 1);
    if (request->tx.count) {
        struct iovec iov = {
            .iov_base = request->data,
            .iov_len = request->tx.count,
        };
        ssize_t ret;

        fsdev_co_throttle_request(server->backend->ctx.fst, THROTTLE_READ,
                                  &iov, 1);
        if (request->cancelled) {
            return -EINTR;
        }
        ret = co_preadv(server, &fid->fs, &iov, 1,
                        (off_t)request->tx.offset);

        if (ret < 0) {
            return ret;
        }
        if ((uint64_t)ret > request->tx.count) {
            return -EIO;
        }
        count = ret;
    } else {
        count = 0;
    }
    reply->data = request->data;
    reply->count = count;
    return 0;
}

static int coroutine_fn stat_fid(Plan9P1Request *request,
                                 Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    struct stat st;
    char name[PLAN9P1_NAMELEN + 1];
    int ret;

    if (!fid) {
        return -ENOENT;
    }
    ret = stat_path(server, &fid->path, &st);
    if (ret < 0) {
        return ret;
    }
    memcpy(name, fid->name, PLAN9P1_NAMELEN);
    name[PLAN9P1_NAMELEN] = 0;
    ret = fill_dir(server, name, &st, &reply->dir);
    if (ret < 0) {
        return ret;
    }
    qid_commit(server, reply->dir.qid.path);
    qid_release(server, reply->dir.qid.path);
    reply->type = PLAN9P1_RSTAT;
    reply->fid = fid->fid;
    return 0;
}

static int coroutine_fn clunk_fid(Plan9P1Request *request,
                                  Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    int ret;

    if (!fid) {
        return -ENOENT;
    }
    ret = close_fid(server, fid);

    g_hash_table_remove(server->fids, fid_key(request->tx.fid));
    if (ret < 0) {
        return ret;
    }
    reply->type = PLAN9P1_RCLUNK;
    reply->fid = request->tx.fid;
    return 0;
}

static void coroutine_fn handle_request(Plan9P1Request *request)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fcall reply = {
        .tag = request->tx.tag,
    };
    int ret = 0;

    if (request->kind == PLAN9P1_REQUEST_CLEANUP) {
        cleanup_fids(server);
        return;
    }
    if (request->kind == PLAN9P1_REQUEST_ERROR) {
        encode_error(request, request->tx.tag, EPROTO, "malformed 9P1 request");
        return;
    }

    switch (request->tx.type) {
    case PLAN9P1_TNOP:
        reply.type = PLAN9P1_RNOP;
        break;
    case PLAN9P1_TSESSION:
        cleanup_fids(server);
        reply.type = PLAN9P1_RSESSION;
        break;
    case PLAN9P1_TATTACH:
        ret = attach_fid(request, &reply);
        break;
    case PLAN9P1_TCLONE:
        ret = clone_fid(request, false, &reply);
        break;
    case PLAN9P1_TWALK: {
        Plan9P1Fid *fid = find_fid(server, request->tx.fid);
        ret = walk_fid(request, fid, &reply.qid);
        if (ret >= 0) {
            reply.type = PLAN9P1_RWALK;
            reply.fid = request->tx.fid;
        }
        break;
    }
    case PLAN9P1_TCLWALK:
        ret = clone_fid(request, true, &reply);
        if (ret >= 0) {
            Plan9P1Fid *fid = find_fid(server, request->tx.newfid);
            ret = walk_fid(request, fid, &reply.qid);
            if (ret < 0) {
                g_hash_table_remove(server->fids, fid_key(request->tx.newfid));
                if (ret == -ENOENT) {
                    memset(&reply.qid, 0, sizeof(reply.qid));
                    reply.type = PLAN9P1_RCLWALK;
                    reply.fid = request->tx.fid;
                    ret = 0;
                }
            }
        }
        break;
    case PLAN9P1_TOPEN:
        ret = open_fid(request, &reply);
        break;
    case PLAN9P1_TREAD:
        ret = read_fid(request, &reply);
        break;
    case PLAN9P1_TSTAT:
        ret = stat_fid(request, &reply);
        break;
    case PLAN9P1_TCLUNK:
        ret = clunk_fid(request, &reply);
        break;
    case PLAN9P1_TFLUSH:
        reply.type = PLAN9P1_RFLUSH;
        break;
    case PLAN9P1_TCREATE:
    case PLAN9P1_TWRITE:
    case PLAN9P1_TREMOVE:
    case PLAN9P1_TWSTAT:
        ret = -EROFS;
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }
    if (ret < 0) {
        encode_error(request, request->tx.tag, -ret, NULL);
    } else {
        encode_fcall(request, &reply);
    }
}

static void clear_requests(Plan9P1Server *server)
{
    Plan9P1Request *request;

    while ((request = g_queue_pop_head(&server->requests))) {
        server->queued_bytes -= request->charge;
        request_free(request);
    }
}

static void clear_replies(Plan9P1Server *server)
{
    Plan9P1Reply *reply;

    while ((reply = g_queue_pop_head(&server->replies))) {
        server->queued_bytes -= reply->len;
        reply_free(reply);
    }
}

static void queue_reply(Plan9P1Server *server, const uint8_t *data, size_t len,
                        uint16_t tag)
{
    Plan9P1Reply *reply;

    if (len > server->max_queued_bytes - server->queued_bytes) {
        server_transport_failed(server);
        return;
    }
    reply = g_new0(Plan9P1Reply, 1);
    reply->data = g_memdup2(data, len);
    reply->len = len;
    reply->tag = tag;
    server->queued_bytes += len;
    g_queue_push_tail(&server->replies, reply);
}

static void request_complete(Plan9P1Request *request)
{
    Plan9P1Server *server = request->server;
    bool cleanup = request->kind == PLAN9P1_REQUEST_CLEANUP;

    assert(server->active == request);
    server->active = NULL;
    server->pending--;
    if (cleanup) {
        server->resetting = false;
    } else if (!server->closing && !server->resetting &&
               !server->connection_failed &&
               !request->cancelled && request->reply_len) {
        queue_reply(server, request->reply, request->reply_len,
                    request->tx.tag);
    }
    request_free(request);

    if ((server->closing || server->resetting) &&
        g_hash_table_size(server->fids)) {
        server_start_cleanup(server);
    } else if (server->resetting) {
        reset_qids(server);
        server->resetting = false;
    } else if (!server->closing && !server->connection_failed) {
        server_flush(server);
        server_kick(server);
    }
    owner_unref(server);
}

static void coroutine_fn request_coroutine(void *opaque)
{
    Plan9P1Request *request = opaque;

    handle_request(request);
    request_complete(request);
}

static void start_request(Plan9P1Server *server, Plan9P1Request *request)
{
    Coroutine *coroutine;

    assert(!server->active);
    server->active = request;
    request->server = server;
    server->pending++;
    owner_ref(server);
    coroutine = qemu_coroutine_create(request_coroutine, request);
    aio_co_schedule(qemu_get_aio_context(), coroutine);
}

static void server_start_cleanup(Plan9P1Server *server)
{
    Plan9P1Request *request;

    if (server->active) {
        return;
    }
    request = g_new0(Plan9P1Request, 1);
    request->kind = PLAN9P1_REQUEST_CLEANUP;
    start_request(server, request);
}

static void server_kick(Plan9P1Server *server)
{
    Plan9P1Request *request;

    if (server->active || server->closing || server->resetting ||
        server->connection_failed) {
        return;
    }
    request = g_queue_pop_head(&server->requests);
    if (!request) {
        return;
    }
    server->queued_bytes -= request->charge;
    start_request(server, request);
}

static void clear_deferred_inputs(Plan9P1Server *server)
{
    Plan9P1DeferredInput *input;

    while ((input = g_queue_pop_head(&server->deferred_inputs))) {
        server->deferred_input_bytes -= input->len;
        deferred_input_free(input);
    }
}

static void server_discard_session(Plan9P1Server *server)
{
    plan9p1_stream_reset(&server->stream);
    clear_requests(server);
    clear_replies(server);
    server->resetting = true;
    if (server->active) {
        server->active->cancelled = true;
    } else if (g_hash_table_size(server->fids)) {
        server_start_cleanup(server);
    } else {
        reset_qids(server);
        server->resetting = false;
    }
}

static void server_transport_failed(Plan9P1Server *server)
{
    server->connection_failed = true;
    if (server->callback_depth) {
        server->deferred_connection_failure = true;
        return;
    }
    clear_deferred_inputs(server);
    server_discard_session(server);
}

static bool server_has_deferred(const Plan9P1Server *server)
{
    return server->deferred_close || server->deferred_reset ||
           server->deferred_connection_failure ||
           server->deferred_inputs.length != 0;
}

static void server_apply_deferred(Plan9P1Server *server)
{
    Plan9P1DeferredInput *input;

    assert(server->callback_depth == 0);
    if (server->deferred_close) {
        server->deferred_close = false;
        clear_deferred_inputs(server);
        server_discard_session(server);
        return;
    }
    if (server->deferred_connection_failure) {
        server->deferred_connection_failure = false;
        clear_deferred_inputs(server);
        server_discard_session(server);
        return;
    }
    if (server->deferred_reset) {
        server->deferred_reset = false;
        clear_deferred_inputs(server);
        server->connection_failed = false;
        server_discard_session(server);
        return;
    }
    while ((input = g_queue_pop_head(&server->deferred_inputs))) {
        Error *local_err = NULL;
        int ret;

        server->deferred_input_bytes -= input->len;
        ret = server_receive_internal(server, input->data, input->len,
                                      &local_err);
        deferred_input_free(input);
        if (ret < 0) {
            error_free(local_err);
            break;
        }
    }
}

static void server_flush(Plan9P1Server *server)
{
    if (server->flushing || server->closing || server->connection_failed) {
        return;
    }
    owner_ref(server);
    server->flushing = true;
    while (!g_queue_is_empty(&server->replies)) {
        Plan9P1Reply *reply = g_queue_peek_head(&server->replies);
        size_t remaining = reply->len - reply->delivered;
        size_t capacity;
        size_t amount;
        int sent;

        owner_ref(server);
        server->callback_depth++;
        capacity = server->transport_ops.can_send(server->transport_opaque);
        server->callback_depth--;
        owner_unref(server);
        if (server_has_deferred(server)) {
            server_apply_deferred(server);
        }
        if (g_queue_peek_head(&server->replies) != reply) {
            break;
        }
        amount = MIN(remaining, capacity);
        if (!amount) {
            break;
        }
        owner_ref(server);
        server->callback_depth++;
        sent = server->transport_ops.send(reply->data + reply->delivered,
                                          amount, server->transport_opaque);
        server->callback_depth--;
        owner_unref(server);
        if (server_has_deferred(server)) {
            server_apply_deferred(server);
        }
        if (g_queue_peek_head(&server->replies) != reply) {
            break;
        }
        if (sent < 0 || (size_t)sent > amount) {
            server_transport_failed(server);
            break;
        }
        if (!sent) {
            break;
        }
        reply->delivered += sent;
        if (reply->delivered == reply->len) {
            g_queue_pop_head(&server->replies);
            server->queued_bytes -= reply->len;
            reply_free(reply);
        }
        if ((size_t)sent < amount) {
            break;
        }
    }
    server->flushing = false;
    owner_unref(server);
}

static bool request_type(uint8_t type)
{
    switch (type) {
    case PLAN9P1_TNOP:
    case PLAN9P1_TFLUSH:
    case PLAN9P1_TCLONE:
    case PLAN9P1_TWALK:
    case PLAN9P1_TOPEN:
    case PLAN9P1_TCREATE:
    case PLAN9P1_TREAD:
    case PLAN9P1_TWRITE:
    case PLAN9P1_TCLUNK:
    case PLAN9P1_TREMOVE:
    case PLAN9P1_TSTAT:
    case PLAN9P1_TWSTAT:
    case PLAN9P1_TCLWALK:
    case PLAN9P1_TSESSION:
    case PLAN9P1_TATTACH:
        return true;
    default:
        return false;
    }
}

static void cancel_tag(Plan9P1Server *server, uint16_t tag)
{
    GList *link = server->requests.head;

    if (server->active && server->active->tx.tag == tag) {
        server->active->cancelled = true;
    }
    while (link) {
        Plan9P1Request *request = link->data;
        GList *next = link->next;

        if (request->tx.tag == tag) {
            g_queue_delete_link(&server->requests, link);
            server->queued_bytes -= request->charge;
            request_free(request);
        }
        link = next;
    }
    link = server->replies.head;
    while (link) {
        Plan9P1Reply *reply = link->data;
        GList *next = link->next;

        if (reply->tag == tag && reply->delivered == 0) {
            g_queue_delete_link(&server->replies, link);
            server->queued_bytes -= reply->len;
            reply_free(reply);
        }
        link = next;
    }
}

static int enqueue_frame(const uint8_t *frame, size_t length,
                         void *opaque, Error **errp)
{
    Plan9P1Server *server = opaque;
    Plan9P1Request *request;
    Error *local_err = NULL;

    if (g_queue_get_length(&server->requests) >= PLAN9P1_MAX_REQUESTS ||
        length > server->max_queued_bytes - server->queued_bytes) {
        error_setg(errp, "9P1 request queue is full");
        return -1;
    }
    request = g_new0(Plan9P1Request, 1);
    request->kind = PLAN9P1_REQUEST_MESSAGE;
    request->charge = length;
    if (plan9p1_decode(frame, length, &request->tx, &local_err) < 0) {
        error_propagate(errp, local_err);
        request_free(request);
        return -1;
    }
    if (!request_type(request->tx.type)) {
        error_setg(errp, "unexpected 9P1 response type %u", request->tx.type);
        request_free(request);
        return -1;
    }
    if (request->tx.type == PLAN9P1_TWRITE && request->tx.count) {
        request->data = g_memdup2(request->tx.data, request->tx.count);
        request->tx.data = request->data;
    }
    if (request->tx.type == PLAN9P1_TFLUSH) {
        cancel_tag(server, request->tx.oldtag);
    } else if (request->tx.type == PLAN9P1_TSESSION) {
        server->connection_failed = false;
        clear_requests(server);
        clear_replies(server);
        if (server->active) {
            server->active->cancelled = true;
        }
    }
    server->queued_bytes += request->charge;
    if (request->tx.type == PLAN9P1_TFLUSH) {
        g_queue_push_head(&server->requests, request);
    } else {
        g_queue_push_tail(&server->requests, request);
    }
    server_kick(server);
    return 0;
}

static void plan9p1_server_instance_init(Object *obj)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    plan9p1_stream_init(&server->stream);
    server->fids = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                         NULL, fid_free);
    server->qid_paths = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                              NULL, qid_identity_free);
    g_queue_init(&server->requests);
    g_queue_init(&server->replies);
    g_queue_init(&server->deferred_inputs);
    server->max_devices = 127;
    server->max_queued_bytes = PLAN9P1_DEFAULT_QUEUE_BYTES;
    server->max_dir_cache_bytes = PLAN9P1_DEFAULT_DIR_CACHE_BYTES;
    server->max_qid_entries = PLAN9P1_DEFAULT_QID_ENTRIES;
    server->guest_address = g_strdup("10.0.2.100");
    server->port = 564;
}

int plan9p1_server_backend_init(Plan9P1Server *server,
                                const char *fsdev_id,
                                const Plan9P1ServerOptions *options,
                                Error **errp)
{
    unsigned int max_devices = options ? options->max_devices : 0;

    if (!server || server->backend) {
        error_setg(errp, "9P1 server backend is already initialized");
        return -1;
    }
    if (max_devices > 127) {
        error_setg(errp, "9P1 maximum device count must not exceed 127");
        return -1;
    }
    server->max_devices = max_devices ? max_devices : 127;
    server->max_queued_bytes = options && options->max_queued_bytes ?
        options->max_queued_bytes : PLAN9P1_DEFAULT_QUEUE_BYTES;
    server->max_dir_cache_bytes = options && options->max_dir_cache_bytes ?
        options->max_dir_cache_bytes : PLAN9P1_DEFAULT_DIR_CACHE_BYTES;
    server->max_qid_entries = options && options->max_qid_entries ?
        options->max_qid_entries : PLAN9P1_DEFAULT_QID_ENTRIES;
    if (server->max_queued_bytes < PLAN9P1_MAX_FRAME) {
        error_setg(errp, "9P1 queue bound is too small for one maximum frame");
        return -1;
    }
    if (v9fs_backend_init(&server->backend_storage, fsdev_id, errp) < 0) {
        return -1;
    }
    server->backend = &server->backend_storage;
    return 0;
}

int plan9p1_server_start(Plan9P1Server *server,
                         const Plan9P1TransportOps *ops,
                         void *transport_opaque,
                         Error **errp)
{
    if (!server || !server->backend) {
        error_setg(errp, "9P1 server requires an initialized backend");
        return -1;
    }
    if (server->started) {
        error_setg(errp, "9P1 server transport is already started");
        return -1;
    }
    if (!ops || !ops->can_send || !ops->send) {
        error_setg(errp, "9P1 server requires complete transport callbacks");
        return -1;
    }
    server->transport_ops = *ops;
    server->transport_opaque = transport_opaque;
    server->started = true;
    return 0;
}

Plan9P1Server *plan9p1_server_new(const char *fsdev_id,
                                  const Plan9P1TransportOps *ops,
                                  void *transport_opaque,
                                  const Plan9P1ServerOptions *options,
                                  Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(
        object_new(TYPE_PLAN9P1_SERVER));

    if (plan9p1_server_backend_init(server, fsdev_id, options, errp) < 0 ||
        plan9p1_server_start(server, ops, transport_opaque, errp) < 0) {
        object_unref(OBJECT(server));
        return NULL;
    }
    return server;
}

static int server_receive_internal(Plan9P1Server *server,
                                   const uint8_t *buf, size_t len,
                                   Error **errp)
{
    Error *local_err = NULL;
    uint16_t tag;
    int ret;

    if (server->connection_failed && server->stream.used == 0 &&
        (!len || !buf || buf[0] != PLAN9P1_TSESSION)) {
        error_setg(errp, "9P1 reconnect must begin with Tsession");
        return -1;
    }
    ret = plan9p1_stream_feed(&server->stream, buf, len, enqueue_frame,
                              server, &local_err);
    if (ret >= 0) {
        return 0;
    }

    if (plan9p1_stream_error_tag(&server->stream, &tag) &&
        request_type(server->stream.frame[0])) {
        Plan9P1Request *request = g_new0(Plan9P1Request, 1);

        request->kind = PLAN9P1_REQUEST_ERROR;
        request->tx.tag = tag;
        request->charge = 3;
        if (server->requests.length < PLAN9P1_MAX_REQUESTS &&
            server->queued_bytes <= server->max_queued_bytes - 3) {
            server->queued_bytes += 3;
            g_queue_push_tail(&server->requests, request);
            plan9p1_stream_reset(&server->stream);
            server_kick(server);
            error_free(local_err);
            return 0;
        }
        request_free(request);
    }
    server_transport_failed(server);
    error_propagate(errp, local_err);
    return -1;
}

int plan9p1_server_receive(Plan9P1Server *server,
                           const uint8_t *buf, size_t len,
                           Error **errp)
{
    Plan9P1DeferredInput *input;
    int ret;

    if (!server) {
        error_setg(errp, "9P1 server is NULL");
        return -1;
    }
    owner_ref(server);
    if (!server->started) {
        error_setg(errp, "9P1 server transport is not started");
        ret = -1;
    } else if (server->closing) {
        error_setg(errp, "9P1 transport is closed");
        ret = -1;
    } else if (server->resetting) {
        error_setg(errp, "9P1 server reset is in progress");
        ret = -1;
    } else if (server->callback_depth) {
        if ((!buf && len) ||
            server->deferred_input_bytes >
                server->max_queued_bytes - server->queued_bytes ||
            len > server->max_queued_bytes - server->queued_bytes -
                  server->deferred_input_bytes) {
            error_setg(errp, "9P1 deferred input queue is full");
            ret = -1;
        } else {
            input = g_new0(Plan9P1DeferredInput, 1);
            input->data = g_memdup2(buf, len);
            input->len = len;
            server->deferred_input_bytes += len;
            g_queue_push_tail(&server->deferred_inputs, input);
            ret = 0;
        }
    } else {
        ret = server_receive_internal(server, buf, len, errp);
    }
    owner_unref(server);
    return ret;
}

void plan9p1_server_can_send(Plan9P1Server *server)
{
    if (server) {
        owner_ref(server);
        server_flush(server);
        if (!server->closing && !server->connection_failed) {
            server_kick(server);
        }
        owner_unref(server);
    }
}

void plan9p1_server_reset(Plan9P1Server *server)
{
    if (!server || server->closing) {
        return;
    }
    if (server->callback_depth) {
        server->resetting = true;
        server->deferred_reset = true;
        return;
    }
    server->connection_failed = false;
    server->resetting = true;
    plan9p1_stream_reset(&server->stream);
    clear_requests(server);
    clear_replies(server);
    if (server->active) {
        server->active->cancelled = true;
    } else if (g_hash_table_size(server->fids)) {
        server_start_cleanup(server);
    } else {
        reset_qids(server);
        server->resetting = false;
    }
}

bool plan9p1_server_busy(const Plan9P1Server *server)
{
    return server && (server->pending || server->requests.length != 0);
}

void plan9p1_server_begin_close(Plan9P1Server *server)
{
    if (!server) {
        return;
    }
    if (server->closing) {
        return;
    }

    server->closing = true;
#ifdef CONFIG_SLIRP
    if (server->guestfwd) {
        QemuSlirpGuestFwd *guestfwd = server->guestfwd;

        server->guestfwd = NULL;
        qemu_slirp_guestfwd_remove(guestfwd);
    }
#endif
    if (server->callback_depth) {
        server->deferred_close = true;
        return;
    }
    plan9p1_stream_reset(&server->stream);
    clear_deferred_inputs(server);
    clear_requests(server);
    clear_replies(server);
    if (server->active) {
        server->active->cancelled = true;
    } else if (g_hash_table_size(server->fids)) {
        server_start_cleanup(server);
    }
}

void plan9p1_server_free(Plan9P1Server *server)
{
    if (!server) {
        return;
    }
    plan9p1_server_begin_close(server);
    object_unref(OBJECT(server));
}

#ifdef CONFIG_SLIRP
static bool plan9p1_server_properties_mutable(Plan9P1Server *server,
                                               Error **errp)
{
    if (server->completed || server->backend || server->started ||
        server->closing) {
        error_setg(errp, "9P1 server properties cannot change after "
                   "completion");
        return false;
    }
    return true;
}

static char *plan9p1_server_get_fsdev(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->fsdev_id);
}

static void plan9p1_server_set_fsdev(Object *obj, const char *value,
                                     Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    g_free(server->fsdev_id);
    server->fsdev_id = g_strdup(value);
}

static char *plan9p1_server_get_netdev(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->netdev_id);
}

static void plan9p1_server_set_netdev(Object *obj, const char *value,
                                      Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    g_free(server->netdev_id);
    server->netdev_id = g_strdup(value);
}

static char *plan9p1_server_get_guest_address(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->guest_address);
}

static void plan9p1_server_set_guest_address(Object *obj, const char *value,
                                              Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    g_free(server->guest_address);
    server->guest_address = g_strdup(value);
}

static void plan9p1_server_get_port(Object *obj, Visitor *visitor,
                                    const char *name, void *opaque,
                                    Error **errp)
{
    uint16_t value = PLAN9P1_SERVER(obj)->port;

    visit_type_uint16(visitor, name, &value, errp);
}

static void plan9p1_server_set_port(Object *obj, Visitor *visitor,
                                    const char *name, void *opaque,
                                    Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);
    uint16_t value;

    if (!visit_type_uint16(visitor, name, &value, errp) ||
        !plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    if (!value) {
        error_setg(errp, "9P1 server port must not be zero");
        return;
    }
    server->port = value;
}

static ssize_t plan9p1_server_guest_write(const void *buf, size_t len,
                                           void *opaque)
{
    Plan9P1Server *server = opaque;
    Error *local_err = NULL;

    if (plan9p1_server_receive(server, buf, len, &local_err) < 0) {
        error_free(local_err);
        return -EIO;
    }
    return len;
}

static void plan9p1_server_guest_can_send(void *opaque)
{
    plan9p1_server_can_send(opaque);
}

static size_t plan9p1_server_transport_can_send(void *opaque)
{
    Plan9P1Server *server = opaque;

    return qemu_slirp_guestfwd_can_send(server->guestfwd);
}

static int plan9p1_server_transport_send(const uint8_t *buf, size_t len,
                                         void *opaque)
{
    Plan9P1Server *server = opaque;

    return qemu_slirp_guestfwd_send(server->guestfwd, buf, len);
}

static const QemuSlirpGuestFwdOps plan9p1_server_guestfwd_ops = {
    .write = plan9p1_server_guest_write,
    .can_send = plan9p1_server_guest_can_send,
};

static const Plan9P1TransportOps plan9p1_server_transport_ops = {
    .can_send = plan9p1_server_transport_can_send,
    .send = plan9p1_server_transport_send,
};

static void plan9p1_server_complete(UserCreatable *uc, Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(uc);
    QemuSlirpPlan9BootpConfig bootp = { 0 };
    QemuSlirpIPv4Config ipv4;
    struct in_addr guest_address;

    if (server->completed) {
        error_setg(errp, "9P1 server is already complete");
        return;
    }
    if (!server->fsdev_id || !server->fsdev_id[0]) {
        error_setg(errp, "9P1 server requires an fsdev property");
        return;
    }
    if (!server->netdev_id || !server->netdev_id[0]) {
        error_setg(errp, "9P1 server requires a netdev property");
        return;
    }
    if (!server->guest_address ||
        inet_pton(AF_INET, server->guest_address, &guest_address) != 1) {
        error_setg(errp, "Invalid 9P1 guest address '%s'",
                   server->guest_address ?: "");
        return;
    }
    if (!server->port) {
        error_setg(errp, "9P1 server port must not be zero");
        return;
    }

    if (plan9p1_server_backend_init(server, server->fsdev_id, NULL, errp) < 0) {
        return;
    }
    if (qemu_slirp_guestfwd_add(server->netdev_id, guest_address,
                                server->port, &plan9p1_server_guestfwd_ops,
                                server, &server->guestfwd, errp) < 0) {
        goto fail_backend;
    }
    if (!qemu_slirp_guestfwd_get_ipv4_config(server->guestfwd, &ipv4, errp)) {
        goto fail_guestfwd;
    }
    bootp.netmask = ipv4.netmask;
    bootp.file_server = guest_address;
    bootp.gateway = ipv4.host;
    if (!qemu_slirp_guestfwd_set_plan9_bootp(server->guestfwd, &bootp,
                                             errp)) {
        goto fail_guestfwd;
    }
    if (plan9p1_server_start(server, &plan9p1_server_transport_ops,
                             server, errp) < 0) {
        goto fail_guestfwd;
    }
    server->completed = true;
    return;

fail_guestfwd:
    qemu_slirp_guestfwd_remove(server->guestfwd);
    server->guestfwd = NULL;
fail_backend:
    v9fs_backend_cleanup(&server->backend_storage);
    server->backend = NULL;
}

static bool plan9p1_server_prepare_delete(UserCreatable *uc, Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(uc);

    if (plan9p1_server_busy(server)) {
        error_setg(errp, "9P1 server has pending filesystem work");
        return false;
    }
    plan9p1_server_begin_close(server);
    return true;
}

static void plan9p1_server_unparent(Object *obj)
{
    plan9p1_server_begin_close(PLAN9P1_SERVER(obj));
}

static void plan9p1_server_class_init(ObjectClass *oc, const void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);
    ObjectProperty *property;

    ucc->complete = plan9p1_server_complete;
    ucc->prepare_delete = plan9p1_server_prepare_delete;
    oc->unparent = plan9p1_server_unparent;

    object_class_property_add_str(oc, "fsdev", plan9p1_server_get_fsdev,
                                  plan9p1_server_set_fsdev);
    object_class_property_add_str(oc, "netdev", plan9p1_server_get_netdev,
                                  plan9p1_server_set_netdev);
    property = object_class_property_add_str(
        oc, "guest-address", plan9p1_server_get_guest_address,
        plan9p1_server_set_guest_address);
    object_property_set_default_str(property, "10.0.2.100");
    property = object_class_property_add(oc, "port", "uint16",
                                         plan9p1_server_get_port,
                                         plan9p1_server_set_port,
                                         NULL, NULL);
    object_property_set_default_uint(property, 564);
}
#endif

static const TypeInfo plan9p1_server_type_info = {
    .name = TYPE_PLAN9P1_SERVER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(Plan9P1Server),
    .instance_init = plan9p1_server_instance_init,
    .instance_finalize = plan9p1_server_instance_finalize,
#ifdef CONFIG_SLIRP
    .class_init = plan9p1_server_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_USER_CREATABLE },
        { }
    },
#endif
};

static void plan9p1_server_register_types(void)
{
    type_register_static(&plan9p1_server_type_info);
}

type_init(plan9p1_server_register_types)
