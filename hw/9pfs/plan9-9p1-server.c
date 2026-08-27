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
#include "crypto/random.h"
#include "crypto/secret_common.h"
#include "hw/core/resettable.h"
#include "migration/blocker.h"
#include "qapi/error.h"
#include "qapi/qapi-types-qom.h"
#include "qapi/visitor.h"
#include "qemu/aio.h"
#include "qemu/coroutine.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "system/reset.h"
#include "qom/object_interfaces.h"

#ifdef CONFIG_SLIRP
#include "net/slirp-guestfwd.h"
#include "net/slirp-il.h"
#include "net/slirp-plan9.h"
#endif

#define PLAN9P1_DEFAULT_QUEUE_BYTES (1024 * 1024)
#define PLAN9P1_MAX_REQUESTS 128
#define PLAN9P1_MAX_DIR_CACHE (16 * 1024 * 1024)
#define PLAN9P1_DEFAULT_DIR_CACHE_BYTES (64 * 1024 * 1024)
#define PLAN9P1_DEFAULT_QID_ENTRIES 65536
#define PLAN9P1_DMDIR UINT32_C(0x80000000)
#define PLAN9P1_DMAPPEND UINT32_C(0x40000000)
#define PLAN9P1_DMLOCK UINT32_C(0x20000000)

#define PLAN9P1_OREAD 0
#define PLAN9P1_OWRITE 1
#define PLAN9P1_ORDWR 2
#define PLAN9P1_OEXEC 3
#define PLAN9P1_OTRUNC 0x10
#define PLAN9P1_OCEXEC 0x20
#define PLAN9P1_ORCLOSE 0x40
#define PLAN9P1_OPEN_MASK 0x73

typedef enum Plan9P1OpenKind {
    PLAN9P1_OPEN_NONE,
    PLAN9P1_OPEN_FILE,
    PLAN9P1_OPEN_DIR,
} Plan9P1OpenKind;

typedef struct Plan9P1Fid {
    struct Plan9P1Server *server;
    uint16_t fid;
    V9fsPath path;
    V9fsPath parent_path;
    V9fsFidOpenState fs;
    Plan9P1Qid qid;
    Plan9P1OpenKind open_kind;
    uint8_t access;
    bool remove_on_clunk;
    bool append_only;
    GByteArray *dir_cache;
    GArray *dir_qids;
    GPtrArray *components;
    uint8_t name[PLAN9P1_NAMELEN];
    /* Plan 9 logical identity only; never translated to a host uid. */
    uint8_t uname[PLAN9P1_NAMELEN];
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
    PLAN9P1_BACKEND_OPEN2,
    PLAN9P1_BACKEND_OPENDIR,
    PLAN9P1_BACKEND_PREADV,
    PLAN9P1_BACKEND_PWRITEV,
    PLAN9P1_BACKEND_FSTAT,
    PLAN9P1_BACKEND_MKDIR,
    PLAN9P1_BACKEND_CHMOD,
    PLAN9P1_BACKEND_CHOWN,
    PLAN9P1_BACKEND_UTIMENSAT,
    PLAN9P1_BACKEND_RENAMEAT,
    PLAN9P1_BACKEND_UNLINKAT,
    PLAN9P1_BACKEND_REMOVE,
    PLAN9P1_BACKEND_REWINDDIR,
    PLAN9P1_BACKEND_READDIR,
    PLAN9P1_BACKEND_CLOSE,
    PLAN9P1_BACKEND_CLOSEDIR,
} Plan9P1BackendOp;

#ifdef CONFIG_SLIRP
typedef struct Plan9P1ILAuthConnection {
    QTAILQ_ENTRY(Plan9P1ILAuthConnection) entry;
    Plan9P1Server *server;
    QemuSlirpILConnection *il;
    Plan9AuthTicketConnection *auth;
    bool linked;
    bool migration_counted;
    bool close_requested;
} Plan9P1ILAuthConnection;
#endif

typedef struct Plan9P1BackendWork {
    Plan9P1BackendOp op;
    V9fsBackend *backend;
    V9fsPath *dirpath;
    V9fsPath *newdirpath;
    V9fsPath *path;
    const char *name;
    const char *newname;
    struct stat *st;
    V9fsFidOpenState *fs;
    struct iovec *iov;
    int iovcnt;
    int flags;
    off_t offset;
    FsCred cred;
    struct timespec times[2];
    bool append;
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
    GHashTable *append_qids;
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
    bool auth_configured;
    bool auth_session_valid;
    const Plan9AuthKeydb *auth_keydb;
    char auth_id[PLAN9_AUTH_NAMELEN];
    char auth_domain[PLAN9_AUTH_DOMLEN];
    Plan9AuthRandomBytes auth_random_bytes;
    void *auth_random_opaque;
    Plan9AuthNowSeconds auth_now_seconds;
    void *auth_now_opaque;
    uint8_t auth_client_challenge[PLAN9_AUTH_CHALLENGE_LEN];
    uint8_t auth_server_challenge[PLAN9_AUTH_CHALLENGE_LEN];
    Plan9P1ReplayState auth_replay;
#ifdef CONFIG_SLIRP
    QemuSlirpGuestFwd *guestfwd;
    QemuSlirpPlan9BootpLease *bootp_lease;
    QemuSlirpILListener *auth_listener;
    QemuSlirpILListener *file_listener;
    QemuSlirpILConnection *file_connection;
    QTAILQ_HEAD(, Plan9P1ILAuthConnection) auth_connections;
    Plan9AuthTicketService *ticket_service;
    Plan9AuthKeydb *owned_keydb;
    Error *migration_blocker;
    unsigned int il_connections;
    bool reset_registered;
    bool il_accepting;
    bool il_reset_draining;
    bool file_close_requested;
#endif
    char *fsdev_id;
    char *netdev_id;
    char *guest_address;
    char *keydb_path;
    char *key_secret_id;
    uint16_t port;
    uint16_t il_port;
    uint16_t auth_port;
    Plan9P1ServerTransport qom_transport;
    ResettableState reset_state;
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
static void invalidate_dir_caches(Plan9P1Server *server);
static void server_auth_session_clear(Plan9P1Server *server);
#ifdef CONFIG_SLIRP
static void plan9p1_server_slirp_cleanup(Plan9P1Server *server);
#endif

bool plan9p1_replay_accept(Plan9P1ReplayState *state, uint32_t id)
{
    uint32_t delta;
    uint32_t bit;

    if (!state) {
        return false;
    }
    /* Unsigned subtraction deliberately gives the historical natural wrap. */
    delta = id - state->low;
    if (delta > 31) {
        return false;
    }
    bit = UINT32_C(1) << delta;
    if (state->used & bit) {
        return false;
    }
    state->used |= bit;
    while (state->used & UINT32_C(0xffff0001)) {
        state->used >>= 1;
        state->low++;
    }
    return true;
}

static void server_auth_session_clear(Plan9P1Server *server)
{
    plan9_auth_clear(server->auth_client_challenge,
                     sizeof(server->auth_client_challenge));
    plan9_auth_clear(server->auth_server_challenge,
                     sizeof(server->auth_server_challenge));
    plan9_auth_clear(&server->auth_replay, sizeof(server->auth_replay));
    server->auth_session_valid = false;
}

static int server_auth_random(Plan9P1Server *server, void *buf, size_t len,
                              Error **errp)
{
    if (server->auth_random_bytes) {
        return server->auth_random_bytes(buf, len,
                                         server->auth_random_opaque, errp);
    }
    return qcrypto_random_bytes(buf, len, errp);
}

static int server_auth_now(Plan9P1Server *server, uint32_t *seconds,
                           Error **errp)
{
    time_t now;

    if (server->auth_now_seconds) {
        *seconds = server->auth_now_seconds(server->auth_now_opaque);
        return 0;
    }
    now = time(NULL);
    if (now == (time_t)-1) {
        error_setg(errp, "cannot read time for Plan 9 file authentication");
        return -1;
    }
    *seconds = MIN((uint64_t)now, UINT32_MAX);
    return 0;
}

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
        if (!ops->open) {
            return -EOPNOTSUPP;
        }
        ret = ops->open(ctx, work->path, work->flags, work->fs);
        break;
    case PLAN9P1_BACKEND_OPEN2:
        if (!ops->open2) {
            return -EOPNOTSUPP;
        }
        ret = ops->open2(ctx, work->dirpath, work->name, work->flags,
                         &work->cred, work->fs);
        break;
    case PLAN9P1_BACKEND_OPENDIR:
        ret = ops->opendir(ctx, work->path, work->fs);
        break;
    case PLAN9P1_BACKEND_PREADV:
        if (!ops->preadv) {
            return -EOPNOTSUPP;
        }
        ret = ops->preadv(ctx, work->fs, work->iov, work->iovcnt,
                          work->offset);
        break;
    case PLAN9P1_BACKEND_PWRITEV:
        if (!ops->pwritev) {
            return -EOPNOTSUPP;
        }
        if (work->append) {
            if (!ops->fstat) {
                return -EOPNOTSUPP;
            }
            ret = ops->fstat(ctx, P9_FID_FILE, work->fs, work->st);
            if (ret < 0) {
                break;
            }
            work->offset = work->st->st_size;
        }
        ret = ops->pwritev(ctx, work->fs, work->iov, work->iovcnt,
                           work->offset);
        break;
    case PLAN9P1_BACKEND_FSTAT:
        if (!ops->fstat) {
            return -EOPNOTSUPP;
        }
        ret = ops->fstat(ctx, P9_FID_FILE, work->fs, work->st);
        break;
    case PLAN9P1_BACKEND_MKDIR:
        if (!ops->mkdir) {
            return -EOPNOTSUPP;
        }
        ret = ops->mkdir(ctx, work->dirpath, work->name, &work->cred);
        break;
    case PLAN9P1_BACKEND_CHMOD:
        if (!ops->chmod) {
            return -EOPNOTSUPP;
        }
        ret = ops->chmod(ctx, work->path, &work->cred);
        break;
    case PLAN9P1_BACKEND_CHOWN:
        if (!ops->chown) {
            return -EOPNOTSUPP;
        }
        ret = ops->chown(ctx, work->path, &work->cred);
        break;
    case PLAN9P1_BACKEND_UTIMENSAT:
        if (!ops->utimensat) {
            return -EOPNOTSUPP;
        }
        ret = ops->utimensat(ctx, work->path, work->times);
        break;
    case PLAN9P1_BACKEND_RENAMEAT:
        if (!ops->renameat) {
            return -EOPNOTSUPP;
        }
        ret = ops->renameat(ctx, work->dirpath, work->name,
                            work->newdirpath, work->newname);
        break;
    case PLAN9P1_BACKEND_UNLINKAT:
        if (!ops->unlinkat) {
            return -EOPNOTSUPP;
        }
        ret = ops->unlinkat(ctx, work->dirpath, work->name, work->flags);
        break;
    case PLAN9P1_BACKEND_REMOVE:
        if (!ops->remove) {
            return -EOPNOTSUPP;
        }
        ret = ops->remove(ctx, work->path->data);
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

static int coroutine_fn co_open2(Plan9P1Server *server, V9fsPath *dirpath,
                                 const char *name, int flags, mode_t mode,
                                 gid_t gid, V9fsFidOpenState *fs)
{
    V9fsPath dircopy;
    V9fsFidOpenState candidate = { 0 };
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_OPEN2,
        .backend = server->backend,
        .name = name,
        .flags = flags,
        .fs = &candidate,
        .cred = {
            .fc_uid = -1,
            .fc_gid = gid,
            .fc_mode = mode,
            .fc_rdev = -1,
        },
    };
    int ret;

    path_init(&dircopy);
    path_copy(&dircopy, dirpath);
    work.dirpath = &dircopy;
    ret = run_backend(&work);
    path_free(&dircopy);
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

static int coroutine_fn co_pwritev(Plan9P1Server *server,
                                   V9fsFidOpenState *fs,
                                   struct iovec *iov, int iovcnt,
                                   off_t offset, bool append)
{
    V9fsFidOpenState candidate = *fs;
    struct stat st;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_PWRITEV,
        .backend = server->backend,
        .fs = &candidate,
        .iov = iov,
        .iovcnt = iovcnt,
        .offset = offset,
        .append = append,
        .st = &st,
    };

    return run_backend(&work);
}

static int coroutine_fn co_mkdir(Plan9P1Server *server, V9fsPath *dirpath,
                                 const char *name, mode_t mode, gid_t gid)
{
    V9fsPath dircopy;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_MKDIR,
        .backend = server->backend,
        .name = name,
        .cred = {
            .fc_uid = -1,
            .fc_gid = gid,
            .fc_mode = mode,
            .fc_rdev = -1,
        },
    };
    int ret;

    path_init(&dircopy);
    path_copy(&dircopy, dirpath);
    work.dirpath = &dircopy;
    ret = run_backend(&work);
    path_free(&dircopy);
    return ret;
}

static int coroutine_fn co_chmod(Plan9P1Server *server, V9fsPath *path,
                                 mode_t mode)
{
    V9fsPath copy;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_CHMOD,
        .backend = server->backend,
        .cred = {
            .fc_uid = -1,
            .fc_gid = -1,
            .fc_mode = mode,
            .fc_rdev = -1,
        },
    };
    int ret;

    path_init(&copy);
    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_free(&copy);
    return ret;
}

static int coroutine_fn co_chown_gid(Plan9P1Server *server, V9fsPath *path,
                                     gid_t gid)
{
    V9fsPath copy;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_CHOWN,
        .backend = server->backend,
        .cred = {
            .fc_uid = -1,
            .fc_gid = gid,
            .fc_mode = -1,
            .fc_rdev = -1,
        },
    };
    int ret;

    path_init(&copy);
    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_free(&copy);
    return ret;
}

static int coroutine_fn co_utimensat(Plan9P1Server *server, V9fsPath *path,
                                     uint32_t mtime)
{
    V9fsPath copy;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_UTIMENSAT,
        .backend = server->backend,
        .times = {
            { .tv_nsec = UTIME_OMIT },
            { .tv_sec = mtime, .tv_nsec = 0 },
        },
    };
    int ret;

    path_init(&copy);
    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_free(&copy);
    return ret;
}

static int coroutine_fn co_renameat(Plan9P1Server *server,
                                    V9fsPath *dirpath, const char *oldname,
                                    const char *newname)
{
    V9fsPath olddir;
    V9fsPath newdir;
    Plan9P1BackendWork work = {
        .op = PLAN9P1_BACKEND_RENAMEAT,
        .backend = server->backend,
        .name = oldname,
        .newname = newname,
    };
    int ret;

    path_init(&olddir);
    path_init(&newdir);
    path_copy(&olddir, dirpath);
    path_copy(&newdir, dirpath);
    work.dirpath = &olddir;
    work.newdirpath = &newdir;
    ret = run_backend(&work);
    path_free(&olddir);
    path_free(&newdir);
    return ret;
}

static int coroutine_fn co_unlink(Plan9P1Server *server,
                                  V9fsPath *dirpath, const char *name,
                                  V9fsPath *path, bool directory)
{
    V9fsPath dircopy;
    V9fsPath pathcopy;
    V9fsPath resolved;
    V9fsPath *remove_path = path;
    Plan9P1BackendWork work = {
        .op = server->backend->ops->unlinkat ?
              PLAN9P1_BACKEND_UNLINKAT : PLAN9P1_BACKEND_REMOVE,
        .backend = server->backend,
        .name = name,
        .flags = directory ? AT_REMOVEDIR : 0,
    };
    int ret;

    path_init(&dircopy);
    path_init(&pathcopy);
    path_init(&resolved);
    if (!server->backend->ops->unlinkat && (!path || !path->data)) {
        ret = co_name_to_path(server, dirpath, name, &resolved);
        if (ret < 0) {
            goto out;
        }
        remove_path = &resolved;
    }
    path_copy(&dircopy, dirpath);
    path_copy(&pathcopy, remove_path);
    work.dirpath = &dircopy;
    work.path = &pathcopy;
    ret = run_backend(&work);
out:
    path_free(&dircopy);
    path_free(&pathcopy);
    path_free(&resolved);
    return ret;
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
    path_free(&fid->parent_path);
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
    plan9_auth_clear(fid, sizeof(*fid));
    g_free(fid);
}

static void qid_identity_free(gpointer opaque)
{
    g_free(opaque);
}

static void request_free(Plan9P1Request *request)
{
    if (request->data) {
        plan9_auth_clear(request->data, request->tx.count);
    }
    g_free(request->data);
    plan9_auth_clear(request, sizeof(*request));
    g_free(request);
}

static void reply_free(Plan9P1Reply *reply)
{
    plan9_auth_clear(reply->data, reply->len);
    g_free(reply->data);
    plan9_auth_clear(reply, sizeof(*reply));
    g_free(reply);
}

static void deferred_input_free(Plan9P1DeferredInput *input)
{
    plan9_auth_clear(input->data, input->len);
    g_free(input->data);
    plan9_auth_clear(input, sizeof(*input));
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
    plan9p1_server_slirp_cleanup(server);
    if (server->reset_registered) {
        qemu_unregister_resettable(obj);
        server->reset_registered = false;
    }
#endif
    server->closing = true;
    server_auth_session_clear(server);
    g_hash_table_unref(server->fids);
    g_hash_table_unref(server->qid_paths);
    g_hash_table_unref(server->append_qids);
    v9fs_backend_cleanup(&server->backend_storage);
    server->backend = NULL;
    g_free(server->fsdev_id);
    g_free(server->netdev_id);
    g_free(server->guest_address);
    g_free(server->keydb_path);
    g_free(server->key_secret_id);
    plan9_auth_clear(server->auth_id, sizeof(server->auth_id));
    plan9_auth_clear(server->auth_domain, sizeof(server->auth_domain));
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
    g_hash_table_remove_all(server->append_qids);
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
    fid->access = PLAN9P1_OREAD;
    return ret;
}

static int coroutine_fn remove_fid_path(Plan9P1Server *server,
                                        Plan9P1Fid *fid)
{
    const char *name;
    int ret;

    if (fid->components->len == 0) {
        return -EPERM;
    }
    name = fid->components->pdata[fid->components->len - 1];
    ret = co_unlink(server, &fid->parent_path, name, &fid->path,
                    fid->qid.path & PLAN9P1_DMDIR);
    if (ret >= 0) {
        invalidate_dir_caches(server);
    }
    return ret;
}

static void coroutine_fn cleanup_fids(Plan9P1Server *server)
{
    GList *fids = g_hash_table_get_values(server->fids);
    GList *link;

    for (link = fids; link; link = link->next) {
        Plan9P1Fid *fid = link->data;
        bool remove = fid->remove_on_clunk;

        fid->remove_on_clunk = false;
        close_fid(server, fid);
        if (remove) {
            remove_fid_path(server, fid);
        }
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
        .first_edition = request->tx.first_edition,
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
    path_init(&fid->parent_path);
    path_copy(&fid->path, path);
    path_copy(&fid->parent_path, path);
    fid->qid = *qid;
    fid->components = g_ptr_array_new_with_free_func(g_free);
    if (name[0] && strcmp(name, "/")) {
        g_ptr_array_add(fid->components, g_strdup(name));
    }
    fixed_string(fid->name, name);
    return fid;
}

static int coroutine_fn resolve_components(Plan9P1Server *server,
                                           GPtrArray *components,
                                           unsigned int count,
                                           V9fsPath *result)
{
    V9fsPath current;
    V9fsPath next;
    unsigned int i;
    int ret;

    path_init(&current);
    path_init(&next);
    ret = co_name_to_path(server, NULL, "/", &current);
    for (i = 0; ret >= 0 && i < count; i++) {
        ret = co_name_to_path(server, &current, components->pdata[i], &next);
        if (ret >= 0) {
            path_copy(&current, &next);
        }
        path_free(&next);
    }
    if (ret >= 0) {
        path_copy(result, &current);
    }
    path_free(&current);
    return ret;
}

static int server_auth_session_begin(Plan9P1Server *server,
                                     const Plan9P1Fcall *request,
                                     Plan9P1Fcall *reply)
{
    Error *local_err = NULL;

    server_auth_session_clear(server);
    memcpy(server->auth_client_challenge, request->challenge,
           sizeof(server->auth_client_challenge));
    if (server_auth_random(server, server->auth_server_challenge,
                           sizeof(server->auth_server_challenge),
                           &local_err)) {
        error_free(local_err);
        server_auth_session_clear(server);
        return -EACCES;
    }
    memcpy(reply->challenge, server->auth_server_challenge,
           sizeof(reply->challenge));
    memcpy(reply->authid, server->auth_id, sizeof(reply->authid));
    memcpy(reply->authdom, server->auth_domain, sizeof(reply->authdom));
    server->auth_session_valid = true;
    return 0;
}

static int server_authenticate_attach(Plan9P1Request *request,
                                      Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    uint8_t server_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN] = { 0 };
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN] = { 0 };
    uint8_t canonical_cuid[PLAN9P1_NAMELEN] = { 0 };
    Plan9AuthTicket ticket = { 0 };
    Plan9AuthAuthenticator authenticator = { 0 };
    Error *local_err = NULL;
    uint32_t now;
    int ret = -EACCES;

    if (!server->auth_session_valid ||
        server_auth_now(server, &now, &local_err) ||
        plan9_auth_keydb_lookup(server->auth_keydb, server->auth_id, now,
                                server_key) != PLAN9_AUTH_KEY_AVAILABLE) {
        goto out;
    }

    memcpy(ticket_wire, request->tx.ticket, sizeof(ticket_wire));
    if (plan9_auth_decrypt(server_key, ticket_wire, sizeof(ticket_wire),
                           &local_err) ||
        plan9_auth_ticket_decode(ticket_wire, sizeof(ticket_wire), &ticket,
                                 &local_err) ||
        ticket.num != PLAN9_AUTH_TS ||
        memcmp(ticket.challenge, server->auth_server_challenge,
               sizeof(ticket.challenge))) {
        goto out;
    }

    memcpy(auth_wire, request->tx.auth, sizeof(auth_wire));
    if (plan9_auth_decrypt(ticket.key, auth_wire, sizeof(auth_wire),
                           &local_err) ||
        plan9_auth_authenticator_decode(auth_wire, sizeof(auth_wire),
                                        &authenticator, &local_err) ||
        authenticator.num != PLAN9_AUTH_AC ||
        memcmp(authenticator.challenge, server->auth_server_challenge,
               sizeof(authenticator.challenge))) {
        goto out;
    }

    /* Preserve the historical order: a bad uname still burns this ID. */
    if (!plan9p1_replay_accept(&server->auth_replay, authenticator.id)) {
        goto out;
    }
    fixed_string(canonical_cuid, ticket.cuid);
    if (memcmp(request->tx.uname, canonical_cuid,
               sizeof(request->tx.uname))) {
        goto out;
    }

    /* This is a Plan 9 logical identity, never a host uid impersonation. */
    fixed_string(request->tx.uname, ticket.suid);
    authenticator.num = PLAN9_AUTH_AS;
    memcpy(authenticator.challenge, server->auth_client_challenge,
           sizeof(authenticator.challenge));
    if (plan9_auth_authenticator_encode(&authenticator, auth_wire,
                                        &local_err) ||
        plan9_auth_encrypt(ticket.key, auth_wire, sizeof(auth_wire),
                           &local_err)) {
        goto out;
    }
    memcpy(reply->auth, auth_wire, sizeof(reply->auth));
    ret = 0;

out:
    error_free(local_err);
    plan9_auth_clear(server_key, sizeof(server_key));
    plan9_auth_clear(ticket_wire, sizeof(ticket_wire));
    plan9_auth_clear(auth_wire, sizeof(auth_wire));
    plan9_auth_clear(canonical_cuid, sizeof(canonical_cuid));
    plan9_auth_ticket_clear(&ticket);
    plan9_auth_clear(&authenticator, sizeof(authenticator));
    if (ret) {
        plan9_auth_clear(reply->auth, sizeof(reply->auth));
    }
    return ret;
}

/*
 * First Edition's built-in boot identity has an all-zero key.  The native
 * file server permits that identity to attach without a configured auth
 * service, but mount -a still sends Tauth and insists on a syntactically
 * valid ticket reply before it continues with the attach.
 */
static int server_first_edition_none_auth(Plan9P1Request *request,
                                          Plan9P1Fcall *reply)
{
    static const uint8_t none_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    Plan9P1Server *server = request->server;
    uint8_t canonical_none[PLAN9P1_NAMELEN] = { 0 };
    uint8_t challenge[PLAN9P1_1E_AUTHCHALLEN] = { 0 };
    uint8_t ticket[15] = { 0 };
    uint8_t response[PLAN9P1_1E_AUTHREPLYLEN] = { 0 };
    Error *local_err = NULL;
    int ret = -EACCES;

    fixed_string(canonical_none, "none");
    if (server->auth_configured ||
        memcmp(request->tx.uname, canonical_none, sizeof(canonical_none))) {
        goto out;
    }
    memcpy(challenge, request->tx.first_edition_challenge,
           sizeof(challenge));
    if (plan9_auth_decrypt(none_key, challenge, sizeof(challenge),
                           &local_err) ||
        challenge[0] != 1) { /* FScchal */
        goto out;
    }

    response[0] = 4; /* FSctick */
    memcpy(response + 1, challenge + 1, 7);
    if (server_auth_random(server, response + 8, 7, &local_err) ||
        server_auth_random(server, ticket + 1, 7, &local_err)) {
        goto out;
    }
    ticket[0] = 5; /* FSstick */
    memcpy(ticket + 8, response + 8, 7);
    if (plan9_auth_encrypt(none_key, ticket, sizeof(ticket), &local_err)) {
        goto out;
    }
    memcpy(response + 15, ticket, sizeof(ticket));
    if (plan9_auth_encrypt(none_key, response, sizeof(response),
                           &local_err)) {
        goto out;
    }
    reply->type = PLAN9P1_RAUTH;
    reply->fid = request->tx.fid;
    memcpy(reply->first_edition_reply, response, sizeof(response));
    ret = 0;

out:
    error_free(local_err);
    plan9_auth_clear(challenge, sizeof(challenge));
    plan9_auth_clear(ticket, sizeof(ticket));
    plan9_auth_clear(response, sizeof(response));
    plan9_auth_clear(canonical_none, sizeof(canonical_none));
    if (ret) {
        plan9_auth_clear(reply->first_edition_reply,
                         sizeof(reply->first_edition_reply));
    }
    return ret;
}

static int coroutine_fn attach_fid(Plan9P1Request *request,
                                   Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid;
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
    fid = new_fid(server, request->tx.fid, &path, &qid, "/");
    memcpy(fid->uname, request->tx.uname, sizeof(fid->uname));
    g_hash_table_insert(server->fids, fid_key(request->tx.fid), fid);
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
    path_copy(&clone->parent_path, &source->parent_path);
    clone->append_only = source->append_only;
    memcpy(clone->name, source->name, sizeof(clone->name));
    memcpy(clone->uname, source->uname, sizeof(clone->uname));
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
    V9fsPath parent;
    struct stat st;
    bool qid_acquired = false;
    int ret;

    if (!name) {
        return -errno;
    }
    if (!fid || fid->open_kind != PLAN9P1_OPEN_NONE) {
        return fid ? -EBUSY : -ENOENT;
    }
    path_init(&path);
    path_init(&parent);
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
    qid_acquired = true;
    if (!strcmp(name, "..")) {
        ret = resolve_components(server, fid->components,
                                 fid->components->len > 1 ?
                                 fid->components->len - 2 : 0, &parent);
        if (ret < 0) {
            goto out;
        }
    } else if (strcmp(name, ".")) {
        path_copy(&parent, &fid->path);
    } else {
        path_copy(&parent, &fid->parent_path);
    }
    if (!strcmp(name, "..") && fid->components->len) {
        g_ptr_array_remove_index(fid->components,
                                 fid->components->len - 1);
    } else if (strcmp(name, ".") && strcmp(name, "..")) {
        g_ptr_array_add(fid->components, g_strdup(name));
    }
    path_copy(&fid->path, &path);
    path_copy(&fid->parent_path, &parent);
    qid_release(server, fid->qid.path);
    fid->qid = *qid;
    qid_commit(server, qid->path);
    qid_acquired = false;
    fid->append_only = g_hash_table_contains(
        server->append_qids, GUINT_TO_POINTER(qid->path));
    if (fid->components->len) {
        fixed_string(fid->name,
                     g_ptr_array_index(fid->components,
                                       fid->components->len - 1));
    } else {
        fixed_string(fid->name, "/");
    }
out:
    if (qid_acquired) {
        qid_release(server, qid->path);
    }
    path_free(&path);
    path_free(&parent);
    return ret;
}

static bool execute_permitted(const struct stat *st)
{
    uid_t uid = geteuid();
    gid_t gid = getegid();
    int count;
    g_autofree gid_t *groups = NULL;

    if (uid == 0) {
        return st->st_mode & 0111;
    }
    if (uid == st->st_uid) {
        return st->st_mode & S_IXUSR;
    }
    if (gid == st->st_gid) {
        return st->st_mode & S_IXGRP;
    }
    count = getgroups(0, NULL);
    if (count > 0) {
        groups = g_new(gid_t, count);
        count = getgroups(count, groups);
        for (int i = 0; i < count; i++) {
            if (groups[i] == st->st_gid) {
                return st->st_mode & S_IXGRP;
            }
        }
    }
    return st->st_mode & S_IXOTH;
}

static int coroutine_fn open_fid(Plan9P1Request *request,
                                 Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    struct stat st;
    unsigned int mode = request->tx.mode;
    unsigned int access = mode & 3;
    bool truncate = false;
    int flags;
    int ret;

    if (!fid) {
        return -ENOENT;
    }
    if (fid->open_kind != PLAN9P1_OPEN_NONE) {
        return -EBUSY;
    }
    if (mode & ~PLAN9P1_OPEN_MASK) {
        return -EINVAL;
    }
    if ((mode & (PLAN9P1_OTRUNC | 3)) ==
        (PLAN9P1_OTRUNC | PLAN9P1_OEXEC)) {
        return -EINVAL;
    }
    if ((server->backend->ctx.export_flags & V9FS_RDONLY) &&
        (access == PLAN9P1_OWRITE || access == PLAN9P1_ORDWR ||
         (mode & (PLAN9P1_OTRUNC | PLAN9P1_ORCLOSE)))) {
        return -EROFS;
    }
    if ((mode & PLAN9P1_ORCLOSE) && fid->components->len == 0) {
        return -EPERM;
    }
    ret = stat_path(server, &fid->path, &st);
    if (ret < 0) {
        return ret;
    }
    if (S_ISDIR(st.st_mode)) {
        if (mode != PLAN9P1_OREAD) {
            return -EISDIR;
        }
        ret = co_opendir(server, &fid->path, &fid->fs);
        if (ret < 0) {
            return ret;
        }
        fid->open_kind = PLAN9P1_OPEN_DIR;
    } else {
        if (access == PLAN9P1_OEXEC && !execute_permitted(&st)) {
            return -EACCES;
        }
        flags = access == PLAN9P1_OWRITE ? O_WRONLY :
                access == PLAN9P1_ORDWR ? O_RDWR : O_RDONLY;
        fid->append_only = g_hash_table_contains(
            server->append_qids, GUINT_TO_POINTER(fid->qid.path));
        if ((mode & PLAN9P1_OTRUNC) && !fid->append_only) {
            flags |= O_TRUNC;
            truncate = true;
        }
        ret = co_open(server, &fid->path, flags, &fid->fs);
        if (ret < 0) {
            return ret;
        }
        fid->open_kind = PLAN9P1_OPEN_FILE;
        if (truncate) {
            invalidate_dir_caches(server);
        }
    }
    fid->access = access;
    fid->remove_on_clunk = mode & PLAN9P1_ORCLOSE;
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
    if (g_hash_table_contains(server->append_qids,
                              GUINT_TO_POINTER(stat_reply.dir.qid.path))) {
        stat_reply.dir.mode |= PLAN9P1_DMAPPEND;
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
    if (fid->open_kind == PLAN9P1_OPEN_FILE &&
        fid->access != PLAN9P1_OREAD && fid->access != PLAN9P1_ORDWR &&
        fid->access != PLAN9P1_OEXEC) {
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

static int create_open_flags(unsigned int access)
{
    switch (access) {
    case PLAN9P1_OREAD:
    case PLAN9P1_OEXEC:
        return O_RDONLY;
    case PLAN9P1_OWRITE:
        return O_WRONLY;
    case PLAN9P1_ORDWR:
        return O_RDWR;
    default:
        return -1;
    }
}

static int coroutine_fn create_fid(Plan9P1Request *request,
                                   Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    g_autofree char *name = decode_name(request->tx.name);
    V9fsPath path;
    V9fsPath parent;
    V9fsFidOpenState candidate = { 0 };
    struct stat parent_st;
    struct stat st;
    Plan9P1Qid qid = { 0 };
    unsigned int access = request->tx.mode & 3;
    uint32_t perm = request->tx.perm;
    mode_t host_mode;
    int flags;
    int ret;
    int cleanup_ret;
    bool directory;
    bool created = false;
    bool opened = false;

    path_init(&path);
    path_init(&parent);
    if (!name) {
        return -errno;
    }
    if (!strcmp(name, ".") || !strcmp(name, "..")) {
        return -EINVAL;
    }
    if (!fid) {
        return -ENOENT;
    }
    if (fid->open_kind != PLAN9P1_OPEN_NONE) {
        return -EBUSY;
    }
    if (request->tx.mode & ~PLAN9P1_OPEN_MASK) {
        return -EINVAL;
    }
    if (server->backend->ctx.export_flags & V9FS_RDONLY) {
        return -EROFS;
    }
    if (perm & ~(PLAN9P1_DMDIR | PLAN9P1_DMAPPEND |
                 PLAN9P1_DMLOCK | UINT32_C(0777))) {
        return -EINVAL;
    }
    if (perm & PLAN9P1_DMLOCK) {
        return -EOPNOTSUPP;
    }
    ret = stat_path(server, &fid->path, &parent_st);
    if (ret < 0) {
        return ret;
    }
    if (!S_ISDIR(parent_st.st_mode)) {
        return -ENOTDIR;
    }
    directory = perm & PLAN9P1_DMDIR;
    if (directory && access != PLAN9P1_OREAD) {
        return -EISDIR;
    }
    path_copy(&parent, &fid->path);
    if (directory) {
        host_mode = (perm & 0777) & (parent_st.st_mode & 0777);
        ret = co_mkdir(server, &parent, name, host_mode, parent_st.st_gid);
        if (ret < 0) {
            goto out;
        }
        created = true;
        ret = co_name_to_path(server, &parent, name, &path);
        if (ret >= 0) {
            ret = stat_path(server, &path, &st);
        }
        if (ret >= 0) {
            ret = co_opendir(server, &path, &candidate);
            opened = ret >= 0;
        }
    } else {
        host_mode = ((perm & 0666) & (parent_st.st_mode & 0666)) |
                    (perm & 0111);
        if (access == PLAN9P1_OEXEC && !(host_mode & S_IXUSR)) {
            ret = -EACCES;
            goto out;
        }
        flags = create_open_flags(access) | O_CREAT | O_EXCL;
        ret = co_open2(server, &parent, name, flags, host_mode,
                       parent_st.st_gid, &candidate);
        if (ret < 0) {
            goto out;
        }
        created = true;
        opened = true;
        ret = co_name_to_path(server, &parent, name, &path);
        if (ret >= 0) {
            ret = stat_path(server, &path, &st);
        }
    }
    if (ret >= 0) {
        ret = qid_from_stat(server, &st, &qid);
    }
    if (ret < 0) {
        goto out;
    }

    path_copy(&fid->parent_path, &parent);
    path_copy(&fid->path, &path);
    qid_release(server, fid->qid.path);
    fid->qid = qid;
    qid_commit(server, qid.path);
    g_ptr_array_add(fid->components, g_strdup(name));
    fixed_string(fid->name, name);
    fid->fs = candidate;
    fid->open_kind = directory ? PLAN9P1_OPEN_DIR : PLAN9P1_OPEN_FILE;
    fid->access = access;
    fid->remove_on_clunk = request->tx.mode & PLAN9P1_ORCLOSE;
    fid->append_only = perm & PLAN9P1_DMAPPEND;
    if (fid->append_only) {
        g_hash_table_add(server->append_qids, GUINT_TO_POINTER(qid.path));
    }
    invalidate_dir_caches(server);
    opened = false;
    created = false;
    reply->type = PLAN9P1_RCREATE;
    reply->fid = fid->fid;
    reply->qid = fid->qid;
out:
    if (ret < 0 && opened) {
        cleanup_ret = co_close(server, &candidate, directory);
        opened = false;
        if (cleanup_ret < 0) {
            warn_report("9pfs: 9P1 create rollback close failed: %s",
                        g_strerror(-cleanup_ret));
        }
    }
    if (ret < 0 && created) {
        invalidate_dir_caches(server);
        cleanup_ret = co_unlink(server, &parent, name, &path, directory);
        created = false;
        if (cleanup_ret < 0) {
            warn_report("9pfs: 9P1 create rollback remove failed: %s",
                        g_strerror(-cleanup_ret));
        }
    }
    if (ret < 0 && qid.path) {
        qid_release(server, qid.path);
    }
    path_free(&path);
    path_free(&parent);
    return ret;
}

static int coroutine_fn write_fid(Plan9P1Request *request,
                                  Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    struct iovec iov;
    ssize_t ret;

    if (server->backend->ctx.export_flags & V9FS_RDONLY) {
        return -EROFS;
    }
    if (!fid) {
        return -ENOENT;
    }
    if (fid->open_kind != PLAN9P1_OPEN_FILE ||
        (fid->access != PLAN9P1_OWRITE && fid->access != PLAN9P1_ORDWR)) {
        return -EBADF;
    }
    if (request->tx.count > PLAN9P1_MAX_DATA ||
        (request->tx.count && !request->tx.data)) {
        return -EINVAL;
    }
    if (!fid->append_only &&
        (request->tx.offset > INT64_MAX ||
         request->tx.offset > INT64_MAX - request->tx.count ||
         (uint64_t)(off_t)request->tx.offset != request->tx.offset)) {
        return -EOVERFLOW;
    }
    reply->type = PLAN9P1_RWRITE;
    reply->fid = fid->fid;
    if (!request->tx.count) {
        reply->count = 0;
        return 0;
    }
    iov = (struct iovec) {
        .iov_base = (void *)request->tx.data,
        .iov_len = request->tx.count,
    };
    fsdev_co_throttle_request(server->backend->ctx.fst, THROTTLE_WRITE,
                              &iov, 1);
    if (request->cancelled) {
        return -EINTR;
    }
    ret = co_pwritev(server, &fid->fs, &iov, 1,
                     (off_t)request->tx.offset, fid->append_only);
    if (ret < 0) {
        return ret;
    }
    if (ret > 0) {
        invalidate_dir_caches(server);
    }
    if ((uint64_t)ret > request->tx.count) {
        return -EIO;
    }
    reply->count = ret;
    return 0;
}

typedef struct Plan9P1RenameCandidate {
    Plan9P1Fid *fid;
    V9fsPath path;
    V9fsPath parent;
} Plan9P1RenameCandidate;

static void rename_candidate_free(gpointer opaque)
{
    Plan9P1RenameCandidate *candidate = opaque;

    path_free(&candidate->path);
    path_free(&candidate->parent);
    g_free(candidate);
}

static bool components_have_prefix(GPtrArray *components,
                                   GPtrArray *prefix)
{
    unsigned int i;

    if (components->len < prefix->len) {
        return false;
    }
    for (i = 0; i < prefix->len; i++) {
        if (strcmp(components->pdata[i], prefix->pdata[i])) {
            return false;
        }
    }
    return true;
}

static int coroutine_fn resolve_renamed_components(Plan9P1Server *server,
                                                   Plan9P1Fid *fid,
                                                   unsigned int index,
                                                   const char *newname,
                                                   unsigned int count,
                                                   V9fsPath *result)
{
    V9fsPath current;
    V9fsPath next;
    unsigned int i;
    int ret;

    path_init(&current);
    path_init(&next);
    ret = co_name_to_path(server, NULL, "/", &current);
    for (i = 0; ret >= 0 && i < count; i++) {
        const char *component = i == index ?
            newname : fid->components->pdata[i];

        ret = co_name_to_path(server, &current, component, &next);
        if (ret >= 0) {
            path_copy(&current, &next);
        }
        path_free(&next);
    }
    if (ret >= 0) {
        path_copy(result, &current);
    }
    path_free(&current);
    return ret;
}

static void invalidate_dir_caches(Plan9P1Server *server)
{
    GList *values = g_hash_table_get_values(server->fids);
    GList *link;

    for (link = values; link; link = link->next) {
        Plan9P1Fid *fid = link->data;
        unsigned int i;

        if (fid->dir_cache) {
            assert(server->dir_cache_bytes >= fid->dir_cache->len);
            server->dir_cache_bytes -= fid->dir_cache->len;
            g_byte_array_unref(fid->dir_cache);
            fid->dir_cache = NULL;
        }
        if (fid->dir_qids) {
            for (i = 0; i < fid->dir_qids->len; i++) {
                qid_release(server,
                            g_array_index(fid->dir_qids, uint32_t, i));
            }
            g_array_unref(fid->dir_qids);
            fid->dir_qids = NULL;
        }
    }
    g_list_free(values);
}

static char *fixed_to_string(const uint8_t value[PLAN9P1_NAMELEN])
{
    return g_strndup((const char *)value,
                     strnlen((const char *)value, PLAN9P1_NAMELEN));
}

static int parse_gid(const char *text, gid_t *gid)
{
    char *end;
    uint64_t value;

    if (!text[0] || !g_ascii_isdigit(text[0])) {
        return -EINVAL;
    }
    errno = 0;
    value = g_ascii_strtoull(text, &end, 10);
    if (errno || *end || (uint64_t)(gid_t)value != value) {
        return -EINVAL;
    }
    *gid = value;
    return 0;
}

static int coroutine_fn rename_fids(Plan9P1Server *server,
                                    Plan9P1Fid *target,
                                    const char *newname)
{
    g_autoptr(GPtrArray) candidates =
        g_ptr_array_new_with_free_func(rename_candidate_free);
    GList *values = g_hash_table_get_values(server->fids);
    GList *link;
    unsigned int index = target->components->len - 1;
    int ret = 0;

    for (link = values; link; link = link->next) {
        Plan9P1Fid *fid = link->data;
        Plan9P1RenameCandidate *candidate;

        if (!components_have_prefix(fid->components, target->components)) {
            continue;
        }
        candidate = g_new0(Plan9P1RenameCandidate, 1);
        candidate->fid = fid;
        path_init(&candidate->path);
        path_init(&candidate->parent);
        ret = resolve_renamed_components(server, fid, index, newname,
                                         fid->components->len,
                                         &candidate->path);
        if (ret >= 0) {
            ret = resolve_renamed_components(server, fid, index, newname,
                                             fid->components->len - 1,
                                             &candidate->parent);
        }
        if (ret < 0) {
            rename_candidate_free(candidate);
            break;
        }
        g_ptr_array_add(candidates, candidate);
    }
    g_list_free(values);
    if (ret < 0) {
        return ret;
    }
    ret = co_renameat(server, &target->parent_path,
                      target->components->pdata[index], newname);
    if (ret < 0) {
        return ret;
    }
    for (unsigned int i = 0; i < candidates->len; i++) {
        Plan9P1RenameCandidate *candidate = candidates->pdata[i];

        g_free(candidate->fid->components->pdata[index]);
        candidate->fid->components->pdata[index] = g_strdup(newname);
        path_copy(&candidate->fid->path, &candidate->path);
        path_copy(&candidate->fid->parent_path, &candidate->parent);
        if (candidate->fid->components->len == target->components->len) {
            fixed_string(candidate->fid->name, newname);
        }
    }
    invalidate_dir_caches(server);
    return 0;
}

static int coroutine_fn wstat_fid(Plan9P1Request *request,
                                  Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    g_autofree char *name = fixed_to_string(request->tx.dir.name);
    g_autofree char *uid = fixed_to_string(request->tx.dir.uid);
    g_autofree char *gid_text = fixed_to_string(request->tx.dir.gid);
    char current_uid[PLAN9P1_NAMELEN + 1];
    struct stat st;
    gid_t gid;
    uint32_t requested_mode = request->tx.dir.mode;
    mode_t mode;
    bool append;
    bool change_mode;
    bool change_gid;
    bool change_mtime;
    bool change_name;
    bool change_append;
    bool mutated = false;
    int ret;

    if (server->backend->ctx.export_flags & V9FS_RDONLY) {
        return -EROFS;
    }
    if (!fid) {
        return -ENOENT;
    }
    if (fid->components->len == 0) {
        return -EPERM;
    }
    if (!name[0] || strchr(name, '/') || !strcmp(name, ".") ||
        !strcmp(name, "..")) {
        return -EINVAL;
    }
    if (requested_mode & PLAN9P1_DMLOCK) {
        return -EOPNOTSUPP;
    }
    if (requested_mode & ~(PLAN9P1_DMDIR | PLAN9P1_DMAPPEND |
                           PLAN9P1_DMLOCK | UINT32_C(0777))) {
        return -EINVAL;
    }
    ret = stat_path(server, &fid->path, &st);
    if (ret < 0) {
        return ret;
    }
    snprintf(current_uid, sizeof(current_uid), "%ju", (uintmax_t)st.st_uid);
    if (strcmp(uid, current_uid)) {
        return -EPERM;
    }
    ret = parse_gid(gid_text, &gid);
    if (ret < 0) {
        return ret;
    }
    mode = requested_mode & 0777;
    append = requested_mode & PLAN9P1_DMAPPEND;
    change_mode = mode != (st.st_mode & 0777);
    change_gid = gid != st.st_gid;
    change_mtime = request->tx.dir.mtime != stat_mtime(&st);
    change_name = strncmp(name, (char *)fid->name, PLAN9P1_NAMELEN);
    change_append = append != g_hash_table_contains(
        server->append_qids, GUINT_TO_POINTER(fid->qid.path));
    if ((change_mode && !server->backend->ops->chmod) ||
        (change_gid && !server->backend->ops->chown) ||
        (change_mtime && !server->backend->ops->utimensat) ||
        (change_name && !server->backend->ops->renameat)) {
        return -EOPNOTSUPP;
    }
    if (change_mode) {
        ret = co_chmod(server, &fid->path, mode);
        if (ret < 0) {
            goto out;
        }
        mutated = true;
    }
    if (change_gid) {
        ret = co_chown_gid(server, &fid->path, gid);
        if (ret < 0) {
            goto out;
        }
        mutated = true;
    }
    if (change_mtime) {
        ret = co_utimensat(server, &fid->path, request->tx.dir.mtime);
        if (ret < 0) {
            goto out;
        }
        mutated = true;
    }
    if (change_name) {
        ret = rename_fids(server, fid, name);
        if (ret < 0) {
            goto out;
        }
        mutated = true;
    }
    if (change_append) {
        GHashTableIter iter;
        gpointer value;

        if (append) {
            g_hash_table_add(server->append_qids,
                             GUINT_TO_POINTER(fid->qid.path));
        } else {
            g_hash_table_remove(server->append_qids,
                                GUINT_TO_POINTER(fid->qid.path));
        }
        g_hash_table_iter_init(&iter, server->fids);
        while (g_hash_table_iter_next(&iter, NULL, &value)) {
            Plan9P1Fid *other = value;

            if (other->qid.path == fid->qid.path) {
                other->append_only = append;
            }
        }
        mutated = true;
    }
    reply->type = PLAN9P1_RWSTAT;
    reply->fid = fid->fid;
    ret = 0;
out:
    if (mutated) {
        invalidate_dir_caches(server);
    }
    return ret;
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
    if (g_hash_table_contains(server->append_qids,
                              GUINT_TO_POINTER(reply->dir.qid.path))) {
        reply->dir.mode |= PLAN9P1_DMAPPEND;
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
    bool remove;
    int close_ret;
    int remove_ret = 0;

    if (!fid) {
        return -ENOENT;
    }
    remove = fid->remove_on_clunk;
    fid->remove_on_clunk = false;
    close_ret = close_fid(server, fid);
    if (remove) {
        remove_ret = remove_fid_path(server, fid);
    }

    g_hash_table_remove(server->fids, fid_key(request->tx.fid));
    if (remove_ret < 0) {
        return remove_ret;
    }
    if (close_ret < 0) {
        return close_ret;
    }
    reply->type = PLAN9P1_RCLUNK;
    reply->fid = request->tx.fid;
    return 0;
}

static int coroutine_fn remove_fid(Plan9P1Request *request,
                                   Plan9P1Fcall *reply)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fid *fid = find_fid(server, request->tx.fid);
    int close_ret;
    int remove_ret;

    if (!fid) {
        return -ENOENT;
    }
    fid->remove_on_clunk = false;
    close_ret = close_fid(server, fid);
    if (server->backend->ctx.export_flags & V9FS_RDONLY) {
        remove_ret = -EROFS;
    } else {
        remove_ret = remove_fid_path(server, fid);
    }
    g_hash_table_remove(server->fids, fid_key(request->tx.fid));
    if (remove_ret < 0) {
        return remove_ret;
    }
    if (close_ret < 0) {
        return close_ret;
    }
    reply->type = PLAN9P1_RREMOVE;
    reply->fid = request->tx.fid;
    return 0;
}

static void coroutine_fn handle_request(Plan9P1Request *request)
{
    Plan9P1Server *server = request->server;
    Plan9P1Fcall reply = {
        .tag = request->tx.tag,
        .first_edition = request->tx.first_edition,
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
    case PLAN9P1_TAUTH:
        if (server_first_edition_none_auth(request, &reply) < 0) {
            encode_error(request, request->tx.tag, EACCES,
                         "authentication failed");
            return;
        }
        break;
    case PLAN9P1_TSESSION:
        cleanup_fids(server);
        reply.type = PLAN9P1_RSESSION;
        if (server->auth_configured &&
            server_auth_session_begin(server, &request->tx, &reply) < 0) {
            encode_error(request, request->tx.tag, EACCES,
                         "authentication failed");
            return;
        }
        break;
    case PLAN9P1_TATTACH:
        if (server->auth_configured &&
            server_authenticate_attach(request, &reply) < 0) {
            encode_error(request, request->tx.tag, EACCES,
                         "authentication failed");
            return;
        }
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
    case PLAN9P1_TCREATE:
        ret = create_fid(request, &reply);
        break;
    case PLAN9P1_TREAD:
        ret = read_fid(request, &reply);
        break;
    case PLAN9P1_TWRITE:
        ret = write_fid(request, &reply);
        break;
    case PLAN9P1_TSTAT:
        ret = stat_fid(request, &reply);
        break;
    case PLAN9P1_TCLUNK:
        ret = clunk_fid(request, &reply);
        break;
    case PLAN9P1_TREMOVE:
        ret = remove_fid(request, &reply);
        break;
    case PLAN9P1_TWSTAT:
        ret = wstat_fid(request, &reply);
        break;
    case PLAN9P1_TFLUSH:
        reply.type = PLAN9P1_RFLUSH;
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
    server_auth_session_clear(server);
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
    server_auth_session_clear(server);
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
        if (server->transport_ops.kind == PLAN9P1_TRANSPORT_RECORD) {
            if (capacity < reply->len) {
                break;
            }
            owner_ref(server);
            server->callback_depth++;
            sent = server->transport_ops.send(reply->data, reply->len,
                                              server->transport_opaque);
            server->callback_depth--;
            owner_unref(server);
            if (server_has_deferred(server)) {
                server_apply_deferred(server);
            }
            if (g_queue_peek_head(&server->replies) != reply) {
                break;
            }
            if (sent == -EAGAIN) {
                break;
            }
            if (sent < 0 || (size_t)sent != reply->len) {
                server_transport_failed(server);
                break;
            }
            g_queue_pop_head(&server->replies);
            server->queued_bytes -= reply->len;
            reply_free(reply);
            continue;
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
    case PLAN9P1_TAUTH:
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
        server_auth_session_clear(server);
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
    server->append_qids = g_hash_table_new(g_direct_hash, g_direct_equal);
    g_queue_init(&server->requests);
    g_queue_init(&server->replies);
    g_queue_init(&server->deferred_inputs);
#ifdef CONFIG_SLIRP
    QTAILQ_INIT(&server->auth_connections);
#endif
    server->max_devices = 127;
    server->max_queued_bytes = PLAN9P1_DEFAULT_QUEUE_BYTES;
    server->max_dir_cache_bytes = PLAN9P1_DEFAULT_DIR_CACHE_BYTES;
    server->max_qid_entries = PLAN9P1_DEFAULT_QID_ENTRIES;
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
    if (ops->kind != PLAN9P1_TRANSPORT_STREAM &&
        ops->kind != PLAN9P1_TRANSPORT_RECORD) {
        error_setg(errp, "9P1 server transport kind is invalid");
        return -1;
    }
    if (server->auth_configured &&
        ops->kind != PLAN9P1_TRANSPORT_RECORD) {
        error_setg(errp, "Plan 9 file authentication requires record "
                   "transport");
        return -1;
    }
    server->transport_ops = *ops;
    server->transport_opaque = transport_opaque;
    server->started = true;
    return 0;
}

int plan9p1_server_configure_auth(Plan9P1Server *server,
                                  const Plan9P1AuthConfig *config,
                                  Error **errp)
{
    uint8_t server_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint32_t now;
    size_t id_len;
    size_t domain_len;
    int ret = -1;

    if (!server || !config || !config->keydb || !config->auth_id ||
        !config->auth_domain) {
        error_setg(errp, "Plan 9 file authentication configuration is "
                   "incomplete");
        goto out;
    }
    if (server->started || server->closing || server->auth_configured) {
        error_setg(errp, "Plan 9 file authentication must be configured "
                   "once before transport start");
        goto out;
    }
    id_len = strnlen(config->auth_id, PLAN9_AUTH_NAMELEN);
    domain_len = strnlen(config->auth_domain, PLAN9_AUTH_DOMLEN);
    if (!id_len || id_len == PLAN9_AUTH_NAMELEN || !domain_len ||
        domain_len == PLAN9_AUTH_DOMLEN) {
        error_setg(errp, "Plan 9 authentication identity or domain is "
                   "not representable");
        goto out;
    }

    server->auth_keydb = config->keydb;
    server->auth_random_bytes = config->random_bytes;
    server->auth_random_opaque = config->random_opaque;
    server->auth_now_seconds = config->now_seconds;
    server->auth_now_opaque = config->now_opaque;
    if (server_auth_now(server, &now, errp)) {
        goto rollback;
    }
    if (plan9_auth_keydb_lookup(config->keydb, config->auth_id, now,
                                server_key) != PLAN9_AUTH_KEY_AVAILABLE) {
        error_setg(errp, "Plan 9 authentication server identity is not "
                   "available");
        goto rollback;
    }
    if (config->auth_id != server->auth_id) {
        g_strlcpy(server->auth_id, config->auth_id, sizeof(server->auth_id));
    }
    if (config->auth_domain != server->auth_domain) {
        g_strlcpy(server->auth_domain, config->auth_domain,
                  sizeof(server->auth_domain));
    }
    server->auth_configured = true;
    ret = 0;
    goto out;

rollback:
    server->auth_keydb = NULL;
    server->auth_random_bytes = NULL;
    server->auth_random_opaque = NULL;
    server->auth_now_seconds = NULL;
    server->auth_now_opaque = NULL;
out:
    plan9_auth_clear(server_key, sizeof(server_key));
    return ret;
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

    if (server->transport_ops.kind == PLAN9P1_TRANSPORT_RECORD) {
        if (server->connection_failed) {
            error_setg(errp, "9P1 record transport is failed");
            return -1;
        }
        if (!buf || !len) {
            error_setg(errp, "9P1 record is empty");
            server_transport_failed(server);
            return -1;
        }
        if (enqueue_frame(buf, len, server, &local_err) == 0) {
            return 0;
        }
        server_transport_failed(server);
        error_propagate(errp, local_err);
        return -1;
    }

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

void plan9p1_server_connection_closed(Plan9P1Server *server)
{
    if (!server || server->closing ||
        server->transport_ops.kind != PLAN9P1_TRANSPORT_RECORD) {
        return;
    }
    if (!server->callback_depth) {
        clear_deferred_inputs(server);
    }
    plan9p1_server_reset(server);
}

void plan9p1_server_reset(Plan9P1Server *server)
{
    if (!server || server->closing) {
        return;
    }
    if (server->callback_depth) {
        server_auth_session_clear(server);
        server->resetting = true;
        server->deferred_reset = true;
        return;
    }
    server->connection_failed = false;
    server->resetting = true;
    server_auth_session_clear(server);
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

bool plan9p1_server_record_connection_ready(const Plan9P1Server *server)
{
    return server && server->started && !server->closing &&
           server->transport_ops.kind == PLAN9P1_TRANSPORT_RECORD &&
           !server->resetting && !server->connection_failed &&
           !server->active && server->requests.length == 0 &&
           server->replies.length == 0 &&
           server->deferred_inputs.length == 0 &&
           g_hash_table_size(server->fids) == 0;
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
    server_auth_session_clear(server);
#ifdef CONFIG_SLIRP
    plan9p1_server_slirp_cleanup(server);
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

static int plan9p1_server_get_transport(Object *obj, Error **errp)
{
    return PLAN9P1_SERVER(obj)->qom_transport;
}

static void plan9p1_server_set_transport(Object *obj, int value, Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (plan9p1_server_properties_mutable(server, errp)) {
        server->qom_transport = value;
    }
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

static char *plan9p1_server_get_auth_id(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->auth_id);
}

static void plan9p1_server_set_auth_id(Object *obj, const char *value,
                                       Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    if (strlen(value) >= sizeof(server->auth_id)) {
        error_setg(errp, "9P1 auth-id must contain at most %u bytes",
                   PLAN9_AUTH_NAMELEN - 1);
        return;
    }
    g_strlcpy(server->auth_id, value, sizeof(server->auth_id));
}

static char *plan9p1_server_get_auth_domain(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->auth_domain);
}

static void plan9p1_server_set_auth_domain(Object *obj, const char *value,
                                           Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    if (strlen(value) >= sizeof(server->auth_domain)) {
        error_setg(errp, "9P1 auth-domain must contain at most %u bytes",
                   PLAN9_AUTH_DOMLEN - 1);
        return;
    }
    g_strlcpy(server->auth_domain, value, sizeof(server->auth_domain));
}

static char *plan9p1_server_get_keydb(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->keydb_path);
}

static void plan9p1_server_set_keydb(Object *obj, const char *value,
                                     Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    g_free(server->keydb_path);
    server->keydb_path = g_strdup(value);
}

static char *plan9p1_server_get_key_secret(Object *obj, Error **errp)
{
    return g_strdup(PLAN9P1_SERVER(obj)->key_secret_id);
}

static void plan9p1_server_set_key_secret(Object *obj, const char *value,
                                          Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);

    if (!plan9p1_server_properties_mutable(server, errp)) {
        return;
    }
    g_free(server->key_secret_id);
    server->key_secret_id = g_strdup(value);
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

static void plan9p1_server_get_il_port(Object *obj, Visitor *visitor,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    uint16_t value = PLAN9P1_SERVER(obj)->il_port;

    visit_type_uint16(visitor, name, &value, errp);
}

static void plan9p1_server_set_il_port(Object *obj, Visitor *visitor,
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
        error_setg(errp, "9P1 IL port must not be zero");
        return;
    }
    server->il_port = value;
}

static void plan9p1_server_get_auth_port(Object *obj, Visitor *visitor,
                                         const char *name, void *opaque,
                                         Error **errp)
{
    uint16_t value = PLAN9P1_SERVER(obj)->auth_port;

    visit_type_uint16(visitor, name, &value, errp);
}

static void plan9p1_server_set_auth_port(Object *obj, Visitor *visitor,
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
        error_setg(errp, "9P1 authentication port must not be zero");
        return;
    }
    server->auth_port = value;
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

static bool plan9p1_server_il_acquire(Plan9P1Server *server)
{
    Error *local_err = NULL;

    if (!server->il_connections) {
        if (!server->migration_blocker) {
            error_setg(&server->migration_blocker,
                       "Plan 9 IL connections must reconnect after migration");
        }
        if (migrate_add_blocker(&server->migration_blocker, &local_err) < 0) {
            error_free(local_err);
            return false;
        }
    }
    server->il_connections++;
    return true;
}

static void plan9p1_server_il_release(Plan9P1Server *server)
{
    assert(server->il_connections);
    if (!--server->il_connections) {
        migrate_del_blocker(&server->migration_blocker);
    }
}

static bool plan9p1_server_il_may_accept(Plan9P1Server *server)
{
    if (!server->completed || server->closing) {
        return false;
    }
    if (!server->il_accepting && server->il_reset_draining &&
        !server->il_connections) {
        server->il_reset_draining = false;
        server->il_accepting = true;
    }
    return server->il_accepting;
}

static size_t plan9p1_server_il_can_send(void *opaque)
{
    Plan9P1Server *server = opaque;

    return server->file_connection ? PLAN9P1_MAX_FRAME : 0;
}

static int plan9p1_server_il_send(const uint8_t *buf, size_t len,
                                  void *opaque)
{
    Plan9P1Server *server = opaque;
    QemuSlirpILConnection *connection = server->file_connection;
    int ret;

    if (!connection) {
        return -ENOTCONN;
    }
    ret = qemu_slirp_il_send_record(connection, buf, len);
    return ret == 0 ? len : ret;
}

static const Plan9P1TransportOps plan9p1_server_il_transport_ops = {
    .can_send = plan9p1_server_il_can_send,
    .send = plan9p1_server_il_send,
    .kind = PLAN9P1_TRANSPORT_RECORD,
};

static void plan9p1_server_file_request_close(Plan9P1Server *server)
{
    if (server->file_connection && !server->file_close_requested) {
        server->file_close_requested = true;
        qemu_slirp_il_connection_close(server->file_connection);
    }
}

static void *plan9p1_server_file_open(QemuSlirpILConnection *connection,
                                      void *opaque)
{
    Plan9P1Server *server = opaque;

    if (!plan9p1_server_il_may_accept(server) || server->file_connection ||
        !plan9p1_server_record_connection_ready(server) ||
        !plan9p1_server_il_acquire(server)) {
        qemu_slirp_il_connection_close(connection);
        return NULL;
    }
    server->file_connection = connection;
    server->file_close_requested = false;
    return server;
}

static void plan9p1_server_file_record(QemuSlirpILConnection *connection,
                                       const uint8_t *data, size_t len,
                                       void *opaque)
{
    Plan9P1Server *server = opaque;
    Error *local_err = NULL;

    if (!server || server->file_connection != connection) {
        qemu_slirp_il_connection_close(connection);
        return;
    }
    if (plan9p1_server_receive(server, data, len, &local_err) < 0) {
        error_free(local_err);
        plan9p1_server_file_request_close(server);
    }
}

static void plan9p1_server_file_can_send(QemuSlirpILConnection *connection,
                                         void *opaque)
{
    Plan9P1Server *server = opaque;

    if (server && server->file_connection == connection) {
        plan9p1_server_can_send(server);
    }
}

static void plan9p1_server_file_close(QemuSlirpILConnection *connection,
                                      void *opaque)
{
    Plan9P1Server *server = opaque;

    /* Rejected synchronous opens deliberately have a NULL opaque value. */
    if (!server || server->file_connection != connection) {
        return;
    }
    server->file_connection = NULL;
    server->file_close_requested = false;
    plan9p1_server_il_release(server);
    plan9p1_server_connection_closed(server);
}

static const QemuSlirpILListenerOps plan9p1_server_file_listener_ops = {
    .open = plan9p1_server_file_open,
    .record = plan9p1_server_file_record,
    .can_send = plan9p1_server_file_can_send,
    .close = plan9p1_server_file_close,
};

static int plan9p1_server_auth_send(const uint8_t *buf, size_t len,
                                    void *opaque)
{
    Plan9P1ILAuthConnection *state = opaque;
    int ret;

    if (!state->il) {
        return -ENOTCONN;
    }
    ret = qemu_slirp_il_send_record(state->il, buf, len);
    return ret == 0 ? len : ret;
}

static void plan9p1_server_auth_request_close(void *opaque)
{
    Plan9P1ILAuthConnection *state = opaque;

    if (state->il && !state->close_requested) {
        state->close_requested = true;
        qemu_slirp_il_connection_close(state->il);
    }
}

static const Plan9AuthTicketTransportOps plan9p1_server_auth_ops = {
    .send_record = plan9p1_server_auth_send,
    .close = plan9p1_server_auth_request_close,
};

static void *plan9p1_server_auth_open(QemuSlirpILConnection *connection,
                                      void *opaque)
{
    Plan9P1Server *server = opaque;
    Plan9P1ILAuthConnection *state;

    if (!plan9p1_server_il_may_accept(server) ||
        !plan9p1_server_il_acquire(server)) {
        qemu_slirp_il_connection_close(connection);
        return NULL;
    }
    state = g_new0(Plan9P1ILAuthConnection, 1);
    state->server = server;
    state->il = connection;
    state->migration_counted = true;
    state->auth = plan9_auth_ticket_connection_new(
        server->ticket_service, &plan9p1_server_auth_ops, state, NULL);
    if (!state->auth) {
        plan9p1_server_il_release(server);
        g_free(state);
        qemu_slirp_il_connection_close(connection);
        return NULL;
    }
    state->linked = true;
    QTAILQ_INSERT_TAIL(&server->auth_connections, state, entry);
    return state;
}

static void plan9p1_server_auth_record(QemuSlirpILConnection *connection,
                                       const uint8_t *data, size_t len,
                                       void *opaque)
{
    Plan9P1ILAuthConnection *state = opaque;
    Error *local_err = NULL;

    if (!state || state->il != connection) {
        qemu_slirp_il_connection_close(connection);
        return;
    }
    if (plan9_auth_ticket_connection_receive_record(state->auth, data, len,
                                                     &local_err) < 0) {
        error_free(local_err);
        plan9p1_server_auth_request_close(state);
    }
}

static void plan9p1_server_auth_can_send(QemuSlirpILConnection *connection,
                                         void *opaque)
{
    Plan9P1ILAuthConnection *state = opaque;

    if (state && state->il == connection) {
        plan9_auth_ticket_connection_can_send(state->auth);
    }
}

static void plan9p1_server_auth_close(QemuSlirpILConnection *connection,
                                      void *opaque)
{
    Plan9P1ILAuthConnection *state = opaque;
    Plan9P1Server *server;

    if (!state) {
        return;
    }
    server = state->server;
    state->il = NULL;
    state->close_requested = false;
    if (state->linked) {
        QTAILQ_REMOVE(&server->auth_connections, state, entry);
        state->linked = false;
    }
    plan9_auth_ticket_connection_free(state->auth);
    state->auth = NULL;
    if (state->migration_counted) {
        state->migration_counted = false;
        plan9p1_server_il_release(server);
    }
    g_free(state);
}

static const QemuSlirpILListenerOps plan9p1_server_auth_listener_ops = {
    .open = plan9p1_server_auth_open,
    .record = plan9p1_server_auth_record,
    .can_send = plan9p1_server_auth_can_send,
    .close = plan9p1_server_auth_close,
};

static void plan9p1_server_slirp_cleanup(Plan9P1Server *server)
{
    QemuSlirpILListener *listener;

    server->il_accepting = false;
    listener = server->file_listener;
    server->file_listener = NULL;
    qemu_slirp_il_listener_remove(listener);
    listener = server->auth_listener;
    server->auth_listener = NULL;
    qemu_slirp_il_listener_remove(listener);
    if (server->guestfwd) {
        QemuSlirpGuestFwd *guestfwd = server->guestfwd;

        server->guestfwd = NULL;
        qemu_slirp_guestfwd_remove(guestfwd);
    }
    plan9_auth_ticket_service_free(server->ticket_service);
    server->ticket_service = NULL;
    plan9_auth_keydb_free(server->owned_keydb);
    server->owned_keydb = NULL;
    server->auth_keydb = NULL;
    qemu_slirp_plan9_bootp_release(&server->bootp_lease);
    if (server->il_connections) {
        server->il_connections = 0;
        migrate_del_blocker(&server->migration_blocker);
    }
    error_free(server->migration_blocker);
    server->migration_blocker = NULL;
}

static int plan9p1_server_current_seconds(uint32_t *seconds, Error **errp)
{
    time_t now = time(NULL);

    if (now < 0 || (uint64_t)now > UINT32_MAX) {
        error_setg(errp, "current time is outside the Plan 9 key range");
        return -1;
    }
    *seconds = now;
    return 0;
}

static ResettableState *plan9p1_server_reset_state(Object *obj)
{
    return &PLAN9P1_SERVER(obj)->reset_state;
}

static void plan9p1_server_reset_hold(Object *obj, ResetType type)
{
    Plan9P1Server *server = PLAN9P1_SERVER(obj);
    Plan9P1ILAuthConnection *state;
    Plan9P1ILAuthConnection *next;

    if (!server->completed || server->closing) {
        return;
    }
    if (server->qom_transport == PLAN9_P1_SERVER_TRANSPORT_IL) {
        server->il_accepting = false;
        server->il_reset_draining = true;
        plan9p1_server_file_request_close(server);
        QTAILQ_FOREACH_SAFE(state, &server->auth_connections, entry, next) {
            plan9p1_server_auth_request_close(state);
        }
    }
    plan9p1_server_reset(server);
}

static void plan9p1_server_complete(UserCreatable *uc, Error **errp)
{
    Plan9P1Server *server = PLAN9P1_SERVER(uc);
    struct in_addr guest_address;
    struct in_addr no_auth = { 0 };
    uint8_t master_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint8_t *secret = NULL;
    size_t secret_len = 0;
    uint32_t now;
    Error *local_err = NULL;
    bool bootp_available;

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

    if (server->qom_transport == PLAN9_P1_SERVER_TRANSPORT_TCP) {
        if (server->auth_id[0] || server->auth_domain[0] ||
            (server->keydb_path && server->keydb_path[0]) ||
            (server->key_secret_id && server->key_secret_id[0])) {
            error_setg(errp, "9P1 authentication properties require "
                       "transport=il");
            return;
        }
    } else {
        if (!server->il_port || !server->auth_port) {
            error_setg(errp, "9P1 IL and authentication ports must not be "
                       "zero");
            return;
        }
        if (!server->auth_id[0] || !server->auth_domain[0] ||
            !server->keydb_path || !server->keydb_path[0] ||
            !server->key_secret_id || !server->key_secret_id[0]) {
            error_setg(errp, "transport=il requires auth-id, auth-domain, "
                       "keydb, and key-secret");
            return;
        }
        if (!qemu_slirp_il_available(server->netdev_id, errp)) {
            return;
        }
        bootp_available = qemu_slirp_plan9_bootp_available(server->netdev_id,
                                                           &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
        if (!bootp_available) {
            error_setg(errp, "Plan 9 BOOTP is unavailable in this libslirp");
            return;
        }
        if (qcrypto_secret_lookup(server->key_secret_id, &secret,
                                  &secret_len, errp) < 0) {
            return;
        }
        if (secret_len != sizeof(master_key)) {
            error_setg(errp, "Plan 9 key Secret must contain exactly %zu "
                       "bytes", sizeof(master_key));
            goto fail_secret;
        }
        memcpy(master_key, secret, sizeof(master_key));
        plan9_auth_clear(secret, secret_len + 1);
        g_free(secret);
        secret = NULL;
        if (plan9p1_server_current_seconds(&now, errp) < 0) {
            goto fail_secret;
        }
        server->owned_keydb = plan9_auth_keydb_load(
            server->keydb_path, master_key, server->auth_id, now, errp);
        if (!server->owned_keydb) {
            goto fail_secret;
        }
    }

    if (plan9p1_server_backend_init(server, server->fsdev_id, NULL, errp) < 0) {
        goto fail_all;
    }

    if (server->qom_transport == PLAN9_P1_SERVER_TRANSPORT_TCP) {
        if (qemu_slirp_guestfwd_add(server->netdev_id, guest_address,
                                    server->port,
                                    &plan9p1_server_guestfwd_ops, server,
                                    &server->guestfwd, errp) < 0) {
            goto fail_all;
        }
        bootp_available = qemu_slirp_plan9_bootp_available(server->netdev_id,
                                                           &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            goto fail_all;
        }
        if (bootp_available &&
            !qemu_slirp_plan9_bootp_claim(server->netdev_id, guest_address,
                                          no_auth, &server->bootp_lease,
                                          errp)) {
            goto fail_all;
        }
        if (plan9p1_server_start(server, &plan9p1_server_transport_ops,
                                 server, errp) < 0) {
            goto fail_all;
        }
    } else {
        Plan9AuthTicketServiceConfig ticket_config = {
            .keydb = server->owned_keydb,
        };
        Plan9P1AuthConfig auth_config = {
            .keydb = server->owned_keydb,
            .auth_id = server->auth_id,
            .auth_domain = server->auth_domain,
        };

        if (!qemu_slirp_plan9_bootp_claim(server->netdev_id, guest_address,
                                          guest_address,
                                          &server->bootp_lease, errp)) {
            goto fail_all;
        }
        server->ticket_service = plan9_auth_ticket_service_new(
            &ticket_config, errp);
        if (!server->ticket_service) {
            goto fail_all;
        }
        if (qemu_slirp_il_listen(server->netdev_id, guest_address,
                                 server->auth_port,
                                 &plan9p1_server_auth_listener_ops, server,
                                 &server->auth_listener, errp) < 0) {
            goto fail_all;
        }
        if (qemu_slirp_il_listen(server->netdev_id, guest_address,
                                 server->il_port,
                                 &plan9p1_server_file_listener_ops, server,
                                 &server->file_listener, errp) < 0) {
            goto fail_all;
        }
        if (plan9p1_server_configure_auth(server, &auth_config, errp) < 0 ||
            plan9p1_server_start(server, &plan9p1_server_il_transport_ops,
                                 server, errp) < 0) {
            goto fail_all;
        }
        server->il_accepting = true;
    }
    server->completed = true;
    qemu_register_resettable(OBJECT(server));
    server->reset_registered = true;
    plan9_auth_clear(master_key, sizeof(master_key));
    return;

fail_all:
    plan9p1_server_slirp_cleanup(server);
    if (server->backend) {
        v9fs_backend_cleanup(&server->backend_storage);
        server->backend = NULL;
    }
fail_secret:
    if (secret) {
        plan9_auth_clear(secret, secret_len + 1);
        g_free(secret);
    }
    plan9_auth_clear(master_key, sizeof(master_key));
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
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    ObjectProperty *property;

    ucc->complete = plan9p1_server_complete;
    ucc->prepare_delete = plan9p1_server_prepare_delete;
    oc->unparent = plan9p1_server_unparent;
    rc->get_state = plan9p1_server_reset_state;
    rc->phases.hold = plan9p1_server_reset_hold;

    object_class_property_add_str(oc, "fsdev", plan9p1_server_get_fsdev,
                                  plan9p1_server_set_fsdev);
    object_class_property_add_str(oc, "netdev", plan9p1_server_get_netdev,
                                  plan9p1_server_set_netdev);
    property = object_class_property_add_str(
        oc, "guest-address", plan9p1_server_get_guest_address,
        plan9p1_server_set_guest_address);
    object_property_set_default_str(property, "10.0.2.100");
    property = object_class_property_add_enum(
        oc, "transport", "Plan9P1ServerTransport",
        &Plan9P1ServerTransport_lookup, plan9p1_server_get_transport,
        plan9p1_server_set_transport);
    object_property_set_default_str(property, "tcp");
    property = object_class_property_add(oc, "port", "uint16",
                                         plan9p1_server_get_port,
                                         plan9p1_server_set_port,
                                         NULL, NULL);
    object_property_set_default_uint(property, 564);
    property = object_class_property_add(oc, "il-port", "uint16",
                                         plan9p1_server_get_il_port,
                                         plan9p1_server_set_il_port,
                                         NULL, NULL);
    object_property_set_default_uint(property, 17008);
    property = object_class_property_add(oc, "auth-port", "uint16",
                                         plan9p1_server_get_auth_port,
                                         plan9p1_server_set_auth_port,
                                         NULL, NULL);
    object_property_set_default_uint(property, 566);
    object_class_property_add_str(oc, "auth-id", plan9p1_server_get_auth_id,
                                  plan9p1_server_set_auth_id);
    object_class_property_add_str(oc, "auth-domain",
                                  plan9p1_server_get_auth_domain,
                                  plan9p1_server_set_auth_domain);
    object_class_property_add_str(oc, "keydb", plan9p1_server_get_keydb,
                                  plan9p1_server_set_keydb);
    object_class_property_add_str(oc, "key-secret",
                                  plan9p1_server_get_key_secret,
                                  plan9p1_server_set_key_secret);
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
        { TYPE_RESETTABLE_INTERFACE },
        { }
    },
#endif
};

static void plan9p1_server_register_types(void)
{
    type_register_static(&plan9p1_server_type_info);
}

type_init(plan9p1_server_register_types)
