/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include <sys/statvfs.h>

#include "block/thread-pool.h"
#include "crypto/hash.h"
#include "crypto/random.h"
#include "hw/9pfs/9p-backend.h"
#include "hw/9pfs/9p.h"
#include "hw/nfs/nfs2-handle.h"
#include "hw/nfs/nfs2-server.h"
#include "hw/nfs/nfs2-xdr.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/aio.h"
#include "qemu/coroutine.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qemu/xattr.h"

#define NFS3_VERSION 3U
#define MOUNT3_VERSION 3U
#define NFS3_FHSIZE 32U
#define NFS3_COOKIEVERFSIZE 8U
#define NFS2_MAX_IDENTITIES 65536U
#define NFS2_MAX_COOKIES 65536U
#define NFS_DUP_CACHE_MAX 256U
#define NFS_DUP_CACHE_TTL_MS 60000
#define NFS3_CREATE_VERIFIER_XATTR "user.qemu.nfs3.createverf"

static bool is_v2_mutator(uint32_t proc);
static bool is_v3_mutator(uint32_t proc);

typedef struct NfsDuplicateEntry {
    uint32_t address;
    uint16_t port;
    uint32_t xid;
    uint8_t digest[32];
    int64_t completed_at;
    uint64_t last_used;
    GByteArray *reply;
    bool in_flight;
    bool abandoned;
    bool sending;
} NfsDuplicateEntry;

enum {
    NFS3PROC_NULL = 0, NFS3PROC_GETATTR = 1, NFS3PROC_SETATTR = 2,
    NFS3PROC_LOOKUP = 3, NFS3PROC_ACCESS = 4, NFS3PROC_READLINK = 5,
    NFS3PROC_READ = 6, NFS3PROC_WRITE = 7, NFS3PROC_CREATE = 8,
    NFS3PROC_MKDIR = 9, NFS3PROC_SYMLINK = 10, NFS3PROC_MKNOD = 11,
    NFS3PROC_REMOVE = 12, NFS3PROC_RMDIR = 13, NFS3PROC_RENAME = 14,
    NFS3PROC_LINK = 15, NFS3PROC_READDIR = 16,
    NFS3PROC_READDIRPLUS = 17, NFS3PROC_FSSTAT = 18,
    NFS3PROC_FSINFO = 19, NFS3PROC_PATHCONF = 20, NFS3PROC_COMMIT = 21,
};

enum {
    NFS3_OK = 0, NFS3ERR_PERM = 1, NFS3ERR_NOENT = 2,
    NFS3ERR_IO = 5, NFS3ERR_NXIO = 6, NFS3ERR_ACCES = 13,
    NFS3ERR_EXIST = 17, NFS3ERR_XDEV = 18, NFS3ERR_NODEV = 19,
    NFS3ERR_NOTDIR = 20, NFS3ERR_ISDIR = 21, NFS3ERR_INVAL = 22,
    NFS3ERR_FBIG = 27, NFS3ERR_NOSPC = 28, NFS3ERR_ROFS = 30,
    NFS3ERR_MLINK = 31, NFS3ERR_NAMETOOLONG = 63,
    NFS3ERR_NOTEMPTY = 66, NFS3ERR_DQUOT = 69, NFS3ERR_STALE = 70,
    NFS3ERR_REMOTE = 71, NFS3ERR_BADHANDLE = 10001,
    NFS3ERR_NOT_SYNC = 10002, NFS3ERR_BAD_COOKIE = 10003,
    NFS3ERR_NOTSUPP = 10004, NFS3ERR_TOOSMALL = 10005,
    NFS3ERR_SERVERFAULT = 10006, NFS3ERR_BADTYPE = 10007,
};

typedef enum BackendOp {
    BACKEND_NAME_TO_PATH,
    BACKEND_LSTAT,
    BACKEND_READLINK,
    BACKEND_OPEN,
    BACKEND_FSTAT,
    BACKEND_CLOSE,
    BACKEND_PREADV,
    BACKEND_OPENDIR,
    BACKEND_SEEKDIR,
    BACKEND_TELLDIR,
    BACKEND_READDIR,
    BACKEND_CLOSEDIR,
    BACKEND_STATFS,
    BACKEND_OPEN2,
    BACKEND_PWRITEV,
    BACKEND_FSYNC,
    BACKEND_CHMOD,
    BACKEND_TRUNCATE,
    BACKEND_UTIMENSAT,
    BACKEND_MKDIR,
    BACKEND_MKNOD,
    BACKEND_SYMLINK,
    BACKEND_UNLINKAT,
    BACKEND_RENAMEAT,
    BACKEND_LINK,
    BACKEND_FGETXATTR,
    BACKEND_FSETXATTR,
} BackendOp;

typedef struct BackendWork {
    BackendOp op;
    V9fsBackend *backend;
    V9fsPath *dir;
    V9fsPath *path;
    const char *name;
    const char *new_name;
    const char *target;
    V9fsPath *new_dir;
    struct stat *st;
    struct statfs *stfs;
    V9fsFidOpenState *open;
    int fid_type;
    struct iovec *iov;
    off_t offset;
    char *buffer;
    size_t length;
    char dirent_name[NAME_MAX + 1];
    uint64_t dirent_ino;
    uint64_t cookie;
    bool eof;
    int flags;
    FsCred cred;
    struct timespec times[2];
    const char *xattr_name;
    void *xattr_value;
    int xattr_flags;
} BackendWork;

typedef struct Nfs2Request {
    Nfs2Server *server;
    Nfs2Service service;
    struct sockaddr_in peer;
    size_t length;
    NfsDuplicateEntry *duplicate;
    uint8_t data[];
} Nfs2Request;

typedef struct NfsIdentityKey {
    dev_t device;
    ino_t inode;
} NfsIdentityKey;

typedef struct NfsCookieKey {
    uint64_t directory;
    uint64_t backend_cookie;
} NfsCookieKey;

struct Nfs2Server {
    V9fsBackend backend;
    Nfs2HandleTable *handles;
    GHashTable *identities;
    GHashTable *cookies_by_backend;
    GHashTable *cookies_by_wire;
    uint32_t next_identity;
    uint32_t next_cookie;
    Nfs2FileHandle root_handle;
    Nfs2TransportOps transport;
    void *transport_opaque;
    unsigned int pending;
    bool writable;
    bool closing;
    uint8_t write_verifier[8];
    GPtrArray *duplicates;
    uint64_t duplicate_sequence;
    uint64_t exclusive_sequence;
    CoMutex mutation_mutex;
};

static void duplicate_free(gpointer opaque)
{
    NfsDuplicateEntry *entry = opaque;

    g_clear_pointer(&entry->reply, g_byte_array_unref);
    g_free(entry);
}

static int64_t server_now_ms(Nfs2Server *server)
{
    return server->transport.clock_ms ?
           server->transport.clock_ms(server->transport_opaque) :
           qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
}

static bool duplicate_matches(const NfsDuplicateEntry *entry,
                              const struct sockaddr_in *peer, uint32_t xid,
                              const uint8_t digest[32])
{
    return !entry->abandoned && entry->address == peer->sin_addr.s_addr &&
           entry->port == peer->sin_port && entry->xid == xid &&
           !memcmp(entry->digest, digest, 32);
}

static void duplicate_expire(Nfs2Server *server, int64_t now)
{
    for (size_t i = server->duplicates->len; i > 0; i--) {
        NfsDuplicateEntry *entry =
            g_ptr_array_index(server->duplicates, i - 1);

        if (!entry->in_flight && !entry->sending &&
            now >= entry->completed_at &&
            now - entry->completed_at >= NFS_DUP_CACHE_TTL_MS) {
            g_ptr_array_remove_index(server->duplicates, i - 1);
        }
    }
}

static bool request_is_mutation(Nfs2Server *server, Nfs2Service service,
                                const uint8_t *data, size_t len,
                                uint32_t *xid)
{
    Nfs2RpcCall call;

    if (service != NFS2_SERVICE_NFS ||
        nfs2_rpc_decode_call(data, len, &call) != NFS2_RPC_DECODE_OK ||
        call.program != NFS2_NFS_PROGRAM) {
        return false;
    }
    *xid = call.xid;
    return (call.version == NFS2_NFS_VERSION &&
            is_v2_mutator(call.procedure)) ||
           (call.version == NFS3_VERSION && is_v3_mutator(call.procedure));
}

static void path_clear(V9fsPath *path)
{
    g_free(path->data);
    memset(path, 0, sizeof(*path));
}

static void path_copy(V9fsPath *to, const V9fsPath *from)
{
    to->data = g_memdup2(from->data, from->size);
    to->size = from->size;
}

static int backend_worker(void *opaque)
{
    BackendWork *work = opaque;
    FileOperations *ops = work->backend->ops;
    FsContext *ctx = &work->backend->ctx;
    int ret;

    errno = 0;
    switch (work->op) {
    case BACKEND_NAME_TO_PATH:
        ret = ops->name_to_path(ctx, work->dir, work->name, work->path);
        break;
    case BACKEND_LSTAT:
        ret = ops->lstat(ctx, work->path, work->st);
        break;
    case BACKEND_READLINK:
        ret = ops->readlink ? ops->readlink(ctx, work->path, work->buffer,
                                           work->length) : -1;
        if (!ops->readlink) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_OPEN:
        ret = ops->open ? ops->open(ctx, work->path, work->flags,
                                    work->open) : -1;
        if (!ops->open) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_FSTAT:
        if (!ops->fstat) {
            errno = EOPNOTSUPP;
            ret = -1;
        } else {
            ret = ops->fstat(ctx, work->fid_type, work->open, work->st);
        }
        break;
    case BACKEND_CLOSE:
        ret = ops->close ? ops->close(ctx, work->open) : 0;
        break;
    case BACKEND_PREADV:
        ret = ops->preadv ? ops->preadv(ctx, work->open, work->iov, 1,
                                       work->offset) : -1;
        if (!ops->preadv) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_OPENDIR:
        ret = ops->opendir(ctx, work->path, work->open);
        break;
    case BACKEND_SEEKDIR:
        if (!ops->seekdir || work->cookie > INT64_MAX) {
            errno = EINVAL;
            ret = -1;
        } else {
            ops->seekdir(ctx, work->open, work->cookie);
            ret = 0;
        }
        break;
    case BACKEND_TELLDIR:
        if (!ops->telldir) {
            errno = EOPNOTSUPP;
            ret = -1;
        } else {
            off_t cookie = ops->telldir(ctx, work->open);

            if (cookie < 0) {
                ret = -1;
            } else {
                work->cookie = cookie;
                ret = 0;
            }
        }
        break;
    case BACKEND_READDIR: {
        struct dirent *entry = ops->readdir(ctx, work->open);

        if (!entry) {
            work->eof = errno == 0;
            ret = errno ? -1 : 0;
        } else {
            g_strlcpy(work->dirent_name, entry->d_name,
                      sizeof(work->dirent_name));
            work->dirent_ino = entry->d_ino;
            ret = 1;
        }
        break;
    }
    case BACKEND_CLOSEDIR:
        ret = ops->closedir ? ops->closedir(ctx, work->open) : 0;
        break;
    case BACKEND_STATFS:
        ret = ops->statfs ? ops->statfs(ctx, work->path, work->stfs) : -1;
        if (!ops->statfs) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_OPEN2:
        ret = ops->open2 ? ops->open2(ctx, work->dir, work->name,
                                      work->flags, &work->cred,
                                      work->open) : -1;
        if (!ops->open2) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_PWRITEV:
        ret = ops->pwritev ? ops->pwritev(ctx, work->open, work->iov, 1,
                                          work->offset) : -1;
        if (!ops->pwritev) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_FSYNC:
        ret = ops->fsync ? ops->fsync(ctx, work->fid_type, work->open, 0) : -1;
        if (!ops->fsync) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_CHMOD:
        ret = ops->chmod ? ops->chmod(ctx, work->path, &work->cred) : -1;
        if (!ops->chmod) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_TRUNCATE:
        ret = ops->truncate ? ops->truncate(ctx, work->path,
                                            work->offset) : -1;
        if (!ops->truncate) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_UTIMENSAT:
        ret = ops->utimensat ? ops->utimensat(ctx, work->path,
                                              work->times) : -1;
        if (!ops->utimensat) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_MKDIR:
        ret = ops->mkdir ? ops->mkdir(ctx, work->dir, work->name,
                                      &work->cred) : -1;
        if (!ops->mkdir) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_MKNOD:
        ret = ops->mknod ? ops->mknod(ctx, work->dir, work->name,
                                      &work->cred) : -1;
        if (!ops->mknod) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_SYMLINK:
        ret = ops->symlink ? ops->symlink(ctx, work->target, work->dir,
                                          work->name, &work->cred) : -1;
        if (!ops->symlink) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_UNLINKAT:
        ret = ops->unlinkat ? ops->unlinkat(ctx, work->dir, work->name,
                                            work->flags) : -1;
        if (!ops->unlinkat) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_RENAMEAT:
        ret = ops->renameat ? ops->renameat(ctx, work->dir, work->name,
                                            work->new_dir,
                                            work->new_name) : -1;
        if (!ops->renameat) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_LINK:
        ret = ops->link ? ops->link(ctx, work->path, work->dir,
                                    work->name) : -1;
        if (!ops->link) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_FGETXATTR:
        ret = ops->fgetxattr ?
              ops->fgetxattr(ctx, work->fid_type, work->open,
                             work->xattr_name, work->xattr_value,
                             work->length) : -1;
        if (!ops->fgetxattr) {
            errno = EOPNOTSUPP;
        }
        break;
    case BACKEND_FSETXATTR:
        ret = ops->fsetxattr ?
              ops->fsetxattr(ctx, work->fid_type, work->open,
                             work->xattr_name, work->xattr_value,
                             work->length, work->xattr_flags) : -1;
        if (!ops->fsetxattr) {
            errno = EOPNOTSUPP;
        }
        break;
    default:
        g_assert_not_reached();
    }
    return ret < 0 ? -errno : ret;
}

static int coroutine_fn run_backend(BackendWork *work)
{
    return thread_pool_submit_co(backend_worker, work);
}

static int coroutine_fn co_name_to_path(Nfs2Server *server,
                                        V9fsPath *dir, const char *name,
                                        V9fsPath *path)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_NAME_TO_PATH,
        .backend = &server->backend,
        .name = name,
        .path = path,
    };
    int ret;

    if (dir) {
        path_copy(&copy, dir);
        work.dir = &copy;
    }
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_lstat(Nfs2Server *server, V9fsPath *path,
                                 struct stat *st)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_LSTAT, .backend = &server->backend, .st = st,
    };
    int ret;

    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_readlink(Nfs2Server *server, V9fsPath *path,
                                    char *buf, size_t len)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_READLINK, .backend = &server->backend,
        .buffer = buf, .length = len,
    };
    int ret;

    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_open(Nfs2Server *server, V9fsPath *path,
                                V9fsFidOpenState *state)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_OPEN, .backend = &server->backend, .open = state,
    };
    int ret;

    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_open_flags(Nfs2Server *server, V9fsPath *path,
                                      int flags, V9fsFidOpenState *state)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_OPEN, .backend = &server->backend, .open = state,
        .flags = flags,
    };
    int ret;

    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_fstat(Nfs2Server *server, int fid_type,
                                 V9fsFidOpenState *state, struct stat *st)
{
    BackendWork work = {
        .op = BACKEND_FSTAT, .backend = &server->backend, .open = state,
        .fid_type = fid_type, .st = st,
    };

    return run_backend(&work);
}

static int coroutine_fn co_simple_open(Nfs2Server *server, BackendOp op,
                                       V9fsFidOpenState *state)
{
    BackendWork work = {
        .op = op, .backend = &server->backend, .open = state,
    };
    return run_backend(&work);
}

static int coroutine_fn co_pread(Nfs2Server *server,
                                 V9fsFidOpenState *state, void *buf,
                                 size_t len, uint64_t offset)
{
    struct iovec iov = { .iov_base = buf, .iov_len = len };
    BackendWork work = {
        .op = BACKEND_PREADV, .backend = &server->backend, .open = state,
        .iov = &iov, .offset = offset,
    };

    if (offset > INT64_MAX) {
        return -EFBIG;
    }
    return run_backend(&work);
}

static int coroutine_fn co_pwrite(Nfs2Server *server,
                                  V9fsFidOpenState *state, const void *buf,
                                  size_t len, uint64_t offset)
{
    struct iovec iov = { .iov_base = (void *)buf, .iov_len = len };
    BackendWork work = {
        .op = BACKEND_PWRITEV, .backend = &server->backend, .open = state,
        .iov = &iov, .offset = offset,
    };

    if (offset > INT64_MAX || len > INT64_MAX - offset) {
        return -EFBIG;
    }
    return run_backend(&work);
}

static int coroutine_fn co_fsync_type(Nfs2Server *server, int fid_type,
                                      V9fsFidOpenState *state)
{
    BackendWork work = {
        .op = BACKEND_FSYNC, .backend = &server->backend, .open = state,
        .fid_type = fid_type,
    };

    return run_backend(&work);
}

static int coroutine_fn co_fsync(Nfs2Server *server,
                                 V9fsFidOpenState *state)
{
    return co_fsync_type(server, P9_FID_FILE, state);
}

static int coroutine_fn co_opendir(Nfs2Server *server, V9fsPath *path,
                                   V9fsFidOpenState *state)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_OPENDIR, .backend = &server->backend, .open = state,
    };
    int ret;

    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_path_change(Nfs2Server *server, BackendOp op,
                                       V9fsPath *path, mode_t mode,
                                       uint64_t size,
                                       const struct timespec times[2])
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = op, .backend = &server->backend,
        .offset = size,
        .cred = { .fc_uid = -1, .fc_gid = -1, .fc_mode = mode,
                  .fc_rdev = -1 },
    };
    int ret;

    if (size > INT64_MAX) {
        return -EFBIG;
    }
    if (times) {
        memcpy(work.times, times, sizeof(work.times));
    }
    path_copy(&copy, path);
    work.path = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_create_open(Nfs2Server *server, V9fsPath *dir,
                                       const char *name, int flags,
                                       mode_t mode, uid_t uid, gid_t gid,
                                       V9fsFidOpenState *state)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_OPEN2, .backend = &server->backend, .name = name,
        .flags = flags, .open = state,
        .cred = { .fc_uid = uid, .fc_gid = gid, .fc_mode = mode,
                  .fc_rdev = -1 },
    };
    int ret;

    path_copy(&copy, dir);
    work.dir = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_create_node(Nfs2Server *server, BackendOp op,
                                       V9fsPath *dir, const char *name,
                                       const char *target, mode_t mode,
                                       dev_t rdev, uid_t uid, gid_t gid)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = op, .backend = &server->backend, .name = name,
        .target = target,
        .cred = { .fc_uid = uid, .fc_gid = gid, .fc_mode = mode,
                  .fc_rdev = rdev },
    };
    int ret;

    path_copy(&copy, dir);
    work.dir = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_unlink(Nfs2Server *server, V9fsPath *dir,
                                  const char *name, bool directory)
{
    V9fsPath copy = { 0 };
    BackendWork work = {
        .op = BACKEND_UNLINKAT, .backend = &server->backend, .name = name,
        .flags = directory ? AT_REMOVEDIR : 0,
    };
    int ret;

    path_copy(&copy, dir);
    work.dir = &copy;
    ret = run_backend(&work);
    path_clear(&copy);
    return ret;
}

static int coroutine_fn co_rename(Nfs2Server *server, V9fsPath *old_dir,
                                  const char *old_name, V9fsPath *new_dir,
                                  const char *new_name)
{
    V9fsPath old_copy = { 0 }, new_copy = { 0 };
    BackendWork work = {
        .op = BACKEND_RENAMEAT, .backend = &server->backend,
        .name = old_name, .new_name = new_name,
    };
    int ret;

    path_copy(&old_copy, old_dir);
    path_copy(&new_copy, new_dir);
    work.dir = &old_copy;
    work.new_dir = &new_copy;
    ret = run_backend(&work);
    path_clear(&old_copy);
    path_clear(&new_copy);
    return ret;
}

static int coroutine_fn co_link(Nfs2Server *server, V9fsPath *old_path,
                                V9fsPath *new_dir, const char *new_name)
{
    V9fsPath old_copy = { 0 }, dir_copy = { 0 };
    BackendWork work = {
        .op = BACKEND_LINK, .backend = &server->backend, .name = new_name,
    };
    int ret;

    path_copy(&old_copy, old_path);
    path_copy(&dir_copy, new_dir);
    work.path = &old_copy;
    work.dir = &dir_copy;
    ret = run_backend(&work);
    path_clear(&old_copy);
    path_clear(&dir_copy);
    return ret;
}

static int coroutine_fn co_fgetxattr(Nfs2Server *server,
                                     V9fsFidOpenState *state,
                                     const char *name, void *value,
                                     size_t length)
{
    BackendWork work = {
        .op = BACKEND_FGETXATTR, .backend = &server->backend,
        .open = state, .fid_type = P9_FID_FILE,
        .xattr_name = name, .xattr_value = value, .length = length,
    };

    return run_backend(&work);
}

static int coroutine_fn co_fsetxattr(Nfs2Server *server,
                                     V9fsFidOpenState *state,
                                     const char *name, void *value,
                                     size_t length, int flags)
{
    BackendWork work = {
        .op = BACKEND_FSETXATTR, .backend = &server->backend,
        .open = state, .fid_type = P9_FID_FILE,
        .xattr_name = name, .xattr_value = value, .length = length,
        .xattr_flags = flags,
    };

    return run_backend(&work);
}

static uint32_t clamp_u32(uint64_t value)
{
    return MIN(value, UINT32_MAX);
}

static guint hash_u64(uint64_t value)
{
    return value ^ (value >> 32);
}

static guint identity_hash(gconstpointer opaque)
{
    const NfsIdentityKey *key = opaque;

    return hash_u64(key->device) ^ (hash_u64(key->inode) * 33);
}

static gboolean identity_equal(gconstpointer left, gconstpointer right)
{
    const NfsIdentityKey *a = left;
    const NfsIdentityKey *b = right;

    return a->device == b->device && a->inode == b->inode;
}

static guint cookie_hash(gconstpointer opaque)
{
    const NfsCookieKey *key = opaque;

    return hash_u64(key->directory) ^
           (hash_u64(key->backend_cookie) * 33);
}

static gboolean cookie_equal(gconstpointer left, gconstpointer right)
{
    const NfsCookieKey *a = left;
    const NfsCookieKey *b = right;

    return a->directory == b->directory &&
           a->backend_cookie == b->backend_cookie;
}

static bool identity_id(Nfs2Server *server, const struct stat *st,
                        bool create, uint32_t *id)
{
    NfsIdentityKey lookup = { .device = st->st_dev, .inode = st->st_ino };
    gpointer value = g_hash_table_lookup(server->identities, &lookup);

    if (value) {
        *id = GPOINTER_TO_UINT(value);
        return true;
    }
    if (!create ||
        g_hash_table_size(server->identities) >= NFS2_MAX_IDENTITIES ||
        server->next_identity == 0) {
        return false;
    }
    NfsIdentityKey *key = g_new(NfsIdentityKey, 1);

    *key = lookup;
    *id = server->next_identity++;
    g_hash_table_insert(server->identities, key, GUINT_TO_POINTER(*id));
    return true;
}

static bool cookie_to_wire(Nfs2Server *server, uint64_t directory,
                           uint64_t backend_cookie, uint32_t *wire_cookie)
{
    NfsCookieKey lookup = {
        .directory = directory, .backend_cookie = backend_cookie,
    };
    gpointer value;

    value = g_hash_table_lookup(server->cookies_by_backend, &lookup);
    if (value) {
        *wire_cookie = GPOINTER_TO_UINT(value);
        return true;
    }
    if (g_hash_table_size(server->cookies_by_backend) >= NFS2_MAX_COOKIES ||
        server->next_cookie == 0) {
        return false;
    }
    NfsCookieKey *key = g_new(NfsCookieKey, 1);

    *key = lookup;
    *wire_cookie = server->next_cookie++;
    g_hash_table_insert(server->cookies_by_backend, key,
                        GUINT_TO_POINTER(*wire_cookie));
    g_hash_table_insert(server->cookies_by_wire,
                        GUINT_TO_POINTER(*wire_cookie), key);
    return true;
}

static bool cookie_from_wire(Nfs2Server *server, uint64_t directory,
                             uint32_t wire_cookie, uint64_t *backend_cookie)
{
    NfsCookieKey *key;

    if (wire_cookie == 0) {
        *backend_cookie = 0;
        return true;
    }
    key = g_hash_table_lookup(server->cookies_by_wire,
                              GUINT_TO_POINTER(wire_cookie));
    if (!key || key->directory != directory) {
        return false;
    }
    *backend_cookie = key->backend_cookie;
    return true;
}

static uint32_t v2_status(int error)
{
    if (error >= 0) {
        return NFS2_NFS_OK;
    }
    switch (-error) {
    case EPERM: return NFS2_NFSERR_PERM;
    case ENOENT: return NFS2_NFSERR_NOENT;
    case EIO: return NFS2_NFSERR_IO;
    case ENXIO: return NFS2_NFSERR_NXIO;
    case EACCES: return NFS2_NFSERR_ACCES;
    case EEXIST: return NFS2_NFSERR_EXIST;
    case ENODEV: return NFS2_NFSERR_NODEV;
    case ENOTDIR: return NFS2_NFSERR_NOTDIR;
    case EISDIR: return NFS2_NFSERR_ISDIR;
    case EFBIG: return NFS2_NFSERR_FBIG;
    case ENOSPC: return NFS2_NFSERR_NOSPC;
    case EROFS: return NFS2_NFSERR_ROFS;
    case ENAMETOOLONG: return NFS2_NFSERR_NAMETOOLONG;
    case ENOTEMPTY: return NFS2_NFSERR_NOTEMPTY;
#ifdef EDQUOT
    case EDQUOT: return NFS2_NFSERR_DQUOT;
#endif
    case ESTALE: return NFS2_NFSERR_STALE;
    default: return NFS2_NFSERR_IO;
    }
}

static uint32_t v3_status(int error)
{
    if (error >= 0) {
        return NFS3_OK;
    }
    switch (-error) {
    case EPERM: return NFS3ERR_PERM;
    case ENOENT: return NFS3ERR_NOENT;
    case EIO: return NFS3ERR_IO;
    case ENXIO: return NFS3ERR_NXIO;
    case EACCES: return NFS3ERR_ACCES;
    case EEXIST: return NFS3ERR_EXIST;
    case EXDEV: return NFS3ERR_XDEV;
    case ENODEV: return NFS3ERR_NODEV;
    case ENOTDIR: return NFS3ERR_NOTDIR;
    case EISDIR: return NFS3ERR_ISDIR;
    case EINVAL: return NFS3ERR_INVAL;
    case EFBIG: return NFS3ERR_FBIG;
    case ENOSPC: return NFS3ERR_NOSPC;
    case EROFS: return NFS3ERR_ROFS;
    case EMLINK: return NFS3ERR_MLINK;
    case ENAMETOOLONG: return NFS3ERR_NAMETOOLONG;
    case ENOTEMPTY: return NFS3ERR_NOTEMPTY;
#ifdef EDQUOT
    case EDQUOT: return NFS3ERR_DQUOT;
#endif
    case ESTALE: return NFS3ERR_STALE;
    case EOPNOTSUPP: return NFS3ERR_NOTSUPP;
    case EBADMSG: return NFS3ERR_BAD_COOKIE;
    case EMSGSIZE: return NFS3ERR_TOOSMALL;
    default: return NFS3ERR_SERVERFAULT;
    }
}

static uint32_t file_type(mode_t mode, bool v3)
{
    if (S_ISREG(mode)) {
        return NFS2_NFREG;
    }
    if (S_ISDIR(mode)) {
        return NFS2_NFDIR;
    }
    if (S_ISBLK(mode)) {
        return NFS2_NFBLK;
    }
    if (S_ISCHR(mode)) {
        return NFS2_NFCHR;
    }
    if (S_ISLNK(mode)) {
        return NFS2_NFLNK;
    }
    if (v3 && S_ISSOCK(mode)) {
        return 6;
    }
    if (v3 && S_ISFIFO(mode)) {
        return 7;
    }
    return NFS2_NFNON;
}

static bool put_u64(Nfs2XdrWriter *w, uint64_t value)
{
    return nfs2_xdr_put_u32(w, value >> 32) &&
           nfs2_xdr_put_u32(w, value);
}

static bool get_u64(Nfs2XdrReader *r, uint64_t *value)
{
    uint32_t high, low;

    if (!nfs2_xdr_u32(r, &high) || !nfs2_xdr_u32(r, &low)) {
        return false;
    }
    *value = ((uint64_t)high << 32) | low;
    return true;
}

static bool put_v2_attr(Nfs2Server *server, Nfs2XdrWriter *w,
                        const struct stat *st)
{
    uint64_t size = st->st_size < 0 ? 0 : st->st_size;
    uint64_t blocks = st->st_blocks < 0 ? 0 : st->st_blocks;
    uint32_t identity;

    return identity_id(server, st, false, &identity) &&
           nfs2_xdr_put_u32(w, file_type(st->st_mode, false)) &&
           nfs2_xdr_put_u32(w, st->st_mode) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_nlink)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_uid)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_gid)) &&
           nfs2_xdr_put_u32(w, clamp_u32(size)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_blksize)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_rdev)) &&
           nfs2_xdr_put_u32(w, clamp_u32(blocks)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_dev)) &&
           nfs2_xdr_put_u32(w, identity) &&
           nfs2_xdr_put_u32(w, clamp_u32(MAX(st->st_atim.tv_sec, 0))) &&
           nfs2_xdr_put_u32(w, st->st_atim.tv_nsec / 1000) &&
           nfs2_xdr_put_u32(w, clamp_u32(MAX(st->st_mtim.tv_sec, 0))) &&
           nfs2_xdr_put_u32(w, st->st_mtim.tv_nsec / 1000) &&
           nfs2_xdr_put_u32(w, clamp_u32(MAX(st->st_ctim.tv_sec, 0))) &&
           nfs2_xdr_put_u32(w, st->st_ctim.tv_nsec / 1000);
}

static bool put_v3_attr(Nfs2XdrWriter *w, const struct stat *st)
{
    uint64_t size = st->st_size < 0 ? 0 : st->st_size;
    uint64_t used = st->st_blocks < 0 ? 0 : (uint64_t)st->st_blocks * 512;

    return nfs2_xdr_put_u32(w, file_type(st->st_mode, true)) &&
           nfs2_xdr_put_u32(w, st->st_mode & 07777) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_nlink)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_uid)) &&
           nfs2_xdr_put_u32(w, clamp_u32(st->st_gid)) &&
           put_u64(w, size) && put_u64(w, used) &&
           nfs2_xdr_put_u32(w, major(st->st_rdev)) &&
           nfs2_xdr_put_u32(w, minor(st->st_rdev)) &&
           put_u64(w, st->st_dev) && put_u64(w, st->st_ino) &&
           nfs2_xdr_put_u32(w, clamp_u32(MAX(st->st_atim.tv_sec, 0))) &&
           nfs2_xdr_put_u32(w, st->st_atim.tv_nsec) &&
           nfs2_xdr_put_u32(w, clamp_u32(MAX(st->st_mtim.tv_sec, 0))) &&
           nfs2_xdr_put_u32(w, st->st_mtim.tv_nsec) &&
           nfs2_xdr_put_u32(w, clamp_u32(MAX(st->st_ctim.tv_sec, 0))) &&
           nfs2_xdr_put_u32(w, st->st_ctim.tv_nsec);
}

static bool put_post_attr(Nfs2XdrWriter *w, const struct stat *st, bool valid)
{
    return nfs2_xdr_put_u32(w, valid) && (!valid || put_v3_attr(w, st));
}

static bool put_wcc(Nfs2XdrWriter *w, const struct stat *before,
                    bool before_valid, const struct stat *after,
                    bool after_valid)
{
    uint64_t size = before_valid && before->st_size > 0 ? before->st_size : 0;

    return nfs2_xdr_put_u32(w, before_valid) &&
           (!before_valid ||
            (put_u64(w, size) &&
             nfs2_xdr_put_u32(w, clamp_u32(MAX(before->st_mtim.tv_sec, 0))) &&
             nfs2_xdr_put_u32(w, before->st_mtim.tv_nsec) &&
             nfs2_xdr_put_u32(w, clamp_u32(MAX(before->st_ctim.tv_sec, 0))) &&
             nfs2_xdr_put_u32(w, before->st_ctim.tv_nsec))) &&
           put_post_attr(w, after, after_valid);
}

typedef struct NfsSetAttr {
    bool mode_set;
    mode_t mode;
    bool size_set;
    uint64_t size;
    bool times_set;
    struct timespec times[2];
} NfsSetAttr;

static bool decode_set_bool(Nfs2XdrReader *r, bool *set)
{
    uint32_t value;

    if (!nfs2_xdr_u32(r, &value) || value > 1) {
        return false;
    }
    *set = value;
    return true;
}

static bool decode_v3_time(Nfs2XdrReader *r, struct timespec *time,
                           bool *set)
{
    uint32_t how, seconds, nanoseconds;

    if (!nfs2_xdr_u32(r, &how) || how > 2) {
        return false;
    }
    if (how == 0) {
        time->tv_nsec = UTIME_OMIT;
        return true;
    }
    *set = true;
    if (how == 1) {
        time->tv_nsec = UTIME_NOW;
        return true;
    }
    if (!nfs2_xdr_u32(r, &seconds) ||
        !nfs2_xdr_u32(r, &nanoseconds) || nanoseconds >= 1000000000) {
        return false;
    }
    time->tv_sec = seconds;
    time->tv_nsec = nanoseconds;
    return true;
}

static bool decode_v3_sattr(Nfs2XdrReader *r, NfsSetAttr *attr)
{
    bool set;
    uint32_t value;

    memset(attr, 0, sizeof(*attr));
    attr->times[0].tv_nsec = attr->times[1].tv_nsec = UTIME_OMIT;
    if (!decode_set_bool(r, &set) ||
        (set && !nfs2_xdr_u32(r, &value))) {
        return false;
    }
    if (set) {
        attr->mode_set = true;
        attr->mode = value & 07777;
    }
    /* UID and GID are deliberately decoded and ignored. */
    for (unsigned int i = 0; i < 2; i++) {
        if (!decode_set_bool(r, &set) ||
            (set && !nfs2_xdr_u32(r, &value))) {
            return false;
        }
    }
    if (!decode_set_bool(r, &set) ||
        (set && !get_u64(r, &attr->size))) {
        return false;
    }
    attr->size_set = set;
    return decode_v3_time(r, &attr->times[0], &attr->times_set) &&
           decode_v3_time(r, &attr->times[1], &attr->times_set);
}

static bool decode_v2_sattr(Nfs2XdrReader *r, NfsSetAttr *attr)
{
    uint32_t mode, ignored, size, atime_sec, atime_usec;
    uint32_t mtime_sec, mtime_usec;

    memset(attr, 0, sizeof(*attr));
    attr->times[0].tv_nsec = attr->times[1].tv_nsec = UTIME_OMIT;
    if (!nfs2_xdr_u32(r, &mode) || !nfs2_xdr_u32(r, &ignored) ||
        !nfs2_xdr_u32(r, &ignored) || !nfs2_xdr_u32(r, &size) ||
        !nfs2_xdr_u32(r, &atime_sec) ||
        !nfs2_xdr_u32(r, &atime_usec) ||
        !nfs2_xdr_u32(r, &mtime_sec) ||
        !nfs2_xdr_u32(r, &mtime_usec)) {
        return false;
    }
    if (atime_usec >= 1000000 && atime_usec != UINT32_MAX) {
        return false;
    }
    if (mtime_usec >= 1000000 && mtime_usec != UINT32_MAX) {
        return false;
    }
    attr->mode_set = mode != UINT32_MAX;
    attr->mode = mode & 07777;
    attr->size_set = size != UINT32_MAX;
    attr->size = size;
    if (atime_sec != UINT32_MAX && atime_usec != UINT32_MAX) {
        attr->times_set = true;
        attr->times[0].tv_sec = atime_sec;
        attr->times[0].tv_nsec = atime_usec * 1000;
    }
    if (mtime_sec != UINT32_MAX && mtime_usec != UINT32_MAX) {
        attr->times_set = true;
        attr->times[1].tv_sec = mtime_sec;
        attr->times[1].tv_nsec = mtime_usec * 1000;
    }
    return true;
}

static int coroutine_fn apply_sattr(Nfs2Server *server, V9fsPath *path,
                                    const NfsSetAttr *attr)
{
    int ret = 0;

    if (attr->mode_set) {
        struct stat current;

        ret = co_lstat(server, path, &current);
        if (ret >= 0) {
            ret = co_path_change(server, BACKEND_CHMOD, path,
                                 (current.st_mode & S_IFMT) | attr->mode,
                                 0, NULL);
        }
    }
    if (ret >= 0 && attr->size_set) {
        ret = co_path_change(server, BACKEND_TRUNCATE, path, 0, attr->size,
                             NULL);
    }
    if (ret >= 0 && attr->times_set) {
        ret = co_path_change(server, BACKEND_UTIMENSAT, path, 0, 0,
                             attr->times);
    }
    return ret;
}

static bool decode_v3_handle(Nfs2XdrReader *r, Nfs2FileHandle *handle,
                             bool require_empty)
{
    const uint8_t *bytes;
    size_t length;

    if (!nfs2_xdr_counted_opaque(r, &bytes, &length, NFS3_FHSIZE) ||
        length != NFS3_FHSIZE || (require_empty && !nfs2_xdr_reader_empty(r))) {
        return false;
    }
    memcpy(handle->bytes, bytes, sizeof(handle->bytes));
    return true;
}

static int coroutine_fn validate_handle_identity(Nfs2Server *server,
                                                 const Nfs2FileHandle *handle,
                                                 V9fsPath *path,
                                                 struct stat *st)
{
    int ret = co_lstat(server, path, st);
    uint32_t identity;

    if (ret == -ENOENT || ret == -ENOTDIR) {
        return -ESTALE;
    }
    if (ret >= 0 &&
        (!identity_id(server, st, false, &identity) ||
         identity != ldq_be_p(handle->bytes + 8))) {
        return -ESTALE;
    }
    return ret;
}

static bool stat_matches_handle(Nfs2Server *server,
                                const Nfs2FileHandle *handle,
                                const struct stat *st)
{
    uint32_t identity;

    return identity_id(server, st, false, &identity) &&
           identity == ldq_be_p(handle->bytes + 8);
}

static int coroutine_fn resolve_backend_path(Nfs2Server *server,
                                             const V9fsPath *absolute,
                                             V9fsPath *backend)
{
    g_auto(GStrv) components = NULL;
    V9fsPath current = { 0 };
    int ret;

    ret = co_name_to_path(server, NULL, "/", &current);
    if (ret < 0) {
        return ret;
    }
    components = g_strsplit(absolute->data + 1, "/", -1);
    for (size_t i = 0; components[i] && components[i][0]; i++) {
        V9fsPath next = { 0 };

        ret = co_name_to_path(server, &current, components[i], &next);
        path_clear(&current);
        if (ret < 0) {
            return ret;
        }
        current = next;
    }
    *backend = current;
    return 0;
}

static int coroutine_fn resolve_handle(Nfs2Server *server,
                                       const Nfs2FileHandle *handle,
                                       V9fsPath *absolute,
                                       V9fsPath *backend,
                                       struct stat *st)
{
    g_autoptr(GPtrArray) aliases =
        nfs2_handle_paths_snapshot(server->handles, handle);

    if (!aliases) {
        return -ESTALE;
    }
    for (size_t alias = 0; alias < aliases->len; alias++) {
        V9fsPath *snapshot = g_ptr_array_index(aliases, alias);
        int ret;

        path_clear(absolute);
        path_clear(backend);
        path_copy(absolute, snapshot);
        ret = resolve_backend_path(server, absolute, backend);
        if (ret >= 0) {
            ret = validate_handle_identity(server, handle, backend, st);
        }
        if (ret >= 0) {
            return 0;
        }
        if (ret != -ESTALE && ret != -ENOENT && ret != -ENOTDIR) {
            return ret;
        }
    }
    return -ESTALE;
}

static char *child_absolute(const char *dir, const char *name)
{
    g_autofree char *joined = g_build_filename(dir, name, NULL);
    char *canonical = g_canonicalize_filename(joined, "/");

    if (canonical[0] != '/') {
        g_free(canonical);
        return NULL;
    }
    return canonical;
}

static int make_handle(Nfs2Server *server, const char *path,
                       const struct stat *st,
                       const Nfs2HandlePathState *path_state,
                       Nfs2FileHandle *handle)
{
    uint32_t identity;

    if (!identity_id(server, st, false, &identity)) {
        if (path_state &&
            !nfs2_handle_path_state_allows(server->handles, path, path_state,
                                           0)) {
            return -ESTALE;
        }
        if (!identity_id(server, st, true, &identity)) {
            return -ENOSPC;
        }
    }
    if (path_state &&
        !nfs2_handle_path_state_allows(server->handles, path, path_state,
                                       identity)) {
        return -ESTALE;
    }
    return nfs2_handle_create(server->handles, identity, path, handle, NULL) ?
           0 : -EIO;
}

static bool write_nfs_status(Nfs2XdrWriter *w, uint32_t status)
{
    return nfs2_xdr_put_u32(w, status);
}

static int service_program(Nfs2Service service)
{
    switch (service) {
    case NFS2_SERVICE_PORTMAP: return NFS2_PMAP_PROGRAM;
    case NFS2_SERVICE_MOUNT: return NFS2_MOUNT_PROGRAM;
    case NFS2_SERVICE_NFS: return NFS2_NFS_PROGRAM;
    default: return -1;
    }
}

static bool service_version(Nfs2Service service, uint32_t version)
{
    if (service == NFS2_SERVICE_PORTMAP) {
        return version == NFS2_PMAP_VERSION;
    }
    if (service == NFS2_SERVICE_MOUNT) {
        return version == NFS2_MOUNT_VERSION || version == MOUNT3_VERSION;
    }
    return version == NFS2_NFS_VERSION || version == NFS3_VERSION;
}

static bool body_empty(const Nfs2RpcCall *call)
{
    return nfs2_xdr_reader_empty(&call->body);
}

static int port_for_mapping(const Nfs2PmapGetPortArgs *args)
{
    if (args->protocol != NFS2_IPPROTO_UDP) {
        return 0;
    }
    if (args->program == NFS2_PMAP_PROGRAM &&
        args->version == NFS2_PMAP_VERSION) {
        return NFS2_PORT_PMAP;
    }
    if (args->program == NFS2_MOUNT_PROGRAM &&
        (args->version == NFS2_MOUNT_VERSION ||
         args->version == MOUNT3_VERSION)) {
        return NFS2_PORT_MOUNT;
    }
    if (args->program == NFS2_NFS_PROGRAM &&
        (args->version == NFS2_NFS_VERSION || args->version == NFS3_VERSION)) {
        return NFS2_PORT_NFS;
    }
    return 0;
}

static bool coroutine_fn dispatch_portmap(Nfs2RpcCall *call,
                                          Nfs2XdrWriter *w)
{
    Nfs2PmapGetPortArgs args;

    switch (call->procedure) {
    case NFS2_PMAP_NULL:
        return body_empty(call) ? nfs2_rpc_reply_success(w, call->xid) :
                                  nfs2_rpc_reply_garbage_args(w, call->xid);
    case NFS2_PMAP_GETPORT:
        if (!nfs2_xdr_decode_pmap_getport(&call->body, &args)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        return nfs2_rpc_reply_success(w, call->xid) &&
               nfs2_xdr_put_u32(w, port_for_mapping(&args));
    default:
        return nfs2_rpc_reply_proc_unavail(w, call->xid);
    }
}

static bool coroutine_fn dispatch_mount(Nfs2Server *server,
                                        Nfs2RpcCall *call,
                                        Nfs2XdrWriter *w)
{
    Nfs2MountMntArgs args;

    switch (call->procedure) {
    case NFS2_MOUNT_NULL:
        return body_empty(call) ? nfs2_rpc_reply_success(w, call->xid) :
                                  nfs2_rpc_reply_garbage_args(w, call->xid);
    case NFS2_MOUNT_MNT:
        if (!nfs2_xdr_decode_mount_mnt(&call->body, &args)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        if (!nfs2_rpc_reply_success(w, call->xid)) {
            return false;
        }
        if (strcmp(args.path, "/")) {
            return write_nfs_status(w, NFS2_NFSERR_ACCES);
        }
        if (!write_nfs_status(w, NFS2_NFS_OK)) {
            return false;
        }
        if (call->version == NFS2_MOUNT_VERSION) {
            return nfs2_xdr_put_opaque(w, server->root_handle.bytes,
                                       sizeof(server->root_handle.bytes));
        }
        return nfs2_xdr_put_counted_opaque(w, server->root_handle.bytes,
                                           sizeof(server->root_handle.bytes),
                                           NFS3_FHSIZE) &&
               nfs2_xdr_put_u32(w, 1) &&
               nfs2_xdr_put_u32(w, NFS2_AUTH_SYS);
    default:
        return nfs2_rpc_reply_proc_unavail(w, call->xid);
    }
}

static bool decode_v2_handle(Nfs2RpcCall *call, Nfs2FileHandle *handle)
{
    return nfs2_xdr_decode_fhandle(&call->body, handle);
}

static bool decode_v2_handle_partial(Nfs2XdrReader *r,
                                     Nfs2FileHandle *handle)
{
    return nfs2_xdr_opaque(r, handle->bytes, sizeof(handle->bytes));
}

static bool coroutine_fn reply_getattr(Nfs2Server *server,
                                       Nfs2RpcCall *call,
                                       Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    int ret;
    bool ok;

    if (!(v3 ? decode_v3_handle(&call->body, &handle, true) :
               decode_v2_handle(call, &handle))) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && ret >= 0) {
        ok = v3 ? put_v3_attr(w, &st) : put_v2_attr(server, w, &st);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool decode_lookup(Nfs2RpcCall *call, bool v3, Nfs2FileHandle *dir,
                          char name[NFS2_MAX_NAME + 1])
{
    if (!v3) {
        Nfs2Diropargs args;

        if (!nfs2_xdr_decode_diropargs(&call->body, &args)) {
            return false;
        }
        *dir = args.dir;
        strcpy(name, args.name);
        return strcmp(name, ".") && strcmp(name, "..") &&
               !strchr(name, '/');
    }
    return decode_v3_handle(&call->body, dir, false) &&
           nfs2_xdr_string(&call->body, name, NFS2_MAX_NAME + 1,
                           NFS2_MAX_NAME) && name[0] && strcmp(name, ".") &&
           strcmp(name, "..") && !strchr(name, '/') &&
           nfs2_xdr_reader_empty(&call->body);
}

static bool coroutine_fn reply_lookup(Nfs2Server *server, Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle dir_handle, result_handle;
    char name[NFS2_MAX_NAME + 1];
    V9fsPath absolute = { 0 }, dir = { 0 }, child = { 0 };
    g_autofree char *child_path = NULL;
    Nfs2HandlePathState child_path_state;
    struct stat dir_st, st;
    int ret;
    bool ok;
    bool dir_resolved;
    bool child_revalidated = false;

    if (!decode_lookup(call, v3, &dir_handle, name)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &dir_handle, &absolute, &dir, &dir_st);
    dir_resolved = ret >= 0;
    if (ret >= 0 && !S_ISDIR(dir_st.st_mode)) {
        ret = -ENOTDIR;
    }
    if (ret >= 0) {
        child_path = child_absolute(absolute.data, name);
        if (!child_path) {
            ret = -EIO;
        } else {
            nfs2_handle_path_state(server->handles, child_path,
                                   &child_path_state);
        }
    }
    if (ret >= 0) {
        ret = co_name_to_path(server, &dir, name, &child);
    }
    if (ret >= 0) {
        ret = co_lstat(server, &child, &st);
    }
    if (dir_resolved) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &dir_handle, &dir,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
        } else if (ret >= 0) {
            dir_st = current_st;
        }
    }
    if (ret >= 0) {
        struct stat current_st;

        child_revalidated = true;
        ret = co_lstat(server, &child, &current_st);
        if (ret == -ENOENT || ret == -ENOTDIR) {
            ret = -ESTALE;
        }
        if (ret >= 0 && (current_st.st_dev != st.st_dev ||
                         current_st.st_ino != st.st_ino)) {
            ret = -ESTALE;
        }
        if (ret >= 0) {
            st = current_st;
        }
    }
    if (child_revalidated) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &dir_handle, &dir,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
        } else if (ret >= 0) {
            dir_st = current_st;
        }
    }
    if (ret >= 0) {
        ret = make_handle(server, child_path, &st, &child_path_state,
                          &result_handle);
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && ret >= 0) {
        if (v3) {
            ok = nfs2_xdr_put_counted_opaque(w, result_handle.bytes,
                                              sizeof(result_handle.bytes),
                                              NFS3_FHSIZE) &&
                 put_post_attr(w, &st, true) &&
                 put_post_attr(w, &dir_st, true);
        } else {
            ok = nfs2_xdr_put_opaque(w, result_handle.bytes,
                                     sizeof(result_handle.bytes)) &&
                 put_v2_attr(server, w, &st);
        }
    } else if (ok && v3) {
        ok = put_post_attr(w, &dir_st, false);
    }
    path_clear(&absolute);
    path_clear(&dir);
    path_clear(&child);
    return ok;
}

static bool coroutine_fn reply_access(Nfs2Server *server, Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    uint32_t requested;
    int ret;
    bool ok;

    if (!decode_v3_handle(&call->body, &handle, false) ||
        !nfs2_xdr_u32(&call->body, &requested) ||
        !nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    requested &= ~(uint32_t)(0x0004 | 0x0008 | 0x0010);
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3_status(ret)) &&
         put_post_attr(w, &st, ret >= 0);
    if (ok && ret >= 0) {
        ok = nfs2_xdr_put_u32(w, requested & 0x003f);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool coroutine_fn reply_readlink(Nfs2Server *server,
                                        Nfs2RpcCall *call,
                                        Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    char target[NFS2_MAX_PATH + 1];
    ssize_t target_length = -1;
    int ret;
    bool ok;
    bool readlink_attempted = false;

    if (!(v3 ? decode_v3_handle(&call->body, &handle, true) :
               decode_v2_handle(call, &handle))) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    if (ret >= 0 && !S_ISLNK(st.st_mode)) {
        ret = -EINVAL;
    }
    if (ret >= 0) {
        readlink_attempted = true;
        ret = co_readlink(server, &path, target, NFS2_MAX_PATH);
        target_length = ret;
    }
    if (readlink_attempted) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &handle, &path,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
        } else if (ret >= 0) {
            st = current_st;
        }
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_post_attr(w, &st, ret >= 0);
    }
    if (ok && ret >= 0) {
        ok = nfs2_xdr_put_counted_opaque(w, target, target_length,
                                          NFS2_MAX_PATH);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool decode_read(Nfs2RpcCall *call, bool v3, Nfs2FileHandle *handle,
                        uint64_t *offset, uint32_t *count)
{
    if (!v3) {
        Nfs2ReadArgs args;

        if (!nfs2_xdr_decode_readargs(&call->body, &args)) {
            return false;
        }
        *handle = args.file;
        *offset = args.offset;
        *count = args.count;
        return true;
    }
    return decode_v3_handle(&call->body, handle, false) &&
           get_u64(&call->body, offset) &&
           nfs2_xdr_u32(&call->body, count) && *count <= NFS2_MAX_DATA &&
           nfs2_xdr_reader_empty(&call->body);
}

static bool coroutine_fn reply_read(Nfs2Server *server, Nfs2RpcCall *call,
                                    Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    V9fsFidOpenState open = { 0 };
    uint8_t data[NFS2_MAX_DATA];
    uint64_t offset;
    uint32_t count;
    int ret, close_ret = 0;
    bool opened = false, open_attempted = false, open_bound = false;
    bool ok;

    if (!decode_read(call, v3, &handle, &offset, &count)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    if (ret >= 0 && !S_ISREG(st.st_mode)) {
        ret = v3 ? -EINVAL : (S_ISDIR(st.st_mode) ? -EISDIR : -EINVAL);
    }
    if (ret >= 0) {
        open_attempted = true;
        ret = co_open(server, &path, &open);
        opened = ret >= 0;
    }
    if (ret >= 0) {
        struct stat opened_st;

        ret = co_fstat(server, P9_FID_FILE, &open, &opened_st);
        if (ret >= 0 && !stat_matches_handle(server, &handle, &opened_st)) {
            ret = -ESTALE;
        }
        if (ret >= 0) {
            st = opened_st;
            open_bound = true;
        }
    }
    if (open_attempted && !open_bound) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &handle, &path,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
        }
    }
    if (ret >= 0) {
        ret = co_pread(server, &open, data, count, offset);
        if (ret > count) {
            ret = -EIO;
        }
    }
    if (opened) {
        close_ret = co_simple_open(server, BACKEND_CLOSE, &open);
        if (ret >= 0 && close_ret < 0) {
            ret = close_ret;
        }
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && ret >= 0) {
        if (v3) {
            ok = put_post_attr(w, &st, true) &&
                 nfs2_xdr_put_u32(w, ret) &&
                 nfs2_xdr_put_u32(w, offset + ret >= (uint64_t)st.st_size) &&
                 nfs2_xdr_put_counted_opaque(w, data, ret, NFS2_MAX_DATA);
        } else {
            ok = put_v2_attr(server, w, &st) &&
                 nfs2_xdr_put_counted_opaque(w, data, ret, NFS2_MAX_DATA);
        }
    } else if (ok && v3) {
        ok = put_post_attr(w, &st, false);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool decode_readdir(Nfs2RpcCall *call, bool v3,
                           Nfs2FileHandle *handle, uint64_t *cookie,
                           uint8_t verifier[NFS3_COOKIEVERFSIZE],
                           uint32_t *dircount, uint32_t *count, bool plus)
{
    if (v3) {
        if (!decode_v3_handle(&call->body, handle, false) ||
            !get_u64(&call->body, cookie) ||
            !nfs2_xdr_opaque(&call->body, verifier,
                             NFS3_COOKIEVERFSIZE)) {
            return false;
        }
        if (plus && (!nfs2_xdr_u32(&call->body, dircount) ||
                     *dircount > NFS2_MAX_DATA)) {
            return false;
        }
        return nfs2_xdr_u32(&call->body, count) &&
               *count <= NFS2_MAX_DATA &&
               nfs2_xdr_reader_empty(&call->body);
    }
    uint32_t cookie32;

    if (!nfs2_xdr_opaque(&call->body, handle->bytes, NFS2_FHSIZE) ||
        !nfs2_xdr_u32(&call->body, &cookie32) ||
        !nfs2_xdr_u32(&call->body, count) || *count < 8 ||
        *count > NFS2_MAX_DATA ||
        !nfs2_xdr_reader_empty(&call->body)) {
        return false;
    }
    *cookie = cookie32;
    memset(verifier, 0, NFS3_COOKIEVERFSIZE);
    *dircount = *count;
    return true;
}

static void directory_verifier(Nfs2Server *server, const struct stat *st,
                               uint8_t verifier[NFS3_COOKIEVERFSIZE])
{
    uint32_t identity = 0;
    uint64_t value;

    identity_id(server, st, false, &identity);
    value = identity ^ ((uint64_t)st->st_mtim.tv_sec << 32) ^
                     st->st_mtim.tv_nsec ^ ((uint64_t)st->st_ctim.tv_sec << 1);

    stq_be_p(verifier, value ? value : 1);
}

static bool coroutine_fn reply_readdir(Nfs2Server *server,
                                       Nfs2RpcCall *call,
                                       Nfs2XdrWriter *w, bool v3, bool plus)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat dir_st = { 0 };
    V9fsFidOpenState open = { 0 };
    uint64_t cookie = 0;
    uint64_t directory_identity;
    uint8_t supplied_verifier[NFS3_COOKIEVERFSIZE];
    uint8_t current_verifier[NFS3_COOKIEVERFSIZE] = { 0 };
    uint32_t dircount = 0, count;
    size_t dir_used = 0;
    int ret;
    bool opened = false, opendir_attempted = false, dir_bound = false;
    bool eof = false, emitted = false, ok;
    bool dir_attr_valid = false;
    bool seek_requested;

    if (!decode_readdir(call, v3, &handle, &cookie, supplied_verifier,
                        &dircount, &count, plus)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &dir_st);
    directory_identity = ldq_be_p(handle.bytes + 8);
    seek_requested = cookie != 0;
    dir_attr_valid = ret >= 0;
    if (ret >= 0 && !S_ISDIR(dir_st.st_mode)) {
        ret = -ENOTDIR;
    }
    if (ret >= 0) {
        V9fsPath copy = { 0 };
        BackendWork work = {
            .op = BACKEND_OPENDIR, .backend = &server->backend, .open = &open,
        };
        path_copy(&copy, &path);
        work.path = &copy;
        opendir_attempted = true;
        ret = run_backend(&work);
        path_clear(&copy);
        opened = ret >= 0;
    }
    if (ret >= 0) {
        struct stat opened_st;

        ret = co_fstat(server, P9_FID_DIR, &open, &opened_st);
        if (ret >= 0 && !stat_matches_handle(server, &handle, &opened_st)) {
            ret = -ESTALE;
        }
        if (ret >= 0) {
            dir_st = opened_st;
            dir_bound = true;
        } else {
            dir_attr_valid = false;
        }
    }
    if (opendir_attempted && !dir_bound) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &handle, &path,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
            dir_attr_valid = false;
        }
    }
    if (ret >= 0 && v3) {
        static const uint8_t zero[NFS3_COOKIEVERFSIZE];

        directory_verifier(server, &dir_st, current_verifier);
        if (memcmp(supplied_verifier, zero, sizeof(zero)) &&
            memcmp(supplied_verifier, current_verifier,
                   sizeof(current_verifier))) {
            ret = -EBADMSG;
        }
    }
    if (ret >= 0 && seek_requested) {
        if (!v3) {
            uint64_t backend_cookie;

            if (!cookie_from_wire(server, directory_identity, cookie,
                                  &backend_cookie)) {
                ret = -EINVAL;
            } else {
                cookie = backend_cookie;
            }
        }
    }
    if (ret >= 0 && seek_requested) {
        BackendWork work = {
            .op = BACKEND_SEEKDIR, .backend = &server->backend,
            .open = &open, .cookie = cookie,
        };

        ret = run_backend(&work);
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_post_attr(w, &dir_st, dir_attr_valid && ret >= 0) &&
             (ret < 0 || nfs2_xdr_put_opaque(w, current_verifier,
                                              sizeof(current_verifier)));
        if (ok && ret >= 0 &&
            nfs2_xdr_writer_size(w) - 28 + 8 > count) {
            ret = -EMSGSIZE;
        }
    }
    while (ok && ret >= 0) {
        BackendWork work = {
            .op = BACKEND_READDIR, .backend = &server->backend,
            .open = &open,
        };
        V9fsPath child = { 0 };
        g_autofree char *child_abs = NULL;
        struct stat child_st;
        Nfs2FileHandle child_handle;
        Nfs2HandlePathState child_path_state;
        uint32_t child_identity = 0;
        bool child_valid = false;
        size_t before = nfs2_xdr_writer_size(w), directory_end;

        ret = run_backend(&work);
        if (ret <= 0) {
            eof = work.eof;
            break;
        }
        if (!strcmp(work.dirent_name, ".") ||
            !strcmp(work.dirent_name, "..")) {
            continue;
        }
        if (plus) {
            child_abs = child_absolute(absolute.data, work.dirent_name);
            if (!child_abs) {
                ret = -EIO;
                break;
            }
            nfs2_handle_path_state(server->handles, child_abs,
                                   &child_path_state);
        }
        if (plus || !v3) {
            ret = co_name_to_path(server, &path, work.dirent_name, &child);
            if (ret >= 0) {
                ret = co_lstat(server, &child, &child_st);
            }
            if (ret >= 0 && !v3 &&
                !identity_id(server, &child_st, true, &child_identity)) {
                ret = -ENOSPC;
            }
            if (ret >= 0 && plus) {
                child_valid = make_handle(server, child_abs, &child_st,
                                          &child_path_state,
                                          &child_handle) >= 0;
            }
            path_clear(&child);
            if (ret < 0) {
                break;
            }
        }
        {
            BackendWork tell = {
                .op = BACKEND_TELLDIR, .backend = &server->backend,
                .open = &open,
            };

            ret = run_backend(&tell);
            if (ret < 0) {
                path_clear(&child);
                break;
            }
            if (!v3) {
                uint32_t wire_cookie;

                if (!cookie_to_wire(server, directory_identity, tell.cookie,
                                    &wire_cookie)) {
                    ret = -ENOSPC;
                    break;
                }
                cookie = wire_cookie;
            } else {
                cookie = tell.cookie;
            }
        }
        if (!nfs2_xdr_put_u32(w, 1) ||
            !(v3 ? put_u64(w, work.dirent_ino) :
                   nfs2_xdr_put_u32(w, child_identity)) ||
            !nfs2_xdr_put_counted_opaque(w, work.dirent_name,
                                          strlen(work.dirent_name),
                                          NFS2_MAX_NAME) ||
            !(v3 ? put_u64(w, cookie) :
                   nfs2_xdr_put_u32(w, clamp_u32(cookie))) ||
            (plus && (!put_post_attr(w, &child_st, child_valid) ||
                      !nfs2_xdr_put_u32(w, child_valid) ||
                      (child_valid &&
                       !nfs2_xdr_put_counted_opaque(w, child_handle.bytes,
                                                   sizeof(child_handle.bytes),
                                                   NFS3_FHSIZE))))) {
            w->cursor = w->start + before;
            eof = false;
            break;
        }
        directory_end = before + 4 + 8 + 4 +
                        QEMU_ALIGN_UP(strlen(work.dirent_name), 4) + 8;
        if (plus && dir_used + directory_end - before > dircount) {
            w->cursor = w->start + before;
            ret = emitted ? 0 : -EMSGSIZE;
            eof = false;
            break;
        }
        if (nfs2_xdr_writer_size(w) - 28 + 8 > count) {
            w->cursor = w->start + before;
            ret = (v3 && !emitted) ? -EMSGSIZE : 0;
            eof = false;
            break;
        }
        dir_used += directory_end - before;
        emitted = true;
    }
    if (opened && (plus || !v3)) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &handle, &path,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
            dir_attr_valid = false;
        } else if (ret >= 0) {
            dir_st = current_st;
        }
    }
    if (opened) {
        int close_ret = co_simple_open(server, BACKEND_CLOSEDIR, &open);
        if (ret >= 0 && close_ret < 0) {
            ret = close_ret;
        }
    }
    if (ret < 0) {
        size_t capacity = w->end - w->start;

        nfs2_xdr_writer_init(w, w->start, capacity);
        ok = nfs2_rpc_reply_success(w, call->xid) &&
             write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
        if (ok && v3) {
            ok = put_post_attr(w, &dir_st, dir_attr_valid);
        }
        path_clear(&absolute);
        path_clear(&path);
        return ok;
    }
    if (ok && ret >= 0) {
        ok = nfs2_xdr_put_u32(w, 0) && nfs2_xdr_put_u32(w, eof);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool coroutine_fn reply_statfs(Nfs2Server *server,
                                      Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    struct statfs fs;
    BackendWork work = {
        .op = BACKEND_STATFS, .backend = &server->backend, .stfs = &fs,
    };
    int ret;
    bool ok;
    bool statfs_attempted = false;

    if (!(v3 ? decode_v3_handle(&call->body, &handle, true) :
               decode_v2_handle(call, &handle))) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    if (ret >= 0) {
        V9fsPath copy = { 0 };
        path_copy(&copy, &path);
        work.path = &copy;
        statfs_attempted = true;
        ret = run_backend(&work);
        path_clear(&copy);
    }
    if (statfs_attempted) {
        struct stat current_st;
        int identity_ret = validate_handle_identity(server, &handle, &path,
                                                     &current_st);

        if (identity_ret < 0) {
            ret = identity_ret;
        } else if (ret >= 0) {
            st = current_st;
        }
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_post_attr(w, &st, ret >= 0);
    }
    if (ok && ret >= 0) {
        if (v3) {
            uint64_t block = MAX((uint64_t)fs.f_bsize, 1);
            ok = put_u64(w, (uint64_t)fs.f_blocks * block) &&
                 put_u64(w, (uint64_t)fs.f_bfree * block) &&
                 put_u64(w, (uint64_t)fs.f_bavail * block) &&
                 put_u64(w, MIN((uint64_t)fs.f_files, UINT64_MAX)) &&
                 put_u64(w, MIN((uint64_t)fs.f_ffree, UINT64_MAX)) &&
                 put_u64(w, MIN((uint64_t)fs.f_ffree, UINT64_MAX)) &&
                 nfs2_xdr_put_u32(w, 0);
        } else {
            ok = nfs2_xdr_put_u32(w, NFS2_MAX_DATA) &&
                 nfs2_xdr_put_u32(w, clamp_u32(fs.f_bsize)) &&
                 nfs2_xdr_put_u32(w, clamp_u32(fs.f_blocks)) &&
                 nfs2_xdr_put_u32(w, clamp_u32(fs.f_bfree)) &&
                 nfs2_xdr_put_u32(w, clamp_u32(fs.f_bavail));
        }
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool coroutine_fn reply_fsinfo(Nfs2Server *server,
                                      Nfs2RpcCall *call, Nfs2XdrWriter *w)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    int ret;
    bool ok;

    if (!decode_v3_handle(&call->body, &handle, true)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3_status(ret)) &&
         put_post_attr(w, &st, ret >= 0);
    if (ok && ret >= 0) {
        ok = nfs2_xdr_put_u32(w, NFS2_MAX_DATA) &&
             nfs2_xdr_put_u32(w, NFS2_MAX_DATA) &&
             nfs2_xdr_put_u32(w, 4096) &&
             nfs2_xdr_put_u32(w, NFS2_MAX_DATA) &&
             nfs2_xdr_put_u32(w, NFS2_MAX_DATA) &&
             nfs2_xdr_put_u32(w, 4096) &&
             nfs2_xdr_put_u32(w, NFS2_MAX_DATA) &&
             put_u64(w, INT64_MAX) &&
             nfs2_xdr_put_u32(w, 0) && nfs2_xdr_put_u32(w, 1) &&
             nfs2_xdr_put_u32(w, 0x0002 | 0x0008);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool coroutine_fn reply_pathconf(Nfs2Server *server,
                                        Nfs2RpcCall *call, Nfs2XdrWriter *w)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat st;
    int ret;
    bool ok;

    if (!decode_v3_handle(&call->body, &handle, true)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &st);
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3_status(ret)) &&
         put_post_attr(w, &st, ret >= 0);
    if (ok && ret >= 0) {
        ok = nfs2_xdr_put_u32(w, 32000) &&
             nfs2_xdr_put_u32(w, NFS2_MAX_NAME) &&
             nfs2_xdr_put_u32(w, 1) && nfs2_xdr_put_u32(w, 1) &&
             nfs2_xdr_put_u32(w, 0) && nfs2_xdr_put_u32(w, 1);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool coroutine_fn reply_setattr(Nfs2Server *server,
                                       Nfs2RpcCall *call,
                                       Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle handle;
    NfsSetAttr attr;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat before = { 0 }, after = { 0 };
    bool guard = false;
    uint32_t guard_sec = 0, guard_nsec = 0;
    int ret;
    bool ok, before_valid, after_valid = false;

    if (!(v3 ? decode_v3_handle(&call->body, &handle, false) :
               decode_v2_handle_partial(&call->body, &handle)) ||
        !(v3 ? decode_v3_sattr(&call->body, &attr) :
               decode_v2_sattr(&call->body, &attr))) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    if (v3 && (!decode_set_bool(&call->body, &guard) ||
               (guard && (!nfs2_xdr_u32(&call->body, &guard_sec) ||
                          !nfs2_xdr_u32(&call->body, &guard_nsec) ||
                          guard_nsec >= 1000000000)))) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    if (!nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &before);
    before_valid = ret >= 0;
    if (ret >= 0 && guard &&
        ((uint64_t)MAX(before.st_ctim.tv_sec, 0) != guard_sec ||
         before.st_ctim.tv_nsec != guard_nsec)) {
        ret = -EAGAIN;
    }
    if (ret >= 0) {
        ret = apply_sattr(server, &path, &attr);
    }
    if (before_valid) {
        int post_ret = validate_handle_identity(server, &handle, &path,
                                                &after);

        after_valid = post_ret >= 0;
        if (ret >= 0 && post_ret < 0) {
            ret = post_ret;
        }
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? (ret == -EAGAIN ? NFS3ERR_NOT_SYNC :
                                   v3_status(ret)) : v2_status(ret));
    if (ok && v3) {
        ok = put_wcc(w, &before, before_valid, &after, after_valid);
    } else if (ok && ret >= 0) {
        ok = put_v2_attr(server, w, &after);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool decode_name_args(Nfs2RpcCall *call, bool v3,
                             Nfs2FileHandle *dir,
                             char name[NFS2_MAX_NAME + 1])
{
    return (v3 ? decode_v3_handle(&call->body, dir, false) :
                 decode_v2_handle_partial(&call->body, dir)) &&
           nfs2_xdr_string(&call->body, name, NFS2_MAX_NAME + 1,
                           NFS2_MAX_NAME) && name[0] && strcmp(name, ".") &&
           strcmp(name, "..") && !strchr(name, '/');
}

static int coroutine_fn resolve_directory(Nfs2Server *server,
                                          const Nfs2FileHandle *handle,
                                          V9fsPath *absolute,
                                          V9fsPath *path, struct stat *st)
{
    int ret = resolve_handle(server, handle, absolute, path, st);

    if (ret >= 0 && !S_ISDIR(st->st_mode)) {
        ret = -ENOTDIR;
    }
    return ret;
}

static bool same_object(const struct stat *a, const struct stat *b)
{
    return a->st_dev == b->st_dev && a->st_ino == b->st_ino;
}

static int coroutine_fn sync_directory(Nfs2Server *server, V9fsPath *dir)
{
    V9fsFidOpenState open = { 0 };
    int ret = co_opendir(server, dir, &open);

    if (ret >= 0) {
        int close_ret;

        ret = co_fsync_type(server, P9_FID_DIR, &open);
        close_ret = co_simple_open(server, BACKEND_CLOSEDIR, &open);
        if (ret >= 0 && close_ret < 0) {
            ret = close_ret;
        }
    }
    return ret;
}

static void coroutine_fn cleanup_exclusive_temp(Nfs2Server *server,
                                                 V9fsPath *dir,
                                                 const char *name,
                                                 V9fsPath *path,
                                                 const struct stat *opened)
{
    struct stat current;

    if ((!path->data && co_name_to_path(server, dir, name, path) < 0) ||
        co_lstat(server, path, &current) < 0 ||
        !same_object(&current, opened)) {
        return;
    }
    co_unlink(server, dir, name, false);
}

static int coroutine_fn exclusive_create(Nfs2Server *server, V9fsPath *dir,
                                          const char *name,
                                          const uint8_t verifier[8],
                                          mode_t mode, uid_t uid, gid_t gid,
                                          V9fsPath *child,
                                          struct stat *after)
{
    FileOperations *ops = server->backend.ops;
    V9fsFidOpenState open = { 0 };
    V9fsPath temp = { 0 };
    struct stat opened_st, current;
    uint8_t stored[8];
    char temp_name[NFS2_MAX_NAME + 1];
    bool opened = false, created = false, published = false;
    bool opened_st_valid = false;
    int ret, close_ret;

    if (!ops->open || !ops->open2 || !ops->fstat || !ops->close ||
        !ops->fgetxattr || !ops->fsetxattr || !ops->fsync ||
        !ops->opendir || !ops->closedir || !ops->link || !ops->unlinkat) {
        return -EOPNOTSUPP;
    }

    ret = co_name_to_path(server, dir, name, child);
    if (ret >= 0) {
        ret = co_lstat(server, child, &current);
    }
    if (ret >= 0) {
        ret = co_open_flags(server, child, O_RDONLY, &open);
        if (ret == -EISDIR || ret == -ELOOP || ret == -EINVAL) {
            return -EEXIST;
        }
        opened = ret >= 0;
    }
    if (opened) {
        ret = co_fstat(server, P9_FID_FILE, &open, &opened_st);
        if (ret >= 0) {
            ret = co_fgetxattr(server, &open, NFS3_CREATE_VERIFIER_XATTR,
                               stored, sizeof(stored));
        }
        if (ret == sizeof(stored)) {
            ret = memcmp(stored, verifier, sizeof(stored)) ? -EEXIST : 0;
        } else if (ret == -ENODATA || ret == -ENOATTR || ret == -ERANGE ||
                   ret >= 0) {
            ret = -EEXIST;
        } else if (ret == -EOPNOTSUPP || ret == -ENOSYS) {
            ret = -EOPNOTSUPP;
        }
        if (ret >= 0) {
            ret = co_lstat(server, child, after);
            if (ret >= 0 && !same_object(&opened_st, after)) {
                ret = -EEXIST;
            }
        }
        close_ret = co_simple_open(server, BACKEND_CLOSE, &open);
        return ret >= 0 && close_ret < 0 ? close_ret : ret;
    }
    if (ret != -ENOENT) {
        path_clear(child);
        return ret;
    }
    path_clear(child);

    for (unsigned int attempt = 0; attempt < 16; attempt++) {
        snprintf(temp_name, sizeof(temp_name),
                 ".qemu-nfs3-exclusive-%016" PRIx64,
                 ++server->exclusive_sequence);
        ret = co_create_open(server, dir, temp_name,
                             O_CREAT | O_EXCL | O_WRONLY, mode,
                             uid, gid, &open);
        if (ret != -EEXIST) {
            break;
        }
    }
    if (ret < 0) {
        return ret == -EEXIST ? -EIO : ret;
    }
    opened = created = true;
    ret = co_fstat(server, P9_FID_FILE, &open, &opened_st);
    opened_st_valid = ret >= 0;
    if (ret >= 0) {
        ret = co_fsetxattr(server, &open, NFS3_CREATE_VERIFIER_XATTR,
                           (void *)verifier, sizeof(stored), XATTR_CREATE);
    }
    if (ret >= 0) {
        ret = co_fsync(server, &open);
    }
    if (ret >= 0) {
        ret = co_name_to_path(server, dir, temp_name, &temp);
    }
    if (ret >= 0) {
        ret = co_lstat(server, &temp, &current);
        if (ret >= 0 && !same_object(&opened_st, &current)) {
            ret = -ESTALE;
        }
    }
    if (ret >= 0) {
        ret = co_link(server, &temp, dir, name);
        published = ret >= 0;
    }
    if (published) {
        int unlink_ret = co_unlink(server, dir, temp_name, false);
        int identity_ret = co_name_to_path(server, dir, name, child);
        int sync_ret;

        if (identity_ret >= 0) {
            identity_ret = co_lstat(server, child, after);
            if (identity_ret >= 0 && !same_object(&opened_st, after)) {
                identity_ret = -ESTALE;
            }
        }
        sync_ret = sync_directory(server, dir);
        ret = unlink_ret < 0 ? unlink_ret :
              identity_ret < 0 ? identity_ret : sync_ret;
    } else if (created && opened_st_valid) {
        cleanup_exclusive_temp(server, dir, temp_name, &temp, &opened_st);
    }
    close_ret = opened ? co_simple_open(server, BACKEND_CLOSE, &open) : 0;
    if (ret >= 0 && close_ret < 0) {
        ret = close_ret;
    }
    path_clear(&temp);
    return ret;
}

static bool coroutine_fn reply_create(Nfs2Server *server,
                                      Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w, bool v3,
                                      uint32_t kind)
{
    Nfs2FileHandle dir_handle, result_handle;
    NfsSetAttr attr = { 0 };
    char name[NFS2_MAX_NAME + 1], target[NFS2_MAX_PATH + 1] = { 0 };
    uint8_t verifier[8] = { 0 };
    V9fsPath absolute = { 0 }, dir = { 0 }, child = { 0 };
    g_autofree char *child_abs = NULL;
    Nfs2HandlePathState path_state;
    struct stat before = { 0 }, after = { 0 };
    V9fsFidOpenState open = { 0 };
    uint32_t create_mode = 0, type = NFS2_NFREG, major_no = 0, minor_no = 0;
    bool opened = false, existing_object = false;
    bool exclusive_existing = false, before_valid = false;
    int ret;
    bool ok;

    if (!decode_name_args(call, v3, &dir_handle, name)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    if (!v3) {
        if (kind == NFS2_NFSPROC_SYMLINK) {
            if (!nfs2_xdr_string(&call->body, target, sizeof(target),
                                 NFS2_MAX_PATH) || !target[0]) {
                return nfs2_rpc_reply_garbage_args(w, call->xid);
            }
        }
        if (!decode_v2_sattr(&call->body, &attr)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        type = kind == NFS2_NFSPROC_MKDIR ? NFS2_NFDIR :
               kind == NFS2_NFSPROC_SYMLINK ? NFS2_NFLNK : NFS2_NFREG;
    } else if (kind == NFS3PROC_CREATE) {
        if (!nfs2_xdr_u32(&call->body, &create_mode) || create_mode > 2) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        if (create_mode == 2) {
            if (!nfs2_xdr_opaque(&call->body, verifier,
                                 sizeof(verifier))) {
                return nfs2_rpc_reply_garbage_args(w, call->xid);
            }
            attr.mode_set = true;
            attr.mode = 0600;
        } else if (!decode_v3_sattr(&call->body, &attr)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
    } else if (kind == NFS3PROC_SYMLINK) {
        if (!decode_v3_sattr(&call->body, &attr) ||
            !nfs2_xdr_string(&call->body, target, sizeof(target),
                             NFS2_MAX_PATH) || !target[0]) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        type = NFS2_NFLNK;
    } else if (kind == NFS3PROC_MKNOD) {
        if (!nfs2_xdr_u32(&call->body, &type)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        if (type < NFS2_NFBLK || type > 7 || type == NFS2_NFLNK) {
            if (!nfs2_xdr_reader_empty(&call->body)) {
                return nfs2_rpc_reply_garbage_args(w, call->xid);
            }
            return nfs2_rpc_reply_success(w, call->xid) &&
                   write_nfs_status(w, NFS3ERR_BADTYPE) &&
                   nfs2_xdr_put_u32(w, 0) && nfs2_xdr_put_u32(w, 0);
        }
        if (!decode_v3_sattr(&call->body, &attr) ||
            ((type == NFS2_NFBLK || type == NFS2_NFCHR) &&
             (!nfs2_xdr_u32(&call->body, &major_no) ||
              !nfs2_xdr_u32(&call->body, &minor_no)))) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
    } else {
        if (!decode_v3_sattr(&call->body, &attr)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        type = NFS2_NFDIR;
    }
    if (type == NFS2_NFLNK) {
        attr.size_set = false;
    }
    if (!nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_directory(server, &dir_handle, &absolute, &dir, &before);
    before_valid = ret >= 0;
    if (ret >= 0) {
        child_abs = child_absolute(absolute.data, name);
        if (!child_abs) {
            ret = -EIO;
        } else {
            nfs2_handle_path_state(server->handles, child_abs, &path_state);
        }
    }
    if (ret >= 0 && v3 && kind == NFS3PROC_CREATE && create_mode == 2) {
        ret = exclusive_create(server, &dir, name, verifier, 0600,
                               before.st_uid, before.st_gid,
                               &child, &after);
        exclusive_existing = ret >= 0;
    }
    if (ret >= 0 && type == NFS2_NFREG &&
        (!v3 || (kind == NFS3PROC_CREATE && create_mode == 0))) {
        ret = co_name_to_path(server, &dir, name, &child);
        if (ret >= 0) {
            ret = co_lstat(server, &child, &after);
            existing_object = ret >= 0;
            if (ret >= 0 && !S_ISREG(after.st_mode)) {
                ret = S_ISDIR(after.st_mode) ? -EISDIR : -EINVAL;
            }
            if (ret == -ENOENT) {
                ret = 0;
                path_clear(&child);
            }
        } else if (ret == -ENOENT) {
            ret = 0;
        }
    }
    if (ret >= 0 && !exclusive_existing && !existing_object) {
        if (type == NFS2_NFREG) {
            int flags = O_CREAT | O_WRONLY;

            if ((v3 && create_mode != 0)) {
                flags |= O_EXCL;
            }
            ret = co_create_open(server, &dir, name, flags,
                                 attr.mode_set ? attr.mode : 0600,
                                 before.st_uid, before.st_gid, &open);
            opened = ret >= 0;
        } else if (type == NFS2_NFDIR) {
            ret = co_create_node(server, BACKEND_MKDIR, &dir, name, NULL,
                                 S_IFDIR | (attr.mode_set ? attr.mode : 0755),
                                 0, before.st_uid, before.st_gid);
        } else if (type == NFS2_NFLNK) {
            ret = co_create_node(server, BACKEND_SYMLINK, &dir, name, target,
                                 S_IFLNK | 0777, 0,
                                 before.st_uid, before.st_gid);
        } else {
            mode_t file_mode = type == NFS2_NFBLK ? S_IFBLK :
                               type == NFS2_NFCHR ? S_IFCHR :
                               type == 6 ? S_IFSOCK : S_IFIFO;

            ret = co_create_node(server, BACKEND_MKNOD, &dir, name, NULL,
                                 file_mode |
                                 (attr.mode_set ? attr.mode : 0600),
                                 makedev(major_no, minor_no),
                                 before.st_uid, before.st_gid);
        }
    }
    if (opened) {
        int close_ret = co_simple_open(server, BACKEND_CLOSE, &open);
        if (ret >= 0 && close_ret < 0) {
            ret = close_ret;
        }
    }
    if (ret >= 0 && !existing_object && !exclusive_existing) {
        ret = co_name_to_path(server, &dir, name, &child);
    }
    if (ret >= 0 && !(v3 && kind == NFS3PROC_CREATE && create_mode == 2)) {
        ret = apply_sattr(server, &child, &attr);
    }
    if (ret >= 0) {
        ret = co_lstat(server, &child, &after);
    }
    if (ret >= 0) {
        ret = make_handle(server, child_abs, &after, &path_state,
                          &result_handle);
    }
    {
        struct stat dir_after = { 0 };
        bool dir_after_valid = false;

        if (before_valid && validate_handle_identity(server, &dir_handle,
                                                      &dir,
                                                      &dir_after) >= 0) {
            dir_after_valid = true;
        }
        ok = nfs2_rpc_reply_success(w, call->xid) &&
             write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
        if (ok && ret >= 0) {
            if (v3) {
                ok = nfs2_xdr_put_u32(w, 1) &&
                     nfs2_xdr_put_counted_opaque(w, result_handle.bytes, 32,
                                                 32) &&
                     put_post_attr(w, &after, true) &&
                     put_wcc(w, &before, before_valid, &dir_after,
                             dir_after_valid);
            } else if (kind != NFS2_NFSPROC_SYMLINK) {
                ok = nfs2_xdr_put_opaque(w, result_handle.bytes, 32) &&
                     put_v2_attr(server, w, &after);
            }
        } else if (ok && v3) {
            ok = put_wcc(w, &before, before_valid, &dir_after,
                         dir_after_valid);
        }
    }
    path_clear(&absolute);
    path_clear(&dir);
    path_clear(&child);
    return ok;
}

static bool coroutine_fn reply_write(Nfs2Server *server,
                                     Nfs2RpcCall *call,
                                     Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    struct stat before = { 0 }, after = { 0 };
    V9fsFidOpenState open = { 0 };
    const uint8_t *data;
    size_t data_len;
    uint64_t offset;
    uint32_t count, stable = 2;
    bool opened = false, before_valid, after_valid = false;
    int ret;
    bool ok;

    if (v3) {
        if (!decode_v3_handle(&call->body, &handle, false) ||
            !get_u64(&call->body, &offset) ||
            !nfs2_xdr_u32(&call->body, &count) ||
            !nfs2_xdr_u32(&call->body, &stable) || stable > 2 ||
            !nfs2_xdr_counted_opaque(&call->body, &data, &data_len,
                                     NFS2_MAX_DATA) ||
            data_len != count || !nfs2_xdr_reader_empty(&call->body)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
    } else {
        uint32_t begin_offset, write_offset, total_count;

        if (!decode_v2_handle_partial(&call->body, &handle) ||
            !nfs2_xdr_u32(&call->body, &begin_offset) ||
            !nfs2_xdr_u32(&call->body, &write_offset) ||
            !nfs2_xdr_u32(&call->body, &total_count)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        offset = write_offset;
        if (!nfs2_xdr_counted_opaque(&call->body, &data, &data_len,
                                     NFS2_MAX_DATA) ||
            !nfs2_xdr_reader_empty(&call->body)) {
            return nfs2_rpc_reply_garbage_args(w, call->xid);
        }
        (void)begin_offset;
        count = data_len;
    }
    ret = resolve_handle(server, &handle, &absolute, &path, &before);
    before_valid = ret >= 0;
    if (ret >= 0 && !S_ISREG(before.st_mode)) {
        ret = S_ISDIR(before.st_mode) ? -EISDIR : -EINVAL;
    }
    if (ret >= 0) {
        ret = co_open_flags(server, &path, O_WRONLY, &open);
        opened = ret >= 0;
    }
    if (ret >= 0) {
        struct stat opened_st;

        ret = co_fstat(server, P9_FID_FILE, &open, &opened_st);
        if (ret >= 0 && !stat_matches_handle(server, &handle, &opened_st)) {
            ret = -ESTALE;
        }
    }
    if (ret >= 0) {
        ret = co_pwrite(server, &open, data, count, offset);
        if (ret >= 0 && ret != count) {
            ret = -EIO;
        }
    }
    if (ret >= 0) {
        ret = co_fsync(server, &open);
    }
    if (opened) {
        int close_ret = co_simple_open(server, BACKEND_CLOSE, &open);
        if (ret >= 0 && close_ret < 0) {
            ret = close_ret;
        }
    }
    if (before_valid) {
        int post_ret = validate_handle_identity(server, &handle, &path,
                                                &after);

        after_valid = post_ret >= 0;
        if (ret >= 0 && post_ret < 0) {
            ret = post_ret;
        }
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_wcc(w, &before, before_valid, &after, after_valid);
        if (ok && ret >= 0) {
            ok = nfs2_xdr_put_u32(w, count) &&
                 nfs2_xdr_put_u32(w, 2) &&
                 nfs2_xdr_put_opaque(w, server->write_verifier,
                                     sizeof(server->write_verifier));
        }
    } else if (ok && ret >= 0) {
        ok = put_v2_attr(server, w, &after);
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool coroutine_fn reply_remove(Nfs2Server *server,
                                      Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w, bool v3,
                                      bool directory)
{
    Nfs2FileHandle dir_handle;
    char name[NFS2_MAX_NAME + 1];
    V9fsPath absolute = { 0 }, dir = { 0 };
    g_autofree char *child_abs = NULL;
    struct stat before = { 0 }, after = { 0 };
    bool before_valid, after_valid = false;
    int ret;
    bool ok;

    if (!decode_name_args(call, v3, &dir_handle, name) ||
        !nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_directory(server, &dir_handle, &absolute, &dir, &before);
    before_valid = ret >= 0;
    if (ret >= 0) {
        child_abs = child_absolute(absolute.data, name);
        ret = child_abs ? 0 : -EIO;
    }
    if (ret >= 0) {
        ret = co_unlink(server, &dir, name, directory);
    }
    if (ret >= 0) {
        nfs2_handle_remove(server->handles, child_abs, NULL);
    }
    if (before_valid && validate_handle_identity(server, &dir_handle, &dir,
                                                  &after) >= 0) {
        after_valid = true;
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_wcc(w, &before, before_valid, &after, after_valid);
    }
    path_clear(&absolute);
    path_clear(&dir);
    return ok;
}

static bool coroutine_fn reply_rename(Nfs2Server *server,
                                      Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle old_handle, new_handle;
    char old_name[NFS2_MAX_NAME + 1], new_name[NFS2_MAX_NAME + 1];
    V9fsPath old_abs = { 0 }, old_dir = { 0 };
    V9fsPath new_abs = { 0 }, new_dir = { 0 };
    g_autofree char *old_child = NULL, *new_child = NULL;
    struct stat old_before = { 0 }, new_before = { 0 };
    struct stat old_after = { 0 }, new_after = { 0 };
    bool old_before_valid = false, new_before_valid = false;
    bool old_after_valid = false, new_after_valid = false;
    int ret;
    bool ok;

    if (!decode_name_args(call, v3, &old_handle, old_name) ||
        !decode_name_args(call, v3, &new_handle, new_name) ||
        !nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_directory(server, &old_handle, &old_abs, &old_dir,
                            &old_before);
    old_before_valid = ret >= 0;
    if (ret >= 0) {
        ret = resolve_directory(server, &new_handle, &new_abs, &new_dir,
                                &new_before);
        new_before_valid = ret >= 0;
    }
    if (ret >= 0) {
        old_child = child_absolute(old_abs.data, old_name);
        new_child = child_absolute(new_abs.data, new_name);
        if (!old_child || !new_child) {
            ret = -EIO;
        }
    }
    if (ret >= 0) {
        ret = co_rename(server, &old_dir, old_name, &new_dir, new_name);
    }
    if (ret >= 0 &&
        !nfs2_handle_rename(server->handles, old_child, new_child, NULL)) {
        ret = -EIO;
    }
    if (old_before_valid &&
        validate_handle_identity(server, &old_handle, &old_dir,
                                 &old_after) >= 0) {
        old_after_valid = true;
    }
    if (new_before_valid &&
        validate_handle_identity(server, &new_handle, &new_dir,
                                 &new_after) >= 0) {
        new_after_valid = true;
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_wcc(w, &old_before, old_before_valid,
                     &old_after, old_after_valid) &&
             put_wcc(w, &new_before, new_before_valid,
                     &new_after, new_after_valid);
    }
    path_clear(&old_abs);
    path_clear(&old_dir);
    path_clear(&new_abs);
    path_clear(&new_dir);
    return ok;
}

static bool coroutine_fn reply_link(Nfs2Server *server, Nfs2RpcCall *call,
                                    Nfs2XdrWriter *w, bool v3)
{
    Nfs2FileHandle file_handle, dir_handle;
    char name[NFS2_MAX_NAME + 1];
    V9fsPath file_abs = { 0 }, file = { 0 };
    V9fsPath dir_abs = { 0 }, dir = { 0 };
    V9fsPath linked = { 0 };
    g_autofree char *new_abs = NULL;
    struct stat file_st = { 0 }, file_after = { 0 }, linked_st = { 0 };
    struct stat dir_before = { 0 }, dir_after = { 0 };
    Nfs2FileHandle ignored_handle;
    bool file_valid = false, file_after_valid = false;
    bool dir_before_valid = false, dir_after_valid = false;
    bool linked_created = false;
    int ret;
    bool ok;

    if (!(v3 ? decode_v3_handle(&call->body, &file_handle, false) :
               decode_v2_handle_partial(&call->body, &file_handle)) ||
        !decode_name_args(call, v3, &dir_handle, name) ||
        !nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    ret = resolve_handle(server, &file_handle, &file_abs, &file, &file_st);
    file_valid = ret >= 0;
    if (ret >= 0 && S_ISDIR(file_st.st_mode)) {
        ret = -EPERM;
    }
    if (ret >= 0) {
        ret = resolve_directory(server, &dir_handle, &dir_abs, &dir,
                                &dir_before);
        dir_before_valid = ret >= 0;
    }
    if (ret >= 0) {
        new_abs = child_absolute(dir_abs.data, name);
        ret = new_abs ? co_link(server, &file, &dir, name) : -EIO;
        linked_created = ret >= 0;
    }
    if (ret >= 0) {
        ret = co_name_to_path(server, &dir, name, &linked);
    }
    if (ret >= 0) {
        ret = co_lstat(server, &linked, &linked_st);
        if (ret >= 0 &&
            (linked_st.st_dev != file_st.st_dev ||
             linked_st.st_ino != file_st.st_ino)) {
            ret = -ESTALE;
        }
    }
    if (file_valid) {
        int post_ret = validate_handle_identity(server, &file_handle, &file,
                                                &file_after);

        file_after_valid = post_ret >= 0;
        if (ret >= 0 && post_ret < 0) {
            ret = post_ret;
        }
    }
    if (ret >= 0) {
        uint32_t identity;

        if (!identity_id(server, &file_after, false, &identity) ||
            !nfs2_handle_create(server->handles, identity, new_abs,
                                &ignored_handle, NULL)) {
            ret = -EIO;
        }
    }
    if (ret < 0 && linked_created) {
        co_unlink(server, &dir, name, false);
        if (file_valid) {
            file_after_valid =
                validate_handle_identity(server, &file_handle, &file,
                                         &file_after) >= 0;
        }
    }
    if (dir_before_valid &&
        validate_handle_identity(server, &dir_handle, &dir,
                                 &dir_after) >= 0) {
        dir_after_valid = true;
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3 ? v3_status(ret) : v2_status(ret));
    if (ok && v3) {
        ok = put_post_attr(w, &file_after, file_after_valid) &&
             put_wcc(w, &dir_before, dir_before_valid,
                     &dir_after, dir_after_valid);
    }
    path_clear(&file_abs);
    path_clear(&file);
    path_clear(&dir_abs);
    path_clear(&dir);
    path_clear(&linked);
    return ok;
}

static bool coroutine_fn reply_commit(Nfs2Server *server,
                                      Nfs2RpcCall *call, Nfs2XdrWriter *w)
{
    Nfs2FileHandle handle;
    V9fsPath absolute = { 0 }, path = { 0 };
    V9fsFidOpenState open = { 0 };
    struct stat before = { 0 }, after = { 0 };
    uint64_t offset;
    uint32_t count;
    bool opened = false, before_valid, after_valid = false;
    int ret;
    bool ok;

    if (!decode_v3_handle(&call->body, &handle, false) ||
        !get_u64(&call->body, &offset) ||
        !nfs2_xdr_u32(&call->body, &count) ||
        !nfs2_xdr_reader_empty(&call->body)) {
        return nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    (void)offset;
    (void)count;
    ret = resolve_handle(server, &handle, &absolute, &path, &before);
    before_valid = ret >= 0;
    if (ret >= 0 && !S_ISREG(before.st_mode)) {
        ret = -EINVAL;
    }
    if (ret >= 0) {
        ret = co_open_flags(server, &path, O_WRONLY, &open);
        opened = ret >= 0;
    }
    if (ret >= 0) {
        struct stat opened_st;

        ret = co_fstat(server, P9_FID_FILE, &open, &opened_st);
        if (ret >= 0 && !stat_matches_handle(server, &handle, &opened_st)) {
            ret = -ESTALE;
        }
    }
    if (ret >= 0) {
        ret = co_fsync(server, &open);
    }
    if (opened) {
        int close_ret = co_simple_open(server, BACKEND_CLOSE, &open);
        if (ret >= 0 && close_ret < 0) {
            ret = close_ret;
        }
    }
    if (before_valid) {
        int post_ret = validate_handle_identity(server, &handle, &path,
                                                &after);

        after_valid = post_ret >= 0;
        if (ret >= 0 && post_ret < 0) {
            ret = post_ret;
        }
    }
    ok = nfs2_rpc_reply_success(w, call->xid) &&
         write_nfs_status(w, v3_status(ret)) &&
         put_wcc(w, &before, before_valid, &after, after_valid);
    if (ok && ret >= 0) {
        ok = nfs2_xdr_put_opaque(w, server->write_verifier,
                                 sizeof(server->write_verifier));
    }
    path_clear(&absolute);
    path_clear(&path);
    return ok;
}

static bool is_v2_mutator(uint32_t proc)
{
    return proc == NFS2_NFSPROC_SETATTR ||
           (proc >= NFS2_NFSPROC_WRITE && proc <= NFS2_NFSPROC_RMDIR);
}

static bool is_v3_mutator(uint32_t proc)
{
    return (proc >= NFS3PROC_SETATTR && proc <= NFS3PROC_SETATTR) ||
           (proc >= NFS3PROC_WRITE && proc <= NFS3PROC_LINK) ||
           proc == NFS3PROC_COMMIT;
}

static bool reply_v3_rofs(Nfs2XdrWriter *w, uint32_t xid, uint32_t proc)
{
    unsigned int absent_attributes;

    switch (proc) {
    case NFS3PROC_RENAME:
        absent_attributes = 4; /* Two wcc_data values. */
        break;
    case NFS3PROC_LINK:
        absent_attributes = 3; /* file post-op attrs and directory wcc. */
        break;
    default:
        absent_attributes = 2; /* One wcc_data value. */
        break;
    }
    if (!nfs2_rpc_reply_success(w, xid) ||
        !write_nfs_status(w, NFS3ERR_ROFS)) {
        return false;
    }
    for (unsigned int i = 0; i < absent_attributes; i++) {
        if (!nfs2_xdr_put_u32(w, 0)) {
            return false;
        }
    }
    return true;
}

static bool coroutine_fn dispatch_nfs_mutation(Nfs2Server *server,
                                               Nfs2RpcCall *call,
                                               Nfs2XdrWriter *w, bool v3)
{
    bool ok;

    qemu_co_mutex_lock(&server->mutation_mutex);
    if (!v3) {
        switch (call->procedure) {
        case NFS2_NFSPROC_SETATTR:
            ok = reply_setattr(server, call, w, false);
            break;
        case NFS2_NFSPROC_WRITE:
            ok = reply_write(server, call, w, false);
            break;
        case NFS2_NFSPROC_CREATE:
        case NFS2_NFSPROC_MKDIR:
        case NFS2_NFSPROC_SYMLINK:
            ok = reply_create(server, call, w, false, call->procedure);
            break;
        case NFS2_NFSPROC_REMOVE:
            ok = reply_remove(server, call, w, false, false);
            break;
        case NFS2_NFSPROC_RMDIR:
            ok = reply_remove(server, call, w, false, true);
            break;
        case NFS2_NFSPROC_RENAME:
            ok = reply_rename(server, call, w, false);
            break;
        case NFS2_NFSPROC_LINK:
            ok = reply_link(server, call, w, false);
            break;
        default:
            g_assert_not_reached();
        }
    } else {
        switch (call->procedure) {
        case NFS3PROC_SETATTR:
            ok = reply_setattr(server, call, w, true);
            break;
        case NFS3PROC_WRITE:
            ok = reply_write(server, call, w, true);
            break;
        case NFS3PROC_CREATE:
        case NFS3PROC_MKDIR:
        case NFS3PROC_SYMLINK:
        case NFS3PROC_MKNOD:
            ok = reply_create(server, call, w, true, call->procedure);
            break;
        case NFS3PROC_REMOVE:
            ok = reply_remove(server, call, w, true, false);
            break;
        case NFS3PROC_RMDIR:
            ok = reply_remove(server, call, w, true, true);
            break;
        case NFS3PROC_RENAME:
            ok = reply_rename(server, call, w, true);
            break;
        case NFS3PROC_LINK:
            ok = reply_link(server, call, w, true);
            break;
        case NFS3PROC_COMMIT:
            ok = reply_commit(server, call, w);
            break;
        default:
            g_assert_not_reached();
        }
    }
    qemu_co_mutex_unlock(&server->mutation_mutex);
    return ok;
}

static bool coroutine_fn dispatch_nfs(Nfs2Server *server, Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w)
{
    bool v3 = call->version == NFS3_VERSION;

    if ((!v3 && is_v2_mutator(call->procedure)) ||
        (v3 && is_v3_mutator(call->procedure))) {
        if (!server->writable) {
            if (v3) {
                return reply_v3_rofs(w, call->xid, call->procedure);
            }
            return nfs2_rpc_reply_success(w, call->xid) &&
                   write_nfs_status(w, NFS2_NFSERR_ROFS);
        }
        return dispatch_nfs_mutation(server, call, w, v3);
    }
    if (call->procedure == 0) {
        return body_empty(call) ? nfs2_rpc_reply_success(w, call->xid) :
                                  nfs2_rpc_reply_garbage_args(w, call->xid);
    }
    if (!v3) {
        switch (call->procedure) {
        case NFS2_NFSPROC_GETATTR:
            return reply_getattr(server, call, w, false);
        case NFS2_NFSPROC_ROOT:
        case NFS2_NFSPROC_WRITECACHE:
            return body_empty(call) ? nfs2_rpc_reply_success(w, call->xid) :
                                      nfs2_rpc_reply_garbage_args(w, call->xid);
        case NFS2_NFSPROC_LOOKUP:
            return reply_lookup(server, call, w, false);
        case NFS2_NFSPROC_READLINK:
            return reply_readlink(server, call, w, false);
        case NFS2_NFSPROC_READ:
            return reply_read(server, call, w, false);
        case NFS2_NFSPROC_READDIR:
            return reply_readdir(server, call, w, false, false);
        case NFS2_NFSPROC_STATFS:
            return reply_statfs(server, call, w, false);
        default:
            return nfs2_rpc_reply_proc_unavail(w, call->xid);
        }
    }
    switch (call->procedure) {
    case NFS3PROC_GETATTR:
        return reply_getattr(server, call, w, true);
    case NFS3PROC_LOOKUP:
        return reply_lookup(server, call, w, true);
    case NFS3PROC_ACCESS:
        return reply_access(server, call, w);
    case NFS3PROC_READLINK:
        return reply_readlink(server, call, w, true);
    case NFS3PROC_READ:
        return reply_read(server, call, w, true);
    case NFS3PROC_READDIR:
        return reply_readdir(server, call, w, true, false);
    case NFS3PROC_READDIRPLUS:
        return reply_readdir(server, call, w, true, true);
    case NFS3PROC_FSSTAT:
        return reply_statfs(server, call, w, true);
    case NFS3PROC_FSINFO:
        return reply_fsinfo(server, call, w);
    case NFS3PROC_PATHCONF:
        return reply_pathconf(server, call, w);
    default:
        return nfs2_rpc_reply_proc_unavail(w, call->xid);
    }
}

static bool coroutine_fn dispatch_request(Nfs2Request *request,
                                          Nfs2XdrWriter *w)
{
    Nfs2RpcCall call;
    Nfs2RpcDecodeResult decode;
    int expected_program = service_program(request->service);

    decode = nfs2_rpc_decode_call(request->data, request->length, &call);
    if (decode != NFS2_RPC_DECODE_OK) {
        uint32_t xid = request->length >= 4 ? ldl_be_p(request->data) : 0;

        if (decode == NFS2_RPC_DECODE_RPC_MISMATCH) {
            return nfs2_rpc_reply_rpc_mismatch(w, xid, NFS2_RPC_VERSION,
                                                NFS2_RPC_VERSION);
        }
        if (decode == NFS2_RPC_DECODE_AUTH_ERROR) {
            return nfs2_rpc_reply_auth_error(w, xid, NFS2_RPC_AUTH_BADCRED);
        }
        return nfs2_rpc_reply_garbage_args(w, xid);
    }
    if (call.program != expected_program) {
        return nfs2_rpc_reply_prog_unavail(w, call.xid);
    }
    if (!service_version(request->service, call.version)) {
        uint32_t low = request->service == NFS2_SERVICE_PORTMAP ? 2 : 1;
        uint32_t high = request->service == NFS2_SERVICE_PORTMAP ? 2 : 3;
        return nfs2_rpc_reply_prog_mismatch(w, call.xid, low, high);
    }
    switch (request->service) {
    case NFS2_SERVICE_PORTMAP:
        return dispatch_portmap(&call, w);
    case NFS2_SERVICE_MOUNT:
        return dispatch_mount(request->server, &call, w);
    case NFS2_SERVICE_NFS:
        return dispatch_nfs(request->server, &call, w);
    default:
        return nfs2_rpc_reply_prog_unavail(w, call.xid);
    }
}

static void coroutine_fn request_entry(void *opaque)
{
    Nfs2Request *request = opaque;
    Nfs2Server *server = request->server;
    uint8_t reply[NFS2_MAX_RPC_DATAGRAM];
    Nfs2XdrWriter writer;

    nfs2_xdr_writer_init(&writer, reply, sizeof(reply));
    if (!dispatch_request(request, &writer)) {
        nfs2_xdr_writer_init(&writer, reply, sizeof(reply));
        nfs2_rpc_reply_system_err(&writer,
                                  request->length >= 4 ?
                                  ldl_be_p(request->data) : 0);
    }
    if (request->duplicate) {
        NfsDuplicateEntry *entry = request->duplicate;

        g_byte_array_set_size(entry->reply, 0);
        g_byte_array_append(entry->reply, reply,
                            nfs2_xdr_writer_size(&writer));
        entry->in_flight = false;
        entry->completed_at = server_now_ms(server);
        entry->last_used = ++server->duplicate_sequence;
    }
    if (!server->closing &&
        (!request->duplicate || !request->duplicate->abandoned)) {
        if (request->duplicate) {
            request->duplicate->sending = true;
        }
        server->transport.send(request->service, &request->peer, reply,
                               nfs2_xdr_writer_size(&writer),
                               server->transport_opaque);
        if (request->duplicate) {
            request->duplicate->sending = false;
        }
    }
    if (request->duplicate && request->duplicate->abandoned) {
        g_ptr_array_remove(server->duplicates, request->duplicate);
    }
    server->pending--;
    g_free(request);
}

Nfs2Server *nfs2_server_new(const char *fsdev_id, bool writable,
                            const Nfs2TransportOps *transport,
                            void *transport_opaque, Error **errp)
{
    Nfs2Server *server;
    uint32_t root_identity;

    if (!fsdev_id || !fsdev_id[0] || !transport || !transport->send) {
        error_setg(errp, "NFS server configuration is incomplete");
        return NULL;
    }
    server = g_new0(Nfs2Server, 1);
    if (v9fs_backend_init(&server->backend, fsdev_id, errp) < 0) {
        g_free(server);
        return NULL;
    }
    if (writable && (server->backend.ctx.export_flags & V9FS_RDONLY)) {
        error_setg(errp, "writable NFS server requires a writable fsdev");
        v9fs_backend_cleanup(&server->backend);
        g_free(server);
        return NULL;
    }
    server->identities = g_hash_table_new_full(identity_hash, identity_equal,
                                                g_free, NULL);
    server->cookies_by_backend = g_hash_table_new_full(cookie_hash,
                                                        cookie_equal,
                                                        g_free, NULL);
    server->cookies_by_wire = g_hash_table_new(g_direct_hash, g_direct_equal);
    server->next_identity = 1;
    server->next_cookie = 1;
    server->duplicates = g_ptr_array_new_with_free_func(duplicate_free);
    server->handles = nfs2_handle_table_new(NULL, 0, errp);
    if (!server->handles ||
        !identity_id(server, &server->backend.root_st, true, &root_identity) ||
        !nfs2_handle_create(server->handles, root_identity, "/",
                            &server->root_handle, errp)) {
        nfs2_handle_table_free(server->handles);
        g_ptr_array_unref(server->duplicates);
        g_hash_table_destroy(server->cookies_by_wire);
        g_hash_table_destroy(server->cookies_by_backend);
        g_hash_table_destroy(server->identities);
        v9fs_backend_cleanup(&server->backend);
        g_free(server);
        return NULL;
    }
    server->transport = *transport;
    server->transport_opaque = transport_opaque;
    server->writable = writable;
    qemu_co_mutex_init(&server->mutation_mutex);
    if (qcrypto_random_bytes(server->write_verifier,
                             sizeof(server->write_verifier), errp) < 0) {
        nfs2_server_free(server);
        return NULL;
    }
    return server;
}

int nfs2_server_receive(Nfs2Server *server, Nfs2Service service,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len, Error **errp)
{
    Nfs2Request *request;
    Coroutine *co;
    NfsDuplicateEntry *duplicate = NULL;

    if (!server || server->closing || !peer || peer->sin_family != AF_INET ||
        (!data && len) || service > NFS2_SERVICE_NFS) {
        error_setg(errp, "invalid NFS datagram");
        return -1;
    }
    if (len > NFS2_MAX_RPC_DATAGRAM) {
        error_setg(errp, "NFS RPC datagram exceeds %u bytes",
                   NFS2_MAX_RPC_DATAGRAM);
        return -1;
    }
    {
        uint32_t xid;

        if (request_is_mutation(server, service, data, len, &xid)) {
            g_autofree uint8_t *allocated_digest = NULL;
            uint8_t *digest = allocated_digest;
            size_t digest_len = 0;
            int64_t now = server_now_ms(server);

            if (qcrypto_hash_bytes(QCRYPTO_HASH_ALGO_SHA256, data, len,
                                   &digest, &digest_len, errp) < 0) {
                return -1;
            }
            allocated_digest = digest;
            g_assert(digest_len == 32);
            duplicate_expire(server, now);
            for (size_t i = 0; i < server->duplicates->len; i++) {
                NfsDuplicateEntry *entry =
                    g_ptr_array_index(server->duplicates, i);

                if (!duplicate_matches(entry, peer, xid, digest)) {
                    continue;
                }
                entry->last_used = ++server->duplicate_sequence;
                if (entry->in_flight || entry->sending) {
                    return 0;
                }
                if (!server->closing) {
                    entry->sending = true;
                    server->transport.send(service, peer,
                                           entry->reply->data,
                                           entry->reply->len,
                                           server->transport_opaque);
                    entry->sending = false;
                    if (entry->abandoned) {
                        g_ptr_array_remove(server->duplicates, entry);
                    }
                }
                return 0;
            }
            if (server->duplicates->len == NFS_DUP_CACHE_MAX) {
                size_t oldest_index = SIZE_MAX;
                uint64_t oldest = UINT64_MAX;

                for (size_t i = 0; i < server->duplicates->len; i++) {
                    NfsDuplicateEntry *entry =
                        g_ptr_array_index(server->duplicates, i);

                    if (!entry->in_flight && !entry->sending &&
                        entry->last_used < oldest) {
                        oldest = entry->last_used;
                        oldest_index = i;
                    }
                }
                if (oldest_index == SIZE_MAX) {
                    uint8_t reply[24];
                    Nfs2XdrWriter writer;

                    nfs2_xdr_writer_init(&writer, reply, sizeof(reply));
                    nfs2_rpc_reply_system_err(&writer, xid);
                    server->transport.send(service, peer, reply,
                                           nfs2_xdr_writer_size(&writer),
                                           server->transport_opaque);
                    return 0;
                }
                g_ptr_array_remove_index(server->duplicates, oldest_index);
            }
            duplicate = g_new0(NfsDuplicateEntry, 1);
            duplicate->address = peer->sin_addr.s_addr;
            duplicate->port = peer->sin_port;
            duplicate->xid = xid;
            memcpy(duplicate->digest, digest, 32);
            duplicate->in_flight = true;
            duplicate->last_used = ++server->duplicate_sequence;
            duplicate->reply = g_byte_array_new();
            g_ptr_array_add(server->duplicates, duplicate);
        }
    }
    request = g_malloc(sizeof(*request) + len);
    request->server = server;
    request->service = service;
    request->peer = *peer;
    request->length = len;
    request->duplicate = duplicate;
    memcpy(request->data, data, len);
    server->pending++;
    co = qemu_coroutine_create(request_entry, request);
    aio_co_schedule(qemu_get_aio_context(), co);
    return 0;
}

bool nfs2_server_busy(const Nfs2Server *server)
{
    return server && server->pending;
}

void nfs2_server_begin_close(Nfs2Server *server)
{
    if (server) {
        server->closing = true;
    }
}

void nfs2_server_reset(Nfs2Server *server)
{
    if (!server) {
        return;
    }
    for (size_t i = server->duplicates->len; i > 0; i--) {
        NfsDuplicateEntry *entry =
            g_ptr_array_index(server->duplicates, i - 1);

        if (entry->in_flight || entry->sending) {
            entry->abandoned = true;
        } else {
            g_ptr_array_remove_index(server->duplicates, i - 1);
        }
    }
}

void nfs2_server_free(Nfs2Server *server)
{
    if (!server) {
        return;
    }
    g_assert(!server->pending);
    server->closing = true;
    g_ptr_array_unref(server->duplicates);
    nfs2_handle_table_free(server->handles);
    g_hash_table_destroy(server->cookies_by_wire);
    g_hash_table_destroy(server->cookies_by_backend);
    g_hash_table_destroy(server->identities);
    v9fs_backend_cleanup(&server->backend);
    g_free(server);
}
