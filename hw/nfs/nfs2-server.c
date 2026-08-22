/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include <sys/statvfs.h>

#include "block/thread-pool.h"
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

#define NFS3_VERSION 3U
#define MOUNT3_VERSION 3U
#define NFS3_FHSIZE 32U
#define NFS3_COOKIEVERFSIZE 8U
#define NFS2_MAX_IDENTITIES 65536U
#define NFS2_MAX_COOKIES 65536U

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
    NFS3ERR_SERVERFAULT = 10006,
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
} BackendOp;

typedef struct BackendWork {
    BackendOp op;
    V9fsBackend *backend;
    V9fsPath *dir;
    V9fsPath *path;
    const char *name;
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
} BackendWork;

typedef struct Nfs2Request {
    Nfs2Server *server;
    Nfs2Service service;
    struct sockaddr_in peer;
    size_t length;
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
};

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
        ret = ops->open ? ops->open(ctx, work->path, O_RDONLY, work->open) : -1;
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

static bool coroutine_fn dispatch_nfs(Nfs2Server *server, Nfs2RpcCall *call,
                                      Nfs2XdrWriter *w)
{
    bool v3 = call->version == NFS3_VERSION;

    if ((!v3 && is_v2_mutator(call->procedure)) ||
        (v3 && is_v3_mutator(call->procedure))) {
        if (v3) {
            return reply_v3_rofs(w, call->xid, call->procedure);
        }
        return nfs2_rpc_reply_success(w, call->xid) &&
               write_nfs_status(w, NFS2_NFSERR_ROFS);
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
    if (!server->closing) {
        server->transport.send(request->service, &request->peer, reply,
                               nfs2_xdr_writer_size(&writer),
                               server->transport_opaque);
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
    server->handles = nfs2_handle_table_new(NULL, 0, errp);
    if (!server->handles ||
        !identity_id(server, &server->backend.root_st, true, &root_identity) ||
        !nfs2_handle_create(server->handles, root_identity, "/",
                            &server->root_handle, errp)) {
        nfs2_handle_table_free(server->handles);
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
    return server;
}

int nfs2_server_receive(Nfs2Server *server, Nfs2Service service,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len, Error **errp)
{
    Nfs2Request *request;
    Coroutine *co;

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
    request = g_malloc(sizeof(*request) + len);
    request->server = server;
    request->service = service;
    request->peer = *peer;
    request->length = len;
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

void nfs2_server_free(Nfs2Server *server)
{
    if (!server) {
        return;
    }
    g_assert(!server->pending);
    server->closing = true;
    nfs2_handle_table_free(server->handles);
    g_hash_table_destroy(server->cookies_by_wire);
    g_hash_table_destroy(server->cookies_by_backend);
    g_hash_table_destroy(server->identities);
    v9fs_backend_cleanup(&server->backend);
    g_free(server);
}
