/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "fsdev/qemu-fsdev.h"
#include "fsdev/qemu-fsdev-throttle.h"
#include "hw/9pfs/9p.h"
#include "hw/9pfs/plan9-9p1-codec.h"
#include "hw/9pfs/plan9-9p1-server.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qom/object_interfaces.h"

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

typedef struct ServerFixture ServerFixture;

typedef enum TransportAction {
    TRANSPORT_ACTION_NONE,
    TRANSPORT_ACTION_RESET_CAN_SEND,
    TRANSPORT_ACTION_FREE_CAN_SEND,
    TRANSPORT_ACTION_NOP_CAN_SEND,
    TRANSPORT_ACTION_RESET_SEND,
    TRANSPORT_ACTION_FREE_SEND,
    TRANSPORT_ACTION_SESSION_SEND,
    TRANSPORT_ACTION_NOP_SEND,
    TRANSPORT_ACTION_FRAGMENTED_NOP_SEND,
} TransportAction;

typedef struct TestTransport {
    GByteArray *output;
    ServerFixture *fixture;
    size_t capacity;
    size_t max_chunk;
    bool fail;
    bool action_done;
    TransportAction action;
} TestTransport;

struct ServerFixture {
    FsDriverEntry fse;
    Plan9P1Server *server;
    TestTransport transport;
    char *root;
    GThread *main_thread;
    bool track_worker;
    bool backend_on_main;
    bool device_exhaustion;
    bool inode_collision;
    bool fail_close;
    bool fail_read;
    bool overreport_read;
    bool short_write;
    bool overreport_write;
    bool fail_remove;
    unsigned int fail_remove_once;
    bool fail_created_lstat;
    unsigned int fail_created_name_to_path;
    unsigned int fail_root_resolve;
    bool variant_walk_parent_inode;
    bool fail_chown;
    bool remove_null_path;
    unsigned int open_file_handles;
    unsigned int close_calls;
    unsigned int remove_calls;
    unsigned int chmod_calls;
    unsigned int chown_calls;
    unsigned int utimensat_calls;
    unsigned int rename_calls;
    unsigned int throttle_reads;
    unsigned int throttle_writes;
    bool literal_stats;
    unsigned int fail_dir_lstat_after;
    unsigned int dir_lstat_count;
    bool variant_dir_inodes;
    bool variant_dir_devices;
    unsigned int dir_scan_device;
    unsigned int backend_calls;
    unsigned int cleanup_calls;
    bool gate_read;
    QemuEvent read_started;
    QemuEvent read_release;
    bool gate_dir_lstat;
    unsigned int gate_dir_lstat_after;
    bool dir_gate_reset;
    QemuEvent dir_started;
    QemuEvent dir_release;
};

static ServerFixture *fixture;

static void note_backend_thread(ServerFixture *f)
{
    f->backend_calls++;
    if (f->track_worker && g_thread_self() == f->main_thread) {
        f->backend_on_main = true;
    }
}

FsDriverEntry *get_fsdev_fsentry(const char *id)
{
    if (fixture && !g_strcmp0(id, fixture->fse.fsdev_id)) {
        return &fixture->fse;
    }
    return NULL;
}

void fsdev_throttle_init(FsThrottle *fst)
{
}

void fsdev_throttle_cleanup(FsThrottle *fst)
{
}

void coroutine_fn fsdev_co_throttle_request(FsThrottle *fst,
                                             ThrottleDirection direction,
                                             struct iovec *iov,
                                             int iovcnt)
{
    g_assert_nonnull(fst);
    g_assert_cmpint(iovcnt, >, 0);
    if (direction == THROTTLE_READ) {
        g_assert_cmpuint(fst->cfg.buckets[THROTTLE_BPS_READ].avg, ==, 1);
        fixture->throttle_reads++;
    } else {
        g_assert_cmpint(direction, ==, THROTTLE_WRITE);
        fixture->throttle_writes++;
    }
}

static int test_init(FsContext *ctx, Error **errp)
{
    ctx->private = fixture;
    return 0;
}

static void test_cleanup(FsContext *ctx)
{
    ServerFixture *f = ctx->private;

    f->cleanup_calls++;
    ctx->private = NULL;
}

static int set_path(V9fsPath *path, const char *value)
{
    g_free(path->data);
    path->data = g_strdup(value);
    path->size = strlen(value) + 1;
    return 0;
}

static int test_name_to_path(FsContext *ctx, V9fsPath *dirpath,
                             const char *name, V9fsPath *path)
{
    ServerFixture *f = ctx->private;
    g_autofree char *parent = NULL;
    g_autofree char *joined = NULL;

    note_backend_thread(f);
    if (!dirpath) {
        if (f->fail_root_resolve) {
            f->fail_root_resolve--;
            errno = EIO;
            return -1;
        }
        if (strcmp(name, "/") && strcmp(name, ".") && strcmp(name, "..")) {
            errno = EINVAL;
            return -1;
        }
        return set_path(path, ".");
    }
    if (!strcmp(name, ".")) {
        return set_path(path, dirpath->data);
    }
    if (!strcmp(name, "..")) {
        if (!strcmp(dirpath->data, ".")) {
            return set_path(path, ".");
        }
        parent = g_path_get_dirname(dirpath->data);
        return set_path(path, parent);
    }
    if (!name[0] || strchr(name, '/')) {
        errno = EINVAL;
        return -1;
    }
    if (f->fail_created_name_to_path &&
        !strcmp(name, "rollback-nopath")) {
        g_autofree char *dir = !strcmp(dirpath->data, ".") ?
            g_strdup(f->root) :
            g_build_filename(f->root, dirpath->data + 2, NULL);
        g_autofree char *host = g_build_filename(dir, name, NULL);

        if (g_file_test(host, G_FILE_TEST_EXISTS)) {
            f->fail_created_name_to_path--;
            errno = EIO;
            return -1;
        }
    }
    joined = !strcmp(dirpath->data, ".") ? g_strdup_printf("./%s", name) :
        g_strdup_printf("%s/%s", dirpath->data, name);
    return set_path(path, joined);
}

static char *host_path(FsContext *ctx, V9fsPath *path)
{
    ServerFixture *f = ctx->private;

    g_assert_true(!strcmp(path->data, ".") ||
                  g_str_has_prefix(path->data, "./"));
    return !strcmp(path->data, ".") ? g_strdup(f->root) :
        g_build_filename(f->root, path->data + 2, NULL);
}

static int test_lstat(FsContext *ctx, V9fsPath *path, struct stat *st)
{
    ServerFixture *f = ctx->private;
    g_autofree char *host = host_path(ctx, path);
    int ret;

    note_backend_thread(f);
    if (f->fail_created_lstat &&
        (g_str_has_suffix(path->data, "/rollback") ||
         g_str_has_suffix(path->data, "/rollback-dir"))) {
        errno = EIO;
        return -1;
    }
    if (g_str_has_prefix(path->data, "./dir/")) {
        f->dir_lstat_count++;
        if (f->gate_dir_lstat &&
            f->dir_lstat_count == f->gate_dir_lstat_after) {
            qemu_event_set(&f->dir_started);
            qemu_event_wait(&f->dir_release);
        }
        if (f->fail_dir_lstat_after &&
            f->dir_lstat_count > f->fail_dir_lstat_after) {
            errno = EIO;
            return -1;
        }
    }
    ret = lstat(host, st);
    if (!ret && f->variant_dir_inodes &&
        g_str_has_prefix(path->data, "./dir/")) {
        st->st_ino += UINT64_C(1) << 20;
    }
    if (!ret && f->variant_dir_devices &&
        g_str_has_prefix(path->data, "./dir/")) {
        st->st_dev += f->dir_scan_device;
    }
    if (!ret && f->literal_stats) {
        memset(st, 0, sizeof(*st));
        st->st_dev = 42;
        st->st_uid = 42;
        st->st_gid = 43;
        if (!strcmp(path->data, ".")) {
            st->st_ino = 0x100;
            st->st_mode = S_IFDIR | 0755;
            st->st_atime = 1000;
            st->st_mtime = 2000;
        } else if (!strcmp(path->data, "./68020")) {
            st->st_ino = 0x101;
            st->st_mode = S_IFDIR | 0755;
            st->st_atime = 1001;
            st->st_mtime = 2001;
        } else if (!strcmp(path->data, "./68020/init")) {
            st->st_ino = 0x102;
            st->st_mode = S_IFREG | 0644;
            st->st_atime = 1002;
            st->st_mtime = 2002;
            st->st_size = 10;
        }
    }
    if (!ret && f->device_exhaustion && strcmp(path->data, ".")) {
        st->st_dev++;
    }
    if (!ret && f->inode_collision && g_str_has_suffix(path->data, "/plain")) {
        st->st_ino = 7;
    } else if (!ret && f->inode_collision &&
               g_str_has_suffix(path->data, "/denied")) {
        st->st_ino = (UINT64_C(1) << 24) | 7;
    }
    if (!ret && f->variant_walk_parent_inode &&
        !strcmp(path->data, "./a")) {
        st->st_ino += UINT64_C(1) << 22;
    }
    return ret;
}

static int test_open(FsContext *ctx, V9fsPath *path, int flags,
                     V9fsFidOpenState *fs)
{
    ServerFixture *f = ctx->private;
    g_autofree char *host = host_path(ctx, path);

    note_backend_thread(f);
    if (g_str_has_suffix(path->data, "/denied")) {
        errno = EACCES;
        return -1;
    }
    fs->fd = open(host, flags);
    if (fs->fd >= 0) {
        f->open_file_handles++;
    }
    return fs->fd;
}

static int test_close(FsContext *ctx, V9fsFidOpenState *fs)
{
    ServerFixture *f = ctx->private;
    int ret;

    note_backend_thread(f);
    f->close_calls++;
    ret = close(fs->fd);
    if (ret == 0) {
        g_assert_cmpuint(f->open_file_handles, >, 0);
        f->open_file_handles--;
    }
    if (f->fail_close) {
        errno = EIO;
        return -1;
    }
    return ret;
}

static int test_opendir(FsContext *ctx, V9fsPath *path,
                        V9fsFidOpenState *fs)
{
    ServerFixture *f = ctx->private;
    g_autofree char *host = host_path(ctx, path);

    note_backend_thread(f);
    fs->dir.stream = opendir(host);
    return fs->dir.stream ? 0 : -1;
}

static int test_closedir(FsContext *ctx, V9fsFidOpenState *fs)
{
    note_backend_thread(ctx->private);
    return closedir(fs->dir.stream);
}

static void test_rewinddir(FsContext *ctx, V9fsFidOpenState *fs)
{
    ServerFixture *f = ctx->private;

    note_backend_thread(f);
    if (f->variant_dir_devices) {
        f->dir_scan_device++;
    }
    rewinddir(fs->dir.stream);
}

static struct dirent *test_readdir(FsContext *ctx, V9fsFidOpenState *fs)
{
    note_backend_thread(ctx->private);
    return readdir(fs->dir.stream);
}

static ssize_t test_preadv(FsContext *ctx, V9fsFidOpenState *fs,
                           const struct iovec *iov, int iovcnt, off_t offset)
{
    ServerFixture *f = ctx->private;

    note_backend_thread(f);
    if (f->gate_read) {
        qemu_event_set(&f->read_started);
        qemu_event_wait(&f->read_release);
    }
    if (f->fail_read) {
        errno = EIO;
        return -1;
    }
    if (f->overreport_read) {
        return iov[0].iov_len + 1;
    }
    return preadv(fs->fd, iov, iovcnt, offset);
}

static int test_fstat(FsContext *ctx, int fid_type,
                      V9fsFidOpenState *fs, struct stat *st)
{
    note_backend_thread(ctx->private);
    return fstat(fs->fd, st);
}

static int test_open2(FsContext *ctx, V9fsPath *dirpath, const char *name,
                      int flags, FsCred *cred, V9fsFidOpenState *fs)
{
    ServerFixture *f = ctx->private;
    g_autofree char *dir = host_path(ctx, dirpath);
    g_autofree char *path = g_build_filename(dir, name, NULL);

    note_backend_thread(f);
    fs->fd = open(path, flags, cred->fc_mode);
    if (fs->fd >= 0) {
        f->open_file_handles++;
    }
    return fs->fd;
}

static int test_mkdir(FsContext *ctx, V9fsPath *dirpath, const char *name,
                      FsCred *cred)
{
    g_autofree char *dir = host_path(ctx, dirpath);
    g_autofree char *path = g_build_filename(dir, name, NULL);

    note_backend_thread(ctx->private);
    return mkdir(path, cred->fc_mode);
}

static ssize_t test_pwritev(FsContext *ctx, V9fsFidOpenState *fs,
                            const struct iovec *iov, int iovcnt, off_t offset)
{
    ServerFixture *f = ctx->private;

    note_backend_thread(f);
    if (f->overreport_write) {
        return iov[0].iov_len + 1;
    }
    if (f->short_write && iov[0].iov_len) {
        struct iovec short_iov = iov[0];

        short_iov.iov_len--;
        return pwritev(fs->fd, &short_iov, 1, offset);
    }
    return pwritev(fs->fd, iov, iovcnt, offset);
}

static int test_chmod(FsContext *ctx, V9fsPath *path, FsCred *cred)
{
    ServerFixture *f = ctx->private;
    g_autofree char *host = host_path(ctx, path);

    note_backend_thread(f);
    f->chmod_calls++;
    return chmod(host, cred->fc_mode);
}

static int test_chown(FsContext *ctx, V9fsPath *path, FsCred *cred)
{
    ServerFixture *f = ctx->private;

    note_backend_thread(f);
    f->chown_calls++;
    if (f->fail_chown) {
        errno = EIO;
        return -1;
    }
    g_assert_cmpuint(cred->fc_uid, ==, (uid_t)-1);
    return 0;
}

static int test_utimensat(FsContext *ctx, V9fsPath *path,
                          const struct timespec *times)
{
    ServerFixture *f = ctx->private;
    g_autofree char *host = host_path(ctx, path);

    note_backend_thread(f);
    f->utimensat_calls++;
    return utimensat(AT_FDCWD, host, times, AT_SYMLINK_NOFOLLOW);
}

static int test_renameat(FsContext *ctx, V9fsPath *olddir,
                         const char *oldname, V9fsPath *newdir,
                         const char *newname)
{
    ServerFixture *f = ctx->private;
    g_autofree char *old_parent = host_path(ctx, olddir);
    g_autofree char *new_parent = host_path(ctx, newdir);
    g_autofree char *old_path = g_build_filename(old_parent, oldname, NULL);
    g_autofree char *new_path = g_build_filename(new_parent, newname, NULL);

    note_backend_thread(f);
    f->rename_calls++;
    return rename(old_path, new_path);
}

static int test_unlinkat(FsContext *ctx, V9fsPath *dirpath,
                         const char *name, int flags)
{
    ServerFixture *f = ctx->private;
    g_autofree char *dir = host_path(ctx, dirpath);
    g_autofree char *path = g_build_filename(dir, name, NULL);

    note_backend_thread(f);
    f->remove_calls++;
    if (f->fail_remove_once) {
        f->fail_remove_once--;
        errno = EIO;
        return -1;
    }
    if (f->fail_remove) {
        errno = EIO;
        return -1;
    }
    return flags & AT_REMOVEDIR ? rmdir(path) : unlink(path);
}

static int test_remove(FsContext *ctx, const char *path)
{
    ServerFixture *f = ctx->private;
    g_autofree char *host = NULL;

    note_backend_thread(f);
    f->remove_calls++;
    if (!path) {
        f->remove_null_path = true;
        errno = EINVAL;
        return -1;
    }
    g_assert_true(!strcmp(path, ".") || g_str_has_prefix(path, "./"));
    host = !strcmp(path, ".") ? g_strdup(f->root) :
        g_build_filename(f->root, path + 2, NULL);
    return remove(host);
}

static FileOperations test_ops = {
    .init = test_init,
    .cleanup = test_cleanup,
    .name_to_path = test_name_to_path,
    .lstat = test_lstat,
    .open = test_open,
    .close = test_close,
    .opendir = test_opendir,
    .closedir = test_closedir,
    .rewinddir = test_rewinddir,
    .readdir = test_readdir,
    .preadv = test_preadv,
    .pwritev = test_pwritev,
    .fstat = test_fstat,
    .open2 = test_open2,
    .mkdir = test_mkdir,
    .chmod = test_chmod,
    .chown = test_chown,
    .utimensat = test_utimensat,
    .renameat = test_renameat,
    .unlinkat = test_unlinkat,
    .remove = test_remove,
};

static size_t transport_can_send(void *opaque)
{
    TestTransport *transport = opaque;

    if (!transport->action_done &&
        (transport->action == TRANSPORT_ACTION_RESET_CAN_SEND ||
         transport->action == TRANSPORT_ACTION_FREE_CAN_SEND ||
         transport->action == TRANSPORT_ACTION_NOP_CAN_SEND)) {
        static const uint8_t tnop[3] = {
            [0] = PLAN9P1_TNOP, [1] = 0x34, [2] = 0x12,
        };

        transport->action_done = true;
        if (transport->action == TRANSPORT_ACTION_RESET_CAN_SEND) {
            plan9p1_server_reset(transport->fixture->server);
        } else if (transport->action == TRANSPORT_ACTION_FREE_CAN_SEND) {
            plan9p1_server_free(transport->fixture->server);
            transport->fixture->server = NULL;
        } else {
            g_assert_cmpint(plan9p1_server_receive(transport->fixture->server,
                                                   tnop, sizeof(tnop),
                                                   &error_abort), ==, 0);
        }
    }

    return transport->capacity;
}

static int transport_send(const uint8_t *buf, size_t len, void *opaque)
{
    TestTransport *transport = opaque;
    size_t sent;

    if (!transport->action_done &&
        transport->action >= TRANSPORT_ACTION_RESET_SEND) {
        static const uint8_t tsession[11] = {
            [0] = PLAN9P1_TSESSION, [1] = 0x34, [2] = 0x12,
        };
        static const uint8_t tnop[3] = {
            [0] = PLAN9P1_TNOP, [1] = 0x34, [2] = 0x12,
        };

        transport->action_done = true;
        if (transport->action == TRANSPORT_ACTION_RESET_SEND) {
            plan9p1_server_reset(transport->fixture->server);
            return 0;
        } else if (transport->action == TRANSPORT_ACTION_FREE_SEND) {
            plan9p1_server_free(transport->fixture->server);
            transport->fixture->server = NULL;
            return 0;
        } else if (transport->action == TRANSPORT_ACTION_SESSION_SEND) {
            g_assert_cmpint(plan9p1_server_receive(transport->fixture->server,
                                                   tsession,
                                                   sizeof(tsession),
                                                   &error_abort), ==, 0);
        } else if (transport->action == TRANSPORT_ACTION_NOP_SEND) {
            g_assert_cmpint(plan9p1_server_receive(transport->fixture->server,
                                                   tnop, sizeof(tnop),
                                                   &error_abort), ==, 0);
        } else {
            size_t i;

            for (i = 0; i < sizeof(tnop); i++) {
                g_assert_cmpint(plan9p1_server_receive(
                                    transport->fixture->server,
                                    tnop + i, 1, &error_abort), ==, 0);
            }
        }
    }

    if (transport->fail) {
        return -1;
    }
    sent = transport->max_chunk ? MIN(len, transport->max_chunk) : len;
    g_byte_array_append(transport->output, buf, sent);
    if (transport->capacity != SIZE_MAX) {
        transport->capacity -= MIN(transport->capacity, sent);
    }
    return sent;
}

static const Plan9P1TransportOps transport_ops = {
    .can_send = transport_can_send,
    .send = transport_send,
};

static void write_file(const char *path, const void *data, size_t len)
{
    GError *err = NULL;

    g_assert_true(g_file_set_contents(path, data, len, &err));
    g_assert_no_error(err);
}

static void fixture_setup(ServerFixture *f, gconstpointer opaque)
{
    g_autofree char *cpu = NULL;
    g_autofree char *dir = NULL;
    g_autofree char *nested = NULL;
    g_autofree char *path = NULL;
    g_autofree uint8_t *big = g_malloc0(PLAN9P1_MAX_DATA);
    GError *gerr = NULL;
    int fd;

    fixture = f;
    qemu_event_init(&f->read_started, false);
    qemu_event_init(&f->read_release, false);
    qemu_event_init(&f->dir_started, false);
    qemu_event_init(&f->dir_release, false);
    f->main_thread = g_thread_self();
    f->root = g_dir_make_tmp("qemu-9p1-server-XXXXXX", &gerr);
    g_assert_no_error(gerr);
    cpu = g_build_filename(f->root, "68020", NULL);
    dir = g_build_filename(f->root, "dir", NULL);
    nested = g_build_filename(f->root, "a", NULL);
    g_assert_cmpint(g_mkdir(cpu, 0700), ==, 0);
    g_assert_cmpint(g_mkdir(dir, 0700), ==, 0);
    g_assert_cmpint(g_mkdir(nested, 0700), ==, 0);
    g_free(nested);
    nested = g_build_filename(f->root, "a", "b", NULL);
    g_assert_cmpint(g_mkdir(nested, 0700), ==, 0);
    path = g_build_filename(cpu, "init", NULL);
    write_file(path, "boot-init\n", 10);
    g_free(path);
    path = g_build_filename(dir, "entry", NULL);
    write_file(path, "directory-entry\n", 16);
    g_free(path);
    path = g_build_filename(dir, "second", NULL);
    write_file(path, "directory-second\n", 17);
    g_free(path);
    path = g_build_filename(f->root, "plain", NULL);
    write_file(path, "plain-file\n", 11);
    g_free(path);
    path = g_build_filename(f->root, "denied", NULL);
    write_file(path, "denied\n", 7);
    g_free(path);
    path = g_build_filename(f->root, "big", NULL);
    write_file(path, big, PLAN9P1_MAX_DATA);
    g_free(path);
    path = g_build_filename(f->root, "large", NULL);
    fd = open(path, O_CREAT | O_RDWR, 0600);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, (off_t)UINT32_MAX + 123), ==, 0);
    g_assert_cmpint(close(fd), ==, 0);

    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)"testfs",
        .path = f->root,
        .ops = &test_ops,
        .export_flags = V9FS_SM_NONE,
    };
    f->fse.fst.cfg.buckets[THROTTLE_BPS_READ].avg = 1;
    f->transport.output = g_byte_array_new();
    f->transport.fixture = f;
    f->transport.capacity = SIZE_MAX;
    f->server = plan9p1_server_new("testfs", &transport_ops,
                                   &f->transport, opaque, &error_abort);
    f->track_worker = true;
}

static void pump_server(Plan9P1Server *server)
{
    unsigned int iterations = 0;

    while (plan9p1_server_busy(server)) {
        g_assert_cmpuint(iterations++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
}

static void fixture_teardown(ServerFixture *f, gconstpointer opaque)
{
    g_autofree char *path = NULL;

    if (f->server) {
        plan9p1_server_free(f->server);
        f->server = NULL;
        while (f->cleanup_calls == 0) {
            aio_poll(qemu_get_aio_context(), true);
        }
    }
    g_assert_cmpuint(f->open_file_handles, ==, 0);
    g_byte_array_unref(f->transport.output);
    path = g_build_filename(f->root, "68020", "init", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "dir", "entry", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "dir", "second", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "plain", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "denied", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "big", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "large", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "68020", NULL);
    g_assert_cmpint(g_rmdir(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "dir", NULL);
    g_assert_cmpint(g_rmdir(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "a", "b", NULL);
    g_assert_cmpint(g_rmdir(path), ==, 0);
    g_free(path);
    path = g_build_filename(f->root, "a", NULL);
    g_assert_cmpint(g_rmdir(path), ==, 0);
    g_assert_cmpint(g_rmdir(f->root), ==, 0);
    g_free(f->root);
    qemu_event_set(&f->read_release);
    qemu_event_destroy(&f->read_release);
    qemu_event_destroy(&f->read_started);
    qemu_event_set(&f->dir_release);
    qemu_event_destroy(&f->dir_release);
    qemu_event_destroy(&f->dir_started);
    fixture = NULL;
}

static void set_name(uint8_t dst[PLAN9P1_NAMELEN], const char *name)
{
    memset(dst, 0, PLAN9P1_NAMELEN);
    memcpy(dst, name, MIN(strlen(name), (size_t)PLAN9P1_NAMELEN));
}

static void send_call(ServerFixture *f, Plan9P1Fcall *call, bool fragmented)
{
    uint8_t wire[PLAN9P1_MAX_FRAME];
    Error *err = NULL;
    ssize_t length;
    size_t i;

    length = plan9p1_encode(wire, sizeof(wire), call, &error_abort);
    if (fragmented) {
        for (i = 0; i < length; i++) {
            g_assert_cmpint(plan9p1_server_receive(f->server, wire + i, 1,
                                                   &err), ==, 0);
            g_assert_null(err);
        }
    } else {
        g_assert_cmpint(plan9p1_server_receive(f->server, wire, length, &err),
                        ==, 0);
        g_assert_null(err);
    }
}

static Plan9P1Fcall take_reply(ServerFixture *f)
{
    Plan9P1Fcall reply;

    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, >, 0);
    g_assert_cmpint(plan9p1_decode(f->transport.output->data,
                                  f->transport.output->len,
                                  &reply, &error_abort), ==, 0);
    g_byte_array_set_size(f->transport.output, 0);
    return reply;
}

static Plan9P1Fcall transact(ServerFixture *f, Plan9P1Fcall *call)
{
    send_call(f, call, false);
    return take_reply(f);
}

static void attach(ServerFixture *f, uint16_t fid, uint16_t tag)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TATTACH,
        .tag = tag,
        .fid = fid,
    };
    Plan9P1Fcall reply = transact(f, &call);

    g_assert_cmpuint(reply.type, ==, PLAN9P1_RATTACH);
    g_assert_cmpuint(reply.fid, ==, fid);
}

static Plan9P1Fcall walk(ServerFixture *f, uint16_t fid, uint16_t tag,
                         const char *name)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TWALK,
        .tag = tag,
        .fid = fid,
    };

    set_name(call.name, name);
    return transact(f, &call);
}

static void test_session_reply(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TSESSION,
        .tag = 0x1234,
    };
    uint8_t expected[87] = { PLAN9P1_RSESSION, 0x34, 0x12 };

    send_call(f, &call, true);
    pump_server(f->server);
    g_assert_cmpmem(f->transport.output->data, f->transport.output->len,
                    expected, sizeof(expected));
    g_assert_false(f->backend_on_main);
}

static void test_boot_sequence(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call = { .type = PLAN9P1_TSESSION, .tag = UINT16_MAX };
    Plan9P1Fcall reply = transact(f, &call);

    g_assert_cmpuint(reply.type, ==, PLAN9P1_RSESSION);
    attach(f, 0, 2);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 3, .fid = 0, .newfid = UINT16_MAX,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCLONE);
    g_assert_cmpuint(reply.fid, ==, 0);
    reply = walk(f, UINT16_MAX, 4, "68020");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWALK);
    reply = walk(f, UINT16_MAX, 5, "init");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 6, .fid = UINT16_MAX,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 7, .fid = UINT16_MAX, .count = 64,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 10);
    g_assert_cmpmem(reply.data, reply.count, "boot-init\n", 10);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 8, .fid = UINT16_MAX,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RSTAT);
    g_assert_cmpuint(reply.dir.length, ==, 10);
    g_assert_cmpmem(reply.dir.name, 4, "init", 4);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 9, .fid = UINT16_MAX,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCLUNK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TFLUSH, .tag = 10, .oldtag = 7,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RFLUSH);
    g_assert_false(f->backend_on_main);
    g_assert_cmpuint(f->backend_calls, >, 10);
}

static void test_fid_and_path_errors(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    Plan9P1Qid root_qid;

    attach(f, 1, 1);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TATTACH, .tag = 2, .fid = 1,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 3, .fid = 65535,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    reply = walk(f, 1, 4, "missing");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 5, .fid = 1,
    };
    root_qid = transact(f, &call).dir.qid;
    reply = walk(f, 1, 6, "..");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(reply.qid.path, ==, root_qid.path);
    reply = walk(f, 1, 7, ".");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 8, .fid = 1, .newfid = 65000,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    call.tag = 9;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    attach(f, 2, 10);
    reply = walk(f, 2, 11, "denied");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 12, .fid = 2,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    g_assert_true(g_str_has_prefix((char *)reply.ename, g_strerror(EACCES)));
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 13, .fid = 1,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
}

static Plan9P1Dir decode_dir_record(const uint8_t *record)
{
    uint8_t wire[121] = { PLAN9P1_RSTAT, 1, 0, 0, 0 };
    Plan9P1Fcall fcall;

    memcpy(wire + 5, record, PLAN9P1_DIRLEN);
    g_assert_cmpint(plan9p1_decode(wire, sizeof(wire), &fcall, &error_abort),
                    ==, 0);
    return fcall.dir;
}

static Plan9P1Dir find_dir_record(const Plan9P1Fcall *reply,
                                  const char *name)
{
    unsigned int offset;

    g_assert_cmpuint(reply->type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply->count % PLAN9P1_DIRLEN, ==, 0);
    for (offset = 0; offset < reply->count; offset += PLAN9P1_DIRLEN) {
        Plan9P1Dir dir = decode_dir_record(reply->data + offset);

        if (!strncmp((char *)dir.name, name, PLAN9P1_NAMELEN)) {
            return dir;
        }
    }
    g_error("directory record '%s' was not found", name);
}

static void test_clwalk_and_directory(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    Plan9P1Dir dir;

    attach(f, 10, 1);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLWALK, .tag = 2, .fid = 10, .newfid = 11,
    };
    set_name(call.name, "dir");
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCLWALK);
    g_assert_cmpuint(reply.fid, ==, 11);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 11,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 11,
        .count = PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, PLAN9P1_DIRLEN);
    dir = decode_dir_record(reply.data);
    g_assert_cmpmem(dir.name, 5, "entry", 5);
    call.offset = PLAN9P1_DIRLEN;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, PLAN9P1_DIRLEN);
    call.offset = 2 * PLAN9P1_DIRLEN;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 0);
    call.offset = 1;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    call.offset = 0;
    call.count = PLAN9P1_DIRLEN + 1;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLWALK, .tag = 5, .fid = 10, .newfid = 12,
    };
    set_name(call.name, "missing");
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCLWALK);
    g_assert_cmpuint(reply.fid, ==, 10);
    g_assert_cmpuint(reply.qid.path, ==, 0);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 6, .fid = 12,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    attach(f, 142, 10);
    g_assert_cmpuint(walk(f, 142, 11, "dir").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 12, .fid = 142,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 13, .fid = 142,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
}

static void test_backpressure(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call = { .type = PLAN9P1_TSESSION, .tag = 1 };
    size_t previous;

    f->transport.capacity = 0;
    send_call(f, &call, false);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
    f->transport.capacity = 10;
    plan9p1_server_can_send(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 10);
    previous = f->transport.output->len;
    f->transport.max_chunk = 3;
    while (f->transport.output->len < 87) {
        f->transport.capacity = 20;
        plan9p1_server_can_send(f->server);
        g_assert_cmpuint(f->transport.output->len, >, previous);
        previous = f->transport.output->len;
    }
    g_assert_cmpuint(f->transport.output->len, ==, 87);
}

typedef struct ReplyCollector {
    GPtrArray *calls;
} ReplyCollector;

static int collect_reply(const uint8_t *frame, size_t len,
                         void *opaque, Error **errp)
{
    ReplyCollector *collector = opaque;
    Plan9P1Fcall *call = g_new(Plan9P1Fcall, 1);

    if (plan9p1_decode(frame, len, call, errp) < 0) {
        g_free(call);
        return -1;
    }
    g_ptr_array_add(collector->calls, call);
    return 0;
}

static void test_coalesced_order(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall session = { .type = PLAN9P1_TSESSION, .tag = 1 };
    Plan9P1Fcall attach_call = {
        .type = PLAN9P1_TATTACH, .tag = 2, .fid = 42,
    };
    uint8_t wire[200];
    ssize_t first;
    ssize_t second;
    Plan9P1Stream stream;
    ReplyCollector collector = {
        .calls = g_ptr_array_new_with_free_func(g_free),
    };

    first = plan9p1_encode(wire, sizeof(wire), &session, &error_abort);
    second = plan9p1_encode(wire + first, sizeof(wire) - first,
                            &attach_call, &error_abort);
    g_assert_cmpint(plan9p1_server_receive(f->server, wire, first + second,
                                           &error_abort), ==, 0);
    pump_server(f->server);
    plan9p1_stream_init(&stream);
    g_assert_cmpint(plan9p1_stream_feed(&stream, f->transport.output->data,
                                        f->transport.output->len,
                                        collect_reply, &collector,
                                        &error_abort), ==, 0);
    g_assert_cmpuint(collector.calls->len, ==, 2);
    g_assert_cmpuint(((Plan9P1Fcall *)collector.calls->pdata[0])->type,
                     ==, PLAN9P1_RSESSION);
    g_assert_cmpuint(((Plan9P1Fcall *)collector.calls->pdata[1])->type,
                     ==, PLAN9P1_RATTACH);
    g_ptr_array_unref(collector.calls);
}

static void test_reset_while_queued(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TATTACH, .tag = 1, .fid = 4,
    };

    send_call(f, &call, false);
    g_assert_true(plan9p1_server_busy(f->server));
    plan9p1_server_reset(f->server);
    pump_server(f->server);
    g_byte_array_set_size(f->transport.output, 0);
    attach(f, 4, 2);
}

static void test_session_reconnect(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall session = {
        .type = PLAN9P1_TSESSION, .tag = UINT16_MAX,
    };

    attach(f, 4, 1);
    g_assert_cmpuint(transact(f, &session).type, ==, PLAN9P1_RSESSION);
    attach(f, 4, 2);
}

static void test_clunk_error_invalidates(ServerFixture *f,
                                         gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 30, 1);
    g_assert_cmpuint(walk(f, 30, 2, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 30,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    f->fail_close = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 4, .fid = 30,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    f->fail_close = false;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 5, .fid = 30,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
}

static void test_free_while_queued(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TATTACH, .tag = 1, .fid = 4,
    };
    unsigned int iterations = 0;

    send_call(f, &call, false);
    plan9p1_server_free(f->server);
    f->server = NULL;
    g_assert_cmpuint(f->cleanup_calls, ==, 0);
    while (!f->cleanup_calls) {
        g_assert_cmpuint(iterations++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
    g_assert_cmpuint(f->cleanup_calls, ==, 1);
}

static void test_prepare_delete_pending(ServerFixture *f,
                                        gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    Error *err = NULL;
    unsigned int iterations = 0;

    attach(f, 31, 1);
    g_assert_cmpuint(walk(f, 31, 2, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 31,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);

    f->gate_read = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 31, .count = 1,
    };
    send_call(f, &call, false);
    aio_poll(qemu_get_aio_context(), false);
    qemu_event_wait(&f->read_started);
    g_assert_true(plan9p1_server_busy(f->server));
    g_assert_false(user_creatable_prepare_delete(USER_CREATABLE(f->server),
                                                  &err));
    g_assert_nonnull(err);
    error_free(err);

    /* A rejected deletion must not close the transport or cancel the read. */
    qemu_event_set(&f->read_release);
    pump_server(f->server);
    reply = take_reply(f);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    call = (Plan9P1Fcall) { .type = PLAN9P1_TNOP, .tag = 5 };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RNOP);

    g_assert_true(user_creatable_prepare_delete(USER_CREATABLE(f->server),
                                                 &error_abort));
    plan9p1_server_free(f->server);
    f->server = NULL;
    while (!f->cleanup_calls) {
        g_assert_cmpuint(iterations++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
}

static void test_qid_guards(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall reply;

    f->device_exhaustion = true;
    attach(f, 1, 1);
    reply = walk(f, 1, 2, "plain");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
}

static void test_qid_collision(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    f->inode_collision = true;
    attach(f, 1, 1);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 2, .fid = 1, .newfid = 2,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    g_assert_cmpuint(walk(f, 1, 3, "plain").type, ==, PLAN9P1_RWALK);
    reply = walk(f, 2, 4, "denied");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
}

static void test_transport_failure(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call = { .type = PLAN9P1_TNOP, .tag = 1 };
    uint8_t wire[PLAN9P1_MAX_FRAME];
    Error *err = NULL;
    ssize_t len;

    attach(f, 8, 0);
    f->transport.fail = true;
    send_call(f, &call, false);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
    call = (Plan9P1Fcall) { .type = PLAN9P1_TNOP, .tag = 2 };
    len = plan9p1_encode(wire, sizeof(wire), &call, &error_abort);
    g_assert_cmpint(plan9p1_server_receive(f->server, wire, len, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;

    f->transport.fail = false;
    call = (Plan9P1Fcall) { .type = PLAN9P1_TSESSION, .tag = 3 };
    len = plan9p1_encode(wire, sizeof(wire), &call, &error_abort);
    for (size_t i = 0; i < len; i++) {
        g_assert_cmpint(plan9p1_server_receive(f->server, wire + i, 1,
                                               &err), ==, 0);
        g_assert_null(err);
    }
    pump_server(f->server);
    g_assert_cmpuint(take_reply(f).type, ==, PLAN9P1_RSESSION);
    attach(f, 9, 4);
    g_assert_cmpuint(walk(f, 9, 5, "plain").type, ==, PLAN9P1_RWALK);
}

static void queue_nop_reply(ServerFixture *f)
{
    Plan9P1Fcall call = { .type = PLAN9P1_TNOP, .tag = 7 };

    f->transport.capacity = 0;
    send_call(f, &call, false);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
    f->transport.capacity = SIZE_MAX;
}

static void test_callback_reset(ServerFixture *f, gconstpointer opaque)
{
    queue_nop_reply(f);
    f->transport.action = TRANSPORT_ACTION_RESET_CAN_SEND;
    plan9p1_server_can_send(f->server);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);

    queue_nop_reply(f);
    f->transport.action_done = false;
    f->transport.action = TRANSPORT_ACTION_RESET_SEND;
    plan9p1_server_can_send(f->server);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
    attach(f, 1, 8);
}

static void test_callback_free(ServerFixture *f, gconstpointer opaque)
{
    unsigned int iterations = 0;

    queue_nop_reply(f);
    f->transport.action = TRANSPORT_ACTION_FREE_CAN_SEND;
    plan9p1_server_can_send(f->server);
    while (!f->cleanup_calls) {
        g_assert_cmpuint(iterations++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
    g_assert_null(f->server);
    g_assert_cmpuint(f->cleanup_calls, ==, 1);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
}

static void test_callback_free_send(ServerFixture *f, gconstpointer opaque)
{
    unsigned int iterations = 0;

    queue_nop_reply(f);
    f->transport.action = TRANSPORT_ACTION_FREE_SEND;
    plan9p1_server_can_send(f->server);
    while (!f->cleanup_calls) {
        g_assert_cmpuint(iterations++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
    g_assert_null(f->server);
    g_assert_cmpuint(f->cleanup_calls, ==, 1);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
}

static void test_callback_session(ServerFixture *f, gconstpointer opaque)
{
    static const uint8_t expected[90] = {
        [0] = PLAN9P1_RNOP, [1] = 7,
        [3] = PLAN9P1_RSESSION, [4] = 0x34, [5] = 0x12,
    };

    queue_nop_reply(f);
    f->transport.action = TRANSPORT_ACTION_SESSION_SEND;
    plan9p1_server_can_send(f->server);
    pump_server(f->server);
    g_assert_cmpmem(f->transport.output->data, f->transport.output->len,
                    expected, sizeof(expected));
}

static void test_callback_nop_common(ServerFixture *f,
                                     TransportAction action)
{
    static const uint8_t expected[6] = {
        [0] = PLAN9P1_RNOP, [1] = 7,
        [3] = PLAN9P1_RNOP, [4] = 0x34, [5] = 0x12,
    };

    queue_nop_reply(f);
    f->transport.action = action;
    plan9p1_server_can_send(f->server);
    pump_server(f->server);
    g_assert_cmpmem(f->transport.output->data, f->transport.output->len,
                    expected, sizeof(expected));
}

static void test_callback_nop(ServerFixture *f, gconstpointer opaque)
{
    test_callback_nop_common(f, TRANSPORT_ACTION_NOP_SEND);
}

static void test_callback_can_send_nop(ServerFixture *f,
                                       gconstpointer opaque)
{
    test_callback_nop_common(f, TRANSPORT_ACTION_NOP_CAN_SEND);
}

static void test_callback_fragmented_nop(ServerFixture *f,
                                         gconstpointer opaque)
{
    test_callback_nop_common(f, TRANSPORT_ACTION_FRAGMENTED_NOP_SEND);
}

static void test_flush_cancels_unsent(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall attach_call = {
        .type = PLAN9P1_TATTACH, .tag = 11, .fid = 1,
    };
    Plan9P1Fcall flush = {
        .type = PLAN9P1_TFLUSH, .tag = 12, .oldtag = 11,
    };
    Plan9P1Fcall reply;

    f->transport.capacity = 0;
    send_call(f, &attach_call, false);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
    send_call(f, &flush, false);
    pump_server(f->server);
    f->transport.capacity = SIZE_MAX;
    plan9p1_server_can_send(f->server);
    reply = take_reply(f);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RFLUSH);
    g_assert_cmpuint(reply.tag, ==, 12);
}

static void test_constructor_validation(ServerFixture *f,
                                        gconstpointer opaque)
{
    Plan9P1ServerOptions too_many = { .max_devices = 128 };
    Plan9P1TransportOps incomplete = { .can_send = transport_can_send };
    Plan9P1Server *server;
    Error *err = NULL;

    server = plan9p1_server_new("missing", &transport_ops, &f->transport,
                                NULL, &err);
    g_assert_null(server);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    server = plan9p1_server_new("testfs", &transport_ops, &f->transport,
                                &too_many, &err);
    g_assert_null(server);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    server = plan9p1_server_new("testfs", &incomplete, &f->transport,
                                NULL, &err);
    g_assert_null(server);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_short_receive(ServerFixture *f, gconstpointer opaque)
{
    Error *err = NULL;

    g_assert_cmpint(plan9p1_server_receive(f->server, NULL, 1, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_malformed_and_fatal(ServerFixture *f, gconstpointer opaque)
{
    uint8_t oversized[15] = {
        PLAN9P1_TREAD, 1, 0, 1, 0,
        [13] = 1, [14] = 32,
    };
    uint8_t unknown = 0xff;
    Error *err = NULL;
    Plan9P1Fcall reply;

    g_assert_cmpint(plan9p1_server_receive(f->server, oversized,
                                           sizeof(oversized), &err), ==, 0);
    g_assert_null(err);
    reply = take_reply(f);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(reply.tag, ==, 1);
    g_assert_cmpint(plan9p1_server_receive(f->server, &unknown, 1, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_maximum_read(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 20, 1);
    g_assert_cmpuint(walk(f, 20, 2, "big").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 20,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 20,
        .count = PLAN9P1_MAX_DATA,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, PLAN9P1_MAX_DATA);
}

static void test_read_error(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 21, 1);
    g_assert_cmpuint(walk(f, 21, 2, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 21,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    f->fail_read = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 21, .count = 1,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    g_assert_true(g_str_has_prefix((char *)reply.ename, g_strerror(EIO)));
}

static void test_read_overreport_and_throttle(ServerFixture *f,
                                              gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 25, 1);
    g_assert_cmpuint(walk(f, 25, 2, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 25,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    f->overreport_read = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 25, .count = 1,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    g_assert_true(g_str_has_prefix((char *)reply.ename, g_strerror(EIO)));
    g_assert_cmpuint(f->throttle_reads, ==, 1);
}

static void test_read_offset_range(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 27, 1);
    g_assert_cmpuint(walk(f, 27, 2, "large").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 27,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 27,
        .offset = (uint64_t)INT32_MAX + 4096, .count = 1,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 1);
    call.tag = 5;
    call.offset = INT64_MAX;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    call.tag = 6;
    call.offset = UINT64_MAX;
    call.count = 0;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
}

static void test_stat_large_file(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 22, 1);
    g_assert_cmpuint(walk(f, 22, 2, "large").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 3, .fid = 22,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RSTAT);
    g_assert_cmpuint(reply.dir.length, ==, 122);
}

static void test_walk_parent_stat_name(ServerFixture *f,
                                       gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 23, 1);
    g_assert_cmpuint(walk(f, 23, 2, "a").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 23, 3, "b").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 23, 4, "..").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 23, 5, ".").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 6, .fid = 23,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RSTAT);
    g_assert_cmpmem(reply.dir.name, 1, "a", 1);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 7, .fid = 23, .newfid = 24,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    g_assert_cmpuint(walk(f, 24, 8, "..").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 9, .fid = 24,
    };
    reply = transact(f, &call);
    g_assert_cmpmem(reply.dir.name, 1, "/", 1);
    call.fid = 23;
    call.tag = 10;
    reply = transact(f, &call);
    g_assert_cmpmem(reply.dir.name, 1, "a", 1);
}

static void test_directory_cache_retry(ServerFixture *f,
                                       gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 25, 1);
    g_assert_cmpuint(walk(f, 25, 2, "dir").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 25,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);

    f->fail_dir_lstat_after = 1;
    f->variant_dir_inodes = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 25,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);

    f->fail_dir_lstat_after = 0;
    f->variant_dir_inodes = false;
    f->dir_lstat_count = 0;
    call.tag = 5;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 2 * PLAN9P1_DIRLEN);
}

static void open_directory(ServerFixture *f, uint16_t fid, uint16_t tag);

static Plan9P1Dir read_cached_entry(ServerFixture *f, uint16_t fid,
                                    uint16_t tag)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TREAD, .tag = tag, .fid = fid,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    Plan9P1Fcall reply = transact(f, &call);

    return find_dir_record(&reply, "entry");
}

static void test_directory_cache_mutations(ServerFixture *f,
                                           gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall stat;
    Plan9P1Dir dir;
    char gid[PLAN9P1_NAMELEN];

    attach(f, 300, 1);
    g_assert_cmpuint(walk(f, 300, 2, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 300, 3);
    g_assert_cmpuint(read_cached_entry(f, 300, 4).length, ==, 16);

    attach(f, 301, 5);
    g_assert_cmpuint(walk(f, 301, 6, "dir").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 301, 7, "entry").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 8, .fid = 301,
        .mode = PLAN9P1_ORDWR,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 9, .fid = 301,
        .offset = 16, .count = 1, .data = (const uint8_t *)"x",
    };
    g_assert_cmpuint(transact(f, &call).count, ==, 1);
    g_assert_cmpuint(read_cached_entry(f, 300, 10).length, ==, 17);

    f->short_write = true;
    call.tag = 11;
    call.offset = 17;
    call.count = 2;
    call.data = (const uint8_t *)"yz";
    g_assert_cmpuint(transact(f, &call).count, ==, 1);
    f->short_write = false;
    g_assert_cmpuint(read_cached_entry(f, 300, 12).length, ==, 18);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 13, .fid = 301,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);

    attach(f, 301, 14);
    g_assert_cmpuint(walk(f, 301, 15, "dir").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 301, 16, "entry").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 17, .fid = 301,
        .mode = PLAN9P1_OWRITE | PLAN9P1_OTRUNC,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    g_assert_cmpuint(read_cached_entry(f, 300, 18).length, ==, 0);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 19, .fid = 301,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 20, .fid = 301, .dir = stat.dir,
    };
    call.dir.mode = PLAN9P1_DMAPPEND | 0600;
    call.dir.mtime = 234567;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);
    dir = read_cached_entry(f, 300, 21);
    g_assert_cmpuint(dir.mode & 0777, ==, 0600);
    g_assert_cmphex(dir.mode & PLAN9P1_DMAPPEND, ==, PLAN9P1_DMAPPEND);
    g_assert_cmpuint(dir.mtime, ==, 234567);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 22, .fid = 301,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 23, .fid = 301, .dir = stat.dir,
    };
    call.dir.mode = PLAN9P1_DMAPPEND | 0640;
    snprintf(gid, sizeof(gid), "%ju",
             (uintmax_t)g_ascii_strtoull((char *)stat.dir.gid, NULL, 10) + 1);
    set_name(call.dir.gid, gid);
    f->fail_chown = true;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    f->fail_chown = false;
    dir = read_cached_entry(f, 300, 24);
    g_assert_cmpuint(dir.mode & 0777, ==, 0640);
}

static void test_walk_parent_rollback(ServerFixture *f,
                                      gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall before;
    Plan9P1Fcall after;

    attach(f, 280, 1);
    g_assert_cmpuint(walk(f, 280, 2, "a").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 280, 3, "b").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 4, .fid = 280,
    };
    before = transact(f, &call);

    f->variant_walk_parent_inode = true;
    f->fail_root_resolve = 1;
    g_assert_cmpuint(walk(f, 280, 5, "..").type, ==, PLAN9P1_RERROR);
    f->variant_walk_parent_inode = false;
    call.tag = 6;
    after = transact(f, &call);
    g_assert_cmpuint(after.type, ==, PLAN9P1_RSTAT);
    g_assert_cmpuint(after.dir.qid.path, ==, before.dir.qid.path);
    g_assert_cmpmem(after.dir.name, 1, "b", 1);

    attach(f, 281, 7);
    g_assert_cmpuint(walk(f, 281, 8, "plain").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 280, 9, "..").type, ==, PLAN9P1_RWALK);
    call.tag = 10;
    after = transact(f, &call);
    g_assert_cmpmem(after.dir.name, 1, "a", 1);
}

static void test_candidate_device_rollback(ServerFixture *f,
                                           gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    unsigned int attempt;

    attach(f, 26, 1);
    g_assert_cmpuint(walk(f, 26, 2, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 26, 3);
    f->variant_dir_devices = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .fid = 26,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    for (attempt = 0; attempt < 4; attempt++) {
        f->dir_lstat_count = 0;
        f->fail_dir_lstat_after = 1;
        call.tag = 4 + attempt;
        reply = transact(f, &call);
        g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    }
    f->dir_lstat_count = 0;
    f->fail_dir_lstat_after = 0;
    call.tag = 8;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 2 * PLAN9P1_DIRLEN);
}

static void open_directory(ServerFixture *f, uint16_t fid, uint16_t tag)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TOPEN, .tag = tag, .fid = fid,
    };

    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
}

static void test_directory_cache_bound(ServerFixture *f,
                                       gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 30, 1);
    g_assert_cmpuint(walk(f, 30, 2, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 30, 3);
    attach(f, 31, 4);
    g_assert_cmpuint(walk(f, 31, 5, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 31, 6);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 7, .fid = 30,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 2 * PLAN9P1_DIRLEN);
    call.tag = 8;
    call.fid = 31;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 9, .fid = 30,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 10, .fid = 31,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 2 * PLAN9P1_DIRLEN);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSESSION, .tag = 11,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSESSION);
    attach(f, 32, 12);
    g_assert_cmpuint(walk(f, 32, 13, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 32, 14);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 15, .fid = 32,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREAD);

    plan9p1_server_reset(f->server);
    pump_server(f->server);
    g_byte_array_set_size(f->transport.output, 0);
    attach(f, 33, 16);
    g_assert_cmpuint(walk(f, 33, 17, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 33, 18);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 19, .fid = 33,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREAD);
}

static void test_directory_incremental_bound(ServerFixture *f,
                                             gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 33, 1);
    g_assert_cmpuint(walk(f, 33, 2, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 33, 3);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 4, .fid = 33,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->dir_lstat_count, ==, 1);
}

static void send_directory_flush_bh(void *opaque)
{
    ServerFixture *f = opaque;
    Plan9P1Fcall flush = {
        .type = PLAN9P1_TFLUSH, .tag = 41, .oldtag = 40,
    };

    if (f->dir_gate_reset) {
        plan9p1_server_reset(f->server);
    } else {
        send_call(f, &flush, false);
    }
    qemu_event_set(&f->dir_release);
}

static gpointer wait_for_directory_gate(gpointer opaque)
{
    ServerFixture *f = opaque;

    qemu_event_wait(&f->dir_started);
    aio_bh_schedule_oneshot(qemu_get_aio_context(), send_directory_flush_bh,
                            f);
    return NULL;
}

static void test_directory_flush_cancellation(ServerFixture *f,
                                               gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    GThread *gate_thread;

    attach(f, 34, 1);
    g_assert_cmpuint(walk(f, 34, 2, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 34, 3);
    f->gate_dir_lstat = true;
    f->gate_dir_lstat_after = 2;
    f->transport.capacity = 0;
    gate_thread = g_thread_new("9p1-dir-flush", wait_for_directory_gate, f);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 40, .fid = 34,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    send_call(f, &call, false);
    f->transport.capacity = SIZE_MAX;
    pump_server(f->server);
    g_thread_join(gate_thread);
    reply = take_reply(f);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RFLUSH);
    g_assert_cmpuint(reply.tag, ==, 41);
    g_assert_cmpuint(f->transport.output->len, ==, 0);

    f->gate_dir_lstat = false;
    f->dir_lstat_count = 0;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 42, .fid = 34,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 2 * PLAN9P1_DIRLEN);
}

static void test_directory_reset_cancellation(ServerFixture *f,
                                               gconstpointer opaque)
{
    Plan9P1Fcall call;
    GThread *gate_thread;

    attach(f, 35, 1);
    g_assert_cmpuint(walk(f, 35, 2, "dir").type, ==, PLAN9P1_RWALK);
    open_directory(f, 35, 3);
    f->gate_dir_lstat = true;
    f->gate_dir_lstat_after = 2;
    f->dir_gate_reset = true;
    gate_thread = g_thread_new("9p1-dir-reset", wait_for_directory_gate, f);
    f->transport.capacity = 0;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 50, .fid = 35,
        .count = 2 * PLAN9P1_DIRLEN,
    };
    send_call(f, &call, false);
    pump_server(f->server);
    g_thread_join(gate_thread);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
    f->transport.capacity = SIZE_MAX;
    attach(f, 35, 51);
}

static void test_qid_entry_bound(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    f->inode_collision = true;
    attach(f, 32, 1);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 2, .fid = 32, .newfid = 33,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    g_assert_cmpuint(walk(f, 32, 3, "plain").type, ==, PLAN9P1_RWALK);
    reply = walk(f, 33, 4, "denied");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 5, .fid = 32,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    reply = walk(f, 33, 6, "denied");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);
    reply = walk(f, 33, 7, "big");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RERROR);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSESSION, .tag = 8,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSESSION);
    attach(f, 34, 9);
    reply = walk(f, 34, 10, "denied");
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWALK);
}

static void literal_exchange(ServerFixture *f,
                             const uint8_t *request, size_t request_len,
                             const uint8_t *expected, size_t expected_len)
{
    g_assert_cmpint(plan9p1_server_receive(f->server, request, request_len,
                                           &error_abort), ==, 0);
    pump_server(f->server);
    g_assert_cmpmem(f->transport.output->data, f->transport.output->len,
                    expected, expected_len);
    g_byte_array_set_size(f->transport.output, 0);
}

static void test_literal_boot_wire(ServerFixture *f, gconstpointer opaque)
{
    static const uint8_t tsession[11] = {
        [0] = 84, [1] = 0xff, [2] = 0xff,
    };
    static const uint8_t rsession[87] = {
        [0] = 85, [1] = 0xff, [2] = 0xff,
    };
    static const uint8_t tattach[146] = {
        [0] = 86, [1] = 1, [3] = 1,
    };
    static const uint8_t rattach[26] = {
        [0] = 87, [1] = 1, [3] = 1,
        [5] = 0x00, [6] = 0x01, [7] = 0x00, [8] = 0x81,
        [9] = 0xd0, [10] = 0x07,
    };
    static const uint8_t tclone[7] = {
        [0] = 60, [1] = 2, [3] = 1, [5] = 2,
    };
    static const uint8_t rclone[5] = {
        [0] = 61, [1] = 2, [3] = 1,
    };
    static const uint8_t twalk_cpu[33] = {
        [0] = 62, [1] = 3, [3] = 2,
        [5] = '6', [6] = '8', [7] = '0', [8] = '2', [9] = '0',
    };
    static const uint8_t rwalk_cpu[13] = {
        [0] = 63, [1] = 3, [3] = 2,
        [5] = 0x01, [6] = 0x01, [7] = 0x00, [8] = 0x81,
        [9] = 0xd1, [10] = 0x07,
    };
    static const uint8_t twalk_init[33] = {
        [0] = 62, [1] = 4, [3] = 2,
        [5] = 'i', [6] = 'n', [7] = 'i', [8] = 't',
    };
    static const uint8_t rwalk_init[13] = {
        [0] = 63, [1] = 4, [3] = 2,
        [5] = 0x02, [6] = 0x01, [7] = 0x00, [8] = 0x01,
        [9] = 0xd2, [10] = 0x07,
    };
    static const uint8_t topen[6] = {
        [0] = 64, [1] = 5, [3] = 2,
    };
    static const uint8_t ropen[13] = {
        [0] = 65, [1] = 5, [3] = 2,
        [5] = 0x02, [6] = 0x01, [7] = 0x00, [8] = 0x01,
        [9] = 0xd2, [10] = 0x07,
    };
    static const uint8_t tread[15] = {
        [0] = 68, [1] = 6, [3] = 2, [13] = 16,
    };
    static const uint8_t rread[18] = {
        [0] = 69, [1] = 6, [3] = 2, [5] = 10,
        [8] = 'b', [9] = 'o', [10] = 'o', [11] = 't', [12] = '-',
        [13] = 'i', [14] = 'n', [15] = 'i', [16] = 't', [17] = '\n',
    };
    static const uint8_t tstat[5] = {
        [0] = 76, [1] = 7, [3] = 2,
    };
    static const uint8_t rstat[121] = {
        [0] = 77, [1] = 7, [3] = 2,
        [5] = 'i', [6] = 'n', [7] = 'i', [8] = 't',
        [33] = '4', [34] = '2', [61] = '4', [62] = '3',
        [89] = 0x02, [90] = 0x01, [91] = 0x00, [92] = 0x01,
        [93] = 0xd2, [94] = 0x07,
        [97] = 0xa4, [98] = 0x01,
        [101] = 0xea, [102] = 0x03,
        [105] = 0xd2, [106] = 0x07,
        [109] = 10, [119] = 1,
    };
    static const uint8_t tclwalk[35] = {
        [0] = 80, [1] = 8, [3] = 1, [5] = 3,
        [7] = '6', [8] = '8', [9] = '0', [10] = '2', [11] = '0',
    };
    static const uint8_t rclwalk[13] = {
        [0] = 81, [1] = 8, [3] = 3,
        [5] = 0x01, [6] = 0x01, [7] = 0x00, [8] = 0x81,
        [9] = 0xd1, [10] = 0x07,
    };
    static const uint8_t tclunk[5] = {
        [0] = 72, [1] = 9, [3] = 2,
    };
    static const uint8_t rclunk[5] = {
        [0] = 73, [1] = 9, [3] = 2,
    };
    static const uint8_t tflush[5] = {
        [0] = 56, [1] = 10, [3] = 6,
    };
    static const uint8_t rflush[3] = {
        [0] = 57, [1] = 10,
    };

    f->literal_stats = true;
    literal_exchange(f, tsession, sizeof(tsession),
                     rsession, sizeof(rsession));
    literal_exchange(f, tattach, sizeof(tattach), rattach, sizeof(rattach));
    literal_exchange(f, tclone, sizeof(tclone), rclone, sizeof(rclone));
    literal_exchange(f, twalk_cpu, sizeof(twalk_cpu),
                     rwalk_cpu, sizeof(rwalk_cpu));
    literal_exchange(f, twalk_init, sizeof(twalk_init),
                     rwalk_init, sizeof(rwalk_init));
    literal_exchange(f, topen, sizeof(topen), ropen, sizeof(ropen));
    literal_exchange(f, tread, sizeof(tread), rread, sizeof(rread));
    literal_exchange(f, tstat, sizeof(tstat), rstat, sizeof(rstat));
    literal_exchange(f, tclwalk, sizeof(tclwalk),
                     rclwalk, sizeof(rclwalk));
    literal_exchange(f, tclunk, sizeof(tclunk), rclunk, sizeof(rclunk));
    literal_exchange(f, tflush, sizeof(tflush), rflush, sizeof(rflush));
}

static void test_flush_active_and_queued(ServerFixture *f,
                                         gconstpointer opaque)
{
    static const uint8_t tnop_old[] = { 50, 40, 0 };
    static const uint8_t tread_old[15] = {
        [0] = 68, [1] = 40, [3] = 40, [13] = 1,
    };
    static const uint8_t tstat_old[] = { 76, 40, 0, 40, 0 };
    static const uint8_t tnop_after[] = { 50, 42, 0 };
    static const uint8_t tflush[] = { 56, 41, 0, 40, 0 };
    static const uint8_t expected[] = { 57, 41, 0, 51, 42, 0 };
    Plan9P1Fcall call;

    attach(f, 40, 1);
    g_assert_cmpuint(walk(f, 40, 2, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 40,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);

    f->transport.capacity = 0;
    g_assert_cmpint(plan9p1_server_receive(f->server, tnop_old,
                                           sizeof(tnop_old),
                                           &error_abort), ==, 0);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);

    f->gate_read = true;
    g_assert_cmpint(plan9p1_server_receive(f->server, tread_old,
                                           sizeof(tread_old),
                                           &error_abort), ==, 0);
    aio_poll(qemu_get_aio_context(), false);
    qemu_event_wait(&f->read_started);
    g_assert_cmpint(plan9p1_server_receive(f->server, tstat_old,
                                           sizeof(tstat_old),
                                           &error_abort), ==, 0);
    g_assert_cmpint(plan9p1_server_receive(f->server, tnop_after,
                                           sizeof(tnop_after),
                                           &error_abort), ==, 0);
    g_assert_cmpint(plan9p1_server_receive(f->server, tflush,
                                           sizeof(tflush),
                                           &error_abort), ==, 0);
    f->transport.capacity = SIZE_MAX;
    plan9p1_server_can_send(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 0);

    qemu_event_set(&f->read_release);
    pump_server(f->server);
    g_assert_cmpmem(f->transport.output->data, f->transport.output->len,
                    expected, sizeof(expected));
    g_byte_array_set_size(f->transport.output, 0);
    plan9p1_server_can_send(f->server);
    aio_poll(qemu_get_aio_context(), false);
    g_assert_cmpuint(f->transport.output->len, ==, 0);
}

static void test_writable_lifecycle(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    struct stat st;
    char gid[PLAN9P1_NAMELEN];
    g_autofree char *path = g_build_filename(f->root, "created", NULL);
    g_autofree char *renamed = g_build_filename(f->root, "renamed", NULL);
    g_autofree char *contents = NULL;
    gsize length;

    attach(f, 100, 1);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCREATE, .tag = 2, .fid = 100,
        .mode = 2, .perm = 0666,
    };
    set_name(call.name, "created");
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCREATE);
    g_assert_cmpuint(reply.fid, ==, 100);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 3, .fid = 100,
        .offset = 4, .count = 3, .data = (const uint8_t *)"def",
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWRITE);
    g_assert_cmpuint(reply.fid, ==, 100);
    g_assert_cmpuint(reply.count, ==, 3);
    call.tag = 4;
    call.offset = 0;
    call.data = (const uint8_t *)"abc";
    g_assert_cmpuint(transact(f, &call).count, ==, 3);
    g_assert_cmpuint(f->throttle_writes, ==, 2);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 5, .fid = 100,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RSTAT);
    g_assert_cmpuint(reply.dir.length, ==, 7);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 6, .fid = 100, .dir = reply.dir,
    };
    set_name(call.dir.name, "renamed");
    snprintf(gid, sizeof(gid), "%ju",
             (uintmax_t)g_ascii_strtoull((char *)reply.dir.gid, NULL, 10) + 1);
    set_name(call.dir.gid, gid);
    call.dir.mode = 0600;
    call.dir.mtime = 123456;
    call.dir.length = 0; /* A complete 2E Dir: length is not truncation. */
    call.dir.qid.path ^= 0x1234;
    call.dir.atime ^= 0x1234;
    call.dir.type ^= 0x1234;
    call.dir.dev ^= 0x1234;
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWSTAT);
    g_assert_cmpuint(reply.fid, ==, 100);
    g_assert_cmpuint(f->chown_calls, ==, 1);
    g_assert_false(g_file_test(path, G_FILE_TEST_EXISTS));
    g_assert_cmpint(lstat(renamed, &st), ==, 0);
    g_assert_cmpuint(st.st_mode & 0777, ==, 0600);
    g_assert_cmpint(st.st_mtime, ==, 123456);
    g_assert_true(g_file_get_contents(renamed, &contents, &length, NULL));
    g_assert_cmpuint(length, ==, 7);
    g_assert_cmpmem(contents, length, "abc\0def", 7);
    g_clear_pointer(&contents, g_free);

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 7, .fid = 100,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    attach(f, 100, 8);
    g_assert_cmpuint(walk(f, 100, 9, "renamed").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 10, .fid = 100,
        .mode = 2 | 0x10,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 11, .fid = 100, .count = 8,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpuint(reply.count, ==, 0);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 12, .fid = 100,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREMOVE);
    g_assert_cmpuint(reply.fid, ==, 100);
    g_assert_false(g_file_test(renamed, G_FILE_TEST_EXISTS));
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 13, .fid = 100,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
}

static Plan9P1Fcall create(ServerFixture *f, uint16_t fid, uint16_t tag,
                          const char *name, uint8_t mode, uint32_t perm)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TCREATE, .tag = tag, .fid = fid,
        .mode = mode, .perm = perm,
    };

    set_name(call.name, name);
    return transact(f, &call);
}

static void test_open_modes_and_create(ServerFixture *f,
                                       gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    g_autofree char *plain = g_build_filename(f->root, "plain", NULL);
    g_autofree char *noexec = g_build_filename(f->root, "noexec", NULL);

    g_assert_cmpint(chmod(plain, 0755), ==, 0);
    for (unsigned int access = 0; access < 4; access++) {
        uint16_t fid = 110 + access;

        attach(f, fid, 1 + access * 3);
        g_assert_cmpuint(walk(f, fid, 2 + access * 3, "plain").type,
                         ==, PLAN9P1_RWALK);
        call = (Plan9P1Fcall) {
            .type = PLAN9P1_TOPEN, .tag = 3 + access * 3,
            .fid = fid, .mode = access | PLAN9P1_OCEXEC,
        };
        g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
        call = (Plan9P1Fcall) {
            .type = PLAN9P1_TREAD, .tag = 20 + access,
            .fid = fid, .count = 1,
        };
        reply = transact(f, &call);
        g_assert_cmpuint(reply.type, ==,
                         access == PLAN9P1_OWRITE ?
                         PLAN9P1_RERROR : PLAN9P1_RREAD);
        call = (Plan9P1Fcall) {
            .type = PLAN9P1_TWRITE, .tag = 30 + access,
            .fid = fid, .count = 1, .data = (const uint8_t *)"X",
        };
        reply = transact(f, &call);
        g_assert_cmpuint(reply.type, ==,
                         access == PLAN9P1_OWRITE || access == PLAN9P1_ORDWR ?
                         PLAN9P1_RWRITE : PLAN9P1_RERROR);
    }

    g_assert_cmpint(chmod(plain, 0644), ==, 0);
    attach(f, 126, 40);
    g_assert_cmpuint(walk(f, 126, 41, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 42, .fid = 126,
        .mode = PLAN9P1_OEXEC,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpint(chmod(plain, 0755), ==, 0);

    attach(f, 127, 43);
    g_assert_cmpuint(create(f, 127, 44, "", PLAN9P1_OWRITE, 0666).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 45, "bad/name", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 46, ".", PLAN9P1_OWRITE, 0666).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 47, "..", PLAN9P1_OWRITE, 0666).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 48, "plain", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 49, "locked", PLAN9P1_OWRITE,
                            PLAN9P1_DMLOCK | 0666).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 50, "bad-mode", 0x80, 0666).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 51, "bad-dir", PLAN9P1_OWRITE,
                            PLAN9P1_DMDIR | 0777).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(create(f, 127, 52, "noexec", PLAN9P1_OEXEC,
                            0666).type, ==, PLAN9P1_RERROR);
    g_assert_false(g_file_test(noexec, G_FILE_TEST_EXISTS));

    attach(f, 128, 53);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 54, .fid = 128,
        .mode = PLAN9P1_OREAD | PLAN9P1_ORCLOSE,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    for (unsigned int modifier = PLAN9P1_OTRUNC;
         modifier <= PLAN9P1_ORCLOSE; modifier <<= 1) {
        uint16_t fid = 190 + modifier;

        attach(f, fid, 70 + modifier);
        g_assert_cmpuint(walk(f, fid, 71 + modifier, "dir").type,
                         ==, PLAN9P1_RWALK);
        call = (Plan9P1Fcall) {
            .type = PLAN9P1_TOPEN, .tag = 72 + modifier, .fid = fid,
            .mode = PLAN9P1_OREAD | modifier,
        };
        g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    }
    attach(f, 260, 140);
    g_assert_cmpuint(walk(f, 260, 141, "dir").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 142, .fid = 260,
        .mode = PLAN9P1_OEXEC,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    attach(f, 261, 143);
    g_assert_cmpuint(walk(f, 261, 144, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 145, .fid = 261,
        .mode = PLAN9P1_OEXEC | PLAN9P1_OTRUNC,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    attach(f, 262, 146);
    g_assert_cmpuint(create(f, 262, 147, "dir-trunc",
                            PLAN9P1_OREAD | PLAN9P1_OTRUNC,
                            PLAN9P1_DMDIR | 0700).type,
                     ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 148, .fid = 262,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);

    attach(f, 263, 149);
    g_assert_cmpuint(create(f, 263, 150, "dir-cexec",
                            PLAN9P1_OREAD | PLAN9P1_OCEXEC,
                            PLAN9P1_DMDIR | 0700).type,
                     ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 151, .fid = 263,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);

    attach(f, 264, 152);
    g_assert_cmpuint(create(f, 264, 153, "dir-orclose",
                            PLAN9P1_OREAD | PLAN9P1_ORCLOSE,
                            PLAN9P1_DMDIR | 0700).type,
                     ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 154, .fid = 264,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    g_autofree char *dir_orclose =
        g_build_filename(f->root, "dir-orclose", NULL);
    g_assert_false(g_file_test(dir_orclose, G_FILE_TEST_EXISTS));

    attach(f, 265, 155);
    g_assert_cmpuint(create(f, 265, 156, "dir-exec", PLAN9P1_OEXEC,
                            PLAN9P1_DMDIR | 0700).type,
                     ==, PLAN9P1_RERROR);

    attach(f, 120, 50);
    g_assert_cmpuint(walk(f, 120, 51, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 52, .fid = 120, .mode = 0x04,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    attach(f, 121, 53);
    g_assert_cmpuint(walk(f, 121, 54, "dir").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 55, .fid = 121,
        .mode = PLAN9P1_OWRITE,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    attach(f, 122, 56);
    reply = create(f, 122, 57, "newdir", PLAN9P1_OREAD,
                   PLAN9P1_DMDIR | 0777);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 58, .fid = 122,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    attach(f, 123, 59);
    g_assert_cmpuint(create(f, 123, 60, "newdir", PLAN9P1_OREAD,
                            PLAN9P1_DMDIR | 0777).type,
                     ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 61, .fid = 123,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    attach(f, 123, 62);
    g_assert_cmpuint(walk(f, 123, 63, "newdir").type, ==, PLAN9P1_RWALK);
    call.fid = 123;
    call.tag = 64;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);

    attach(f, 124, 65);
    f->fail_created_lstat = true;
    g_assert_cmpuint(create(f, 124, 66, "rollback", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    f->fail_created_lstat = false;
    g_autofree char *rollback = g_build_filename(f->root, "rollback", NULL);
    g_assert_false(g_file_test(rollback, G_FILE_TEST_EXISTS));
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 67, .fid = 124,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSTAT);

    attach(f, 125, 68);
    f->fail_created_lstat = true;
    g_assert_cmpuint(create(f, 125, 69, "rollback-dir", PLAN9P1_OREAD,
                            PLAN9P1_DMDIR | 0777).type,
                     ==, PLAN9P1_RERROR);
    f->fail_created_lstat = false;
    g_autofree char *rollback_dir =
        g_build_filename(f->root, "rollback-dir", NULL);
    g_assert_false(g_file_test(rollback_dir, G_FILE_TEST_EXISTS));
}

static void test_append_orclose_and_write_edges(ServerFixture *f,
                                                gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    g_autofree char *path = g_build_filename(f->root, "append", NULL);
    g_autofree char *wstat_path =
        g_build_filename(f->root, "wstat-append", NULL);
    g_autofree char *data = NULL;
    gsize length;
    g_autofree uint8_t *maximum = g_malloc0(PLAN9P1_MAX_DATA);

    attach(f, 130, 1);
    reply = create(f, 130, 2, "append", PLAN9P1_ORDWR,
                   PLAN9P1_DMAPPEND | 0666);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 3, .fid = 130,
        .offset = UINT64_MAX, .count = 1, .data = (const uint8_t *)"a",
    };
    g_assert_cmpuint(transact(f, &call).count, ==, 1);
    call.tag = 4;
    call.offset = 0;
    call.data = (const uint8_t *)"b";
    g_assert_cmpuint(transact(f, &call).count, ==, 1);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 5, .fid = 130,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    attach(f, 130, 6);
    g_assert_cmpuint(walk(f, 130, 7, "append").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 8, .fid = 130,
        .mode = PLAN9P1_ORDWR | PLAN9P1_OTRUNC,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 9, .fid = 130,
        .count = 0, .data = NULL,
    };
    g_assert_cmpuint(transact(f, &call).count, ==, 0);
    call.tag = 10;
    call.count = PLAN9P1_MAX_DATA;
    call.data = maximum;
    g_assert_cmpuint(transact(f, &call).count, ==, PLAN9P1_MAX_DATA);
    g_assert_true(g_file_get_contents(path, &data, &length, NULL));
    g_assert_cmpuint(length, ==, PLAN9P1_MAX_DATA + 2);

    attach(f, 131, 11);
    g_assert_cmpuint(create(f, 131, 12, "orclose",
                            PLAN9P1_OWRITE | PLAN9P1_ORCLOSE,
                            0666).type, ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 13, .fid = 131,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    g_autofree char *orclose = g_build_filename(f->root, "orclose", NULL);
    g_assert_false(g_file_test(orclose, G_FILE_TEST_EXISTS));

    attach(f, 132, 14);
    g_assert_cmpuint(walk(f, 132, 15, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 16, .fid = 132,
        .mode = PLAN9P1_OWRITE,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 17, .fid = 132,
        .offset = UINT64_MAX, .count = 1, .data = (const uint8_t *)"x",
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    f->short_write = true;
    call.offset = 0;
    call.count = 2;
    call.data = (const uint8_t *)"xy";
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RWRITE);
    g_assert_cmpuint(reply.count, ==, 1);
    f->short_write = false;
    f->overreport_write = true;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    f->overreport_write = false;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 18, .fid = 130,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);

    attach(f, 133, 19);
    g_assert_cmpuint(create(f, 133, 20, "orclose-fail",
                            PLAN9P1_OWRITE | PLAN9P1_ORCLOSE,
                            0666).type, ==, PLAN9P1_RCREATE);
    f->fail_remove = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 21, .fid = 133,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    f->fail_remove = false;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 22, .fid = 133,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSESSION, .tag = 23,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSESSION);
    g_autofree char *orclose_fail =
        g_build_filename(f->root, "orclose-fail", NULL);
    g_assert_true(g_file_test(orclose_fail, G_FILE_TEST_EXISTS));
    g_assert_cmpint(g_remove(orclose_fail), ==, 0);

    attach(f, 134, 30);
    g_assert_cmpuint(create(f, 134, 31, "wstat-append", PLAN9P1_ORDWR,
                            0666).type, ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 32, .fid = 134,
        .count = 1, .data = (const uint8_t *)"p",
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWRITE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 33, .fid = 134,
    };
    reply = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 34, .fid = 134, .dir = reply.dir,
    };
    call.dir.mode |= PLAN9P1_DMAPPEND;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 35, .fid = 134,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
    attach(f, 134, 36);
    g_assert_cmpuint(walk(f, 134, 37, "wstat-append").type,
                     ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 38, .fid = 134,
        .mode = PLAN9P1_OWRITE | PLAN9P1_OTRUNC,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 39, .fid = 134,
        .offset = UINT64_MAX, .count = 1, .data = (const uint8_t *)"q",
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWRITE);
    g_clear_pointer(&data, g_free);
    g_assert_true(g_file_get_contents(wstat_path, &data, &length, NULL));
    g_assert_cmpuint(length, ==, 2);
    g_assert_cmpmem(data, length, "pq", 2);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 40, .fid = 134,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);
}

static void test_create_rollback_ownership(ServerFixture *f,
                                           gconstpointer opaque)
{
    int (*saved_unlinkat)(FsContext *, V9fsPath *, const char *, int) =
        test_ops.unlinkat;
    int (*saved_remove)(FsContext *, const char *) = test_ops.remove;
    Plan9P1Fcall call;
    Plan9P1Fcall reply;
    g_autofree char *path = g_build_filename(f->root, "rollback", NULL);
    g_autofree char *nopath =
        g_build_filename(f->root, "rollback-nopath", NULL);
    g_autofree char *marker =
        g_build_filename(f->root, "rollback-cache-marker", NULL);
    unsigned int removes;

    attach(f, 271, 1);
    open_directory(f, 271, 2);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 3, .fid = 271,
        .count = (PLAN9P1_MAX_DATA / PLAN9P1_DIRLEN) * PLAN9P1_DIRLEN,
    };
    reply = transact(f, &call);
    find_dir_record(&reply, "plain");
    write_file(marker, "marker", 6);

    attach(f, 270, 1);
    f->fail_created_lstat = true;
    f->fail_close = true;
    g_assert_cmpuint(create(f, 270, 2, "rollback", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    f->fail_created_lstat = false;
    f->fail_close = false;
    g_assert_cmpuint(f->open_file_handles, ==, 0);
    g_assert_cmpuint(f->close_calls, ==, 1);
    g_assert_cmpuint(f->remove_calls, ==, 1);
    g_assert_false(g_file_test(path, G_FILE_TEST_EXISTS));
    call.tag = 4;
    reply = transact(f, &call);
    find_dir_record(&reply, "rollback-cache-marker");

    removes = f->remove_calls;
    f->fail_created_lstat = true;
    f->fail_remove_once = 1;
    g_assert_cmpuint(create(f, 270, 4, "rollback", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    f->fail_created_lstat = false;
    g_assert_cmpuint(f->open_file_handles, ==, 0);
    g_assert_cmpuint(f->remove_calls, ==, removes + 1);
    g_assert_true(g_file_test(path, G_FILE_TEST_EXISTS));
    call.tag = 5;
    reply = transact(f, &call);
    find_dir_record(&reply, "rollback");
    g_assert_cmpint(g_remove(path), ==, 0);

    test_ops.unlinkat = NULL;
    f->fail_created_name_to_path = 1;
    g_assert_cmpuint(create(f, 270, 5, "rollback-nopath", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    g_assert_false(f->remove_null_path);
    g_assert_false(g_file_test(nopath, G_FILE_TEST_EXISTS));

    test_ops.remove = NULL;
    f->fail_created_lstat = true;
    g_assert_cmpuint(create(f, 270, 6, "rollback", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    f->fail_created_lstat = false;
    g_assert_true(g_file_test(path, G_FILE_TEST_EXISTS));
    test_ops.remove = saved_remove;
    g_assert_cmpint(g_remove(path), ==, 0);
    test_ops.unlinkat = saved_unlinkat;

    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 7, .fid = 270,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSTAT);
    g_assert_cmpint(g_remove(marker), ==, 0);
}

static void test_append_identity_survives_remove(ServerFixture *f,
                                                 gconstpointer opaque)
{
    g_autofree char *first = g_build_filename(f->root, "append-a", NULL);
    g_autofree char *second = g_build_filename(f->root, "append-b", NULL);
    Plan9P1Fcall call;
    Plan9P1Fcall stat;

    write_file(first, "x", 1);
    g_assert_cmpint(link(first, second), ==, 0);
    attach(f, 290, 1);
    g_assert_cmpuint(walk(f, 290, 2, "append-a").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 3, .fid = 290,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 4, .fid = 290, .dir = stat.dir,
    };
    call.dir.mode |= PLAN9P1_DMAPPEND;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);

    attach(f, 291, 5);
    g_assert_cmpuint(walk(f, 291, 6, "append-b").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 7, .fid = 291, .newfid = 292,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 8, .fid = 290,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 9, .fid = 292,
    };
    stat = transact(f, &call);
    g_assert_cmpuint(stat.type, ==, PLAN9P1_RSTAT);
    g_assert_cmphex(stat.dir.mode & PLAN9P1_DMAPPEND,
                    ==, PLAN9P1_DMAPPEND);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 10, .fid = 291,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLUNK, .tag = 11, .fid = 292,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLUNK);
}

static void test_wstat_remove_failures(ServerFixture *f,
                                       gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 140, 1);
    g_assert_cmpuint(walk(f, 140, 2, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 3, .fid = 140,
    };
    reply = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 4, .fid = 140, .dir = reply.dir,
    };
    set_name(call.dir.uid, "4294967294");
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call.dir = reply.dir;
    call.dir.mode |= PLAN9P1_DMLOCK;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call.dir = reply.dir;
    set_name(call.dir.gid, "12x");
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);

    f->fail_remove = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 5, .fid = 140,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 6, .fid = 140,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    f->fail_remove = false;

    attach(f, 143, 14);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 15, .fid = 143,
    };
    reply = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 16, .fid = 143, .dir = reply.dir,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 17, .fid = 143,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSTAT);

    attach(f, 141, 7);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 8, .fid = 141,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 9, .fid = 141,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
}

static void test_missing_mutation_ops(ServerFixture *f,
                                      gconstpointer opaque)
{
    int (*saved_open2)(FsContext *, V9fsPath *, const char *, int,
                       FsCred *, V9fsFidOpenState *) = test_ops.open2;
    ssize_t (*saved_pwritev)(FsContext *, V9fsFidOpenState *,
                             const struct iovec *, int, off_t) =
        test_ops.pwritev;
    int (*saved_unlinkat)(FsContext *, V9fsPath *, const char *, int) =
        test_ops.unlinkat;
    int (*saved_remove)(FsContext *, const char *) = test_ops.remove;
    int (*saved_chmod)(FsContext *, V9fsPath *, FsCred *) = test_ops.chmod;
    int (*saved_chown)(FsContext *, V9fsPath *, FsCred *) = test_ops.chown;
    int (*saved_utimensat)(FsContext *, V9fsPath *, const struct timespec *) =
        test_ops.utimensat;
    int (*saved_renameat)(FsContext *, V9fsPath *, const char *,
                          V9fsPath *, const char *) = test_ops.renameat;
    Plan9P1Fcall call;
    Plan9P1Fcall stat;
    char gid[PLAN9P1_NAMELEN];

    attach(f, 180, 1);
    test_ops.open2 = NULL;
    g_assert_cmpuint(create(f, 180, 2, "unsupported", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RERROR);
    test_ops.open2 = saved_open2;

    g_assert_cmpuint(create(f, 180, 3, "supported", PLAN9P1_OWRITE,
                            0666).type, ==, PLAN9P1_RCREATE);
    test_ops.pwritev = NULL;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 4, .fid = 180,
        .count = 1, .data = (const uint8_t *)"x",
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    test_ops.pwritev = saved_pwritev;
    test_ops.unlinkat = NULL;
    test_ops.remove = NULL;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 5, .fid = 180,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    test_ops.unlinkat = saved_unlinkat;
    test_ops.remove = saved_remove;
    g_autofree char *path = g_build_filename(f->root, "supported", NULL);
    g_assert_cmpint(g_remove(path), ==, 0);

    attach(f, 181, 6);
    g_assert_cmpuint(walk(f, 181, 7, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 8, .fid = 181,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 9, .fid = 181, .dir = stat.dir,
    };
    set_name(call.dir.name, "preflight-name");
    call.dir.mode = 0600;
    call.dir.mtime++;
    snprintf(gid, sizeof(gid), "%ju",
             (uintmax_t)g_ascii_strtoull((char *)stat.dir.gid, NULL, 10) + 1);
    set_name(call.dir.gid, gid);

    test_ops.chown = NULL;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->chmod_calls, ==, 0);
    g_assert_cmpuint(f->chown_calls, ==, 0);
    g_assert_cmpuint(f->utimensat_calls, ==, 0);
    g_assert_cmpuint(f->rename_calls, ==, 0);
    test_ops.chown = saved_chown;
    call.tag++;
    test_ops.utimensat = NULL;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->chmod_calls, ==, 0);
    g_assert_cmpuint(f->chown_calls, ==, 0);
    g_assert_cmpuint(f->utimensat_calls, ==, 0);
    g_assert_cmpuint(f->rename_calls, ==, 0);
    test_ops.utimensat = saved_utimensat;
    call.tag++;
    test_ops.renameat = NULL;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->chmod_calls, ==, 0);
    g_assert_cmpuint(f->chown_calls, ==, 0);
    g_assert_cmpuint(f->utimensat_calls, ==, 0);
    g_assert_cmpuint(f->rename_calls, ==, 0);
    test_ops.renameat = saved_renameat;
    call.tag++;
    test_ops.chmod = NULL;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->chmod_calls, ==, 0);
    g_assert_cmpuint(f->chown_calls, ==, 0);
    g_assert_cmpuint(f->utimensat_calls, ==, 0);
    g_assert_cmpuint(f->rename_calls, ==, 0);
    test_ops.chmod = saved_chmod;
}

static void recreate_read_only(ServerFixture *f)
{
    unsigned int cleanup_target = f->cleanup_calls + 1;

    plan9p1_server_reset(f->server);
    pump_server(f->server);
    plan9p1_server_free(f->server);
    f->server = NULL;
    while (f->cleanup_calls < cleanup_target) {
        aio_poll(qemu_get_aio_context(), true);
    }
    f->fse.export_flags |= V9FS_RDONLY;
    f->server = plan9p1_server_new("testfs", &transport_ops,
                                   &f->transport, NULL, &error_abort);
}

static void test_read_only_preflight(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    unsigned int calls;

    recreate_read_only(f);
    attach(f, 150, 1);
    calls = f->backend_calls;
    g_assert_cmpuint(create(f, 150, 2, "blocked", PLAN9P1_OWRITE, 0666).type,
                     ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->backend_calls, ==, calls);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 3, .fid = 150,
        .mode = PLAN9P1_OWRITE,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->backend_calls, ==, calls);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 4, .fid = 150,
        .count = 1, .data = (const uint8_t *)"x",
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->backend_calls, ==, calls);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 5, .fid = 150,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
    g_assert_cmpuint(f->backend_calls, ==, calls);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 6, .fid = 150,
    };
    send_call(f, &call, false);
    pump_server(f->server);
    g_assert_cmpuint(f->transport.output->len, ==, 67);
    g_byte_array_set_size(f->transport.output, 0);
    g_assert_cmpuint(f->backend_calls, ==, calls);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 7, .fid = 150,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
}

static void test_orclose_cleanup(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    g_autofree char *session_file =
        g_build_filename(f->root, "session-remove", NULL);
    g_autofree char *reset_file =
        g_build_filename(f->root, "reset-remove", NULL);
    g_autofree char *free_file =
        g_build_filename(f->root, "free-remove", NULL);
    g_autofree char *failed_file =
        g_build_filename(f->root, "failed-remove", NULL);
    unsigned int cleanup_target;

    attach(f, 160, 1);
    g_assert_cmpuint(create(f, 160, 2, "session-remove",
                            PLAN9P1_OWRITE | PLAN9P1_ORCLOSE, 0666).type,
                     ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSESSION, .tag = 3,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSESSION);
    g_assert_false(g_file_test(session_file, G_FILE_TEST_EXISTS));

    attach(f, 161, 4);
    g_assert_cmpuint(create(f, 161, 5, "reset-remove",
                            PLAN9P1_OWRITE | PLAN9P1_ORCLOSE, 0666).type,
                     ==, PLAN9P1_RCREATE);
    plan9p1_server_reset(f->server);
    pump_server(f->server);
    g_assert_false(g_file_test(reset_file, G_FILE_TEST_EXISTS));

    attach(f, 163, 6);
    g_assert_cmpuint(create(f, 163, 7, "failed-remove",
                            PLAN9P1_OWRITE | PLAN9P1_ORCLOSE, 0666).type,
                     ==, PLAN9P1_RCREATE);
    f->fail_remove = true;
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSESSION, .tag = 8,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSESSION);
    f->fail_remove = false;
    g_assert_true(g_file_test(failed_file, G_FILE_TEST_EXISTS));
    call.tag = 9;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSESSION);
    g_assert_true(g_file_test(failed_file, G_FILE_TEST_EXISTS));
    g_assert_cmpint(g_remove(failed_file), ==, 0);

    attach(f, 162, 10);
    g_assert_cmpuint(create(f, 162, 11, "free-remove",
                            PLAN9P1_OWRITE | PLAN9P1_ORCLOSE, 0666).type,
                     ==, PLAN9P1_RCREATE);
    cleanup_target = f->cleanup_calls + 1;
    plan9p1_server_free(f->server);
    f->server = NULL;
    while (f->cleanup_calls < cleanup_target) {
        aio_poll(qemu_get_aio_context(), true);
    }
    g_assert_false(g_file_test(free_file, G_FILE_TEST_EXISTS));
}

static void test_rename_live_fids(ServerFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall stat;

    attach(f, 170, 1);
    g_assert_cmpuint(walk(f, 170, 2, "a").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 3, .fid = 170, .newfid = 171,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    call.newfid = 172;
    call.tag = 4;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    g_assert_cmpuint(walk(f, 172, 5, "b").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(create(f, 172, 6, "child", PLAN9P1_ORDWR, 0666).type,
                     ==, PLAN9P1_RCREATE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 7, .fid = 170,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 8, .fid = 170, .dir = stat.dir,
    };
    set_name(call.dir.name, "z");
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 9, .fid = 171,
    };
    g_assert_cmpmem(transact(f, &call).dir.name, 1, "z", 1);
    call.fid = 172;
    call.tag = 10;
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RSTAT);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWRITE, .tag = 11, .fid = 172,
        .count = 1, .data = (const uint8_t *)"q",
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWRITE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREMOVE, .tag = 12, .fid = 172,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RREMOVE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 13, .fid = 171,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 14, .fid = 171, .dir = stat.dir,
    };
    set_name(call.dir.name, "a");
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);

    attach(f, 173, 15);
    g_assert_cmpuint(walk(f, 173, 16, "plain").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TCLONE, .tag = 17, .fid = 173, .newfid = 174,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RCLONE);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 18, .fid = 173,
    };
    stat = transact(f, &call);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 19, .fid = 173, .dir = stat.dir,
    };
    set_name(call.dir.name, "file-renamed");
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 20, .fid = 174,
    };
    stat = transact(f, &call);
    g_assert_cmpmem(stat.dir.name, 12, "file-renamed", 12);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 21, .fid = 174,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TWSTAT, .tag = 22, .fid = 173, .dir = stat.dir,
    };
    set_name(call.dir.name, "plain");
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RWSTAT);
}

int main(int argc, char **argv)
{
    Plan9P1ServerOptions one_device = { .max_devices = 1 };
    Plan9P1ServerOptions two_devices = { .max_devices = 2 };
    Plan9P1ServerOptions dir_bound = {
        .max_dir_cache_bytes = 2 * PLAN9P1_DIRLEN,
    };
    Plan9P1ServerOptions one_dir_record = {
        .max_dir_cache_bytes = PLAN9P1_DIRLEN,
    };
    Plan9P1ServerOptions qid_bound = { .max_qid_entries = 2 };
    Plan9P1ServerOptions cache_retry_qids = { .max_qid_entries = 4 };
    Plan9P1ServerOptions walk_rollback_qids = { .max_qid_entries = 4 };

    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    qemu_init_main_loop(&error_abort);
    g_test_add("/plan9-9p1-server/session", ServerFixture, NULL,
               fixture_setup, test_session_reply, fixture_teardown);
    g_test_add("/plan9-9p1-server/boot-sequence", ServerFixture, NULL,
               fixture_setup, test_boot_sequence, fixture_teardown);
    g_test_add("/plan9-9p1-server/fid-path-errors", ServerFixture, NULL,
               fixture_setup, test_fid_and_path_errors, fixture_teardown);
    g_test_add("/plan9-9p1-server/clwalk-directory", ServerFixture, NULL,
               fixture_setup, test_clwalk_and_directory, fixture_teardown);
    g_test_add("/plan9-9p1-server/backpressure", ServerFixture, NULL,
               fixture_setup, test_backpressure, fixture_teardown);
    g_test_add("/plan9-9p1-server/coalesced-order", ServerFixture, NULL,
               fixture_setup, test_coalesced_order, fixture_teardown);
    g_test_add("/plan9-9p1-server/reset-queued", ServerFixture, NULL,
               fixture_setup, test_reset_while_queued, fixture_teardown);
    g_test_add("/plan9-9p1-server/session-reconnect", ServerFixture, NULL,
               fixture_setup, test_session_reconnect, fixture_teardown);
    g_test_add("/plan9-9p1-server/clunk-error", ServerFixture, NULL,
               fixture_setup, test_clunk_error_invalidates, fixture_teardown);
    g_test_add("/plan9-9p1-server/free-queued", ServerFixture, NULL,
               fixture_setup, test_free_while_queued, fixture_teardown);
    g_test_add("/plan9-9p1-server/prepare-delete-pending", ServerFixture,
               NULL, fixture_setup, test_prepare_delete_pending,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/qid-guards", ServerFixture, &one_device,
               fixture_setup, test_qid_guards, fixture_teardown);
    g_test_add("/plan9-9p1-server/qid-collision", ServerFixture, NULL,
               fixture_setup, test_qid_collision, fixture_teardown);
    g_test_add("/plan9-9p1-server/transport-failure", ServerFixture, NULL,
               fixture_setup, test_transport_failure, fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-reset", ServerFixture, NULL,
               fixture_setup, test_callback_reset, fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-free", ServerFixture, NULL,
               fixture_setup, test_callback_free, fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-free-send", ServerFixture, NULL,
               fixture_setup, test_callback_free_send, fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-session", ServerFixture, NULL,
               fixture_setup, test_callback_session, fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-nop", ServerFixture, NULL,
               fixture_setup, test_callback_nop, fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-can-send-nop", ServerFixture,
               NULL, fixture_setup, test_callback_can_send_nop,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/callback-fragmented-nop", ServerFixture,
               NULL, fixture_setup, test_callback_fragmented_nop,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/flush-cancels-unsent", ServerFixture, NULL,
               fixture_setup, test_flush_cancels_unsent, fixture_teardown);
    g_test_add("/plan9-9p1-server/constructor-validation", ServerFixture,
               NULL, fixture_setup, test_constructor_validation,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/short-receive", ServerFixture, NULL,
               fixture_setup, test_short_receive, fixture_teardown);
    g_test_add("/plan9-9p1-server/malformed-fatal", ServerFixture, NULL,
               fixture_setup, test_malformed_and_fatal, fixture_teardown);
    g_test_add("/plan9-9p1-server/maximum-read", ServerFixture, NULL,
               fixture_setup, test_maximum_read, fixture_teardown);
    g_test_add("/plan9-9p1-server/read-error", ServerFixture, NULL,
               fixture_setup, test_read_error, fixture_teardown);
    g_test_add("/plan9-9p1-server/read-overreport-throttle", ServerFixture,
               NULL, fixture_setup, test_read_overreport_and_throttle,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/read-offset-range", ServerFixture, NULL,
               fixture_setup, test_read_offset_range, fixture_teardown);
    g_test_add("/plan9-9p1-server/stat-large-file", ServerFixture, NULL,
               fixture_setup, test_stat_large_file, fixture_teardown);
    g_test_add("/plan9-9p1-server/walk-parent-stat-name", ServerFixture,
               NULL, fixture_setup, test_walk_parent_stat_name,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/directory-cache-retry", ServerFixture,
               &cache_retry_qids, fixture_setup, test_directory_cache_retry,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/directory-cache-mutations",
               ServerFixture, NULL, fixture_setup,
               test_directory_cache_mutations, fixture_teardown);
    g_test_add("/plan9-9p1-server/walk-parent-rollback", ServerFixture,
               &walk_rollback_qids, fixture_setup, test_walk_parent_rollback,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/candidate-device-rollback",
               ServerFixture, &two_devices, fixture_setup,
               test_candidate_device_rollback, fixture_teardown);
    g_test_add("/plan9-9p1-server/directory-cache-bound", ServerFixture,
               &dir_bound, fixture_setup, test_directory_cache_bound,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/directory-incremental-bound",
               ServerFixture, &one_dir_record, fixture_setup,
               test_directory_incremental_bound, fixture_teardown);
    g_test_add("/plan9-9p1-server/directory-flush-cancellation",
               ServerFixture, NULL, fixture_setup,
               test_directory_flush_cancellation, fixture_teardown);
    g_test_add("/plan9-9p1-server/directory-reset-cancellation",
               ServerFixture, NULL, fixture_setup,
               test_directory_reset_cancellation, fixture_teardown);
    g_test_add("/plan9-9p1-server/qid-entry-bound", ServerFixture,
               &qid_bound, fixture_setup, test_qid_entry_bound,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/literal-boot-wire", ServerFixture, NULL,
               fixture_setup, test_literal_boot_wire, fixture_teardown);
    g_test_add("/plan9-9p1-server/flush-active-queued", ServerFixture, NULL,
               fixture_setup, test_flush_active_and_queued,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/writable-lifecycle", ServerFixture, NULL,
               fixture_setup, test_writable_lifecycle, fixture_teardown);
    g_test_add("/plan9-9p1-server/open-modes-create", ServerFixture, NULL,
               fixture_setup, test_open_modes_and_create, fixture_teardown);
    g_test_add("/plan9-9p1-server/append-orclose-write-edges", ServerFixture,
               NULL, fixture_setup, test_append_orclose_and_write_edges,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/create-rollback-ownership", ServerFixture,
               NULL, fixture_setup, test_create_rollback_ownership,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/append-identity-remove", ServerFixture,
               NULL, fixture_setup, test_append_identity_survives_remove,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/wstat-remove-failures", ServerFixture,
               NULL, fixture_setup, test_wstat_remove_failures,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/missing-mutation-ops", ServerFixture,
               NULL, fixture_setup, test_missing_mutation_ops,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/read-only-preflight", ServerFixture,
               NULL, fixture_setup, test_read_only_preflight,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/orclose-cleanup", ServerFixture,
               NULL, fixture_setup, test_orclose_cleanup,
               fixture_teardown);
    g_test_add("/plan9-9p1-server/rename-live-fids", ServerFixture,
               NULL, fixture_setup, test_rename_live_fids,
               fixture_teardown);
    return g_test_run();
}
