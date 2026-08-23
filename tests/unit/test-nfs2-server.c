/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "fsdev/qemu-fsdev.h"
#include "hw/9pfs/9p.h"
#include "hw/nfs/nfs2-protocol.h"
#include "hw/nfs/nfs2-server.h"
#include "hw/nfs/nfs2-xdr.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/main-loop.h"
#include "qemu/xattr.h"

typedef struct FakeOpen {
    bool directory;
    bool emitted;
    uint64_t ino;
    mode_t mode;
    bool verifier_present;
    uint8_t verifier[8];
    struct dirent entry;
} FakeOpen;

typedef struct Fixture {
    FsDriverEntry fse;
    Nfs2Server *server;
    GByteArray *reply;
    GThread *main_thread;
    bool backend_on_main;
    uint64_t kernel_ino;
    dev_t kernel_dev;
    uint64_t root_ino;
    uint64_t link_ino;
    int lstat_error;
    bool replace_on_open;
    bool fail_replaced_open;
    bool replace_on_opendir;
    bool fail_replaced_opendir;
    bool replace_on_readlink;
    bool fail_replaced_readlink;
    bool replace_on_lookup;
    bool replace_child_after_lstat;
    bool replace_parent_on_second_child_lstat;
    unsigned int child_lstat_calls;
    bool remove_child_on_lookup;
    bool fail_replaced_statfs;
    bool kernel_resolution_notdir;
    bool postvalidate_notdir;
    unsigned int name_to_path_calls;
    uint64_t backend_cookie;
    uint64_t last_seek_cookie;
    bool zero_cookie_continuation;
    unsigned int mutation_calls;
    unsigned int setuid_calls;
    mode_t kernel_mode;
    off_t kernel_size;
    char object_name[NFS2_MAX_NAME + 1];
    char linked_name[NFS2_MAX_NAME + 1];
    mode_t object_mode;
    uint64_t object_ino;
    bool object_present;
    bool linked_present;
    mode_t linked_mode;
    uint64_t linked_ino;
    bool linked_verifier_present;
    uint8_t linked_verifier[8];
    uint32_t mutation_xid;
    unsigned int send_count;
    unsigned int clock_calls;
    int64_t clock_ms;
    int mutation_error;
    bool reset_on_send;
    bool verifier_present;
    bool fail_verifier_set;
    uint8_t verifier[8];
    GMutex fake_lock;
    gint race_active;
    gint race_max;
    bool race_delay;
    int link_error;
    bool replace_link_result;
    bool fail_link_lstat_once;
    bool track_exclusive;
    unsigned int exclusive_order;
    unsigned int verifier_set_order;
    unsigned int file_fsync_order;
    unsigned int publish_order;
    unsigned int temp_unlink_order;
    unsigned int dir_fsync_order;
    unsigned int dir_fsync_calls;
    unsigned int unlink_calls;
    int file_fsync_error;
    int dir_fsync_error;
    int unlink_error;
    bool replace_after_fget;
    bool replace_after_fset;
    bool exclusive_publish_collision;
    bool replace_temp_before_publish;
    uint64_t created_open_ino;
    unsigned int named_exclusive_creates;
} Fixture;

static Fixture *current;

FsDriverEntry *get_fsdev_fsentry(const char *id)
{
    return current && !g_strcmp0(id, current->fse.fsdev_id) ?
           &current->fse : NULL;
}

void fsdev_throttle_init(FsThrottle *fst) { }
void fsdev_throttle_cleanup(FsThrottle *fst) { }

static void note_backend(void)
{
    current->backend_on_main |= g_thread_self() == current->main_thread;
}

static void fake_race_window(void)
{
    gint active, observed;

    if (!current->race_delay) {
        return;
    }
    active = g_atomic_int_add(&current->race_active, 1) + 1;
    do {
        observed = g_atomic_int_get(&current->race_max);
    } while (active > observed &&
             !g_atomic_int_compare_and_exchange(&current->race_max,
                                                observed, active));
    g_usleep(30000);
    g_atomic_int_add(&current->race_active, -1);
}

static int fake_init(FsContext *ctx, Error **errp) { return 0; }
static void fake_cleanup(FsContext *ctx) { }

static int set_path(V9fsPath *path, const char *text)
{
    g_free(path->data);
    path->data = g_strdup(text);
    path->size = strlen(text) + 1;
    return 0;
}

static int fake_name_to_path(FsContext *ctx, V9fsPath *dir,
                             const char *name, V9fsPath *path)
{
    note_backend();
    current->name_to_path_calls++;
    if (!dir) {
        if (strcmp(name, "/")) {
            errno = EINVAL;
            return -1;
        }
        return set_path(path, "/");
    }
    if (!name[0] || strchr(name, '/')) {
        errno = EINVAL;
        return -1;
    }
    if (!strcmp(dir->data, "/") &&
        (!strcmp(name, "kernel") || !strcmp(name, "link") ||
         !strcmp(name, "collision") || !strcmp(name, "hardlink") ||
         (current->linked_present && !strcmp(name,
                                              current->linked_name)) ||
         (current->object_present && !strcmp(name,
                                              current->object_name)))) {
        g_autofree char *joined = g_strconcat("/", name, NULL);

        if (!strcmp(name, "kernel") && current->kernel_resolution_notdir) {
            errno = ENOTDIR;
            return -1;
        }
        if (current->remove_child_on_lookup) {
            current->root_ino++;
            current->remove_child_on_lookup = false;
            errno = ENOENT;
            return -1;
        }
        if (current->replace_on_lookup) {
            current->root_ino++;
            current->replace_on_lookup = false;
        }
        return set_path(path, joined);
    }
    errno = ENOENT;
    return -1;
}

static int fake_lstat(FsContext *ctx, V9fsPath *path, struct stat *st)
{
    note_backend();
    if (current->postvalidate_notdir) {
        current->postvalidate_notdir = false;
        errno = ENOTDIR;
        return -1;
    }
    memset(st, 0, sizeof(*st));
    st->st_dev = 9;
    st->st_uid = 12;
    st->st_gid = 34;
    st->st_blksize = 4096;
    st->st_atim.tv_sec = st->st_mtim.tv_sec = st->st_ctim.tv_sec = 1;
    if (!strcmp(path->data, "/")) {
        st->st_mode = S_IFDIR | 0755;
        st->st_nlink = 2;
        st->st_ino = current->root_ino;
        return 0;
    }
    if (!strcmp(path->data, "/kernel")) {
        if (current->lstat_error) {
            errno = current->lstat_error;
            return -1;
        }
        st->st_dev = current->kernel_dev;
        st->st_mode = current->kernel_mode;
        st->st_nlink = current->linked_present &&
                       current->linked_ino == current->kernel_ino ? 2 : 1;
        st->st_ino = current->kernel_ino;
        st->st_size = current->kernel_size;
        st->st_blocks = 1;
        if (current->replace_child_after_lstat) {
            current->kernel_ino++;
            current->replace_child_after_lstat = false;
        }
        if (current->replace_parent_on_second_child_lstat &&
            ++current->child_lstat_calls == 2) {
            current->root_ino++;
            current->replace_parent_on_second_child_lstat = false;
        }
        return 0;
    }
    if (!strcmp(path->data, "/link")) {
        st->st_mode = S_IFLNK | 0777;
        st->st_nlink = 1;
        st->st_ino = current->link_ino;
        st->st_size = 6;
        return 0;
    }
    if (!strcmp(path->data, "/collision")) {
        st->st_dev = 8;
        st->st_mode = S_IFREG | 0444;
        st->st_nlink = 1;
        st->st_ino = 2;
        st->st_size = 3;
        return 0;
    }
    if (!strcmp(path->data, "/hardlink")) {
        st->st_dev = 9;
        st->st_mode = S_IFREG | 0555;
        st->st_nlink = 2;
        st->st_ino = UINT64_C(0x100000002);
        st->st_size = 6;
        return 0;
    }
    if (current->linked_present && path->data[0] == '/' &&
        !strcmp(path->data + 1, current->linked_name)) {
        if (current->fail_link_lstat_once) {
            current->fail_link_lstat_once = false;
            errno = EIO;
            return -1;
        }
        if (current->replace_link_result) {
            current->linked_ino++;
            current->linked_verifier_present = false;
            current->replace_link_result = false;
        }
        st->st_mode = current->linked_mode;
        st->st_nlink = current->linked_ino == current->kernel_ino ||
                       (current->object_present &&
                        current->linked_ino == current->object_ino) ? 2 : 1;
        st->st_ino = current->linked_ino;
        return 0;
    }
    if (current->object_present && path->data[0] == '/' &&
        !strcmp(path->data + 1, current->object_name)) {
        st->st_mode = current->object_mode;
        st->st_nlink = current->linked_present &&
                       current->linked_ino == current->object_ino ? 2 : 1;
        st->st_ino = current->object_ino;
        st->st_size = 0;
        return 0;
    }
    errno = ENOENT;
    return -1;
}

static ssize_t fake_readlink(FsContext *ctx, V9fsPath *path,
                             char *buffer, size_t size)
{
    static const char target[] = "kernel";

    note_backend();
    if (strcmp(path->data, "/link")) {
        errno = EINVAL;
        return -1;
    }
    memcpy(buffer, target, MIN(size, sizeof(target) - 1));
    if (current->replace_on_readlink) {
        current->link_ino++;
        current->replace_on_readlink = false;
    }
    if (current->fail_replaced_readlink) {
        current->link_ino++;
        current->fail_replaced_readlink = false;
        errno = EINVAL;
        return -1;
    }
    return MIN(size, sizeof(target) - 1);
}

static int fake_open(FsContext *ctx, V9fsPath *path, int flags,
                     V9fsFidOpenState *state)
{
    bool kernel = !strcmp(path->data, "/kernel");
    bool object = current->object_present && path->data[0] == '/' &&
                  !strcmp(path->data + 1, current->object_name);
    bool linked = current->linked_present && path->data[0] == '/' &&
                  !strcmp(path->data + 1, current->linked_name);

    note_backend();
    if (!kernel && !object && !linked) {
        errno = EISDIR;
        return -1;
    }
    if (current->fail_replaced_open) {
        current->kernel_ino++;
        current->fail_replaced_open = false;
        current->postvalidate_notdir = true;
        errno = ELOOP;
        return -1;
    }
    state->private = g_new0(FakeOpen, 1);
    if (current->replace_on_open) {
        current->kernel_ino++;
        current->replace_on_open = false;
    }
    ((FakeOpen *)state->private)->ino = kernel ? current->kernel_ino :
                                        linked ? current->linked_ino :
                                        current->object_ino;
    ((FakeOpen *)state->private)->mode = kernel ? current->kernel_mode :
                                         linked ? current->linked_mode :
                                         current->object_mode;
    if (!kernel) {
        ((FakeOpen *)state->private)->verifier_present =
            linked ? current->linked_verifier_present :
            current->verifier_present;
        memcpy(((FakeOpen *)state->private)->verifier,
               linked ? current->linked_verifier : current->verifier,
               sizeof(current->verifier));
    }
    return 0;
}

static int fake_close(FsContext *ctx, V9fsFidOpenState *state)
{
    note_backend();
    g_free(state->private);
    state->private = NULL;
    return 0;
}

static ssize_t fake_preadv(FsContext *ctx, V9fsFidOpenState *state,
                           const struct iovec *iov, int iovcnt, off_t offset)
{
    static const char data[] = "NetBSD";
    size_t length;

    note_backend();
    if (offset >= sizeof(data) - 1) {
        return 0;
    }
    length = MIN(iov[0].iov_len, sizeof(data) - 1 - offset);
    memcpy(iov[0].iov_base, data + offset, length);
    return length;
}

static ssize_t fake_pwritev(FsContext *ctx, V9fsFidOpenState *state,
                            const struct iovec *iov, int iovcnt, off_t offset)
{
    note_backend();
    current->mutation_calls++;
    current->kernel_size = MAX(current->kernel_size,
                               offset + (off_t)iov[0].iov_len);
    return iov[0].iov_len;
}

static int fake_fsync(FsContext *ctx, int fid_type,
                      V9fsFidOpenState *state, int datasync)
{
    note_backend();
    current->mutation_calls++;
    if (current->track_exclusive) {
        unsigned int order = ++current->exclusive_order;

        if (fid_type == P9_FID_DIR) {
            current->dir_fsync_calls++;
            current->dir_fsync_order = order;
            if (current->dir_fsync_error) {
                errno = current->dir_fsync_error;
                return -1;
            }
        } else {
            current->file_fsync_order = order;
            if (current->file_fsync_error) {
                errno = current->file_fsync_error;
                return -1;
            }
        }
    }
    if (current->mutation_error) {
        errno = current->mutation_error;
        return -1;
    }
    return 0;
}

static int fake_chmod(FsContext *ctx, V9fsPath *path, FsCred *cred)
{
    note_backend();
    current->mutation_calls++;
    current->kernel_mode = (current->kernel_mode & S_IFMT) |
                           (cred->fc_mode & 07777);
    return 0;
}

static int fake_truncate(FsContext *ctx, V9fsPath *path, off_t size)
{
    note_backend();
    current->mutation_calls++;
    current->kernel_size = size;
    return 0;
}

static int fake_utimensat(FsContext *ctx, V9fsPath *path,
                          const struct timespec *times)
{
    note_backend();
    current->mutation_calls++;
    return 0;
}

static int fake_open2(FsContext *ctx, V9fsPath *dir, const char *name,
                      int flags, FsCred *cred, V9fsFidOpenState *state)
{
    note_backend();
    current->mutation_calls++;
    if (g_str_has_prefix(name, ".qemu-nfs3-exclusive-")) {
        current->named_exclusive_creates++;
    }
    if (current->object_present && !strcmp(name, current->object_name) &&
        (flags & O_EXCL)) {
        errno = EEXIST;
        return -1;
    }
    g_strlcpy(current->object_name, name, sizeof(current->object_name));
    current->object_mode = S_IFREG | (cred->fc_mode & 07777);
    current->object_ino++;
    current->object_present = true;
    current->verifier_present = false;
    state->private = g_new0(FakeOpen, 1);
    ((FakeOpen *)state->private)->ino = current->object_ino;
    ((FakeOpen *)state->private)->mode = current->object_mode;
    current->created_open_ino = current->object_ino;
    return 0;
}

static int fake_open_tmpfile(FsContext *ctx, V9fsPath *dir, FsCred *cred,
                             V9fsFidOpenState *state)
{
    FakeOpen *open;

    note_backend();
    current->mutation_calls++;
    open = g_new0(FakeOpen, 1);
    open->ino = ++current->object_ino;
    open->mode = S_IFREG | (cred->fc_mode & 07777);
    current->created_open_ino = open->ino;
    state->private = open;
    return 0;
}

static int fake_mkdir(FsContext *ctx, V9fsPath *dir, const char *name,
                      FsCred *cred)
{
    note_backend();
    current->mutation_calls++;
    g_strlcpy(current->object_name, name, sizeof(current->object_name));
    current->object_mode = S_IFDIR | (cred->fc_mode & 07777);
    current->object_ino++;
    current->object_present = true;
    return 0;
}

static int fake_mknod(FsContext *ctx, V9fsPath *dir, const char *name,
                      FsCred *cred)
{
    note_backend();
    current->mutation_calls++;
    g_strlcpy(current->object_name, name, sizeof(current->object_name));
    current->object_mode = cred->fc_mode;
    current->object_ino++;
    current->object_present = true;
    return 0;
}

static int fake_symlink(FsContext *ctx, const char *target, V9fsPath *dir,
                        const char *name, FsCred *cred)
{
    note_backend();
    current->mutation_calls++;
    g_strlcpy(current->object_name, name, sizeof(current->object_name));
    current->object_mode = S_IFLNK | 0777;
    current->object_ino++;
    current->object_present = true;
    return 0;
}

static int fake_unlinkat(FsContext *ctx, V9fsPath *dir, const char *name,
                         int flags)
{
    note_backend();
    current->mutation_calls++;
    if (current->track_exclusive) {
        current->unlink_calls++;
        if (g_str_has_prefix(name, ".qemu-nfs3-exclusive-")) {
            current->temp_unlink_order = ++current->exclusive_order;
        }
        if (current->unlink_error) {
            errno = current->unlink_error;
            return -1;
        }
    }
    fake_race_window();
    g_mutex_lock(&current->fake_lock);
    if (current->linked_present && !strcmp(name, current->linked_name)) {
        current->linked_present = false;
        g_mutex_unlock(&current->fake_lock);
        return 0;
    }
    if (!current->object_present || strcmp(name, current->object_name)) {
        g_mutex_unlock(&current->fake_lock);
        errno = ENOENT;
        return -1;
    }
    if (current->linked_present &&
        current->linked_ino == current->object_ino) {
        g_strlcpy(current->object_name, current->linked_name,
                  sizeof(current->object_name));
        current->linked_present = false;
    } else {
        current->object_present = false;
        current->verifier_present = false;
    }
    g_mutex_unlock(&current->fake_lock);
    return 0;
}

static int fake_renameat(FsContext *ctx, V9fsPath *olddir,
                         const char *oldname, V9fsPath *newdir,
                         const char *newname)
{
    note_backend();
    current->mutation_calls++;
    fake_race_window();
    g_mutex_lock(&current->fake_lock);
    if (!current->object_present || strcmp(oldname, current->object_name)) {
        g_mutex_unlock(&current->fake_lock);
        errno = ENOENT;
        return -1;
    }
    g_strlcpy(current->object_name, newname, sizeof(current->object_name));
    g_mutex_unlock(&current->fake_lock);
    return 0;
}

static int fake_link(FsContext *ctx, V9fsPath *oldpath, V9fsPath *newdir,
                     const char *name)
{
    note_backend();
    current->mutation_calls++;
    if (current->link_error) {
        errno = current->link_error;
        return -1;
    }
    if (current->exclusive_publish_collision) {
        current->exclusive_publish_collision = false;
        g_strlcpy(current->linked_name, name,
                  sizeof(current->linked_name));
        current->linked_mode = S_IFREG | 0600;
        current->linked_ino = current->object_ino + 100;
        current->linked_present = true;
        errno = EEXIST;
        return -1;
    }
    if (current->replace_temp_before_publish &&
        g_str_has_prefix(oldpath->data, "/.qemu-nfs3-exclusive-")) {
        current->replace_temp_before_publish = false;
        current->object_ino++;
        current->verifier_present = false;
    }
    if (current->track_exclusive &&
        g_str_has_prefix(oldpath->data, "/.qemu-nfs3-exclusive-")) {
        current->publish_order = ++current->exclusive_order;
    }
    g_strlcpy(current->linked_name, name, sizeof(current->linked_name));
    current->linked_mode = !strcmp(oldpath->data, "/kernel") ?
                           current->kernel_mode : current->object_mode;
    current->linked_ino = !strcmp(oldpath->data, "/kernel") ?
                          current->kernel_ino : current->object_ino;
    current->linked_verifier_present = current->verifier_present;
    memcpy(current->linked_verifier, current->verifier,
           sizeof(current->linked_verifier));
    current->linked_present = true;
    return 0;
}

static int fake_flinkat(FsContext *ctx, int fid_type,
                        V9fsFidOpenState *state, V9fsPath *newdir,
                        const char *name)
{
    FakeOpen *open = state->private;

    note_backend();
    current->mutation_calls++;
    if (fid_type != P9_FID_FILE) {
        errno = EBADF;
        return -1;
    }
    if (current->exclusive_publish_collision) {
        current->exclusive_publish_collision = false;
        g_strlcpy(current->linked_name, name,
                  sizeof(current->linked_name));
        current->linked_mode = S_IFREG | 0600;
        current->linked_ino = current->object_ino + 100;
        current->linked_verifier_present = false;
        current->linked_present = true;
        errno = EEXIST;
        return -1;
    }
    if (current->replace_temp_before_publish) {
        current->replace_temp_before_publish = false;
        current->object_ino++;
        current->verifier_present = false;
    }
    if (current->track_exclusive) {
        current->publish_order = ++current->exclusive_order;
    }
    g_strlcpy(current->linked_name, name, sizeof(current->linked_name));
    current->linked_mode = open->mode;
    current->linked_ino = open->ino;
    current->linked_verifier_present = open->verifier_present;
    memcpy(current->linked_verifier, open->verifier,
           sizeof(current->linked_verifier));
    current->linked_present = true;
    return 0;
}

static int fake_setuid(FsContext *ctx, uid_t uid)
{
    current->setuid_calls++;
    return 0;
}

static ssize_t fake_lgetxattr(FsContext *ctx, V9fsPath *path,
                              const char *name, void *value, size_t size)
{
    note_backend();
    if (strcmp(name, "user.qemu.nfs3.createverf") ||
        !current->object_present || !current->verifier_present) {
        errno = ENODATA;
        return -1;
    }
    if (size < sizeof(current->verifier)) {
        errno = ERANGE;
        return -1;
    }
    memcpy(value, current->verifier, sizeof(current->verifier));
    return sizeof(current->verifier);
}

static int fake_lsetxattr(FsContext *ctx, V9fsPath *path, const char *name,
                          void *value, size_t size, int flags)
{
    note_backend();
    if (current->fail_verifier_set) {
        errno = EOPNOTSUPP;
        return -1;
    }
    if (strcmp(name, "user.qemu.nfs3.createverf") ||
        size != sizeof(current->verifier)) {
        errno = EINVAL;
        return -1;
    }
    if ((flags & XATTR_CREATE) && current->verifier_present) {
        errno = EEXIST;
        return -1;
    }
    memcpy(current->verifier, value, sizeof(current->verifier));
    current->verifier_present = true;
    return 0;
}

static ssize_t fake_fgetxattr(FsContext *ctx, int fid_type,
                              V9fsFidOpenState *state, const char *name,
                              void *value, size_t size)
{
    FakeOpen *open = state->private;

    note_backend();
    if (fid_type != P9_FID_FILE ||
        strcmp(name, "user.qemu.nfs3.createverf") ||
        !open->verifier_present) {
        errno = ENODATA;
        return -1;
    }
    if (size < sizeof(open->verifier)) {
        errno = ERANGE;
        return -1;
    }
    memcpy(value, open->verifier, sizeof(open->verifier));
    if (current->replace_after_fget) {
        current->replace_after_fget = false;
        if (current->linked_present && current->linked_ino == open->ino) {
            current->linked_ino++;
            current->linked_verifier_present = false;
        } else {
            current->object_ino++;
            current->verifier_present = false;
        }
    }
    return sizeof(open->verifier);
}

static int fake_fsetxattr(FsContext *ctx, int fid_type,
                          V9fsFidOpenState *state, const char *name,
                          void *value, size_t size, int flags)
{
    FakeOpen *open = state->private;

    note_backend();
    if (current->track_exclusive) {
        current->verifier_set_order = ++current->exclusive_order;
    }
    if (current->fail_verifier_set) {
        errno = EOPNOTSUPP;
        return -1;
    }
    if (fid_type != P9_FID_FILE ||
        strcmp(name, "user.qemu.nfs3.createverf") ||
        size != sizeof(open->verifier)) {
        errno = EINVAL;
        return -1;
    }
    if ((flags & XATTR_CREATE) && open->verifier_present) {
        errno = EEXIST;
        return -1;
    }
    memcpy(open->verifier, value, sizeof(open->verifier));
    open->verifier_present = true;
    if (current->object_present && current->object_ino == open->ino) {
        memcpy(current->verifier, value, sizeof(current->verifier));
        current->verifier_present = true;
    }
    if (current->replace_after_fset) {
        current->replace_after_fset = false;
        current->object_ino++;
        current->verifier_present = false;
    }
    return 0;
}

static int fake_opendir(FsContext *ctx, V9fsPath *path,
                        V9fsFidOpenState *state)
{
    FakeOpen *open;

    note_backend();
    if (strcmp(path->data, "/")) {
        errno = ENOTDIR;
        return -1;
    }
    if (current->fail_replaced_opendir) {
        current->root_ino++;
        current->fail_replaced_opendir = false;
        current->postvalidate_notdir = true;
        errno = ENOTDIR;
        return -1;
    }
    open = g_new0(FakeOpen, 1);
    open->directory = true;
    if (current->replace_on_opendir) {
        current->root_ino++;
        current->replace_on_opendir = false;
    }
    open->ino = current->root_ino;
    state->private = open;
    return 0;
}

static struct dirent *fake_readdir(FsContext *ctx, V9fsFidOpenState *state)
{
    FakeOpen *open = state->private;

    note_backend();
    errno = 0;
    if (open->emitted) {
        return NULL;
    }
    open->emitted = true;
    open->entry.d_ino = UINT64_C(0x100000002);
    g_strlcpy(open->entry.d_name, "kernel", sizeof(open->entry.d_name));
    return &open->entry;
}

static off_t fake_telldir(FsContext *ctx, V9fsFidOpenState *state)
{
    FakeOpen *open = state->private;

    note_backend();
    return open->emitted ? current->backend_cookie : 0;
}

static void fake_seekdir(FsContext *ctx, V9fsFidOpenState *state,
                         off_t offset)
{
    FakeOpen *open = state->private;

    note_backend();
    current->last_seek_cookie = offset;
    open->emitted = current->zero_cookie_continuation || offset != 0;
}

static int fake_statfs(FsContext *ctx, V9fsPath *path, struct statfs *st)
{
    note_backend();
    if (current->fail_replaced_statfs) {
        current->root_ino++;
        current->fail_replaced_statfs = false;
        errno = EIO;
        return -1;
    }
    memset(st, 0, sizeof(*st));
    st->f_bsize = 4096;
    st->f_blocks = 100;
    st->f_bfree = 50;
    st->f_bavail = 40;
    st->f_files = 10;
    st->f_ffree = 8;
    return 0;
}

static int fake_fstat(FsContext *ctx, int fid_type,
                      V9fsFidOpenState *state, struct stat *st)
{
    FakeOpen *open = state->private;

    note_backend();
    memset(st, 0, sizeof(*st));
    st->st_dev = 9;
    st->st_ino = open->ino;
    st->st_mode = open->directory ? S_IFDIR | 0755 : open->mode;
    st->st_nlink = open->directory ? 2 : 1;
    st->st_size = open->directory ? 0 : 6;
    return 0;
}

static FileOperations fake_ops = {
    .init = fake_init,
    .cleanup = fake_cleanup,
    .name_to_path = fake_name_to_path,
    .lstat = fake_lstat,
    .readlink = fake_readlink,
    .open = fake_open,
    .open2 = fake_open2,
    .open_tmpfile = fake_open_tmpfile,
    .close = fake_close,
    .preadv = fake_preadv,
    .pwritev = fake_pwritev,
    .fstat = fake_fstat,
    .opendir = fake_opendir,
    .readdir = fake_readdir,
    .telldir = fake_telldir,
    .seekdir = fake_seekdir,
    .closedir = fake_close,
    .statfs = fake_statfs,
    .fsync = fake_fsync,
    .chmod = fake_chmod,
    .truncate = fake_truncate,
    .utimensat = fake_utimensat,
    .mkdir = fake_mkdir,
    .mknod = fake_mknod,
    .symlink = fake_symlink,
    .unlinkat = fake_unlinkat,
    .renameat = fake_renameat,
    .link = fake_link,
    .setuid = fake_setuid,
    .lgetxattr = fake_lgetxattr,
    .lsetxattr = fake_lsetxattr,
    .fgetxattr = fake_fgetxattr,
    .fsetxattr = fake_fsetxattr,
    .flinkat = fake_flinkat,
};

static int send_reply(Nfs2Service service, const struct sockaddr_in *peer,
                      const uint8_t *data, size_t len, void *opaque)
{
    Fixture *f = opaque;

    g_byte_array_set_size(f->reply, 0);
    g_byte_array_append(f->reply, data, len);
    f->send_count++;
    if (f->reset_on_send) {
        f->reset_on_send = false;
        nfs2_server_reset(f->server);
    }
    return 0;
}

static int64_t fake_clock_ms(void *opaque)
{
    Fixture *f = opaque;

    f->clock_calls++;
    return f->clock_ms;
}

static const Nfs2TransportOps transport = {
    .send = send_reply,
    .clock_ms = fake_clock_ms,
};

static void setup(Fixture *f, gconstpointer opaque)
{
    current = f;
    g_mutex_init(&f->fake_lock);
    f->main_thread = g_thread_self();
    f->reply = g_byte_array_new();
    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)"fake-nfs",
        .path = (char *)"/fake",
        .export_flags = V9FS_SM_MAPPED | V9FS_RDONLY,
        .ops = &fake_ops,
    };
    f->kernel_ino = UINT64_C(0x100000002);
    f->kernel_dev = 9;
    f->root_ino = 1;
    f->link_ino = 3;
    f->backend_cookie = 1;
    f->kernel_mode = S_IFREG | 0555;
    f->kernel_size = 6;
    f->object_ino = 100;
    f->server = nfs2_server_new("fake-nfs", false, &transport, f,
                                &error_abort);
    f->backend_on_main = false;
}

static void teardown(Fixture *f, gconstpointer opaque)
{
    nfs2_server_free(f->server);
    g_byte_array_unref(f->reply);
    g_mutex_clear(&f->fake_lock);
    current = NULL;
}

static size_t rpc_call(uint8_t *buf, size_t capacity, uint32_t program,
                       uint32_t version, uint32_t procedure,
                       const void *body, size_t body_len)
{
    Nfs2XdrWriter w;

    nfs2_xdr_writer_init(&w, buf, capacity);
    g_assert_true(nfs2_xdr_put_u32(&w, 0x12345678));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_RPC_CALL));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_RPC_VERSION));
    g_assert_true(nfs2_xdr_put_u32(&w, program));
    g_assert_true(nfs2_xdr_put_u32(&w, version));
    g_assert_true(nfs2_xdr_put_u32(&w, procedure));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_AUTH_NULL));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_AUTH_NULL));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_opaque(&w, body, body_len));
    return nfs2_xdr_writer_size(&w);
}

static size_t rpc_call_auth_sys(uint8_t *buf, size_t capacity,
                                uint32_t version, uint32_t procedure,
                                const void *body, size_t body_len)
{
    uint8_t credential[128];
    Nfs2XdrWriter cred, w;

    nfs2_xdr_writer_init(&cred, credential, sizeof(credential));
    g_assert_true(nfs2_xdr_put_u32(&cred, 1));
    g_assert_true(nfs2_xdr_put_counted_opaque(&cred, "untrusted", 9, 255));
    g_assert_true(nfs2_xdr_put_u32(&cred, 0));
    g_assert_true(nfs2_xdr_put_u32(&cred, 0));
    g_assert_true(nfs2_xdr_put_u32(&cred, 3));
    g_assert_true(nfs2_xdr_put_u32(&cred, 1));
    g_assert_true(nfs2_xdr_put_u32(&cred, 2));
    g_assert_true(nfs2_xdr_put_u32(&cred, 3));
    nfs2_xdr_writer_init(&w, buf, capacity);
    g_assert_true(nfs2_xdr_put_u32(&w, 0x41555448));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_RPC_CALL));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_RPC_VERSION));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_NFS_PROGRAM));
    g_assert_true(nfs2_xdr_put_u32(&w, version));
    g_assert_true(nfs2_xdr_put_u32(&w, procedure));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_AUTH_SYS));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, credential,
                                               nfs2_xdr_writer_size(&cred),
                                               sizeof(credential)));
    g_assert_true(nfs2_xdr_put_u32(&w, NFS2_AUTH_NULL));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_opaque(&w, body, body_len));
    return nfs2_xdr_writer_size(&w);
}

static void request_peer(Fixture *f, Nfs2Service service, const void *data,
                         size_t len, uint16_t port)
{
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(0x0a00020f),
    };
    unsigned int polls = 0;

    g_byte_array_set_size(f->reply, 0);
    g_assert_cmpint(nfs2_server_receive(f->server, service, &peer, data, len,
                                        &error_abort), ==, 0);
    while (nfs2_server_busy(f->server)) {
        g_assert_cmpuint(polls++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
    g_assert_cmpuint(f->reply->len, >=, 24);
}

static void request(Fixture *f, Nfs2Service service, const void *data,
                    size_t len)
{
    request_peer(f, service, data, len, 900);
}

static uint32_t reply_word(Fixture *f, size_t word)
{
    g_assert_cmpuint((word + 1) * 4, <=, f->reply->len);
    return ldl_be_p(f->reply->data + word * 4);
}

static Nfs2FileHandle mount_root(Fixture *f, uint32_t version)
{
    uint8_t call[128], body[8] = { 0 };
    Nfs2FileHandle handle;
    size_t len;

    stl_be_p(body, 1);
    body[4] = '/';
    len = rpc_call(call, sizeof(call), NFS2_MOUNT_PROGRAM, version,
                   NFS2_MOUNT_MNT, body, sizeof(body));
    request(f, NFS2_SERVICE_MOUNT, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_SUCCESS);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    if (version == 3) {
        g_assert_cmpuint(reply_word(f, 7), ==, NFS2_FHSIZE);
        memcpy(handle.bytes, f->reply->data + 32, sizeof(handle.bytes));
        g_assert_cmpuint(reply_word(f, 16), ==, 1);
        g_assert_cmpuint(reply_word(f, 17), ==, NFS2_AUTH_SYS);
    } else {
        memcpy(handle.bytes, f->reply->data + 28, sizeof(handle.bytes));
    }
    return handle;
}

static Nfs2FileHandle lookup(Fixture *f, uint32_t version,
                             const Nfs2FileHandle *root, const char *name)
{
    uint8_t call[160], body[64];
    Nfs2XdrWriter w;
    Nfs2FileHandle result;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    if (version == 3) {
        g_assert_true(nfs2_xdr_put_counted_opaque(&w, root->bytes,
                                                   sizeof(root->bytes), 32));
    } else {
        g_assert_true(nfs2_xdr_put_opaque(&w, root->bytes,
                                          sizeof(root->bytes)));
    }
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, name, strlen(name), 255));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, version,
                   version == 3 ? 3 : NFS2_NFSPROC_LOOKUP,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    if (version == 3) {
        g_assert_cmpuint(reply_word(f, 7), ==, 32);
        memcpy(result.bytes, f->reply->data + 32, sizeof(result.bytes));
    } else {
        memcpy(result.bytes, f->reply->data + 28, sizeof(result.bytes));
    }
    return result;
}

static void test_portmap_dual(Fixture *f, gconstpointer opaque)
{
    uint8_t call[128], body[16];
    const uint32_t versions[] = { 1, 2, 3, 3 };
    const uint32_t programs[] = {
        NFS2_MOUNT_PROGRAM, NFS2_NFS_PROGRAM,
        NFS2_MOUNT_PROGRAM, NFS2_NFS_PROGRAM,
    };
    const uint32_t ports[] = {
        NFS2_PORT_MOUNT, NFS2_PORT_NFS, NFS2_PORT_MOUNT, NFS2_PORT_NFS,
    };

    for (size_t i = 0; i < G_N_ELEMENTS(versions); i++) {
        stl_be_p(body, programs[i]);
        stl_be_p(body + 4, versions[i]);
        stl_be_p(body + 8, NFS2_IPPROTO_UDP);
        stl_be_p(body + 12, 0);
        size_t len = rpc_call(call, sizeof(call), NFS2_PMAP_PROGRAM,
                              NFS2_PMAP_VERSION, NFS2_PMAP_GETPORT,
                              body, sizeof(body));
        request(f, NFS2_SERVICE_PORTMAP, call, len);
        g_assert_cmpuint(reply_word(f, 6), ==, ports[i]);
    }
}

static void test_v2_bootstrap(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root = mount_root(f, 1);
    Nfs2FileHandle file = lookup(f, 2, &root, "kernel");
    uint8_t call[160], body[44];
    size_t len;

    memcpy(body, file.bytes, sizeof(file.bytes));
    stl_be_p(body + 32, 0);
    stl_be_p(body + 36, 1024);
    stl_be_p(body + 40, 1024);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READ, body, sizeof(body));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 24), ==, 6);
    g_assert_cmpmem(f->reply->data + 100, 6, "NetBSD", 6);
}

static void test_v3_kernel_flow(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root = mount_root(f, 3);
    Nfs2FileHandle file = lookup(f, 3, &root, "kernel");
    uint8_t call[192], body[64];
    Nfs2XdrWriter w;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, NFS2_NFDIR);

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 19,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 29), ==, NFS2_MAX_DATA);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0x3f));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 4,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 29) & 0x1c, ==, 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_opaque(&w, "\0\0\0\0\0\0\0\0", 8));
    g_assert_true(nfs2_xdr_put_u32(&w, 1024));
    g_assert_true(nfs2_xdr_put_u32(&w, 4096));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 17,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);
    g_assert_false(f->backend_on_main);
}

static void test_read_metadata_surfaces(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root2 = mount_root(f, 1);
    Nfs2FileHandle root3 = mount_root(f, 3);
    Nfs2FileHandle link;
    uint8_t call[192], body[80];
    Nfs2XdrWriter w;
    size_t len;

    memcpy(body, root2.bytes, 32);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, body, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, NFS2_NFDIR);

    memset(body + 32, 0, 4);
    stl_be_p(body + 36, 1024);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READDIR, body, 40);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_STATFS, body, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 7), ==, NFS2_MAX_DATA);

    for (uint32_t proc = NFS2_NFSPROC_ROOT;
         proc <= NFS2_NFSPROC_WRITECACHE;
         proc += NFS2_NFSPROC_WRITECACHE - NFS2_NFSPROC_ROOT) {
        len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                       proc, NULL, 0);
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_SUCCESS);
    }

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root3.bytes, 32, 32));
    for (uint32_t proc = 18; proc <= 20; proc += 2) {
        len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, proc,
                       body, nfs2_xdr_writer_size(&w));
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 6), ==, 0);
    }

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root3.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_opaque(&w, "\0\0\0\0\0\0\0\0", 8));
    g_assert_true(nfs2_xdr_put_u32(&w, 1024));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 16,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);

    /* LOOKUP above yielded /kernel; resolve /link separately. */
    link = (Nfs2FileHandle) { 0 };
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root3.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "link", 4, 255));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 3,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    memcpy(link.bytes, f->reply->data + 32, 32);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, link.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 5,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 29), ==, 6);
}

static void test_bounds_and_errors(Fixture *f, gconstpointer opaque)
{
    uint8_t call[128], body[16] = { 0 };
    g_autofree uint8_t *too_big = g_malloc0(NFS2_MAX_RPC_DATAGRAM + 1);
    Error *err = NULL;
    size_t len;

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 4, 0, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_PROG_MISMATCH);

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 99, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_PROC_UNAVAIL);

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_GARBAGE_ARGS);

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 21, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 30);
    g_assert_cmpuint(f->reply->len, ==, 36);

    stl_be_p(body, 4);
    memcpy(body + 4, "/bad", 4);
    len = rpc_call(call, sizeof(call), NFS2_MOUNT_PROGRAM, 3,
                   NFS2_MOUNT_MNT, body, 8);
    request(f, NFS2_SERVICE_MOUNT, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_ACCES);

    g_assert_cmpint(nfs2_server_receive(f->server, NFS2_SERVICE_NFS,
                                        &(struct sockaddr_in) {
                                            .sin_family = AF_INET,
                                        }, too_big,
                                        NFS2_MAX_RPC_DATAGRAM + 1, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_stale_identity_and_access(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root = mount_root(f, 3);
    Nfs2FileHandle file = lookup(f, 3, &root, "kernel");
    uint8_t call[160], body[64];
    Nfs2XdrWriter w;
    size_t len;

    /* Replacing an object at an authenticated path must stale the old fh. */
    f->kernel_ino++;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);
    f->kernel_ino--;

    /* Task 8 is read-only even if a later writable export is configured. */
    nfs2_server_free(f->server);
    f->fse.export_flags = V9FS_SM_MAPPED;
    f->server = nfs2_server_new("fake-nfs", true, &transport, f,
                                &error_abort);
    root = mount_root(f, 3);
    file = lookup(f, 3, &root, "kernel");
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0x3f));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 4,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 29) & 0x1c, ==, 0);
}

static void readdir3(Fixture *f, const Nfs2FileHandle *dir, bool plus,
                     uint64_t cookie, const uint8_t verifier[8],
                     uint32_t dircount, uint32_t maxcount)
{
    uint8_t call[192], body[80];
    Nfs2XdrWriter w;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, dir->bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, cookie >> 32));
    g_assert_true(nfs2_xdr_put_u32(&w, cookie));
    g_assert_true(nfs2_xdr_put_opaque(&w, verifier, 8));
    if (plus) {
        g_assert_true(nfs2_xdr_put_u32(&w, dircount));
    }
    g_assert_true(nfs2_xdr_put_u32(&w, maxcount));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3,
                   plus ? 17 : 16, body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
}

static void test_readdir3_bounds(Fixture *f, gconstpointer opaque)
{
    static const uint8_t zero[8];
    Nfs2FileHandle root = mount_root(f, 3);
    Nfs2FileHandle forged = root;
    uint8_t verifier[8], wrong[8];

    readdir3(f, &root, true, 0, zero, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    memcpy(verifier, f->reply->data + 29 * 4, sizeof(verifier));
    g_assert_cmpint(memcmp(verifier, zero, sizeof(verifier)), !=, 0);

    readdir3(f, &root, true, 1, verifier, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);

    memcpy(wrong, verifier, sizeof(wrong));
    wrong[0] ^= 1;
    readdir3(f, &root, true, 1, wrong, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 10003);

    readdir3(f, &root, true, 0, zero, 1, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 10005);
    readdir3(f, &root, true, 0, zero, 31, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 10005);
    readdir3(f, &root, true, 0, zero, 32, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpmem(f->reply->data + 29 * 4, 8, verifier, 8);
    readdir3(f, &root, true, 0, zero, 1024, 263);
    g_assert_cmpuint(reply_word(f, 6), ==, 10005);
    readdir3(f, &root, true, 0, zero, 1024, 264);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    readdir3(f, &root, true, 0, zero, 1024, 1);
    g_assert_cmpuint(reply_word(f, 6), ==, 10005);

    forged.bytes[0] ^= 1;
    readdir3(f, &forged, false, 0, zero, 0, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);
    g_assert_cmpuint(reply_word(f, 7), ==, 0);
    readdir3(f, &forged, true, 0, zero, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);
    g_assert_cmpuint(reply_word(f, 7), ==, 0);
}

static void test_protocol_vectors(Fixture *f, gconstpointer opaque)
{
    uint8_t call[192], body[64] = { 0 };
    Nfs2FileHandle root, file, link;
    size_t len;

    len = rpc_call(call, sizeof(call), NFS2_PMAP_PROGRAM, NFS2_PMAP_VERSION,
                   NFS2_PMAP_NULL, NULL, 0);
    request(f, NFS2_SERVICE_PORTMAP, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_SUCCESS);
    stl_be_p(body, NFS2_NFS_PROGRAM); stl_be_p(body + 4, 4);
    stl_be_p(body + 8, NFS2_IPPROTO_UDP);
    len = rpc_call(call, sizeof(call), NFS2_PMAP_PROGRAM, NFS2_PMAP_VERSION,
                   NFS2_PMAP_GETPORT, body, 16);
    request(f, NFS2_SERVICE_PORTMAP, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2, 0, NULL, 0);
    request(f, NFS2_SERVICE_PORTMAP, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_PROG_UNAVAIL);

    stl_be_p(body, 4); memcpy(body + 4, "/bad", 4);
    len = rpc_call(call, sizeof(call), NFS2_MOUNT_PROGRAM, 1,
                   NFS2_MOUNT_MNT, body, 8);
    request(f, NFS2_SERVICE_MOUNT, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_ACCES);

    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_NULL, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_SUCCESS);
    root = mount_root(f, 1);
    link = lookup(f, 2, &root, "link");
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READLINK, link.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, 6);

    file = lookup(f, 2, &root, "kernel");
    memcpy(body, file.bytes, 32); stl_be_p(body + 32, 99);
    stl_be_p(body + 36, 8192); stl_be_p(body + 40, 8192);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READ, body, 44);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 24), ==, 0);
    stl_be_p(body + 36, 8193);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READ, body, 44);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_GARBAGE_ARGS);

    f->lstat_error = EIO;
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, file.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_IO);
    f->lstat_error = 0;
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, file.bytes, 4);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_GARBAGE_ARGS);

}

static void test_open_identity_race(Fixture *f, gconstpointer opaque)
{
    static const uint8_t zero[8];
    Nfs2FileHandle root = mount_root(f, 3);
    Nfs2FileHandle file = lookup(f, 3, &root, "kernel");
    uint8_t call[192], body[64];
    Nfs2XdrWriter w;
    size_t len;

    f->replace_on_open = true;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 8));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 6,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    f->kernel_ino--;
    f->fail_replaced_open = true;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    root = mount_root(f, 3);
    f->replace_on_opendir = true;
    readdir3(f, &root, true, 0, zero, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    f->root_ino--;
    f->fail_replaced_opendir = true;
    readdir3(f, &root, true, 0, zero, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);
}

static void test_path_identity_race(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root = mount_root(f, 3);
    Nfs2FileHandle link = lookup(f, 3, &root, "link");
    uint8_t call[192], body[80];
    Nfs2XdrWriter w;
    size_t len;

    /*
     * The private export excludes hostile exact-ABA swaps; ordinary swaps
     * during path-only backend operations must still fail closed.
     */
    f->replace_on_readlink = true;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, link.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 5,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    f->link_ino--;
    f->fail_replaced_readlink = true;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 18,
                   body, nfs2_xdr_writer_size(&w));
    f->fail_replaced_statfs = true;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);
    f->root_ino--;

    f->replace_on_lookup = true;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "kernel", 6, 255));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 3,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    f->root_ino--;
    f->remove_child_on_lookup = true;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    f->root_ino--;
    f->replace_on_lookup = true;
    readdir3(f, &root, true, 0, (const uint8_t[8]) { 0 }, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);

    f->root_ino--;
    f->remove_child_on_lookup = true;
    readdir3(f, &root, true, 0, (const uint8_t[8]) { 0 }, 1024, 4096);
    g_assert_cmpuint(reply_word(f, 6), ==, 70);
    g_assert_cmpuint(reply_word(f, 7), ==, 0);
}

static void test_v2_component_validation(Fixture *f, gconstpointer opaque)
{
    static const char *const invalid[] = { "", ".", "..", "bad/name" };
    Nfs2FileHandle root = mount_root(f, 1);
    uint8_t call[512], body[320];

    for (size_t i = 0; i < G_N_ELEMENTS(invalid); i++) {
        Nfs2XdrWriter w;
        unsigned int calls = f->name_to_path_calls;

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        g_assert_true(nfs2_xdr_put_opaque(&w, root.bytes, 32));
        g_assert_true(nfs2_xdr_put_counted_opaque(&w, invalid[i],
                                                   strlen(invalid[i]), 256));
        size_t len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                              NFS2_NFSPROC_LOOKUP, body,
                              nfs2_xdr_writer_size(&w));
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_GARBAGE_ARGS);
        g_assert_cmpuint(f->name_to_path_calls, ==, calls);
    }
    memset(body + 36, 'x', 256);
    stl_be_p(body + 32, 256);
    {
        unsigned int calls = f->name_to_path_calls;
        size_t len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                              NFS2_NFSPROC_LOOKUP, body, 32 + 4 + 256);

        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_GARBAGE_ARGS);
        g_assert_cmpuint(f->name_to_path_calls, ==, calls);
    }
}

static void test_identity_and_v2_mappings(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root2 = mount_root(f, 1);
    Nfs2FileHandle kernel = lookup(f, 2, &root2, "kernel");
    Nfs2FileHandle collision = lookup(f, 2, &root2, "collision");
    Nfs2FileHandle hardlink, replacement;
    uint8_t call[192], body[64];
    uint32_t kernel_fileid, collision_fileid, wire_cookie;
    size_t len;

    g_assert_cmpint(memcmp(kernel.bytes, collision.bytes,
                           sizeof(kernel.bytes)), !=, 0);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, kernel.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    kernel_fileid = reply_word(f, 17);
    g_assert_cmpuint(reply_word(f, 8), ==, S_IFREG | 0555);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, collision.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    collision_fileid = reply_word(f, 17);
    g_assert_cmpuint(kernel_fileid, !=, collision_fileid);
    g_assert_cmpuint(kernel_fileid, !=, UINT32_MAX);

    f->kernel_resolution_notdir = true;
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, kernel.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_STALE);
    f->kernel_resolution_notdir = false;

    f->kernel_dev = 8;
    f->kernel_ino = 2;
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, kernel.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_STALE);

    f->kernel_dev = 9;
    f->kernel_ino = UINT64_C(0x100000002);
    hardlink = lookup(f, 2, &root2, "hardlink");
    g_assert_cmpmem(kernel.bytes, sizeof(kernel.bytes), hardlink.bytes,
                    sizeof(hardlink.bytes));
    f->kernel_resolution_notdir = true;
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, kernel.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    f->kernel_resolution_notdir = false;
    f->kernel_dev = 8;
    f->kernel_ino = 2;
    replacement = lookup(f, 2, &root2, "kernel");
    g_assert_cmpmem(collision.bytes, sizeof(collision.bytes),
                    replacement.bytes, sizeof(replacement.bytes));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, kernel.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);

    f->kernel_dev = 9;
    f->kernel_ino = UINT64_C(0x100000002);
    f->backend_cookie = UINT64_C(0x100000077);
    memcpy(body, root2.bytes, 32);
    stl_be_p(body + 32, 0);
    stl_be_p(body + 36, 32);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READDIR, body, 40);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 8), ==, kernel_fileid);
    wire_cookie = reply_word(f, 12);
    g_assert_cmpuint(wire_cookie, !=, UINT32_MAX);
    stl_be_p(body + 32, wire_cookie);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READDIR, body, 40);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(f->last_seek_cookie, ==, f->backend_cookie);

    f->backend_cookie = 0;
    f->zero_cookie_continuation = true;
    stl_be_p(body + 32, 0);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READDIR, body, 40);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    wire_cookie = reply_word(f, 12);
    g_assert_cmpuint(wire_cookie, !=, 0);
    f->last_seek_cookie = UINT64_MAX;
    stl_be_p(body + 32, wire_cookie);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READDIR, body, 40);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(f->last_seek_cookie, ==, 0);

    stl_be_p(body + 32, 0);
    stl_be_p(body + 36, 4);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READDIR, body, 40);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_GARBAGE_ARGS);

    f->replace_child_after_lstat = true;
    memcpy(body, root2.bytes, 32);
    stl_be_p(body + 32, 6);
    memcpy(body + 36, "kernel", 6);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_LOOKUP, body, 44);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_STALE);

    f->replace_parent_on_second_child_lstat = true;
    f->child_lstat_calls = 0;
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_LOOKUP, body, 44);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_STALE);
}

static void test_v3_wire_semantics(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root = mount_root(f, 3);
    Nfs2FileHandle file = lookup(f, 3, &root, "kernel");
    uint8_t call[192], body[64];
    Nfs2XdrWriter w;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 8), ==, 0555);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 8));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 6,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 22);
}

static void make_writable(Fixture *f)
{
    nfs2_server_free(f->server);
    f->fse.export_flags = V9FS_SM_MAPPED;
    f->server = nfs2_server_new("fake-nfs", true, &transport, f,
                                &error_abort);
}

static void test_mutations(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root, file;
    uint8_t call[256], body[160];
    Nfs2XdrWriter w;
    uint8_t verifier[8];
    size_t len;

    /* Read-only rejection precedes even a missing procedure body. */
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_SETATTR, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_ROFS);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3,
                   7, NULL, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 30);
    g_assert_cmpuint(f->mutation_calls, ==, 0);

    make_writable(f);
    root = mount_root(f, 3);
    file = lookup(f, 3, &root, "kernel");

    /* A literal NFSv2 SETATTR: mode, ignored uid/gid, size, atime, mtime. */
    memcpy(body, file.bytes, 32);
    stl_be_p(body + 32, 0640);
    stl_be_p(body + 36, 12345);
    stl_be_p(body + 40, 23456);
    stl_be_p(body + 44, 9);
    stl_be_p(body + 48, 10);
    stl_be_p(body + 52, 11);
    stl_be_p(body + 56, 12);
    stl_be_p(body + 60, 13);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_SETATTR, body, 64);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFS_OK);
    g_assert_cmpuint(f->kernel_mode & 07777, ==, 0640);
    g_assert_cmpint(f->kernel_size, ==, 9);

    /* A literal NFSv3 FILE_SYNC WRITE at a 64-bit offset. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 16));
    g_assert_true(nfs2_xdr_put_u32(&w, 4));
    g_assert_true(nfs2_xdr_put_u32(&w, 2)); /* FILE_SYNC */
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "data", 4,
                                               NFS2_MAX_DATA));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 7,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);  /* pre-op WCC */
    g_assert_cmpuint(reply_word(f, 14), ==, 1); /* post-op attrs */
    g_assert_cmpuint(reply_word(f, 36), ==, 4);
    g_assert_cmpuint(reply_word(f, 37), ==, 2); /* FILE_SYNC */
    memcpy(verifier, f->reply->data + f->reply->len - 8, 8);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 21,
                   body, nfs2_xdr_writer_size(&w));
    stl_be_p(call, 0x434f4d4d);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpmem(f->reply->data + f->reply->len - 8, 8,
                    verifier, sizeof(verifier));
    g_assert_cmpuint(f->setuid_calls, ==, 0);
}

static void put_name2(Nfs2XdrWriter *w, const Nfs2FileHandle *dir,
                      const char *name)
{
    g_assert_true(nfs2_xdr_put_opaque(w, dir->bytes, 32));
    g_assert_true(nfs2_xdr_put_counted_opaque(w, name, strlen(name), 255));
}

static void put_name3(Nfs2XdrWriter *w, const Nfs2FileHandle *dir,
                      const char *name)
{
    g_assert_true(nfs2_xdr_put_counted_opaque(w, dir->bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_counted_opaque(w, name, strlen(name), 255));
}

static void put_sattr2(Nfs2XdrWriter *w, mode_t mode)
{
    g_assert_true(nfs2_xdr_put_u32(w, mode));
    for (unsigned int i = 0; i < 7; i++) {
        g_assert_true(nfs2_xdr_put_u32(w, UINT32_MAX));
    }
}

static void put_sattr3(Nfs2XdrWriter *w, mode_t mode)
{
    g_assert_true(nfs2_xdr_put_u32(w, 1));
    g_assert_true(nfs2_xdr_put_u32(w, mode));
    g_assert_true(nfs2_xdr_put_u32(w, 0)); /* uid */
    g_assert_true(nfs2_xdr_put_u32(w, 0)); /* gid */
    g_assert_true(nfs2_xdr_put_u32(w, 0)); /* size */
    g_assert_true(nfs2_xdr_put_u32(w, 0)); /* atime */
    g_assert_true(nfs2_xdr_put_u32(w, 0)); /* mtime */
}

static void send_nfs(Fixture *f, uint32_t version, uint32_t proc,
                     const uint8_t *body, size_t body_len, uint32_t expected)
{
    uint8_t call[512];
    g_autoptr(GByteArray) first_reply = g_byte_array_new();
    unsigned int calls;
    size_t len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, version,
                          proc, body, body_len);

    stl_be_p(call, ++f->mutation_xid);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, expected);
    g_byte_array_append(first_reply, f->reply->data, f->reply->len);
    calls = f->mutation_calls;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_cmpmem(f->reply->data, f->reply->len,
                    first_reply->data, first_reply->len);
}

static void test_v2_mutation_procedures(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root, object;
    uint8_t body[384];
    Nfs2XdrWriter w;

    make_writable(f);
    root = mount_root(f, 1);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "v2file");
    put_sattr2(&w, 0600);
    send_nfs(f, 2, NFS2_NFSPROC_CREATE, body,
             nfs2_xdr_writer_size(&w), 0);
    memcpy(object.bytes, f->reply->data + 28, 32);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, object.bytes, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 3));
    g_assert_true(nfs2_xdr_put_u32(&w, 3));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "v2!", 3, 8192));
    send_nfs(f, 2, NFS2_NFSPROC_WRITE, body,
             nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, object.bytes, 32));
    put_name2(&w, &root, "v2link");
    send_nfs(f, 2, NFS2_NFSPROC_LINK, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "v2link");
    send_nfs(f, 2, NFS2_NFSPROC_REMOVE, body,
             nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "v2sym");
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "kernel", 6, 1024));
    put_sattr2(&w, 0777);
    send_nfs(f, 2, NFS2_NFSPROC_SYMLINK, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "v2sym");
    send_nfs(f, 2, NFS2_NFSPROC_REMOVE, body,
             nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "v2dir");
    put_sattr2(&w, 0750);
    send_nfs(f, 2, NFS2_NFSPROC_MKDIR, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "v2dir");
    send_nfs(f, 2, NFS2_NFSPROC_RMDIR, body,
             nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "old");
    put_sattr2(&w, 0600);
    send_nfs(f, 2, NFS2_NFSPROC_CREATE, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "old");
    put_name2(&w, &root, "new");
    send_nfs(f, 2, NFS2_NFSPROC_RENAME, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name2(&w, &root, "new");
    send_nfs(f, 2, NFS2_NFSPROC_REMOVE, body,
             nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpuint(f->setuid_calls, ==, 0);
}

static void test_v3_mutation_procedures(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root, kernel;
    uint8_t body[384];
    Nfs2XdrWriter w;

    make_writable(f);
    root = mount_root(f, 3);
    kernel = lookup(f, 3, &root, "kernel");

    /* UNCHECKED, GUARDED, and EXCLUSIVE create modes. */
    for (uint32_t mode = 0; mode < 3; mode++) {
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name3(&w, &root, mode == 0 ? "unchecked" :
                             mode == 1 ? "guarded" : "exclusive");
        g_assert_true(nfs2_xdr_put_u32(&w, mode));
        if (mode == 2) {
            g_assert_true(nfs2_xdr_put_opaque(&w, "verifier", 8));
        } else {
            put_sattr3(&w, 0600);
        }
        send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
        if (mode == 1) {
            send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 17);
        }
        if (mode == 2) {
            send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
            memcpy(body + nfs2_xdr_writer_size(&w) - 8, "different", 8);
            send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 17);
        }
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name3(&w, &root, mode == 0 ? "unchecked" :
                             mode == 1 ? "guarded" : "exclusive");
        send_nfs(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
    }

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "v3dir");
    put_sattr3(&w, 0750);
    send_nfs(f, 3, 9, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "v3dir");
    send_nfs(f, 3, 13, body, nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "v3sym");
    put_sattr3(&w, 0777);
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "kernel", 6, 1024));
    send_nfs(f, 3, 10, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "v3sym");
    send_nfs(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);

    for (uint32_t type = 3; type <= 7; type++) {
        if (type == 5) {
            continue;
        }
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name3(&w, &root, "special");
        g_assert_true(nfs2_xdr_put_u32(&w, type));
        put_sattr3(&w, 0600);
        if (type == 3 || type == 4) {
            g_assert_true(nfs2_xdr_put_u32(&w, 1));
            g_assert_true(nfs2_xdr_put_u32(&w, 2));
        }
        send_nfs(f, 3, 11, body, nfs2_xdr_writer_size(&w), 0);
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name3(&w, &root, "special");
        send_nfs(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
    }
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "badtype");
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    send_nfs(f, 3, 11, body, nfs2_xdr_writer_size(&w), 10007);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    put_name3(&w, &root, "v3link");
    send_nfs(f, 3, 15, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "v3link");
    send_nfs(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "old3");
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    put_sattr3(&w, 0600);
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "old3");
    put_name3(&w, &root, "new3");
    send_nfs(f, 3, 14, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "new3");
    send_nfs(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    send_nfs(f, 3, 21, body, nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpuint(f->setuid_calls, ==, 0);
}

static size_t commit_call(uint8_t *call, size_t capacity,
                          const Nfs2FileHandle *file, uint32_t xid,
                          uint32_t count)
{
    uint8_t body[64];
    Nfs2XdrWriter w;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file->bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, count));
    len = rpc_call(call, capacity, NFS2_NFS_PROGRAM, 3, 21,
                   body, nfs2_xdr_writer_size(&w));
    stl_be_p(call, xid);
    return len;
}

static void receive_async(Fixture *f, const uint8_t *call, size_t len,
                          uint16_t port)
{
    struct sockaddr_in peer = {
        .sin_family = AF_INET, .sin_port = htons(port),
        .sin_addr.s_addr = htonl(0x0a00020f),
    };

    g_assert_cmpint(nfs2_server_receive(f->server, NFS2_SERVICE_NFS,
                                        &peer, call, len, &error_abort), ==,
                    0);
}

static void drain(Fixture *f)
{
    unsigned int polls = 0;

    while (nfs2_server_busy(f->server)) {
        g_assert_cmpuint(polls++, <, 100000);
        aio_poll(qemu_get_aio_context(), true);
    }
}

static void test_duplicate_cache(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root, kernel;
    uint8_t call[160], changed[160], first[160];
    g_autoptr(GByteArray) first_reply = g_byte_array_new();
    size_t len;
    unsigned int calls, sends;

    make_writable(f);
    root = mount_root(f, 3);
    kernel = lookup(f, 3, &root, "kernel");
    len = commit_call(call, sizeof(call), &kernel, 0xabc, 0);

    request_peer(f, NFS2_SERVICE_NFS, call, len, 900);
    calls = f->mutation_calls;
    g_byte_array_append(first_reply, f->reply->data, f->reply->len);
    request_peer(f, NFS2_SERVICE_NFS, call, len, 900);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_cmpmem(f->reply->data, f->reply->len,
                    first_reply->data, first_reply->len);

    memcpy(changed, call, len);
    stl_be_p(changed + len - 4, 1);
    request_peer(f, NFS2_SERVICE_NFS, changed, len, 900);
    g_assert_cmpuint(f->mutation_calls, ==, ++calls);
    request_peer(f, NFS2_SERVICE_NFS, call, len, 901);
    g_assert_cmpuint(f->mutation_calls, ==, ++calls);

    f->clock_ms = 60000;
    request_peer(f, NFS2_SERVICE_NFS, call, len, 900);
    g_assert_cmpuint(f->mutation_calls, ==, ++calls);

    len = commit_call(call, sizeof(call), &kernel, 0xabd, 0);
    f->mutation_error = EIO;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 5);
    calls = f->mutation_calls;
    g_byte_array_set_size(first_reply, 0);
    g_byte_array_append(first_reply, f->reply->data, f->reply->len);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_cmpmem(f->reply->data, f->reply->len,
                    first_reply->data, first_reply->len);
    f->mutation_error = 0;

    nfs2_server_reset(f->server);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(f->mutation_calls, ==, calls + 1);

    /* Completed-only LRU: the 257th key evicts the oldest completed key. */
    nfs2_server_reset(f->server);
    calls = f->mutation_calls;
    for (uint32_t xid = 1; xid <= 256; xid++) {
        len = commit_call(call, sizeof(call), &kernel, xid, 0);
        if (xid == 1) {
            memcpy(first, call, len);
        }
        request(f, NFS2_SERVICE_NFS, call, len);
    }
    len = commit_call(call, sizeof(call), &kernel, 257, 0);
    request(f, NFS2_SERVICE_NFS, call, len);
    request(f, NFS2_SERVICE_NFS, first, len);
    g_assert_cmpuint(f->mutation_calls, ==, calls + 258);

    /* All entries in flight: refuse the 257th without executing it. */
    nfs2_server_reset(f->server);
    calls = f->mutation_calls;
    sends = f->send_count;
    for (uint32_t xid = 1; xid <= 256; xid++) {
        len = commit_call(call, sizeof(call), &kernel, xid, 0);
        if (xid == 1) {
            memcpy(first, call, len);
        }
        receive_async(f, call, len, 900);
    }
    len = commit_call(call, sizeof(call), &kernel, 257, 0);
    receive_async(f, call, len, 900);
    g_assert_cmpuint(f->send_count, ==, sends + 1);
    g_assert_cmpuint(reply_word(f, 5), ==, NFS2_RPC_SYSTEM_ERR);
    receive_async(f, first, len, 900);
    g_assert_cmpuint(f->send_count, ==, sends + 1);
    drain(f);
    g_assert_cmpuint(f->mutation_calls, ==, calls + 256);
}

static void test_auth_sys_and_readonly_fsdev(Fixture *f,
                                             gconstpointer opaque)
{
    Nfs2Server *rejected;
    Error *error = NULL;
    Nfs2FileHandle root, kernel;
    uint8_t call[256], body[64];
    Nfs2XdrWriter w;
    uid_t uid = getuid();
    gid_t gid = getgid();
    size_t len;

    rejected = nfs2_server_new("fake-nfs", true, &transport, f, &error);
    g_assert_null(rejected);
    g_assert_nonnull(error);
    error_free(error);

    make_writable(f);
    root = mount_root(f, 3);
    kernel = lookup(f, 3, &root, "kernel");
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    len = rpc_call_auth_sys(call, sizeof(call), 3, 21, body,
                            nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
    g_assert_cmpuint(getuid(), ==, uid);
    g_assert_cmpuint(getgid(), ==, gid);
    g_assert_cmpuint(f->setuid_calls, ==, 0);
}

static void test_mutation_lifetime(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root, kernel;
    uint8_t call[160];
    size_t len;
    unsigned int sends;

    make_writable(f);
    root = mount_root(f, 3);
    kernel = lookup(f, 3, &root, "kernel");
    sends = f->send_count;
    len = commit_call(call, sizeof(call), &kernel, 0x1001, 0);
    receive_async(f, call, len, 900);
    len = commit_call(call, sizeof(call), &kernel, 0x1002, 0);
    receive_async(f, call, len, 900);
    g_assert_true(nfs2_server_busy(f->server));
    nfs2_server_reset(f->server);
    nfs2_server_begin_close(f->server);
    drain(f);
    g_assert_cmpuint(f->send_count, ==, sends);
    nfs2_server_free(f->server);
    f->server = NULL;
}

static void test_review_commit_vector(Fixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root3, kernel3;
    uint8_t call[256], body[128];
    Nfs2XdrWriter w;
    size_t len;

    make_writable(f);
    root3 = mount_root(f, 3);
    kernel3 = lookup(f, 3, &root3, "kernel");

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel3.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 21,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, 0);
}

static void test_review_v2_write_totalcount(Fixture *f,
                                            gconstpointer opaque)
{
    Nfs2FileHandle root2, kernel2;
    uint8_t call[256], body[128];
    Nfs2XdrWriter w;
    size_t len;

    make_writable(f);
    root2 = mount_root(f, 1);
    kernel2 = lookup(f, 2, &root2, "kernel");
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, kernel2.bytes, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "x", 1, 8192));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_WRITE, body, nfs2_xdr_writer_size(&w));
    stl_be_p(call, 0x56454354);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFS_OK);
}

static void test_review_unregistered_remove_inode_zero(Fixture *f,
                                                       gconstpointer opaque)
{
    Nfs2FileHandle root;
    uint8_t body[80];
    Nfs2XdrWriter w;

    nfs2_server_free(f->server);
    f->server = NULL;
    f->root_ino = 0;
    f->object_present = true;
    f->object_ino = 333;
    f->object_mode = S_IFREG | 0600;
    g_strlcpy(f->object_name, "preexisting", sizeof(f->object_name));
    f->fse.export_flags = V9FS_SM_MAPPED;
    f->server = nfs2_server_new("fake-nfs", true, &transport, f,
                                &error_abort);
    root = mount_root(f, 3);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "preexisting");
    send_nfs(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);
}

static void test_review_readonly_mutation_cache(Fixture *f,
                                                gconstpointer opaque)
{
    static const uint32_t v2_procs[] = {
        NFS2_NFSPROC_SETATTR, NFS2_NFSPROC_WRITE, NFS2_NFSPROC_CREATE,
        NFS2_NFSPROC_REMOVE, NFS2_NFSPROC_RENAME, NFS2_NFSPROC_LINK,
        NFS2_NFSPROC_SYMLINK, NFS2_NFSPROC_MKDIR, NFS2_NFSPROC_RMDIR,
    };
    static const uint32_t v3_procs[] = {
        2, 7, 8, 9, 10, 11, 12, 13, 14, 15, 21,
    };
    uint8_t call[96];
    g_autoptr(GByteArray) first = g_byte_array_new();
    unsigned int expected_clock_calls = f->clock_calls;

    for (size_t i = 0; i < G_N_ELEMENTS(v2_procs); i++) {
        size_t len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                              v2_procs[i], NULL, 0);

        stl_be_p(call, 0x2000 + i);
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 6), ==, NFS2_NFSERR_ROFS);
        g_byte_array_set_size(first, 0);
        g_byte_array_append(first, f->reply->data, f->reply->len);
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpmem(f->reply->data, f->reply->len,
                        first->data, first->len);
        expected_clock_calls += 3;
        g_assert_cmpuint(f->clock_calls, ==, expected_clock_calls);
    }
    for (size_t i = 0; i < G_N_ELEMENTS(v3_procs); i++) {
        size_t len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3,
                              v3_procs[i], NULL, 0);

        stl_be_p(call, 0x3000 + i);
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 6), ==, 30);
        g_byte_array_set_size(first, 0);
        g_byte_array_append(first, f->reply->data, f->reply->len);
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpmem(f->reply->data, f->reply->len,
                        first->data, first->len);
        expected_clock_calls += 3;
        g_assert_cmpuint(f->clock_calls, ==, expected_clock_calls);
    }
}

static void test_review_reset_suppresses_and_reentrant(Fixture *f,
                                                       gconstpointer opaque)
{
    Nfs2FileHandle root, kernel;
    uint8_t call[160];
    unsigned int sends;
    size_t len;

    make_writable(f);
    root = mount_root(f, 3);
    kernel = lookup(f, 3, &root, "kernel");
    len = commit_call(call, sizeof(call), &kernel, 0x52535431, 0);
    sends = f->send_count;
    receive_async(f, call, len, 900);
    nfs2_server_reset(f->server);
    drain(f);
    g_assert_cmpuint(f->send_count, ==, sends);

    stl_be_p(call, 0x52535432);
    f->reset_on_send = true;
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_false(f->reset_on_send);
    g_assert_false(nfs2_server_busy(f->server));
}

static void test_review_concurrent_rename_remove(Fixture *f,
                                                 gconstpointer opaque)
{
    Nfs2FileHandle root;
    uint8_t create_body[128], rename_body[160], remove_body[96];
    uint8_t rename_call[256], remove_call[192];
    Nfs2XdrWriter w;
    size_t rename_len, remove_len;

    make_writable(f);
    root = mount_root(f, 3);

    nfs2_xdr_writer_init(&w, create_body, sizeof(create_body));
    put_name3(&w, &root, "race-old");
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    put_sattr3(&w, 0600);
    send_nfs(f, 3, 8, create_body, nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, rename_body, sizeof(rename_body));
    put_name3(&w, &root, "race-old");
    put_name3(&w, &root, "race-new");
    rename_len = rpc_call(rename_call, sizeof(rename_call), NFS2_NFS_PROGRAM,
                          3, 14, rename_body, nfs2_xdr_writer_size(&w));
    stl_be_p(rename_call, 0x52414331);
    nfs2_xdr_writer_init(&w, remove_body, sizeof(remove_body));
    put_name3(&w, &root, "race-old");
    remove_len = rpc_call(remove_call, sizeof(remove_call), NFS2_NFS_PROGRAM,
                          3, 12, remove_body, nfs2_xdr_writer_size(&w));
    stl_be_p(remove_call, 0x52414332);
    f->race_delay = true;
    receive_async(f, rename_call, rename_len, 900);
    receive_async(f, remove_call, remove_len, 900);
    drain(f);
    g_assert_cmpint(g_atomic_int_get(&f->race_max), ==, 1);
    g_assert_true(f->object_present);
    g_assert_cmpstr(f->object_name, ==, "race-new");

    nfs2_xdr_writer_init(&w, remove_body, sizeof(remove_body));
    put_name3(&w, &root, "race-new");
    send_nfs(f, 3, 12, remove_body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, create_body, sizeof(create_body));
    put_name3(&w, &root, "race-old");
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    put_sattr3(&w, 0600);
    send_nfs(f, 3, 8, create_body, nfs2_xdr_writer_size(&w), 0);
    g_atomic_int_set(&f->race_max, 0);
    stl_be_p(remove_call, 0x52414333);
    stl_be_p(rename_call, 0x52414334);
    receive_async(f, remove_call, remove_len, 900);
    receive_async(f, rename_call, rename_len, 900);
    drain(f);
    g_assert_cmpint(g_atomic_int_get(&f->race_max), ==, 1);
    g_assert_false(f->object_present);
}

static void test_review_exclusive_xattr_failures(Fixture *f,
                                                 gconstpointer opaque)
{
    Nfs2FileHandle root;
    uint8_t body[128];
    Nfs2XdrWriter w;
    unsigned int calls;

    fake_ops.fgetxattr = NULL;
    make_writable(f);
    root = mount_root(f, 3);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "unsupported");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "verify01", 8));
    calls = f->mutation_calls;
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 10004);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_false(f->object_present);
    fake_ops.fgetxattr = fake_fgetxattr;

    fake_ops.fsetxattr = NULL;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "unsupported-set");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "verify00", 8));
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 10004);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_false(f->object_present);
    fake_ops.fsetxattr = fake_fsetxattr;

    fake_ops.flinkat = NULL;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "unsupported-link");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "verify09", 8));
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 10004);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_false(f->object_present);
    fake_ops.flinkat = fake_flinkat;

    fake_ops.open_tmpfile = NULL;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "unsupported-tmpfile");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "verify08", 8));
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 10004);
    g_assert_cmpuint(f->mutation_calls, ==, calls);
    g_assert_false(f->object_present);
    fake_ops.open_tmpfile = fake_open_tmpfile;

    f->fail_verifier_set = true;
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, &root, "rollback");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "verify02", 8));
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), 10004);
    g_assert_false(f->object_present);
    g_assert_false(f->verifier_present);
    f->fail_verifier_set = false;
}

static void send_exclusive(Fixture *f, const Nfs2FileHandle *root,
                           const char *name, const uint8_t verifier[8],
                           uint32_t expected)
{
    uint8_t body[128];
    Nfs2XdrWriter w;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name3(&w, root, name);
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, verifier, 8));
    send_nfs(f, 3, 8, body, nfs2_xdr_writer_size(&w), expected);
}

static void test_review_exclusive_durability_order(Fixture *f,
                                                   gconstpointer opaque)
{
    static const uint8_t verifier[8] = "durable";
    Nfs2FileHandle root;

    make_writable(f);
    root = mount_root(f, 3);
    f->track_exclusive = true;
    send_exclusive(f, &root, "durable", verifier, 0);
    g_assert_cmpuint(f->verifier_set_order, >, 0);
    g_assert_cmpuint(f->file_fsync_order, >, f->verifier_set_order);
    g_assert_cmpuint(f->publish_order, >, f->file_fsync_order);
    g_assert_cmpuint(f->dir_fsync_order, >, f->publish_order);
    g_assert_cmpuint(f->named_exclusive_creates, ==, 0);
    g_assert_cmpuint(f->unlink_calls, ==, 0);
}

static void test_review_exclusive_replacement_binding(Fixture *f,
                                                      gconstpointer opaque)
{
    static const uint8_t verifier[8] = "binding";
    Nfs2FileHandle root;

    make_writable(f);
    root = mount_root(f, 3);
    f->track_exclusive = true;
    send_exclusive(f, &root, "retry-bind", verifier, 0);
    f->unlink_calls = 0;
    f->replace_after_fget = true;
    send_exclusive(f, &root, "retry-bind", verifier, 17);
    g_assert_true(f->linked_present);
    g_assert_false(f->linked_verifier_present);
    g_assert_cmpuint(f->unlink_calls, ==, 0);

    f->object_present = false;
    f->verifier_present = false;
    f->replace_after_fset = true;
    f->unlink_calls = 0;
    send_exclusive(f, &root, "new-bind", verifier, 0);
    g_assert_true(f->linked_present);
    g_assert_cmpstr(f->linked_name, ==, "new-bind");
    g_assert_true(f->linked_verifier_present);
    g_assert_false(f->object_present);
    g_assert_cmpuint(f->unlink_calls, ==, 0);

    f->object_present = false;
    f->exclusive_publish_collision = true;
    send_exclusive(f, &root, "publish-collision", verifier, 17);
    g_assert_false(f->object_present);
    g_assert_true(f->linked_present);
    g_assert_cmpstr(f->linked_name, ==, "publish-collision");

    f->linked_present = false;
    f->replace_temp_before_publish = true;
    send_exclusive(f, &root, "fd-publish", verifier, 0);
    g_assert_true(f->linked_present);
    g_assert_cmpstr(f->linked_name, ==, "fd-publish");
    g_assert_cmpuint(f->linked_ino, ==, f->created_open_ino);
    g_assert_true(f->linked_verifier_present);
    g_assert_false(f->object_present);
}

static void test_review_exclusive_fsync_and_rollback_failures(
    Fixture *f, gconstpointer opaque)
{
    static const uint8_t verifier[8] = "failure";
    Nfs2FileHandle root;

    make_writable(f);
    root = mount_root(f, 3);
    f->track_exclusive = true;

    f->file_fsync_error = EIO;
    send_exclusive(f, &root, "file-sync", verifier, 5);
    g_assert_false(f->object_present);
    g_assert_false(f->linked_present);
    f->file_fsync_error = 0;

    f->dir_fsync_error = EIO;
    send_exclusive(f, &root, "dir-sync", verifier, 5);
    g_assert_true(f->linked_present);
    g_assert_true(f->linked_verifier_present);
    g_assert_cmpuint(f->dir_fsync_calls, ==, 1);
    send_exclusive(f, &root, "dir-sync", verifier, 5);
    g_assert_cmpuint(f->dir_fsync_calls, ==, 2);
    f->dir_fsync_error = 0;
    send_exclusive(f, &root, "dir-sync", verifier, 0);
    g_assert_cmpuint(f->dir_fsync_calls, ==, 3);
    f->linked_present = false;
    f->object_present = false;
    f->verifier_present = false;

    f->fail_verifier_set = true;
    f->unlink_error = EIO;
    f->unlink_calls = 0;
    send_exclusive(f, &root, "unlink-fail", verifier, 10004);
    g_assert_cmpuint(f->unlink_calls, ==, 0);
    g_assert_false(f->object_present);
    g_assert_false(f->linked_present);
    g_assert_false(f->verifier_present);
}

static void test_review_duplicate_setattr_write(Fixture *f,
                                                gconstpointer opaque)
{
    Nfs2FileHandle root2, root3, file2, file3;
    uint8_t body[160];
    Nfs2XdrWriter w;

    make_writable(f);
    root2 = mount_root(f, 1);
    root3 = mount_root(f, 3);
    file2 = lookup(f, 2, &root2, "kernel");
    file3 = lookup(f, 3, &root3, "kernel");

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, file2.bytes, 32));
    put_sattr2(&w, 0644);
    send_nfs(f, 2, NFS2_NFSPROC_SETATTR, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file3.bytes, 32, 32));
    put_sattr3(&w, 0600);
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    send_nfs(f, 3, 2, body, nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, file2.bytes, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "2", 1, 8192));
    send_nfs(f, 2, NFS2_NFSPROC_WRITE, body,
             nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, file3.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "3", 1, 8192));
    send_nfs(f, 3, 7, body, nfs2_xdr_writer_size(&w), 0);
}

static void test_review_link_postattrs_and_validation(Fixture *f,
                                                      gconstpointer opaque)
{
    Nfs2FileHandle root, kernel;
    uint8_t body[128];
    Nfs2XdrWriter w;

    make_writable(f);
    root = mount_root(f, 3);
    kernel = lookup(f, 3, &root, "kernel");
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    put_name3(&w, &root, "failed-link");
    f->link_error = EIO;
    send_nfs(f, 3, 15, body, nfs2_xdr_writer_size(&w), 5);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);
    f->link_error = 0;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    put_name3(&w, &root, "replaced-link");
    f->replace_link_result = true;
    send_nfs(f, 3, 15, body, nfs2_xdr_writer_size(&w), 70);
    g_assert_true(f->linked_present);
    g_assert_cmpstr(f->linked_name, ==, "replaced-link");
    g_assert_cmpuint(f->linked_ino, !=, f->kernel_ino);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);
    g_assert_cmpuint(reply_word(f, 10), ==, 1);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    put_name3(&w, &root, "postfail-link");
    f->fail_link_lstat_once = true;
    send_nfs(f, 3, 15, body, nfs2_xdr_writer_size(&w), 5);
    g_assert_true(f->linked_present);
    g_assert_cmpstr(f->linked_name, ==, "postfail-link");
    g_assert_cmpuint(f->linked_ino, ==, f->kernel_ino);
    g_assert_cmpuint(reply_word(f, 7), ==, 1);
    g_assert_cmpuint(reply_word(f, 10), ==, 2);
}

static void test_review_link_alias_lifecycle(Fixture *f,
                                             gconstpointer opaque)
{
    Nfs2FileHandle roots[2], objects[2];
    const uint32_t versions[2] = { 2, 3 };
    uint8_t call[192], body[160];
    Nfs2XdrWriter w;
    size_t len;

    make_writable(f);
    roots[0] = mount_root(f, 1);
    roots[1] = mount_root(f, 3);
    for (unsigned int i = 0; i < G_N_ELEMENTS(versions); i++) {
        const uint32_t version = versions[i];
        const char *source = version == 2 ? "alias2-src" : "alias3-src";
        const char *destination = version == 2 ? "alias2-dst" : "alias3-dst";

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        if (version == 2) {
            put_name2(&w, &roots[i], source);
            put_sattr2(&w, 0600);
        } else {
            put_name3(&w, &roots[i], source);
            g_assert_true(nfs2_xdr_put_u32(&w, 1));
            put_sattr3(&w, 0600);
        }
        send_nfs(f, version, version == 2 ? NFS2_NFSPROC_CREATE : 8,
                 body, nfs2_xdr_writer_size(&w), 0);
        memcpy(objects[i].bytes, f->reply->data + (version == 2 ? 28 : 36),
               sizeof(objects[i].bytes));

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        if (version == 2) {
            g_assert_true(nfs2_xdr_put_opaque(&w, objects[i].bytes, 32));
            put_name2(&w, &roots[i], destination);
        } else {
            g_assert_true(nfs2_xdr_put_counted_opaque(&w, objects[i].bytes,
                                                       32, 32));
            put_name3(&w, &roots[i], destination);
        }
        send_nfs(f, version, version == 2 ? NFS2_NFSPROC_LINK : 15,
                 body, nfs2_xdr_writer_size(&w), 0);

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        if (version == 2) {
            put_name2(&w, &roots[i], source);
        } else {
            put_name3(&w, &roots[i], source);
        }
        send_nfs(f, version, version == 2 ? NFS2_NFSPROC_REMOVE : 12,
                 body, nfs2_xdr_writer_size(&w), 0);

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        if (version == 2) {
            g_assert_true(nfs2_xdr_put_opaque(&w, objects[i].bytes, 32));
        } else {
            g_assert_true(nfs2_xdr_put_counted_opaque(&w, objects[i].bytes,
                                                       32, 32));
        }
        len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, version,
                       NFS2_NFSPROC_GETATTR, body,
                       nfs2_xdr_writer_size(&w));
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 6), ==, 0);

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        if (version == 2) {
            g_assert_true(nfs2_xdr_put_opaque(&w, objects[i].bytes, 32));
            g_assert_true(nfs2_xdr_put_u32(&w, 0));
            g_assert_true(nfs2_xdr_put_u32(&w, 6));
            g_assert_true(nfs2_xdr_put_u32(&w, 6));
        } else {
            g_assert_true(nfs2_xdr_put_counted_opaque(&w, objects[i].bytes,
                                                       32, 32));
            g_assert_true(nfs2_xdr_put_u32(&w, 0));
            g_assert_true(nfs2_xdr_put_u32(&w, 0));
            g_assert_true(nfs2_xdr_put_u32(&w, 6));
        }
        len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, version,
                       NFS2_NFSPROC_READ, body,
                       nfs2_xdr_writer_size(&w));
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(reply_word(f, 6), ==, 0);
        g_assert_cmpmem(f->reply->data + f->reply->len - 8, 6,
                        "NetBSD", 6);

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        if (version == 2) {
            put_name2(&w, &roots[i], destination);
        } else {
            put_name3(&w, &roots[i], destination);
        }
        send_nfs(f, version, version == 2 ? NFS2_NFSPROC_REMOVE : 12,
                 body, nfs2_xdr_writer_size(&w), 0);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qemu_init_main_loop(&error_abort);
    g_test_add("/nfs/server/portmap-dual", Fixture, NULL, setup,
               test_portmap_dual, teardown);
    g_test_add("/nfs/server/v2-bootstrap", Fixture, NULL, setup,
               test_v2_bootstrap, teardown);
    g_test_add("/nfs/server/v3-kernel-flow", Fixture, NULL, setup,
               test_v3_kernel_flow, teardown);
    g_test_add("/nfs/server/read-metadata", Fixture, NULL, setup,
               test_read_metadata_surfaces, teardown);
    g_test_add("/nfs/server/bounds-errors", Fixture, NULL, setup,
               test_bounds_and_errors, teardown);
    g_test_add("/nfs/server/stale-identity-access", Fixture, NULL, setup,
               test_stale_identity_and_access, teardown);
    g_test_add("/nfs/server/readdir3-bounds", Fixture, NULL, setup,
               test_readdir3_bounds, teardown);
    g_test_add("/nfs/server/protocol-vectors", Fixture, NULL, setup,
               test_protocol_vectors, teardown);
    g_test_add("/nfs/server/open-identity-race", Fixture, NULL, setup,
               test_open_identity_race, teardown);
    g_test_add("/nfs/server/path-identity-race", Fixture, NULL, setup,
               test_path_identity_race, teardown);
    g_test_add("/nfs/server/v2-component-validation", Fixture, NULL, setup,
               test_v2_component_validation, teardown);
    g_test_add("/nfs/server/identity-v2-mappings", Fixture, NULL, setup,
               test_identity_and_v2_mappings, teardown);
    g_test_add("/nfs/server/v3-wire-semantics", Fixture, NULL, setup,
               test_v3_wire_semantics, teardown);
    g_test_add("/nfs2/mutations", Fixture, NULL, setup,
               test_mutations, teardown);
    g_test_add("/nfs2/v2-mutation-procedures", Fixture, NULL, setup,
               test_v2_mutation_procedures, teardown);
    g_test_add("/nfs3/mutation-procedures", Fixture, NULL, setup,
               test_v3_mutation_procedures, teardown);
    g_test_add("/nfs/cache/exact-peer-body-expiry-lru-inflight", Fixture,
               NULL, setup, test_duplicate_cache, teardown);
    g_test_add("/nfs/security/auth-sys-readonly-fsdev", Fixture, NULL,
               setup, test_auth_sys_and_readonly_fsdev, teardown);
    g_test_add("/nfs/cache/reset-close-inflight-lifetime", Fixture, NULL,
               setup, test_mutation_lifetime, teardown);
    g_test_add("/nfs/review/commit-vector", Fixture, NULL, setup,
               test_review_commit_vector, teardown);
    g_test_add("/nfs/review/v2-write-totalcount", Fixture, NULL, setup,
               test_review_v2_write_totalcount, teardown);
    g_test_add("/nfs/review/unregistered-remove-inode-zero", Fixture, NULL,
               setup, test_review_unregistered_remove_inode_zero, teardown);
    g_test_add("/nfs/review/readonly-mutation-cache", Fixture, NULL, setup,
               test_review_readonly_mutation_cache, teardown);
    g_test_add("/nfs/review/reset-suppresses-reentrant", Fixture, NULL, setup,
               test_review_reset_suppresses_and_reentrant, teardown);
    g_test_add("/nfs/review/concurrent-rename-remove", Fixture, NULL, setup,
               test_review_concurrent_rename_remove, teardown);
    g_test_add("/nfs/review/exclusive-xattr-failures", Fixture, NULL, setup,
               test_review_exclusive_xattr_failures, teardown);
    g_test_add("/nfs/review/exclusive-durability-order", Fixture, NULL,
               setup, test_review_exclusive_durability_order, teardown);
    g_test_add("/nfs/review/exclusive-replacement-binding", Fixture, NULL,
               setup, test_review_exclusive_replacement_binding, teardown);
    g_test_add("/nfs/review/exclusive-fsync-rollback-failures", Fixture, NULL,
               setup, test_review_exclusive_fsync_and_rollback_failures,
               teardown);
    g_test_add("/nfs/review/duplicate-setattr-write", Fixture, NULL, setup,
               test_review_duplicate_setattr_write, teardown);
    g_test_add("/nfs/review/link-postattrs-validation", Fixture, NULL, setup,
               test_review_link_postattrs_and_validation, teardown);
    g_test_add("/nfs/review/link-alias-lifecycle", Fixture, NULL, setup,
               test_review_link_alias_lifecycle, teardown);
    return g_test_run();
}
