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

typedef struct FakeOpen {
    bool directory;
    bool emitted;
    uint64_t ino;
    struct dirent entry;
} FakeOpen;

typedef struct Fixture {
    FsDriverEntry fse;
    Nfs2Server *server;
    GByteArray *reply;
    GThread *main_thread;
    bool backend_on_main;
    uint64_t kernel_ino;
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
    bool remove_child_on_lookup;
    bool fail_replaced_statfs;
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
        (!strcmp(name, "kernel") || !strcmp(name, "link"))) {
        g_autofree char *joined = g_strconcat("/", name, NULL);

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
        st->st_mode = S_IFREG | 0555;
        st->st_nlink = 1;
        st->st_ino = current->kernel_ino;
        st->st_size = 6;
        st->st_blocks = 1;
        return 0;
    }
    if (!strcmp(path->data, "/link")) {
        st->st_mode = S_IFLNK | 0777;
        st->st_nlink = 1;
        st->st_ino = current->link_ino;
        st->st_size = 6;
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
    note_backend();
    if (strcmp(path->data, "/kernel")) {
        errno = EISDIR;
        return -1;
    }
    if (current->fail_replaced_open) {
        current->kernel_ino++;
        current->fail_replaced_open = false;
        errno = ELOOP;
        return -1;
    }
    state->private = g_new0(FakeOpen, 1);
    if (current->replace_on_open) {
        current->kernel_ino++;
        current->replace_on_open = false;
    }
    ((FakeOpen *)state->private)->ino = current->kernel_ino;
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
    return open->emitted ? 1 : 0;
}

static void fake_seekdir(FsContext *ctx, V9fsFidOpenState *state,
                         off_t offset)
{
    FakeOpen *open = state->private;

    note_backend();
    open->emitted = offset != 0;
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
    st->st_mode = open->directory ? S_IFDIR | 0755 : S_IFREG | 0555;
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
    .close = fake_close,
    .preadv = fake_preadv,
    .fstat = fake_fstat,
    .opendir = fake_opendir,
    .readdir = fake_readdir,
    .telldir = fake_telldir,
    .seekdir = fake_seekdir,
    .closedir = fake_close,
    .statfs = fake_statfs,
};

static int send_reply(Nfs2Service service, const struct sockaddr_in *peer,
                      const uint8_t *data, size_t len, void *opaque)
{
    Fixture *f = opaque;

    g_byte_array_set_size(f->reply, 0);
    g_byte_array_append(f->reply, data, len);
    return 0;
}

static const Nfs2TransportOps transport = { .send = send_reply };

static void setup(Fixture *f, gconstpointer opaque)
{
    current = f;
    f->main_thread = g_thread_self();
    f->reply = g_byte_array_new();
    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)"fake-nfs",
        .path = (char *)"/fake",
        .export_flags = V9FS_SM_MAPPED | V9FS_RDONLY,
        .ops = &fake_ops,
    };
    f->kernel_ino = UINT64_C(0x100000002);
    f->root_ino = 1;
    f->link_ino = 3;
    f->server = nfs2_server_new("fake-nfs", false, &transport, f,
                                &error_abort);
    f->backend_on_main = false;
}

static void teardown(Fixture *f, gconstpointer opaque)
{
    nfs2_server_free(f->server);
    g_byte_array_unref(f->reply);
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

static void request(Fixture *f, Nfs2Service service, const void *data,
                    size_t len)
{
    const struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_port = htons(900),
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
    return g_test_run();
}
