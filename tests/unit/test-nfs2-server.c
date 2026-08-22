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
    struct dirent entry;
} FakeOpen;

typedef struct Fixture {
    FsDriverEntry fse;
    Nfs2Server *server;
    GByteArray *reply;
    GThread *main_thread;
    bool backend_on_main;
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
        st->st_ino = 1;
        return 0;
    }
    if (!strcmp(path->data, "/kernel")) {
        st->st_mode = S_IFREG | 0555;
        st->st_nlink = 1;
        st->st_ino = UINT64_C(0x100000002);
        st->st_size = 6;
        st->st_blocks = 1;
        return 0;
    }
    if (!strcmp(path->data, "/link")) {
        st->st_mode = S_IFLNK | 0777;
        st->st_nlink = 1;
        st->st_ino = 3;
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
    state->private = g_new0(FakeOpen, 1);
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
    open = g_new0(FakeOpen, 1);
    open->directory = true;
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
    memset(st, 0, sizeof(*st));
    st->f_bsize = 4096;
    st->f_blocks = 100;
    st->f_bfree = 50;
    st->f_bavail = 40;
    st->f_files = 10;
    st->f_ffree = 8;
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
                             const Nfs2FileHandle *root)
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
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "kernel", 6, 255));
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
    Nfs2FileHandle file = lookup(f, 2, &root);
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
    Nfs2FileHandle file = lookup(f, 3, &root);
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
    return g_test_run();
}
