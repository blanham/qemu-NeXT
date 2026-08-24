/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include <sys/xattr.h>

#include "fsdev/qemu-fsdev.h"
#include "fsdev/qemu-fsdev-throttle.h"
#include "hw/9pfs/9p.h"
#include "hw/nfs/nfs2-protocol.h"
#include "hw/nfs/nfs2-server.h"
#include "hw/nfs/nfs2-xdr.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/main-loop.h"

typedef struct LocalFixture {
    FsDriverEntry fse;
    Nfs2Server *server;
    GByteArray *reply;
    char *root;
    char *outside;
    uint32_t xid;
} LocalFixture;

static LocalFixture *current;

FsDriverEntry *get_fsdev_fsentry(const char *id)
{
    return current && !g_strcmp0(id, current->fse.fsdev_id) ?
           &current->fse : NULL;
}

void fsdev_throttle_init(FsThrottle *fst) { }
void fsdev_throttle_cleanup(FsThrottle *fst) { }
void coroutine_fn fsdev_co_throttle_request(FsThrottle *fst,
                                             ThrottleDirection direction,
                                             struct iovec *iov, int iovcnt)
{
}
int fsdev_throttle_parse_opts(QemuOpts *opts, FsThrottle *fst, Error **errp)
{
    return 0;
}

void v9fs_path_free(V9fsPath *path)
{
    g_free(path->data);
    memset(path, 0, sizeof(*path));
}

int v9fs_path_sprintf(V9fsPath *path, const char *fmt, ...)
{
    va_list ap;
    int ret;

    v9fs_path_free(path);
    va_start(ap, fmt);
    ret = g_vasprintf(&path->data, fmt, ap);
    va_end(ap);
    if (ret >= 0) {
        path->size = ret + 1;
    }
    return ret < 0 ? -1 : 0;
}

void v9fs_path_copy(V9fsPath *dst, const V9fsPath *src)
{
    v9fs_path_free(dst);
    dst->data = g_memdup2(src->data, src->size);
    dst->size = src->size;
}

static int send_reply(Nfs2Service service, const struct sockaddr_in *peer,
                      const uint8_t *data, size_t len, void *opaque)
{
    LocalFixture *f = opaque;

    g_byte_array_set_size(f->reply, 0);
    g_byte_array_append(f->reply, data, len);
    return 0;
}

static const Nfs2TransportOps transport = { .send = send_reply };

static size_t rpc_call(uint8_t *buf, size_t capacity, uint32_t program,
                       uint32_t version, uint32_t procedure,
                       const void *body, size_t body_len)
{
    Nfs2XdrWriter w;

    nfs2_xdr_writer_init(&w, buf, capacity);
    g_assert_true(nfs2_xdr_put_u32(&w, 77));
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

static void request(LocalFixture *f, Nfs2Service service,
                    const uint8_t *data, size_t len)
{
    const struct sockaddr_in peer = {
        .sin_family = AF_INET, .sin_port = htons(901),
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
    g_assert_cmpuint(f->reply->len, >=, 28);
}

static uint32_t word(LocalFixture *f, size_t index)
{
    g_assert_cmpuint((index + 1) * 4, <=, f->reply->len);
    return ldl_be_p(f->reply->data + index * 4);
}

static Nfs2FileHandle mount_root_version(LocalFixture *f, uint32_t version)
{
    uint8_t body[8] = { 0 }, call[128];
    Nfs2FileHandle handle;
    size_t len;

    stl_be_p(body, 1);
    body[4] = '/';
    len = rpc_call(call, sizeof(call), NFS2_MOUNT_PROGRAM, version,
                   NFS2_MOUNT_MNT, body, sizeof(body));
    request(f, NFS2_SERVICE_MOUNT, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    if (version == 3) {
        g_assert_cmpuint(word(f, 7), ==, 32);
        memcpy(handle.bytes, f->reply->data + 32, 32);
    } else {
        memcpy(handle.bytes, f->reply->data + 28, 32);
    }
    return handle;
}

static Nfs2FileHandle mount_root(LocalFixture *f)
{
    return mount_root_version(f, 3);
}

static Nfs2FileHandle lookup_version(LocalFixture *f, uint32_t version,
                                     const Nfs2FileHandle *dir,
                                     const char *name, uint32_t expected)
{
    uint8_t body[80], call[180];
    Nfs2FileHandle handle = { 0 };
    Nfs2XdrWriter w;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    if (version == 3) {
        g_assert_true(nfs2_xdr_put_counted_opaque(&w, dir->bytes, 32, 32));
    } else {
        g_assert_true(nfs2_xdr_put_opaque(&w, dir->bytes, 32));
    }
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, name, strlen(name), 255));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, version,
                   version == 3 ? 3 : NFS2_NFSPROC_LOOKUP,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, expected);
    if (!expected) {
        if (version == 3) {
            g_assert_cmpuint(word(f, 7), ==, 32);
            memcpy(handle.bytes, f->reply->data + 32, 32);
        } else {
            memcpy(handle.bytes, f->reply->data + 28, 32);
        }
    }
    return handle;
}

static Nfs2FileHandle lookup(LocalFixture *f, const Nfs2FileHandle *dir,
                             const char *name, uint32_t expected)
{
    return lookup_version(f, 3, dir, name, expected);
}

static void set_mapped(const char *path, uid_t uid, gid_t gid, mode_t mode)
{
    uint32_t wire_uid = cpu_to_le32(uid);
    uint32_t wire_gid = cpu_to_le32(gid);
    uint32_t wire_mode = cpu_to_le32(mode);

    g_assert_cmpint(setxattr(path, "user.virtfs.uid", &wire_uid,
                             sizeof(wire_uid), 0), ==, 0);
    g_assert_cmpint(setxattr(path, "user.virtfs.gid", &wire_gid,
                             sizeof(wire_gid), 0), ==, 0);
    g_assert_cmpint(setxattr(path, "user.virtfs.mode", &wire_mode,
                             sizeof(wire_mode), 0), ==, 0);
}

static void put_name(Nfs2XdrWriter *w, uint32_t version,
                     const Nfs2FileHandle *dir, const char *name)
{
    if (version == 3) {
        g_assert_true(nfs2_xdr_put_counted_opaque(w, dir->bytes, 32, 32));
    } else {
        g_assert_true(nfs2_xdr_put_opaque(w, dir->bytes, 32));
    }
    g_assert_true(nfs2_xdr_put_counted_opaque(w, name, strlen(name), 255));
}

static void put_sattr2(Nfs2XdrWriter *w, mode_t mode)
{
    g_assert_true(nfs2_xdr_put_u32(w, mode));
    for (unsigned int i = 0; i < 7; i++) {
        g_assert_true(nfs2_xdr_put_u32(w, UINT32_MAX));
    }
}

static void put_sattr3(Nfs2XdrWriter *w, mode_t mode,
                       bool large_size)
{
    g_assert_true(nfs2_xdr_put_u32(w, 1));
    g_assert_true(nfs2_xdr_put_u32(w, mode));
    g_assert_true(nfs2_xdr_put_u32(w, 1));
    g_assert_true(nfs2_xdr_put_u32(w, 9999)); /* decoded, ignored uid */
    g_assert_true(nfs2_xdr_put_u32(w, 1));
    g_assert_true(nfs2_xdr_put_u32(w, 9998)); /* decoded, ignored gid */
    g_assert_true(nfs2_xdr_put_u32(w, large_size));
    if (large_size) {
        uint64_t size = (uint64_t)INT32_MAX + 16384;

        g_assert_true(nfs2_xdr_put_u32(w, size >> 32));
        g_assert_true(nfs2_xdr_put_u32(w, size));
    }
    g_assert_true(nfs2_xdr_put_u32(w, 0));
    g_assert_true(nfs2_xdr_put_u32(w, 0));
}

static void mutate(LocalFixture *f, uint32_t version, uint32_t proc,
                   const uint8_t *body, size_t body_len, uint32_t expected)
{
    uint8_t call[512];
    size_t len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, version,
                          proc, body, body_len);

    stl_be_p(call, ++f->xid);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, expected);
}

static void setup(LocalFixture *f, gconstpointer opaque)
{
    g_autofree char *kernel = NULL;
    g_autofree char *escape = NULL;
    g_autofree char *sparse = NULL;
    GError *error = NULL;

    current = f;
    f->root = g_dir_make_tmp("qemu-nfs-local-XXXXXX", &error);
    g_assert_no_error(error);
    f->outside = g_strdup_printf("%s-outside", f->root);
    g_assert_true(g_file_set_contents(f->outside, "outside-secret", -1,
                                      &error));
    g_assert_no_error(error);
    kernel = g_build_filename(f->root, "netbsd", NULL);
    g_assert_true(g_file_set_contents(kernel, "kernel-local", -1, &error));
    g_assert_no_error(error);
    set_mapped(kernel, 4242, 4343, S_IFREG | 0555);
    escape = g_build_filename(f->root, "escape", NULL);
    g_assert_cmpint(symlink(f->outside, escape), ==, 0);
    sparse = g_build_filename(f->root, "sparse", NULL);
    {
        static const char marker[] = "offset64";
        int fd = open(sparse, O_CREAT | O_RDWR, 0600);

        g_assert_cmpint(fd, >=, 0);
        g_assert_cmpint(pwrite(fd, marker, sizeof(marker) - 1,
                               (off_t)INT32_MAX + 4096), ==,
                        sizeof(marker) - 1);
        g_assert_cmpint(close(fd), ==, 0);
    }
    set_mapped(sparse, 1, 2, S_IFREG | 0444);
    set_mapped(f->root, 1001, 1002, S_IFDIR | 0755);
    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)"local-nfs",
        .path = f->root,
        .ops = &local_ops,
        .export_flags = V9FS_SM_MAPPED | V9FS_RDONLY,
        .max_xattr = V9FS_MAX_XATTR_DEFAULT,
        .fmode = 0600,
        .dmode = 0700,
    };
    f->reply = g_byte_array_new();
    f->server = nfs2_server_new("local-nfs", false, &transport, f,
                                &error_abort);
}

static void teardown(LocalFixture *f, gconstpointer opaque)
{
    g_autofree char *kernel = g_build_filename(f->root, "netbsd", NULL);
    g_autofree char *escape = g_build_filename(f->root, "escape", NULL);
    g_autofree char *sparse = g_build_filename(f->root, "sparse", NULL);

    nfs2_server_free(f->server);
    g_byte_array_unref(f->reply);
    g_assert_cmpint(g_remove(kernel), ==, 0);
    g_assert_cmpint(g_remove(escape), ==, 0);
    g_assert_cmpint(g_remove(sparse), ==, 0);
    g_assert_cmpint(g_rmdir(f->root), ==, 0);
    g_assert_cmpint(g_remove(f->outside), ==, 0);
    g_free(f->outside);
    g_free(f->root);
    current = NULL;
}

static void test_mapped_and_confined(LocalFixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root = mount_root(f);
    Nfs2FileHandle kernel = lookup(f, &root, "netbsd", 0);
    Nfs2FileHandle escape;
    Nfs2FileHandle sparse;
    uint8_t body[64], call[180];
    Nfs2XdrWriter w;
    size_t len;

    /* LOOKUP3 object post-op fattr: uid/gid come from mapped xattrs. */
    g_assert_cmpuint(word(f, 20), ==, 4242);
    g_assert_cmpuint(word(f, 21), ==, 4343);
    g_assert_cmpuint(word(f, 18) & 07777, ==, 0555);

    /* Exercise the real local mapped-xattr directory backend. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, root.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_opaque(&w, "\0\0\0\0\0\0\0\0", 8));
    g_assert_true(nfs2_xdr_put_u32(&w, 8192));
    g_assert_true(nfs2_xdr_put_u32(&w, 8192));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 17,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    g_assert_nonnull(memmem(f->reply->data, f->reply->len, "netbsd", 6));
    g_assert_nonnull(memmem(f->reply->data, f->reply->len, "sparse", 6));

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 6,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    g_assert_cmpuint(word(f, 29), ==, 12);
    g_assert_cmpmem(f->reply->data + 128, 12, "kernel-local", 12);

    escape = lookup(f, &root, "escape", 0);
    g_assert_cmpuint(word(f, 17), ==, NFS2_NFLNK);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, escape.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 6,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 22); /* NFS3ERR_INVAL, never followed. */
    g_assert_null(g_strstr_len((char *)f->reply->data, f->reply->len,
                               "outside-secret"));

    sparse = lookup(f, &root, "sparse", 0);
    g_assert_cmpuint(word(f, 22), ==, 0);
    g_assert_cmpuint(word(f, 23), >, INT32_MAX);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, sparse.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, (uint32_t)INT32_MAX + 4096));
    g_assert_true(nfs2_xdr_put_u32(&w, 8));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 6,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    g_assert_cmpuint(word(f, 29), ==, 8);
    g_assert_cmpmem(f->reply->data + 128, 8, "offset64", 8);

    /* A path-preserving external replacement cannot inherit an old fh. */
    {
        g_autofree char *kernel_path = g_build_filename(f->root, "netbsd",
                                                        NULL);
        g_autofree char *replacement = g_build_filename(f->root, "new", NULL);
        GError *error = NULL;

        g_assert_true(g_file_set_contents(replacement, "replacement", -1,
                                          &error));
        g_assert_no_error(error);
        set_mapped(replacement, 4242, 4343, S_IFREG | 0555);
        g_assert_cmpint(g_rename(replacement, kernel_path), ==, 0);
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        g_assert_true(nfs2_xdr_put_counted_opaque(&w, kernel.bytes, 32, 32));
        len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1,
                       body, nfs2_xdr_writer_size(&w));
        request(f, NFS2_SERVICE_NFS, call, len);
        g_assert_cmpuint(word(f, 6), ==, 70);
    }
}

static void test_local_writable(LocalFixture *f, gconstpointer opaque)
{
    Nfs2FileHandle root2, root3, netbsd, sparse, object, directory, child;
    uint8_t body[384], call[512];
    Nfs2XdrWriter w;
    g_autofree char *path1 = NULL;
    g_autofree char *path2 = NULL;
    struct stat st1, st2;
    size_t len;

    nfs2_server_free(f->server);
    f->fse.export_flags = V9FS_SM_MAPPED;
    f->server = nfs2_server_new("local-nfs", true, &transport, f,
                                &error_abort);
    root2 = mount_root_version(f, 1);
    root3 = mount_root_version(f, 3);
    netbsd = lookup_version(f, 2, &root2, "netbsd", 0);
    sparse = lookup(f, &root3, "sparse", 0);

    /* Complete NFSv2 write surface. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, netbsd.bytes, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0640));
    g_assert_true(nfs2_xdr_put_u32(&w, 9999));
    g_assert_true(nfs2_xdr_put_u32(&w, 9998));
    g_assert_true(nfs2_xdr_put_u32(&w, 12));
    for (unsigned int i = 0; i < 4; i++) {
        g_assert_true(nfs2_xdr_put_u32(&w, UINT32_MAX));
    }
    mutate(f, 2, NFS2_NFSPROC_SETATTR, body,
           nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpuint(word(f, 10), ==, 4242);
    g_assert_cmpuint(word(f, 11), ==, 4343);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, netbsd.bytes, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "V2", 2, 8192));
    mutate(f, 2, NFS2_NFSPROC_WRITE, body,
           nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2a");
    put_sattr2(&w, 0600);
    mutate(f, 2, NFS2_NFSPROC_CREATE, body,
           nfs2_xdr_writer_size(&w), 0);
    memcpy(object.bytes, f->reply->data + 28, 32);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_opaque(&w, object.bytes, 32));
    put_name(&w, 2, &root2, "v2hard");
    mutate(f, 2, NFS2_NFSPROC_LINK, body, nfs2_xdr_writer_size(&w), 0);
    path1 = g_build_filename(f->root, "v2a", NULL);
    path2 = g_build_filename(f->root, "v2hard", NULL);
    g_assert_cmpint(lstat(path1, &st1), ==, 0);
    g_assert_cmpint(lstat(path2, &st2), ==, 0);
    g_assert_cmpuint(st1.st_ino, ==, st2.st_ino);
    g_clear_pointer(&path1, g_free);
    g_clear_pointer(&path2, g_free);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2dst");
    put_sattr2(&w, 0600);
    mutate(f, 2, NFS2_NFSPROC_CREATE, body,
           nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2a");
    put_name(&w, 2, &root2, "v2dst");
    mutate(f, 2, NFS2_NFSPROC_RENAME, body,
           nfs2_xdr_writer_size(&w), 0);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_GETATTR, object.bytes, 32);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    for (const char *name = "v2hard"; name;
         name = !strcmp(name, "v2hard") ? "v2dst" : NULL) {
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name(&w, 2, &root2, name);
        mutate(f, 2, NFS2_NFSPROC_REMOVE, body,
               nfs2_xdr_writer_size(&w), 0);
    }

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2sym");
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, f->outside,
                                               strlen(f->outside), 1024));
    put_sattr2(&w, 0777);
    mutate(f, 2, NFS2_NFSPROC_SYMLINK, body,
           nfs2_xdr_writer_size(&w), 0);
    object = lookup_version(f, 2, &root2, "v2sym", 0);
    memcpy(body, object.bytes, 32);
    stl_be_p(body + 32, 0); stl_be_p(body + 36, 32);
    stl_be_p(body + 40, 32);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 2,
                   NFS2_NFSPROC_READ, body, 44);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), !=, 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2sym");
    mutate(f, 2, NFS2_NFSPROC_REMOVE, body,
           nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2dir");
    put_sattr2(&w, 0750);
    mutate(f, 2, NFS2_NFSPROC_MKDIR, body,
           nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 2, &root2, "v2dir");
    mutate(f, 2, NFS2_NFSPROC_RMDIR, body,
           nfs2_xdr_writer_size(&w), 0);

    /* NFSv3 64-bit SETATTR/WRITE/COMMIT and ignored asserted ownership. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, sparse.bytes, 32, 32));
    put_sattr3(&w, 0644, true);
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    mutate(f, 3, 2, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, sparse.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, (uint32_t)INT32_MAX + 20000));
    g_assert_true(nfs2_xdr_put_u32(&w, 5));
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, "large", 5, 8192));
    mutate(f, 3, 7, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, sparse.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    mutate(f, 3, 21, body, nfs2_xdr_writer_size(&w), 0);
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1,
                   body, 36);
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 12), ==, 0);
    g_assert_cmpuint(word(f, 13), >, INT32_MAX);
    g_assert_cmpuint(word(f, 10), ==, 1);
    g_assert_cmpuint(word(f, 11), ==, 2);

    for (uint32_t mode = 0; mode < 3; mode++) {
        const char *name = mode == 0 ? "unchecked" :
                           mode == 1 ? "guarded" : "exclusive";

        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name(&w, 3, &root3, name);
        g_assert_true(nfs2_xdr_put_u32(&w, mode));
        if (mode == 2) {
            g_assert_true(nfs2_xdr_put_opaque(&w, "verifier", 8));
        } else {
            put_sattr3(&w, 0600, false);
        }
        mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
        if (mode == 1) {
            mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 17);
        } else if (mode == 2) {
            mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
            memcpy(body + nfs2_xdr_writer_size(&w) - 8, "different", 8);
            mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 17);
        }
        object = lookup(f, &root3, name, 0);
        g_assert_cmpuint(word(f, 20), ==, 1001);
        g_assert_cmpuint(word(f, 21), ==, 1002);
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name(&w, 3, &root3, name);
        mutate(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
    }

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root3, "v3dir"); put_sattr3(&w, 0750, false);
    mutate(f, 3, 9, body, nfs2_xdr_writer_size(&w), 0);
    memcpy(directory.bytes, f->reply->data + 36, 32);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &directory, "child");
    g_assert_true(nfs2_xdr_put_u32(&w, 1)); put_sattr3(&w, 0600, false);
    mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
    memcpy(child.bytes, f->reply->data + 36, 32);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root3, "v3dir");
    put_name(&w, 3, &root3, "v3new");
    mutate(f, 3, 14, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, child.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1, body,
                   nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &directory, "child");
    mutate(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body)); put_name(&w, 3, &root3,
                                                          "v3new");
    mutate(f, 3, 13, body, nfs2_xdr_writer_size(&w), 0);

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root3, "v3sym"); put_sattr3(&w, 0777, false);
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, f->outside,
                                               strlen(f->outside), 1024));
    mutate(f, 3, 10, body, nfs2_xdr_writer_size(&w), 0);
    object = lookup(f, &root3, "v3sym", 0);
    g_assert_cmpuint(word(f, 17), ==, NFS2_NFLNK);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, object.bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 6, body,
                   nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 22);
    nfs2_xdr_writer_init(&w, body, sizeof(body)); put_name(&w, 3, &root3,
                                                          "v3sym");
    mutate(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);

    for (uint32_t type = 3; type <= 7; type++) {
        if (type == 5) {
            continue;
        }
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name(&w, 3, &root3, "special");
        g_assert_true(nfs2_xdr_put_u32(&w, type));
        put_sattr3(&w, 0600, false);
        if (type == 3 || type == 4) {
            g_assert_true(nfs2_xdr_put_u32(&w, 1));
            g_assert_true(nfs2_xdr_put_u32(&w, 2));
        }
        mutate(f, 3, 11, body, nfs2_xdr_writer_size(&w), 0);
        nfs2_xdr_writer_init(&w, body, sizeof(body)); put_name(&w, 3,
                                                               &root3,
                                                               "special");
        mutate(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
    }

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, sparse.bytes, 32, 32));
    put_name(&w, 3, &root3, "v3hard");
    mutate(f, 3, 15, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body)); put_name(&w, 3, &root3,
                                                          "v3hard");
    mutate(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);

    for (const char *name = "r1"; name;
         name = !strcmp(name, "r1") ? "r2" : NULL) {
        nfs2_xdr_writer_init(&w, body, sizeof(body));
        put_name(&w, 3, &root3, name);
        g_assert_true(nfs2_xdr_put_u32(&w, 1)); put_sattr3(&w, 0600, false);
        mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
        if (!strcmp(name, "r1")) {
            memcpy(object.bytes, f->reply->data + 36, 32);
        }
    }
    nfs2_xdr_writer_init(&w, body, sizeof(body)); put_name(&w, 3, &root3,
                                                          "r1");
    put_name(&w, 3, &root3, "r2");
    mutate(f, 3, 14, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, object.bytes, 32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1, body,
                   nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body)); put_name(&w, 3, &root3,
                                                          "r2");
    mutate(f, 3, 12, body, nfs2_xdr_writer_size(&w), 0);
}

static void put_sattr3_unset(Nfs2XdrWriter *w)
{
    for (unsigned int i = 0; i < 6; i++) {
        g_assert_true(nfs2_xdr_put_u32(w, 0));
    }
}

static void test_local_review_regressions(LocalFixture *f,
                                          gconstpointer opaque)
{
    Nfs2FileHandle root, sparse, symlink_handle, hardlink_handle;
    uint8_t body[384], call[512];
    Nfs2XdrWriter w;
    g_autofree char *netbsd_path = g_build_filename(f->root, "netbsd", NULL);
    g_autofree char *exclusive_path = g_build_filename(f->root, "persist",
                                                        NULL);
    g_autofree char *symlink_path = g_build_filename(f->root, "sized-sym",
                                                      NULL);
    g_autofree char *hardlink_path = g_build_filename(f->root, "post-link",
                                                       NULL);
    g_autofree char *rename_source = g_build_filename(f->root,
                                                       "rename-source", NULL);
    g_autofree char *rename_destination = g_build_filename(
        f->root, "rename-destination", NULL);
    g_autofree char *outside_contents = NULL;
    gsize outside_length;
    uint32_t uid, gid, mode;
    size_t len;

    nfs2_server_free(f->server);
    f->fse.export_flags = V9FS_SM_MAPPED;
    f->server = nfs2_server_new("local-nfs", true, &transport, f,
                                &error_abort);
    root = mount_root(f);
    sparse = lookup(f, &root, "sparse", 0);

    /* UNCHECKED existing objects retain ownership and unspecified mode. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root, "netbsd");
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    put_sattr3_unset(&w);
    mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpint(getxattr(netbsd_path, "user.virtfs.uid", &uid,
                             sizeof(uid)), ==, sizeof(uid));
    g_assert_cmpint(getxattr(netbsd_path, "user.virtfs.gid", &gid,
                             sizeof(gid)), ==, sizeof(gid));
    g_assert_cmpint(getxattr(netbsd_path, "user.virtfs.mode", &mode,
                             sizeof(mode)), ==, sizeof(mode));
    g_assert_cmpuint(le32_to_cpu(uid), ==, 4242);
    g_assert_cmpuint(le32_to_cpu(gid), ==, 4343);
    g_assert_cmpuint(le32_to_cpu(mode) & 07777, ==, 0555);

    /* EXCLUSIVE verifier survives server recreation and is object-bound. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root, "persist");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "persist!", 8));
    mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
    {
        g_autoptr(GDir) dir = g_dir_open(f->root, 0, NULL);
        const char *entry;

        g_assert_nonnull(dir);
        while ((entry = g_dir_read_name(dir))) {
            g_assert_false(g_str_has_prefix(entry,
                                             ".qemu-nfs3-exclusive-"));
        }
    }
    nfs2_server_free(f->server);
    f->server = nfs2_server_new("local-nfs", true, &transport, f,
                                &error_abort);
    root = mount_root(f);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root, "persist");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "persist!", 8));
    mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpint(g_remove(exclusive_path), ==, 0);
    g_assert_cmpint(g_close(g_open(exclusive_path, O_CREAT | O_WRONLY,
                                   0600), NULL), ==, TRUE);
    set_mapped(exclusive_path, 1001, 1002, S_IFREG | 0600);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root, "persist");
    g_assert_true(nfs2_xdr_put_u32(&w, 2));
    g_assert_true(nfs2_xdr_put_opaque(&w, "persist!", 8));
    mutate(f, 3, 8, body, nfs2_xdr_writer_size(&w), 17);
    g_assert_cmpint(g_remove(exclusive_path), ==, 0);
    nfs2_server_free(f->server);
    f->server = nfs2_server_new("local-nfs", true, &transport, f,
                                &error_abort);
    root = mount_root(f);
    sparse = lookup(f, &root, "sparse", 0);

    /* A supplied symlink size is legal but must never truncate its target. */
    g_assert_true(g_file_get_contents(f->outside, &outside_contents,
                                      &outside_length, NULL));
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root, "sized-sym");
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    g_assert_true(nfs2_xdr_put_u32(&w, 0777));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, f->outside,
                                               strlen(f->outside), 1024));
    mutate(f, 3, 10, body, nfs2_xdr_writer_size(&w), 0);
    memcpy(symlink_handle.bytes, f->reply->data + 36,
           sizeof(symlink_handle.bytes));
    {
        g_autofree char *after = NULL, *placeholder = NULL;
        gsize after_length, placeholder_length;

        g_assert_true(g_file_get_contents(f->outside, &after, &after_length,
                                          NULL));
        g_assert_cmpmem(after, after_length, outside_contents, outside_length);
        g_assert_true(g_file_get_contents(symlink_path, &placeholder,
                                          &placeholder_length, NULL));
        g_assert_cmpmem(placeholder, placeholder_length,
                        f->outside, strlen(f->outside));
    }

    /* SETATTR size must not truncate a mapped symlink placeholder. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, symlink_handle.bytes,
                                               32, 32));
    g_assert_true(nfs2_xdr_put_u32(&w, 1)); /* mode */
    g_assert_true(nfs2_xdr_put_u32(&w, 0600));
    g_assert_true(nfs2_xdr_put_u32(&w, 0)); /* uid */
    g_assert_true(nfs2_xdr_put_u32(&w, 0)); /* gid */
    g_assert_true(nfs2_xdr_put_u32(&w, 1)); /* size */
    g_assert_true(nfs2_xdr_put_u32(&w, 0));
    g_assert_true(nfs2_xdr_put_u32(&w, 1));
    g_assert_true(nfs2_xdr_put_u32(&w, 0)); /* atime */
    g_assert_true(nfs2_xdr_put_u32(&w, 0)); /* mtime */
    g_assert_true(nfs2_xdr_put_u32(&w, 0)); /* guard */
    mutate(f, 3, 2, body, nfs2_xdr_writer_size(&w), 22);
    {
        g_autofree char *placeholder = NULL;
        gsize placeholder_length;

        g_assert_true(g_file_get_contents(symlink_path, &placeholder,
                                          &placeholder_length, NULL));
        g_assert_cmpmem(placeholder, placeholder_length,
                        f->outside, strlen(f->outside));
        g_assert_cmpint(getxattr(symlink_path, "user.virtfs.mode", &mode,
                                 sizeof(mode)), ==, sizeof(mode));
        g_assert_cmpuint(le32_to_cpu(mode) & 07777, ==, 0777);
    }
    g_assert_cmpint(g_remove(symlink_path), ==, 0);

    /* A same-inode RENAME no-op preserves a destination-only handle. */
    g_assert_true(g_file_set_contents(rename_source, "same inode", -1, NULL));
    set_mapped(rename_source, 1001, 1002, S_IFREG | 0600);
    g_assert_cmpint(link(rename_source, rename_destination), ==, 0);
    hardlink_handle = lookup(f, &root, "rename-destination", 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    put_name(&w, 3, &root, "rename-source");
    put_name(&w, 3, &root, "rename-destination");
    mutate(f, 3, 14, body, nfs2_xdr_writer_size(&w), 0);
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, hardlink_handle.bytes,
                                               32, 32));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 1, body,
                   nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    g_assert_cmpint(g_remove(rename_source), ==, 0);
    g_assert_cmpint(g_remove(rename_destination), ==, 0);

    /* LINK3 returns the source's true post-link nlink. */
    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, sparse.bytes, 32, 32));
    put_name(&w, 3, &root, "post-link");
    mutate(f, 3, 15, body, nfs2_xdr_writer_size(&w), 0);
    g_assert_cmpuint(word(f, 7), ==, 1);
    g_assert_cmpuint(word(f, 10), ==, 2);
    g_assert_cmpint(g_remove(hardlink_path), ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qemu_init_main_loop(&error_abort);
    g_test_add("/nfs/server/local-mapped-confined", LocalFixture, NULL,
               setup, test_mapped_and_confined, teardown);
    g_test_add("/nfs2/local-writable", LocalFixture, NULL,
               setup, test_local_writable, teardown);
    g_test_add("/nfs3/local-review-regressions", LocalFixture, NULL,
               setup, test_local_review_regressions, teardown);
    return g_test_run();
}
