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

static Nfs2FileHandle mount_root(LocalFixture *f)
{
    uint8_t body[8] = { 0 }, call[128];
    Nfs2FileHandle handle;
    size_t len;

    stl_be_p(body, 1);
    body[4] = '/';
    len = rpc_call(call, sizeof(call), NFS2_MOUNT_PROGRAM, 3,
                   NFS2_MOUNT_MNT, body, sizeof(body));
    request(f, NFS2_SERVICE_MOUNT, call, len);
    g_assert_cmpuint(word(f, 6), ==, 0);
    g_assert_cmpuint(word(f, 7), ==, 32);
    memcpy(handle.bytes, f->reply->data + 32, 32);
    return handle;
}

static Nfs2FileHandle lookup(LocalFixture *f, const Nfs2FileHandle *dir,
                             const char *name, uint32_t expected)
{
    uint8_t body[80], call[180];
    Nfs2FileHandle handle = { 0 };
    Nfs2XdrWriter w;
    size_t len;

    nfs2_xdr_writer_init(&w, body, sizeof(body));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, dir->bytes, 32, 32));
    g_assert_true(nfs2_xdr_put_counted_opaque(&w, name, strlen(name), 255));
    len = rpc_call(call, sizeof(call), NFS2_NFS_PROGRAM, 3, 3,
                   body, nfs2_xdr_writer_size(&w));
    request(f, NFS2_SERVICE_NFS, call, len);
    g_assert_cmpuint(word(f, 6), ==, expected);
    if (!expected) {
        g_assert_cmpuint(word(f, 7), ==, 32);
        memcpy(handle.bytes, f->reply->data + 32, 32);
    }
    return handle;
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
    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)"local-nfs",
        .path = f->root,
        .ops = &local_ops,
        .export_flags = V9FS_SM_MAPPED | V9FS_RDONLY,
        .max_xattr = V9FS_MAX_XATTR_DEFAULT,
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
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qemu_init_main_loop(&error_abort);
    g_test_add("/nfs/server/local-mapped-confined", LocalFixture, NULL,
               setup, test_mapped_and_confined, teardown);
    return g_test_run();
}
