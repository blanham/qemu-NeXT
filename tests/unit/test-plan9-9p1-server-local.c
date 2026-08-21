/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Plan 9 Second Edition server integration with QEMU's local fsdev. */

#include "qemu/osdep.h"

#include "fsdev/qemu-fsdev.h"
#include "fsdev/qemu-fsdev-throttle.h"
#include "hw/9pfs/9p.h"
#include "hw/9pfs/plan9-9p1-codec.h"
#include "hw/9pfs/plan9-9p1-server.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"

typedef struct LocalFixture {
    FsDriverEntry fse;
    Plan9P1Server *server;
    GByteArray *output;
    char *root;
    char *outside;
} LocalFixture;

static LocalFixture *fixture;

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

int fsdev_throttle_parse_opts(QemuOpts *opts, FsThrottle *fst, Error **errp)
{
    return 0;
}

void v9fs_path_free(V9fsPath *path)
{
    g_free(path->data);
    path->data = NULL;
    path->size = 0;
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

static size_t can_send(void *opaque)
{
    return SIZE_MAX;
}

static int send_bytes(const uint8_t *buf, size_t len, void *opaque)
{
    GByteArray *output = opaque;

    g_byte_array_append(output, buf, len);
    return len;
}

static const Plan9P1TransportOps transport_ops = {
    .can_send = can_send,
    .send = send_bytes,
};

static void write_file(const char *path, const char *contents)
{
    GError *err = NULL;

    g_assert_true(g_file_set_contents(path, contents, -1, &err));
    g_assert_no_error(err);
}

static void setup(LocalFixture *f, gconstpointer opaque)
{
    g_autofree char *dir = NULL;
    g_autofree char *file = NULL;
    g_autofree char *link = NULL;
    GError *err = NULL;

    fixture = f;
    f->root = g_dir_make_tmp("qemu-9p1-local-XXXXXX", &err);
    g_assert_no_error(err);
    f->outside = g_strdup_printf("%s-outside", f->root);
    write_file(f->outside, "outside\n");
    dir = g_build_filename(f->root, "a", NULL);
    g_assert_cmpint(g_mkdir(dir, 0700), ==, 0);
    file = g_build_filename(dir, "file", NULL);
    write_file(file, "local-backend\n");
    link = g_build_filename(f->root, "escape", NULL);
    g_assert_cmpint(symlink(f->outside, link), ==, 0);

    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)"localfs",
        .path = f->root,
        .ops = &local_ops,
        .export_flags = V9FS_SM_NONE | V9FS_RDONLY,
        .max_xattr = V9FS_MAX_XATTR_DEFAULT,
    };
    f->output = g_byte_array_new();
    f->server = plan9p1_server_new("localfs", &transport_ops, f->output,
                                   NULL, &error_abort);
}

static void pump(Plan9P1Server *server)
{
    unsigned int iterations = 0;

    while (plan9p1_server_busy(server)) {
        g_assert_cmpuint(iterations++, <, 10000);
        aio_poll(qemu_get_aio_context(), true);
    }
}

static Plan9P1Fcall transact(LocalFixture *f, Plan9P1Fcall *call)
{
    uint8_t wire[PLAN9P1_MAX_FRAME];
    Plan9P1Fcall reply;
    ssize_t len;

    len = plan9p1_encode(wire, sizeof(wire), call, &error_abort);
    g_assert_cmpint(plan9p1_server_receive(f->server, wire, len,
                                           &error_abort), ==, 0);
    pump(f->server);
    g_assert_cmpuint(f->output->len, >, 0);
    g_assert_cmpint(plan9p1_decode(f->output->data, f->output->len,
                                  &reply, &error_abort), ==, 0);
    g_byte_array_set_size(f->output, 0);
    return reply;
}

static void set_name(uint8_t name[PLAN9P1_NAMELEN], const char *value)
{
    memset(name, 0, PLAN9P1_NAMELEN);
    memcpy(name, value, MIN(strlen(value), (size_t)PLAN9P1_NAMELEN));
}

static void attach(LocalFixture *f, uint16_t fid)
{
    Plan9P1Fcall call = {
        .type = PLAN9P1_TATTACH,
        .tag = 1,
        .fid = fid,
    };

    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RATTACH);
}

static Plan9P1Fcall walk(LocalFixture *f, uint16_t fid, uint16_t tag,
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

static void test_local_backend(LocalFixture *f, gconstpointer opaque)
{
    Plan9P1Fcall call;
    Plan9P1Fcall reply;

    attach(f, 1);
    g_assert_cmpuint(walk(f, 1, 2, "a").type, ==, PLAN9P1_RWALK);
    g_assert_cmpuint(walk(f, 1, 3, "file").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 4, .fid = 1,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_ROPEN);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TREAD, .tag = 5, .fid = 1, .count = 32,
    };
    reply = transact(f, &call);
    g_assert_cmpuint(reply.type, ==, PLAN9P1_RREAD);
    g_assert_cmpmem(reply.data, reply.count, "local-backend\n", 14);

    attach(f, 2);
    g_assert_cmpuint(walk(f, 2, 6, "..").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TSTAT, .tag = 7, .fid = 2,
    };
    reply = transact(f, &call);
    g_assert_cmpmem(reply.dir.name, 1, "/", 1);

    attach(f, 3);
    g_assert_cmpuint(walk(f, 3, 8, "escape").type, ==, PLAN9P1_RWALK);
    call = (Plan9P1Fcall) {
        .type = PLAN9P1_TOPEN, .tag = 9, .fid = 3,
    };
    g_assert_cmpuint(transact(f, &call).type, ==, PLAN9P1_RERROR);
}

static void teardown(LocalFixture *f, gconstpointer opaque)
{
    g_autofree char *dir = g_build_filename(f->root, "a", NULL);
    g_autofree char *file = g_build_filename(dir, "file", NULL);
    g_autofree char *link = g_build_filename(f->root, "escape", NULL);

    plan9p1_server_reset(f->server);
    pump(f->server);
    plan9p1_server_free(f->server);
    f->server = NULL;
    g_byte_array_unref(f->output);
    g_assert_cmpint(g_remove(file), ==, 0);
    g_assert_cmpint(g_remove(link), ==, 0);
    g_assert_cmpint(g_rmdir(dir), ==, 0);
    g_assert_cmpint(g_rmdir(f->root), ==, 0);
    g_assert_cmpint(g_remove(f->outside), ==, 0);
    g_free(f->outside);
    g_free(f->root);
    fixture = NULL;
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    qemu_init_main_loop(&error_abort);
    g_test_add("/plan9-9p1-server/local-backend", LocalFixture, NULL,
               setup, test_local_backend, teardown);
    return g_test_run();
}
