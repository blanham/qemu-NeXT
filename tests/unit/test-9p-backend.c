/* SPDX-License-Identifier: GPL-2.0-or-later */
/* QEMU 9p backend lifecycle tests. */
/* Copyright (c) 2026 Bryce Lanham */

#include "qemu/osdep.h"

#include "fsdev/qemu-fsdev.h"
#include "hw/9pfs/9p-backend.h"
#include "qapi/error.h"

typedef struct BackendFixture {
    FsDriverEntry fse;
    V9fsBackend backend;
    const char *registered_id;
    int init_result;
    int name_to_path_result;
    int lstat_result;
    mode_t root_mode;
    dev_t root_dev;
    int init_calls;
    int cleanup_calls;
    int throttle_init_calls;
    int throttle_cleanup_calls;
} BackendFixture;

static BackendFixture *fixture;

FsDriverEntry *get_fsdev_fsentry(const char *id)
{
    if (fixture->registered_id && !g_strcmp0(id, fixture->registered_id)) {
        return &fixture->fse;
    }
    return NULL;
}

void fsdev_throttle_init(FsThrottle *fst)
{
    g_assert_true(fst == fixture->backend.ctx.fst);
    fixture->throttle_init_calls++;
}

void fsdev_throttle_cleanup(FsThrottle *fst)
{
    g_assert_true(fst == fixture->backend.ctx.fst);
    fixture->throttle_cleanup_calls++;
}

static int fake_init(FsContext *ctx, Error **errp)
{
    fixture->init_calls++;
    g_assert_cmpstr(ctx->fs_root, ==, fixture->fse.path);
    g_assert_true(ctx->fs_root != fixture->fse.path);
    g_assert_cmpint(ctx->export_flags, ==, fixture->fse.export_flags);
    g_assert_cmpint(ctx->fmode, ==, fixture->fse.fmode);
    g_assert_cmpint(ctx->dmode, ==, fixture->fse.dmode);
    g_assert_true(ctx->fst == &fixture->backend.throttle);
    if (fixture->init_result < 0) {
        error_setg(errp, "fake init failure");
    }
    return fixture->init_result;
}

static void fake_cleanup(FsContext *ctx)
{
    g_assert_true(ctx == &fixture->backend.ctx);
    fixture->cleanup_calls++;
}

static int fake_name_to_path(FsContext *ctx, V9fsPath *dirpath,
                             const char *name, V9fsPath *path)
{
    g_assert_true(ctx == &fixture->backend.ctx);
    g_assert_null(dirpath);
    g_assert_cmpstr(name, ==, "/");
    if (fixture->name_to_path_result < 0) {
        errno = ENOENT;
        return fixture->name_to_path_result;
    }
    path->data = g_strdup("/");
    path->size = 2;
    return 0;
}

static int fake_lstat(FsContext *ctx, V9fsPath *path, struct stat *st)
{
    g_assert_true(ctx == &fixture->backend.ctx);
    g_assert_cmpstr(path->data, ==, "/");
    if (fixture->lstat_result < 0) {
        return fixture->lstat_result;
    }
    st->st_mode = fixture->root_mode;
    st->st_dev = fixture->root_dev;
    return 0;
}

static FileOperations fake_ops = {
    .init = fake_init,
    .cleanup = fake_cleanup,
    .name_to_path = fake_name_to_path,
    .lstat = fake_lstat,
};

static void fixture_setup(BackendFixture *f, gconstpointer opaque)
{
    fixture = f;
    f->registered_id = "testfs";
    f->root_mode = S_IFDIR | 0755;
    f->root_dev = 42;
    f->fse = (FsDriverEntry) {
        .fsdev_id = (char *)f->registered_id,
        .path = (char *)"/export",
        .export_flags = V9FS_RDONLY | V9FS_REMAP_INODES,
        .ops = &fake_ops,
        .fmode = 0600,
        .dmode = 0700,
        .max_xattr = 17,
    };
    f->fse.fst.cfg.op_size = 4096;
}

static void fixture_teardown(BackendFixture *f, gconstpointer opaque)
{
    v9fs_backend_cleanup(&f->backend);
    fixture = NULL;
}

static void test_missing_fsdev(BackendFixture *f, gconstpointer opaque)
{
    Error *err = NULL;

    f->registered_id = NULL;
    g_assert_cmpint(v9fs_backend_init(&f->backend, "missing", &err), <, 0);
    g_assert_nonnull(err);
    g_assert_nonnull(strstr(error_get_pretty(err),
                            "couldn't find fsdev with the id = missing"));
    g_assert_false(f->backend.initialized);
    g_assert_cmpint(f->init_calls, ==, 0);
    g_assert_cmpint(f->cleanup_calls, ==, 0);
    error_free(err);
}

static void test_success_and_idempotent_cleanup(BackendFixture *f,
                                                gconstpointer opaque)
{
    Error *err = NULL;
    Error *again_err = NULL;

    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &err), ==, 0);
    g_assert_null(err);
    g_assert_true(f->backend.initialized);
    g_assert_true(f->backend.ops == &fake_ops);
    g_assert_cmpstr(f->backend.ctx.fs_root, ==, "/export");
    g_assert_cmpuint(f->backend.ctx.xattr_fid_limit, ==, 17);
    g_assert_cmpuint(f->backend.throttle.cfg.op_size, ==, 4096);
    g_assert_true(f->backend.ctx.fst == &f->backend.throttle);
    g_assert_cmpuint(f->backend.root_st.st_dev, ==, 42);
    g_assert_true(S_ISDIR(f->backend.root_st.st_mode));
    g_assert_nonnull(f->backend.qpd_table.map);
    g_assert_nonnull(f->backend.qpp_table.map);
    g_assert_nonnull(f->backend.qpf_table.map);
    g_assert_cmpint(f->init_calls, ==, 1);
    g_assert_cmpint(f->throttle_init_calls, ==, 1);

    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &again_err), <,
                    0);
    g_assert_nonnull(strstr(error_get_pretty(again_err),
                            "backend is already initialized"));
    g_assert_cmpint(f->init_calls, ==, 1);
    error_free(again_err);

    v9fs_backend_cleanup(&f->backend);
    g_assert_false(f->backend.initialized);
    g_assert_null(f->backend.ops);
    g_assert_null(f->backend.ctx.fs_root);
    g_assert_null(f->backend.qpd_table.map);
    g_assert_null(f->backend.qpp_table.map);
    g_assert_null(f->backend.qpf_table.map);
    g_assert_cmpint(f->cleanup_calls, ==, 1);
    g_assert_cmpint(f->throttle_cleanup_calls, ==, 1);
    error_free(err);

    v9fs_backend_cleanup(&f->backend);
    g_assert_cmpint(f->cleanup_calls, ==, 1);
    g_assert_cmpint(f->throttle_cleanup_calls, ==, 1);

    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &err), ==, 0);
    g_assert_null(err);
    v9fs_backend_cleanup(&f->backend);
    g_assert_cmpint(f->init_calls, ==, 2);
    g_assert_cmpint(f->cleanup_calls, ==, 2);
    g_assert_cmpint(f->throttle_init_calls, ==, 2);
    g_assert_cmpint(f->throttle_cleanup_calls, ==, 2);
}

static void test_init_failure_unwinds(BackendFixture *f, gconstpointer opaque)
{
    Error *err = NULL;

    f->init_result = -1;
    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &err), <, 0);
    g_assert_nonnull(
        strstr(error_get_pretty(err), "cannot initialize fsdev 'testfs'"));
    g_assert_cmpint(f->init_calls, ==, 1);
    g_assert_cmpint(f->cleanup_calls, ==, 1);
    g_assert_cmpint(f->throttle_init_calls, ==, 0);
    g_assert_false(f->backend.initialized);
    g_assert_null(f->backend.ctx.fs_root);
    error_free(err);
}

static void test_name_failure_unwinds(BackendFixture *f, gconstpointer opaque)
{
    Error *err = NULL;

    f->name_to_path_result = -1;
    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &err), <, 0);
    g_assert_nonnull(
        strstr(error_get_pretty(err), "error in converting name to path"));
    g_assert_cmpint(f->cleanup_calls, ==, 1);
    g_assert_cmpint(f->throttle_init_calls, ==, 0);
    g_assert_null(f->backend.ctx.fs_root);
    error_free(err);
}

static void test_missing_root_unwinds(BackendFixture *f, gconstpointer opaque)
{
    Error *err = NULL;

    f->lstat_result = -1;
    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &err), <, 0);
    g_assert_nonnull(
        strstr(error_get_pretty(err), "share path /export does not exist"));
    g_assert_cmpint(f->cleanup_calls, ==, 1);
    g_assert_cmpint(f->throttle_init_calls, ==, 0);
    error_free(err);
}

static void test_non_directory_root_unwinds(BackendFixture *f,
                                            gconstpointer opaque)
{
    Error *err = NULL;

    f->root_mode = S_IFREG | 0644;
    g_assert_cmpint(v9fs_backend_init(&f->backend, "testfs", &err), <, 0);
    g_assert_nonnull(
        strstr(error_get_pretty(err), "share path /export is not a directory"));
    g_assert_cmpint(f->cleanup_calls, ==, 1);
    g_assert_cmpint(f->throttle_init_calls, ==, 0);
    error_free(err);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add("/9p-backend/missing-fsdev", BackendFixture, NULL, fixture_setup,
               test_missing_fsdev, fixture_teardown);
    g_test_add("/9p-backend/success-cleanup", BackendFixture, NULL,
               fixture_setup, test_success_and_idempotent_cleanup,
               fixture_teardown);
    g_test_add("/9p-backend/init-failure", BackendFixture, NULL, fixture_setup,
               test_init_failure_unwinds, fixture_teardown);
    g_test_add("/9p-backend/name-failure", BackendFixture, NULL, fixture_setup,
               test_name_failure_unwinds, fixture_teardown);
    g_test_add("/9p-backend/missing-root", BackendFixture, NULL, fixture_setup,
               test_missing_root_unwinds, fixture_teardown);
    g_test_add("/9p-backend/non-directory", BackendFixture, NULL, fixture_setup,
               test_non_directory_root_unwinds, fixture_teardown);

    return g_test_run();
}
