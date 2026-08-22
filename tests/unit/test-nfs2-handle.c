#include "qemu/osdep.h"

#include "hw/nfs/nfs2-handle.h"
#include "qapi/error.h"

static const uint8_t key[32] = { 0, 1, 2, 3, 4, 5, 6, 7 };

static Nfs2HandleTable *new_table(void)
{
    return nfs2_handle_table_new(key, sizeof(key), &error_abort);
}

static void assert_resolves(Nfs2HandleTable *table,
                            const Nfs2FileHandle *handle,
                            const char *expected)
{
    V9fsPath path = { 0 };

    g_assert_true(nfs2_handle_resolve(table, handle, &path));
    g_assert_cmpstr(path.data, ==, expected);
    g_assert_cmpuint(path.size, ==, strlen(expected) + 1);
    g_free(path.data);
}

static void test_exact_vector_and_resolution(void)
{
    static const uint8_t expected[32] = {
        'Q', 'N', '2', 'F', 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
        0x37, 0x91, 0xc3, 0x69, 0x30, 0x3f,
        0x83, 0xaf, 0x47, 0x3e, 0xd0, 0xc7,
    };
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle root;

    g_assert_true(nfs2_handle_create(table, 1, "/", &root, &error_abort));
    g_assert_cmpmem(root.bytes, sizeof(root.bytes), expected, sizeof(expected));
    assert_resolves(table, &root, "/");
}

static void test_forgery_and_version_rejected(void)
{
    static const Nfs2FileHandle version_two = { .bytes = {
        'Q', 'N', '2', 'F', 0, 0, 0, 2,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
        0x2a, 0xc9, 0xd9, 0xe9, 0xba, 0x7c,
        0x83, 0xc9, 0xf1, 0x8a, 0x92, 0xb2,
    } };
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle valid;
    Nfs2FileHandle forged;
    V9fsPath path = { 0 };

    g_assert_true(nfs2_handle_create(table, 1, "/", &valid, &error_abort));
    forged = valid;
    forged.bytes[31] ^= 1;
    g_assert_false(nfs2_handle_resolve(table, &forged, &path));
    g_assert_false(nfs2_handle_resolve(table, &version_two, &path));
}

static void test_generation_and_removal(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle old_handle;
    Nfs2FileHandle new_handle;
    V9fsPath path = { 0 };

    g_assert_true(nfs2_handle_create(table, 7, "/gone", &old_handle,
                                     &error_abort));
    g_assert_true(nfs2_handle_remove(table, "/gone", &error_abort));
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 0);
    g_assert_false(nfs2_handle_resolve(table, &old_handle, &path));

    g_assert_true(nfs2_handle_create(table, 7, "/replacement", &new_handle,
                                     &error_abort));
    g_assert_cmpint(memcmp(old_handle.bytes, new_handle.bytes,
                           sizeof(old_handle.bytes)), !=, 0);
    g_assert_cmpuint(old_handle.bytes[19], ==, 1);
    g_assert_cmpuint(new_handle.bytes[19], ==, 2);
    assert_resolves(table, &new_handle, "/replacement");
}

static void test_path_and_record_bounds(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    g_autofree char *maximum = g_malloc(NFS2_MAX_PATH + 1);
    g_autofree char *too_long = g_malloc(NFS2_MAX_PATH + 2);
    Nfs2FileHandle handle;
    Error *error = NULL;

    maximum[0] = '/';
    memset(maximum + 1, 'a', NFS2_MAX_PATH - 1);
    maximum[NFS2_MAX_PATH] = 0;
    too_long[0] = '/';
    memset(too_long + 1, 'b', NFS2_MAX_PATH);
    too_long[NFS2_MAX_PATH + 1] = 0;

    g_assert_true(nfs2_handle_create(table, 1, maximum, &handle,
                                     &error_abort));
    g_assert_false(nfs2_handle_create(table, 2, too_long, &handle, &error));
    g_assert_nonnull(error);
    error_free(error);

    for (uint64_t id = 2; id <= NFS2_MAX_HANDLE_RECORDS; id++) {
        g_autofree char *path = g_strdup_printf("/file/%" PRIu64, id);

        g_assert_true(nfs2_handle_create(table, id, path, &handle,
                                         &error_abort));
    }
    error = NULL;
    g_assert_false(nfs2_handle_create(table, NFS2_MAX_HANDLE_RECORDS + 1,
                                      "/overflow", &handle, &error));
    g_assert_nonnull(error);
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==,
                     NFS2_MAX_HANDLE_RECORDS);
    error_free(error);
}

static void test_alias_bound(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle handle;
    Error *error = NULL;

    for (uint64_t alias = 0; alias < NFS2_MAX_HANDLE_RECORDS; alias++) {
        g_autofree char *path = g_strdup_printf("/alias/%" PRIu64, alias);

        g_assert_true(nfs2_handle_create(table, 1, path, &handle,
                                         &error_abort));
    }
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 1);
    g_assert_false(nfs2_handle_create(table, 1, "/alias/overflow", &handle,
                                      &error));
    g_assert_nonnull(error);
    error_free(error);
}

static void test_sustained_id_churn(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle handle;

    for (uint64_t id = 1; id <= NFS2_MAX_HANDLE_RECORDS + 1024; id++) {
        g_assert_true(nfs2_handle_create(table, id, "/churn", &handle,
                                         &error_abort));
        g_assert_true(nfs2_handle_remove(table, "/churn", &error_abort));
        g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 0);
    }
}

static void test_generation_exhaustion(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle last;
    Nfs2FileHandle alias;
    Nfs2FileHandle rejected;
    V9fsPath path = { 0 };
    Error *error = NULL;

    nfs2_handle_table_set_next_generation_for_test(table, UINT32_MAX);
    g_assert_true(nfs2_handle_create(table, 1, "/last", &last,
                                     &error_abort));
    g_assert_cmpuint(last.bytes[16], ==, 0xff);
    g_assert_cmpuint(last.bytes[17], ==, 0xff);
    g_assert_cmpuint(last.bytes[18], ==, 0xff);
    g_assert_cmpuint(last.bytes[19], ==, 0xff);
    g_assert_true(nfs2_handle_create(table, 1, "/last-alias", &alias,
                                     &error_abort));
    g_assert_cmpmem(last.bytes, sizeof(last.bytes),
                    alias.bytes, sizeof(alias.bytes));

    g_assert_true(nfs2_handle_remove(table, "/last", &error_abort));
    assert_resolves(table, &last, "/last-alias");
    g_assert_true(nfs2_handle_remove(table, "/last-alias", &error_abort));
    g_assert_false(nfs2_handle_resolve(table, &last, &path));
    g_assert_false(nfs2_handle_create(table, 2, "/rejected", &rejected,
                                      &error));
    g_assert_nonnull(error);
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 0);
    error_free(error);
}

static void test_hmac_failure_is_unpublished(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle handle;
    Error *error = NULL;

    nfs2_handle_table_fail_next_mac_for_test(table);
    g_assert_false(nfs2_handle_create(table, 1, "/failed", &handle, &error));
    g_assert_nonnull(error);
    error_free(error);
    error = NULL;
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 0);

    g_assert_true(nfs2_handle_create(table, 2, "/success", &handle,
                                     &error_abort));
    g_assert_cmpuint(handle.bytes[16], ==, 0);
    g_assert_cmpuint(handle.bytes[17], ==, 0);
    g_assert_cmpuint(handle.bytes[18], ==, 0);
    g_assert_cmpuint(handle.bytes[19], ==, 1);

    nfs2_handle_table_fail_next_mac_for_test(table);
    g_assert_false(nfs2_handle_create(table, 2, "/failed-alias", &handle,
                                      &error));
    g_assert_nonnull(error);
    error_free(error);
    g_assert_true(nfs2_handle_remove(table, "/success", &error_abort));
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 0);
}

static void test_rename_prefix_is_atomic(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    g_autofree char *long_prefix = g_malloc(NFS2_MAX_PATH + 1);
    Nfs2FileHandle top;
    Nfs2FileHandle child;
    Nfs2FileHandle other;
    Nfs2FileHandle destination;
    Error *error = NULL;

    g_assert_true(nfs2_handle_create(table, 1, "/old", &top, &error_abort));
    g_assert_true(nfs2_handle_create(table, 2, "/old/child", &child,
                                     &error_abort));
    g_assert_true(nfs2_handle_create(table, 3, "/older", &other,
                                     &error_abort));

    g_assert_true(nfs2_handle_rename(table, "/old", "/new", &error_abort));
    assert_resolves(table, &top, "/new");
    assert_resolves(table, &child, "/new/child");
    assert_resolves(table, &other, "/older");

    long_prefix[0] = '/';
    memset(long_prefix + 1, 'x', NFS2_MAX_PATH - 1);
    long_prefix[NFS2_MAX_PATH] = 0;
    g_assert_true(nfs2_handle_create(table, 4, long_prefix, &destination,
                                     &error_abort));
    g_assert_false(nfs2_handle_rename(table, "/new", long_prefix, &error));
    g_assert_nonnull(error);
    error_free(error);
    assert_resolves(table, &top, "/new");
    assert_resolves(table, &child, "/new/child");
    assert_resolves(table, &destination, long_prefix);
}

static void test_rename_replaces_destination(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle source;
    Nfs2FileHandle source_child;
    Nfs2FileHandle destination;
    Nfs2FileHandle destination_child;
    V9fsPath path = { 0 };

    g_assert_true(nfs2_handle_create(table, 1, "/source", &source,
                                     &error_abort));
    g_assert_true(nfs2_handle_create(table, 2, "/source/child",
                                     &source_child, &error_abort));
    g_assert_true(nfs2_handle_create(table, 3, "/destination", &destination,
                                     &error_abort));
    g_assert_true(nfs2_handle_create(table, 4, "/destination/child",
                                     &destination_child, &error_abort));

    g_assert_true(nfs2_handle_rename(table, "/source", "/destination",
                                     &error_abort));
    assert_resolves(table, &source, "/destination");
    assert_resolves(table, &source_child, "/destination/child");
    g_assert_false(nfs2_handle_resolve(table, &destination, &path));
    g_assert_false(nfs2_handle_resolve(table, &destination_child, &path));
    g_assert_cmpuint(nfs2_handle_table_record_count(table), ==, 2);
}

static void test_rename_same_record_is_noop(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle handle;
    Nfs2FileHandle alias;

    g_assert_true(nfs2_handle_create(table, 1, "/source", &handle,
                                     &error_abort));
    g_assert_true(nfs2_handle_create(table, 1, "/destination", &alias,
                                     &error_abort));
    g_assert_true(nfs2_handle_rename(table, "/source", "/destination",
                                     &error_abort));
    g_assert_true(nfs2_handle_remove(table, "/source", &error_abort));
    assert_resolves(table, &handle, "/destination");
}

static void test_hard_link_alias_fallback(void)
{
    g_autoptr(Nfs2HandleTable) table = new_table();
    Nfs2FileHandle primary;
    Nfs2FileHandle alias;
    V9fsPath path = { 0 };

    g_assert_true(nfs2_handle_create(table, 9, "/primary", &primary,
                                     &error_abort));
    g_assert_true(nfs2_handle_create(table, 9, "/alias", &alias,
                                     &error_abort));
    g_assert_cmpmem(primary.bytes, sizeof(primary.bytes),
                    alias.bytes, sizeof(alias.bytes));

    g_assert_true(nfs2_handle_remove(table, "/primary", &error_abort));
    assert_resolves(table, &primary, "/alias");
    g_assert_true(nfs2_handle_remove(table, "/alias", &error_abort));
    g_assert_false(nfs2_handle_resolve(table, &primary, &path));
}

static void test_random_generation_and_clear(void)
{
    g_autoptr(Nfs2HandleTable) first =
        nfs2_handle_table_new(NULL, 0, &error_abort);
    g_autoptr(Nfs2HandleTable) second =
        nfs2_handle_table_new(NULL, 0, &error_abort);
    Nfs2FileHandle one;
    Nfs2FileHandle two;
    V9fsPath path = { 0 };

    g_assert_true(nfs2_handle_create(first, 1, "/", &one, &error_abort));
    g_assert_true(nfs2_handle_create(second, 1, "/", &two, &error_abort));
    g_assert_cmpint(memcmp(one.bytes, two.bytes, sizeof(one.bytes)), !=, 0);

    nfs2_handle_table_clear(first);
    g_assert_cmpuint(nfs2_handle_table_record_count(first), ==, 0);
    g_assert_false(nfs2_handle_resolve(first, &one, &path));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/nfs2-handle/exact-vector",
                    test_exact_vector_and_resolution);
    g_test_add_func("/nfs2-handle/reject-forgery-version",
                    test_forgery_and_version_rejected);
    g_test_add_func("/nfs2-handle/generation-removal",
                    test_generation_and_removal);
    g_test_add_func("/nfs2-handle/bounds", test_path_and_record_bounds);
    g_test_add_func("/nfs2-handle/alias-bound", test_alias_bound);
    g_test_add_func("/nfs2-handle/id-churn", test_sustained_id_churn);
    g_test_add_func("/nfs2-handle/generation-exhaustion",
                    test_generation_exhaustion);
    g_test_add_func("/nfs2-handle/hmac-failure-unpublished",
                    test_hmac_failure_is_unpublished);
    g_test_add_func("/nfs2-handle/rename-prefix", test_rename_prefix_is_atomic);
    g_test_add_func("/nfs2-handle/rename-replacement",
                    test_rename_replaces_destination);
    g_test_add_func("/nfs2-handle/rename-same-record",
                    test_rename_same_record_is_noop);
    g_test_add_func("/nfs2-handle/hard-link", test_hard_link_alias_fallback);
    g_test_add_func("/nfs2-handle/random-clear",
                    test_random_generation_and_clear);

    return g_test_run();
}
