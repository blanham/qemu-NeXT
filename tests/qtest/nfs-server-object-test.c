/*
 * Guest-only NFS server user-creatable object tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

static char *test_root;

static QTestState *start_vm(const char *extra)
{
    return qtest_initf("-machine none -nodefaults "
                       "-fsdev local,id=root,path=%s,"
                       "security_model=mapped-xattr "
                       "-netdev user,id=nextnet %s",
                       test_root, extra ?: "");
}

static void assert_qmp_error_contains(QDict *response, const char *needle)
{
    QDict *error;

    g_assert_nonnull(response);
    error = qdict_get_qdict(response, "error");
    g_assert_nonnull(error);
    g_assert_nonnull(strstr(qdict_get_str(error, "desc"), needle));
    qobject_unref(response);
}

static void assert_qmp_success(QDict *response)
{
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    qobject_unref(response);
}

static QDict *object_add_response(QTestState *qts, const char *id,
                                  const char *fsdev, const char *netdev,
                                  const char *root_path, bool writable)
{
    return qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'nfs-server','id':%s,'fsdev':%s,'netdev':%s,"
        "'root-path':%s,'writable':%i}}",
        id, fsdev, netdev, root_path, writable);
}

static void object_add(QTestState *qts, const char *id, bool success)
{
    QDict *response = object_add_response(qts, id, "root", "nextnet", "/",
                                          false);

    if (success) {
        assert_qmp_success(response);
    } else {
        g_assert_true(qdict_haskey(response, "error"));
        qobject_unref(response);
    }
}

static void object_del(QTestState *qts, const char *id)
{
    assert_qmp_success(qtest_qmp(qts,
        "{'execute':'object-del','arguments':{'id':%s}}", id));
}

static void test_cli_creation(void)
{
    QTestState *qts = start_vm(
        "-object nfs-server,id=nfs,fsdev=root,netdev=nextnet,writable=on");

    qtest_quit(qts);
}

static void test_defaults_delete_recreate_reset(void)
{
    QTestState *qts = start_vm("");
    QDict *response;

    object_add(qts, "nfs", true);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/nfs','property':'root-path'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "/");
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/nfs','property':'writable'}}");
    g_assert_false(qdict_get_bool(response, "return"));
    qobject_unref(response);
    response = qtest_qmp(qts,
        "{'execute':'qom-set','arguments':{"
        "'path':'/objects/nfs','property':'root-path','value':'/changed'}}");
    assert_qmp_error_contains(response, "cannot change");
    response = qtest_qmp(qts,
        "{'execute':'migrate','arguments':{'uri':'file:/dev/null'}}");
    assert_qmp_error_contains(response, "not migratable");
    qtest_system_reset(qts);
    object_del(qts, "nfs");
    object_add(qts, "nfs", true);
    object_del(qts, "nfs");
    qtest_quit(qts);
}

static void test_validation_and_unwind(void)
{
    QTestState *qts = start_vm("-netdev socket,id=socketnet,listen=:0");
    g_autofree char *max_root = g_strnfill(255, 'a');
    g_autofree char *too_long_root = g_strnfill(256, 'a');
    QDict *response;

    max_root[0] = '/';
    too_long_root[0] = '/';

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'nfs-server','id':'missing'}}");
    assert_qmp_error_contains(response, "fsdev");
    response = object_add_response(qts, "badfs", "missing", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "fsdev");
    response = object_add_response(qts, "badnet", "root", "missing", "/",
                                   false);
    assert_qmp_error_contains(response, "netdev");
    response = object_add_response(qts, "notuser", "root", "socketnet", "/",
                                   false);
    assert_qmp_error_contains(response, "user-mode");
    response = object_add_response(qts, "relative", "root", "nextnet",
                                   "relative", false);
    assert_qmp_error_contains(response, "absolute");
    response = object_add_response(qts, "empty", "root", "nextnet", "",
                                   false);
    assert_qmp_error_contains(response, "absolute");
    response = object_add_response(qts, "too-long", "root", "nextnet",
                                   too_long_root, false);
    assert_qmp_error_contains(response, "1-255");
    response = object_add_response(qts, "unsupported-root", "root", "nextnet",
                                   "/export", false);
    assert_qmp_error_contains(response, "must be /");
    response = object_add_response(qts, "max-root", "root", "nextnet",
                                   max_root, false);
    assert_qmp_error_contains(response, "must be /");

    object_add(qts, "owner", true);
    response = object_add_response(qts, "collision", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "owns the BOOTP root lease");
    object_del(qts, "owner");
    object_add(qts, "after-unwind", true);
    object_del(qts, "after-unwind");
    qtest_quit(qts);
}

static void test_readonly_and_ipv4_disabled(void)
{
    QTestState *qts = qtest_initf(
        "-machine none -nodefaults "
        "-fsdev local,id=readonly,path=%s,security_model=mapped-xattr,"
        "readonly=on -netdev user,id=nextnet",
        test_root);
    QDict *response = object_add_response(qts, "nfs", "readonly", "nextnet",
                                          "/", true);

    assert_qmp_error_contains(response, "writable fsdev");
    qtest_quit(qts);

    qts = qtest_initf(
        "-machine none -nodefaults "
        "-fsdev local,id=root,path=%s,security_model=mapped-xattr "
        "-netdev user,id=nextnet,ipv4=off,ipv6=on",
        test_root);
    response = object_add_response(qts, "nfs", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "IPv4 is disabled");
    qtest_quit(qts);
}

static void test_migration_blocker_unwind(void)
{
    QTestState *qts = start_vm("-only-migratable");
    QDict *response;

    response = object_add_response(qts, "first", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "only-migratable");
    response = object_add_response(qts, "second", "root", "nextnet", "/",
                                   false);
    assert_qmp_error_contains(response, "only-migratable");
    qtest_quit(qts);
}

static void test_no_host_listener(void)
{
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(2049),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    QTestState *qts;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(bind(fd, (struct sockaddr *)&address, sizeof(address)),
                    ==, 0);
    qts = start_vm("");
    object_add(qts, "nfs", true);
    object_del(qts, "nfs");
    qtest_quit(qts);
    close(fd);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    test_root = g_dir_make_tmp("qemu-nfs-object-XXXXXX", NULL);
    g_assert_nonnull(test_root);

    qtest_add_func("nfs-server-object/cli-creation", test_cli_creation);
    qtest_add_func("nfs-server-object/defaults-delete-recreate-reset",
                   test_defaults_delete_recreate_reset);
    qtest_add_func("nfs-server-object/validation-unwind",
                   test_validation_and_unwind);
    qtest_add_func("nfs-server-object/readonly-ipv4-disabled",
                   test_readonly_and_ipv4_disabled);
    qtest_add_func("nfs-server-object/migration-blocker-unwind",
                   test_migration_blocker_unwind);
    qtest_add_func("nfs-server-object/no-host-listener",
                   test_no_host_listener);
    ret = g_test_run();

    g_rmdir(test_root);
    g_free(test_root);
    return ret;
}
