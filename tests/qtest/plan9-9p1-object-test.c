/*
 * Plan 9 Second Edition 9P1 user-creatable object tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

static char *test_root;
static char *keydb_path;
static char *secret_path;

static const uint8_t keydb_record[] = {
    0x75, 0x6f, 0x1a, 0x43, 0x45, 0x3a, 0x6d, 0xb7, 0xf1, 0x32,
    0x56, 0x77, 0x3c, 0x36, 0x05, 0xb9, 0x68, 0xaf, 0xca, 0xd1,
    0x57, 0x13, 0xa7, 0x3c, 0x8f, 0xab, 0xcf, 0x37, 0xd7, 0xd2,
    0x12, 0xeb, 0x03, 0xbd, 0x8d, 0x02, 0xe1, 0x88, 0x1c, 0x9f,
    0x2b,
};

static const uint8_t master_key[] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd,
};

static QTestState *start_vm(const char *extra)
{
    return qtest_initf("-machine none -nodefaults "
                       "-fsdev local,id=testfs,path=%s,security_model=none "
                       "-netdev user,id=nextnet %s",
                       test_root, extra ?: "");
}

#ifdef CONFIG_SLIRP_IL
static QTestState *start_vm_with_secret(const char *path)
{
    return qtest_initf("-machine none -nodefaults "
                       "-fsdev local,id=testfs,path=%s,security_model=none "
                       "-netdev user,id=nextnet "
                       "-object secret,id=p9key,file=%s",
                       test_root, path);
}
#endif

static void assert_qmp_error(QDict *response)
{
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "error"));
    qobject_unref(response);
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

static void object_add(QTestState *qts, const char *id,
                       const char *fsdev, const char *netdev,
                       const char *address, unsigned int port,
                       bool success)
{
    QDict *response;

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':%s,"
        "'fsdev':%s,'netdev':%s,'guest-address':%s,'port':%u}}",
        id, fsdev, netdev, address, port);
    if (success) {
        g_assert_true(qdict_haskey(response, "return"));
        qobject_unref(response);
    } else {
        assert_qmp_error(response);
    }
}

static void object_del(QTestState *qts, const char *id, bool success)
{
    QDict *response = qtest_qmp(qts,
        "{'execute':'object-del','arguments':{'id':%s}}", id);

    if (success) {
        g_assert_true(qdict_haskey(response, "return"));
        qobject_unref(response);
    } else {
        assert_qmp_error(response);
    }
}

static void test_cli_late_creation(void)
{
    QTestState *qts = start_vm(
        "-object plan9-9p1-server,id=p9,fsdev=testfs,netdev=nextnet");

    qtest_quit(qts);
}

static void test_defaults_and_recreate(void)
{
    QTestState *qts = start_vm("");
    QDict *response;

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'p9',"
        "'fsdev':'testfs','netdev':'nextnet'}}");
    g_assert_true(qdict_haskey(response, "return"));
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'transport'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "tcp");
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'guest-address'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "10.0.2.100");
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'port'}}");
    g_assert_cmpuint(qdict_get_int(response, "return"), ==, 564);
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'il-port'}}");
    g_assert_cmpuint(qdict_get_int(response, "return"), ==, 17008);
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'auth-port'}}");
    g_assert_cmpuint(qdict_get_int(response, "return"), ==, 566);
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-set','arguments':{"
        "'path':'/objects/p9','property':'port','value':565}}");
    assert_qmp_error(response);

    object_del(qts, "p9", true);
    object_add(qts, "p9", "testfs", "nextnet", "10.0.2.100", 564, true);
    object_del(qts, "p9", true);
    qtest_quit(qts);
}

static void test_il_requires_complete_auth(void)
{
    QTestState *qts = start_vm("");
    QDict *response;

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'p9',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il'}}");
    assert_qmp_error_contains(response, "requires auth-id");

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'missing-id',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-domain':'lab','keydb':%s,'key-secret':'p9key'}}",
        keydb_path);
    assert_qmp_error_contains(response, "requires auth-id");
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'missing-domain',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-id':'p9fs','keydb':%s,'key-secret':'p9key'}}",
        keydb_path);
    assert_qmp_error_contains(response, "requires auth-id");
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'missing-keydb',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-id':'p9fs','auth-domain':'lab','key-secret':'p9key'}}");
    assert_qmp_error_contains(response, "requires auth-id");
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'missing-secret-id',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-id':'p9fs','auth-domain':'lab','keydb':%s}}",
        keydb_path);
    assert_qmp_error_contains(response, "requires auth-id");

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'tcp-auth',"
        "'fsdev':'testfs','netdev':'nextnet','auth-id':'p9fs'}}");
    assert_qmp_error(response);
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'tcp-domain',"
        "'fsdev':'testfs','netdev':'nextnet','auth-domain':'lab'}}");
    assert_qmp_error_contains(response, "transport=il");
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'tcp-keydb',"
        "'fsdev':'testfs','netdev':'nextnet','keydb':%s}}", keydb_path);
    assert_qmp_error_contains(response, "transport=il");
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'tcp-secret',"
        "'fsdev':'testfs','netdev':'nextnet','key-secret':'p9key'}}");
    assert_qmp_error_contains(response, "transport=il");
    qtest_quit(qts);
}

#ifndef CONFIG_SLIRP_IL
static void test_il_unavailable_precedes_resources(void)
{
    QTestState *qts = start_vm("");
    QDict *response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'p9',"
        "'fsdev':'missing','netdev':'nextnet','transport':'il',"
        "'auth-id':'p9fs','auth-domain':'lab',"
        "'keydb':'/definitely/missing','key-secret':'missing'}}");

    assert_qmp_error_contains(response, "unavailable");
    qtest_quit(qts);
}
#endif

static void test_property_validation(void)
{
    QTestState *qts = start_vm("");
    QDict *response;
    g_autofree char *long_id = g_strnfill(28, 'i');
    g_autofree char *long_domain = g_strnfill(48, 'd');

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'bad-transport',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'udp'}}");
    assert_qmp_error(response);
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'bad-id',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-id':%s}}", long_id);
    assert_qmp_error(response);
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'bad-domain',"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-domain':%s}}", long_domain);
    assert_qmp_error(response);
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'zero-il',"
        "'fsdev':'testfs','netdev':'nextnet','il-port':0}}");
    assert_qmp_error(response);
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'zero-auth',"
        "'fsdev':'testfs','netdev':'nextnet','auth-port':0}}");
    assert_qmp_error(response);
    qtest_quit(qts);
}

#ifdef CONFIG_SLIRP_IL
static QDict *object_add_il_response(QTestState *qts, const char *id,
                                     unsigned int il_port,
                                     unsigned int auth_port,
                                     const char *keydb,
                                     const char *secret_id)
{
    return qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':%s,"
        "'fsdev':'testfs','netdev':'nextnet','transport':'il',"
        "'auth-id':'p9fs','auth-domain':'lab',"
        "'keydb':%s,'key-secret':%s,"
        "'il-port':%u,'auth-port':%u}}",
        id, keydb, secret_id, il_port, auth_port);
}

static void object_add_il(QTestState *qts, const char *id,
                          unsigned int il_port, unsigned int auth_port,
                          const char *keydb, const char *secret_id,
                          bool success)
{
    QDict *response = object_add_il_response(qts, id, il_port, auth_port,
                                             keydb, secret_id);

    if (success) {
        g_assert_true(qdict_haskey(response, "return"));
        qobject_unref(response);
    } else {
        assert_qmp_error(response);
    }
}

static void test_il_secret_keydb_and_unwind(void)
{
    QTestState *qts;
    QDict *response;
    g_autofree char *empty_secret = g_build_filename(test_root, "empty", NULL);
    g_autofree char *short_secret = g_build_filename(test_root, "short", NULL);
    g_autofree char *long_secret = g_build_filename(test_root, "long", NULL);
    g_autofree char *nul_secret = g_build_filename(test_root, "nul", NULL);
    g_autofree char *bad_keydb = g_build_filename(test_root, "badkeys", NULL);
    static const uint8_t short_key[6] = { 1, 2, 3, 4, 5, 6 };
    static const uint8_t long_key[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    static const uint8_t nul_key[7] = { 1, 2, 0, 4, 5, 6, 7 };

    g_assert_true(g_file_set_contents(empty_secret, "", 0, NULL));
    qts = start_vm("");
    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'secret','id':'p9key','file':%s}}", empty_secret);
    if (qdict_haskey(response, "return")) {
        qobject_unref(response);
        object_add_il(qts, "empty", 17008, 566, keydb_path, "p9key",
                      false);
    } else {
        assert_qmp_error(response);
    }
    qtest_quit(qts);

    g_assert_true(g_file_set_contents(short_secret, (const char *)short_key,
                                      sizeof(short_key), NULL));
    qts = start_vm_with_secret(short_secret);
    object_add_il(qts, "short", 17008, 566, keydb_path, "p9key", false);
    qtest_quit(qts);

    g_assert_true(g_file_set_contents(long_secret, (const char *)long_key,
                                      sizeof(long_key), NULL));
    qts = start_vm_with_secret(long_secret);
    object_add_il(qts, "long", 17008, 566, keydb_path, "p9key", false);
    qtest_quit(qts);

    g_assert_true(g_file_set_contents(nul_secret, (const char *)nul_key,
                                      sizeof(nul_key), NULL));
    qts = start_vm_with_secret(nul_secret);
    response = object_add_il_response(qts, "nul", 17008, 566,
                                      keydb_path, "p9key");
    g_assert_nonnull(qdict_get_qdict(response, "error"));
    g_assert_null(strstr(qdict_get_str(qdict_get_qdict(response, "error"),
                                      "desc"), "exactly"));
    qobject_unref(response);
    qtest_quit(qts);

    g_assert_true(g_file_set_contents(bad_keydb, "", 0, NULL));
    qts = start_vm_with_secret(secret_path);
    object_add_il(qts, "bad-keydb", 17008, 566, bad_keydb, "p9key", false);
    object_add_il(qts, "missing-secret", 17008, 566, keydb_path,
                  "missing", false);
    object_add_il(qts, "collision", 17008, 17008, keydb_path, "p9key",
                  false);
    object_add_il(qts, "after-collision", 17008, 566, keydb_path, "p9key",
                  true);
    object_del(qts, "after-collision", true);
    qtest_quit(qts);

    unlink(empty_secret);
    unlink(short_secret);
    unlink(long_secret);
    unlink(nul_secret);
    unlink(bad_keydb);
}

static void test_il_defaults_and_recreate(void)
{
    QTestState *qts = start_vm_with_secret(secret_path);
    QDict *response;

    object_add_il(qts, "p9", 17008, 566, keydb_path, "p9key", true);
    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'transport'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "il");
    qobject_unref(response);
    object_add_il(qts, "bootp-owner", 17009, 567, keydb_path, "p9key",
                  false);
    object_del(qts, "p9", true);
    object_add_il(qts, "p9", 17008, 566, keydb_path, "p9key", true);
    object_del(qts, "p9", true);
    qtest_quit(qts);
}
#endif

static void test_validation_and_unwind(void)
{
    QTestState *qts = start_vm("-netdev socket,id=socketnet,listen=:0");
    QDict *response;

    response = qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'plan9-9p1-server','id':'missing'}}");
    assert_qmp_error(response);

    object_add(qts, "badfs", "missing", "nextnet", "10.0.2.100", 564,
               false);
    object_add(qts, "badnet", "testfs", "missing", "10.0.2.100", 564,
               false);
    object_add(qts, "notuser", "testfs", "socketnet", "10.0.2.100", 564,
               false);
    object_add(qts, "badaddr", "testfs", "nextnet", "not-an-address", 564,
               false);
    object_add(qts, "outside", "testfs", "nextnet", "192.0.2.1", 564,
               false);
    object_add(qts, "network", "testfs", "nextnet", "10.0.2.0", 564,
               false);
    object_add(qts, "host", "testfs", "nextnet", "10.0.2.2", 564,
               false);
    object_add(qts, "dns", "testfs", "nextnet", "10.0.2.3", 564,
               false);
    object_add(qts, "broadcast", "testfs", "nextnet", "10.0.2.255", 564,
               false);
    object_add(qts, "zero", "testfs", "nextnet", "10.0.2.100", 0, false);

    object_add(qts, "owner", "testfs", "nextnet", "10.0.2.100", 564,
               true);
    object_add(qts, "duplicate", "testfs", "nextnet", "10.0.2.100", 564,
               false);
#ifdef CONFIG_SLIRP_PLAN9_BOOTP
    /* This adds a distinct endpoint, then fails because BOOTP is owned. */
    object_add(qts, "bootpfail", "testfs", "nextnet", "10.0.2.101", 565,
               false);
#else
    /*
     * Older system libslirp lacks Plan 9 BOOTP. TCP remains usable without
     * the vendor option, so there is no BOOTP resource to collide here.
     */
    object_add(qts, "bootpfail", "testfs", "nextnet", "10.0.2.101", 565,
               true);
    object_del(qts, "bootpfail", true);
#endif
    object_del(qts, "owner", true);
    /* The failed or explicitly removed object must have freed its endpoint. */
    object_add(qts, "afterunwind", "testfs", "nextnet", "10.0.2.101", 565,
               true);
    object_del(qts, "afterunwind", true);
    qtest_quit(qts);
}

static void test_ipv4_disabled(void)
{
    QTestState *qts = qtest_initf(
        "-machine none -nodefaults "
        "-fsdev local,id=testfs,path=%s,security_model=none "
        "-netdev user,id=nextnet,ipv4=off,ipv6=on",
        test_root);

    object_add(qts, "p9", "testfs", "nextnet", "10.0.2.100", 564, false);
    qtest_quit(qts);
}

static void test_custom_network(void)
{
    QTestState *qts = qtest_initf(
        "-machine none -nodefaults "
        "-fsdev local,id=testfs,path=%s,security_model=none "
        "-netdev user,id=nextnet,net=172.20.16.0/20",
        test_root);

    /* Success proves BOOTP used this stack's /20 mask and gateway. */
    object_add(qts, "p9", "testfs", "nextnet", "172.20.16.100", 564,
               true);
    object_del(qts, "p9", true);
    qtest_quit(qts);
}

static void test_no_host_listener(void)
{
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    socklen_t address_len = sizeof(address);
    QTestState *qts;
    int listener;

    listener = socket(AF_INET, SOCK_STREAM, 0);
    g_assert_cmpint(listener, >=, 0);
    g_assert_cmpint(bind(listener, (struct sockaddr *)&address,
                         sizeof(address)), ==, 0);
    g_assert_cmpint(getsockname(listener, (struct sockaddr *)&address,
                               &address_len), ==, 0);
    g_assert_cmpint(listen(listener, 1), ==, 0);

    qts = start_vm("");
    object_add(qts, "p9", "testfs", "nextnet", "10.0.2.100",
               ntohs(address.sin_port), true);
    object_del(qts, "p9", true);
    qtest_quit(qts);
    close(listener);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    test_root = g_dir_make_tmp("qemu-9p1-object-XXXXXX", NULL);
    g_assert_nonnull(test_root);
    keydb_path = g_build_filename(test_root, "keys", NULL);
    secret_path = g_build_filename(test_root, "master", NULL);
    g_assert_true(g_file_set_contents(keydb_path, (const char *)keydb_record,
                                      sizeof(keydb_record), NULL));
    g_assert_true(g_file_set_contents(secret_path, (const char *)master_key,
                                      sizeof(master_key), NULL));

    qtest_add_func("plan9-9p1-object/cli-late", test_cli_late_creation);
    qtest_add_func("plan9-9p1-object/defaults-recreate",
                   test_defaults_and_recreate);
    qtest_add_func("plan9-9p1-object/validation-unwind",
                   test_validation_and_unwind);
    qtest_add_func("plan9-9p1-object/il-requires-complete-auth",
                   test_il_requires_complete_auth);
#ifndef CONFIG_SLIRP_IL
    qtest_add_func("plan9-9p1-object/il-unavailable-precedes-resources",
                   test_il_unavailable_precedes_resources);
#endif
    qtest_add_func("plan9-9p1-object/property-validation",
                   test_property_validation);
#ifdef CONFIG_SLIRP_IL
    qtest_add_func("plan9-9p1-object/il-secret-keydb-unwind",
                   test_il_secret_keydb_and_unwind);
    qtest_add_func("plan9-9p1-object/il-defaults-recreate",
                   test_il_defaults_and_recreate);
#endif
    qtest_add_func("plan9-9p1-object/ipv4-disabled", test_ipv4_disabled);
    qtest_add_func("plan9-9p1-object/custom-network", test_custom_network);
    qtest_add_func("plan9-9p1-object/no-host-listener", test_no_host_listener);
    ret = g_test_run();

    unlink(secret_path);
    unlink(keydb_path);
    g_free(secret_path);
    g_free(keydb_path);
    g_rmdir(test_root);
    g_free(test_root);
    return ret;
}
