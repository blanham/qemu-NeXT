/*
 * Plan 9 Second Edition 9P1 user-creatable object tests
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
                       "-fsdev local,id=testfs,path=%s,security_model=none "
                       "-netdev user,id=nextnet %s",
                       test_root, extra ?: "");
}

static void assert_qmp_error(QDict *response)
{
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "error"));
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
        "'path':'/objects/p9','property':'guest-address'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "10.0.2.100");
    qobject_unref(response);

    response = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/objects/p9','property':'port'}}");
    g_assert_cmpuint(qdict_get_int(response, "return"), ==, 564);
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
    /* This adds a distinct endpoint, then fails because BOOTP is owned. */
    object_add(qts, "bootpfail", "testfs", "nextnet", "10.0.2.101", 565,
               false);
    object_del(qts, "owner", true);
    /* The failed completion must have removed its endpoint. */
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

    qtest_add_func("plan9-9p1-object/cli-late", test_cli_late_creation);
    qtest_add_func("plan9-9p1-object/defaults-recreate",
                   test_defaults_and_recreate);
    qtest_add_func("plan9-9p1-object/validation-unwind",
                   test_validation_and_unwind);
    qtest_add_func("plan9-9p1-object/ipv4-disabled", test_ipv4_disabled);
    qtest_add_func("plan9-9p1-object/custom-network", test_custom_network);
    qtest_add_func("plan9-9p1-object/no-host-listener", test_no_host_listener);
    ret = g_test_run();

    g_rmdir(test_root);
    g_free(test_root);
    return ret;
}
