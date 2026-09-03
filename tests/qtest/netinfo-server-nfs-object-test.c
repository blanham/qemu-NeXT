/*
 * NetInfo and NFS server user-creatable object coexistence test
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

static char *test_root;

static QTestState *start_vm(void)
{
    return qtest_initf("-machine none -nodefaults "
                       "-fsdev local,id=root,path=%s,"
                       "security_model=mapped-xattr "
                       "-netdev user,id=nextnet", test_root);
}

static void assert_qmp_success(QDict *response)
{
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    qobject_unref(response);
}

static void object_add(QTestState *qts, const char *id)
{
    assert_qmp_success(qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'netinfo-server','id':%s,'netdev':'nextnet'}}", id));
}

static void object_del(QTestState *qts, const char *id)
{
    assert_qmp_success(qtest_qmp(qts,
        "{'execute':'object-del','arguments':{'id':%s}}", id));
}

static void test_nfs_coexistence(void)
{
    QTestState *qts = start_vm();

    assert_qmp_success(qtest_qmp(qts,
        "{'execute':'object-add','arguments':{"
        "'qom-type':'nfs-server','id':'nfs','fsdev':'root',"
        "'netdev':'nextnet','root-path':'/','writable':false}}"));
    object_add(qts, "ni0");
    object_del(qts, "ni0");
    object_del(qts, "nfs");
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    test_root = g_dir_make_tmp("qemu-netinfo-nfs-object-XXXXXX", NULL);
    g_assert_nonnull(test_root);

    qtest_add_func("netinfo-server-nfs-object/nfs-coexistence",
                   test_nfs_coexistence);
    ret = g_test_run();

    g_rmdir(test_root);
    g_free(test_root);
    return ret;
}
