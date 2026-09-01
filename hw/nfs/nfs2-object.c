/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "hw/nfs/nfs2-server.h"
#include "migration/blocker.h"
#include "net/slirp-bootp.h"
#include "qapi/error.h"
#include "system/qtest.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qom/object_interfaces.h"
#include "system/reset.h"

#define TYPE_NFS_SERVER "nfs-server"
OBJECT_DECLARE_SIMPLE_TYPE(NfsServerObject, NFS_SERVER)

struct NfsServerObject {
    Object parent_obj;
    char *fsdev_id;
    char *netdev_id;
    char *root_path;
    bool writable;
    bool completed;
    bool reset_registered;
    Nfs2Server *server;
    QemuSlirpRpcRegistration *test_rpc_registration;
    QemuSlirpBootpRootLease *bootp_lease;
    Error *migration_blocker;
    ResettableState reset_state;
};

#define NFS_TEST_RPC_PROGRAM 200001U
#define NFS_TEST_RPC_VERSION 1U
#define NFS_TEST_RPC_PORT 4001U

static OncRpcDispatchResult nfs_test_rpc_dispatch(
    OncRpcRequest *request, const OncRpcCall *call, void *opaque)
{
    uint8_t reply[32];
    OncRpcXdrWriter writer;

    (void)opaque;
    onc_rpc_xdr_writer_init(&writer, reply, sizeof(reply));
    if (!onc_rpc_reply_success(&writer, call->xid) ||
        !onc_rpc_request_reply(request, reply,
                               onc_rpc_xdr_writer_size(&writer))) {
        return ONC_RPC_DISPATCH_DROP;
    }
    return ONC_RPC_DISPATCH_REPLIED;
}

static bool nfs_test_rpc_enabled(void)
{
    return qtest_enabled() &&
           g_strcmp0(g_getenv("QEMU_NFS_TEST_RPC"), "1") == 0;
}

static bool nfs_test_rpc_register(NfsServerObject *object, Error **errp)
{
    const OncRpcProgram program = {
        .program = NFS_TEST_RPC_PROGRAM,
        .version_low = NFS_TEST_RPC_VERSION,
        .version_high = NFS_TEST_RPC_VERSION,
        .port = NFS_TEST_RPC_PORT,
        .transports = ONC_RPC_TRANSPORT_UDP,
        .dispatch = nfs_test_rpc_dispatch,
    };

    return qemu_slirp_rpc_register(object->netdev_id, &program,
                                   &object->test_rpc_registration, errp) == 0;
}

static void nfs_test_rpc_unregister(NfsServerObject *object)
{
    qemu_slirp_rpc_unregister(object->test_rpc_registration);
    object->test_rpc_registration = NULL;
}

static bool nfs_server_properties_mutable(NfsServerObject *object,
                                          Error **errp)
{
    if (object->completed || object->server || object->bootp_lease) {
        error_setg(errp, "NFS server properties cannot change after "
                   "completion");
        return false;
    }
    return true;
}

static char *nfs_server_get_fsdev(Object *obj, Error **errp)
{
    return g_strdup(NFS_SERVER(obj)->fsdev_id);
}

static void nfs_server_set_fsdev(Object *obj, const char *value, Error **errp)
{
    NfsServerObject *object = NFS_SERVER(obj);

    if (!nfs_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->fsdev_id);
    object->fsdev_id = g_strdup(value);
}

static char *nfs_server_get_netdev(Object *obj, Error **errp)
{
    return g_strdup(NFS_SERVER(obj)->netdev_id);
}

static void nfs_server_set_netdev(Object *obj, const char *value, Error **errp)
{
    NfsServerObject *object = NFS_SERVER(obj);

    if (!nfs_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->netdev_id);
    object->netdev_id = g_strdup(value);
}

static char *nfs_server_get_root_path(Object *obj, Error **errp)
{
    return g_strdup(NFS_SERVER(obj)->root_path);
}

static void nfs_server_set_root_path(Object *obj, const char *value,
                                     Error **errp)
{
    NfsServerObject *object = NFS_SERVER(obj);

    if (!nfs_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->root_path);
    object->root_path = g_strdup(value);
}

static bool nfs_server_get_writable(Object *obj, Error **errp)
{
    return NFS_SERVER(obj)->writable;
}

static void nfs_server_set_writable(Object *obj, bool value, Error **errp)
{
    NfsServerObject *object = NFS_SERVER(obj);

    if (nfs_server_properties_mutable(object, errp)) {
        object->writable = value;
    }
}

static int64_t nfs_server_clock_ms(void *opaque)
{
    return qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
}

static void nfs_server_request_ref(void *opaque)
{
    object_ref(OBJECT(opaque));
}

static void nfs_server_request_unref(void *opaque)
{
    object_unref(OBJECT(opaque));
}

static const Nfs2TransportOps nfs_server_transport_ops = {
    .clock_ms = nfs_server_clock_ms,
    .request_ref = nfs_server_request_ref,
    .request_unref = nfs_server_request_unref,
};

static void nfs_server_cleanup(NfsServerObject *object)
{
    nfs_test_rpc_unregister(object);
    if (object->server) {
        nfs2_server_begin_close(object->server);
    }
    if (object->reset_registered) {
        qemu_unregister_resettable(OBJECT(object));
        object->reset_registered = false;
    }
    if (object->migration_blocker) {
        migrate_del_blocker(&object->migration_blocker);
    }
    qemu_slirp_bootp_root_release(&object->bootp_lease);
    if (object->server && !nfs2_server_busy(object->server)) {
        nfs2_server_free(object->server);
        object->server = NULL;
    }
    object->completed = false;
}

static ResettableState *nfs_server_reset_state(Object *obj)
{
    return &NFS_SERVER(obj)->reset_state;
}

static void nfs_server_reset_hold(Object *obj, ResetType type)
{
    NfsServerObject *object = NFS_SERVER(obj);

    if (object->completed) {
        nfs2_server_reset(object->server);
    }
}

static void nfs_server_complete(UserCreatable *uc, Error **errp)
{
    NfsServerObject *object = NFS_SERVER(uc);
    size_t root_length;

    if (object->completed) {
        error_setg(errp, "NFS server is already complete");
        return;
    }
    if (!object->fsdev_id || !object->fsdev_id[0]) {
        error_setg(errp, "NFS server requires an fsdev property");
        return;
    }
    if (!object->netdev_id || !object->netdev_id[0]) {
        error_setg(errp, "NFS server requires a netdev property");
        return;
    }
    root_length = object->root_path ? strlen(object->root_path) : 0;
    if (!root_length || object->root_path[0] != '/' || root_length > 255) {
        error_setg(errp, "NFS root-path must be an absolute 1-255 byte path");
        return;
    }
    if (strcmp(object->root_path, "/")) {
        error_setg(errp, "NFS root-path must be /");
        return;
    }

    if (!qemu_slirp_bootp_root_claim(object->netdev_id, object->root_path,
                                     &object->bootp_lease, errp)) {
        return;
    }
    object->server = nfs2_server_new(object->fsdev_id, object->netdev_id,
                                     object->writable,
                                     &nfs_server_transport_ops, object, errp);
    if (!object->server) {
        goto fail;
    }
    /*
     * qtest runs in a separate process from its test code, so it cannot call
     * the registry API directly.  A test-only environment switch adds one
     * independent registration to this object; the qtest then exercises the
     * real shared PMAP DUMP with NFS and this second program in one SLiRP
     * stack.  Normal QEMU invocations never enable it.
     */
    if (nfs_test_rpc_enabled() && !nfs_test_rpc_register(object, errp)) {
        goto fail;
    }
    error_setg(&object->migration_blocker,
               "NFS server listener and file-handle state is not migratable");
    if (migrate_add_blocker(&object->migration_blocker, errp) < 0) {
        goto fail;
    }
    object->completed = true;
    qemu_register_resettable(OBJECT(object));
    object->reset_registered = true;
    return;

fail:
    nfs_server_cleanup(object);
}

static bool nfs_server_prepare_delete(UserCreatable *uc, Error **errp)
{
    NfsServerObject *object = NFS_SERVER(uc);

    if (nfs2_server_busy(object->server)) {
        error_setg(errp, "NFS server has pending filesystem work");
        return false;
    }
    nfs_server_cleanup(object);
    return true;
}

static void nfs_server_unparent(Object *obj)
{
    nfs_server_cleanup(NFS_SERVER(obj));
}

static void nfs_server_instance_finalize(Object *obj)
{
    NfsServerObject *object = NFS_SERVER(obj);

    nfs_server_cleanup(object);
    g_free(object->fsdev_id);
    g_free(object->netdev_id);
    g_free(object->root_path);
}

static void nfs_server_class_init(ObjectClass *oc, const void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    ObjectProperty *property;

    ucc->complete = nfs_server_complete;
    ucc->prepare_delete = nfs_server_prepare_delete;
    oc->unparent = nfs_server_unparent;
    rc->get_state = nfs_server_reset_state;
    rc->phases.hold = nfs_server_reset_hold;

    object_class_property_add_str(oc, "fsdev", nfs_server_get_fsdev,
                                  nfs_server_set_fsdev);
    object_class_property_add_str(oc, "netdev", nfs_server_get_netdev,
                                  nfs_server_set_netdev);
    property = object_class_property_add_str(
        oc, "root-path", nfs_server_get_root_path, nfs_server_set_root_path);
    object_property_set_default_str(property, "/");
    property = object_class_property_add_bool(
        oc, "writable", nfs_server_get_writable, nfs_server_set_writable);
    object_property_set_default_bool(property, false);
}

static const TypeInfo nfs_server_type_info = {
    .name = TYPE_NFS_SERVER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(NfsServerObject),
    .instance_finalize = nfs_server_instance_finalize,
    .class_init = nfs_server_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_USER_CREATABLE },
        { TYPE_RESETTABLE_INTERFACE },
        { }
    },
};

static void nfs_server_register_types(void)
{
    type_register_static(&nfs_server_type_info);
}

type_init(nfs_server_register_types)
