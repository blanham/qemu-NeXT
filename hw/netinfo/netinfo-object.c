/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "hw/netinfo/netinfo-server.h"
#include "migration/blocker.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qom/object_interfaces.h"
#include "system/reset.h"

#define TYPE_NETINFO_SERVER "netinfo-server"
OBJECT_DECLARE_SIMPLE_TYPE(NetInfoServerObject, NETINFO_SERVER)

struct NetInfoServerObject {
    Object parent_obj;
    char *netdev_id;
    char *config_path;
    char *domain_tag;
    char *state_path;
    uint16_t binder_udp_port;
    uint16_t binder_tcp_port;
    uint16_t database_udp_port;
    uint16_t database_tcp_port;
    bool writable;
    bool completed;
    bool reset_registered;
    NetInfoServer *server;
    Error *migration_blocker;
    ResettableState reset_state;
};

static bool netinfo_server_properties_mutable(NetInfoServerObject *object,
                                              Error **errp)
{
    if (object->completed || object->server) {
        error_setg(errp, "NetInfo server properties cannot change after "
                   "completion");
        return false;
    }
    return true;
}

static char *netinfo_server_get_netdev(Object *obj, Error **errp)
{
    return g_strdup(NETINFO_SERVER(obj)->netdev_id);
}

static void netinfo_server_set_netdev(Object *obj, const char *value,
                                      Error **errp)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    if (!netinfo_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->netdev_id);
    object->netdev_id = g_strdup(value);
}

static char *netinfo_server_get_config(Object *obj, Error **errp)
{
    return g_strdup(NETINFO_SERVER(obj)->config_path);
}

static void netinfo_server_set_config(Object *obj, const char *value,
                                      Error **errp)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    if (!netinfo_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->config_path);
    object->config_path = g_strdup(value);
}

static char *netinfo_server_get_domain_tag(Object *obj, Error **errp)
{
    return g_strdup(NETINFO_SERVER(obj)->domain_tag);
}

static void netinfo_server_set_domain_tag(Object *obj, const char *value,
                                          Error **errp)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    if (!netinfo_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->domain_tag);
    object->domain_tag = g_strdup(value);
}

static char *netinfo_server_get_state(Object *obj, Error **errp)
{
    return g_strdup(NETINFO_SERVER(obj)->state_path);
}

static void netinfo_server_set_state(Object *obj, const char *value,
                                     Error **errp)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    if (!netinfo_server_properties_mutable(object, errp)) {
        return;
    }
    g_free(object->state_path);
    object->state_path = g_strdup(value);
}

static bool netinfo_server_get_writable(Object *obj, Error **errp)
{
    return NETINFO_SERVER(obj)->writable;
}

static void netinfo_server_set_writable(Object *obj, bool value, Error **errp)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    if (netinfo_server_properties_mutable(object, errp)) {
        object->writable = value;
    }
}

#define DEFINE_NETINFO_PORT_ACCESSORS(suffix, field, label)                 \
    static void netinfo_server_get_##suffix(                                \
        Object *obj, Visitor *visitor, const char *name, void *opaque,       \
        Error **errp)                                                        \
    {                                                                        \
        uint16_t value = NETINFO_SERVER(obj)->field;                        \
                                                                               \
        visit_type_uint16(visitor, name, &value, errp);                     \
    }                                                                        \
                                                                               \
    static void netinfo_server_set_##suffix(                                \
        Object *obj, Visitor *visitor, const char *name, void *opaque,       \
        Error **errp)                                                        \
    {                                                                        \
        NetInfoServerObject *object = NETINFO_SERVER(obj);                   \
        uint16_t value;                                                       \
                                                                               \
        if (!visit_type_uint16(visitor, name, &value, errp) ||               \
            !netinfo_server_properties_mutable(object, errp)) {              \
            return;                                                           \
        }                                                                      \
        if (!value) {                                                          \
            error_setg(errp, "NetInfo %s port must not be zero", label);     \
            return;                                                           \
        }                                                                      \
        object->field = value;                                                 \
    }

DEFINE_NETINFO_PORT_ACCESSORS(binder_udp_port, binder_udp_port, "binder UDP")
DEFINE_NETINFO_PORT_ACCESSORS(binder_tcp_port, binder_tcp_port, "binder TCP")
DEFINE_NETINFO_PORT_ACCESSORS(database_udp_port, database_udp_port,
                             "database UDP")
DEFINE_NETINFO_PORT_ACCESSORS(database_tcp_port, database_tcp_port,
                             "database TCP")

static void netinfo_server_cleanup(NetInfoServerObject *object)
{
    if (object->server) {
        netinfo_server_free(object->server);
        object->server = NULL;
    }
    if (object->reset_registered) {
        qemu_unregister_resettable(OBJECT(object));
        object->reset_registered = false;
    }
    if (object->migration_blocker) {
        migrate_del_blocker(&object->migration_blocker);
    }
    object->completed = false;
}

static ResettableState *netinfo_server_reset_state(Object *obj)
{
    return &NETINFO_SERVER(obj)->reset_state;
}

static void netinfo_server_reset_hold(Object *obj, ResetType type)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    (void)object;
    (void)type;
    /* The read-only database and its registrations are reset-independent. */
}

static NetInfoDb *netinfo_server_load_database(NetInfoServerObject *object,
                                               Error **errp)
{
    NetInfoDb *db;

    if (object->config_path && !object->config_path[0]) {
        error_setg(errp, "NetInfo config path must not be empty");
        return NULL;
    }
    if (object->config_path) {
        db = netinfo_db_load_json5(object->config_path, false, errp);
        if (!db) {
            return NULL;
        }
        if (object->domain_tag &&
            strcmp(object->domain_tag, netinfo_db_tag(db)) != 0) {
            error_setg(errp, "NetInfo domain-tag '%s' does not match "
                       "config domain tag '%s'", object->domain_tag,
                       netinfo_db_tag(db));
            netinfo_db_free(db);
            return NULL;
        }
        return db;
    }

    db = netinfo_db_new_default_with_tag(object->domain_tag ?: "network");
    if (!db) {
        error_setg(errp, "NetInfo domain-tag must be a non-empty name");
    }
    return db;
}

static void netinfo_server_complete(UserCreatable *uc, Error **errp)
{
    NetInfoServerObject *object = NETINFO_SERVER(uc);
    NetInfoServerPortConfig ports;
    g_autoptr(NetInfoDb) db = NULL;

    if (object->completed) {
        error_setg(errp, "NetInfo server is already complete");
        return;
    }
    if (!object->netdev_id || !object->netdev_id[0]) {
        error_setg(errp, "NetInfo server requires a netdev property");
        return;
    }
    if (object->writable) {
        error_setg(errp, "NetInfo writable mode is not supported");
        return;
    }
    ports.binder_udp_port = object->binder_udp_port;
    ports.binder_tcp_port = object->binder_tcp_port;
    ports.database_udp_port = object->database_udp_port;
    ports.database_tcp_port = object->database_tcp_port;
    if (!ports.binder_udp_port || !ports.binder_tcp_port ||
        !ports.database_udp_port || !ports.database_tcp_port) {
        error_setg(errp, "NetInfo server ports must not be zero");
        return;
    }

    /* Build and validate the immutable database before any RPC registration. */
    db = netinfo_server_load_database(object, errp);
    if (!db) {
        return;
    }
    object->server = netinfo_server_new_with_ports(
        g_steal_pointer(&db), object->netdev_id, UINT32_C(0x0a000202), &ports,
        errp);
    if (!object->server) {
        return;
    }

    error_setg(&object->migration_blocker,
               "NetInfo server state is not migratable");
    if (migrate_add_blocker(&object->migration_blocker, errp) < 0) {
        netinfo_server_cleanup(object);
        return;
    }
    object->completed = true;
    qemu_register_resettable(OBJECT(object));
    object->reset_registered = true;
}

static bool netinfo_server_prepare_delete(UserCreatable *uc, Error **errp)
{
    netinfo_server_cleanup(NETINFO_SERVER(uc));
    return true;
}

static void netinfo_server_unparent(Object *obj)
{
    netinfo_server_cleanup(NETINFO_SERVER(obj));
}

static void netinfo_server_instance_finalize(Object *obj)
{
    NetInfoServerObject *object = NETINFO_SERVER(obj);

    netinfo_server_cleanup(object);
    g_free(object->netdev_id);
    g_free(object->config_path);
    g_free(object->domain_tag);
    g_free(object->state_path);
}

static void netinfo_server_class_init(ObjectClass *oc, const void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    ObjectProperty *property;

    ucc->complete = netinfo_server_complete;
    ucc->prepare_delete = netinfo_server_prepare_delete;
    oc->unparent = netinfo_server_unparent;
    rc->get_state = netinfo_server_reset_state;
    rc->phases.hold = netinfo_server_reset_hold;

    object_class_property_add_str(oc, "netdev", netinfo_server_get_netdev,
                                  netinfo_server_set_netdev);
    object_class_property_add_str(oc, "config", netinfo_server_get_config,
                                  netinfo_server_set_config);
    object_class_property_add_str(oc, "domain-tag",
                                  netinfo_server_get_domain_tag,
                                  netinfo_server_set_domain_tag);
    object_class_property_add_str(oc, "state", netinfo_server_get_state,
                                  netinfo_server_set_state);
    property = object_class_property_add_bool(
        oc, "writable", netinfo_server_get_writable,
        netinfo_server_set_writable);
    object_property_set_default_bool(property, false);

    property = object_class_property_add(
        oc, "binder-udp-port", "uint16", netinfo_server_get_binder_udp_port,
        netinfo_server_set_binder_udp_port, NULL, NULL);
    object_property_set_default_uint(property, NIBIND_UDP_PORT);
    property = object_class_property_add(
        oc, "binder-tcp-port", "uint16", netinfo_server_get_binder_tcp_port,
        netinfo_server_set_binder_tcp_port, NULL, NULL);
    object_property_set_default_uint(property, NIBIND_TCP_PORT);
    property = object_class_property_add(
        oc, "database-udp-port", "uint16",
        netinfo_server_get_database_udp_port,
        netinfo_server_set_database_udp_port, NULL, NULL);
    object_property_set_default_uint(property, NI_UDP_PORT);
    property = object_class_property_add(
        oc, "database-tcp-port", "uint16",
        netinfo_server_get_database_tcp_port,
        netinfo_server_set_database_tcp_port, NULL, NULL);
    object_property_set_default_uint(property, NI_TCP_PORT);
}

static const TypeInfo netinfo_server_type_info = {
    .name = TYPE_NETINFO_SERVER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(NetInfoServerObject),
    .instance_finalize = netinfo_server_instance_finalize,
    .class_init = netinfo_server_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_USER_CREATABLE },
        { TYPE_RESETTABLE_INTERFACE },
        { }
    },
};

static void netinfo_server_register_types(void)
{
    type_register_static(&netinfo_server_type_info);
}

type_init(netinfo_server_register_types)
