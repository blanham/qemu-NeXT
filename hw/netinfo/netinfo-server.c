/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "hw/netinfo/netinfo-server.h"
#include "hw/netinfo/netinfo-xdr.h"

typedef bool (*NetInfoEncodeBody)(OncRpcXdrWriter *writer,
                                  const void *opaque);

typedef enum NetInfoReplyKind {
    NETINFO_REPLY_SUCCESS,
    NETINFO_REPLY_PROC_UNAVAIL,
    NETINFO_REPLY_GARBAGE_ARGS,
    NETINFO_REPLY_AUTH_ERROR,
    NETINFO_REPLY_SYSTEM_ERR,
} NetInfoReplyKind;

struct NetInfoServer {
    NetInfoDb *db;
    uint32_t local_addr;
    NetInfoServerPortConfig ports;
    QemuSlirpRpcRegistration *binder_udp;
    QemuSlirpRpcRegistration *binder_tcp;
    QemuSlirpRpcRegistration *database_udp;
    QemuSlirpRpcRegistration *database_tcp;
};

#define DEFINE_NETINFO_BODY_ENCODER(name, type, encoder)                 \
    static bool name(OncRpcXdrWriter *writer, const void *opaque)         \
    {                                                                     \
        const type *value = opaque;                                       \
                                                                            \
        return encoder(writer, value);                                    \
    }

DEFINE_NETINFO_BODY_ENCODER(encode_bind_getregister_body,
                            NiBindGetRegisterResult,
                            ni_xdr_encode_bind_getregister_result)
DEFINE_NETINFO_BODY_ENCODER(encode_bind_listreg_body, NiBindListRegResult,
                            ni_xdr_encode_bind_listreg_result)
DEFINE_NETINFO_BODY_ENCODER(encode_id_result_body, NiIdResult,
                            ni_xdr_encode_id_result)
DEFINE_NETINFO_BODY_ENCODER(encode_parent_result_body, NiParentResult,
                            ni_xdr_encode_parent_result)
DEFINE_NETINFO_BODY_ENCODER(encode_children_result_body, NiChildrenResult,
                            ni_xdr_encode_children_result)
DEFINE_NETINFO_BODY_ENCODER(encode_list_result_body, NiListResult,
                            ni_xdr_encode_list_result)
DEFINE_NETINFO_BODY_ENCODER(encode_property_list_result_body,
                            NiPropertyListResult,
                            ni_xdr_encode_property_list_result)
DEFINE_NETINFO_BODY_ENCODER(encode_create_result_body, NiCreateResult,
                            ni_xdr_encode_create_result)
DEFINE_NETINFO_BODY_ENCODER(encode_lookup_result_body, NiLookupResult,
                            ni_xdr_encode_lookup_result)
DEFINE_NETINFO_BODY_ENCODER(encode_name_list_result_body, NiNameListResult,
                            ni_xdr_encode_name_list_result)
DEFINE_NETINFO_BODY_ENCODER(encode_read_name_result_body, NiReadNameResult,
                            ni_xdr_encode_read_name_result)
DEFINE_NETINFO_BODY_ENCODER(encode_rparent_result_body, NiRParentResult,
                            ni_xdr_encode_rparent_result)
DEFINE_NETINFO_BODY_ENCODER(encode_read_all_result_body, NiReadAllResult,
                            ni_xdr_encode_read_all_result)
DEFINE_NETINFO_BODY_ENCODER(encode_list_all_result_body, NiListAllResult,
                            ni_xdr_encode_list_all_result)
DEFINE_NETINFO_BODY_ENCODER(encode_property_list_body, NiPropertyList,
                            ni_xdr_encode_property_list)

static bool encode_status_body(OncRpcXdrWriter *writer, const void *opaque)
{
    return ni_xdr_encode_status(writer, *(const NiStatus *)opaque);
}

static bool netinfo_write_reply(OncRpcXdrWriter *writer, uint32_t xid,
                                NetInfoReplyKind kind,
                                NetInfoEncodeBody body, const void *body_data)
{
    bool ok;

    switch (kind) {
    case NETINFO_REPLY_SUCCESS:
        ok = onc_rpc_reply_success(writer, xid);
        break;
    case NETINFO_REPLY_PROC_UNAVAIL:
        ok = onc_rpc_reply_proc_unavail(writer, xid);
        break;
    case NETINFO_REPLY_GARBAGE_ARGS:
        ok = onc_rpc_reply_garbage_args(writer, xid);
        break;
    case NETINFO_REPLY_AUTH_ERROR:
        ok = onc_rpc_reply_auth_error(writer, xid, ONC_RPC_AUTH_TOOWEAK);
        break;
    case NETINFO_REPLY_SYSTEM_ERR:
        ok = onc_rpc_reply_system_err(writer, xid);
        break;
    default:
        return false;
    }
    return ok && (!body || body(writer, body_data));
}

static OncRpcDispatchResult netinfo_reply(OncRpcRequest *request,
                                          const OncRpcCall *call,
                                          NetInfoReplyKind kind,
                                          NetInfoEncodeBody body,
                                          const void *body_data)
{
    size_t maximum;
    g_autofree uint8_t *buffer = NULL;
    OncRpcXdrWriter writer;

    if (!request || !call) {
        if (request) {
            onc_rpc_request_drop(request);
        }
        return ONC_RPC_DISPATCH_DROP;
    }
    maximum = onc_rpc_request_is_tcp(request) ? ONC_RPC_MAX_TCP_RECORD :
                                                 ONC_RPC_MAX_DATAGRAM;
    buffer = g_malloc(maximum);
    onc_rpc_xdr_writer_init(&writer, buffer, maximum);
    if (!netinfo_write_reply(&writer, call->xid, kind, body, body_data)) {
        onc_rpc_xdr_writer_init(&writer, buffer, maximum);
        if (!onc_rpc_reply_system_err(&writer, call->xid)) {
            onc_rpc_request_drop(request);
            return ONC_RPC_DISPATCH_DROP;
        }
    }
    if (!onc_rpc_request_reply(request, buffer,
                               onc_rpc_xdr_writer_size(&writer))) {
        onc_rpc_request_drop(request);
        return ONC_RPC_DISPATCH_DROP;
    }
    return ONC_RPC_DISPATCH_REPLIED;
}

static OncRpcDispatchResult netinfo_reply_success(OncRpcRequest *request,
                                                  const OncRpcCall *call)
{
    return netinfo_reply(request, call, NETINFO_REPLY_SUCCESS, NULL, NULL);
}

static OncRpcDispatchResult netinfo_reply_status(OncRpcRequest *request,
                                                 const OncRpcCall *call,
                                                 NiStatus status)
{
    return netinfo_reply(request, call, NETINFO_REPLY_SUCCESS,
                         encode_status_body, &status);
}

static OncRpcDispatchResult netinfo_reply_id(OncRpcRequest *request,
                                             const OncRpcCall *call,
                                             NiStatus status, const NiId *id)
{
    NiIdResult result;

    ni_id_result_init(&result);
    result.status = status;
    if (status == NI_OK && id) {
        result.has_id = true;
        result.id = *id;
    } else if (status == NI_OK) {
        result.status = NI_SYSTEMERR;
    }
    OncRpcDispatchResult dispatch = netinfo_reply(
        request, call, NETINFO_REPLY_SUCCESS, encode_id_result_body, &result);
    ni_id_result_clear(&result);
    return dispatch;
}

static OncRpcDispatchResult netinfo_reply_garbage(OncRpcRequest *request,
                                                  const OncRpcCall *call)
{
    return netinfo_reply(request, call, NETINFO_REPLY_GARBAGE_ARGS, NULL, NULL);
}

static OncRpcDispatchResult netinfo_reply_proc_unavail(
    OncRpcRequest *request, const OncRpcCall *call)
{
    return netinfo_reply(request, call, NETINFO_REPLY_PROC_UNAVAIL, NULL, NULL);
}

static OncRpcDispatchResult netinfo_reply_auth_error(OncRpcRequest *request,
                                                     const OncRpcCall *call)
{
    return netinfo_reply(request, call, NETINFO_REPLY_AUTH_ERROR, NULL, NULL);
}

static OncRpcDispatchResult netinfo_reply_system_error(
    OncRpcRequest *request, const OncRpcCall *call)
{
    return netinfo_reply(request, call, NETINFO_REPLY_SYSTEM_ERR, NULL, NULL);
}

static bool netinfo_body_empty(const OncRpcCall *call)
{
    return call && onc_rpc_xdr_reader_empty(&call->body);
}

static OncRpcDispatchResult netinfo_reply_decoded_status(
    OncRpcRequest *request, const OncRpcCall *call)
{
    return netinfo_reply_status(request, call, NI_RDONLY);
}

static OncRpcDispatchResult netinfo_binder_dispatch(OncRpcRequest *request,
                                                    const OncRpcCall *call,
                                                    void *opaque)
{
    NetInfoServer *server = opaque;

    if (!server || !request || !call) {
        if (request) {
            onc_rpc_request_drop(request);
        }
        return ONC_RPC_DISPATCH_DROP;
    }
    switch (call->procedure) {
    case NIBIND_PING:
        return netinfo_body_empty(call) ?
               netinfo_reply_success(request, call) :
               netinfo_reply_garbage(request, call);

    case NIBIND_GETREGISTER: {
        OncRpcXdrReader reader = call->body;
        NiName tag;
        NiBindGetRegisterResult result;

        ni_name_init(&tag);
        ni_bind_getregister_result_init(&result);
        if (!ni_xdr_decode_name(&reader, &tag)) {
            ni_name_clear(&tag);
            ni_bind_getregister_result_clear(&result);
            return netinfo_reply_garbage(request, call);
        }
        result.status = g_strcmp0(tag, netinfo_db_tag(server->db)) == 0 ?
                        NI_OK : NI_NOTAG;
        if (result.status == NI_OK) {
            result.addrs.udp_port = server->ports.database_udp_port;
            result.addrs.tcp_port = server->ports.database_tcp_port;
        }
        ni_name_clear(&tag);
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_bind_getregister_body, &result);

            ni_bind_getregister_result_clear(&result);
            return dispatch;
        }
    }

    case NIBIND_LISTREG: {
        NiBindListRegResult result;

        ni_bind_listreg_result_init(&result);
        if (!netinfo_body_empty(call)) {
            ni_bind_listreg_result_clear(&result);
            return netinfo_reply_garbage(request, call);
        }
        result.status = NI_OK;
        result.count = 1;
        result.registrations = g_new0(NiBindRegistration, 1);
        ni_bind_registration_init(&result.registrations[0]);
        result.registrations[0].tag = g_strdup(netinfo_db_tag(server->db));
        result.registrations[0].addrs.udp_port =
            server->ports.database_udp_port;
        result.registrations[0].addrs.tcp_port =
            server->ports.database_tcp_port;
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_bind_listreg_body, &result);

            ni_bind_listreg_result_clear(&result);
            return dispatch;
        }
    }

    case NIBIND_REGISTER: {
        OncRpcXdrReader reader = call->body;
        NiBindRegistration registration;

        ni_bind_registration_init(&registration);
        if (!ni_xdr_decode_bind_registration(&reader, &registration)) {
            ni_bind_registration_clear(&registration);
            return netinfo_reply_garbage(request, call);
        }
        ni_bind_registration_clear(&registration);
        return netinfo_reply_decoded_status(request, call);
    }

    case NIBIND_UNREGISTER:
    case NIBIND_CREATEMASTER:
    case NIBIND_DESTROYDOMAIN: {
        OncRpcXdrReader reader = call->body;
        NiName name;

        ni_name_init(&name);
        if (!ni_xdr_decode_name(&reader, &name)) {
            ni_name_clear(&name);
            return netinfo_reply_garbage(request, call);
        }
        ni_name_clear(&name);
        return netinfo_reply_decoded_status(request, call);
    }

    case NIBIND_CREATECLONE: {
        OncRpcXdrReader reader = call->body;
        NiBindCloneArgs args;

        ni_bind_clone_args_init(&args);
        if (!ni_xdr_decode_bind_clone_args(&reader, &args)) {
            ni_bind_clone_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_bind_clone_args_clear(&args);
        return netinfo_reply_decoded_status(request, call);
    }

    case NIBIND_BIND: {
        OncRpcXdrReader reader = call->body;
        NiBindArgs args;
        bool matches;

        ni_bind_args_init(&args);
        if (!ni_xdr_decode_bind_args(&reader, &args)) {
            ni_bind_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        matches = g_strcmp0(args.server_tag, netinfo_db_tag(server->db)) == 0;
        ni_bind_args_clear(&args);
        if (!matches) {
            onc_rpc_request_drop(request);
            return ONC_RPC_DISPATCH_DROP;
        }
        return netinfo_reply_success(request, call);
    }

    default:
        return netinfo_reply_proc_unavail(request, call);
    }
}

static bool netinfo_collect_statistics(NetInfoDb *db, NiPropertyList *stats)
{
    GArray *pending = g_array_new(false, false, sizeof(NiId));
    NiId root = { 0 };
    size_t nodes = 0;
    size_t properties = 0;
    size_t values = 0;
    bool ok = false;

    if (netinfo_db_root(db, &root) != NI_OK) {
        goto out;
    }
    g_array_append_val(pending, root);
    while (pending->len) {
        NiId id;
        NiIdList children;
        NiPropertyList property_list;

        if (nodes == NI_SERVICE_MAX_NODES) {
            goto out;
        }
        id = g_array_index(pending, NiId, pending->len - 1);
        g_array_set_size(pending, pending->len - 1);
        ni_property_list_init(&property_list);
        if (netinfo_db_read(db, &id, &property_list) != NI_OK) {
            ni_property_list_clear(&property_list);
            goto out;
        }
        nodes++;
        properties += property_list.count;
        for (size_t i = 0; i < property_list.count; i++) {
            values += property_list.properties[i].values.count;
        }
        ni_property_list_clear(&property_list);

        ni_id_list_init(&children);
        if (netinfo_db_children(db, &id, &children) != NI_OK) {
            ni_id_list_clear(&children);
            goto out;
        }
        for (size_t i = 0; i < children.count; i++) {
            NiId child = { .nii_object = children.values[i] };

            g_array_append_val(pending, child);
        }
        ni_id_list_clear(&children);
    }

    stats->count = 3;
    stats->properties = g_new0(NiProperty, stats->count);
    for (size_t i = 0; i < stats->count; i++) {
        ni_property_init(&stats->properties[i]);
        stats->properties[i].values.count = 1;
        stats->properties[i].values.values = g_new0(NiName, 1);
    }
    stats->properties[0].name = g_strdup("nodes");
    stats->properties[0].values.values[0] = g_strdup_printf("%zu", nodes);
    stats->properties[1].name = g_strdup("properties");
    stats->properties[1].values.values[0] =
        g_strdup_printf("%zu", properties);
    stats->properties[2].name = g_strdup("values");
    stats->properties[2].values.values[0] = g_strdup_printf("%zu", values);
    ok = true;
out:
    g_array_unref(pending);
    return ok;
}

static bool netinfo_binding_matches(const NetInfoServer *server,
                                    const NiBinding *binding)
{
    return binding &&
           g_strcmp0(binding->tag, netinfo_db_tag(server->db)) == 0 &&
           binding->addr == server->local_addr;
}

static OncRpcDispatchResult netinfo_database_dispatch(
    OncRpcRequest *request, const OncRpcCall *call, void *opaque)
{
    NetInfoServer *server = opaque;
    bool tcp;

    if (!server || !request || !call) {
        if (request) {
            onc_rpc_request_drop(request);
        }
        return ONC_RPC_DISPATCH_DROP;
    }
    tcp = onc_rpc_request_is_tcp(request);
    if (!tcp && call->procedure != NI_PING && call->procedure != NI_BIND) {
        return netinfo_reply_auth_error(request, call);
    }

    switch (call->procedure) {
    case NI_PING:
        return netinfo_body_empty(call) ?
               netinfo_reply_success(request, call) :
               netinfo_reply_garbage(request, call);

    case NI_STATISTICS: {
        NiPropertyList stats;

        ni_property_list_init(&stats);
        if (!netinfo_body_empty(call)) {
            ni_property_list_clear(&stats);
            return netinfo_reply_garbage(request, call);
        }
        if (!netinfo_collect_statistics(server->db, &stats)) {
            ni_property_list_clear(&stats);
            return netinfo_reply_system_error(request, call);
        }
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_property_list_body, &stats);

            ni_property_list_clear(&stats);
            return dispatch;
        }
    }

    case NI_ROOT: {
        NiId id = { 0 };

        if (!netinfo_body_empty(call)) {
            return netinfo_reply_garbage(request, call);
        }
        return netinfo_reply_id(request, call, netinfo_db_root(server->db, &id),
                                &id);
    }

    case NI_SELF: {
        OncRpcXdrReader reader = call->body;
        NiId id;
        NiStatus status;

        if (!ni_xdr_decode_id(&reader, &id)) {
            return netinfo_reply_garbage(request, call);
        }
        status = netinfo_db_self(server->db, &id);
        return netinfo_reply_id(request, call, status, &id);
    }

    case NI_PARENT: {
        OncRpcXdrReader reader = call->body;
        NiId id, parent;
        NiParentResult result;

        if (!ni_xdr_decode_id(&reader, &id)) {
            return netinfo_reply_garbage(request, call);
        }
        ni_parent_result_init(&result);
        result.status = netinfo_db_parent(server->db, &id, &parent);
        if (result.status == NI_OK) {
            result.stuff.object_id = parent.nii_object;
            result.stuff.self_id = id;
        }
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_parent_result_body, &result);

            ni_parent_result_clear(&result);
            return dispatch;
        }
    }

    case NI_READ: {
        OncRpcXdrReader reader = call->body;
        NiId id;
        NiPropertyList properties;
        NiPropertyListResult result;

        if (!ni_xdr_decode_id(&reader, &id)) {
            return netinfo_reply_garbage(request, call);
        }
        ni_property_list_init(&properties);
        ni_property_list_result_init(&result);
        result.status = netinfo_db_read(server->db, &id, &properties);
        if (result.status == NI_OK) {
            result.stuff.id = id;
            result.stuff.props = properties;
            ni_property_list_init(&properties);
        }
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_property_list_result_body, &result);

            ni_property_list_clear(&properties);
            ni_property_list_result_clear(&result);
            return dispatch;
        }
    }

    case NI_CHILDREN: {
        OncRpcXdrReader reader = call->body;
        NiId id;
        NiIdList children;
        NiChildrenResult result;

        if (!ni_xdr_decode_id(&reader, &id)) {
            return netinfo_reply_garbage(request, call);
        }
        ni_id_list_init(&children);
        ni_children_result_init(&result);
        result.status = netinfo_db_children(server->db, &id, &children);
        if (result.status == NI_OK) {
            result.stuff.children = children;
            ni_id_list_init(&children);
            result.stuff.self_id = id;
        }
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_children_result_body, &result);

            ni_id_list_clear(&children);
            ni_children_result_clear(&result);
            return dispatch;
        }
    }

    case NI_LOOKUP: {
        OncRpcXdrReader reader = call->body;
        NiLookupArgs args;
        NiIdList found;
        NiLookupResult result;

        ni_lookup_args_init(&args);
        ni_id_list_init(&found);
        if (!ni_xdr_decode_lookup_args(&reader, &args)) {
            ni_lookup_args_clear(&args);
            ni_id_list_clear(&found);
            return netinfo_reply_garbage(request, call);
        }
        ni_lookup_result_init(&result);
        result.status = netinfo_db_lookup(server->db, &args.id, args.key,
                                          args.value, &found);
        if (result.status == NI_OK) {
            result.stuff.idlist = found;
            ni_id_list_init(&found);
            result.stuff.self_id = args.id;
        }
        ni_lookup_args_clear(&args);
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_lookup_result_body, &result);

            ni_id_list_clear(&found);
            ni_lookup_result_clear(&result);
            return dispatch;
        }
    }

    case NI_LIST: {
        OncRpcXdrReader reader = call->body;
        NiNameArgs args;
        NiEntryList entries;
        NiListResult result;

        ni_name_args_init(&args);
        ni_entry_list_init(&entries);
        if (!ni_xdr_decode_name_args(&reader, &args)) {
            ni_name_args_clear(&args);
            ni_entry_list_clear(&entries);
            return netinfo_reply_garbage(request, call);
        }
        ni_list_result_init(&result);
        result.status = netinfo_db_list(server->db, &args.id, args.name,
                                        &entries);
        if (result.status == NI_OK) {
            result.stuff.entries = entries;
            ni_entry_list_init(&entries);
            result.stuff.self_id = args.id;
        }
        ni_name_args_clear(&args);
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_list_result_body, &result);

            ni_entry_list_clear(&entries);
            ni_list_result_clear(&result);
            return dispatch;
        }
    }

    case NI_READPROP: {
        OncRpcXdrReader reader = call->body;
        NiPropArgs args;
        NiNameList values;
        NiNameListResult result;

        ni_prop_args_init(&args);
        ni_name_list_init(&values);
        if (!ni_xdr_decode_prop_args(&reader, &args)) {
            ni_prop_args_clear(&args);
            ni_name_list_clear(&values);
            return netinfo_reply_garbage(request, call);
        }
        ni_name_list_result_init(&result);
        result.status = netinfo_db_readprop(server->db, &args.id,
                                            args.prop_index, &values);
        if (result.status == NI_OK) {
            result.stuff.values = values;
            ni_name_list_init(&values);
            result.stuff.self_id = args.id;
        }
        ni_prop_args_clear(&args);
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_name_list_result_body, &result);

            ni_name_list_clear(&values);
            ni_name_list_result_clear(&result);
            return dispatch;
        }
    }

    case NI_LISTPROPS: {
        OncRpcXdrReader reader = call->body;
        NiId id;
        NiNameList names;
        NiNameListResult result;

        ni_name_list_init(&names);
        if (!ni_xdr_decode_id(&reader, &id)) {
            ni_name_list_clear(&names);
            return netinfo_reply_garbage(request, call);
        }
        ni_name_list_result_init(&result);
        result.status = netinfo_db_listprops(server->db, &id, &names);
        if (result.status == NI_OK) {
            result.stuff.values = names;
            ni_name_list_init(&names);
            result.stuff.self_id = id;
        }
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_name_list_result_body, &result);

            ni_name_list_clear(&names);
            ni_name_list_result_clear(&result);
            return dispatch;
        }
    }

    case NI_READNAME: {
        OncRpcXdrReader reader = call->body;
        NiNameIndexArgs args;
        NiName name;
        NiReadNameResult result;

        ni_name_index_args_init(&args);
        ni_name_init(&name);
        if (!ni_xdr_decode_name_index_args(&reader, &args)) {
            ni_name_index_args_clear(&args);
            ni_name_clear(&name);
            return netinfo_reply_garbage(request, call);
        }
        ni_read_name_result_init(&result);
        result.status = netinfo_db_readname(server->db, &args.id,
                                            args.prop_index, args.name_index,
                                            &name);
        if (result.status == NI_OK) {
            result.stuff.id = args.id;
            result.stuff.name = name;
            ni_name_init(&name);
        }
        ni_name_index_args_clear(&args);
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_read_name_result_body, &result);

            ni_name_clear(&name);
            ni_read_name_result_clear(&result);
            return dispatch;
        }
    }

    case NI_RPARENT: {
        NiRParentResult result;

        if (!netinfo_body_empty(call)) {
            return netinfo_reply_garbage(request, call);
        }
        ni_rparent_result_init(&result);
        result.status = NI_NETROOT;
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_rparent_result_body, &result);

            ni_rparent_result_clear(&result);
            return dispatch;
        }
    }

    case NI_BIND: {
        OncRpcXdrReader reader = call->body;
        NiBinding binding;
        bool matches;

        ni_binding_init(&binding);
        if (!ni_xdr_decode_binding(&reader, &binding)) {
            ni_binding_clear(&binding);
            return netinfo_reply_garbage(request, call);
        }
        matches = netinfo_binding_matches(server, &binding);
        ni_binding_clear(&binding);
        if (!matches) {
            onc_rpc_request_drop(request);
            return ONC_RPC_DISPATCH_DROP;
        }
        return netinfo_reply_success(request, call);
    }

    case NI_LOOKUPREAD: {
        OncRpcXdrReader reader = call->body;
        NiLookupArgs args;
        NiId found;
        NiPropertyList properties;
        NiPropertyListResult result;

        ni_lookup_args_init(&args);
        ni_property_list_init(&properties);
        if (!ni_xdr_decode_lookup_args(&reader, &args)) {
            ni_lookup_args_clear(&args);
            ni_property_list_clear(&properties);
            return netinfo_reply_garbage(request, call);
        }
        ni_property_list_result_init(&result);
        result.status = netinfo_db_lookup_read(server->db, &args.id,
                                               args.key, args.value, &found,
                                               &properties);
        if (result.status == NI_OK) {
            /*
             * The result carries the refreshed parent ID; the selected child
             * is represented only by the returned properties.
             */
            result.stuff.id = args.id;
            result.stuff.props = properties;
            ni_property_list_init(&properties);
        }
        ni_lookup_args_clear(&args);
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_property_list_result_body, &result);

            ni_property_list_clear(&properties);
            ni_property_list_result_clear(&result);
            return dispatch;
        }
    }

    case NI_CREATE: {
        OncRpcXdrReader reader = call->body;
        NiCreateArgs args;
        NiCreateResult result;

        ni_create_args_init(&args);
        if (!ni_xdr_decode_create_args(&reader, &args)) {
            ni_create_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_create_args_clear(&args);
        ni_create_result_init(&result);
        result.status = NI_RDONLY;
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_create_result_body, &result);

            ni_create_result_clear(&result);
            return dispatch;
        }
    }

    case NI_DESTROY: {
        OncRpcXdrReader reader = call->body;
        NiDestroyArgs args;

        ni_destroy_args_init(&args);
        if (!ni_xdr_decode_destroy_args(&reader, &args)) {
            ni_destroy_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_destroy_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_WRITE: {
        OncRpcXdrReader reader = call->body;
        NiPropertyListStuff stuff;

        ni_property_list_stuff_init(&stuff);
        if (!ni_xdr_decode_property_list_stuff(&reader, &stuff)) {
            ni_property_list_stuff_clear(&stuff);
            return netinfo_reply_garbage(request, call);
        }
        ni_property_list_stuff_clear(&stuff);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_CREATEPROP: {
        OncRpcXdrReader reader = call->body;
        NiCreatePropArgs args;

        ni_create_prop_args_init(&args);
        if (!ni_xdr_decode_create_prop_args(&reader, &args)) {
            ni_create_prop_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_create_prop_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_DESTROYPROP: {
        OncRpcXdrReader reader = call->body;
        NiPropArgs args;

        ni_prop_args_init(&args);
        if (!ni_xdr_decode_prop_args(&reader, &args)) {
            ni_prop_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_prop_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_WRITEPROP: {
        OncRpcXdrReader reader = call->body;
        NiWritePropArgs args;

        ni_write_prop_args_init(&args);
        if (!ni_xdr_decode_write_prop_args(&reader, &args)) {
            ni_write_prop_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_write_prop_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_RENAMEPROP: {
        OncRpcXdrReader reader = call->body;
        NiPropNameArgs args;

        ni_prop_name_args_init(&args);
        if (!ni_xdr_decode_prop_name_args(&reader, &args)) {
            ni_prop_name_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_prop_name_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_CREATENAME: {
        OncRpcXdrReader reader = call->body;
        NiCreateNameArgs args;

        ni_create_name_args_init(&args);
        if (!ni_xdr_decode_create_name_args(&reader, &args)) {
            ni_create_name_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_create_name_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_DESTROYNAME: {
        OncRpcXdrReader reader = call->body;
        NiNameIndexArgs args;

        ni_name_index_args_init(&args);
        if (!ni_xdr_decode_name_index_args(&reader, &args)) {
            ni_name_index_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_name_index_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_WRITENAME: {
        OncRpcXdrReader reader = call->body;
        NiWriteNameArgs args;

        ni_write_name_args_init(&args);
        if (!ni_xdr_decode_write_name_args(&reader, &args)) {
            ni_write_name_args_clear(&args);
            return netinfo_reply_garbage(request, call);
        }
        ni_write_name_args_clear(&args);
        return netinfo_reply_id(request, call, NI_RDONLY, NULL);
    }

    case NI_LISTALL: {
        OncRpcXdrReader reader = call->body;
        NiId id;
        NiListAllResult result;

        if (!ni_xdr_decode_id(&reader, &id)) {
            return netinfo_reply_garbage(request, call);
        }
        ni_list_all_result_init(&result);
        result.status = NI_RDONLY;
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_list_all_result_body, &result);

            ni_list_all_result_clear(&result);
            return dispatch;
        }
    }

    case NI_READALL: {
        OncRpcXdrReader reader = call->body;
        uint32_t checksum;
        NiReadAllResult result;

        if (!onc_rpc_xdr_u32(&reader, &checksum) ||
            !onc_rpc_xdr_reader_empty(&reader)) {
            return netinfo_reply_garbage(request, call);
        }
        ni_read_all_result_init(&result);
        result.status = NI_RDONLY;
        {
            OncRpcDispatchResult dispatch = netinfo_reply(
                request, call, NETINFO_REPLY_SUCCESS,
                encode_read_all_result_body, &result);

            ni_read_all_result_clear(&result);
            return dispatch;
        }
    }

    case NI_CRASHED: {
        OncRpcXdrReader reader = call->body;
        uint32_t checksum;

        if (!onc_rpc_xdr_u32(&reader, &checksum) ||
            !onc_rpc_xdr_reader_empty(&reader)) {
            return netinfo_reply_garbage(request, call);
        }
        /*
         * The historical XDR result type for NI_CRASHED is void.  Preserve
         * that wire shape as the single exception to the read-only service's
         * NI_RDONLY results.
         */
        return netinfo_reply_success(request, call);
    }

    case NI_RESYNC:
        return netinfo_body_empty(call) ?
               netinfo_reply_status(request, call, NI_RDONLY) :
               netinfo_reply_garbage(request, call);

    default:
        return netinfo_reply_proc_unavail(request, call);
    }
}

static void netinfo_unregister_all(NetInfoServer *server)
{
    if (!server) {
        return;
    }
    qemu_slirp_rpc_unregister(server->database_tcp);
    server->database_tcp = NULL;
    qemu_slirp_rpc_unregister(server->database_udp);
    server->database_udp = NULL;
    qemu_slirp_rpc_unregister(server->binder_tcp);
    server->binder_tcp = NULL;
    qemu_slirp_rpc_unregister(server->binder_udp);
    server->binder_udp = NULL;
}

void netinfo_server_free(NetInfoServer *server)
{
    if (!server) {
        return;
    }
    netinfo_unregister_all(server);
    netinfo_db_free(server->db);
    g_free(server);
}

NetInfoServer *netinfo_server_new_with_ports(
    NetInfoDb *db, const char *netdev_id, uint32_t local_addr,
    const NetInfoServerPortConfig *ports, Error **errp)
{
    NetInfoServer *server;
    OncRpcProgram binder_udp = {
        .program = NIBIND_PROG,
        .version_low = NIBIND_VERS,
        .version_high = NIBIND_VERS,
        .port = ports ? ports->binder_udp_port : 0,
        .transports = ONC_RPC_TRANSPORT_UDP,
        .dispatch = netinfo_binder_dispatch,
    };
    OncRpcProgram binder_tcp = {
        .program = NIBIND_PROG,
        .version_low = NIBIND_VERS,
        .version_high = NIBIND_VERS,
        .port = ports ? ports->binder_tcp_port : 0,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = netinfo_binder_dispatch,
    };
    OncRpcProgram database_udp = {
        .program = NI_PROG,
        .version_low = NI_VERS,
        .version_high = NI_VERS,
        .port = ports ? ports->database_udp_port : 0,
        .transports = ONC_RPC_TRANSPORT_UDP,
        .dispatch = netinfo_database_dispatch,
    };
    OncRpcProgram database_tcp = {
        .program = NI_PROG,
        .version_low = NI_VERS,
        .version_high = NI_VERS,
        .port = ports ? ports->database_tcp_port : 0,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = netinfo_database_dispatch,
    };

    if (!db || !netdev_id || !netdev_id[0] || !netinfo_db_tag(db) ||
        !netinfo_db_tag(db)[0] || !ports || !ports->binder_udp_port ||
        !ports->binder_tcp_port || !ports->database_udp_port ||
        !ports->database_tcp_port) {
        error_setg(errp, "NetInfo server configuration is incomplete");
        netinfo_db_free(db);
        return NULL;
    }
    server = g_new0(NetInfoServer, 1);
    server->db = db;
    server->local_addr = local_addr;
    server->ports = *ports;
    /* The registry copies each descriptor, so bind its opaque owner first. */
    binder_udp.opaque = server;
    binder_tcp.opaque = server;
    database_udp.opaque = server;
    database_tcp.opaque = server;
    if (qemu_slirp_rpc_register(netdev_id, &binder_udp,
                                &server->binder_udp, errp) < 0 ||
        qemu_slirp_rpc_register(netdev_id, &binder_tcp,
                                &server->binder_tcp, errp) < 0 ||
        qemu_slirp_rpc_register(netdev_id, &database_udp,
                                &server->database_udp, errp) < 0 ||
        qemu_slirp_rpc_register(netdev_id, &database_tcp,
                                &server->database_tcp, errp) < 0) {
        netinfo_server_free(server);
        return NULL;
    }
    return server;
}

NetInfoServer *netinfo_server_new_with_local_addr(NetInfoDb *db,
                                                  const char *netdev_id,
                                                  uint32_t local_addr,
                                                  Error **errp)
{
    const NetInfoServerPortConfig ports = {
        .binder_udp_port = NIBIND_UDP_PORT,
        .binder_tcp_port = NIBIND_TCP_PORT,
        .database_udp_port = NI_UDP_PORT,
        .database_tcp_port = NI_TCP_PORT,
    };

    return netinfo_server_new_with_ports(db, netdev_id, local_addr, &ports,
                                         errp);
}

NetInfoServer *netinfo_server_new(NetInfoDb *db, const char *netdev_id,
                                  Error **errp)
{
    return netinfo_server_new_with_local_addr(db, netdev_id,
                                              UINT32_C(0x0a000202), errp);
}
