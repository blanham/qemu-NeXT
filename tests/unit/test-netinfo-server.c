/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "hw/nfs/nfs2-protocol.h"
#include "hw/netinfo/netinfo-server.h"
#include "hw/netinfo/netinfo-xdr.h"
#include "qapi/error.h"

/*
 * This target deliberately uses a registration/request double.  The real
 * registry is exercised by test-onc-rpc; this harness keeps each server
 * procedure test deterministic while still checking the exact OncRpcProgram
 * matrix and request ownership/reply contract.
 */
struct QemuSlirpRpcRegistration {
    OncRpcProgram program;
    bool unregistered;
};

struct OncRpcRequest {
    OncRpcCall call;
    struct sockaddr_in peer;
    GByteArray *reply;
    bool tcp;
    bool dropped;
    unsigned replies;
    unsigned refs;
};

typedef struct ServerHarness {
    QemuSlirpRpcRegistration *registrations[8];
    size_t registration_count;
    unsigned unregister_count;
    ssize_t fail_registration;
} ServerHarness;

static ServerHarness *current_harness;

static bool rpc_registration_conflicts(const OncRpcProgram *left,
                                       const OncRpcProgram *right)
{
    return left->program == right->program &&
           left->port == right->port &&
           (left->transports & right->transports) &&
           left->version_low <= right->version_high &&
           right->version_low <= left->version_high;
}

int qemu_slirp_rpc_register(const char *netdev_id,
                            const OncRpcProgram *program,
                            QemuSlirpRpcRegistration **registration,
                            Error **errp)
{
    QemuSlirpRpcRegistration *entry;

    g_assert_nonnull(current_harness);
    if (!netdev_id || !program || !registration ||
        current_harness->registration_count >=
            G_N_ELEMENTS(current_harness->registrations)) {
        error_setg(errp, "invalid fake RPC registration");
        return -1;
    }
    if (!program->program || !program->port || !program->dispatch ||
        program->version_low > program->version_high ||
        !program->transports ||
        (program->transports & ~(unsigned)(ONC_RPC_TRANSPORT_UDP |
                                           ONC_RPC_TRANSPORT_TCP))) {
        error_setg(errp, "invalid fake RPC program");
        return -1;
    }
    for (size_t i = 0; i < current_harness->registration_count; i++) {
        QemuSlirpRpcRegistration *existing =
            current_harness->registrations[i];

        if (!existing->unregistered &&
            rpc_registration_conflicts(&existing->program, program)) {
            error_setg(errp, "conflicting fake RPC registration");
            return -1;
        }
    }
    if (current_harness->fail_registration >= 0 &&
        current_harness->registration_count ==
            (size_t)current_harness->fail_registration) {
        error_setg(errp, "injected fake RPC registration failure");
        *registration = NULL;
        return -1;
    }
    entry = g_new0(QemuSlirpRpcRegistration, 1);
    entry->program = *program;
    current_harness->registrations[current_harness->registration_count++] =
        entry;
    *registration = entry;
    return 0;
}

void qemu_slirp_rpc_unregister(QemuSlirpRpcRegistration *registration)
{
    if (!registration) {
        return;
    }
    g_assert_false(registration->unregistered);
    registration->unregistered = true;
    current_harness->unregister_count++;
}

const OncRpcCall *onc_rpc_request_call(const OncRpcRequest *request)
{
    return request ? &request->call : NULL;
}

const uint8_t *onc_rpc_request_data(const OncRpcRequest *request,
                                    size_t *length)
{
    if (length) {
        *length = request && request->call.data_length ?
                  request->call.data_length : 0;
    }
    return request ? request->call.data : NULL;
}

const struct sockaddr_in *onc_rpc_request_peer(const OncRpcRequest *request)
{
    return request ? &request->peer : NULL;
}

bool onc_rpc_request_is_tcp(const OncRpcRequest *request)
{
    return request && request->tcp;
}

OncRpcRequest *onc_rpc_request_ref(OncRpcRequest *request)
{
    if (request) {
        g_assert_cmpuint(request->refs, >, 0);
        request->refs++;
    }
    return request;
}

void onc_rpc_request_unref(OncRpcRequest *request)
{
    if (!request) {
        return;
    }
    g_assert_cmpuint(request->refs, >, 0);
    if (--request->refs == 0) {
        g_clear_pointer(&request->reply, g_byte_array_unref);
        g_free(request);
    }
}

bool onc_rpc_request_reply(OncRpcRequest *request, const uint8_t *data,
                           size_t length)
{
    if (!request || request->dropped || (length && !data) ||
        request->replies) {
        return false;
    }
    g_byte_array_append(request->reply, data, length);
    request->replies++;
    return true;
}

void onc_rpc_request_drop(OncRpcRequest *request)
{
    if (request) {
        request->dropped = true;
    }
}

static OncRpcRequest *request_new(uint32_t xid, uint32_t program,
                                  uint32_t version, uint32_t procedure,
                                  const uint8_t *body, size_t body_length,
                                  bool tcp)
{
    OncRpcRequest *request = g_new0(OncRpcRequest, 1);

    request->call.xid = xid;
    request->call.program = program;
    request->call.version = version;
    request->call.procedure = procedure;
    request->call.auth_flavor = ONC_RPC_AUTH_NULL;
    request->call.data = body;
    request->call.data_length = body_length;
    onc_rpc_xdr_reader_init(&request->call.body, body, body_length);
    request->peer.sin_family = AF_INET;
    request->peer.sin_addr.s_addr = htonl(0x0a00020f);
    request->peer.sin_port = htons(49152);
    request->reply = g_byte_array_new();
    request->tcp = tcp;
    request->refs = 1;
    return request;
}

static QemuSlirpRpcRegistration *find_registration(uint32_t program,
                                                   uint16_t port,
                                                   unsigned transport)
{
    for (size_t i = 0; i < current_harness->registration_count; i++) {
        QemuSlirpRpcRegistration *entry =
            current_harness->registrations[i];

        if (!entry->unregistered && entry->program.program == program &&
            entry->program.port == port &&
            entry->program.transports == transport) {
            return entry;
        }
    }
    return NULL;
}

static OncRpcDispatchResult dispatch_request(OncRpcRequest *request,
                                             uint16_t port)
{
    QemuSlirpRpcRegistration *entry = find_registration(
        request->call.program, port,
        request->tcp ? ONC_RPC_TRANSPORT_TCP : ONC_RPC_TRANSPORT_UDP);

    g_assert_nonnull(entry);
    return entry->program.dispatch(request, &request->call,
                                   entry->program.opaque);
}

static uint32_t reply_accepted_body(OncRpcRequest *request,
                                    OncRpcXdrReader *body)
{
    OncRpcXdrReader reader;
    uint32_t xid, type, reply_status, flavor, auth_length, accept_status;

    onc_rpc_xdr_reader_init(&reader, request->reply->data,
                            request->reply->len);
    g_assert_true(onc_rpc_xdr_u32(&reader, &xid));
    g_assert_cmpuint(xid, ==, request->call.xid);
    g_assert_true(onc_rpc_xdr_u32(&reader, &type));
    g_assert_cmpuint(type, ==, ONC_RPC_REPLY);
    g_assert_true(onc_rpc_xdr_u32(&reader, &reply_status));
    g_assert_cmpuint(reply_status, ==, ONC_RPC_MSG_ACCEPTED);
    g_assert_true(onc_rpc_xdr_u32(&reader, &flavor));
    g_assert_true(onc_rpc_xdr_u32(&reader, &auth_length));
    g_assert_cmpuint(flavor, ==, ONC_RPC_AUTH_NULL);
    g_assert_cmpuint(auth_length, ==, 0);
    g_assert_true(onc_rpc_xdr_u32(&reader, &accept_status));
    g_assert_cmpuint(accept_status, ==, ONC_RPC_SUCCESS);
    if (body) {
        *body = reader;
    }
    return accept_status;
}

static void assert_reply_garbage_args(OncRpcRequest *request)
{
    uint8_t expected[24];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, expected, sizeof(expected));
    g_assert_true(onc_rpc_reply_garbage_args(&writer, request->call.xid));
    g_assert_cmpuint(request->reply->len, ==, sizeof(expected));
    g_assert_cmpmem(request->reply->data, request->reply->len,
                    expected, sizeof(expected));
}

static void assert_reply_proc_unavail(OncRpcRequest *request)
{
    uint8_t expected[24];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, expected, sizeof(expected));
    g_assert_true(onc_rpc_reply_proc_unavail(&writer, request->call.xid));
    g_assert_cmpuint(request->reply->len, ==, sizeof(expected));
    g_assert_cmpmem(request->reply->data, request->reply->len,
                    expected, sizeof(expected));
}

static void assert_reply_auth_tooweak(OncRpcRequest *request)
{
    uint8_t expected[20];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, expected, sizeof(expected));
    g_assert_true(onc_rpc_reply_auth_error(&writer, request->call.xid,
                                           ONC_RPC_AUTH_TOOWEAK));
    g_assert_cmpuint(request->reply->len, ==, sizeof(expected));
    g_assert_cmpmem(request->reply->data, request->reply->len,
                    expected, sizeof(expected));
}

static void assert_reply_status_golden(OncRpcRequest *request,
                                       NiStatus status)
{
    uint8_t expected[28];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, expected, sizeof(expected));
    g_assert_true(onc_rpc_reply_success(&writer, request->call.xid));
    g_assert_true(ni_xdr_encode_status(&writer, status));
    g_assert_cmpuint(request->reply->len, ==, sizeof(expected));
    g_assert_cmpmem(request->reply->data, request->reply->len,
                    expected, sizeof(expected));
}

static void assert_reply_status_only(OncRpcRequest *request,
                                     uint32_t procedure)
{
    OncRpcXdrReader body;

    assert_reply_status_golden(request, NI_RDONLY);
    reply_accepted_body(request, &body);
    switch (procedure) {
    case NI_CREATE: {
        NiCreateResult result;

        ni_create_result_init(&result);
        g_assert_true(ni_xdr_decode_create_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_RDONLY);
        ni_create_result_clear(&result);
        break;
    }
    case NI_LISTALL: {
        NiListAllResult result;

        ni_list_all_result_init(&result);
        g_assert_true(ni_xdr_decode_list_all_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_RDONLY);
        ni_list_all_result_clear(&result);
        break;
    }
    case NI_READALL: {
        NiReadAllResult result;

        ni_read_all_result_init(&result);
        g_assert_true(ni_xdr_decode_read_all_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_RDONLY);
        ni_read_all_result_clear(&result);
        break;
    }
    case NI_RESYNC: {
        NiStatus status;

        g_assert_true(ni_xdr_decode_status(&body, &status));
        g_assert_cmpint(status, ==, NI_RDONLY);
        break;
    }
    default: {
        NiIdResult result;

        ni_id_result_init(&result);
        g_assert_true(ni_xdr_decode_id_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_RDONLY);
        ni_id_result_clear(&result);
        break;
    }
    }
    g_assert_true(onc_rpc_xdr_reader_empty(&body));
}

static void assert_reply_success_shape(OncRpcRequest *request,
                                       uint32_t procedure)
{
    OncRpcXdrReader body;

    reply_accepted_body(request, &body);
    switch (procedure) {
    case NI_PING:
    case NI_BIND:
    case NI_CRASHED:
        g_assert_true(onc_rpc_xdr_reader_empty(&body));
        break;
    case NI_STATISTICS: {
        NiPropertyList result;

        ni_property_list_init(&result);
        g_assert_true(ni_xdr_decode_property_list(&body, &result));
        ni_property_list_clear(&result);
        break;
    }
    case NI_ROOT:
    case NI_SELF: {
        NiIdResult result;

        ni_id_result_init(&result);
        g_assert_true(ni_xdr_decode_id_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_id_result_clear(&result);
        break;
    }
    case NI_PARENT: {
        NiParentResult result;

        ni_parent_result_init(&result);
        g_assert_true(ni_xdr_decode_parent_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_parent_result_clear(&result);
        break;
    }
    case NI_READ:
    case NI_LOOKUPREAD: {
        NiPropertyListResult result;

        ni_property_list_result_init(&result);
        g_assert_true(ni_xdr_decode_property_list_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_property_list_result_clear(&result);
        break;
    }
    case NI_CHILDREN: {
        NiChildrenResult result;

        ni_children_result_init(&result);
        g_assert_true(ni_xdr_decode_children_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_children_result_clear(&result);
        break;
    }
    case NI_LOOKUP: {
        NiLookupResult result;

        ni_lookup_result_init(&result);
        g_assert_true(ni_xdr_decode_lookup_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_lookup_result_clear(&result);
        break;
    }
    case NI_LIST: {
        NiListResult result;

        ni_list_result_init(&result);
        g_assert_true(ni_xdr_decode_list_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_list_result_clear(&result);
        break;
    }
    case NI_READPROP:
    case NI_LISTPROPS: {
        NiNameListResult result;

        ni_name_list_result_init(&result);
        g_assert_true(ni_xdr_decode_name_list_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_name_list_result_clear(&result);
        break;
    }
    case NI_READNAME: {
        NiReadNameResult result;

        ni_read_name_result_init(&result);
        g_assert_true(ni_xdr_decode_read_name_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        ni_read_name_result_clear(&result);
        break;
    }
    case NI_RPARENT: {
        NiRParentResult result;

        ni_rparent_result_init(&result);
        g_assert_true(ni_xdr_decode_rparent_result(&body, &result));
        g_assert_cmpint(result.status, ==, NI_NETROOT);
        ni_rparent_result_clear(&result);
        break;
    }
    default:
        g_assert_not_reached();
    }
    g_assert_true(onc_rpc_xdr_reader_empty(&body));
}

static bool make_binder_body(uint32_t procedure, uint8_t *buffer,
                             size_t capacity, size_t *length)
{
    OncRpcXdrWriter writer;
    bool ok = false;

    onc_rpc_xdr_writer_init(&writer, buffer, capacity);
    switch (procedure) {
    case NIBIND_REGISTER: {
        NiBindRegistration registration;

        ni_bind_registration_init(&registration);
        registration.tag = g_strdup("network");
        registration.addrs.udp_port = NI_UDP_PORT;
        registration.addrs.tcp_port = NI_TCP_PORT;
        ok = ni_xdr_encode_bind_registration(&writer, &registration);
        ni_bind_registration_clear(&registration);
        break;
    }
    case NIBIND_UNREGISTER:
    case NIBIND_CREATEMASTER:
    case NIBIND_DESTROYDOMAIN: {
        NiName name;

        ni_name_init(&name);
        name = g_strdup("network");
        ok = ni_xdr_encode_name(&writer, name);
        ni_name_clear(&name);
        break;
    }
    case NIBIND_CREATECLONE: {
        NiBindCloneArgs args;

        ni_bind_clone_args_init(&args);
        args.tag = g_strdup("network");
        args.master_name = g_strdup("localhost");
        args.master_addr = UINT32_C(0x0a000202);
        args.master_tag = g_strdup("network");
        ok = ni_xdr_encode_bind_clone_args(&writer, &args);
        ni_bind_clone_args_clear(&args);
        break;
    }
    default:
        ok = false;
        break;
    }
    if (ok && length) {
        *length = onc_rpc_xdr_writer_size(&writer);
    }
    return ok;
}

static bool make_database_body(uint32_t procedure, uint8_t *buffer,
                               size_t capacity, size_t *length)
{
    static const NiId root = { .nii_object = 0, .nii_instance = 0 };
    static const NiId machines = { .nii_object = 1, .nii_instance = 0 };
    static const NiId host = { .nii_object = 2, .nii_instance = 0 };
    OncRpcXdrWriter writer;
    bool ok = true;

    onc_rpc_xdr_writer_init(&writer, buffer, capacity);
    switch (procedure) {
    case NI_PING:
    case NI_STATISTICS:
    case NI_ROOT:
    case NI_RPARENT:
    case NI_RESYNC:
        break;
    case NI_SELF:
    case NI_PARENT:
    case NI_READ:
    case NI_CHILDREN:
    case NI_LISTPROPS:
        ok = ni_xdr_encode_id(&writer, &root);
        break;
    case NI_CREATE: {
        NiCreateArgs args;

        ni_create_args_init(&args);
        args.id = root;
        args.where = 0;
        ok = ni_xdr_encode_create_args(&writer, &args);
        ni_create_args_clear(&args);
        break;
    }
    case NI_DESTROY: {
        NiDestroyArgs args = { .parent_id = root, .self_id = machines };

        ok = ni_xdr_encode_destroy_args(&writer, &args);
        break;
    }
    case NI_WRITE: {
        NiPropertyListStuff stuff;

        ni_property_list_stuff_init(&stuff);
        stuff.id = root;
        ok = ni_xdr_encode_property_list_stuff(&writer, &stuff);
        ni_property_list_stuff_clear(&stuff);
        break;
    }
    case NI_LOOKUP:
    case NI_LOOKUPREAD: {
        NiLookupArgs args = {
            .id = root,
            .key = (char *)"name",
            .value = (char *)"machines",
        };

        ok = ni_xdr_encode_lookup_args(&writer, &args);
        break;
    }
    case NI_LIST: {
        NiNameArgs args = { .id = root, .name = (char *)"name" };

        ok = ni_xdr_encode_name_args(&writer, &args);
        break;
    }
    case NI_CREATEPROP: {
        NiCreatePropArgs args;

        ni_create_prop_args_init(&args);
        args.id = root;
        args.prop.name = g_strdup("new");
        args.where = 0;
        ok = ni_xdr_encode_create_prop_args(&writer, &args);
        ni_create_prop_args_clear(&args);
        break;
    }
    case NI_DESTROYPROP:
    case NI_READPROP: {
        NiPropArgs args = { .id = host, .prop_index = 1 };

        ok = ni_xdr_encode_prop_args(&writer, &args);
        break;
    }
    case NI_WRITEPROP: {
        NiWritePropArgs args;

        ni_write_prop_args_init(&args);
        args.id = host;
        args.prop_index = 1;
        args.values.count = 1;
        args.values.values = g_new0(NiName, 1);
        args.values.values[0] = g_strdup("replacement");
        ok = ni_xdr_encode_write_prop_args(&writer, &args);
        ni_write_prop_args_clear(&args);
        break;
    }
    case NI_RENAMEPROP: {
        NiPropNameArgs args = {
            .id = host,
            .prop_index = 1,
            .name = (char *)"renamed",
        };

        ok = ni_xdr_encode_prop_name_args(&writer, &args);
        break;
    }
    case NI_CREATENAME: {
        NiCreateNameArgs args = {
            .id = host,
            .prop_index = 1,
            .name = (char *)"replacement",
            .where = 0,
        };

        ok = ni_xdr_encode_create_name_args(&writer, &args);
        break;
    }
    case NI_DESTROYNAME:
    case NI_READNAME: {
        NiNameIndexArgs args = {
            .id = host,
            .prop_index = 1,
            .name_index = 0,
        };

        ok = ni_xdr_encode_name_index_args(&writer, &args);
        break;
    }
    case NI_WRITENAME: {
        NiWriteNameArgs args = {
            .id = host,
            .prop_index = 1,
            .name_index = 0,
            .name = (char *)"replacement",
        };

        ok = ni_xdr_encode_write_name_args(&writer, &args);
        break;
    }
    case NI_LISTALL:
        ok = ni_xdr_encode_id(&writer, &root);
        break;
    case NI_BIND: {
        NiBinding binding = {
            .tag = (char *)"network",
            .addr = UINT32_C(0x0a000202),
        };

        ok = ni_xdr_encode_binding(&writer, &binding);
        break;
    }
    case NI_READALL:
    case NI_CRASHED:
        ok = onc_rpc_xdr_put_u32(&writer, UINT32_C(0x12345678));
        break;
    default:
        ok = false;
        break;
    }
    if (ok && length) {
        *length = onc_rpc_xdr_writer_size(&writer);
    }
    return ok;
}

static void harness_setup(ServerHarness *harness)
{
    memset(harness, 0, sizeof(*harness));
    harness->fail_registration = -1;
    current_harness = harness;
}

static void harness_teardown(ServerHarness *harness)
{
    for (size_t i = 0; i < harness->registration_count; i++) {
        g_free(harness->registrations[i]);
    }
    current_harness = NULL;
}

static NetInfoServer *server_setup(ServerHarness *harness)
{
    NetInfoDb *db = netinfo_db_new_default();

    g_assert_nonnull(db);
    return netinfo_server_new_with_local_addr(
        db, "testnet", UINT32_C(0x0a000202), &error_abort);
}

static void test_registration_matrix(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    static const struct {
        uint32_t program;
        uint16_t port;
        unsigned transport;
    } expected[] = {
        { NIBIND_PROG, NIBIND_UDP_PORT, ONC_RPC_TRANSPORT_UDP },
        { NIBIND_PROG, NIBIND_TCP_PORT, ONC_RPC_TRANSPORT_TCP },
        { NI_PROG, NI_UDP_PORT, ONC_RPC_TRANSPORT_UDP },
        { NI_PROG, NI_TCP_PORT, ONC_RPC_TRANSPORT_TCP },
    };

    harness_setup(&harness);
    server = server_setup(&harness);
    g_assert_cmpuint(harness.registration_count, ==, G_N_ELEMENTS(expected));
    for (size_t i = 0; i < G_N_ELEMENTS(expected); i++) {
        g_assert_cmpuint(harness.registrations[i]->program.program, ==,
                         expected[i].program);
        g_assert_cmpuint(harness.registrations[i]->program.version_low, ==,
                         expected[i].program == NIBIND_PROG ? NIBIND_VERS :
                         NI_VERS);
        g_assert_cmpuint(harness.registrations[i]->program.version_high, ==,
                         expected[i].program == NIBIND_PROG ? NIBIND_VERS :
                         NI_VERS);
        g_assert_cmpuint(harness.registrations[i]->program.port, ==,
                         expected[i].port);
        g_assert_cmpuint(harness.registrations[i]->program.transports, ==,
                         expected[i].transport);
    }
    netinfo_server_free(server);
    g_assert_cmpuint(harness.unregister_count, ==, 4);
    harness_teardown(&harness);
}

static OncRpcDispatchResult registry_probe_dispatch(OncRpcRequest *request,
                                                    const OncRpcCall *call,
                                                    void *opaque)
{
    (void)call;
    (void)opaque;
    onc_rpc_request_drop(request);
    return ONC_RPC_DISPATCH_DROP;
}

static void test_nfs_and_netinfo_shared_registry(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    QemuSlirpRpcRegistration *mount_registration = NULL;
    QemuSlirpRpcRegistration *nfs_registration = NULL;
    Error *err = NULL;
    const OncRpcProgram mount_program = {
        .program = NFS2_MOUNT_PROGRAM,
        .version_low = NFS2_MOUNT_VERSION,
        .version_high = 3U,
        .port = 635,
        .transports = ONC_RPC_TRANSPORT_UDP,
        .dispatch = registry_probe_dispatch,
    };
    const OncRpcProgram nfs_program = {
        .program = NFS2_NFS_PROGRAM,
        .version_low = NFS2_NFS_VERSION,
        .version_high = 3U,
        .port = 2049,
        .transports = ONC_RPC_TRANSPORT_UDP,
        .dispatch = registry_probe_dispatch,
    };

    harness_setup(&harness);
    g_assert_cmpint(qemu_slirp_rpc_register("testnet", &mount_program,
                                            &mount_registration, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpint(qemu_slirp_rpc_register("testnet", &nfs_program,
                                            &nfs_registration, &err), ==, 0);
    g_assert_null(err);
    server = server_setup(&harness);
    g_assert_cmpuint(harness.registration_count, ==, 6);
    g_assert_nonnull(find_registration(NFS2_MOUNT_PROGRAM, 635,
                                       ONC_RPC_TRANSPORT_UDP));
    g_assert_nonnull(find_registration(NFS2_NFS_PROGRAM, 2049,
                                       ONC_RPC_TRANSPORT_UDP));
    g_assert_nonnull(find_registration(NIBIND_PROG, NIBIND_UDP_PORT,
                                       ONC_RPC_TRANSPORT_UDP));
    g_assert_nonnull(find_registration(NI_PROG, NI_TCP_PORT,
                                       ONC_RPC_TRANSPORT_TCP));
    for (size_t i = 0; i < harness.registration_count; i++) {
        for (size_t j = i + 1; j < harness.registration_count; j++) {
            g_assert_false(rpc_registration_conflicts(
                &harness.registrations[i]->program,
                &harness.registrations[j]->program));
        }
    }

    {
        QemuSlirpRpcRegistration *duplicate = NULL;

        g_assert_cmpint(qemu_slirp_rpc_register("testnet", &nfs_program,
                                                &duplicate, &err), ==, -1);
        g_assert_null(duplicate);
        g_assert_nonnull(err);
        error_free(err);
        err = NULL;
    }
    netinfo_server_free(server);
    g_assert_cmpuint(harness.unregister_count, ==, 4);
    qemu_slirp_rpc_unregister(nfs_registration);
    qemu_slirp_rpc_unregister(mount_registration);
    g_assert_cmpuint(harness.unregister_count, ==, 6);
    harness_teardown(&harness);
}

static void test_binder_getregister_listreg_and_mutation(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[256];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    OncRpcRequest *request;
    NiBindGetRegisterResult getregister;
    NiBindListRegResult listreg;
    NiBindCloneArgs clone;
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(ni_xdr_encode_name(&writer, "network"));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(1, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_GETREGISTER, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    g_assert_cmpuint(request->replies, ==, 1);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 2, 0x94,
            0, 0, 2, 0x96,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    reply_accepted_body(request, &reply);
    ni_bind_getregister_result_init(&getregister);
    g_assert_true(ni_xdr_decode_bind_getregister_result(&reply, &getregister));
    g_assert_cmpint(getregister.status, ==, NI_OK);
    g_assert_cmpuint(getregister.addrs.udp_port, ==, NI_UDP_PORT);
    g_assert_cmpuint(getregister.addrs.tcp_port, ==, NI_TCP_PORT);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    ni_bind_getregister_result_clear(&getregister);
    onc_rpc_request_unref(request);

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(ni_xdr_encode_name(&writer, "other"));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(2, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_GETREGISTER, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    ni_bind_getregister_result_init(&getregister);
    g_assert_true(ni_xdr_decode_bind_getregister_result(&reply, &getregister));
    g_assert_cmpint(getregister.status, ==, NI_NOTAG);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    ni_bind_getregister_result_clear(&getregister);
    onc_rpc_request_unref(request);

    request = request_new(3, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_LISTREG, NULL, 0, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    ni_bind_listreg_result_init(&listreg);
    g_assert_true(ni_xdr_decode_bind_listreg_result(&reply, &listreg));
    g_assert_cmpint(listreg.status, ==, NI_OK);
    g_assert_cmpuint(listreg.count, ==, 1);
    g_assert_cmpstr(listreg.registrations[0].tag, ==, "network");
    g_assert_cmpuint(listreg.registrations[0].addrs.udp_port, ==, NI_UDP_PORT);
    g_assert_cmpuint(listreg.registrations[0].addrs.tcp_port, ==, NI_TCP_PORT);
    ni_bind_listreg_result_clear(&listreg);
    onc_rpc_request_unref(request);

    request = request_new(4, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_PING, NULL, 0, true);
    g_assert_cmpint(dispatch_request(request, NIBIND_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    g_assert_cmpuint(request->replies, ==, 1);
    g_assert_cmpuint(request->reply->len, ==, 24);
    reply_accepted_body(request, &reply);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    onc_rpc_request_unref(request);

    {
        NiBindRegistration registration = {
            .tag = (char *)"network",
            .addrs = { .udp_port = 0, .tcp_port = 0 },
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_bind_registration(&writer,
                                                      &registration));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(5, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_REGISTER, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiStatus status;

        g_assert_true(ni_xdr_decode_status(&reply, &status));
        g_assert_cmpint(status, ==, NI_RDONLY);
    }
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    onc_rpc_request_unref(request);

    ni_bind_clone_args_init(&clone);
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    clone.tag = g_strdup("network");
    clone.master_name = g_strdup("localhost");
    clone.master_addr = UINT32_C(0x0a000202);
    clone.master_tag = g_strdup("network");
    g_assert_true(ni_xdr_encode_bind_clone_args(&writer, &clone));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(6, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_CREATECLONE, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiStatus status;

        g_assert_true(ni_xdr_decode_status(&reply, &status));
        g_assert_cmpint(status, ==, NI_RDONLY);
    }
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    ni_bind_clone_args_clear(&clone);
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_binder_silent_mismatch_and_malformed(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[64];
    OncRpcXdrWriter writer;
    OncRpcRequest *request;
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);

    {
        NiBindArgs args = {
            .client_addr = UINT32_C(0x0a00020f),
            .client_tag = (char *)"network",
            .server_tag = (char *)"other",
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_bind_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(7, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_BIND, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_DROP);
    g_assert_cmpuint(request->replies, ==, 0);
    g_assert_true(request->dropped);
    onc_rpc_request_unref(request);

    {
        NiBindArgs args = {
            .client_addr = UINT32_C(0x0a00020f),
            .client_tag = (char *)"network",
            .server_tag = (char *)"network",
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_bind_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(70, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_BIND, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 70, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    g_assert_cmpuint(request->reply->len, ==, 24);
    {
        OncRpcXdrReader reply;

        reply_accepted_body(request, &reply);
        g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    }
    onc_rpc_request_unref(request);

    request = request_new(8, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_GETREGISTER, body, 3, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 8, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    g_assert_cmpuint(request->replies, ==, 1);
    g_assert_cmpuint(request->reply->len, ==, 24);
    {
        OncRpcXdrReader reply;
        uint32_t xid, type, message_status, flavor, auth_length, accept_status;

        onc_rpc_xdr_reader_init(&reply, request->reply->data,
                                request->reply->len);
        g_assert_true(onc_rpc_xdr_u32(&reply, &xid));
        g_assert_true(onc_rpc_xdr_u32(&reply, &type));
        g_assert_true(onc_rpc_xdr_u32(&reply, &message_status));
        g_assert_true(onc_rpc_xdr_u32(&reply, &flavor));
        g_assert_true(onc_rpc_xdr_u32(&reply, &auth_length));
        g_assert_true(onc_rpc_xdr_u32(&reply, &accept_status));
        g_assert_cmpuint(xid, ==, request->call.xid);
        g_assert_cmpuint(type, ==, ONC_RPC_REPLY);
        g_assert_cmpuint(message_status, ==, ONC_RPC_MSG_ACCEPTED);
        g_assert_cmpuint(flavor, ==, ONC_RPC_AUTH_NULL);
        g_assert_cmpuint(auth_length, ==, 0);
        g_assert_cmpuint(accept_status, ==, ONC_RPC_GARBAGE_ARGS);
        g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    }
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_binder_mutation_matrix(void)
{
    static const uint32_t procedures[] = {
        NIBIND_REGISTER,
        NIBIND_UNREGISTER,
        NIBIND_CREATEMASTER,
        NIBIND_CREATECLONE,
        NIBIND_DESTROYDOMAIN,
    };
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[1024];
    OncRpcXdrWriter writer;

    harness_setup(&harness);
    server = server_setup(&harness);
    for (size_t i = 0; i < G_N_ELEMENTS(procedures); i++) {
        OncRpcRequest *request;
        size_t body_length;

        g_assert_true(make_binder_body(procedures[i], body, sizeof(body),
                                       &body_length));
        request = request_new(100 + i, NIBIND_PROG, NIBIND_VERS,
                              procedures[i], body, body_length, false);
        g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        if (i == 0) {
            static const uint8_t expected[] = {
                0, 0, 0, 100, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 13,
            };

            g_assert_cmpmem(request->reply->data, request->reply->len,
                            expected, sizeof(expected));
        }
        assert_reply_status_golden(request, NI_RDONLY);
        {
            OncRpcXdrReader reply;
            NiStatus status;

            reply_accepted_body(request, &reply);
            g_assert_true(ni_xdr_decode_status(&reply, &status));
            g_assert_cmpint(status, ==, NI_RDONLY);
            g_assert_true(onc_rpc_xdr_reader_empty(&reply));
        }
        onc_rpc_request_unref(request);

        onc_rpc_xdr_writer_init(&writer, body + body_length,
                                sizeof(body) - body_length);
        g_assert_true(onc_rpc_xdr_put_u32(&writer, UINT32_C(0xfeedface)));
        request = request_new(200 + i, NIBIND_PROG, NIBIND_VERS,
                              procedures[i], body, body_length + 4, false);
        g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        assert_reply_garbage_args(request);
        onc_rpc_request_unref(request);

        request = request_new(300 + i, NIBIND_PROG, NIBIND_VERS,
                              procedures[i], body, body_length - 1, false);
        g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        assert_reply_garbage_args(request);
        onc_rpc_request_unref(request);
    }
    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_nondefault_domain_tag_and_local_address(void)
{
    ServerHarness harness;
    NetInfoDb *db;
    NetInfoServer *server;
    uint8_t body[256];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    OncRpcRequest *request;
    size_t body_length;
    const uint32_t local_addr = UINT32_C(0xc0a8012a);

    harness_setup(&harness);
    db = netinfo_db_new_with_tag("lab-domain");
    g_assert_nonnull(db);
    server = netinfo_server_new_with_local_addr(db, "testnet", local_addr,
                                                &error_abort);

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(ni_xdr_encode_name(&writer, "lab-domain"));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(400, NIBIND_PROG, NIBIND_VERS,
                          NIBIND_GETREGISTER, body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NIBIND_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiBindGetRegisterResult result;

        ni_bind_getregister_result_init(&result);
        g_assert_true(ni_xdr_decode_bind_getregister_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.addrs.udp_port, ==, NI_UDP_PORT);
        g_assert_cmpuint(result.addrs.tcp_port, ==, NI_TCP_PORT);
        ni_bind_getregister_result_clear(&result);
    }
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    onc_rpc_request_unref(request);

    {
        NiBinding binding = {
            .tag = (char *)"lab-domain",
            .addr = local_addr,
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_binding(&writer, &binding));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(401, NI_PROG, NI_VERS, NI_BIND,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    g_assert_cmpuint(request->reply->len, ==, 24);
    reply_accepted_body(request, &reply);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_database_tcp_reads_and_udp_auth(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[256];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    OncRpcRequest *request;
    NiId id = { .nii_object = 0, .nii_instance = 0 };
    NiPropertyListResult result;
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(ni_xdr_encode_id(&writer, &id));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(9, NI_PROG, NI_VERS, NI_READ,
                          body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NI_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    g_assert_cmpuint(request->reply->len, ==, 20);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 9, 0, 0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0, 5,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    onc_rpc_request_unref(request);

    request = request_new(10, NI_PROG, NI_VERS, NI_READ,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    ni_property_list_result_init(&result);
    g_assert_true(ni_xdr_decode_property_list_result(&reply, &result));
    g_assert_cmpint(result.status, ==, NI_OK);
    g_assert_cmpuint(result.stuff.id.nii_object, ==, 0);
    g_assert_cmpuint(result.stuff.id.nii_instance, ==, 0x24);
    g_assert_cmpuint(result.stuff.props.count, ==, 2);
    g_assert_cmpstr(result.stuff.props.properties[0].name, ==, "name");
    g_assert_cmpstr(result.stuff.props.properties[0].values.values[0], ==,
                    "/");
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    ni_property_list_result_clear(&result);
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_database_read_procedures_refresh_instances(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[512];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    OncRpcRequest *request;
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);

    request = request_new(16, NI_PROG, NI_VERS, NI_ROOT, NULL, 0, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiIdResult result;

        ni_id_result_init(&result);
        g_assert_true(ni_xdr_decode_id_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.id.nii_object, ==, 0);
        g_assert_cmpuint(result.id.nii_instance, ==, 0x24);
        ni_id_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiId id = { .nii_object = 0, .nii_instance = 1 };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_id(&writer, &id));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(17, NI_PROG, NI_VERS, NI_SELF,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiIdResult result;

        ni_id_result_init(&result);
        g_assert_true(ni_xdr_decode_id_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.id.nii_instance, ==, 0x24);
        ni_id_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiId id = { .nii_object = 2, .nii_instance = 0 };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_id(&writer, &id));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(18, NI_PROG, NI_VERS, NI_PARENT,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiParentResult result;

        ni_parent_result_init(&result);
        g_assert_true(ni_xdr_decode_parent_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.stuff.object_id, ==, 1);
        g_assert_cmpuint(result.stuff.self_id.nii_instance, ==, 1);
        ni_parent_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiId id = { .nii_object = 1, .nii_instance = 0 };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_id(&writer, &id));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(19, NI_PROG, NI_VERS, NI_CHILDREN,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiChildrenResult result;

        ni_children_result_init(&result);
        g_assert_true(ni_xdr_decode_children_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.stuff.children.count, ==, 1);
        g_assert_cmpuint(result.stuff.children.values[0], ==, 2);
        g_assert_cmpuint(result.stuff.self_id.nii_instance, ==, 1);
        ni_children_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiLookupArgs args = {
            .id = { .nii_object = 0, .nii_instance = 0 },
            .key = (char *)"name",
            .value = (char *)"machines",
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_lookup_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(20, NI_PROG, NI_VERS, NI_LOOKUP,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiLookupResult result;

        ni_lookup_result_init(&result);
        g_assert_true(ni_xdr_decode_lookup_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.stuff.idlist.count, ==, 1);
        g_assert_cmpuint(result.stuff.idlist.values[0], ==, 1);
        g_assert_cmpuint(result.stuff.self_id.nii_instance, ==, 0x24);
        ni_lookup_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiNameArgs args = {
            .id = { .nii_object = 0, .nii_instance = 0 },
            .name = (char *)"name",
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_name_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(21, NI_PROG, NI_VERS, NI_LIST,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiListResult result;

        ni_list_result_init(&result);
        g_assert_true(ni_xdr_decode_list_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.stuff.entries.count, ==, 1);
        g_assert_true(result.stuff.entries.entries[0].has_names);
        g_assert_cmpstr(result.stuff.entries.entries[0].names.values[0], ==,
                        "machines");
        ni_list_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiPropArgs args = {
            .id = { .nii_object = 2, .nii_instance = 0 },
            .prop_index = 1,
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_prop_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(22, NI_PROG, NI_VERS, NI_READPROP,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiNameListResult result;

        ni_name_list_result_init(&result);
        g_assert_true(ni_xdr_decode_name_list_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpstr(result.stuff.values.values[0], ==, "10.0.2.2");
        g_assert_cmpuint(result.stuff.self_id.nii_instance, ==, 1);
        ni_name_list_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiId id = { .nii_object = 2, .nii_instance = 0 };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_id(&writer, &id));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(23, NI_PROG, NI_VERS, NI_LISTPROPS,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiNameListResult result;

        ni_name_list_result_init(&result);
        g_assert_true(ni_xdr_decode_name_list_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.stuff.values.count, ==, 3);
        g_assert_cmpstr(result.stuff.values.values[2], ==, "serves");
        ni_name_list_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiNameIndexArgs args = {
            .id = { .nii_object = 2, .nii_instance = 0 },
            .prop_index = 1,
            .name_index = 0,
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_name_index_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(24, NI_PROG, NI_VERS, NI_READNAME,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiReadNameResult result;

        ni_read_name_result_init(&result);
        g_assert_true(ni_xdr_decode_read_name_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpstr(result.stuff.name, ==, "10.0.2.2");
        g_assert_cmpuint(result.stuff.id.nii_instance, ==, 1);
        ni_read_name_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    {
        NiLookupArgs args = {
            .id = { .nii_object = 1, .nii_instance = 0 },
            .key = (char *)"name",
            .value = (char *)"localhost",
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_lookup_args(&writer, &args));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(25, NI_PROG, NI_VERS, NI_LOOKUPREAD,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiPropertyListResult result;

        ni_property_list_result_init(&result);
        g_assert_true(ni_xdr_decode_property_list_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_OK);
        g_assert_cmpuint(result.stuff.id.nii_object, ==, 1);
        g_assert_cmpuint(result.stuff.id.nii_instance, ==, 1);
        g_assert_cmpuint(result.stuff.props.count, ==, 3);
        g_assert_cmpstr(result.stuff.props.properties[0].name, ==, "name");
        ni_property_list_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_database_bind_rparent_statistics_and_readonly(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[512];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    OncRpcRequest *request;
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);

    {
        NiBinding binding = {
            .tag = (char *)"network",
            .addr = UINT32_C(0x0a000202),
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_binding(&writer, &binding));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(11, NI_PROG, NI_VERS, NI_BIND,
                          body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NI_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 11, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    g_assert_cmpuint(request->reply->len, ==, 24);
    onc_rpc_request_unref(request);

    {
        NiBinding binding = {
            .tag = (char *)"network",
            .addr = UINT32_C(0x0a00020f),
        };

        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_binding(&writer, &binding));
        body_length = onc_rpc_xdr_writer_size(&writer);
    }
    request = request_new(12, NI_PROG, NI_VERS, NI_BIND,
                          body, body_length, false);
    g_assert_cmpint(dispatch_request(request, NI_UDP_PORT), ==,
                    ONC_RPC_DISPATCH_DROP);
    g_assert_cmpuint(request->replies, ==, 0);
    onc_rpc_request_unref(request);

    request = request_new(13, NI_PROG, NI_VERS, NI_RPARENT,
                          NULL, 0, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 13, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 11,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    reply_accepted_body(request, &reply);
    {
        NiRParentResult result;

        ni_rparent_result_init(&result);
        g_assert_true(ni_xdr_decode_rparent_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_NETROOT);
        g_assert_true(onc_rpc_xdr_reader_empty(&reply));
        ni_rparent_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    request = request_new(14, NI_PROG, NI_VERS, NI_STATISTICS,
                          NULL, 0, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiPropertyList stats;

        ni_property_list_init(&stats);
        g_assert_true(ni_xdr_decode_property_list(&reply, &stats));
        g_assert_cmpuint(stats.count, ==, 3);
        g_assert_cmpstr(stats.properties[0].name, ==, "nodes");
        g_assert_cmpstr(stats.properties[0].values.values[0], ==, "3");
        g_assert_cmpstr(stats.properties[1].name, ==, "properties");
        g_assert_cmpstr(stats.properties[1].values.values[0], ==, "6");
        g_assert_cmpstr(stats.properties[2].name, ==, "values");
        g_assert_cmpstr(stats.properties[2].values.values[0], ==, "6");
        for (size_t i = 0; i < stats.count; i++) {
            for (size_t j = 0; j < stats.properties[i].values.count; j++) {
                g_assert_null(strstr(stats.properties[i].values.values[j],
                                     "testnet"));
            }
        }
        ni_property_list_clear(&stats);
    }
    onc_rpc_request_unref(request);

    {
        NiPropertyListStuff write;

        ni_property_list_stuff_init(&write);
        write.id.nii_object = 0;
        write.id.nii_instance = 0x24;
        onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
        g_assert_true(ni_xdr_encode_property_list_stuff(&writer, &write));
        body_length = onc_rpc_xdr_writer_size(&writer);
        ni_property_list_stuff_clear(&write);
    }
    request = request_new(15, NI_PROG, NI_VERS, NI_WRITE,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    reply_accepted_body(request, &reply);
    {
        NiIdResult result;

        ni_id_result_init(&result);
        g_assert_true(ni_xdr_decode_id_result(&reply, &result));
        g_assert_cmpint(result.status, ==, NI_RDONLY);
        g_assert_true(onc_rpc_xdr_reader_empty(&reply));
        ni_id_result_clear(&result);
    }
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_database_tcp_full_procedure_matrix(void)
{
    static const struct {
        uint32_t procedure;
        bool mutation;
    } procedures[] = {
        { NI_PING, false },
        { NI_STATISTICS, false },
        { NI_ROOT, false },
        { NI_SELF, false },
        { NI_PARENT, false },
        { NI_CREATE, true },
        { NI_DESTROY, true },
        { NI_READ, false },
        { NI_WRITE, true },
        { NI_CHILDREN, false },
        { NI_LOOKUP, false },
        { NI_LIST, false },
        { NI_CREATEPROP, true },
        { NI_DESTROYPROP, true },
        { NI_READPROP, false },
        { NI_WRITEPROP, true },
        { NI_RENAMEPROP, true },
        { NI_LISTPROPS, false },
        { NI_CREATENAME, true },
        { NI_DESTROYNAME, true },
        { NI_READNAME, false },
        { NI_WRITENAME, true },
        { NI_RPARENT, false },
        { NI_LISTALL, true },
        { NI_BIND, false },
        { NI_READALL, true },
        { NI_CRASHED, true },
        { NI_RESYNC, true },
        { NI_LOOKUPREAD, false },
    };
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[8192];
    OncRpcXdrWriter writer;

    harness_setup(&harness);
    server = server_setup(&harness);
    for (size_t i = 0; i < G_N_ELEMENTS(procedures); i++) {
        OncRpcRequest *request;
        size_t body_length;

        g_assert_true(make_database_body(procedures[i].procedure, body,
                                         sizeof(body), &body_length));
        request = request_new(500 + i, NI_PROG, NI_VERS,
                              procedures[i].procedure, body, body_length, true);
        g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        if (procedures[i].procedure == NI_CRASHED) {
            /*
             * The historical XDR result type for NI_CRASHED is void, even on
             * this read-only server.
             */
            assert_reply_success_shape(request, procedures[i].procedure);
        } else if (procedures[i].mutation) {
            assert_reply_status_only(request, procedures[i].procedure);
        } else {
            assert_reply_success_shape(request, procedures[i].procedure);
        }
        onc_rpc_request_unref(request);

        onc_rpc_xdr_writer_init(&writer, body + body_length,
                                sizeof(body) - body_length);
        g_assert_true(onc_rpc_xdr_put_u32(&writer, UINT32_C(0xfeedface)));
        request = request_new(600 + i, NI_PROG, NI_VERS,
                              procedures[i].procedure, body, body_length + 4,
                              true);
        g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        assert_reply_garbage_args(request);
        onc_rpc_request_unref(request);

        if (body_length) {
            request = request_new(700 + i, NI_PROG, NI_VERS,
                                  procedures[i].procedure, body,
                                  body_length - 1, true);
            g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                            ONC_RPC_DISPATCH_REPLIED);
            assert_reply_garbage_args(request);
            onc_rpc_request_unref(request);
        }
    }

    for (size_t i = 0; i < 2; i++) {
        uint32_t procedure = i ? UINT32_MAX : 29;
        OncRpcRequest *request = request_new(800 + i, NI_PROG, NI_VERS,
                                             procedure, NULL, 0, true);

        g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        assert_reply_proc_unavail(request);
        onc_rpc_request_unref(request);
    }
    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_database_udp_procedure_matrix(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[256];
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);
    for (uint32_t procedure = 0; procedure <= NI_LOOKUPREAD; procedure++) {
        OncRpcRequest *request;

        body_length = 0;
        if (procedure == NI_BIND) {
            g_assert_true(make_database_body(procedure, body, sizeof(body),
                                             &body_length));
        }
        request = request_new(900 + procedure, NI_PROG, NI_VERS, procedure,
                              body_length ? body : NULL, body_length, false);
        g_assert_cmpint(dispatch_request(request, NI_UDP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        if (procedure == NI_PING || procedure == NI_BIND) {
            assert_reply_success_shape(request, procedure);
        } else {
            if (procedure == NI_STATISTICS) {
                static const uint8_t expected[] = {
                    0, 0, 3, 0x85, 0, 0, 0, 1, 0, 0, 0, 1,
                    0, 0, 0, 1, 0, 0, 0, 5,
                };

                g_assert_cmpmem(request->reply->data, request->reply->len,
                                expected, sizeof(expected));
            }
            assert_reply_auth_tooweak(request);
        }
        onc_rpc_request_unref(request);
    }
    for (size_t i = 0; i < 2; i++) {
        uint32_t procedure = i ? UINT32_MAX : 29;
        OncRpcRequest *request = request_new(1000 + i, NI_PROG, NI_VERS,
                                             procedure, NULL, 0, false);

        g_assert_cmpint(dispatch_request(request, NI_UDP_PORT), ==,
                        ONC_RPC_DISPATCH_REPLIED);
        assert_reply_auth_tooweak(request);
        onc_rpc_request_unref(request);
    }
    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_database_crashed_readonly_and_trailing(void)
{
    ServerHarness harness;
    NetInfoServer *server;
    uint8_t body[16];
    OncRpcXdrWriter writer;
    OncRpcRequest *request;
    size_t body_length;

    harness_setup(&harness);
    server = server_setup(&harness);

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, UINT32_C(0x12345678)));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(26, NI_PROG, NI_VERS, NI_CRASHED,
                          body, body_length, true);
    /*
     * The historical XDR result type for NI_CRASHED is void.  Preserve that
     * wire shape as the single exception to the read-only service's NI_RDONLY
     * results.
     */
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    {
        static const uint8_t expected[] = {
            0, 0, 0, 26, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        };

        g_assert_cmpmem(request->reply->data, request->reply->len,
                        expected, sizeof(expected));
    }
    {
        OncRpcXdrReader reply;

        reply_accepted_body(request, &reply);
        g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    }
    onc_rpc_request_unref(request);

    g_assert_true(onc_rpc_xdr_put_u32(&writer, UINT32_C(0xfeedface)));
    body_length = onc_rpc_xdr_writer_size(&writer);
    request = request_new(27, NI_PROG, NI_VERS, NI_CRASHED,
                          body, body_length, true);
    g_assert_cmpint(dispatch_request(request, NI_TCP_PORT), ==,
                    ONC_RPC_DISPATCH_REPLIED);
    g_assert_cmpuint(request->reply->len, ==, 24);
    {
        OncRpcXdrReader reply;
        uint32_t xid, type, message_status, flavor, auth_length, accept_status;

        onc_rpc_xdr_reader_init(&reply, request->reply->data,
                                request->reply->len);
        g_assert_true(onc_rpc_xdr_u32(&reply, &xid));
        g_assert_true(onc_rpc_xdr_u32(&reply, &type));
        g_assert_true(onc_rpc_xdr_u32(&reply, &message_status));
        g_assert_true(onc_rpc_xdr_u32(&reply, &flavor));
        g_assert_true(onc_rpc_xdr_u32(&reply, &auth_length));
        g_assert_true(onc_rpc_xdr_u32(&reply, &accept_status));
        g_assert_cmpuint(xid, ==, request->call.xid);
        g_assert_cmpuint(type, ==, ONC_RPC_REPLY);
        g_assert_cmpuint(message_status, ==, ONC_RPC_MSG_ACCEPTED);
        g_assert_cmpuint(flavor, ==, ONC_RPC_AUTH_NULL);
        g_assert_cmpuint(auth_length, ==, 0);
        g_assert_cmpuint(accept_status, ==, ONC_RPC_GARBAGE_ARGS);
        g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    }
    onc_rpc_request_unref(request);

    netinfo_server_free(server);
    harness_teardown(&harness);
}

static void test_registration_failure_unwinds(void)
{
    ServerHarness harness;
    NetInfoDb *db;
    NetInfoServer *server;
    Error *err = NULL;

    harness_setup(&harness);
    harness.fail_registration = 2;
    db = netinfo_db_new_default();
    g_assert_nonnull(db);
    server = netinfo_server_new(db, "testnet", &err);
    g_assert_null(server);
    g_assert_nonnull(err);
    g_assert_cmpuint(harness.registration_count, ==, 2);
    g_assert_cmpuint(harness.unregister_count, ==, 2);
    error_free(err);
    /* Constructor failure consumes the database. */
    harness_teardown(&harness);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/netinfo-server/registration/matrix",
                    test_registration_matrix);
    g_test_add_func("/netinfo-server/registration/nfs-coexistence",
                    test_nfs_and_netinfo_shared_registry);
    g_test_add_func("/netinfo-server/binder/getregister-listreg-mutation",
                    test_binder_getregister_listreg_and_mutation);
    g_test_add_func("/netinfo-server/binder/silent-mismatch-malformed",
                    test_binder_silent_mismatch_and_malformed);
    g_test_add_func("/netinfo-server/binder/mutation-matrix",
                    test_binder_mutation_matrix);
    g_test_add_func("/netinfo-server/binder/nondefault-domain",
                    test_nondefault_domain_tag_and_local_address);
    g_test_add_func("/netinfo-server/database/tcp-read-udp-auth",
                    test_database_tcp_reads_and_udp_auth);
    g_test_add_func("/netinfo-server/database/read-procedures-refresh",
                    test_database_read_procedures_refresh_instances);
    g_test_add_func("/netinfo-server/database/bind-rparent-stats-readonly",
                    test_database_bind_rparent_statistics_and_readonly);
    g_test_add_func("/netinfo-server/database/tcp-full-matrix",
                    test_database_tcp_full_procedure_matrix);
    g_test_add_func("/netinfo-server/database/udp-full-matrix",
                    test_database_udp_procedure_matrix);
    g_test_add_func("/netinfo-server/database/crashed-readonly-trailing",
                    test_database_crashed_readonly_and_trailing);
    g_test_add_func("/netinfo-server/registration/failure-unwind",
                    test_registration_failure_unwinds);
    return g_test_run();
}
