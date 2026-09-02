/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "qemu/queue.h"
#include "qemu/bswap.h"
#include "net/slirp-rpc-internal.h"
#include "qapi/error.h"

#define ONC_RPC_PORTMAP_PROGRAM 100000U
#define ONC_RPC_PORTMAP_VERSION 2U
#define ONC_RPC_PORTMAP_PORT 111U

typedef struct QemuSlirpRpcEndpoint QemuSlirpRpcEndpoint;
typedef struct QemuSlirpRpcTcpConnection QemuSlirpRpcTcpConnection;
typedef struct QemuSlirpRpcTcpReply QemuSlirpRpcTcpReply;

struct QemuSlirpRpcRegistry {
    QemuSlirpUdpRegistry *udp_registry;
    QemuSlirpStreamRegistry *stream_registry;
    QemuSlirpRpcEndpoint *udp_portmap;
    QemuSlirpRpcEndpoint *tcp_portmap;
    QTAILQ_HEAD(, QemuSlirpRpcEndpoint) endpoints;
    QTAILQ_HEAD(, QemuSlirpRpcRegistration) registrations;
    QTAILQ_HEAD(, OncRpcRequest) requests;
    QTAILQ_HEAD(, QemuSlirpRpcTcpConnection) tcp_connections;
    bool valid;
    bool free_pending;
    unsigned refs;
    unsigned callback_depth;
};

struct QemuSlirpRpcRegistration {
    QTAILQ_ENTRY(QemuSlirpRpcRegistration) entry;
    QemuSlirpRpcRegistry *registry;
    QemuSlirpRpcEndpoint *udp_endpoint;
    QemuSlirpRpcEndpoint *tcp_endpoint;
    OncRpcProgram program;
    unsigned refs;
    bool linked;
    bool registry_ref;
    bool caller_ref;
};

struct QemuSlirpRpcEndpoint {
    QTAILQ_ENTRY(QemuSlirpRpcEndpoint) entry;
    QemuSlirpRpcRegistry *registry;
    QemuSlirpUdpListener *udp_listener;
    QemuSlirpStreamListener *tcp_listener;
    uint16_t port;
    unsigned transport;
    unsigned registrations;
    unsigned refs;
};

struct OncRpcRequest {
    QTAILQ_ENTRY(OncRpcRequest) entry;
    QemuSlirpRpcRegistry *registry;
    QemuSlirpUdpListener *udp_listener;
    QemuSlirpStream *stream;
    QemuSlirpRpcTcpConnection *tcp_connection;
    OncRpcCall call;
    struct sockaddr_in peer;
    uint8_t *data;
    size_t data_length;
    GByteArray *synthetic_reply;
    OncRpcRequest *callit_parent;
    uint16_t callit_port;
    unsigned refs;
    bool linked;
    bool registry_ref;
    bool tcp;
    bool synthetic;
    bool terminal;
    bool reply_queued;
};

struct QemuSlirpRpcTcpReply {
    QTAILQ_ENTRY(QemuSlirpRpcTcpReply) entry;
    OncRpcRequest *request;
    uint8_t *data;
    size_t length;
    size_t offset;
};

struct QemuSlirpRpcTcpConnection {
    QTAILQ_ENTRY(QemuSlirpRpcTcpConnection) entry;
    QemuSlirpRpcRegistry *registry;
    QemuSlirpRpcEndpoint *endpoint;
    QemuSlirpStream *stream;
    struct sockaddr_in peer;
    uint16_t port;
    OncRpcTcpRecordDecoder decoder;
    unsigned refs;
    bool linked;
    bool closed;
    bool callback_depth;
    QTAILQ_HEAD(, QemuSlirpRpcTcpReply) replies;
    size_t queued_bytes;
    bool flushing;
};

static void rpc_endpoint_ref(QemuSlirpRpcEndpoint *endpoint)
{
    g_assert(endpoint != NULL);
    g_assert(endpoint->refs != 0);
    endpoint->refs++;
}

static void rpc_endpoint_unref(QemuSlirpRpcEndpoint *endpoint)
{
    g_assert(endpoint != NULL);
    g_assert(endpoint->refs != 0);
    if (!--endpoint->refs) {
        g_free(endpoint);
    }
}

static void rpc_tcp_connection_ref(QemuSlirpRpcTcpConnection *connection)
{
    g_assert(connection != NULL);
    g_assert(connection->refs != 0);
    connection->refs++;
}

static void rpc_tcp_connection_unref(QemuSlirpRpcTcpConnection *connection)
{
    g_assert(connection != NULL);
    g_assert(connection->refs != 0);
    if (!--connection->refs) {
        rpc_endpoint_unref(connection->endpoint);
        g_free(connection);
    }
}

static void rpc_registry_ref(QemuSlirpRpcRegistry *registry)
{
    g_assert(registry != NULL);
    registry->refs++;
}

static void rpc_registry_maybe_free(QemuSlirpRpcRegistry *registry)
{
    if (registry->free_pending && registry->refs == 0 &&
        QTAILQ_EMPTY(&registry->registrations) &&
        QTAILQ_EMPTY(&registry->requests) &&
        QTAILQ_EMPTY(&registry->tcp_connections) &&
        QTAILQ_EMPTY(&registry->endpoints)) {
        g_free(registry);
    }
}

static void rpc_registry_unref(QemuSlirpRpcRegistry *registry)
{
    g_assert(registry != NULL);
    g_assert(registry->refs != 0);
    registry->refs--;
    rpc_registry_maybe_free(registry);
}

static void rpc_registration_ref(QemuSlirpRpcRegistration *registration)
{
    g_assert(registration != NULL);
    g_assert(registration->refs != 0);
    registration->refs++;
}

static void rpc_registration_unref(QemuSlirpRpcRegistration *registration)
{
    g_assert(registration != NULL);
    g_assert(registration->refs != 0);
    if (!--registration->refs) {
        g_free(registration);
    }
}

static void rpc_request_ref_internal(OncRpcRequest *request)
{
    g_assert(request != NULL);
    g_assert(request->refs != 0);
    request->refs++;
}

static void rpc_request_registry_detach(OncRpcRequest *request)
{
    QemuSlirpRpcRegistry *registry = request->registry;

    if (request->linked) {
        QTAILQ_REMOVE(&registry->requests, request, entry);
        request->linked = false;
    }
    request->udp_listener = NULL;
    request->stream = NULL;
    request->tcp_connection = NULL;
    if (request->registry_ref) {
        request->registry_ref = false;
        request->registry = NULL;
        rpc_registry_unref(registry);
    }
}

static void rpc_request_unref_internal(OncRpcRequest *request)
{
    g_assert(request != NULL);
    g_assert(request->refs != 0);
    if (--request->refs) {
        return;
    }
    if (request->linked || request->registry_ref) {
        rpc_request_registry_detach(request);
    }
    if (request->callit_parent) {
        onc_rpc_request_unref(request->callit_parent);
        request->callit_parent = NULL;
    }
    if (request->synthetic_reply) {
        g_byte_array_unref(request->synthetic_reply);
    }
    g_free(request->data);
    g_free(request);
}

static void rpc_request_terminal(OncRpcRequest *request)
{
    request->terminal = true;
    rpc_request_registry_detach(request);
}

static void rpc_tcp_reply_free(QemuSlirpRpcTcpReply *reply)
{
    OncRpcRequest *request = reply->request;

    request->reply_queued = false;
    onc_rpc_request_unref(request);
    g_free(reply->data);
    g_free(reply);
}

static void rpc_tcp_reply_remove_request(OncRpcRequest *request)
{
    QemuSlirpRpcTcpConnection *connection = request->tcp_connection;
    QemuSlirpRpcTcpReply *reply, *next;

    if (!connection || !request->reply_queued) {
        return;
    }
    QTAILQ_FOREACH_SAFE(reply, &connection->replies, entry, next) {
        if (reply->request != request) {
            continue;
        }
        QTAILQ_REMOVE(&connection->replies, reply, entry);
        connection->queued_bytes -= reply->length - reply->offset;
        rpc_tcp_reply_free(reply);
        return;
    }
    g_assert_not_reached();
}

static void rpc_request_cancel(OncRpcRequest *request)
{
    if (!request || request->terminal) {
        return;
    }
    rpc_tcp_reply_remove_request(request);
    rpc_request_terminal(request);
}

static OncRpcRequest *rpc_request_new(QemuSlirpRpcRegistry *registry,
                                      const uint8_t *data, size_t length,
                                      const struct sockaddr_in *peer,
                                      bool tcp,
                                      QemuSlirpUdpListener *udp_listener,
                                      QemuSlirpStream *stream,
                                      QemuSlirpRpcTcpConnection *tcp_connection)
{
    OncRpcRequest *request;

    request = g_new0(OncRpcRequest, 1);
    request->registry = registry;
    request->udp_listener = udp_listener;
    request->stream = stream;
    request->tcp_connection = tcp_connection;
    request->tcp = tcp;
    request->refs = 1;
    request->linked = true;
    request->registry_ref = true;
    if (peer) {
        request->peer = *peer;
    }
    request->data = g_memdup2(data, length);
    request->data_length = length;
    rpc_registry_ref(registry);
    QTAILQ_INSERT_TAIL(&registry->requests, request, entry);
    return request;
}

static OncRpcRequest *rpc_request_new_callit(
    QemuSlirpRpcRegistry *registry, OncRpcRequest *parent,
    const OncRpcCall *call, const uint8_t *args, size_t args_length)
{
    OncRpcRequest *request;

    request = rpc_request_new(registry, args, args_length, &parent->peer,
                              false, parent->udp_listener, NULL, NULL);
    request->synthetic = true;
    request->call = *call;
    request->call.data = request->data;
    request->call.data_length = args_length;
    onc_rpc_xdr_reader_init(&request->call.body, request->data, args_length);
    request->callit_parent = onc_rpc_request_ref(parent);
    request->callit_port = 0;
    request->synthetic_reply = g_byte_array_new();
    return request;
}

static QemuSlirpRpcRegistration *rpc_find_exact(
    QemuSlirpRpcRegistry *registry, uint32_t program, uint32_t version,
    unsigned transport, uint16_t port)
{
    QemuSlirpRpcRegistration *registration;

    QTAILQ_FOREACH(registration, &registry->registrations, entry) {
        if (registration->program.program == program &&
            version >= registration->program.version_low &&
            version <= registration->program.version_high &&
            (registration->program.transports & transport) &&
            (!port || registration->program.port == port)) {
            return registration;
        }
    }
    return NULL;
}

static bool rpc_find_version_range(QemuSlirpRpcRegistry *registry,
                                   uint32_t program, unsigned transport,
                                   uint16_t port,
                                   uint32_t *low, uint32_t *high)
{
    QemuSlirpRpcRegistration *registration;
    bool found = false;

    QTAILQ_FOREACH(registration, &registry->registrations, entry) {
        if (registration->program.program != program ||
            !(registration->program.transports & transport) ||
            (port && registration->program.port != port)) {
            continue;
        }
        if (!found || registration->program.version_low < *low) {
            *low = registration->program.version_low;
        }
        if (!found || registration->program.version_high > *high) {
            *high = registration->program.version_high;
        }
        found = true;
    }
    return found;
}

static unsigned rpc_transport_for_protocol(uint32_t protocol)
{
    if (protocol == IPPROTO_UDP) {
        return ONC_RPC_TRANSPORT_UDP;
    }
    if (protocol == IPPROTO_TCP) {
        return ONC_RPC_TRANSPORT_TCP;
    }
    return 0;
}

static bool rpc_request_send_bytes(OncRpcRequest *request,
                                   const uint8_t *data, size_t length);

static bool rpc_request_reply_synthetic(OncRpcRequest *request,
                                        const uint8_t *data, size_t length)
{
    OncRpcRequest *parent;
    OncRpcXdrReader reader;
    const uint8_t *verifier;
    size_t verifier_length;
    uint32_t xid, message_type, reply_status, verifier_flavor;
    uint32_t accept_status;
    uint8_t wrapped[ONC_RPC_MAX_DATAGRAM];
    OncRpcXdrWriter writer;
    size_t wrapped_length;
    bool result;

    if (!request->callit_parent || length > ONC_RPC_MAX_DATAGRAM) {
        return false;
    }
    parent = request->callit_parent;

    /*
     * The target callback returns a complete RPC reply.  CALLIT exports only
     * the successful procedure result, so validate and strip that envelope
     * before constructing the portmapper response.
     */
    onc_rpc_xdr_reader_init(&reader, data, length);
    if (!onc_rpc_xdr_u32(&reader, &xid) ||
        !onc_rpc_xdr_u32(&reader, &message_type) ||
        !onc_rpc_xdr_u32(&reader, &reply_status) ||
        !onc_rpc_xdr_u32(&reader, &verifier_flavor) ||
        !onc_rpc_xdr_counted_opaque(&reader, &verifier, &verifier_length,
                                    ONC_RPC_MAX_AUTH_BYTES) ||
        !onc_rpc_xdr_u32(&reader, &accept_status) ||
        xid != request->call.xid || message_type != ONC_RPC_REPLY ||
        reply_status != ONC_RPC_MSG_ACCEPTED ||
        verifier_flavor != ONC_RPC_AUTH_NULL || verifier_length != 0 ||
        accept_status != ONC_RPC_SUCCESS) {
        rpc_request_terminal(request);
        onc_rpc_request_drop(parent);
        request->callit_parent = NULL;
        onc_rpc_request_unref(parent);
        return false;
    }
    g_byte_array_set_size(request->synthetic_reply, 0);
    g_byte_array_append(request->synthetic_reply, reader.cursor,
                        onc_rpc_xdr_reader_remaining(&reader));
    rpc_request_terminal(request);

    onc_rpc_xdr_writer_init(&writer, wrapped, sizeof(wrapped));
    result = onc_rpc_reply_success(&writer, parent->call.xid) &&
             onc_rpc_xdr_put_u32(&writer, request->callit_port) &&
             onc_rpc_xdr_put_counted_opaque(
                 &writer, request->synthetic_reply->data,
                 request->synthetic_reply->len, ONC_RPC_MAX_DATAGRAM);
    if (!result) {
        onc_rpc_request_drop(parent);
    } else {
        wrapped_length = onc_rpc_xdr_writer_size(&writer);
        result = rpc_request_send_bytes(parent, wrapped, wrapped_length);
        if (!result) {
            onc_rpc_request_drop(parent);
        }
    }
    request->callit_parent = NULL;
    onc_rpc_request_unref(parent);
    return result;
}

static void rpc_tcp_reply_discard(QemuSlirpRpcTcpConnection *connection)
{
    QemuSlirpRpcTcpReply *reply;

    if (!connection) {
        return;
    }
    while ((reply = QTAILQ_FIRST(&connection->replies))) {
        QTAILQ_REMOVE(&connection->replies, reply, entry);
        connection->queued_bytes -= reply->length - reply->offset;
        rpc_request_terminal(reply->request);
        rpc_tcp_reply_free(reply);
    }
    g_assert(connection->queued_bytes == 0);
}

static bool rpc_tcp_flush(QemuSlirpRpcTcpConnection *connection)
{
    QemuSlirpRpcTcpReply *reply;
    QemuSlirpRpcRegistry *registry;
    bool result = true;
    bool open;

    if (!connection || connection->flushing) {
        return true;
    }
    rpc_tcp_connection_ref(connection);
    registry = connection->registry;
    if (registry) {
        rpc_registry_ref(registry);
    }
    connection->flushing = true;
    while (!connection->closed &&
           (reply = QTAILQ_FIRST(&connection->replies))) {
        QemuSlirpStream *stream = connection->stream;
        size_t space;
        size_t chunk;
        int ret;

        if (!stream) {
            result = false;
            break;
        }
        space = qemu_slirp_stream_can_send(stream);
        if (!space) {
            /*
             * The stream adapter reports readiness only after a send has
             * observed backpressure.  A full-capacity chunk can consume the
             * entire current window without setting that state, so probe the
             * head with one byte to arm the adapter for the next ACK.  A
             * conforming backend must reject this probe without accepting
             * any data when can_send() returned zero.
             */
            ret = qemu_slirp_stream_send(stream, reply->data + reply->offset,
                                         1);
            if (ret == -EAGAIN) {
                break;
            }
            result = false;
            break;
        }
        chunk = MIN(space, reply->length - reply->offset);
        ret = qemu_slirp_stream_send(stream, reply->data + reply->offset,
                                     chunk);
        if (ret == -EAGAIN) {
            break;
        }
        if (ret != 0 || connection->closed) {
            result = false;
            break;
        }
        reply->offset += chunk;
        connection->queued_bytes -= chunk;
        if (reply->offset == reply->length) {
            QTAILQ_REMOVE(&connection->replies, reply, entry);
            rpc_request_terminal(reply->request);
            rpc_tcp_reply_free(reply);
        }
    }
    connection->flushing = false;
    open = !connection->closed;
    if (registry) {
        rpc_registry_unref(registry);
    }
    rpc_tcp_connection_unref(connection);
    return result && open;
}

static bool rpc_request_send_bytes(OncRpcRequest *request,
                                   const uint8_t *data, size_t length)
{
    QemuSlirpRpcTcpConnection *connection;
    QemuSlirpRpcRegistry *registry;
    QemuSlirpRpcTcpReply *reply;
    uint8_t *record;
    size_t record_length;
    bool result;
    int ret;

    if (!request || request->terminal || !request->registry ||
        (length && !data)) {
        return false;
    }
    if (length > (request->tcp ? ONC_RPC_MAX_TCP_RECORD :
                  ONC_RPC_MAX_DATAGRAM)) {
        return false;
    }
    registry = request->registry;
    rpc_registry_ref(registry);
    if (request->tcp) {
        connection = request->tcp_connection;
        if (!connection || connection->closed || !request->stream ||
            request->reply_queued) {
            rpc_registry_unref(registry);
            return false;
        }
        if (length > SIZE_MAX - 4) {
            rpc_registry_unref(registry);
            return false;
        }
        record_length = length + 4;
        record = g_malloc(record_length);
        stl_be_p(record, UINT32_C(0x80000000) | length);
        if (length) {
            memcpy(record + 4, data, length);
        }
        rpc_tcp_connection_ref(connection);
        if (record_length > ONC_RPC_MAX_TCP_RECORD + 4 ||
            connection->queued_bytes >
                (ONC_RPC_MAX_TCP_RECORD + 4) - record_length) {
            QemuSlirpStream *stream = request->stream;

            g_free(record);
            qemu_slirp_stream_close(stream);
            if (!request->terminal) {
                rpc_request_terminal(request);
            }
            rpc_tcp_connection_unref(connection);
            rpc_registry_unref(registry);
            return false;
        }
        reply = g_new0(QemuSlirpRpcTcpReply, 1);
        reply->request = onc_rpc_request_ref(request);
        reply->data = record;
        reply->length = record_length;
        request->reply_queued = true;
        QTAILQ_INSERT_TAIL(&connection->replies, reply, entry);
        connection->queued_bytes += record_length;
        result = rpc_tcp_flush(connection);
        if (!result && !connection->closed) {
            QemuSlirpStream *stream = connection->stream;

            if (stream) {
                qemu_slirp_stream_close(stream);
            }
        }
        rpc_tcp_connection_unref(connection);
        rpc_registry_unref(registry);
        return result;
    } else {
        if (!request->udp_listener) {
            rpc_registry_unref(registry);
            return false;
        }
        ret = qemu_slirp_udp_send(request->udp_listener, &request->peer,
                                  data, length);
    }
    if (ret == 0) {
        rpc_request_terminal(request);
    } else if (request->tcp) {
        QemuSlirpStream *stream = request->stream;

        rpc_request_terminal(request);
        if (stream) {
            qemu_slirp_stream_close(stream);
        }
    }
    rpc_registry_unref(registry);
    return ret == 0;
}

static bool rpc_build_and_send(OncRpcRequest *request,
                               bool (*builder)(OncRpcXdrWriter *, uint32_t),
                               size_t capacity)
{
    uint8_t *buffer = g_malloc(capacity);
    OncRpcXdrWriter writer;
    bool result;

    onc_rpc_xdr_writer_init(&writer, buffer, capacity);
    result = builder(&writer, request->call.xid) &&
             rpc_request_send_bytes(request, buffer,
                                    onc_rpc_xdr_writer_size(&writer));
    g_free(buffer);
    return result;
}

static bool rpc_reply_prog_mismatch_request(OncRpcRequest *request,
                                            uint32_t low, uint32_t high)
{
    uint8_t buffer[32];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, buffer, sizeof(buffer));
    return onc_rpc_reply_prog_mismatch(&writer, request->call.xid, low, high) &&
           rpc_request_send_bytes(request, buffer,
                                  onc_rpc_xdr_writer_size(&writer));
}

static bool rpc_reply_success_u32(OncRpcRequest *request, uint32_t value)
{
    uint8_t buffer[32];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, buffer, sizeof(buffer));
    return onc_rpc_reply_success(&writer, request->call.xid) &&
           onc_rpc_xdr_put_u32(&writer, value) &&
           rpc_request_send_bytes(request, buffer,
                                  onc_rpc_xdr_writer_size(&writer));
}

static bool rpc_reply_success_bool(OncRpcRequest *request, bool value)
{
    uint8_t buffer[32];
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&writer, buffer, sizeof(buffer));
    return onc_rpc_reply_success(&writer, request->call.xid) &&
           onc_rpc_xdr_put_bool(&writer, value) &&
           rpc_request_send_bytes(request, buffer,
                                   onc_rpc_xdr_writer_size(&writer));
}

static bool rpc_dump_put_mapping(OncRpcXdrWriter *writer,
                                 const QemuSlirpRpcRegistration *registration,
                                 uint32_t version, uint32_t protocol)
{
    return onc_rpc_xdr_put_bool(writer, true) &&
           onc_rpc_xdr_put_u32(writer, registration->program.program) &&
           onc_rpc_xdr_put_u32(writer, version) &&
           onc_rpc_xdr_put_u32(writer, protocol) &&
           onc_rpc_xdr_put_u32(writer, registration->program.port);
}

static bool rpc_call_body_empty(const OncRpcCall *call)
{
    return onc_rpc_xdr_reader_empty(&call->body);
}

static OncRpcDispatchResult rpc_dispatch_callit(
    OncRpcRequest *request, QemuSlirpRpcRegistry *registry)
{
    OncRpcXdrReader body = request->call.body;
    const uint8_t *args;
    size_t args_length;
    uint32_t program, version, procedure;
    QemuSlirpRpcRegistration *registration;
    OncRpcCall target_call;
    OncRpcRequest *target_request;
    OncRpcDispatchResult result;

    if (!onc_rpc_xdr_u32(&body, &program) ||
        !onc_rpc_xdr_u32(&body, &version) ||
        !onc_rpc_xdr_u32(&body, &procedure) ||
        !onc_rpc_xdr_counted_opaque(&body, &args, &args_length,
                                    ONC_RPC_MAX_DATAGRAM) ||
        !onc_rpc_xdr_reader_empty(&body)) {
        rpc_build_and_send(request, onc_rpc_reply_garbage_args,
                           ONC_RPC_MAX_DATAGRAM);
        return ONC_RPC_DISPATCH_REPLIED;
    }
    registration = rpc_find_exact(registry, program, version,
                                  ONC_RPC_TRANSPORT_UDP, 0);
    if (!registration) {
        /* PMAP CALLIT deliberately has no error reply for an absent target. */
        onc_rpc_request_drop(request);
        return ONC_RPC_DISPATCH_DROP;
    }
    target_call = request->call;
    target_call.auth_flavor = ONC_RPC_AUTH_NULL;
    target_call.uid = 0;
    target_call.gid = 0;
    target_call.machine[0] = '\0';
    target_call.group_count = 0;
    memset(target_call.groups, 0, sizeof(target_call.groups));
    target_call.program = program;
    target_call.version = version;
    target_call.procedure = procedure;
    target_call.data = args;
    target_call.data_length = args_length;
    onc_rpc_xdr_reader_init(&target_call.body, args, args_length);
    target_request = rpc_request_new_callit(registry, request, &target_call,
                                            args, args_length);
    target_request->callit_port = registration->program.port;
    rpc_registration_ref(registration);
    registry->callback_depth++;
    result = registration->program.dispatch(target_request,
                                            &target_request->call,
                                            registration->program.opaque);
    registry->callback_depth--;
    rpc_registration_unref(registration);
    if (result == ONC_RPC_DISPATCH_ASYNC) {
        if (!target_request->terminal) {
            rpc_request_unref_internal(target_request);
            return ONC_RPC_DISPATCH_ASYNC;
        }
    } else if (result == ONC_RPC_DISPATCH_REPLIED &&
               !target_request->terminal) {
        onc_rpc_request_drop(target_request);
    } else if (result == ONC_RPC_DISPATCH_DROP) {
        onc_rpc_request_drop(target_request);
    }
    rpc_request_unref_internal(target_request);
    if (!request->terminal && !request->reply_queued) {
        onc_rpc_request_drop(request);
    }
    return ONC_RPC_DISPATCH_REPLIED;
}

static bool rpc_dispatch_portmap(OncRpcRequest *request,
                                 QemuSlirpRpcRegistry *registry)
{
    OncRpcXdrReader body = request->call.body;
    uint32_t program, version, protocol, port;
    unsigned transport;
    QemuSlirpRpcRegistration *registration;
    uint8_t *reply;
    size_t capacity = MIN((size_t)ONC_RPC_MAX_DATAGRAM,
                          (size_t)65536);
    OncRpcXdrWriter writer;

    switch (request->call.procedure) {
    case 0:
        if (!rpc_call_body_empty(&request->call)) {
            rpc_build_and_send(request, onc_rpc_reply_garbage_args, capacity);
        } else {
            rpc_build_and_send(request, onc_rpc_reply_success, capacity);
        }
        return ONC_RPC_DISPATCH_REPLIED;
    case 1:
    case 2:
        if (!onc_rpc_xdr_u32(&body, &program) ||
            !onc_rpc_xdr_u32(&body, &version) ||
            !onc_rpc_xdr_u32(&body, &protocol) ||
            !onc_rpc_xdr_u32(&body, &port) ||
            !onc_rpc_xdr_reader_empty(&body)) {
            rpc_build_and_send(request, onc_rpc_reply_garbage_args, capacity);
            return ONC_RPC_DISPATCH_REPLIED;
        }
        /* SET and UNSET are intentionally not exposed by this registry. */
        rpc_reply_success_bool(request, false);
        return ONC_RPC_DISPATCH_REPLIED;
    case 3:
        if (!onc_rpc_xdr_u32(&body, &program) ||
            !onc_rpc_xdr_u32(&body, &version) ||
            !onc_rpc_xdr_u32(&body, &protocol) ||
            !onc_rpc_xdr_u32(&body, &port) ||
            !onc_rpc_xdr_reader_empty(&body)) {
            rpc_build_and_send(request, onc_rpc_reply_garbage_args, capacity);
            return ONC_RPC_DISPATCH_REPLIED;
        }
        transport = rpc_transport_for_protocol(protocol);
        registration = transport ? rpc_find_exact(registry, program, version,
                                                   transport, 0) : NULL;
        rpc_reply_success_u32(request, registration ? registration->program.port
                                                    : 0);
        return ONC_RPC_DISPATCH_REPLIED;
    case 4:
        if (!rpc_call_body_empty(&request->call)) {
            rpc_build_and_send(request, onc_rpc_reply_garbage_args, capacity);
            return ONC_RPC_DISPATCH_REPLIED;
        }
        reply = g_malloc(capacity);
        onc_rpc_xdr_writer_init(&writer, reply, capacity);
        if (!onc_rpc_reply_success(&writer, request->call.xid)) {
            g_free(reply);
            onc_rpc_request_drop(request);
            return ONC_RPC_DISPATCH_DROP;
        }
        {
            QemuSlirpRpcRegistration *entry;
            bool dump_ok = true;

            QTAILQ_FOREACH(entry, &registry->registrations, entry) {
                uint32_t dump_version = entry->program.version_low;

                for (;;) {
                    if ((entry->program.transports & ONC_RPC_TRANSPORT_UDP) &&
                        !rpc_dump_put_mapping(&writer, entry, dump_version,
                                              IPPROTO_UDP)) {
                        dump_ok = false;
                        break;
                    }
                    if ((entry->program.transports & ONC_RPC_TRANSPORT_TCP) &&
                        !rpc_dump_put_mapping(&writer, entry, dump_version,
                                              IPPROTO_TCP)) {
                        dump_ok = false;
                        break;
                    }
                    if (dump_version == entry->program.version_high) {
                        break;
                    }
                    dump_version++;
                }
                if (!dump_ok) {
                    break;
                }
            }
            dump_ok = dump_ok && onc_rpc_xdr_put_bool(&writer, false);
            if (dump_ok) {
                rpc_request_send_bytes(request, reply,
                                       onc_rpc_xdr_writer_size(&writer));
            } else {
                onc_rpc_request_drop(request);
            }
        }
        g_free(reply);
        return ONC_RPC_DISPATCH_REPLIED;
    case 5:
        if (request->tcp) {
            rpc_build_and_send(request, onc_rpc_reply_proc_unavail, capacity);
            return ONC_RPC_DISPATCH_REPLIED;
        }
        return rpc_dispatch_callit(request, registry);
    default:
        rpc_build_and_send(request, onc_rpc_reply_proc_unavail, capacity);
        return ONC_RPC_DISPATCH_REPLIED;
    }
}

static bool rpc_request_is_portmap_endpoint(
    const OncRpcRequest *request, const QemuSlirpRpcRegistry *registry)
{
    if (request->tcp) {
        return request->tcp_connection && registry->tcp_portmap &&
               request->tcp_connection->port == ONC_RPC_PORTMAP_PORT &&
               request->tcp_connection->endpoint == registry->tcp_portmap;
    }
    return registry->udp_portmap && request->udp_listener &&
           request->udp_listener == registry->udp_portmap->udp_listener;
}

static void rpc_dispatch_packet(QemuSlirpRpcRegistry *registry,
                                const uint8_t *data, size_t length,
                                const struct sockaddr_in *peer, bool tcp,
                                uint16_t port,
                                QemuSlirpUdpListener *udp_listener,
                                QemuSlirpStream *stream,
                                QemuSlirpRpcTcpConnection *tcp_connection)
{
    OncRpcRequest *request;
    QemuSlirpRpcRegistration *registration;
    OncRpcDecodeResult decode;
    unsigned transport = tcp ? ONC_RPC_TRANSPORT_TCP : ONC_RPC_TRANSPORT_UDP;
    uint32_t low, high;
    uint32_t xid = length >= 4 ? ldl_be_p(data) : 0;
    uint8_t error_reply[32];
    OncRpcXdrWriter error_writer;
    OncRpcDispatchResult result;

    request = rpc_request_new(registry, data, length, peer, tcp,
                              udp_listener, stream, tcp_connection);
    decode = onc_rpc_decode_call_bounded(request->data, length,
                                         tcp ? ONC_RPC_MAX_TCP_RECORD :
                                               ONC_RPC_MAX_DATAGRAM,
                                         &request->call);
    if (decode != ONC_RPC_DECODE_OK) {
        onc_rpc_xdr_writer_init(&error_writer, error_reply,
                                sizeof(error_reply));
        switch (decode) {
        case ONC_RPC_DECODE_RPC_MISMATCH:
            onc_rpc_reply_rpc_mismatch(&error_writer, xid, ONC_RPC_VERSION,
                                       ONC_RPC_VERSION);
            break;
        case ONC_RPC_DECODE_AUTH_ERROR:
            onc_rpc_reply_auth_error(&error_writer, xid,
                                     ONC_RPC_AUTH_BADCRED);
            break;
        case ONC_RPC_DECODE_TOO_LARGE:
        case ONC_RPC_DECODE_GARBAGE_ARGS:
        default:
            onc_rpc_reply_garbage_args(&error_writer, xid);
            break;
        }
        if (onc_rpc_xdr_writer_size(&error_writer)) {
            rpc_request_send_bytes(request, error_reply,
                                   onc_rpc_xdr_writer_size(&error_writer));
        } else {
            onc_rpc_request_drop(request);
        }
        rpc_request_unref_internal(request);
        return;
    }

    if (request->call.program == ONC_RPC_PORTMAP_PROGRAM &&
        rpc_request_is_portmap_endpoint(request, registry)) {
        if (request->call.version != ONC_RPC_PORTMAP_VERSION) {
            rpc_reply_prog_mismatch_request(request, ONC_RPC_PORTMAP_VERSION,
                                            ONC_RPC_PORTMAP_VERSION);
        } else {
            rpc_dispatch_portmap(request, registry);
        }
        rpc_request_unref_internal(request);
        return;
    }
    registration = rpc_find_exact(registry, request->call.program,
                                  request->call.version, transport, port);
    if (!registration) {
        if (rpc_find_version_range(registry, request->call.program, transport,
                                   port,
                                   &low, &high)) {
            rpc_reply_prog_mismatch_request(request, low, high);
        } else {
            rpc_build_and_send(request, onc_rpc_reply_prog_unavail,
                               tcp ? 64 : 64);
        }
        rpc_request_unref_internal(request);
        return;
    }

    rpc_registration_ref(registration);
    registry->callback_depth++;
    result = registration->program.dispatch(request, &request->call,
                                            registration->program.opaque);
    registry->callback_depth--;
    rpc_registration_unref(registration);
    if ((result == ONC_RPC_DISPATCH_REPLIED ||
         result == ONC_RPC_DISPATCH_DROP) && !request->terminal &&
        !request->reply_queued) {
        onc_rpc_request_drop(request);
    }
    rpc_request_unref_internal(request);
}

static void rpc_udp_datagram(QemuSlirpUdpListener *listener,
                             const struct sockaddr_in *peer,
                             const uint8_t *data, size_t length, void *opaque)
{
    QemuSlirpRpcEndpoint *endpoint = opaque;
    QemuSlirpRpcRegistry *registry = endpoint ? endpoint->registry : NULL;

    if (endpoint) {
        rpc_endpoint_ref(endpoint);
    }
    if (!registry || !registry->valid ||
        endpoint->transport != ONC_RPC_TRANSPORT_UDP) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    rpc_registry_ref(registry);
    rpc_dispatch_packet(registry, data, length, peer, false, endpoint->port,
                        listener, NULL, NULL);
    rpc_registry_unref(registry);
    rpc_endpoint_unref(endpoint);
}

static void rpc_tcp_connected(QemuSlirpStream *stream,
                              const struct sockaddr_in *peer, void *opaque)
{
    QemuSlirpRpcEndpoint *endpoint = opaque;
    QemuSlirpRpcRegistry *registry = endpoint ? endpoint->registry : NULL;
    QemuSlirpRpcTcpConnection *connection;

    if (endpoint) {
        rpc_endpoint_ref(endpoint);
    }
    if (!registry || !registry->valid ||
        endpoint->transport != ONC_RPC_TRANSPORT_TCP) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    rpc_registry_ref(registry);
    connection = g_new0(QemuSlirpRpcTcpConnection, 1);
    connection->registry = registry;
    connection->endpoint = endpoint;
    rpc_endpoint_ref(endpoint);
    connection->stream = stream;
    connection->port = endpoint->port;
    if (peer) {
        connection->peer = *peer;
    }
    connection->refs = 1;
    connection->linked = true;
    onc_rpc_tcp_record_decoder_init(&connection->decoder,
                                   ONC_RPC_MAX_TCP_RECORD);
    QTAILQ_INIT(&connection->replies);
    QTAILQ_INSERT_TAIL(&registry->tcp_connections, connection, entry);
    rpc_endpoint_unref(endpoint);
}

static QemuSlirpRpcTcpConnection *rpc_tcp_find(
    QemuSlirpRpcRegistry *registry, QemuSlirpStream *stream)
{
    QemuSlirpRpcTcpConnection *connection;

    QTAILQ_FOREACH(connection, &registry->tcp_connections, entry) {
        if (connection->stream == stream) {
            return connection;
        }
    }
    return NULL;
}

static void rpc_tcp_finish_close(QemuSlirpRpcTcpConnection *connection);

static void rpc_tcp_can_send(QemuSlirpStream *stream, void *opaque)
{
    QemuSlirpRpcEndpoint *endpoint = opaque;
    QemuSlirpRpcRegistry *registry = endpoint ? endpoint->registry : NULL;
    QemuSlirpRpcTcpConnection *connection;
    bool flush_ok;

    if (endpoint) {
        rpc_endpoint_ref(endpoint);
    }
    if (!registry || !registry->valid) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    connection = rpc_tcp_find(registry, stream);
    if (!connection || connection->closed) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    rpc_registry_ref(registry);
    rpc_tcp_connection_ref(connection);
    connection->callback_depth = true;
    flush_ok = rpc_tcp_flush(connection);
    if (!flush_ok && !connection->closed) {
        qemu_slirp_stream_close(stream);
    }
    connection->callback_depth = false;
    rpc_tcp_finish_close(connection);
    rpc_tcp_connection_unref(connection);
    rpc_registry_unref(registry);
    rpc_endpoint_unref(endpoint);
}

static void rpc_tcp_finish_close(QemuSlirpRpcTcpConnection *connection)
{
    QemuSlirpRpcRegistry *registry;

    if (!connection || !connection->closed || connection->callback_depth ||
        !connection->linked) {
        return;
    }
    registry = connection->registry;
    QTAILQ_REMOVE(&registry->tcp_connections, connection, entry);
    connection->linked = false;
    onc_rpc_tcp_record_decoder_cleanup(&connection->decoder);
    connection->stream = NULL;
    /* Drop the persistent registry and connection references. */
    rpc_registry_unref(registry);
    rpc_tcp_connection_unref(connection);
}

static bool rpc_tcp_record(const uint8_t *data, size_t length, void *opaque)
{
    QemuSlirpRpcTcpConnection *connection = opaque;

    if (connection->closed || !connection->registry->valid) {
        return true;
    }
    rpc_dispatch_packet(connection->registry, data, length, &connection->peer,
                        true, connection->port, NULL, connection->stream,
                        connection);
    return true;
}

static void rpc_tcp_receive(QemuSlirpStream *stream, const uint8_t *data,
                            size_t length, void *opaque)
{
    QemuSlirpRpcEndpoint *endpoint = opaque;
    QemuSlirpRpcRegistry *registry = endpoint ? endpoint->registry : NULL;
    QemuSlirpRpcTcpConnection *connection;

    if (endpoint) {
        rpc_endpoint_ref(endpoint);
    }
    if (!registry || !registry->valid) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    connection = rpc_tcp_find(registry, stream);
    if (!connection || connection->closed) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    rpc_registry_ref(registry);
    rpc_tcp_connection_ref(connection);
    connection->callback_depth = true;
    if (!onc_rpc_tcp_record_decoder_feed(&connection->decoder, data, length,
                                         rpc_tcp_record, connection)) {
        qemu_slirp_stream_close(stream);
    }
    connection->callback_depth = false;
    rpc_tcp_finish_close(connection);
    rpc_tcp_connection_unref(connection);
    rpc_registry_unref(registry);
    rpc_endpoint_unref(endpoint);
}

static void rpc_tcp_closed(QemuSlirpStream *stream, void *opaque)
{
    QemuSlirpRpcEndpoint *endpoint = opaque;
    QemuSlirpRpcRegistry *registry = endpoint ? endpoint->registry : NULL;
    QemuSlirpRpcTcpConnection *connection;
    OncRpcRequest *request, *next;

    if (endpoint) {
        rpc_endpoint_ref(endpoint);
    }
    if (!registry) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    connection = rpc_tcp_find(registry, stream);
    if (!connection) {
        if (endpoint) {
            rpc_endpoint_unref(endpoint);
        }
        return;
    }
    rpc_registry_ref(registry);
    rpc_tcp_connection_ref(connection);
    connection->closed = true;
    rpc_tcp_reply_discard(connection);
    QTAILQ_FOREACH_SAFE(request, &registry->requests, entry, next) {
        if (request->tcp_connection == connection) {
            rpc_request_cancel(request);
        }
    }
    rpc_tcp_finish_close(connection);
    rpc_tcp_connection_unref(connection);
    rpc_registry_unref(registry);
    rpc_endpoint_unref(endpoint);
}

static const QemuSlirpUdpListenerOps rpc_udp_ops = {
    .datagram = rpc_udp_datagram,
};

static const QemuSlirpStreamOps rpc_stream_ops = {
    .connected = rpc_tcp_connected,
    .receive = rpc_tcp_receive,
    .can_send = rpc_tcp_can_send,
    .closed = rpc_tcp_closed,
};

QemuSlirpRpcRegistry *qemu_slirp_rpc_registry_new(
    QemuSlirpUdpRegistry *udp_registry,
    QemuSlirpStreamRegistry *stream_registry)
{
    QemuSlirpRpcRegistry *registry;

    if (!udp_registry || !stream_registry) {
        return NULL;
    }
    registry = g_new0(QemuSlirpRpcRegistry, 1);
    registry->udp_registry = udp_registry;
    registry->stream_registry = stream_registry;
    registry->valid = true;
    registry->refs = 1;
    QTAILQ_INIT(&registry->registrations);
    QTAILQ_INIT(&registry->endpoints);
    QTAILQ_INIT(&registry->requests);
    QTAILQ_INIT(&registry->tcp_connections);
    return registry;
}

static QemuSlirpRpcEndpoint *rpc_endpoint_find(
    QemuSlirpRpcRegistry *registry, uint16_t port, unsigned transport)
{
    QemuSlirpRpcEndpoint *endpoint;

    QTAILQ_FOREACH(endpoint, &registry->endpoints, entry) {
        if (endpoint->port == port && endpoint->transport == transport) {
            return endpoint;
        }
    }
    return NULL;
}

static QemuSlirpRpcEndpoint *rpc_endpoint_new(
    QemuSlirpRpcRegistry *registry, uint16_t port, unsigned transport,
    QemuSlirpUdpListenFlags udp_flags, Error **errp)
{
    QemuSlirpRpcEndpoint *endpoint;
    int ret;

    endpoint = g_new0(QemuSlirpRpcEndpoint, 1);
    endpoint->registry = registry;
    endpoint->port = port;
    endpoint->transport = transport;
    endpoint->refs = 1;
    QTAILQ_INSERT_TAIL(&registry->endpoints, endpoint, entry);

    if (transport == ONC_RPC_TRANSPORT_UDP) {
        ret = qemu_slirp_udp_registry_listen_full(
            registry->udp_registry, port, udp_flags, &rpc_udp_ops, endpoint,
            &endpoint->udp_listener, errp);
    } else {
        ret = qemu_slirp_stream_registry_listen(
            registry->stream_registry, port, &rpc_stream_ops, endpoint,
            &endpoint->tcp_listener, errp);
    }
    if (ret < 0) {
        QTAILQ_REMOVE(&registry->endpoints, endpoint, entry);
        rpc_endpoint_unref(endpoint);
        return NULL;
    }
    endpoint->registrations = 1;
    return endpoint;
}

static void rpc_cancel_udp_requests(QemuSlirpRpcRegistry *registry,
                                    QemuSlirpUdpListener *listener)
{
    OncRpcRequest *request, *next;

    QTAILQ_FOREACH_SAFE(request, &registry->requests, entry, next) {
        if (!request->tcp && request->udp_listener == listener) {
            rpc_request_cancel(request);
        }
    }
}

static void rpc_endpoint_release(QemuSlirpRpcEndpoint *endpoint)
{
    QemuSlirpRpcRegistry *registry;

    g_assert(endpoint != NULL);
    g_assert(endpoint->registrations != 0);
    if (--endpoint->registrations) {
        return;
    }
    registry = endpoint->registry;
    QTAILQ_REMOVE(&registry->endpoints, endpoint, entry);
    if (endpoint->udp_listener) {
        QemuSlirpUdpListener *listener = endpoint->udp_listener;

        rpc_cancel_udp_requests(registry, listener);
        endpoint->udp_listener = NULL;
        qemu_slirp_udp_listener_remove(listener);
    }
    if (endpoint->tcp_listener) {
        QemuSlirpStreamListener *listener = endpoint->tcp_listener;

        endpoint->tcp_listener = NULL;
        qemu_slirp_stream_listener_remove(listener);
    }
    rpc_endpoint_unref(endpoint);
}

static QemuSlirpRpcEndpoint *rpc_endpoint_acquire(
    QemuSlirpRpcRegistry *registry, uint16_t port, unsigned transport,
    Error **errp)
{
    QemuSlirpRpcEndpoint *endpoint;

    endpoint = rpc_endpoint_find(registry, port, transport);
    if (endpoint) {
        endpoint->registrations++;
        return endpoint;
    }
    return rpc_endpoint_new(registry, port, transport,
                            QEMU_SLIRP_UDP_LISTEN_DEFAULT, errp);
}

static int rpc_portmap_acquire(QemuSlirpRpcRegistry *registry, Error **errp)
{
    QemuSlirpRpcEndpoint *udp_endpoint;
    QemuSlirpRpcEndpoint *tcp_endpoint;

    if (registry->udp_portmap || registry->tcp_portmap) {
        g_assert(registry->udp_portmap && registry->tcp_portmap);
        return 0;
    }

    udp_endpoint = rpc_endpoint_new(
        registry, ONC_RPC_PORTMAP_PORT, ONC_RPC_TRANSPORT_UDP,
        QEMU_SLIRP_UDP_LISTEN_BROADCAST, errp);
    if (!udp_endpoint) {
        return -1;
    }

    tcp_endpoint = rpc_endpoint_new(
        registry, ONC_RPC_PORTMAP_PORT, ONC_RPC_TRANSPORT_TCP,
        QEMU_SLIRP_UDP_LISTEN_DEFAULT, errp);
    if (!tcp_endpoint) {
        rpc_endpoint_release(udp_endpoint);
        return -1;
    }

    registry->udp_portmap = udp_endpoint;
    registry->tcp_portmap = tcp_endpoint;
    return 0;
}

static void rpc_portmap_release(QemuSlirpRpcRegistry *registry)
{
    QemuSlirpRpcEndpoint *endpoint;

    if (registry->udp_portmap) {
        endpoint = registry->udp_portmap;
        registry->udp_portmap = NULL;
        rpc_endpoint_release(endpoint);
    }
    if (registry->tcp_portmap) {
        endpoint = registry->tcp_portmap;
        registry->tcp_portmap = NULL;
        rpc_endpoint_release(endpoint);
    }
}

int qemu_slirp_rpc_registry_register(
    QemuSlirpRpcRegistry *registry, const OncRpcProgram *program,
    QemuSlirpRpcRegistration **registration_out, Error **errp)
{
    QemuSlirpRpcRegistration *entry;
    QemuSlirpRpcEndpoint *udp_endpoint = NULL;
    QemuSlirpRpcEndpoint *tcp_endpoint = NULL;

    if (registration_out) {
        *registration_out = NULL;
    }
    if (!registry || !registry->valid) {
        error_setg(errp, "SLiRP RPC registry is invalid");
        return -1;
    }
    if (!registration_out || !program || !program->dispatch ||
        !program->program || !program->port ||
        program->port == ONC_RPC_PORTMAP_PORT ||
        program->version_low > program->version_high ||
        !program->transports ||
        (program->transports & ~(unsigned)(ONC_RPC_TRANSPORT_UDP |
                                           ONC_RPC_TRANSPORT_TCP)) ||
        program->program == ONC_RPC_PORTMAP_PROGRAM) {
        error_setg(errp, "Invalid SLiRP ONC RPC program registration");
        return -1;
    }
    QTAILQ_FOREACH(entry, &registry->registrations, entry) {
        if (entry->program.program == program->program &&
            entry->program.port == program->port &&
            (entry->program.transports & program->transports) &&
            entry->program.version_low <= program->version_high &&
            program->version_low <= entry->program.version_high) {
            error_setg(errp, "Conflicting SLiRP ONC RPC registration");
            return -1;
        }
    }

    if (program->transports & ONC_RPC_TRANSPORT_UDP) {
        udp_endpoint = rpc_endpoint_acquire(
            registry, program->port, ONC_RPC_TRANSPORT_UDP, errp);
        if (!udp_endpoint) {
            goto fail;
        }
    }
    if (program->transports & ONC_RPC_TRANSPORT_TCP) {
        tcp_endpoint = rpc_endpoint_acquire(
            registry, program->port, ONC_RPC_TRANSPORT_TCP, errp);
        if (!tcp_endpoint) {
            goto fail;
        }
    }
    if (QTAILQ_EMPTY(&registry->registrations) &&
        rpc_portmap_acquire(registry, errp) < 0) {
        goto fail;
    }
    entry = g_new0(QemuSlirpRpcRegistration, 1);
    entry->registry = registry;
    entry->udp_endpoint = udp_endpoint;
    entry->tcp_endpoint = tcp_endpoint;
    entry->program = *program;
    entry->refs = 2;
    entry->linked = true;
    entry->registry_ref = true;
    entry->caller_ref = true;
    rpc_registry_ref(registry);
    QTAILQ_INSERT_TAIL(&registry->registrations, entry, entry);
    *registration_out = entry;
    return 0;

fail:
    if (tcp_endpoint) {
        rpc_endpoint_release(tcp_endpoint);
    }
    if (udp_endpoint) {
        rpc_endpoint_release(udp_endpoint);
    }
    if (QTAILQ_EMPTY(&registry->registrations)) {
        rpc_portmap_release(registry);
    }
    return -1;
}

void qemu_slirp_rpc_registry_unregister(QemuSlirpRpcRegistration *entry)
{
    QemuSlirpRpcRegistry *registry;
    QemuSlirpRpcEndpoint *udp_endpoint;
    QemuSlirpRpcEndpoint *tcp_endpoint;

    if (!entry || !entry->caller_ref) {
        return;
    }
    entry->caller_ref = false;
    registry = entry->registry;
    if (entry->linked) {
        udp_endpoint = entry->udp_endpoint;
        tcp_endpoint = entry->tcp_endpoint;
        entry->udp_endpoint = NULL;
        entry->tcp_endpoint = NULL;
        rpc_registry_ref(registry);
        QTAILQ_REMOVE(&registry->registrations, entry, entry);
        entry->linked = false;
        entry->registry = NULL;
        entry->registry_ref = false;
        rpc_registry_unref(registry);
        if (tcp_endpoint) {
            rpc_endpoint_release(tcp_endpoint);
        }
        if (udp_endpoint) {
            rpc_endpoint_release(udp_endpoint);
        }
        if (QTAILQ_EMPTY(&registry->registrations)) {
            rpc_portmap_release(registry);
        }
        rpc_registration_unref(entry);
        rpc_registry_unref(registry);
    }
    rpc_registration_unref(entry);
}

void qemu_slirp_rpc_registry_invalidate(QemuSlirpRpcRegistry *registry)
{
    QemuSlirpRpcRegistration *entry;
    QemuSlirpRpcEndpoint *udp_endpoint;
    QemuSlirpRpcEndpoint *tcp_endpoint;
    OncRpcRequest *request, *next;
    QemuSlirpRpcTcpConnection *connection, *connection_next;

    if (!registry) {
        return;
    }
    rpc_registry_ref(registry);
    if (!registry->valid) {
        rpc_registry_unref(registry);
        return;
    }
    registry->valid = false;
    QTAILQ_FOREACH_SAFE(connection, &registry->tcp_connections, entry,
                        connection_next) {
        rpc_tcp_connection_ref(connection);
        rpc_tcp_reply_discard(connection);
        rpc_tcp_connection_unref(connection);
    }
    QTAILQ_FOREACH_SAFE(request, &registry->requests, entry, next) {
        rpc_request_cancel(request);
    }
    while (!QTAILQ_EMPTY(&registry->registrations)) {
        entry = QTAILQ_FIRST(&registry->registrations);
        udp_endpoint = entry->udp_endpoint;
        tcp_endpoint = entry->tcp_endpoint;
        entry->udp_endpoint = NULL;
        entry->tcp_endpoint = NULL;
        rpc_registration_ref(entry);
        QTAILQ_REMOVE(&registry->registrations, entry, entry);
        entry->linked = false;
        entry->registry = NULL;
        entry->registry_ref = false;
        rpc_registry_unref(registry);
        if (tcp_endpoint) {
            rpc_endpoint_release(tcp_endpoint);
        }
        if (udp_endpoint) {
            rpc_endpoint_release(udp_endpoint);
        }
        rpc_registration_unref(entry);
        rpc_registration_unref(entry);
    }
    rpc_portmap_release(registry);
    rpc_registry_unref(registry);
}

void qemu_slirp_rpc_registry_free(QemuSlirpRpcRegistry *registry)
{
    if (!registry) {
        return;
    }
    qemu_slirp_rpc_registry_invalidate(registry);
    registry->free_pending = true;
    rpc_registry_unref(registry);
}

const OncRpcCall *onc_rpc_request_call(const OncRpcRequest *request)
{
    return request && request->registry ? &request->call :
           request ? &request->call : NULL;
}

const uint8_t *onc_rpc_request_data(const OncRpcRequest *request,
                                    size_t *length)
{
    if (length) {
        *length = request ? request->data_length : 0;
    }
    return request ? request->data : NULL;
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
        rpc_request_ref_internal(request);
    }
    return request;
}

void onc_rpc_request_unref(OncRpcRequest *request)
{
    if (request) {
        rpc_request_unref_internal(request);
    }
}

bool onc_rpc_request_reply(OncRpcRequest *request, const uint8_t *data,
                           size_t length)
{
    bool result;

    if (!request || request->terminal || request->reply_queued ||
        (length && !data)) {
        return false;
    }
    rpc_request_ref_internal(request);
    if (request->synthetic) {
        result = rpc_request_reply_synthetic(request, data, length);
    } else {
        result = rpc_request_send_bytes(request, data, length);
        if (!result && !request->terminal && !request->reply_queued) {
            rpc_request_terminal(request);
        }
    }
    rpc_request_unref_internal(request);
    return result;
}

void onc_rpc_request_drop(OncRpcRequest *request)
{
    rpc_request_cancel(request);
}
