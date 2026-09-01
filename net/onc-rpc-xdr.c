/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "net/onc-rpc.h"
#include "qemu/bswap.h"

static bool xdr_padded_size(size_t length, size_t *padded)
{
    if (length > SIZE_MAX - 3) {
        return false;
    }
    *padded = (length + 3) & ~(size_t)3;
    return true;
}

static size_t xdr_writer_remaining(const OncRpcXdrWriter *writer)
{
    return writer->cursor <= writer->end ? writer->end - writer->cursor : 0;
}

void onc_rpc_xdr_reader_init(OncRpcXdrReader *reader, const void *data,
                             size_t length)
{
    reader->cursor = data;
    reader->end = data ? reader->cursor + length : NULL;
}

size_t onc_rpc_xdr_reader_remaining(const OncRpcXdrReader *reader)
{
    return reader && reader->cursor && reader->end &&
           reader->cursor <= reader->end ?
           reader->end - reader->cursor : 0;
}

bool onc_rpc_xdr_reader_empty(const OncRpcXdrReader *reader)
{
    return reader && reader->cursor == reader->end;
}

bool onc_rpc_xdr_u32(OncRpcXdrReader *reader, uint32_t *value)
{
    const uint8_t *cursor;

    if (!reader || !value || onc_rpc_xdr_reader_remaining(reader) < 4) {
        return false;
    }
    cursor = reader->cursor;
    *value = ldl_be_p(cursor);
    reader->cursor = cursor + 4;
    return true;
}

bool onc_rpc_xdr_i32(OncRpcXdrReader *reader, int32_t *value)
{
    uint32_t wire_value;

    if (!value || !onc_rpc_xdr_u32(reader, &wire_value)) {
        return false;
    }
    *value = (int32_t)wire_value;
    return true;
}

bool onc_rpc_xdr_bool(OncRpcXdrReader *reader, bool *value)
{
    OncRpcXdrReader temporary;
    uint32_t wire_value;

    if (!reader || !value) {
        return false;
    }
    temporary = *reader;
    if (!onc_rpc_xdr_u32(&temporary, &wire_value) || wire_value > 1) {
        return false;
    }
    *value = wire_value != 0;
    *reader = temporary;
    return true;
}

bool onc_rpc_xdr_opaque(OncRpcXdrReader *reader, uint8_t *out, size_t exact)
{
    const uint8_t *cursor;
    size_t padded;

    if (!reader || !xdr_padded_size(exact, &padded) ||
        onc_rpc_xdr_reader_remaining(reader) < padded) {
        return false;
    }
    cursor = reader->cursor;
    /* Padding carries no value; old clients may leave it uninitialized. */
    if (out && exact) {
        memcpy(out, cursor, exact);
    }
    reader->cursor = cursor + padded;
    return true;
}

bool onc_rpc_xdr_counted_opaque(OncRpcXdrReader *reader,
                                const uint8_t **out, size_t *length,
                                size_t maximum)
{
    OncRpcXdrReader temporary;
    uint32_t wire_length;
    size_t padded;

    if (!reader) {
        return false;
    }
    temporary = *reader;
    if (!onc_rpc_xdr_u32(&temporary, &wire_length) ||
        wire_length > maximum || !xdr_padded_size(wire_length, &padded) ||
        onc_rpc_xdr_reader_remaining(&temporary) < padded) {
        return false;
    }
    /* Padding carries no value; old clients may leave it uninitialized. */
    if (out) {
        *out = temporary.cursor;
    }
    if (length) {
        *length = wire_length;
    }
    temporary.cursor += padded;
    *reader = temporary;
    return true;
}

bool onc_rpc_xdr_counted_array(OncRpcXdrReader *reader,
                               const uint8_t **out, size_t *count,
                               size_t element_size, size_t maximum)
{
    OncRpcXdrReader temporary;
    uint32_t wire_count;
    size_t bytes;
    size_t padded;

    if (!reader) {
        return false;
    }
    temporary = *reader;
    if (!onc_rpc_xdr_u32(&temporary, &wire_count) || wire_count > maximum ||
        element_size % 4 || (wire_count && !element_size) ||
        (element_size && wire_count > SIZE_MAX / element_size)) {
        return false;
    }
    bytes = (size_t)wire_count * element_size;
    if (!xdr_padded_size(bytes, &padded) ||
        onc_rpc_xdr_reader_remaining(&temporary) < padded) {
        return false;
    }
    if (out) {
        *out = temporary.cursor;
    }
    if (count) {
        *count = wire_count;
    }
    temporary.cursor += padded;
    *reader = temporary;
    return true;
}

bool onc_rpc_xdr_string(OncRpcXdrReader *reader, char *out, size_t capacity,
                        size_t maximum)
{
    OncRpcXdrReader temporary;
    const uint8_t *value;
    size_t length;

    if (!reader || !out || !capacity) {
        return false;
    }
    temporary = *reader;
    if (!onc_rpc_xdr_counted_opaque(&temporary, &value, &length, maximum) ||
        length >= capacity || memchr(value, '\0', length)) {
        return false;
    }
    memcpy(out, value, length);
    out[length] = '\0';
    *reader = temporary;
    return true;
}

void onc_rpc_xdr_writer_init(OncRpcXdrWriter *writer, void *data,
                             size_t length)
{
    writer->start = data;
    writer->cursor = data;
    writer->end = data ? writer->cursor + length : NULL;
}

size_t onc_rpc_xdr_writer_size(const OncRpcXdrWriter *writer)
{
    return writer && writer->cursor && writer->start &&
           writer->cursor >= writer->start ?
           writer->cursor - writer->start : 0;
}

bool onc_rpc_xdr_put_u32(OncRpcXdrWriter *writer, uint32_t value)
{
    if (!writer || xdr_writer_remaining(writer) < 4) {
        return false;
    }
    stl_be_p(writer->cursor, value);
    writer->cursor += 4;
    return true;
}

bool onc_rpc_xdr_put_i32(OncRpcXdrWriter *writer, int32_t value)
{
    return onc_rpc_xdr_put_u32(writer, (uint32_t)value);
}

bool onc_rpc_xdr_put_bool(OncRpcXdrWriter *writer, bool value)
{
    return onc_rpc_xdr_put_u32(writer, value ? 1 : 0);
}

bool onc_rpc_xdr_put_opaque(OncRpcXdrWriter *writer, const void *data,
                            size_t length)
{
    size_t padded;

    if (!writer || (length && !data) || !xdr_padded_size(length, &padded) ||
        xdr_writer_remaining(writer) < padded) {
        return false;
    }
    if (length) {
        memcpy(writer->cursor, data, length);
    }
    memset(writer->cursor + length, 0, padded - length);
    writer->cursor += padded;
    return true;
}

bool onc_rpc_xdr_put_counted_opaque(OncRpcXdrWriter *writer,
                                    const void *data, size_t length,
                                    size_t maximum)
{
    OncRpcXdrWriter temporary;
    size_t padded;

    if (!writer || length > maximum || length > UINT32_MAX ||
        (length && !data) || !xdr_padded_size(length, &padded) ||
        padded > SIZE_MAX - 4) {
        return false;
    }
    temporary = *writer;
    if (!onc_rpc_xdr_put_u32(&temporary, length) ||
        !onc_rpc_xdr_put_opaque(&temporary, data, length)) {
        return false;
    }
    *writer = temporary;
    return true;
}

bool onc_rpc_xdr_put_counted_array(OncRpcXdrWriter *writer, const void *data,
                                   size_t count, size_t element_size,
                                   size_t maximum)
{
    OncRpcXdrWriter temporary;
    size_t bytes;

    if (!writer || count > maximum || count > UINT32_MAX ||
        element_size % 4 || (count && !element_size) ||
        (element_size && count > SIZE_MAX / element_size)) {
        return false;
    }
    bytes = count * element_size;
    if (bytes && !data) {
        return false;
    }
    temporary = *writer;
    if (!onc_rpc_xdr_put_u32(&temporary, count) ||
        !onc_rpc_xdr_put_opaque(&temporary, data, bytes)) {
        return false;
    }
    *writer = temporary;
    return true;
}

bool onc_rpc_xdr_put_string(OncRpcXdrWriter *writer, const char *value,
                            size_t maximum)
{
    size_t length;

    if (!value) {
        return false;
    }
    length = strlen(value);
    return onc_rpc_xdr_put_counted_opaque(writer, value, length, maximum);
}

static bool decode_auth_blob(OncRpcXdrReader *reader, uint32_t *flavor,
                             const uint8_t **data, size_t *length,
                             bool *oversized)
{
    OncRpcXdrReader temporary;
    uint32_t wire_length;

    *oversized = false;
    temporary = *reader;
    if (!onc_rpc_xdr_u32(&temporary, flavor) ||
        !onc_rpc_xdr_u32(&temporary, &wire_length)) {
        return false;
    }
    if (wire_length > ONC_RPC_MAX_AUTH_BYTES) {
        *oversized = true;
        return false;
    }
    temporary.cursor -= 4;
    if (!onc_rpc_xdr_counted_opaque(&temporary, data, length,
                                    ONC_RPC_MAX_AUTH_BYTES)) {
        return false;
    }
    *reader = temporary;
    return true;
}

static bool decode_auth_sys(const uint8_t *data, size_t length,
                            OncRpcCall *call)
{
    OncRpcXdrReader auth;
    uint32_t stamp;
    uint32_t group_count;

    onc_rpc_xdr_reader_init(&auth, data, length);
    if (!onc_rpc_xdr_u32(&auth, &stamp) ||
        !onc_rpc_xdr_string(&auth, call->machine, sizeof(call->machine),
                            ONC_RPC_MAX_AUTH_MACHINE) ||
        !onc_rpc_xdr_u32(&auth, &call->uid) ||
        !onc_rpc_xdr_u32(&auth, &call->gid) ||
        !onc_rpc_xdr_u32(&auth, &group_count) ||
        group_count > ONC_RPC_MAX_AUTH_GROUPS) {
        return false;
    }
    (void)stamp;
    for (size_t i = 0; i < group_count; i++) {
        if (!onc_rpc_xdr_u32(&auth, &call->groups[i])) {
            return false;
        }
    }
    call->group_count = group_count;
    return onc_rpc_xdr_reader_empty(&auth);
}

OncRpcDecodeResult onc_rpc_decode_call_bounded(const uint8_t *data,
                                               size_t length,
                                               size_t maximum,
                                               OncRpcCall *call)
{
    OncRpcXdrReader reader;
    const uint8_t *credential;
    const uint8_t *verifier;
    size_t credential_length;
    size_t verifier_length;
    uint32_t message_type;
    uint32_t rpc_version;
    uint32_t verifier_flavor;
    bool oversized = false;

    if (!data || !call) {
        return ONC_RPC_DECODE_GARBAGE_ARGS;
    }
    maximum = MIN(maximum, (size_t)ONC_RPC_MAX_TCP_RECORD);
    if (length > maximum) {
        return ONC_RPC_DECODE_TOO_LARGE;
    }
    memset(call, 0, sizeof(*call));
    call->data = data;
    call->data_length = length;
    onc_rpc_xdr_reader_init(&reader, data, length);
    if (!onc_rpc_xdr_u32(&reader, &call->xid) ||
        !onc_rpc_xdr_u32(&reader, &message_type) ||
        !onc_rpc_xdr_u32(&reader, &rpc_version)) {
        return ONC_RPC_DECODE_GARBAGE_ARGS;
    }
    if (message_type != ONC_RPC_CALL) {
        return ONC_RPC_DECODE_GARBAGE_ARGS;
    }
    if (rpc_version != ONC_RPC_VERSION) {
        return ONC_RPC_DECODE_RPC_MISMATCH;
    }
    if (!onc_rpc_xdr_u32(&reader, &call->program) ||
        !onc_rpc_xdr_u32(&reader, &call->version) ||
        !onc_rpc_xdr_u32(&reader, &call->procedure) ||
        !decode_auth_blob(&reader, &call->auth_flavor, &credential,
                          &credential_length, &oversized)) {
        return oversized ? ONC_RPC_DECODE_AUTH_ERROR :
                           ONC_RPC_DECODE_GARBAGE_ARGS;
    }
    switch (call->auth_flavor) {
    case ONC_RPC_AUTH_NULL:
        if (credential_length != 0) {
            return ONC_RPC_DECODE_AUTH_ERROR;
        }
        break;
    case ONC_RPC_AUTH_SYS:
        if (!decode_auth_sys(credential, credential_length, call)) {
            return ONC_RPC_DECODE_AUTH_ERROR;
        }
        break;
    default:
        return ONC_RPC_DECODE_AUTH_ERROR;
    }
    if (!decode_auth_blob(&reader, &verifier_flavor, &verifier,
                          &verifier_length, &oversized)) {
        return oversized ? ONC_RPC_DECODE_AUTH_ERROR :
                           ONC_RPC_DECODE_GARBAGE_ARGS;
    }
    if (verifier_flavor != ONC_RPC_AUTH_NULL || verifier_length != 0) {
        return ONC_RPC_DECODE_AUTH_ERROR;
    }
    call->body = reader;
    return ONC_RPC_DECODE_OK;
}

OncRpcDecodeResult onc_rpc_decode_call(const uint8_t *data, size_t length,
                                       OncRpcCall *call)
{
    return onc_rpc_decode_call_bounded(data, length, ONC_RPC_MAX_DATAGRAM,
                                       call);
}

static bool rpc_reply_accepted(OncRpcXdrWriter *writer, uint32_t xid,
                               OncRpcAcceptStatus status, bool mismatch,
                               uint32_t low, uint32_t high)
{
    OncRpcXdrWriter temporary;
    size_t words = mismatch ? 8 : 6;

    if (!writer || words > SIZE_MAX / 4 ||
        xdr_writer_remaining(writer) < words * 4) {
        return false;
    }
    temporary = *writer;
    if (!onc_rpc_xdr_put_u32(&temporary, xid) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_REPLY) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_MSG_ACCEPTED) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_AUTH_NULL) ||
        !onc_rpc_xdr_put_u32(&temporary, 0) ||
        !onc_rpc_xdr_put_u32(&temporary, status) ||
        (mismatch && (!onc_rpc_xdr_put_u32(&temporary, low) ||
                      !onc_rpc_xdr_put_u32(&temporary, high)))) {
        return false;
    }
    *writer = temporary;
    return true;
}

bool onc_rpc_reply_success(OncRpcXdrWriter *writer, uint32_t xid)
{
    return rpc_reply_accepted(writer, xid, ONC_RPC_SUCCESS, false, 0, 0);
}

bool onc_rpc_reply_prog_unavail(OncRpcXdrWriter *writer, uint32_t xid)
{
    return rpc_reply_accepted(writer, xid, ONC_RPC_PROG_UNAVAIL, false, 0, 0);
}

bool onc_rpc_reply_prog_mismatch(OncRpcXdrWriter *writer, uint32_t xid,
                                 uint32_t low, uint32_t high)
{
    return rpc_reply_accepted(writer, xid, ONC_RPC_PROG_MISMATCH, true, low,
                              high);
}

bool onc_rpc_reply_proc_unavail(OncRpcXdrWriter *writer, uint32_t xid)
{
    return rpc_reply_accepted(writer, xid, ONC_RPC_PROC_UNAVAIL, false, 0, 0);
}

bool onc_rpc_reply_garbage_args(OncRpcXdrWriter *writer, uint32_t xid)
{
    return rpc_reply_accepted(writer, xid, ONC_RPC_GARBAGE_ARGS, false, 0, 0);
}

bool onc_rpc_reply_system_err(OncRpcXdrWriter *writer, uint32_t xid)
{
    return rpc_reply_accepted(writer, xid, ONC_RPC_SYSTEM_ERR, false, 0, 0);
}

bool onc_rpc_reply_rpc_mismatch(OncRpcXdrWriter *writer, uint32_t xid,
                                uint32_t low, uint32_t high)
{
    OncRpcXdrWriter temporary;

    if (!writer || xdr_writer_remaining(writer) < 6 * 4) {
        return false;
    }
    temporary = *writer;
    if (!onc_rpc_xdr_put_u32(&temporary, xid) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_REPLY) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_MSG_DENIED) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_REJECT_MISMATCH) ||
        !onc_rpc_xdr_put_u32(&temporary, low) ||
        !onc_rpc_xdr_put_u32(&temporary, high)) {
        return false;
    }
    *writer = temporary;
    return true;
}

bool onc_rpc_reply_auth_error(OncRpcXdrWriter *writer, uint32_t xid,
                              OncRpcAuthStatus status)
{
    OncRpcXdrWriter temporary;

    if (!writer || xdr_writer_remaining(writer) < 5 * 4) {
        return false;
    }
    temporary = *writer;
    if (!onc_rpc_xdr_put_u32(&temporary, xid) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_REPLY) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_MSG_DENIED) ||
        !onc_rpc_xdr_put_u32(&temporary, ONC_RPC_REJECT_AUTH_ERROR) ||
        !onc_rpc_xdr_put_u32(&temporary, status)) {
        return false;
    }
    *writer = temporary;
    return true;
}

void onc_rpc_tcp_record_decoder_init(OncRpcTcpRecordDecoder *decoder,
                                     size_t maximum_record)
{
    memset(decoder, 0, sizeof(*decoder));
    decoder->maximum_record = maximum_record ?
                              MIN(maximum_record, ONC_RPC_MAX_TCP_RECORD) :
                              ONC_RPC_MAX_TCP_RECORD;
}

static bool record_reserve(OncRpcTcpRecordDecoder *decoder, size_t length)
{
    size_t capacity;

    if (length <= decoder->record_capacity) {
        return true;
    }
    capacity = decoder->record_capacity ? decoder->record_capacity :
               MIN((size_t)4096, decoder->maximum_record);
    while (capacity < length) {
        if (capacity > decoder->maximum_record / 2) {
            capacity = decoder->maximum_record;
            break;
        }
        capacity *= 2;
    }
    if (capacity < length || capacity > decoder->maximum_record) {
        return false;
    }
    decoder->record = g_realloc(decoder->record, capacity);
    decoder->record_capacity = capacity;
    return true;
}

static bool record_deliver(OncRpcTcpRecordDecoder *decoder,
                           OncRpcRecordCallback callback, void *opaque)
{
    const uint8_t *record = decoder->record;
    size_t record_length = decoder->record_length;
    bool accepted;

    if (!callback) {
        decoder->failed = true;
        return false;
    }
    /* Keep the callback's borrowed bytes valid, but commit parser state. */
    decoder->record_length = 0;
    decoder->record_active = false;
    decoder->fragment_final = false;
    decoder->callback_active = true;
    accepted = callback(record, record_length, opaque);
    decoder->callback_active = false;
    if (!accepted) {
        decoder->failed = true;
    }
    return accepted;
}

bool onc_rpc_tcp_record_decoder_feed(OncRpcTcpRecordDecoder *decoder,
                                     const uint8_t *data, size_t length,
                                     OncRpcRecordCallback callback,
                                     void *opaque)
{
    if (!decoder || decoder->failed || decoder->callback_active ||
        (length && !data) || (length && !callback)) {
        return false;
    }
    while (length) {
        size_t chunk;

        if (!decoder->fragment_remaining) {
            while (decoder->marker_length < sizeof(decoder->marker) &&
                   length) {
                decoder->marker[decoder->marker_length++] = *data++;
                length--;
            }
            if (decoder->marker_length < sizeof(decoder->marker)) {
                return true;
            }
            {
                uint32_t marker = ldl_be_p(decoder->marker);
                size_t fragment_length = marker & UINT32_C(0x7fffffff);

                decoder->fragment_final = marker & UINT32_C(0x80000000);
                decoder->marker_length = 0;
                if (fragment_length > decoder->maximum_record ||
                    decoder->record_length >
                    decoder->maximum_record - fragment_length ||
                    !record_reserve(decoder,
                                    decoder->record_length + fragment_length)) {
                    decoder->failed = true;
                    return false;
                }
                decoder->fragment_remaining = fragment_length;
                decoder->record_active = true;
                if (!fragment_length && decoder->fragment_final) {
                    if (!record_deliver(decoder, callback, opaque)) {
                        return false;
                    }
                }
                if (!decoder->fragment_remaining) {
                    continue;
                }
            }
        }

        chunk = MIN(decoder->fragment_remaining, length);
        memcpy(decoder->record + decoder->record_length, data, chunk);
        decoder->record_length += chunk;
        decoder->fragment_remaining -= chunk;
        data += chunk;
        length -= chunk;
        if (decoder->fragment_remaining) {
            continue;
        }
        if (decoder->fragment_final) {
            if (!record_deliver(decoder, callback, opaque)) {
                return false;
            }
        }
    }
    return true;
}

bool onc_rpc_tcp_record_decoder_finish(const OncRpcTcpRecordDecoder *decoder)
{
    return decoder && !decoder->failed && !decoder->callback_active &&
           !decoder->marker_length &&
           !decoder->fragment_remaining && !decoder->record_active &&
           !decoder->record_length;
}

void onc_rpc_tcp_record_decoder_cleanup(OncRpcTcpRecordDecoder *decoder)
{
    if (!decoder || decoder->callback_active) {
        return;
    }
    g_free(decoder->record);
    memset(decoder, 0, sizeof(*decoder));
}
