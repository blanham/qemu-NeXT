/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QEMU_NET_ONC_RPC_H
#define QEMU_NET_ONC_RPC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ONC RPC v2 and the bounds shared by the in-process services. */
#define ONC_RPC_VERSION 2U
#define ONC_RPC_MAX_DATAGRAM (32U * 1024U)
#define ONC_RPC_MAX_TCP_RECORD (1U * 1024U * 1024U)
#define ONC_RPC_MAX_AUTH_MACHINE 255U
#define ONC_RPC_MAX_AUTH_GROUPS 16U
#define ONC_RPC_MAX_AUTH_BYTES 400U

typedef enum OncRpcMessageType {
    ONC_RPC_CALL = 0,
    ONC_RPC_REPLY = 1,
} OncRpcMessageType;

typedef enum OncRpcReplyStatus {
    ONC_RPC_MSG_ACCEPTED = 0,
    ONC_RPC_MSG_DENIED = 1,
} OncRpcReplyStatus;

typedef enum OncRpcAcceptStatus {
    ONC_RPC_SUCCESS = 0,
    ONC_RPC_PROG_UNAVAIL = 1,
    ONC_RPC_PROG_MISMATCH = 2,
    ONC_RPC_PROC_UNAVAIL = 3,
    ONC_RPC_GARBAGE_ARGS = 4,
    ONC_RPC_SYSTEM_ERR = 5,
} OncRpcAcceptStatus;

typedef enum OncRpcRejectStatus {
    ONC_RPC_REJECT_MISMATCH = 0,
    ONC_RPC_REJECT_AUTH_ERROR = 1,
} OncRpcRejectStatus;

typedef enum OncRpcAuthFlavor {
    ONC_RPC_AUTH_NULL = 0,
    ONC_RPC_AUTH_SYS = 1,
} OncRpcAuthFlavor;

typedef enum OncRpcAuthStatus {
    ONC_RPC_AUTH_OK = 0,
    ONC_RPC_AUTH_BADCRED = 1,
    ONC_RPC_AUTH_REJECTEDCRED = 2,
    ONC_RPC_AUTH_BADVERF = 3,
    ONC_RPC_AUTH_REJECTEDVERF = 4,
    ONC_RPC_AUTH_TOOWEAK = 5,
    ONC_RPC_AUTH_INVALIDRESP = 6,
    ONC_RPC_AUTH_FAILED = 7,
} OncRpcAuthStatus;

typedef enum OncRpcDecodeResult {
    ONC_RPC_DECODE_OK,
    ONC_RPC_DECODE_GARBAGE_ARGS,
    ONC_RPC_DECODE_RPC_MISMATCH,
    ONC_RPC_DECODE_AUTH_ERROR,
    ONC_RPC_DECODE_TOO_LARGE,
} OncRpcDecodeResult;

typedef struct OncRpcXdrReader {
    const uint8_t *cursor;
    const uint8_t *end;
} OncRpcXdrReader;

typedef struct OncRpcXdrWriter {
    uint8_t *start;
    uint8_t *cursor;
    uint8_t *end;
} OncRpcXdrWriter;

void onc_rpc_xdr_reader_init(OncRpcXdrReader *reader, const void *data,
                             size_t length);
size_t onc_rpc_xdr_reader_remaining(const OncRpcXdrReader *reader);
bool onc_rpc_xdr_reader_empty(const OncRpcXdrReader *reader);
bool onc_rpc_xdr_u32(OncRpcXdrReader *reader, uint32_t *value);
bool onc_rpc_xdr_i32(OncRpcXdrReader *reader, int32_t *value);
bool onc_rpc_xdr_bool(OncRpcXdrReader *reader, bool *value);
bool onc_rpc_xdr_opaque(OncRpcXdrReader *reader, uint8_t *out,
                        size_t exact);
bool onc_rpc_xdr_counted_opaque(OncRpcXdrReader *reader,
                                const uint8_t **out, size_t *length,
                                size_t maximum);
/*
 * element_size is the fixed, already-XDR-encoded stride of each element.  A
 * non-zero stride must be a multiple of four; variable-size elements must be
 * encoded one at a time instead.
 */
bool onc_rpc_xdr_counted_array(OncRpcXdrReader *reader,
                               const uint8_t **out, size_t *count,
                               size_t element_size, size_t maximum);
bool onc_rpc_xdr_string(OncRpcXdrReader *reader, char *out, size_t capacity,
                        size_t maximum);

void onc_rpc_xdr_writer_init(OncRpcXdrWriter *writer, void *data,
                             size_t length);
size_t onc_rpc_xdr_writer_size(const OncRpcXdrWriter *writer);
bool onc_rpc_xdr_put_u32(OncRpcXdrWriter *writer, uint32_t value);
bool onc_rpc_xdr_put_i32(OncRpcXdrWriter *writer, int32_t value);
bool onc_rpc_xdr_put_bool(OncRpcXdrWriter *writer, bool value);
bool onc_rpc_xdr_put_opaque(OncRpcXdrWriter *writer, const void *data,
                            size_t length);
bool onc_rpc_xdr_put_counted_opaque(OncRpcXdrWriter *writer,
                                    const void *data, size_t length,
                                    size_t maximum);
/* element_size has the same already-XDR-encoded, four-byte stride contract. */
bool onc_rpc_xdr_put_counted_array(OncRpcXdrWriter *writer, const void *data,
                                   size_t count, size_t element_size,
                                   size_t maximum);
bool onc_rpc_xdr_put_string(OncRpcXdrWriter *writer, const char *value,
                            size_t maximum);

typedef struct OncRpcCall {
    uint32_t xid;
    uint32_t program;
    uint32_t version;
    uint32_t procedure;
    uint32_t auth_flavor;
    uint32_t uid;
    uint32_t gid;
    char machine[ONC_RPC_MAX_AUTH_MACHINE + 1];
    uint32_t groups[ONC_RPC_MAX_AUTH_GROUPS];
    size_t group_count;
    const uint8_t *data;
    size_t data_length;
    OncRpcXdrReader body;
} OncRpcCall;

OncRpcDecodeResult onc_rpc_decode_call(const uint8_t *data, size_t length,
                                       OncRpcCall *call);
/*
 * Decode a call subject to an explicit transport record limit.  The limit is
 * capped at ONC_RPC_MAX_TCP_RECORD; callers should select the limit that
 * applies to their transport (the wrapper above uses the UDP limit).
 */
OncRpcDecodeResult onc_rpc_decode_call_bounded(const uint8_t *data,
                                               size_t length,
                                               size_t maximum,
                                               OncRpcCall *call);

bool onc_rpc_reply_success(OncRpcXdrWriter *writer, uint32_t xid);
bool onc_rpc_reply_prog_unavail(OncRpcXdrWriter *writer, uint32_t xid);
bool onc_rpc_reply_prog_mismatch(OncRpcXdrWriter *writer, uint32_t xid,
                                 uint32_t low, uint32_t high);
bool onc_rpc_reply_proc_unavail(OncRpcXdrWriter *writer, uint32_t xid);
bool onc_rpc_reply_garbage_args(OncRpcXdrWriter *writer, uint32_t xid);
bool onc_rpc_reply_system_err(OncRpcXdrWriter *writer, uint32_t xid);
bool onc_rpc_reply_rpc_mismatch(OncRpcXdrWriter *writer, uint32_t xid,
                                uint32_t low, uint32_t high);
bool onc_rpc_reply_auth_error(OncRpcXdrWriter *writer, uint32_t xid,
                              OncRpcAuthStatus status);

typedef bool (*OncRpcRecordCallback)(const uint8_t *data, size_t length,
                                     void *opaque);

/*
 * Stateful RFC 5531 record-marker decoder for a single TCP connection.
 *
 * A record callback receives a borrowed pointer that is valid only while the
 * callback runs.  The callback must not reenter feed(), finish(), or
 * cleanup() on this decoder; such reentry is rejected (cleanup is a no-op
 * until the caller retries it).  Delivery state is committed before the
 * callback starts.  The decoder and any containing owner storage must remain
 * alive until onc_rpc_tcp_record_decoder_feed() returns; if the callback
 * synchronously tears down that owner, defer destruction until feed() has
 * returned.
 */
typedef struct OncRpcTcpRecordDecoder {
    uint8_t marker[4];
    size_t marker_length;
    uint8_t *record;
    size_t record_length;
    size_t record_capacity;
    size_t fragment_remaining;
    size_t maximum_record;
    bool fragment_final;
    bool record_active;
    bool failed;
    bool callback_active;
} OncRpcTcpRecordDecoder;

void onc_rpc_tcp_record_decoder_init(OncRpcTcpRecordDecoder *decoder,
                                     size_t maximum_record);
bool onc_rpc_tcp_record_decoder_feed(OncRpcTcpRecordDecoder *decoder,
                                     const uint8_t *data, size_t length,
                                     OncRpcRecordCallback callback,
                                     void *opaque);
bool onc_rpc_tcp_record_decoder_finish(const OncRpcTcpRecordDecoder *decoder);
void onc_rpc_tcp_record_decoder_cleanup(OncRpcTcpRecordDecoder *decoder);

#endif /* QEMU_NET_ONC_RPC_H */
