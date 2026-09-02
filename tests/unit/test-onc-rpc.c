/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "net/onc-rpc.h"
#include "net/slirp-rpc-internal.h"
#include "net/slirp-stream-internal.h"
#include "net/slirp-udp-internal.h"
#include "qapi/error.h"
#include "qemu/bswap.h"

static const uint8_t pmap_getport[] = {
    0x12, 0x34, 0x56, 0x78, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 1, 0x86, 0xa0, 0, 0, 0, 2, 0, 0, 0, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0x86, 0xa3, 0, 0, 0, 2, 0, 0, 0, 17, 0, 0, 0, 0,
};

static const uint8_t mount_mnt_root[] = {
    0xca, 0xfe, 0xba, 0xbe, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 1, 0x86, 0xa5, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, '/', 0, 0, 0,
};

static const uint8_t nfs_getattr[] = {
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 1, 0x86, 0xa3, 0, 0, 0, 2, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
};

static const uint8_t auth_sys_root[] = {
    0xde, 0xad, 0xbe, 0xef, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 1, 0x86, 0xa3, 0, 0, 0, 2, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 32,
    1, 2, 3, 4, 0, 0, 0, 4, 'n', 'e', 'x', 't',
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 0, 0, 0, 0, 0, 0, 10,
    0, 0, 0, 0, 0, 0, 0, 0,
};

static OncRpcCall decode_ok(const uint8_t *data, size_t len)
{
    OncRpcCall call;

    g_assert_cmpint(onc_rpc_decode_call(data, len, &call), ==,
                    ONC_RPC_DECODE_OK);
    return call;
}

static void test_golden_calls(void)
{
    OncRpcCall call;

    call = decode_ok(pmap_getport, sizeof(pmap_getport));
    g_assert_cmphex(call.xid, ==, 0x12345678);
    g_assert_cmpuint(call.program, ==, 100000);
    g_assert_cmpuint(call.version, ==, 2);
    g_assert_cmpuint(call.procedure, ==, 3);
    g_assert_cmpuint(call.auth_flavor, ==, ONC_RPC_AUTH_NULL);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&call.body), ==, 16);

    call = decode_ok(mount_mnt_root, sizeof(mount_mnt_root));
    g_assert_cmphex(call.xid, ==, 0xcafebabe);
    g_assert_cmpuint(call.program, ==, 100005);
    g_assert_cmpuint(call.version, ==, 1);
    g_assert_cmpuint(call.procedure, ==, 1);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&call.body), ==, 8);

    call = decode_ok(nfs_getattr, sizeof(nfs_getattr));
    g_assert_cmpuint(call.program, ==, 100003);
    g_assert_cmpuint(call.procedure, ==, 1);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&call.body), ==, 32);
}

static void test_auth_sys(void)
{
    OncRpcCall call = decode_ok(auth_sys_root, sizeof(auth_sys_root));

    g_assert_cmpuint(call.auth_flavor, ==, ONC_RPC_AUTH_SYS);
    g_assert_cmpstr(call.machine, ==, "next");
    g_assert_cmpuint(call.uid, ==, 0);
    g_assert_cmpuint(call.gid, ==, 0);
    g_assert_cmpuint(call.group_count, ==, 2);
    g_assert_cmpuint(call.groups[0], ==, 0);
    g_assert_cmpuint(call.groups[1], ==, 10);
    g_assert_true(onc_rpc_xdr_reader_empty(&call.body));
}

static void test_reader_rejections(void)
{
    static const uint8_t short_word[] = { 0, 0, 0 };
    static const uint8_t huge_count[] = { 0xff, 0xff, 0xff, 0xff };
    static const uint8_t nonzero_padding[] = {
        0, 0, 0, 1, 'x', 0xad, 0xbe, 0xef,
    };
    OncRpcXdrReader r;
    const uint8_t *start;
    uint32_t value;
    const uint8_t *opaque;
    size_t length;
    char string[8];

    onc_rpc_xdr_reader_init(&r, short_word, sizeof(short_word));
    start = r.cursor;
    g_assert_false(onc_rpc_xdr_u32(&r, &value));
    g_assert_true(r.cursor == start);

    onc_rpc_xdr_reader_init(&r, huge_count, sizeof(huge_count));
    start = r.cursor;
    g_assert_false(onc_rpc_xdr_counted_opaque(&r, &opaque, &length,
                                              UINT32_MAX));
    g_assert_true(r.cursor == start);

    onc_rpc_xdr_reader_init(&r, nonzero_padding, sizeof(nonzero_padding));
    g_assert_true(onc_rpc_xdr_string(&r, string, sizeof(string), 7));
    g_assert_cmpstr(string, ==, "x");
    g_assert_true(onc_rpc_xdr_reader_empty(&r));

    onc_rpc_xdr_reader_init(&r, short_word, sizeof(short_word));
    start = r.cursor;
    g_assert_false(onc_rpc_xdr_opaque(&r, NULL, SIZE_MAX));
    g_assert_true(r.cursor == start);

    {
        static const uint8_t bad_bool[] = { 0, 0, 0, 2 };

        onc_rpc_xdr_reader_init(&r, bad_bool, sizeof(bad_bool));
        start = r.cursor;
        g_assert_false(onc_rpc_xdr_bool(&r, NULL));
        g_assert_true(r.cursor == start);
    }
}

static void test_rpc_rejections(void)
{
    uint8_t vector[sizeof(auth_sys_root)];
    uint8_t oversized[ONC_RPC_MAX_DATAGRAM + 1] = { 0 };
    OncRpcCall call;

    g_assert_cmpint(onc_rpc_decode_call(pmap_getport, 39, &call), ==,
                    ONC_RPC_DECODE_GARBAGE_ARGS);
    g_assert_cmpint(onc_rpc_decode_call(oversized, sizeof(oversized), &call),
                    ==, ONC_RPC_DECODE_TOO_LARGE);

    memcpy(vector, auth_sys_root, sizeof(vector));
    vector[11] = 3;
    g_assert_cmpint(onc_rpc_decode_call(vector, sizeof(vector), &call), ==,
                    ONC_RPC_DECODE_RPC_MISMATCH);

    memcpy(vector, auth_sys_root, sizeof(vector));
    vector[55] = ONC_RPC_MAX_AUTH_GROUPS + 1;
    g_assert_cmpint(onc_rpc_decode_call(vector, sizeof(vector), &call), ==,
                    ONC_RPC_DECODE_AUTH_ERROR);
}

static void test_rpc_transport_bounds(void)
{
    size_t length = ONC_RPC_MAX_DATAGRAM + 4;
    g_autofree uint8_t *packet = g_malloc0(length);
    OncRpcXdrWriter writer;
    OncRpcCall call;

    onc_rpc_xdr_writer_init(&writer, packet, 40);
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 1));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_CALL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 100003));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 2));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 1));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 40);

    g_assert_cmpint(onc_rpc_decode_call(packet, length, &call), ==,
                    ONC_RPC_DECODE_TOO_LARGE);
    g_assert_cmpint(onc_rpc_decode_call_bounded(packet, length,
                                                ONC_RPC_MAX_TCP_RECORD,
                                                &call), ==,
                    ONC_RPC_DECODE_OK);
    g_assert_cmpuint(call.data_length, ==, length);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&call.body), ==,
                    length - 40);
}

static void test_scalars_and_arrays(void)
{
    static const uint8_t nonaligned[] = {
        0, 0, 0, 1, 1, 2, 3, 0,
    };
    uint8_t buffer[128];
    static const uint32_t values[] = { 1, 0xfeedface };
    OncRpcXdrWriter w;
    OncRpcXdrReader r;
    const uint8_t *bytes;
    size_t count;
    int32_t signed_value;
    bool boolean;
    char string[16];

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_xdr_put_bool(&w, true));
    g_assert_true(onc_rpc_xdr_put_i32(&w, -123));
    g_assert_true(onc_rpc_xdr_put_counted_array(&w, values,
                                                G_N_ELEMENTS(values),
                                                sizeof(values[0]), 4));
    g_assert_true(onc_rpc_xdr_put_string(&w, "bounded", 7));

    onc_rpc_xdr_reader_init(&r, buffer, onc_rpc_xdr_writer_size(&w));
    g_assert_true(onc_rpc_xdr_bool(&r, &boolean));
    g_assert_true(boolean);
    g_assert_true(onc_rpc_xdr_i32(&r, &signed_value));
    g_assert_cmpint(signed_value, ==, -123);
    g_assert_true(onc_rpc_xdr_counted_array(&r, &bytes, &count,
                                             sizeof(values[0]), 4));
    g_assert_cmpuint(count, ==, G_N_ELEMENTS(values));
    g_assert_cmpmem(bytes, count * sizeof(values[0]), values, sizeof(values));
    g_assert_true(onc_rpc_xdr_string(&r, string, sizeof(string), 7));
    g_assert_cmpstr(string, ==, "bounded");
    g_assert_true(onc_rpc_xdr_reader_empty(&r));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    {
        uint8_t *start = w.cursor;

        g_assert_false(onc_rpc_xdr_put_counted_array(&w, values, 1, 3, 4));
        g_assert_true(w.cursor == start);
    }

    onc_rpc_xdr_reader_init(&r, nonaligned, sizeof(nonaligned));
    {
        const uint8_t *start = r.cursor;

        g_assert_false(onc_rpc_xdr_counted_array(&r, &bytes, &count, 3, 4));
        g_assert_true(r.cursor == start);
    }

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_false(onc_rpc_xdr_put_string(&w, "too-long", 7));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, 0);
}

static void test_writer_bounds(void)
{
    uint8_t buffer[16] = { 0 };
    uint8_t values[4] = { 1, 2, 3, 4 };
    OncRpcXdrWriter writer;
    uint8_t *start;

    onc_rpc_xdr_writer_init(&writer, buffer, 3);
    start = writer.cursor;
    g_assert_false(onc_rpc_xdr_put_u32(&writer, 1));
    g_assert_true(writer.cursor == start);

    onc_rpc_xdr_writer_init(&writer, buffer, 7);
    start = writer.cursor;
    g_assert_false(onc_rpc_xdr_put_counted_opaque(&writer, values,
                                                  sizeof(values),
                                                  sizeof(values)));
    g_assert_true(writer.cursor == start);

    onc_rpc_xdr_writer_init(&writer, buffer, sizeof(buffer));
    start = writer.cursor;
    g_assert_false(onc_rpc_xdr_put_opaque(&writer, values, SIZE_MAX));
    g_assert_true(writer.cursor == start);

    onc_rpc_xdr_writer_init(&writer, buffer, sizeof(buffer));
    start = writer.cursor;
    g_assert_false(onc_rpc_xdr_put_counted_opaque(&writer, values, SIZE_MAX,
                                                  SIZE_MAX));
    g_assert_true(writer.cursor == start);

    onc_rpc_xdr_writer_init(&writer, buffer, sizeof(buffer));
    start = writer.cursor;
    g_assert_false(onc_rpc_xdr_put_counted_array(&writer, values, SIZE_MAX,
                                                 sizeof(values[0]), SIZE_MAX));
    g_assert_true(writer.cursor == start);
}

static void test_reply_vectors(void)
{
    static const uint8_t success[] = {
        0x12, 0x34, 0x56, 0x78, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    static const uint8_t prog_mismatch[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 0, 0, 0, 2,
    };
    static const uint8_t prog_unavail[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    };
    static const uint8_t proc_unavail[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
    };
    static const uint8_t garbage_args[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4,
    };
    static const uint8_t system_err[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5,
    };
    static const uint8_t rpc_mismatch[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2,
    };
    static const uint8_t auth_error[] = {
        0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
        0, 0, 0, 1, 0, 0, 0, 1,
    };
    uint8_t buffer[32];
    OncRpcXdrWriter w;
    uint8_t *start;

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_success(&w, 0x12345678));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(success));
    g_assert_cmpmem(buffer, sizeof(success), success, sizeof(success));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_prog_mismatch(&w, 1, 1, 2));
    g_assert_cmpmem(buffer, sizeof(prog_mismatch), prog_mismatch,
                    sizeof(prog_mismatch));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_prog_unavail(&w, 1));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(prog_unavail));
    g_assert_cmpmem(buffer, sizeof(prog_unavail), prog_unavail,
                    sizeof(prog_unavail));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_proc_unavail(&w, 1));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(proc_unavail));
    g_assert_cmpmem(buffer, sizeof(proc_unavail), proc_unavail,
                    sizeof(proc_unavail));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_garbage_args(&w, 1));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(garbage_args));
    g_assert_cmpmem(buffer, sizeof(garbage_args), garbage_args,
                    sizeof(garbage_args));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_system_err(&w, 1));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(system_err));
    g_assert_cmpmem(buffer, sizeof(system_err), system_err,
                    sizeof(system_err));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_rpc_mismatch(&w, 1, 2, 2));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(rpc_mismatch));
    g_assert_cmpmem(buffer, sizeof(rpc_mismatch), rpc_mismatch,
                    sizeof(rpc_mismatch));

    onc_rpc_xdr_writer_init(&w, buffer, sizeof(buffer));
    g_assert_true(onc_rpc_reply_auth_error(&w, 1, ONC_RPC_AUTH_BADCRED));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&w), ==, sizeof(auth_error));
    g_assert_cmpmem(buffer, sizeof(auth_error), auth_error,
                    sizeof(auth_error));

    onc_rpc_xdr_writer_init(&w, buffer, 23);
    start = w.cursor;
    g_assert_false(onc_rpc_reply_success(&w, 0x12345678));
    g_assert_true(w.cursor == start);

    onc_rpc_xdr_writer_init(&w, buffer, 0);
    start = w.cursor;
    g_assert_false(onc_rpc_reply_prog_unavail(&w, 1));
    g_assert_true(w.cursor == start);
    g_assert_false(onc_rpc_reply_prog_mismatch(&w, 1, 1, 2));
    g_assert_true(w.cursor == start);
    g_assert_false(onc_rpc_reply_proc_unavail(&w, 1));
    g_assert_true(w.cursor == start);
    g_assert_false(onc_rpc_reply_garbage_args(&w, 1));
    g_assert_true(w.cursor == start);
    g_assert_false(onc_rpc_reply_system_err(&w, 1));
    g_assert_true(w.cursor == start);
    g_assert_false(onc_rpc_reply_rpc_mismatch(&w, 1, 2, 2));
    g_assert_true(w.cursor == start);
    g_assert_false(onc_rpc_reply_auth_error(&w, 1, ONC_RPC_AUTH_BADCRED));
    g_assert_true(w.cursor == start);
}

typedef struct RecordCapture {
    GPtrArray *records;
} RecordCapture;

static bool capture_record(const uint8_t *data, size_t len, void *opaque)
{
    RecordCapture *capture = opaque;

    g_ptr_array_add(capture->records, g_bytes_new(data, len));
    return true;
}

static void capture_init(RecordCapture *capture)
{
    capture->records = g_ptr_array_new_with_free_func(
        (GDestroyNotify)g_bytes_unref);
}

static void capture_clear(RecordCapture *capture)
{
    g_ptr_array_unref(capture->records);
}

static void test_record_fragmented_header(void)
{
    static const uint8_t packet[] = { 0x80, 0, 0, 3, 'a', 'b', 'c' };
    OncRpcTcpRecordDecoder decoder;
    RecordCapture capture;

    capture_init(&capture);
    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_true(onc_rpc_tcp_record_decoder_feed(&decoder, packet, 2,
                                                  capture_record, &capture));
    g_assert_true(onc_rpc_tcp_record_decoder_feed(&decoder, packet + 2,
                                                  sizeof(packet) - 2,
                                                  capture_record, &capture));
    g_assert_cmpuint(capture.records->len, ==, 1);
    g_assert_true(onc_rpc_tcp_record_decoder_finish(&decoder));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
    capture_clear(&capture);
}

static void test_record_multiple_fragments(void)
{
    static const uint8_t packet[] = {
        0, 0, 0, 3, 'a', 'b', 'c', 0x80, 0, 0, 2, 'd', 'e',
    };
    OncRpcTcpRecordDecoder decoder;
    RecordCapture capture;
    GBytes *record;
    gsize length;
    const uint8_t *data;

    capture_init(&capture);
    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_true(onc_rpc_tcp_record_decoder_feed(&decoder, packet,
                                                  sizeof(packet),
                                                  capture_record, &capture));
    g_assert_cmpuint(capture.records->len, ==, 1);
    record = g_ptr_array_index(capture.records, 0);
    data = g_bytes_get_data(record, &length);
    g_assert_cmpuint(length, ==, 5);
    g_assert_cmpmem(data, length, "abcde", 5);
    g_assert_true(onc_rpc_tcp_record_decoder_finish(&decoder));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
    capture_clear(&capture);
}

static void test_record_oversized_fragment(void)
{
    uint8_t marker[4];
    OncRpcTcpRecordDecoder decoder;
    RecordCapture capture;

    stl_be_p(marker, ONC_RPC_MAX_TCP_RECORD + 1);
    capture_init(&capture);
    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_false(onc_rpc_tcp_record_decoder_feed(&decoder, marker,
                                                   sizeof(marker),
                                                   capture_record, &capture));
    g_assert_cmpuint(capture.records->len, ==, 0);
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
    capture_clear(&capture);
}

static void test_record_two_in_one_receive(void)
{
    static const uint8_t packet[] = {
        0x80, 0, 0, 1, 'x', 0x80, 0, 0, 2, 'y', 'z',
    };
    OncRpcTcpRecordDecoder decoder;
    RecordCapture capture;

    capture_init(&capture);
    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_true(onc_rpc_tcp_record_decoder_feed(&decoder, packet,
                                                  sizeof(packet),
                                                  capture_record, &capture));
    g_assert_cmpuint(capture.records->len, ==, 2);
    g_assert_true(onc_rpc_tcp_record_decoder_finish(&decoder));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
    capture_clear(&capture);
}

static void test_record_truncated_close(void)
{
    static const uint8_t partial_header[] = { 0x80, 0 };
    static const uint8_t partial_body[] = { 0x80, 0, 0, 3, 'x' };
    OncRpcTcpRecordDecoder decoder;
    RecordCapture capture;

    capture_init(&capture);
    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_true(onc_rpc_tcp_record_decoder_feed(&decoder, partial_header,
                                                  sizeof(partial_header),
                                                  capture_record, &capture));
    g_assert_false(onc_rpc_tcp_record_decoder_finish(&decoder));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);

    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_true(onc_rpc_tcp_record_decoder_feed(&decoder, partial_body,
                                                  sizeof(partial_body),
                                                  capture_record, &capture));
    g_assert_false(onc_rpc_tcp_record_decoder_finish(&decoder));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
    capture_clear(&capture);
}

typedef struct ReentrantRecordCapture {
    GPtrArray *records;
    OncRpcTcpRecordDecoder *decoder;
    bool recursive_result;
} ReentrantRecordCapture;

static bool capture_record_plain(const uint8_t *data, size_t len, void *opaque)
{
    ReentrantRecordCapture *capture = opaque;

    g_ptr_array_add(capture->records, g_bytes_new(data, len));
    return true;
}

static bool capture_record_reentrant(const uint8_t *data, size_t len,
                                     void *opaque)
{
    static const uint8_t nested[] = { 0x80, 0, 0, 1, 'n' };
    ReentrantRecordCapture *capture = opaque;

    capture->recursive_result = onc_rpc_tcp_record_decoder_feed(
        capture->decoder, nested, sizeof(nested), capture_record_plain,
        capture);
    g_ptr_array_add(capture->records, g_bytes_new(data, len));
    return true;
}

static void test_record_callback_reentry(void)
{
    static const uint8_t packet[] = { 0x80, 0, 0, 1, 'x' };
    ReentrantRecordCapture capture;
    OncRpcTcpRecordDecoder decoder;

    capture.records = g_ptr_array_new_with_free_func(
        (GDestroyNotify)g_bytes_unref);
    capture.decoder = &decoder;
    capture.recursive_result = true;
    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_true(onc_rpc_tcp_record_decoder_feed(
        &decoder, packet, sizeof(packet), capture_record_reentrant, &capture));
    g_assert_false(capture.recursive_result);
    g_assert_cmpuint(capture.records->len, ==, 1);
    g_assert_true(onc_rpc_tcp_record_decoder_finish(&decoder));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
    g_ptr_array_unref(capture.records);
}

static bool reject_record(const uint8_t *data, size_t len, void *opaque)
{
    (void)data;
    (void)len;
    (void)opaque;
    return false;
}

static void test_record_callback_failure(void)
{
    static const uint8_t packet[] = { 0x80, 0, 0, 1, 'x' };
    OncRpcTcpRecordDecoder decoder;

    onc_rpc_tcp_record_decoder_init(&decoder, ONC_RPC_MAX_TCP_RECORD);
    g_assert_false(onc_rpc_tcp_record_decoder_feed(
        &decoder, packet, sizeof(packet), reject_record, NULL));
    g_assert_false(onc_rpc_tcp_record_decoder_finish(&decoder));
    g_assert_false(onc_rpc_tcp_record_decoder_feed(
        &decoder, packet, sizeof(packet), reject_record, NULL));
    onc_rpc_tcp_record_decoder_cleanup(&decoder);
}

typedef struct RpcFakeUdp RpcFakeUdp;
typedef struct RpcFakeUdpEndpoint {
    RpcFakeUdp *fake;
    QemuSlirpUdpBackendCallbacks callbacks;
    void *callbacks_opaque;
    uint16_t port;
    QemuSlirpUdpListenFlags flags;
    bool active;
} RpcFakeUdpEndpoint;

struct RpcFakeUdp {
    RpcFakeUdpEndpoint endpoints[8];
    unsigned nendpoints;
    unsigned listens;
    unsigned removes;
    GByteArray *reply;
    struct sockaddr_in peer;
};

typedef struct RpcFakeTcp RpcFakeTcp;
typedef struct RpcFakeTcpEndpoint {
    RpcFakeTcp *fake;
    QemuSlirpStreamBackendCallbacks callbacks;
    void *callbacks_opaque;
    uint16_t port;
    bool active;
} RpcFakeTcpEndpoint;

struct RpcFakeTcp {
    RpcFakeTcpEndpoint endpoints[8];
    unsigned nendpoints;
    unsigned listens;
    unsigned removes;
    GByteArray *reply;
    void *connection;
    RpcFakeTcpEndpoint *connection_endpoint;
    struct sockaddr_in peer;
    bool close_on_remove;
    bool close_on_send;
    bool fail_listen;
    uint16_t fail_port;
    bool send_eagain_once;
    bool send_eio_once;
    bool enforce_send_space;
    size_t send_space;
    GByteArray *wire;
    unsigned closes;
    unsigned sends;
};

static int rpc_fake_udp_listen(
    void *opaque, struct in_addr address, uint16_t port,
    QemuSlirpUdpListenFlags flags,
    const QemuSlirpUdpBackendCallbacks *callbacks, void *callbacks_opaque,
    void **backend_listener)
{
    RpcFakeUdp *fake = opaque;
    RpcFakeUdpEndpoint *endpoint = NULL;
    unsigned i;

    (void)address;
    for (i = 0; i < G_N_ELEMENTS(fake->endpoints); i++) {
        if (!fake->endpoints[i].active) {
            endpoint = &fake->endpoints[i];
            break;
        }
    }
    g_assert_nonnull(endpoint);
    endpoint->fake = fake;
    endpoint->port = port;
    endpoint->flags = flags;
    endpoint->callbacks = *callbacks;
    endpoint->callbacks_opaque = callbacks_opaque;
    endpoint->active = true;
    fake->nendpoints = MAX(fake->nendpoints, i + 1);
    fake->listens++;
    *backend_listener = endpoint;
    return 0;
}

static void rpc_fake_udp_remove(void *opaque, void *backend_listener)
{
    RpcFakeUdp *fake = opaque;
    RpcFakeUdpEndpoint *endpoint = backend_listener;

    g_assert_true(endpoint->fake == fake);
    g_assert_true(endpoint->active);
    fake->removes++;
    endpoint->active = false;
    endpoint->callbacks_opaque = NULL;
}

static int rpc_fake_udp_send(void *opaque, void *backend_listener,
                             const struct sockaddr_in *peer,
                             const uint8_t *data, size_t len)
{
    RpcFakeUdpEndpoint *endpoint = backend_listener;
    RpcFakeUdp *fake = endpoint->fake;

    g_assert_true(opaque == fake);
    g_assert_true(endpoint->active);
    g_byte_array_set_size(fake->reply, 0);
    g_byte_array_append(fake->reply, data, len);
    fake->peer = *peer;
    return 0;
}

static const QemuSlirpUdpBackendOps rpc_fake_udp_ops = {
    .listen = rpc_fake_udp_listen,
    .listener_remove = rpc_fake_udp_remove,
    .send = rpc_fake_udp_send,
};

static int rpc_fake_tcp_listen(
    void *opaque, struct in_addr address, uint16_t port,
    const QemuSlirpStreamBackendCallbacks *callbacks, void *callbacks_opaque,
    void **backend_listener)
{
    RpcFakeTcp *fake = opaque;
    RpcFakeTcpEndpoint *endpoint = NULL;
    unsigned i;

    (void)address;
    if (fake->fail_listen && fake->fail_port == port) {
        return -1;
    }
    for (i = 0; i < G_N_ELEMENTS(fake->endpoints); i++) {
        if (!fake->endpoints[i].active) {
            endpoint = &fake->endpoints[i];
            break;
        }
    }
    g_assert_nonnull(endpoint);
    endpoint->fake = fake;
    endpoint->port = port;
    endpoint->callbacks = *callbacks;
    endpoint->callbacks_opaque = callbacks_opaque;
    endpoint->active = true;
    fake->nendpoints = MAX(fake->nendpoints, i + 1);
    fake->listens++;
    *backend_listener = endpoint;
    return 0;
}

static void rpc_fake_tcp_remove(void *opaque, void *backend_listener)
{
    RpcFakeTcp *fake = opaque;
    RpcFakeTcpEndpoint *endpoint = backend_listener;

    g_assert_true(endpoint->fake == fake);
    g_assert_true(endpoint->active);
    fake->removes++;
    if (fake->close_on_remove && fake->connection_endpoint == endpoint &&
        fake->connection) {
        endpoint->callbacks.closed(fake->connection,
                                   endpoint->callbacks_opaque);
    }
    endpoint->active = false;
    endpoint->callbacks_opaque = NULL;
}

static size_t rpc_fake_tcp_can_send(void *opaque, void *backend_connection)
{
    RpcFakeTcp *fake = opaque;

    (void)backend_connection;
    return fake->enforce_send_space ? fake->send_space :
                                      ONC_RPC_MAX_TCP_RECORD + 4;
}

static int rpc_fake_tcp_send(void *opaque, void *backend_connection,
                             const uint8_t *data, size_t len)
{
    RpcFakeTcp *fake = opaque;

    g_assert_nonnull(fake->connection_endpoint);
    g_assert_true(backend_connection == fake->connection);
    fake->sends++;
    if (fake->send_eagain_once) {
        fake->send_eagain_once = false;
        return -EAGAIN;
    }
    if (fake->send_eio_once) {
        fake->send_eio_once = false;
        return -EIO;
    }
    if (fake->close_on_send) {
        fake->close_on_send = false;
        fake->closes++;
        fake->connection_endpoint->callbacks.closed(
            fake->connection, fake->connection_endpoint->callbacks_opaque);
        return 0;
    }
    if (fake->enforce_send_space && len > fake->send_space) {
        return -EAGAIN;
    }
    if (fake->enforce_send_space) {
        fake->send_space -= len;
        g_byte_array_append(fake->wire, data, len);
    }
    g_byte_array_set_size(fake->reply, 0);
    g_byte_array_append(fake->reply, data, len);
    return 0;
}

static void rpc_fake_tcp_close(void *opaque, void *backend_connection)
{
    RpcFakeTcp *fake = opaque;

    g_assert_nonnull(fake->connection_endpoint);
    g_assert_true(backend_connection == fake->connection);
    fake->closes++;
    fake->connection_endpoint->callbacks.closed(
        backend_connection, fake->connection_endpoint->callbacks_opaque);
}

static const QemuSlirpStreamBackendOps rpc_fake_tcp_ops = {
    .listen = rpc_fake_tcp_listen,
    .listener_remove = rpc_fake_tcp_remove,
    .can_send = rpc_fake_tcp_can_send,
    .send = rpc_fake_tcp_send,
    .connection_close = rpc_fake_tcp_close,
};

typedef struct RpcHarness {
    RpcFakeUdp udp;
    RpcFakeTcp tcp;
    QemuSlirpUdpRegistry *udp_registry;
    QemuSlirpStreamRegistry *stream_registry;
    QemuSlirpRpcRegistry *rpc;
    struct in_addr vhost;
} RpcHarness;

static void rpc_harness_init(RpcHarness *harness)
{
    memset(harness, 0, sizeof(*harness));
    harness->vhost.s_addr = htonl(0x0a000202);
    harness->udp.reply = g_byte_array_new();
    harness->tcp.reply = g_byte_array_new();
    harness->tcp.wire = g_byte_array_new();
    harness->udp_registry = qemu_slirp_udp_registry_new(
        true, harness->vhost, &rpc_fake_udp_ops, &harness->udp);
    harness->stream_registry = qemu_slirp_stream_registry_new(
        true, harness->vhost, &rpc_fake_tcp_ops, &harness->tcp);
    harness->rpc = qemu_slirp_rpc_registry_new(harness->udp_registry,
                                               harness->stream_registry);
    g_assert_nonnull(harness->rpc);
}

static void rpc_harness_cleanup(RpcHarness *harness)
{
    qemu_slirp_rpc_registry_free(harness->rpc);
    qemu_slirp_udp_registry_free(harness->udp_registry);
    qemu_slirp_stream_registry_free(harness->stream_registry);
    g_byte_array_unref(harness->udp.reply);
    g_byte_array_unref(harness->tcp.reply);
    g_byte_array_unref(harness->tcp.wire);
}

static size_t rpc_build_call(uint8_t *buffer, size_t capacity, uint32_t xid,
                             uint32_t program, uint32_t version,
                             uint32_t procedure, const uint8_t *body,
                             size_t body_length)
{
    OncRpcXdrWriter writer;

    g_assert_cmpuint(capacity, >=, 40 + body_length);
    onc_rpc_xdr_writer_init(&writer, buffer, capacity);
    g_assert_true(onc_rpc_xdr_put_u32(&writer, xid));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_CALL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, version));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, procedure));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_opaque(&writer, body, body_length));
    return onc_rpc_xdr_writer_size(&writer);
}

static size_t rpc_build_call_auth_sys(uint8_t *buffer, size_t capacity,
                                      uint32_t xid, uint32_t program,
                                      uint32_t version, uint32_t procedure,
                                      const uint8_t *body, size_t body_length)
{
    uint8_t credential[128];
    OncRpcXdrWriter credential_writer;
    OncRpcXdrWriter writer;

    onc_rpc_xdr_writer_init(&credential_writer, credential,
                            sizeof(credential));
    g_assert_true(onc_rpc_xdr_put_u32(&credential_writer, 0x01020304));
    g_assert_true(onc_rpc_xdr_put_string(&credential_writer, "outer", 255));
    g_assert_true(onc_rpc_xdr_put_u32(&credential_writer, 1000));
    g_assert_true(onc_rpc_xdr_put_u32(&credential_writer, 1001));
    g_assert_true(onc_rpc_xdr_put_u32(&credential_writer, 1));
    g_assert_true(onc_rpc_xdr_put_u32(&credential_writer, 1002));
    onc_rpc_xdr_writer_init(&writer, buffer, capacity);
    g_assert_true(onc_rpc_xdr_put_u32(&writer, xid));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_CALL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_VERSION));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, version));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, procedure));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_SYS));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(
        &writer, credential, onc_rpc_xdr_writer_size(&credential_writer),
        ONC_RPC_MAX_AUTH_BYTES));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, ONC_RPC_AUTH_NULL));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_opaque(&writer, body, body_length));
    return onc_rpc_xdr_writer_size(&writer);
}

static size_t rpc_build_record(uint8_t *buffer, size_t capacity,
                               const uint8_t *payload, size_t length)
{
    g_assert_cmpuint(capacity, >=, length + 4);
    stl_be_p(buffer, UINT32_C(0x80000000) | length);
    memcpy(buffer + 4, payload, length);
    return length + 4;
}

static uint32_t rpc_reply_status(const uint8_t *data, size_t length,
                                 OncRpcXdrReader *body)
{
    OncRpcXdrReader reader;
    uint32_t xid, type, status, flavor, auth_length;

    onc_rpc_xdr_reader_init(&reader, data, length);
    g_assert_true(onc_rpc_xdr_u32(&reader, &xid));
    g_assert_true(onc_rpc_xdr_u32(&reader, &type));
    g_assert_true(onc_rpc_xdr_u32(&reader, &status));
    g_assert_cmpuint(type, ==, ONC_RPC_REPLY);
    g_assert_cmpuint(status, ==, ONC_RPC_MSG_ACCEPTED);
    g_assert_true(onc_rpc_xdr_u32(&reader, &flavor));
    g_assert_true(onc_rpc_xdr_u32(&reader, &auth_length));
    g_assert_cmpuint(flavor, ==, ONC_RPC_AUTH_NULL);
    g_assert_cmpuint(auth_length, ==, 0);
    g_assert_true(onc_rpc_xdr_u32(&reader, &status));
    *body = reader;
    return status;
}

typedef struct RpcProgramState {
    unsigned calls;
    uint32_t last_procedure;
    uint32_t last_auth_flavor;
    uint32_t last_uid;
    uint32_t last_gid;
    size_t last_group_count;
    char last_machine[ONC_RPC_MAX_AUTH_MACHINE + 1];
    uint32_t result;
    bool async;
    bool teardown;
    bool error_reply;
    bool malformed_reply;
    QemuSlirpRpcRegistry *registry;
    OncRpcRequest *pending;
} RpcProgramState;

typedef struct RpcQueueProgramState {
    unsigned calls;
    size_t reply_length;
    bool tag_xid;
    bool async;
    OncRpcRequest *pending[2];
    unsigned pending_count;
} RpcQueueProgramState;

static OncRpcDispatchResult rpc_program_dispatch(
    OncRpcRequest *request, const OncRpcCall *call, void *opaque)
{
    RpcProgramState *state = opaque;
    uint8_t reply[128];
    OncRpcXdrWriter writer;

    state->calls++;
    state->last_procedure = call->procedure;
    state->last_auth_flavor = call->auth_flavor;
    state->last_uid = call->uid;
    state->last_gid = call->gid;
    state->last_group_count = call->group_count;
    g_strlcpy(state->last_machine, call->machine,
              sizeof(state->last_machine));
    if (state->teardown) {
        if (state->registry) {
            qemu_slirp_rpc_registry_invalidate(state->registry);
        }
        return ONC_RPC_DISPATCH_DROP;
    }
    if (state->async) {
        state->pending = onc_rpc_request_ref(request);
        return ONC_RPC_DISPATCH_ASYNC;
    }
    onc_rpc_xdr_writer_init(&writer, reply, sizeof(reply));
    if (state->malformed_reply) {
        static const uint8_t malformed[] = { 0, 0, 0, 1 };

        g_assert_false(onc_rpc_request_reply(request, malformed,
                                             sizeof(malformed)));
        return ONC_RPC_DISPATCH_REPLIED;
    }
    if (state->error_reply) {
        g_assert_true(onc_rpc_reply_proc_unavail(&writer, call->xid));
    } else {
        g_assert_true(onc_rpc_reply_success(&writer, call->xid));
    }
    if (state->error_reply) {
        g_assert_false(onc_rpc_request_reply(request, reply,
                                             onc_rpc_xdr_writer_size(&writer)));
        return ONC_RPC_DISPATCH_REPLIED;
    }
    g_assert_true(onc_rpc_xdr_put_u32(&writer, state->result));
    g_assert_true(onc_rpc_request_reply(request, reply,
                                        onc_rpc_xdr_writer_size(&writer)));
    return ONC_RPC_DISPATCH_REPLIED;
}

static OncRpcDispatchResult rpc_queue_program_dispatch(
    OncRpcRequest *request, const OncRpcCall *call, void *opaque)
{
    RpcQueueProgramState *state = opaque;
    uint8_t *reply;
    size_t i;

    state->calls++;
    if (state->async) {
        g_assert_cmpuint(state->pending_count, <, G_N_ELEMENTS(state->pending));
        state->pending[state->pending_count++] = onc_rpc_request_ref(request);
        return ONC_RPC_DISPATCH_ASYNC;
    }
    reply = g_malloc(state->reply_length);
    for (i = 0; i < state->reply_length; i++) {
        reply[i] = (uint8_t)(i * 37 + 11);
    }
    if (state->tag_xid && state->reply_length >= sizeof(call->xid)) {
        stl_be_p(reply, call->xid);
    }
    g_assert_true(onc_rpc_request_reply(request, reply, state->reply_length));
    g_free(reply);
    return ONC_RPC_DISPATCH_REPLIED;
}

static OncRpcProgram rpc_program(uint32_t program, uint32_t low,
                                 uint32_t high, uint16_t port,
                                 unsigned transports,
                                 RpcProgramState *state)
{
    return (OncRpcProgram) {
        .program = program,
        .version_low = low,
        .version_high = high,
        .port = port,
        .transports = transports,
        .dispatch = rpc_program_dispatch,
        .opaque = state,
    };
}

static RpcFakeUdpEndpoint *rpc_fake_udp_endpoint(RpcFakeUdp *fake,
                                                  uint16_t port)
{
    unsigned i;

    for (i = 0; i < fake->nendpoints; i++) {
        if (fake->endpoints[i].active && fake->endpoints[i].port == port) {
            return &fake->endpoints[i];
        }
    }
    return NULL;
}

static RpcFakeTcpEndpoint *rpc_fake_tcp_endpoint(RpcFakeTcp *fake,
                                                  uint16_t port)
{
    unsigned i;

    for (i = 0; i < fake->nendpoints; i++) {
        if (fake->endpoints[i].active && fake->endpoints[i].port == port) {
            return &fake->endpoints[i];
        }
    }
    return NULL;
}

static void rpc_send_udp(RpcHarness *harness, const uint8_t *data, size_t len)
{
    RpcFakeUdpEndpoint *endpoint = rpc_fake_udp_endpoint(&harness->udp, 111);
    struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };

    g_assert_nonnull(endpoint);
    endpoint->callbacks.datagram(&peer, data, len,
                                 endpoint->callbacks_opaque);
}

static void rpc_send_udp_port(RpcHarness *harness, uint16_t port,
                              const uint8_t *data, size_t len)
{
    RpcFakeUdpEndpoint *endpoint = rpc_fake_udp_endpoint(&harness->udp, port);
    struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49152),
    };

    g_assert_nonnull(endpoint);
    endpoint->callbacks.datagram(&peer, data, len,
                                 endpoint->callbacks_opaque);
}

static void rpc_connect_tcp_port(RpcHarness *harness, uint16_t port)
{
    RpcFakeTcpEndpoint *endpoint = rpc_fake_tcp_endpoint(&harness->tcp, port);

    harness->tcp.connection = GINT_TO_POINTER(0x1234);
    harness->tcp.peer = (struct sockaddr_in) {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(0x0a00020f),
        .sin_port = htons(49153),
    };
    g_assert_nonnull(endpoint);
    harness->tcp.connection_endpoint = endpoint;
    endpoint->callbacks.connected(harness->tcp.connection,
                                  &harness->tcp.peer,
                                  endpoint->callbacks_opaque);
}

static void rpc_connect_tcp(RpcHarness *harness)
{
    rpc_connect_tcp_port(harness, 111);
}

static void rpc_send_tcp(RpcHarness *harness, const uint8_t *data, size_t len)
{
    g_assert_nonnull(harness->tcp.connection_endpoint);
    harness->tcp.connection_endpoint->callbacks.receive(
        harness->tcp.connection, data, len,
        harness->tcp.connection_endpoint->callbacks_opaque);
}

static void rpc_close_tcp(RpcHarness *harness)
{
    g_assert_nonnull(harness->tcp.connection_endpoint);
    rpc_fake_tcp_close(&harness->tcp, harness->tcp.connection);
}

static void rpc_ready_tcp(RpcHarness *harness)
{
    g_assert_nonnull(harness->tcp.connection_endpoint);
    harness->tcp.connection_endpoint->callbacks.can_send(
        harness->tcp.connection,
        harness->tcp.connection_endpoint->callbacks_opaque);
}

static void rpc_ready_tcp_with_space(RpcHarness *harness, size_t space)
{
    harness->tcp.send_space = space;
    rpc_ready_tcp(harness);
}

static void test_rpc_registry_registration(void)
{
    RpcHarness harness;
    RpcProgramState one = { .result = 11 };
    RpcProgramState two = { .result = 22 };
    RpcProgramState overlap = { .result = 33 };
    OncRpcProgram p1 = rpc_program(200001, 1, 2, 4001,
                                   ONC_RPC_TRANSPORT_UDP, &one);
    OncRpcProgram p2 = rpc_program(200001, 3, 4, 4001,
                                   ONC_RPC_TRANSPORT_UDP, &two);
    OncRpcProgram p_overlap = rpc_program(200001, 2, 3, 4001,
                                          ONC_RPC_TRANSPORT_UDP, &overlap);
    QemuSlirpRpcRegistration *r1 = NULL;
    QemuSlirpRpcRegistration *r2 = NULL;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &p1, &r1, &err), ==, 0);
    g_assert_nonnull(r1);
    g_assert_cmpuint(harness.udp.listens, ==, 2);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &p1, &r2, &err), ==, -1);
    g_assert_null(r2);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &p_overlap, &r2, &err), ==, -1);
    g_assert_null(r2);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &p2, &r2, &err), ==, 0);
    g_assert_nonnull(r2);
    qemu_slirp_rpc_registry_unregister(r1);
    g_assert_cmpuint(harness.udp.removes, ==, 0);
    qemu_slirp_rpc_registry_unregister(r2);
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_portmapper(void)
{
    RpcHarness harness;
    RpcProgramState udp_state = { .result = 17 };
    RpcProgramState tcp_state = { .result = 23 };
    OncRpcProgram udp = rpc_program(200010, 1, 3, 4010,
                                    ONC_RPC_TRANSPORT_UDP, &udp_state);
    OncRpcProgram tcp = rpc_program(200011, 3, 3, 4011,
                                    ONC_RPC_TRANSPORT_TCP, &tcp_state);
    QemuSlirpRpcRegistration *udp_reg = NULL;
    QemuSlirpRpcRegistration *tcp_reg = NULL;
    uint8_t body[64], call[128], record[132];
    size_t body_len, call_len, record_len;
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    uint32_t value, port;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &udp, &udp_reg, &err), ==, 0);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &tcp, &tcp_reg, &err), ==, 0);
    g_assert_cmpuint(harness.udp.listens, ==, 2);
    g_assert_cmpuint(harness.tcp.listens, ==, 2);
    g_assert_cmpuint(rpc_fake_udp_endpoint(&harness.udp, 111)->flags, ==,
                     QEMU_SLIRP_UDP_LISTEN_BROADCAST);

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, udp.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 2));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, IPPROTO_UDP));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = rpc_build_call(call, sizeof(call), 1, 100000, 2, 3, body,
                              body_len);
    rpc_send_udp(&harness, call, call_len);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, udp.port);

    /* The TCP portmapper endpoint answers with the same GETPORT semantics. */
    rpc_connect_tcp(&harness);
    body_len = rpc_build_call(body, sizeof(body), 2, 100000, 2, 3,
                              (const uint8_t[]) {
                                  0, 3, 0x0d, 0x4b,
                                  0, 0, 0, 3,
                                  0, 0, 0, 6,
                                  0, 0, 0, 0,
                              }, 16);
    record_len = rpc_build_record(record, sizeof(record), body, body_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(harness.tcp.reply->len, >, 4);
    g_assert_cmpuint(ldl_be_p(harness.tcp.reply->data) & 0x7fffffff, ==,
                     harness.tcp.reply->len - 4);
    value = rpc_reply_status(harness.tcp.reply->data + 4,
                             harness.tcp.reply->len - 4, &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, tcp.port);

    /* DUMP preserves registration order and includes every transport entry. */
    g_byte_array_set_size(harness.udp.reply, 0);
    call_len = rpc_build_call(call, sizeof(call), 3, 100000, 2, 4, NULL, 0);
    rpc_send_udp(&harness, call, call_len);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    for (value = udp.version_low; value <= udp.version_high; value++) {
        bool more;

        g_assert_true(onc_rpc_xdr_bool(&reply, &more));
        g_assert_true(more);
        g_assert_true(onc_rpc_xdr_u32(&reply, &port));
        g_assert_cmpuint(port, ==, udp.program);
        g_assert_true(onc_rpc_xdr_u32(&reply, &port));
        g_assert_cmpuint(port, ==, value);
        g_assert_true(onc_rpc_xdr_u32(&reply, &port));
        g_assert_cmpuint(port, ==, IPPROTO_UDP);
        g_assert_true(onc_rpc_xdr_u32(&reply, &port));
        g_assert_cmpuint(port, ==, udp.port);
    }
    {
        bool more;

        g_assert_true(onc_rpc_xdr_bool(&reply, &more));
        g_assert_true(more);
    }
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, tcp.program);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, tcp.version_low);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, IPPROTO_TCP);
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, tcp.port);
    {
        bool more;

        g_assert_true(onc_rpc_xdr_bool(&reply, &more));
        g_assert_false(more);
    }

    qemu_slirp_rpc_registry_unregister(udp_reg);
    qemu_slirp_rpc_registry_unregister(tcp_reg);
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    g_assert_cmpuint(harness.tcp.removes, ==, 2);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_portmapper_udp_first_owns_both(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 31 };
    OncRpcProgram program = rpc_program(200020, 1, 1, 4020,
                                         ONC_RPC_TRANSPORT_UDP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    g_assert_nonnull(registration);
    g_assert_cmpuint(harness.udp.listens, ==, 2);
    g_assert_cmpuint(harness.tcp.listens, ==, 1);
    g_assert_nonnull(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_nonnull(rpc_fake_tcp_endpoint(&harness.tcp, 111));
    g_assert_cmpuint(rpc_fake_udp_endpoint(&harness.udp, 111)->flags, ==,
                     QEMU_SLIRP_UDP_LISTEN_BROADCAST);

    qemu_slirp_rpc_registry_unregister(registration);
    g_assert_null(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_null(rpc_fake_tcp_endpoint(&harness.tcp, 111));
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    g_assert_cmpuint(harness.tcp.removes, ==, 1);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_portmapper_tcp_first_udp_discovery(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 37 };
    OncRpcProgram program = rpc_program(200021, 2, 2, 4021,
                                         ONC_RPC_TRANSPORT_TCP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t body[32], call[96];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    uint32_t value, port;
    size_t call_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    g_assert_nonnull(registration);
    g_assert_cmpuint(harness.udp.listens, ==, 1);
    g_assert_cmpuint(harness.tcp.listens, ==, 2);
    g_assert_nonnull(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_nonnull(rpc_fake_tcp_endpoint(&harness.tcp, 111));

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.version_low));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, IPPROTO_TCP));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    call_len = rpc_build_call(call, sizeof(call), 11, 100000, 2, 3, body,
                              onc_rpc_xdr_writer_size(&writer));
    rpc_send_udp(&harness, call, call_len);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_u32(&reply, &port));
    g_assert_cmpuint(port, ==, program.port);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_portmapper_retained_until_last(void)
{
    RpcHarness harness;
    RpcProgramState udp_state = { .result = 41 };
    RpcProgramState tcp_state = { .result = 43 };
    OncRpcProgram udp = rpc_program(200022, 1, 1, 4022,
                                    ONC_RPC_TRANSPORT_UDP, &udp_state);
    OncRpcProgram tcp = rpc_program(200023, 1, 1, 4023,
                                    ONC_RPC_TRANSPORT_TCP, &tcp_state);
    QemuSlirpRpcRegistration *udp_registration = NULL;
    QemuSlirpRpcRegistration *tcp_registration = NULL;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &udp, &udp_registration, &err), ==, 0);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &tcp, &tcp_registration, &err), ==, 0);
    g_assert_nonnull(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_nonnull(rpc_fake_tcp_endpoint(&harness.tcp, 111));

    qemu_slirp_rpc_registry_unregister(udp_registration);
    g_assert_nonnull(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_nonnull(rpc_fake_tcp_endpoint(&harness.tcp, 111));
    g_assert_cmpuint(harness.udp.removes, ==, 1);
    g_assert_cmpuint(harness.tcp.removes, ==, 0);

    qemu_slirp_rpc_registry_unregister(tcp_registration);
    g_assert_null(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_null(rpc_fake_tcp_endpoint(&harness.tcp, 111));
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    g_assert_cmpuint(harness.tcp.removes, ==, 2);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_portmapper_rollback(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 47 };
    OncRpcProgram program = rpc_program(200024, 1, 1, 4024,
                                         ONC_RPC_TRANSPORT_UDP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.fail_listen = true;
    harness.tcp.fail_port = 111;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, -1);
    g_assert_null(registration);
    g_assert_nonnull(err);
    g_assert_null(rpc_fake_udp_endpoint(&harness.udp, 111));
    g_assert_null(rpc_fake_tcp_endpoint(&harness.tcp, 111));
    g_assert_null(rpc_fake_udp_endpoint(&harness.udp, program.port));
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    error_free(err);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_portmap_dispatch_only_on_111(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 53 };
    OncRpcProgram program = rpc_program(200025, 1, 1, 4025,
                                         ONC_RPC_TRANSPORT_UDP |
                                         ONC_RPC_TRANSPORT_TCP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t body[32], call[96], record[100];
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    uint32_t value;
    size_t body_len, call_len, record_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);

    call_len = rpc_build_call(call, sizeof(call), 21, 100000, 2, 0, NULL, 0);
    rpc_send_udp_port(&harness, program.port, call, call_len);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_PROG_UNAVAIL);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.version_low));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, IPPROTO_UDP));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = rpc_build_call(call, sizeof(call), 22, 100000, 2, 3, body,
                              body_len);
    rpc_send_udp_port(&harness, program.port, call, call_len);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_PROG_UNAVAIL);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 23, 100000, 2, 0, NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    value = rpc_reply_status(harness.tcp.reply->data + 4,
                             harness.tcp.reply->len - 4, &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_PROG_UNAVAIL);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.version_low));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, IPPROTO_TCP));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = rpc_build_call(call, sizeof(call), 24, 100000, 2, 3, body,
                              body_len);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    value = rpc_reply_status(harness.tcp.reply->data + 4,
                             harness.tcp.reply->len - 4, &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_PROG_UNAVAIL);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    rpc_close_tcp(&harness);

    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_dump_uint32_max(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 43 };
    OncRpcProgram program = rpc_program(200018, UINT32_MAX - 1, UINT32_MAX,
                                         4018, ONC_RPC_TRANSPORT_UDP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128];
    size_t call_len;
    OncRpcXdrReader reply;
    uint32_t value;
    bool more;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    call_len = rpc_build_call(call, sizeof(call), 7, 100000, 2, 4, NULL, 0);
    rpc_send_udp(&harness, call, call_len);
    g_assert_cmpuint(rpc_reply_status(harness.udp.reply->data,
                                      harness.udp.reply->len, &reply), ==,
                     ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_bool(&reply, &more));
    g_assert_true(more);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, program.program);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, UINT32_MAX - 1);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, IPPROTO_UDP);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, program.port);
    g_assert_true(onc_rpc_xdr_bool(&reply, &more));
    g_assert_true(more);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, program.program);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, UINT32_MAX);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, IPPROTO_UDP);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, program.port);
    g_assert_true(onc_rpc_xdr_bool(&reply, &more));
    g_assert_false(more);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_dump_overflow(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 44 };
    OncRpcProgram program = rpc_program(200019, 1, 2000, 4019,
                                         ONC_RPC_TRANSPORT_UDP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128];
    size_t call_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    call_len = rpc_build_call(call, sizeof(call), 8, 100000, 2, 4, NULL, 0);
    rpc_send_udp(&harness, call, call_len);
    g_assert_cmpuint(harness.udp.reply->len, ==, 0);
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_version_mismatch(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 41 };
    OncRpcProgram program = rpc_program(200012, 2, 3, 4012,
                                         ONC_RPC_TRANSPORT_UDP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128];
    size_t call_len;
    OncRpcXdrReader reply;
    uint32_t value, low, high;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    call_len = rpc_build_call(call, sizeof(call), 4, program.program, 4, 1,
                              NULL, 0);
    rpc_send_udp_port(&harness, program.port, call, call_len);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_PROG_MISMATCH);
    g_assert_true(onc_rpc_xdr_u32(&reply, &low));
    g_assert_true(onc_rpc_xdr_u32(&reply, &high));
    g_assert_cmpuint(low, ==, program.version_low);
    g_assert_cmpuint(high, ==, program.version_high);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    qemu_slirp_rpc_registry_unregister(registration);
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_udp_request_endpoint_lifetime(void)
{
    RpcHarness harness;
    RpcProgramState shared_one = { .result = 45, .async = true };
    RpcProgramState shared_two = { .result = 46 };
    RpcProgramState final_state = { .result = 47, .async = true };
    OncRpcProgram one = rpc_program(200015, 1, 1, 4015,
                                    ONC_RPC_TRANSPORT_UDP, &shared_one);
    OncRpcProgram two = rpc_program(200015, 2, 2, 4015,
                                    ONC_RPC_TRANSPORT_UDP, &shared_two);
    OncRpcProgram final = rpc_program(200016, 1, 1, 4016,
                                      ONC_RPC_TRANSPORT_UDP, &final_state);
    QemuSlirpRpcRegistration *one_registration = NULL;
    QemuSlirpRpcRegistration *two_registration = NULL;
    QemuSlirpRpcRegistration *final_registration = NULL;
    uint8_t call[128], reply_bytes[64];
    size_t call_len;
    OncRpcXdrWriter writer;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &one, &one_registration, &err), ==, 0);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &two, &two_registration, &err), ==, 0);
    call_len = rpc_build_call(call, sizeof(call), 9, one.program, 1, 1,
                              NULL, 0);
    rpc_send_udp_port(&harness, one.port, call, call_len);
    g_assert_nonnull(shared_one.pending);
    qemu_slirp_rpc_registry_unregister(one_registration);
    onc_rpc_xdr_writer_init(&writer, reply_bytes, sizeof(reply_bytes));
    g_assert_true(onc_rpc_reply_success(&writer, 9));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, shared_one.result));
    g_assert_true(onc_rpc_request_reply(shared_one.pending, reply_bytes,
                                        onc_rpc_xdr_writer_size(&writer)));
    onc_rpc_request_unref(shared_one.pending);
    shared_one.pending = NULL;
    qemu_slirp_rpc_registry_unregister(two_registration);

    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &final, &final_registration, &err), ==, 0);
    call_len = rpc_build_call(call, sizeof(call), 10, final.program, 1, 1,
                              NULL, 0);
    rpc_send_udp_port(&harness, final.port, call, call_len);
    g_assert_nonnull(final_state.pending);
    qemu_slirp_rpc_registry_unregister(final_registration);
    g_assert_false(onc_rpc_request_reply(final_state.pending, reply_bytes, 4));
    onc_rpc_request_unref(final_state.pending);
    final_state.pending = NULL;
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_callback_teardown(void)
{
    RpcHarness harness;
    RpcProgramState state = { .teardown = true };
    OncRpcProgram program = rpc_program(200013, 1, 1, 4013,
                                         ONC_RPC_TRANSPORT_UDP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128];
    size_t call_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    state.registry = harness.rpc;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    g_byte_array_set_size(harness.udp.reply, 0);
    call_len = rpc_build_call(call, sizeof(call), 5, program.program, 1, 1,
                              NULL, 0);
    rpc_send_udp_port(&harness, program.port, call, call_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(harness.udp.reply->len, ==, 0);
    g_assert_cmpuint(harness.udp.removes, ==, 2);
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_teardown_during_receive(void)
{
    RpcHarness harness;
    RpcProgramState state = { .teardown = true };
    OncRpcProgram program = rpc_program(200014, 1, 1, 4014,
                                         ONC_RPC_TRANSPORT_TCP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    size_t call_len, record_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.close_on_remove = true;
    state.registry = harness.rpc;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 6, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(harness.tcp.removes, ==, 2);
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_registry_callit_async_and_cancel(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 99 };
    RpcProgramState tcp_state = { .result = 100 };
    OncRpcProgram program = rpc_program(200020, 1, 1, 4020,
                                         ONC_RPC_TRANSPORT_UDP, &state);
    OncRpcProgram tcp_program = rpc_program(200021, 1, 1, 4021,
                                             ONC_RPC_TRANSPORT_TCP,
                                             &tcp_state);
    QemuSlirpRpcRegistration *registration = NULL;
    QemuSlirpRpcRegistration *tcp_registration = NULL;
    uint8_t body[128], call[192], record[196], reply_bytes[64];
    size_t body_len, callit_body_len, call_len, record_len;
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    const uint8_t *opaque;
    uint32_t value;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &tcp_program, &tcp_registration, &err),
                    ==, 0);

    /* CALLIT invokes UDP only and wraps the target's encoded result. */
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.version_low));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 7));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(&writer, "arg", 3, 128));
    body_len = onc_rpc_xdr_writer_size(&writer);
    callit_body_len = body_len;
    call_len = rpc_build_call_auth_sys(call, sizeof(call), 10, 100000, 2, 5,
                                       body, body_len);
    rpc_send_udp(&harness, call, call_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(state.last_procedure, ==, 7);
    g_assert_cmpuint(state.last_auth_flavor, ==, ONC_RPC_AUTH_NULL);
    g_assert_cmpuint(state.last_uid, ==, 0);
    g_assert_cmpuint(state.last_gid, ==, 0);
    g_assert_cmpuint(state.last_group_count, ==, 0);
    g_assert_cmpstr(state.last_machine, ==, "");
    g_assert_cmpuint(harness.udp.reply->len, >, 0);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, program.port);
    g_assert_true(onc_rpc_xdr_counted_opaque(&reply, &opaque, &body_len,
                                              sizeof(reply_bytes)));
    g_assert_cmpuint(body_len, ==, 4);
    g_assert_cmphex(ldl_be_p(opaque), ==, state.result);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    /* Target errors and malformed target replies are silent CALLIT drops. */
    state.error_reply = true;
    g_byte_array_set_size(harness.udp.reply, 0);
    call_len = rpc_build_call(call, sizeof(call), 17, 100000, 2, 5, body,
                              callit_body_len);
    rpc_send_udp(&harness, call, call_len);
    g_assert_cmpuint(harness.udp.reply->len, ==, 0);
    state.error_reply = false;
    state.malformed_reply = true;
    g_byte_array_set_size(harness.udp.reply, 0);
    call_len = rpc_build_call(call, sizeof(call), 18, 100000, 2, 5, body,
                              callit_body_len);
    rpc_send_udp(&harness, call, call_len);
    g_assert_cmpuint(harness.udp.reply->len, ==, 0);
    state.malformed_reply = false;

    /* CALLIT silently drops a request whose UDP target is not registered. */
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 200099));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 1));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 7));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(&writer, "arg", 3, 128));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = rpc_build_call(call, sizeof(call), 10, 100000, 2, 5, body,
                              body_len);
    g_byte_array_set_size(harness.udp.reply, 0);
    rpc_send_udp(&harness, call, call_len);
    g_assert_cmpuint(harness.udp.reply->len, ==, 0);

    /*
     * An asynchronous CALLIT keeps the outer request until its child replies.
     */
    state.async = true;
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.version_low));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 8));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(&writer, "async", 5, 128));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = rpc_build_call(call, sizeof(call), 11, 100000, 2, 5, body,
                              body_len);
    g_byte_array_set_size(harness.udp.reply, 0);
    rpc_send_udp(&harness, call, call_len);
    g_assert_nonnull(state.pending);
    onc_rpc_xdr_writer_init(&writer, reply_bytes, sizeof(reply_bytes));
    g_assert_true(onc_rpc_reply_success(&writer, 11));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, state.result));
    g_assert_true(onc_rpc_request_reply(state.pending, reply_bytes,
                                        onc_rpc_xdr_writer_size(&writer)));
    g_assert_cmpuint(harness.udp.reply->len, >, 0);
    value = rpc_reply_status(harness.udp.reply->data, harness.udp.reply->len,
                             &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, program.port);
    g_assert_true(onc_rpc_xdr_counted_opaque(&reply, &opaque, &body_len,
                                              sizeof(reply_bytes)));
    g_assert_cmpuint(body_len, ==, 4);
    g_assert_cmphex(ldl_be_p(opaque), ==, state.result);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    onc_rpc_request_unref(state.pending);
    state.pending = NULL;
    state.async = false;

    /* CALLIT over TCP is unavailable even when UDP targets are registered. */
    onc_rpc_xdr_writer_init(&writer, body, sizeof(body));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.program));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, program.version_low));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 7));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(&writer, "arg", 3, 128));
    body_len = onc_rpc_xdr_writer_size(&writer);
    call_len = rpc_build_call(call, sizeof(call), 12, 100000, 2, 5, body,
                              body_len);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    g_byte_array_set_size(harness.tcp.reply, 0);
    rpc_connect_tcp(&harness);
    rpc_send_tcp(&harness, record, record_len);
    value = rpc_reply_status(harness.tcp.reply->data + 4,
                             harness.tcp.reply->len - 4, &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_PROC_UNAVAIL);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));
    rpc_close_tcp(&harness);

    /* An asynchronous request survives dispatch and can be replied later. */
    state.async = true;
    g_byte_array_set_size(harness.udp.reply, 0);
    call_len = rpc_build_call(call, sizeof(call), 13, program.program, 1, 8,
                              NULL, 0);
    rpc_send_udp_port(&harness, program.port, call, call_len);
    g_assert_nonnull(state.pending);
    onc_rpc_xdr_writer_init(&writer, reply_bytes, sizeof(reply_bytes));
    g_assert_true(onc_rpc_reply_success(&writer, 13));
    g_assert_true(onc_rpc_request_reply(state.pending, reply_bytes,
                                        onc_rpc_xdr_writer_size(&writer)));
    g_assert_cmpuint(harness.udp.reply->len, >, 0);
    onc_rpc_request_unref(state.pending);
    state.pending = NULL;

    /* A retained TCP request is cancelled when its connection closes. */
    tcp_state.async = true;
    rpc_connect_tcp_port(&harness, tcp_program.port);
    call_len = rpc_build_call(call, sizeof(call), 13, tcp_program.program, 1,
                              9, NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_nonnull(tcp_state.pending);
    rpc_close_tcp(&harness);
    g_assert_false(onc_rpc_request_reply(tcp_state.pending, reply_bytes, 4));
    onc_rpc_request_unref(tcp_state.pending);
    tcp_state.pending = NULL;

    /* Invalidation cancels retained requests and removes both endpoints. */
    state.async = true;
    call_len = rpc_build_call(call, sizeof(call), 12, program.program, 1, 9,
                              NULL, 0);
    rpc_send_udp_port(&harness, program.port, call, call_len);
    g_assert_nonnull(state.pending);
    qemu_slirp_rpc_registry_invalidate(harness.rpc);
    g_assert_false(onc_rpc_request_reply(state.pending, reply_bytes, 4));
    onc_rpc_request_unref(state.pending);
    state.pending = NULL;
    qemu_slirp_rpc_registry_unregister(registration);
    qemu_slirp_rpc_registry_unregister(tcp_registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_large_partial_progress(void)
{
    enum { PROGRAM = 200024, PORT = 4024 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .reply_length = 128 * 1024 + 4096,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    size_t call_len, record_len;
    unsigned sends;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.enforce_send_space = true;
    harness.tcp.send_space = 128 * 1024;
    harness.tcp.send_eagain_once = true;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 20, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 0);

    rpc_ready_tcp_with_space(&harness, 128 * 1024);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128 * 1024);
    g_assert_cmpuint(harness.tcp.send_space, ==, 0);
    sends = harness.tcp.sends;
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, sends + 1);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128 * 1024);
    g_assert_cmpuint(harness.tcp.send_space, ==, 0);
    sends = harness.tcp.sends;
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, sends + 1);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128 * 1024);
    rpc_ready_tcp_with_space(&harness, 4096);
    g_assert_cmpuint(harness.tcp.wire->len, ==,
                    128 * 1024 + 4096);
    rpc_ready_tcp_with_space(&harness, 4);
    g_assert_cmpuint(harness.tcp.wire->len, ==, state.reply_length + 4);
    g_assert_cmphex(ldl_be_p(harness.tcp.wire->data) & 0x7fffffff, ==,
                    state.reply_length);
    for (size_t i = 0; i < state.reply_length; i++) {
        g_assert_cmphex(harness.tcp.wire->data[i + 4], ==,
                        (uint8_t)(i * 37 + 11));
    }
    rpc_close_tcp(&harness);
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_zero_length(void)
{
    enum { PROGRAM = 200030, PORT = 4030 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .async = true,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    size_t call_len, record_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 70, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.pending_count, ==, 1);

    g_assert_true(onc_rpc_request_reply(state.pending[0], NULL, 0));
    g_assert_cmpuint(harness.tcp.reply->len, ==, 4);
    g_assert_cmphex((uint32_t)ldl_be_p(harness.tcp.reply->data), ==,
                    UINT32_C(0x80000000));

    onc_rpc_request_unref(state.pending[0]);
    state.pending[0] = NULL;
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_queue_order(void)
{
    enum { PROGRAM = 200025, PORT = 4025 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .reply_length = 8192,
        .tag_xid = true,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    size_t call_len, record_len;
    size_t attempts = 0;
    size_t offset;
    uint32_t xids[2] = { 21, 22 };
    unsigned sends;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.enforce_send_space = true;
    harness.tcp.send_space = 128;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    for (size_t i = 0; i < G_N_ELEMENTS(xids); i++) {
        harness.tcp.send_eagain_once = true;
        call_len = rpc_build_call(call, sizeof(call), xids[i], program.program,
                                  1, 1, NULL, 0);
        record_len = rpc_build_record(record, sizeof(record), call, call_len);
        rpc_send_tcp(&harness, record, record_len);
    }
    g_assert_cmpuint(state.calls, ==, G_N_ELEMENTS(xids));
    g_assert_cmpuint(harness.tcp.wire->len, ==, 0);

    rpc_ready_tcp_with_space(&harness, 128);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128);
    g_assert_cmpuint(harness.tcp.send_space, ==, 0);
    sends = harness.tcp.sends;
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, sends + 1);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128);
    g_assert_cmpuint(harness.tcp.send_space, ==, 0);
    sends = harness.tcp.sends;
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, sends + 1);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128);
    while (harness.tcp.wire->len < 2 * (state.reply_length + 4) &&
           attempts++ < 256) {
        rpc_ready_tcp_with_space(&harness, 128);
    }
    g_assert_cmpuint(harness.tcp.wire->len, ==,
                    2 * (state.reply_length + 4));
    offset = 0;
    for (size_t i = 0; i < G_N_ELEMENTS(xids); i++) {
        size_t length;

        g_assert_cmpuint(harness.tcp.wire->len - offset, >=, 4);
        length = ldl_be_p(harness.tcp.wire->data + offset) & 0x7fffffff;
        g_assert_cmpuint(length, ==, state.reply_length);
        g_assert_cmphex(ldl_be_p(harness.tcp.wire->data + offset + 4), ==,
                        xids[i]);
        offset += length + 4;
    }
    g_assert_cmpuint(offset, ==, harness.tcp.wire->len);
    rpc_close_tcp(&harness);
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_queue_cap_closes(void)
{
    enum { PROGRAM = 200026, PORT = 4026 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .reply_length = ONC_RPC_MAX_TCP_RECORD - 64,
        .async = true,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    g_autofree uint8_t *reply = NULL;
    size_t call_len, record_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.enforce_send_space = true;
    harness.tcp.send_space = 0;
    reply = g_malloc0(state.reply_length);
    reply[0] = 1;
    reply[1] = 2;
    reply[2] = 3;
    reply[3] = 4;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    for (uint32_t xid = 30; xid < 32; xid++) {
        call_len = rpc_build_call(call, sizeof(call), xid, program.program, 1,
                                  1, NULL, 0);
        record_len = rpc_build_record(record, sizeof(record), call, call_len);
        rpc_send_tcp(&harness, record, record_len);
    }
    g_assert_cmpuint(state.pending_count, ==, 2);
    g_assert_true(onc_rpc_request_reply(
        state.pending[0], reply, state.reply_length));
    g_assert_false(onc_rpc_request_reply(
        state.pending[1], reply, state.reply_length));
    g_assert_cmpuint(harness.tcp.closes, ==, 1);
    g_assert_false(onc_rpc_request_reply(
        state.pending[0], reply, state.reply_length));
    g_assert_false(onc_rpc_request_reply(
        state.pending[1], reply, state.reply_length));
    onc_rpc_request_unref(state.pending[0]);
    onc_rpc_request_unref(state.pending[1]);
    state.pending[0] = NULL;
    state.pending[1] = NULL;
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_partial_close_cancels_once(void)
{
    enum { PROGRAM = 200027, PORT = 4027 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .reply_length = 128 * 1024 + 4096,
        .async = true,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    g_autofree uint8_t *reply = NULL;
    size_t call_len, record_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.enforce_send_space = true;
    harness.tcp.send_space = 128 * 1024;
    reply = g_malloc0(state.reply_length);
    reply[0] = 1;
    reply[1] = 2;
    reply[2] = 3;
    reply[3] = 4;
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 40, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.pending_count, ==, 1);
    g_assert_true(onc_rpc_request_reply(
        state.pending[0], reply, state.reply_length));
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128 * 1024);
    rpc_close_tcp(&harness);
    g_assert_cmpuint(harness.tcp.closes, ==, 1);
    g_assert_false(onc_rpc_request_reply(
        state.pending[0], reply, state.reply_length));
    g_assert_false(onc_rpc_request_reply(
        state.pending[0], reply, state.reply_length));
    onc_rpc_request_unref(state.pending[0]);
    state.pending[0] = NULL;
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_invalidate_discards_partial_once(void)
{
    enum { PROGRAM = 200029, PORT = 4029 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .reply_length = 128 * 1024 + 4096,
        .async = true,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    g_autofree uint8_t *reply = NULL;
    size_t call_len, record_len;
    unsigned sends;
    size_t wire_length;
    Error *err = NULL;

    rpc_harness_init(&harness);
    harness.tcp.enforce_send_space = true;
    harness.tcp.send_space = 128 * 1024;
    reply = g_malloc0(state.reply_length);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 60, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(state.pending_count, ==, 1);
    g_assert_true(onc_rpc_request_reply(state.pending[0], reply,
                                        state.reply_length));
    g_assert_cmpuint(harness.tcp.wire->len, ==, 128 * 1024);
    sends = harness.tcp.sends;
    wire_length = harness.tcp.wire->len;

    qemu_slirp_rpc_registry_invalidate(harness.rpc);
    g_assert_cmpuint(harness.tcp.sends, ==, sends);
    g_assert_cmpuint(harness.tcp.wire->len, ==, wire_length);
    g_assert_cmpuint(harness.tcp.removes, ==, 2);
    g_assert_false(onc_rpc_request_reply(state.pending[0], reply,
                                         state.reply_length));
    g_assert_false(onc_rpc_request_reply(state.pending[0], reply,
                                         state.reply_length));

    qemu_slirp_rpc_registry_invalidate(harness.rpc);
    g_assert_cmpuint(harness.tcp.sends, ==, sends);
    g_assert_cmpuint(harness.tcp.wire->len, ==, wire_length);
    g_assert_false(onc_rpc_request_reply(state.pending[0], reply,
                                         state.reply_length));
    qemu_slirp_rpc_registry_unregister(registration);
    onc_rpc_request_unref(state.pending[0]);
    state.pending[0] = NULL;
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_eagain(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 101 };
    OncRpcProgram program = rpc_program(200022, 1, 1, 4022,
                                         ONC_RPC_TRANSPORT_TCP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132], reply_bytes[64];
    size_t call_len, record_len;
    OncRpcXdrWriter writer;
    OncRpcXdrReader reply;
    uint32_t value;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    harness.tcp.send_eagain_once = true;
    call_len = rpc_build_call(call, sizeof(call), 15, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.calls, ==, 1);
    g_assert_cmpuint(harness.tcp.sends, ==, 1);
    g_assert_cmpuint(harness.tcp.reply->len, ==, 0);
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, 2);
    g_assert_cmpuint(harness.tcp.reply->len, >, 0);
    value = rpc_reply_status(harness.tcp.reply->data + 4,
                             harness.tcp.reply->len - 4, &reply);
    g_assert_cmpuint(value, ==, ONC_RPC_SUCCESS);
    g_assert_true(onc_rpc_xdr_u32(&reply, &value));
    g_assert_cmpuint(value, ==, state.result);
    g_assert_true(onc_rpc_xdr_reader_empty(&reply));

    state.async = true;
    g_byte_array_set_size(harness.tcp.reply, 0);
    call_len = rpc_build_call(call, sizeof(call), 16, program.program, 1, 2,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_nonnull(state.pending);
    onc_rpc_xdr_writer_init(&writer, reply_bytes, sizeof(reply_bytes));
    g_assert_true(onc_rpc_reply_success(&writer, 16));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, state.result));
    harness.tcp.send_eagain_once = true;
    g_assert_true(onc_rpc_request_reply(state.pending, reply_bytes,
                                        onc_rpc_xdr_writer_size(&writer)));
    g_assert_cmpuint(harness.tcp.sends, ==, 3);
    g_assert_cmpuint(harness.tcp.reply->len, ==, 0);
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, 4);
    g_assert_cmpuint(harness.tcp.reply->len, >, 0);
    onc_rpc_request_unref(state.pending);
    state.pending = NULL;
    rpc_close_tcp(&harness);
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_fatal_closes_after_iteration(void)
{
    RpcHarness harness;
    RpcProgramState state = { .result = 102 };
    OncRpcProgram program = rpc_program(200023, 1, 1, 4023,
                                         ONC_RPC_TRANSPORT_TCP, &state);
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132];
    size_t call_len, record_len;
    Error *err = NULL;

    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);

    /* Queue two requests without caller-owned references. */
    harness.tcp.send_eagain_once = true;
    call_len = rpc_build_call(call, sizeof(call), 17, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    harness.tcp.send_eagain_once = true;
    call_len = rpc_build_call(call, sizeof(call), 18, program.program, 1, 2,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.calls, ==, 2);
    g_assert_cmpuint(harness.tcp.sends, ==, 2);

    /* The first retry fails; the fake close callback cancels the second. */
    harness.tcp.send_eio_once = true;
    rpc_ready_tcp(&harness);
    g_assert_cmpuint(harness.tcp.sends, ==, 3);
    g_assert_cmpuint(harness.tcp.reply->len, ==, 0);

    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

static void test_rpc_tcp_reply_send_close_reentrant(void)
{
    enum { PROGRAM = 200028, PORT = 4028 };
    RpcHarness harness;
    RpcQueueProgramState state = {
        .reply_length = 64,
        .async = true,
    };
    OncRpcProgram program = {
        .program = PROGRAM,
        .version_low = 1,
        .version_high = 1,
        .port = PORT,
        .transports = ONC_RPC_TRANSPORT_TCP,
        .dispatch = rpc_queue_program_dispatch,
        .opaque = &state,
    };
    QemuSlirpRpcRegistration *registration = NULL;
    uint8_t call[128], record[132], reply[64];
    size_t call_len, record_len;
    Error *err = NULL;

    memset(reply, 0x5a, sizeof(reply));
    rpc_harness_init(&harness);
    g_assert_cmpint(qemu_slirp_rpc_registry_register(
                        harness.rpc, &program, &registration, &err), ==, 0);
    rpc_connect_tcp_port(&harness, program.port);
    call_len = rpc_build_call(call, sizeof(call), 50, program.program, 1, 1,
                              NULL, 0);
    record_len = rpc_build_record(record, sizeof(record), call, call_len);
    rpc_send_tcp(&harness, record, record_len);
    g_assert_cmpuint(state.pending_count, ==, 1);

    harness.tcp.close_on_send = true;
    g_assert_false(onc_rpc_request_reply(state.pending[0], reply,
                                         sizeof(reply)));
    g_assert_cmpuint(harness.tcp.closes, ==, 1);
    g_assert_cmpuint(harness.tcp.wire->len, ==, 0);
    g_assert_false(onc_rpc_request_reply(state.pending[0], reply,
                                         sizeof(reply)));

    onc_rpc_request_unref(state.pending[0]);
    state.pending[0] = NULL;
    qemu_slirp_rpc_registry_unregister(registration);
    rpc_harness_cleanup(&harness);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/onc-rpc/calls/golden", test_golden_calls);
    g_test_add_func("/onc-rpc/calls/auth-sys", test_auth_sys);
    g_test_add_func("/onc-rpc/reject/readers", test_reader_rejections);
    g_test_add_func("/onc-rpc/reject/calls", test_rpc_rejections);
    g_test_add_func("/onc-rpc/reject/transport-bounds",
                    test_rpc_transport_bounds);
    g_test_add_func("/onc-rpc/xdr/scalars-arrays", test_scalars_and_arrays);
    g_test_add_func("/onc-rpc/xdr/writer-bounds", test_writer_bounds);
    g_test_add_func("/onc-rpc/replies/golden", test_reply_vectors);
    g_test_add_func("/onc-rpc/tcp/fragmented-header",
                    test_record_fragmented_header);
    g_test_add_func("/onc-rpc/tcp/multiple-fragments",
                    test_record_multiple_fragments);
    g_test_add_func("/onc-rpc/tcp/oversized-fragment",
                    test_record_oversized_fragment);
    g_test_add_func("/onc-rpc/tcp/two-records",
                    test_record_two_in_one_receive);
    g_test_add_func("/onc-rpc/tcp/truncated-close",
                    test_record_truncated_close);
    g_test_add_func("/onc-rpc/tcp/callback-reentry",
                    test_record_callback_reentry);
    g_test_add_func("/onc-rpc/tcp/callback-failure",
                    test_record_callback_failure);
    g_test_add_func("/onc-rpc/registry/registration",
                    test_rpc_registry_registration);
    g_test_add_func("/onc-rpc/registry/portmapper",
                    test_rpc_registry_portmapper);
    g_test_add_func("/onc-rpc/registry/portmapper-udp-first-owns-both",
                    test_rpc_registry_portmapper_udp_first_owns_both);
    g_test_add_func("/onc-rpc/registry/portmapper-tcp-first-udp-discovery",
                    test_rpc_registry_portmapper_tcp_first_udp_discovery);
    g_test_add_func("/onc-rpc/registry/portmapper-retained-until-last",
                    test_rpc_registry_portmapper_retained_until_last);
    g_test_add_func("/onc-rpc/registry/portmapper-rollback",
                    test_rpc_registry_portmapper_rollback);
    g_test_add_func("/onc-rpc/registry/portmap-dispatch-only-on-111",
                    test_rpc_portmap_dispatch_only_on_111);
    g_test_add_func("/onc-rpc/registry/dump-uint32-max",
                    test_rpc_registry_dump_uint32_max);
    g_test_add_func("/onc-rpc/registry/dump-overflow",
                    test_rpc_registry_dump_overflow);
    g_test_add_func("/onc-rpc/registry/version-mismatch",
                    test_rpc_registry_version_mismatch);
    g_test_add_func("/onc-rpc/registry/udp-request-endpoint-lifetime",
                    test_rpc_udp_request_endpoint_lifetime);
    g_test_add_func("/onc-rpc/registry/callback-teardown",
                    test_rpc_registry_callback_teardown);
    g_test_add_func("/onc-rpc/registry/tcp-teardown-during-receive",
                    test_rpc_tcp_teardown_during_receive);
    g_test_add_func("/onc-rpc/registry/callit-async-cancel",
                    test_rpc_registry_callit_async_and_cancel);
    g_test_add_func("/onc-rpc/registry/tcp-reply-eagain",
                    test_rpc_tcp_reply_eagain);
    g_test_add_func("/onc-rpc/registry/tcp-reply-fatal-close",
                    test_rpc_tcp_reply_fatal_closes_after_iteration);
    g_test_add_func("/onc-rpc/registry/tcp-reply-send-close-reentrant",
                    test_rpc_tcp_reply_send_close_reentrant);
    g_test_add_func("/onc-rpc/registry/tcp-reply-large-partial",
                    test_rpc_tcp_reply_large_partial_progress);
    g_test_add_func("/onc-rpc/registry/tcp-reply-zero-length",
                    test_rpc_tcp_reply_zero_length);
    g_test_add_func("/onc-rpc/registry/tcp-reply-queue-order",
                    test_rpc_tcp_reply_queue_order);
    g_test_add_func("/onc-rpc/registry/tcp-reply-queue-cap",
                    test_rpc_tcp_reply_queue_cap_closes);
    g_test_add_func("/onc-rpc/registry/tcp-reply-partial-close",
                    test_rpc_tcp_reply_partial_close_cancels_once);
    g_test_add_func("/onc-rpc/registry/tcp-reply-invalidate-partial",
                    test_rpc_tcp_reply_invalidate_discards_partial_once);
    return g_test_run();
}
