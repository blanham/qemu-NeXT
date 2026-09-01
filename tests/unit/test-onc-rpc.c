/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "net/onc-rpc.h"
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
    return g_test_run();
}
