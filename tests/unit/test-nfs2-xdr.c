#include "qemu/osdep.h"

#include "hw/nfs/nfs2-protocol.h"
#include "hw/nfs/nfs2-xdr.h"

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

static const uint8_t nfs_lookup[] = {
    0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 1, 0x86, 0xa3, 0, 0, 0, 2, 0, 0, 0, 4,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    0, 0, 0, 6, 'n', 'e', 't', 'b', 's', 'd', 0, 0,
};

static const uint8_t nfs_read[] = {
    0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 2,
    0, 1, 0x86, 0xa3, 0, 0, 0, 2, 0, 0, 0, 6,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    0, 0, 0x10, 0, 0, 0, 4, 0, 0, 0, 4, 0,
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

static Nfs2RpcCall decode_ok(const uint8_t *data, size_t len)
{
    Nfs2RpcCall call;

    g_assert_cmpint(nfs2_rpc_decode_call(data, len, &call), ==,
                    NFS2_RPC_DECODE_OK);
    return call;
}

static void test_golden_pmap(void)
{
    Nfs2RpcCall call = decode_ok(pmap_getport, sizeof(pmap_getport));
    Nfs2PmapGetPortArgs args;

    g_assert_cmphex(call.xid, ==, 0x12345678);
    g_assert_cmpuint(call.program, ==, NFS2_PMAP_PROGRAM);
    g_assert_cmpuint(call.version, ==, NFS2_PMAP_VERSION);
    g_assert_cmpuint(call.procedure, ==, NFS2_PMAP_GETPORT);
    g_assert_true(nfs2_xdr_decode_pmap_getport(&call.body, &args));
    g_assert_cmpuint(args.program, ==, NFS2_NFS_PROGRAM);
    g_assert_cmpuint(args.version, ==, NFS2_NFS_VERSION);
    g_assert_cmpuint(args.protocol, ==, NFS2_IPPROTO_UDP);
    g_assert_cmpuint(args.port, ==, 0);
}

static void test_golden_mount(void)
{
    Nfs2RpcCall call = decode_ok(mount_mnt_root, sizeof(mount_mnt_root));
    Nfs2MountMntArgs args;

    g_assert_cmpuint(call.program, ==, NFS2_MOUNT_PROGRAM);
    g_assert_cmpuint(call.version, ==, NFS2_MOUNT_VERSION);
    g_assert_cmpuint(call.procedure, ==, NFS2_MOUNT_MNT);
    g_assert_true(nfs2_xdr_decode_mount_mnt(&call.body, &args));
    g_assert_cmpstr(args.path, ==, "/");
}

static void test_golden_getattr(void)
{
    Nfs2RpcCall call = decode_ok(nfs_getattr, sizeof(nfs_getattr));
    Nfs2FileHandle handle;

    g_assert_cmpuint(call.procedure, ==, NFS2_NFSPROC_GETATTR);
    g_assert_true(nfs2_xdr_decode_fhandle(&call.body, &handle));
    for (size_t i = 0; i < sizeof(handle.bytes); i++) {
        g_assert_cmpuint(handle.bytes[i], ==, i);
    }
}

static void test_golden_lookup(void)
{
    Nfs2RpcCall call = decode_ok(nfs_lookup, sizeof(nfs_lookup));
    Nfs2Diropargs args;

    g_assert_cmpuint(call.procedure, ==, NFS2_NFSPROC_LOOKUP);
    g_assert_true(nfs2_xdr_decode_diropargs(&call.body, &args));
    g_assert_cmpstr(args.name, ==, "netbsd");
    g_assert_cmpuint(args.dir.bytes[31], ==, 31);
}

static void test_golden_read(void)
{
    Nfs2RpcCall call = decode_ok(nfs_read, sizeof(nfs_read));
    Nfs2ReadArgs args;

    g_assert_cmpuint(call.procedure, ==, NFS2_NFSPROC_READ);
    g_assert_true(nfs2_xdr_decode_readargs(&call.body, &args));
    g_assert_cmpuint(args.offset, ==, 4096);
    g_assert_cmpuint(args.count, ==, 1024);
    g_assert_cmpuint(args.total_count, ==, 1024);
}

static void test_golden_auth_sys(void)
{
    Nfs2RpcCall call = decode_ok(auth_sys_root, sizeof(auth_sys_root));

    g_assert_cmpuint(call.auth_flavor, ==, NFS2_AUTH_SYS);
    g_assert_cmpstr(call.machine, ==, "next");
    g_assert_cmpuint(call.uid, ==, 0);
    g_assert_cmpuint(call.gid, ==, 0);
    g_assert_cmpuint(call.group_count, ==, 2);
    g_assert_cmpuint(call.groups[0], ==, 0);
    g_assert_cmpuint(call.groups[1], ==, 10);
    g_assert_true(nfs2_xdr_reader_empty(&call.body));
}

static void test_reader_rejections(void)
{
    static const uint8_t short_word[] = { 0, 0, 0 };
    static const uint8_t huge_count[] = { 0xff, 0xff, 0xff, 0xff };
    static const uint8_t bad_padding[] = { 0, 0, 0, 1, 'x', 1, 0, 0 };
    Nfs2XdrReader r;
    const uint8_t *start;
    uint32_t value;
    const uint8_t *opaque;
    size_t length;
    char string[8];

    nfs2_xdr_reader_init(&r, short_word, sizeof(short_word));
    start = r.cursor;
    g_assert_false(nfs2_xdr_u32(&r, &value));
    g_assert_true(r.cursor == start);

    nfs2_xdr_reader_init(&r, huge_count, sizeof(huge_count));
    start = r.cursor;
    g_assert_false(nfs2_xdr_counted_opaque(&r, &opaque, &length,
                                           UINT32_MAX));
    g_assert_true(r.cursor == start);

    nfs2_xdr_reader_init(&r, bad_padding, sizeof(bad_padding));
    start = r.cursor;
    g_assert_false(nfs2_xdr_string(&r, string, sizeof(string), 7));
    g_assert_true(r.cursor == start);
}

static void test_rpc_rejections(void)
{
    uint8_t vector[sizeof(auth_sys_root)];
    uint8_t oversized[NFS2_MAX_RPC_DATAGRAM + 1] = { 0 };
    Nfs2RpcCall call;

    g_assert_cmpint(nfs2_rpc_decode_call(pmap_getport, 39, &call), ==,
                    NFS2_RPC_DECODE_GARBAGE_ARGS);
    g_assert_cmpint(nfs2_rpc_decode_call(oversized, sizeof(oversized), &call),
                    ==, NFS2_RPC_DECODE_TOO_LARGE);

    memcpy(vector, auth_sys_root, sizeof(vector));
    vector[11] = 3;
    g_assert_cmpint(nfs2_rpc_decode_call(vector, sizeof(vector), &call), ==,
                    NFS2_RPC_DECODE_RPC_MISMATCH);

    memcpy(vector, auth_sys_root, sizeof(vector));
    vector[55] = NFS2_MAX_AUTH_GROUPS + 1;
    g_assert_cmpint(nfs2_rpc_decode_call(vector, sizeof(vector), &call), ==,
                    NFS2_RPC_DECODE_AUTH_ERROR);
}

static void test_procedure_rejections(void)
{
    uint8_t trailing[sizeof(nfs_getattr) + 4];
    uint8_t short_handle[sizeof(nfs_getattr) - 1];
    uint8_t read_too_large[sizeof(nfs_read)];
    uint8_t long_path[40 + 4 + NFS2_MAX_PATH + 4] = { 0 };
    Nfs2RpcCall call;
    Nfs2FileHandle handle;
    Nfs2ReadArgs read_args;
    Nfs2MountMntArgs mount_args;
    Nfs2PmapGetPortArgs pmap_args;

    call = decode_ok(pmap_getport, sizeof(pmap_getport) - 1);
    g_assert_false(nfs2_xdr_decode_pmap_getport(&call.body, &pmap_args));

    memcpy(trailing, nfs_getattr, sizeof(nfs_getattr));
    memset(trailing + sizeof(nfs_getattr), 0, 4);
    call = decode_ok(trailing, sizeof(trailing));
    g_assert_false(nfs2_xdr_decode_fhandle(&call.body, &handle));

    memcpy(short_handle, nfs_getattr, sizeof(short_handle));
    call = decode_ok(short_handle, sizeof(short_handle));
    g_assert_false(nfs2_xdr_decode_fhandle(&call.body, &handle));

    memcpy(read_too_large, nfs_read, sizeof(read_too_large));
    read_too_large[78] = 0x20;
    read_too_large[79] = 1;
    read_too_large[82] = 0x20;
    read_too_large[83] = 1;
    call = decode_ok(read_too_large, sizeof(read_too_large));
    g_assert_false(nfs2_xdr_decode_readargs(&call.body, &read_args));

    memcpy(long_path, mount_mnt_root, 40);
    long_path[40] = 0;
    long_path[41] = 0;
    long_path[42] = 4;
    long_path[43] = 1;
    memset(long_path + 44, 'x', NFS2_MAX_PATH + 1);
    call = decode_ok(long_path, sizeof(long_path));
    g_assert_false(nfs2_xdr_decode_mount_mnt(&call.body, &mount_args));
}

static void test_write_bounds(void)
{
    size_t body_size = NFS2_FHSIZE + 16 + NFS2_MAX_DATA + 4;
    g_autofree uint8_t *packet = g_malloc0(40 + body_size);
    Nfs2XdrWriter writer;
    Nfs2RpcCall call;
    Nfs2WriteArgs args;
    uint8_t handle[NFS2_FHSIZE] = { 0 };
    uint8_t data[NFS2_MAX_DATA] = { 0 };
    size_t size;

    memcpy(packet, nfs_getattr, 40);
    packet[23] = NFS2_NFSPROC_WRITE;
    nfs2_xdr_writer_init(&writer, packet + 40, body_size);
    g_assert_true(nfs2_xdr_put_opaque(&writer, handle, sizeof(handle)));
    g_assert_true(nfs2_xdr_put_u32(&writer, 0));
    g_assert_true(nfs2_xdr_put_u32(&writer, 0));
    g_assert_true(nfs2_xdr_put_u32(&writer, NFS2_MAX_DATA));
    g_assert_true(nfs2_xdr_put_counted_opaque(&writer, data, sizeof(data),
                                              NFS2_MAX_DATA));
    size = 40 + nfs2_xdr_writer_size(&writer);
    call = decode_ok(packet, size);
    g_assert_true(nfs2_xdr_decode_writeargs(&call.body, &args));
    g_assert_cmpuint(args.data_length, ==, NFS2_MAX_DATA);

    packet[40 + NFS2_FHSIZE + 10] = 0x20;
    packet[40 + NFS2_FHSIZE + 11] = 1;
    call = decode_ok(packet, size);
    g_assert_false(nfs2_xdr_decode_writeargs(&call.body, &args));
}

static void test_reply_vectors_and_overflow(void)
{
    static const uint8_t success[] = {
        0x12, 0x34, 0x56, 0x78, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    uint8_t buffer[32];
    Nfs2XdrWriter writer;
    uint8_t *start;

    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_success(&writer, 0x12345678));
    g_assert_cmpuint(nfs2_xdr_writer_size(&writer), ==, sizeof(success));
    g_assert_cmpmem(buffer, sizeof(success), success, sizeof(success));

    nfs2_xdr_writer_init(&writer, buffer, 23);
    start = writer.cursor;
    g_assert_false(nfs2_rpc_reply_success(&writer, 0x12345678));
    g_assert_true(writer.cursor == start);

    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_prog_unavail(&writer, 1));
    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_prog_mismatch(&writer, 1, 1, 2));
    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_proc_unavail(&writer, 1));
    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_garbage_args(&writer, 1));
    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_rpc_mismatch(&writer, 1, 2, 2));
    nfs2_xdr_writer_init(&writer, buffer, sizeof(buffer));
    g_assert_true(nfs2_rpc_reply_auth_error(&writer, 1,
                                            NFS2_RPC_AUTH_BADCRED));

    nfs2_xdr_writer_init(&writer, buffer, 0);
    start = writer.cursor;
    g_assert_false(nfs2_rpc_reply_prog_unavail(&writer, 1));
    g_assert_true(writer.cursor == start);
    g_assert_false(nfs2_rpc_reply_prog_mismatch(&writer, 1, 1, 2));
    g_assert_true(writer.cursor == start);
    g_assert_false(nfs2_rpc_reply_proc_unavail(&writer, 1));
    g_assert_true(writer.cursor == start);
    g_assert_false(nfs2_rpc_reply_garbage_args(&writer, 1));
    g_assert_true(writer.cursor == start);
    g_assert_false(nfs2_rpc_reply_rpc_mismatch(&writer, 1, 2, 2));
    g_assert_true(writer.cursor == start);
    g_assert_false(nfs2_rpc_reply_auth_error(&writer, 1,
                                             NFS2_RPC_AUTH_BADCRED));
    g_assert_true(writer.cursor == start);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/nfs2-xdr/golden/pmap-getport", test_golden_pmap);
    g_test_add_func("/nfs2-xdr/golden/mount-mnt", test_golden_mount);
    g_test_add_func("/nfs2-xdr/golden/getattr", test_golden_getattr);
    g_test_add_func("/nfs2-xdr/golden/lookup", test_golden_lookup);
    g_test_add_func("/nfs2-xdr/golden/read", test_golden_read);
    g_test_add_func("/nfs2-xdr/golden/auth-sys", test_golden_auth_sys);
    g_test_add_func("/nfs2-xdr/reject/readers", test_reader_rejections);
    g_test_add_func("/nfs2-xdr/reject/rpc", test_rpc_rejections);
    g_test_add_func("/nfs2-xdr/reject/procedures",
                    test_procedure_rejections);
    g_test_add_func("/nfs2-xdr/reject/write-bounds", test_write_bounds);
    g_test_add_func("/nfs2-xdr/replies", test_reply_vectors_and_overflow);
    return g_test_run();
}
