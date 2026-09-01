/* SPDX-License-Identifier: GPL-2.0-or-later */
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

static OncRpcCall decode_ok(const uint8_t *data, size_t len)
{
    OncRpcCall call;

    g_assert_cmpint(onc_rpc_decode_call(data, len, &call), ==,
                    ONC_RPC_DECODE_OK);
    return call;
}

static void test_nfs_argument_codecs(void)
{
    OncRpcCall call;
    Nfs2PmapGetPortArgs pmap;
    Nfs2MountMntArgs mount;
    Nfs2FileHandle handle;
    Nfs2Diropargs diropargs;
    Nfs2ReadArgs read;

    call = decode_ok(pmap_getport, sizeof(pmap_getport));
    g_assert_true(nfs2_xdr_decode_pmap_getport(&call.body, &pmap));
    g_assert_cmpuint(pmap.program, ==, NFS2_NFS_PROGRAM);
    g_assert_cmpuint(pmap.version, ==, NFS2_NFS_VERSION);
    g_assert_cmpuint(pmap.protocol, ==, NFS2_IPPROTO_UDP);
    g_assert_cmpuint(pmap.port, ==, 0);

    call = decode_ok(mount_mnt_root, sizeof(mount_mnt_root));
    g_assert_true(nfs2_xdr_decode_mount_mnt(&call.body, &mount));
    g_assert_cmpstr(mount.path, ==, "/");

    call = decode_ok(nfs_getattr, sizeof(nfs_getattr));
    g_assert_true(nfs2_xdr_decode_fhandle(&call.body, &handle));
    for (size_t i = 0; i < sizeof(handle.bytes); i++) {
        g_assert_cmpuint(handle.bytes[i], ==, i);
    }

    call = decode_ok(nfs_lookup, sizeof(nfs_lookup));
    g_assert_true(nfs2_xdr_decode_diropargs(&call.body, &diropargs));
    g_assert_cmpstr(diropargs.name, ==, "netbsd");
    g_assert_cmpuint(diropargs.dir.bytes[31], ==, 31);

    call = decode_ok(nfs_read, sizeof(nfs_read));
    g_assert_true(nfs2_xdr_decode_readargs(&call.body, &read));
    g_assert_cmpuint(read.offset, ==, 4096);
    g_assert_cmpuint(read.count, ==, 1024);
    g_assert_cmpuint(read.total_count, ==, 1024);
}

static void test_procedure_rejections(void)
{
    uint8_t trailing[sizeof(nfs_getattr) + 4];
    uint8_t short_handle[sizeof(nfs_getattr) - 1];
    uint8_t read_too_large[sizeof(nfs_read)];
    uint8_t long_path[40 + 4 + NFS2_MAX_PATH + 4] = { 0 };
    OncRpcCall call;
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
    OncRpcXdrWriter writer;
    OncRpcCall call;
    Nfs2WriteArgs args;
    uint8_t handle[NFS2_FHSIZE] = { 0 };
    uint8_t data[NFS2_MAX_DATA] = { 0 };
    size_t size;

    memcpy(packet, nfs_getattr, 40);
    packet[23] = NFS2_NFSPROC_WRITE;
    onc_rpc_xdr_writer_init(&writer, packet + 40, body_size);
    g_assert_true(onc_rpc_xdr_put_opaque(&writer, handle, sizeof(handle)));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
    g_assert_true(onc_rpc_xdr_put_u32(&writer, NFS2_MAX_DATA));
    g_assert_true(onc_rpc_xdr_put_counted_opaque(&writer, data, sizeof(data),
                                                 NFS2_MAX_DATA));
    size = 40 + onc_rpc_xdr_writer_size(&writer);
    call = decode_ok(packet, size);
    g_assert_true(nfs2_xdr_decode_writeargs(&call.body, &args));
    g_assert_cmpuint(args.data_length, ==, NFS2_MAX_DATA);

    packet[40 + NFS2_FHSIZE + 10] = 0x20;
    packet[40 + NFS2_FHSIZE + 11] = 1;
    call = decode_ok(packet, size);
    g_assert_false(nfs2_xdr_decode_writeargs(&call.body, &args));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/nfs2-xdr/arguments/golden",
                    test_nfs_argument_codecs);
    g_test_add_func("/nfs2-xdr/reject/procedures",
                    test_procedure_rejections);
    g_test_add_func("/nfs2-xdr/reject/write-bounds", test_write_bounds);
    return g_test_run();
}
