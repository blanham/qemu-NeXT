/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "hw/nfs/nfs2-xdr.h"

bool nfs2_xdr_decode_pmap_getport(OncRpcXdrReader *r,
                                  Nfs2PmapGetPortArgs *args)
{
    OncRpcXdrReader tmp = *r;
    Nfs2PmapGetPortArgs value;

    if (!onc_rpc_xdr_u32(&tmp, &value.program) ||
        !onc_rpc_xdr_u32(&tmp, &value.version) ||
        !onc_rpc_xdr_u32(&tmp, &value.protocol) ||
        !onc_rpc_xdr_u32(&tmp, &value.port) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_mount_mnt(OncRpcXdrReader *r, Nfs2MountMntArgs *args)
{
    OncRpcXdrReader tmp = *r;
    Nfs2MountMntArgs value;

    if (!onc_rpc_xdr_string(&tmp, value.path, sizeof(value.path),
                            NFS2_MAX_PATH) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_fhandle(OncRpcXdrReader *r, Nfs2FileHandle *handle)
{
    OncRpcXdrReader tmp = *r;
    Nfs2FileHandle value;

    if (!onc_rpc_xdr_opaque(&tmp, value.bytes, sizeof(value.bytes)) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *handle = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_diropargs(OncRpcXdrReader *r, Nfs2Diropargs *args)
{
    OncRpcXdrReader tmp = *r;
    Nfs2Diropargs value;

    if (!onc_rpc_xdr_opaque(&tmp, value.dir.bytes, sizeof(value.dir.bytes)) ||
        !onc_rpc_xdr_string(&tmp, value.name, sizeof(value.name),
                            NFS2_MAX_NAME) || !value.name[0] ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_readargs(OncRpcXdrReader *r, Nfs2ReadArgs *args)
{
    OncRpcXdrReader tmp = *r;
    Nfs2ReadArgs value;

    if (!onc_rpc_xdr_opaque(&tmp, value.file.bytes, sizeof(value.file.bytes)) ||
        !onc_rpc_xdr_u32(&tmp, &value.offset) ||
        !onc_rpc_xdr_u32(&tmp, &value.count) ||
        !onc_rpc_xdr_u32(&tmp, &value.total_count) ||
        value.count > NFS2_MAX_DATA || value.total_count > NFS2_MAX_DATA ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_writeargs(OncRpcXdrReader *r, Nfs2WriteArgs *args)
{
    OncRpcXdrReader tmp = *r;
    Nfs2WriteArgs value;

    if (!onc_rpc_xdr_opaque(&tmp, value.file.bytes, sizeof(value.file.bytes)) ||
        !onc_rpc_xdr_u32(&tmp, &value.begin_offset) ||
        !onc_rpc_xdr_u32(&tmp, &value.offset) ||
        !onc_rpc_xdr_u32(&tmp, &value.total_count) ||
        value.total_count > NFS2_MAX_DATA ||
        !onc_rpc_xdr_counted_opaque(&tmp, &value.data, &value.data_length,
                                    NFS2_MAX_DATA) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}
