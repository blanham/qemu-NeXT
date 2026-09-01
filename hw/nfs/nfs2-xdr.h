#ifndef HW_NFS_NFS2_XDR_H
#define HW_NFS_NFS2_XDR_H

#include "net/onc-rpc.h"
#include "hw/nfs/nfs2-protocol.h"

bool nfs2_xdr_decode_mount_mnt(OncRpcXdrReader *r, Nfs2MountMntArgs *args);
bool nfs2_xdr_decode_fhandle(OncRpcXdrReader *r, Nfs2FileHandle *handle);
bool nfs2_xdr_decode_diropargs(OncRpcXdrReader *r, Nfs2Diropargs *args);
bool nfs2_xdr_decode_readargs(OncRpcXdrReader *r, Nfs2ReadArgs *args);
bool nfs2_xdr_decode_writeargs(OncRpcXdrReader *r, Nfs2WriteArgs *args);

#endif
