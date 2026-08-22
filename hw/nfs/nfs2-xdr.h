#ifndef HW_NFS_NFS2_XDR_H
#define HW_NFS_NFS2_XDR_H

#include "hw/nfs/nfs2-protocol.h"

typedef struct Nfs2XdrReader {
    const uint8_t *cursor;
    const uint8_t *end;
} Nfs2XdrReader;

typedef struct Nfs2XdrWriter {
    uint8_t *start;
    uint8_t *cursor;
    uint8_t *end;
} Nfs2XdrWriter;

typedef enum Nfs2RpcDecodeResult {
    NFS2_RPC_DECODE_OK,
    NFS2_RPC_DECODE_GARBAGE_ARGS,
    NFS2_RPC_DECODE_RPC_MISMATCH,
    NFS2_RPC_DECODE_AUTH_ERROR,
    NFS2_RPC_DECODE_TOO_LARGE,
} Nfs2RpcDecodeResult;

typedef struct Nfs2RpcCall {
    uint32_t xid;
    uint32_t program;
    uint32_t version;
    uint32_t procedure;
    uint32_t auth_flavor;
    uint32_t uid;
    uint32_t gid;
    char machine[NFS2_MAX_AUTH_MACHINE + 1];
    uint32_t groups[NFS2_MAX_AUTH_GROUPS];
    size_t group_count;
    Nfs2XdrReader body;
} Nfs2RpcCall;

void nfs2_xdr_reader_init(Nfs2XdrReader *r, const void *data, size_t len);
bool nfs2_xdr_reader_empty(const Nfs2XdrReader *r);
bool nfs2_xdr_u32(Nfs2XdrReader *r, uint32_t *value);
bool nfs2_xdr_opaque(Nfs2XdrReader *r, uint8_t *out, size_t exact);
bool nfs2_xdr_counted_opaque(Nfs2XdrReader *r, const uint8_t **out,
                             size_t *length, size_t maximum);
bool nfs2_xdr_string(Nfs2XdrReader *r, char *out, size_t capacity,
                     size_t maximum);

void nfs2_xdr_writer_init(Nfs2XdrWriter *w, void *data, size_t len);
size_t nfs2_xdr_writer_size(const Nfs2XdrWriter *w);
bool nfs2_xdr_put_u32(Nfs2XdrWriter *w, uint32_t value);
bool nfs2_xdr_put_opaque(Nfs2XdrWriter *w, const void *data, size_t len);
bool nfs2_xdr_put_counted_opaque(Nfs2XdrWriter *w, const void *data,
                                 size_t len, size_t maximum);

Nfs2RpcDecodeResult nfs2_rpc_decode_call(const uint8_t *data, size_t len,
                                         Nfs2RpcCall *call);
bool nfs2_xdr_decode_pmap_getport(Nfs2XdrReader *r,
                                  Nfs2PmapGetPortArgs *args);
bool nfs2_xdr_decode_mount_mnt(Nfs2XdrReader *r, Nfs2MountMntArgs *args);
bool nfs2_xdr_decode_fhandle(Nfs2XdrReader *r, Nfs2FileHandle *handle);
bool nfs2_xdr_decode_diropargs(Nfs2XdrReader *r, Nfs2Diropargs *args);
bool nfs2_xdr_decode_readargs(Nfs2XdrReader *r, Nfs2ReadArgs *args);
bool nfs2_xdr_decode_writeargs(Nfs2XdrReader *r, Nfs2WriteArgs *args);

bool nfs2_rpc_reply_success(Nfs2XdrWriter *w, uint32_t xid);
bool nfs2_rpc_reply_prog_unavail(Nfs2XdrWriter *w, uint32_t xid);
bool nfs2_rpc_reply_prog_mismatch(Nfs2XdrWriter *w, uint32_t xid,
                                  uint32_t low, uint32_t high);
bool nfs2_rpc_reply_proc_unavail(Nfs2XdrWriter *w, uint32_t xid);
bool nfs2_rpc_reply_garbage_args(Nfs2XdrWriter *w, uint32_t xid);
bool nfs2_rpc_reply_rpc_mismatch(Nfs2XdrWriter *w, uint32_t xid,
                                 uint32_t low, uint32_t high);
bool nfs2_rpc_reply_auth_error(Nfs2XdrWriter *w, uint32_t xid,
                               Nfs2RpcAuthStatus status);

#endif
