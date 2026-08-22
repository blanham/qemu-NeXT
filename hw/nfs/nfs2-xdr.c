#include "qemu/osdep.h"

#include "hw/nfs/nfs2-xdr.h"
#include "qemu/bswap.h"

static bool xdr_padded_size(size_t len, size_t *padded)
{
    if (len > SIZE_MAX - 3) {
        return false;
    }
    *padded = (len + 3) & ~(size_t)3;
    return true;
}

static size_t xdr_reader_remaining(const Nfs2XdrReader *r)
{
    return r->cursor <= r->end ? r->end - r->cursor : 0;
}

static size_t xdr_writer_remaining(const Nfs2XdrWriter *w)
{
    return w->cursor <= w->end ? w->end - w->cursor : 0;
}

void nfs2_xdr_reader_init(Nfs2XdrReader *r, const void *data, size_t len)
{
    r->cursor = data;
    r->end = r->cursor + len;
}

bool nfs2_xdr_reader_empty(const Nfs2XdrReader *r)
{
    return r->cursor == r->end;
}

bool nfs2_xdr_u32(Nfs2XdrReader *r, uint32_t *value)
{
    const uint8_t *p = r->cursor;

    if (xdr_reader_remaining(r) < 4) {
        return false;
    }
    *value = ldl_be_p(p);
    r->cursor = p + 4;
    return true;
}

bool nfs2_xdr_opaque(Nfs2XdrReader *r, uint8_t *out, size_t exact)
{
    const uint8_t *p = r->cursor;
    size_t padded;

    if (!xdr_padded_size(exact, &padded) ||
        xdr_reader_remaining(r) < padded) {
        return false;
    }
    for (size_t i = exact; i < padded; i++) {
        if (p[i] != 0) {
            return false;
        }
    }
    if (out && exact) {
        memcpy(out, p, exact);
    }
    r->cursor = p + padded;
    return true;
}

bool nfs2_xdr_counted_opaque(Nfs2XdrReader *r, const uint8_t **out,
                             size_t *length, size_t maximum)
{
    Nfs2XdrReader tmp = *r;
    uint32_t wire_length;
    size_t padded;

    if (!nfs2_xdr_u32(&tmp, &wire_length) || wire_length > maximum ||
        !xdr_padded_size(wire_length, &padded) ||
        xdr_reader_remaining(&tmp) < padded) {
        return false;
    }
    for (size_t i = wire_length; i < padded; i++) {
        if (tmp.cursor[i] != 0) {
            return false;
        }
    }
    if (out) {
        *out = tmp.cursor;
    }
    if (length) {
        *length = wire_length;
    }
    tmp.cursor += padded;
    *r = tmp;
    return true;
}

bool nfs2_xdr_string(Nfs2XdrReader *r, char *out, size_t capacity,
                     size_t maximum)
{
    Nfs2XdrReader tmp = *r;
    const uint8_t *value;
    size_t length;

    if (!out || !capacity ||
        !nfs2_xdr_counted_opaque(&tmp, &value, &length, maximum) ||
        length >= capacity || memchr(value, '\0', length)) {
        return false;
    }
    memcpy(out, value, length);
    out[length] = '\0';
    *r = tmp;
    return true;
}

void nfs2_xdr_writer_init(Nfs2XdrWriter *w, void *data, size_t len)
{
    w->start = data;
    w->cursor = data;
    w->end = w->cursor + len;
}

size_t nfs2_xdr_writer_size(const Nfs2XdrWriter *w)
{
    return w->cursor - w->start;
}

bool nfs2_xdr_put_u32(Nfs2XdrWriter *w, uint32_t value)
{
    if (xdr_writer_remaining(w) < 4) {
        return false;
    }
    stl_be_p(w->cursor, value);
    w->cursor += 4;
    return true;
}

bool nfs2_xdr_put_opaque(Nfs2XdrWriter *w, const void *data, size_t len)
{
    size_t padded;

    if ((len && !data) || !xdr_padded_size(len, &padded) ||
        xdr_writer_remaining(w) < padded) {
        return false;
    }
    if (len) {
        memcpy(w->cursor, data, len);
    }
    memset(w->cursor + len, 0, padded - len);
    w->cursor += padded;
    return true;
}

bool nfs2_xdr_put_counted_opaque(Nfs2XdrWriter *w, const void *data,
                                 size_t len, size_t maximum)
{
    size_t padded;

    if (len > maximum || len > UINT32_MAX || (len && !data) ||
        !xdr_padded_size(len, &padded) || padded > SIZE_MAX - 4 ||
        xdr_writer_remaining(w) < 4 + padded) {
        return false;
    }
    stl_be_p(w->cursor, len);
    if (len) {
        memcpy(w->cursor + 4, data, len);
    }
    memset(w->cursor + 4 + len, 0, padded - len);
    w->cursor += 4 + padded;
    return true;
}

static bool xdr_decode_auth_blob(Nfs2XdrReader *r, uint32_t *flavor,
                                 const uint8_t **data, size_t *len,
                                 bool *oversized)
{
    Nfs2XdrReader tmp = *r;
    uint32_t wire_len;

    *oversized = false;
    if (!nfs2_xdr_u32(&tmp, flavor) || !nfs2_xdr_u32(&tmp, &wire_len)) {
        return false;
    }
    if (wire_len > NFS2_MAX_AUTH_BYTES) {
        *oversized = true;
        return false;
    }
    tmp.cursor -= 4;
    if (!nfs2_xdr_counted_opaque(&tmp, data, len, NFS2_MAX_AUTH_BYTES)) {
        return false;
    }
    *r = tmp;
    return true;
}

static bool xdr_decode_auth_sys(const uint8_t *data, size_t len,
                                Nfs2RpcCall *call)
{
    Nfs2XdrReader auth;
    uint32_t stamp;
    uint32_t group_count;

    nfs2_xdr_reader_init(&auth, data, len);
    if (!nfs2_xdr_u32(&auth, &stamp) ||
        !nfs2_xdr_string(&auth, call->machine, sizeof(call->machine),
                         NFS2_MAX_AUTH_MACHINE) ||
        !nfs2_xdr_u32(&auth, &call->uid) ||
        !nfs2_xdr_u32(&auth, &call->gid) ||
        !nfs2_xdr_u32(&auth, &group_count) ||
        group_count > NFS2_MAX_AUTH_GROUPS) {
        return false;
    }
    for (size_t i = 0; i < group_count; i++) {
        if (!nfs2_xdr_u32(&auth, &call->groups[i])) {
            return false;
        }
    }
    call->group_count = group_count;
    return nfs2_xdr_reader_empty(&auth);
}

Nfs2RpcDecodeResult nfs2_rpc_decode_call(const uint8_t *data, size_t len,
                                         Nfs2RpcCall *call)
{
    Nfs2XdrReader r;
    const uint8_t *credential;
    const uint8_t *verifier;
    size_t credential_len;
    size_t verifier_len;
    uint32_t message_type;
    uint32_t rpc_version;
    uint32_t verifier_flavor;
    bool oversized = false;

    if (!data || !call) {
        return NFS2_RPC_DECODE_GARBAGE_ARGS;
    }
    if (len > NFS2_MAX_RPC_DATAGRAM) {
        return NFS2_RPC_DECODE_TOO_LARGE;
    }
    memset(call, 0, sizeof(*call));
    nfs2_xdr_reader_init(&r, data, len);
    if (!nfs2_xdr_u32(&r, &call->xid) ||
        !nfs2_xdr_u32(&r, &message_type) ||
        !nfs2_xdr_u32(&r, &rpc_version)) {
        return NFS2_RPC_DECODE_GARBAGE_ARGS;
    }
    if (message_type != NFS2_RPC_CALL) {
        return NFS2_RPC_DECODE_GARBAGE_ARGS;
    }
    if (rpc_version != NFS2_RPC_VERSION) {
        return NFS2_RPC_DECODE_RPC_MISMATCH;
    }
    if (!nfs2_xdr_u32(&r, &call->program) ||
        !nfs2_xdr_u32(&r, &call->version) ||
        !nfs2_xdr_u32(&r, &call->procedure) ||
        !xdr_decode_auth_blob(&r, &call->auth_flavor, &credential,
                              &credential_len, &oversized)) {
        return oversized ? NFS2_RPC_DECODE_AUTH_ERROR :
                           NFS2_RPC_DECODE_GARBAGE_ARGS;
    }
    switch (call->auth_flavor) {
    case NFS2_AUTH_NULL:
        if (credential_len != 0) {
            return NFS2_RPC_DECODE_AUTH_ERROR;
        }
        break;
    case NFS2_AUTH_SYS:
        if (!xdr_decode_auth_sys(credential, credential_len, call)) {
            return NFS2_RPC_DECODE_AUTH_ERROR;
        }
        break;
    default:
        return NFS2_RPC_DECODE_AUTH_ERROR;
    }
    if (!xdr_decode_auth_blob(&r, &verifier_flavor, &verifier,
                              &verifier_len, &oversized)) {
        return oversized ? NFS2_RPC_DECODE_AUTH_ERROR :
                           NFS2_RPC_DECODE_GARBAGE_ARGS;
    }
    if (verifier_flavor != NFS2_AUTH_NULL || verifier_len != 0) {
        return NFS2_RPC_DECODE_AUTH_ERROR;
    }
    call->body = r;
    return NFS2_RPC_DECODE_OK;
}

bool nfs2_xdr_decode_pmap_getport(Nfs2XdrReader *r,
                                  Nfs2PmapGetPortArgs *args)
{
    Nfs2XdrReader tmp = *r;
    Nfs2PmapGetPortArgs value;

    if (!nfs2_xdr_u32(&tmp, &value.program) ||
        !nfs2_xdr_u32(&tmp, &value.version) ||
        !nfs2_xdr_u32(&tmp, &value.protocol) ||
        !nfs2_xdr_u32(&tmp, &value.port) ||
        !nfs2_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_mount_mnt(Nfs2XdrReader *r, Nfs2MountMntArgs *args)
{
    Nfs2XdrReader tmp = *r;
    Nfs2MountMntArgs value;

    if (!nfs2_xdr_string(&tmp, value.path, sizeof(value.path),
                         NFS2_MAX_PATH) || !nfs2_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_fhandle(Nfs2XdrReader *r, Nfs2FileHandle *handle)
{
    Nfs2XdrReader tmp = *r;
    Nfs2FileHandle value;

    if (!nfs2_xdr_opaque(&tmp, value.bytes, sizeof(value.bytes)) ||
        !nfs2_xdr_reader_empty(&tmp)) {
        return false;
    }
    *handle = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_diropargs(Nfs2XdrReader *r, Nfs2Diropargs *args)
{
    Nfs2XdrReader tmp = *r;
    Nfs2Diropargs value;

    if (!nfs2_xdr_opaque(&tmp, value.dir.bytes, sizeof(value.dir.bytes)) ||
        !nfs2_xdr_string(&tmp, value.name, sizeof(value.name),
                         NFS2_MAX_NAME) || !value.name[0] ||
        !nfs2_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_readargs(Nfs2XdrReader *r, Nfs2ReadArgs *args)
{
    Nfs2XdrReader tmp = *r;
    Nfs2ReadArgs value;

    if (!nfs2_xdr_opaque(&tmp, value.file.bytes, sizeof(value.file.bytes)) ||
        !nfs2_xdr_u32(&tmp, &value.offset) ||
        !nfs2_xdr_u32(&tmp, &value.count) ||
        !nfs2_xdr_u32(&tmp, &value.total_count) ||
        value.count > NFS2_MAX_DATA || value.total_count > NFS2_MAX_DATA ||
        !nfs2_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

bool nfs2_xdr_decode_writeargs(Nfs2XdrReader *r, Nfs2WriteArgs *args)
{
    Nfs2XdrReader tmp = *r;
    Nfs2WriteArgs value;

    if (!nfs2_xdr_opaque(&tmp, value.file.bytes, sizeof(value.file.bytes)) ||
        !nfs2_xdr_u32(&tmp, &value.begin_offset) ||
        !nfs2_xdr_u32(&tmp, &value.offset) ||
        !nfs2_xdr_u32(&tmp, &value.total_count) ||
        value.total_count > NFS2_MAX_DATA ||
        !nfs2_xdr_counted_opaque(&tmp, &value.data, &value.data_length,
                                 NFS2_MAX_DATA) ||
        !nfs2_xdr_reader_empty(&tmp)) {
        return false;
    }
    *args = value;
    *r = tmp;
    return true;
}

static bool rpc_reply_accepted(Nfs2XdrWriter *w, uint32_t xid,
                               Nfs2RpcAcceptStatus status,
                               bool mismatch, uint32_t low, uint32_t high)
{
    size_t words = mismatch ? 8 : 6;

    if (xdr_writer_remaining(w) < words * 4) {
        return false;
    }
    nfs2_xdr_put_u32(w, xid);
    nfs2_xdr_put_u32(w, NFS2_RPC_REPLY);
    nfs2_xdr_put_u32(w, NFS2_RPC_MSG_ACCEPTED);
    nfs2_xdr_put_u32(w, NFS2_AUTH_NULL);
    nfs2_xdr_put_u32(w, 0);
    nfs2_xdr_put_u32(w, status);
    if (mismatch) {
        nfs2_xdr_put_u32(w, low);
        nfs2_xdr_put_u32(w, high);
    }
    return true;
}

bool nfs2_rpc_reply_success(Nfs2XdrWriter *w, uint32_t xid)
{
    return rpc_reply_accepted(w, xid, NFS2_RPC_SUCCESS, false, 0, 0);
}

bool nfs2_rpc_reply_prog_unavail(Nfs2XdrWriter *w, uint32_t xid)
{
    return rpc_reply_accepted(w, xid, NFS2_RPC_PROG_UNAVAIL, false, 0, 0);
}

bool nfs2_rpc_reply_prog_mismatch(Nfs2XdrWriter *w, uint32_t xid,
                                  uint32_t low, uint32_t high)
{
    return rpc_reply_accepted(w, xid, NFS2_RPC_PROG_MISMATCH, true, low,
                              high);
}

bool nfs2_rpc_reply_proc_unavail(Nfs2XdrWriter *w, uint32_t xid)
{
    return rpc_reply_accepted(w, xid, NFS2_RPC_PROC_UNAVAIL, false, 0, 0);
}

bool nfs2_rpc_reply_garbage_args(Nfs2XdrWriter *w, uint32_t xid)
{
    return rpc_reply_accepted(w, xid, NFS2_RPC_GARBAGE_ARGS, false, 0, 0);
}

bool nfs2_rpc_reply_system_err(Nfs2XdrWriter *w, uint32_t xid)
{
    return rpc_reply_accepted(w, xid, NFS2_RPC_SYSTEM_ERR, false, 0, 0);
}

bool nfs2_rpc_reply_rpc_mismatch(Nfs2XdrWriter *w, uint32_t xid,
                                 uint32_t low, uint32_t high)
{
    if (xdr_writer_remaining(w) < 6 * 4) {
        return false;
    }
    nfs2_xdr_put_u32(w, xid);
    nfs2_xdr_put_u32(w, NFS2_RPC_REPLY);
    nfs2_xdr_put_u32(w, NFS2_RPC_MSG_DENIED);
    nfs2_xdr_put_u32(w, NFS2_RPC_REJECT_MISMATCH);
    nfs2_xdr_put_u32(w, low);
    nfs2_xdr_put_u32(w, high);
    return true;
}

bool nfs2_rpc_reply_auth_error(Nfs2XdrWriter *w, uint32_t xid,
                               Nfs2RpcAuthStatus status)
{
    if (xdr_writer_remaining(w) < 5 * 4) {
        return false;
    }
    nfs2_xdr_put_u32(w, xid);
    nfs2_xdr_put_u32(w, NFS2_RPC_REPLY);
    nfs2_xdr_put_u32(w, NFS2_RPC_MSG_DENIED);
    nfs2_xdr_put_u32(w, NFS2_RPC_REJECT_AUTH_ERROR);
    nfs2_xdr_put_u32(w, status);
    return true;
}
