/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the names of the University of Illinois/NCSA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qemu/osdep.h"
#include "hw/9pfs/plan9-9p1-codec.h"
#include "qapi/error.h"

typedef struct Plan9P1Cursor {
    uint8_t *p;
    uint8_t *end;
} Plan9P1Cursor;

static uint16_t load_u16(const uint8_t *p)
{
    return p[0] | (p[1] << 8);
}

static uint32_t load_u32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t load_u64(const uint8_t *p)
{
    return load_u32(p) | ((uint64_t)load_u32(p + 4) << 32);
}

static void store_u16(uint8_t *p, uint16_t value)
{
    p[0] = value;
    p[1] = value >> 8;
}

static void store_u32(uint8_t *p, uint32_t value)
{
    p[0] = value;
    p[1] = value >> 8;
    p[2] = value >> 16;
    p[3] = value >> 24;
}

static void store_u64(uint8_t *p, uint64_t value)
{
    store_u32(p, value);
    store_u32(p + 4, value >> 32);
}

static size_t fixed_length(int64_t type)
{
    switch (type) {
    case PLAN9P1_TNOP:
    case PLAN9P1_RNOP:
    case PLAN9P1_RFLUSH:
        return 3;
    case PLAN9P1_RERROR:
        return 67;
    case PLAN9P1_TFLUSH:
    case PLAN9P1_RCLONE:
    case PLAN9P1_TCLUNK:
    case PLAN9P1_RCLUNK:
    case PLAN9P1_TREMOVE:
    case PLAN9P1_RREMOVE:
    case PLAN9P1_TSTAT:
    case PLAN9P1_RWSTAT:
        return 5;
    case PLAN9P1_TOPEN:
        return 6;
    case PLAN9P1_TCLONE:
    case PLAN9P1_RWRITE:
        return 7;
    case PLAN9P1_1E_TSESSION:
    case PLAN9P1_1E_RSESSION:
        return 3;
    case PLAN9P1_1E_TAUTH:
        return 69;
    case PLAN9P1_1E_RAUTH:
        return 35;
    case PLAN9P1_TSESSION:
        return 11;
    case PLAN9P1_RWALK:
    case PLAN9P1_ROPEN:
    case PLAN9P1_RCREATE:
    case PLAN9P1_RCLWALK:
        return 13;
    case PLAN9P1_TREAD:
        return 15;
    case PLAN9P1_RATTACH:
        return 26;
    case PLAN9P1_1E_RATTACH:
        return 13;
    case PLAN9P1_TWALK:
        return 33;
    case PLAN9P1_TCLWALK:
        return 35;
    case PLAN9P1_TCREATE:
        return 38;
    case PLAN9P1_RSESSION:
        return 87;
    case PLAN9P1_RSTAT:
    case PLAN9P1_TWSTAT:
        return 121;
    case PLAN9P1_TATTACH:
        return 146;
    case PLAN9P1_1E_TATTACH:
        return 89;
    case PLAN9P1_RREAD:
    case PLAN9P1_TWRITE:
        return 0;
    default:
        return SIZE_MAX;
    }
}

static int message_length(const uint8_t *buf, size_t available,
                          size_t *length, Error **errp)
{
    size_t fixed;
    size_t count_offset;
    size_t base;
    uint16_t count;

    if (!available) {
        *length = 0;
        return 0;
    }

    fixed = fixed_length(buf[0]);
    if (fixed == SIZE_MAX) {
        error_setg(errp, "unknown 9P1 message type %u", buf[0]);
        return -1;
    }
    if (fixed) {
        *length = fixed;
        return 1;
    }

    if (buf[0] == PLAN9P1_RREAD) {
        count_offset = 5;
        base = 8;
    } else {
        count_offset = 13;
        base = 16;
    }
    if (available < count_offset + 2) {
        *length = 0;
        return 0;
    }

    count = load_u16(buf + count_offset);
    if (count > PLAN9P1_MAX_DATA) {
        error_setg(errp, "9P1 data count %u exceeds maximum %u",
                   count, PLAN9P1_MAX_DATA);
        return -1;
    }
    *length = base + count;
    return 1;
}

void plan9p1_stream_init(Plan9P1Stream *stream)
{
    memset(stream, 0, sizeof(*stream));
}

void plan9p1_stream_reset(Plan9P1Stream *stream)
{
    memset(stream, 0, sizeof(*stream));
}

static void stream_fail(Plan9P1Stream *stream)
{
    stream->failed = true;
    if (stream->used >= 3) {
        stream->error_tag = load_u16(stream->frame + 1);
        stream->error_tag_valid = true;
    }
}

int plan9p1_stream_feed(Plan9P1Stream *stream,
                        const uint8_t *buf, size_t len,
                        Plan9P1FrameFn emit, void *opaque,
                        Error **errp)
{
    if (!stream) {
        error_setg(errp, "9P1 stream is NULL");
        return -1;
    }
    if (stream->failed) {
        error_setg(errp, "9P1 stream must be reset after a framing error");
        return -1;
    }
    if (len && !buf) {
        error_setg(errp, "9P1 input buffer is NULL");
        stream_fail(stream);
        return -1;
    }
    if (!emit) {
        error_setg(errp, "9P1 frame callback is NULL");
        stream_fail(stream);
        return -1;
    }

    while (len) {
        size_t frame_length;
        int ready;

        if (stream->used == sizeof(stream->frame)) {
            error_setg(errp, "9P1 frame exceeds maximum size");
            stream_fail(stream);
            return -1;
        }
        stream->frame[stream->used++] = *buf++;
        len--;

        ready = message_length(stream->frame, stream->used,
                               &frame_length, errp);
        if (ready < 0) {
            stream_fail(stream);
            return -1;
        }
        if (!ready || stream->used < frame_length) {
            continue;
        }
        if (stream->used != frame_length) {
            error_setg(errp, "internal 9P1 framing overflow");
            stream_fail(stream);
            return -1;
        }
        if (emit(stream->frame, stream->used, opaque, errp) < 0) {
            stream_fail(stream);
            return -1;
        }
        stream->used = 0;
        stream->error_tag_valid = false;
    }
    return 0;
}

bool plan9p1_stream_error_tag(const Plan9P1Stream *stream, uint16_t *tag)
{
    if (!stream || !stream->error_tag_valid) {
        return false;
    }
    if (tag) {
        *tag = stream->error_tag;
    }
    return true;
}

static bool cursor_get(Plan9P1Cursor *cursor, void *dst, size_t length)
{
    if (!length) {
        return true;
    }
    if (length > (size_t)(cursor->end - cursor->p)) {
        return false;
    }
    memcpy(dst, cursor->p, length);
    cursor->p += length;
    return true;
}

static bool cursor_get_u8(Plan9P1Cursor *cursor, uint8_t *value)
{
    return cursor_get(cursor, value, sizeof(*value));
}

static bool cursor_get_u16(Plan9P1Cursor *cursor, uint16_t *value)
{
    uint8_t bytes[2];

    if (!cursor_get(cursor, bytes, sizeof(bytes))) {
        return false;
    }
    *value = load_u16(bytes);
    return true;
}

static bool cursor_get_u32(Plan9P1Cursor *cursor, uint32_t *value)
{
    uint8_t bytes[4];

    if (!cursor_get(cursor, bytes, sizeof(bytes))) {
        return false;
    }
    *value = load_u32(bytes);
    return true;
}

static bool cursor_get_u64(Plan9P1Cursor *cursor, uint64_t *value)
{
    uint8_t bytes[8];

    if (!cursor_get(cursor, bytes, sizeof(bytes))) {
        return false;
    }
    *value = load_u64(bytes);
    return true;
}

static bool cursor_get_qid(Plan9P1Cursor *cursor, Plan9P1Qid *qid)
{
    return cursor_get_u32(cursor, &qid->path) &&
           cursor_get_u32(cursor, &qid->vers);
}

static bool cursor_get_slice(Plan9P1Cursor *cursor, size_t length,
                             const uint8_t **data)
{
    if (length > (size_t)(cursor->end - cursor->p)) {
        return false;
    }
    *data = cursor->p;
    cursor->p += length;
    return true;
}

static bool cursor_get_dir(Plan9P1Cursor *cursor, Plan9P1Dir *dir)
{
    uint32_t length_low;
    uint32_t length_high;

    if (!cursor_get(cursor, dir->name, sizeof(dir->name)) ||
        !cursor_get(cursor, dir->uid, sizeof(dir->uid)) ||
        !cursor_get(cursor, dir->gid, sizeof(dir->gid)) ||
        !cursor_get_qid(cursor, &dir->qid) ||
        !cursor_get_u32(cursor, &dir->mode) ||
        !cursor_get_u32(cursor, &dir->atime) ||
        !cursor_get_u32(cursor, &dir->mtime) ||
        !cursor_get_u32(cursor, &length_low) ||
        !cursor_get_u32(cursor, &length_high) ||
        !cursor_get_u16(cursor, &dir->type) ||
        !cursor_get_u16(cursor, &dir->dev) || length_high != 0) {
        return false;
    }
    dir->length = length_low;
    return true;
}

static int validate_message_length(const uint8_t *buf, size_t len,
                                   size_t *expected, Error **errp)
{
    int ready;

    ready = message_length(buf, len, expected, errp);
    if (ready <= 0) {
        if (!ready) {
            error_setg(errp, "incomplete 9P1 message header");
        }
        return -1;
    }
    if (len != *expected) {
        error_setg(errp, "9P1 message type %u has length %zu, expected %zu",
                   buf[0], len, *expected);
        return -1;
    }
    return 0;
}

int plan9p1_decode(const uint8_t *buf, size_t len,
                   Plan9P1Fcall *fcall, Error **errp)
{
    Plan9P1Cursor cursor;
    size_t expected;
    uint8_t pad;

    if (!fcall) {
        error_setg(errp, "9P1 output fcall is NULL");
        return -1;
    }
    memset(fcall, 0, sizeof(*fcall));
    if (!buf || !len) {
        error_setg(errp, "9P1 message is empty");
        return -1;
    }

    switch (buf[0]) {
    case PLAN9P1_1E_TSESSION:
        fcall->type = PLAN9P1_TSESSION;
        fcall->first_edition = true;
        break;
    case PLAN9P1_1E_RSESSION:
        fcall->type = PLAN9P1_RSESSION;
        fcall->first_edition = true;
        break;
    case PLAN9P1_1E_TAUTH:
        fcall->type = PLAN9P1_TAUTH;
        fcall->first_edition = true;
        break;
    case PLAN9P1_1E_RAUTH:
        fcall->type = PLAN9P1_RAUTH;
        fcall->first_edition = true;
        break;
    case PLAN9P1_1E_TATTACH:
        fcall->type = PLAN9P1_TATTACH;
        fcall->first_edition = true;
        break;
    case PLAN9P1_1E_RATTACH:
        fcall->type = PLAN9P1_RATTACH;
        fcall->first_edition = true;
        break;
    default:
        fcall->type = buf[0];
        break;
    }
    if (len >= 3) {
        fcall->tag = load_u16(buf + 1);
    }
    if (validate_message_length(buf, len, &expected, errp) < 0) {
        return -1;
    }

    cursor.p = (uint8_t *)buf + 3;
    cursor.end = (uint8_t *)buf + len;
#define GET(field) do { if (!(field)) { goto malformed; } } while (0)
    switch (fcall->type) {
    case PLAN9P1_TNOP:
    case PLAN9P1_RNOP:
    case PLAN9P1_RFLUSH:
        break;
    case PLAN9P1_RERROR:
        GET(cursor_get(&cursor, fcall->ename, sizeof(fcall->ename)));
        break;
    case PLAN9P1_TFLUSH:
        GET(cursor_get_u16(&cursor, &fcall->oldtag));
        break;
    case PLAN9P1_TCLONE:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u16(&cursor, &fcall->newfid));
        break;
    case PLAN9P1_RCLONE:
    case PLAN9P1_TCLUNK:
    case PLAN9P1_RCLUNK:
    case PLAN9P1_TREMOVE:
    case PLAN9P1_RREMOVE:
    case PLAN9P1_TSTAT:
    case PLAN9P1_RWSTAT:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        break;
    case PLAN9P1_TWALK:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get(&cursor, fcall->name, sizeof(fcall->name)));
        break;
    case PLAN9P1_RWALK:
    case PLAN9P1_ROPEN:
    case PLAN9P1_RCREATE:
    case PLAN9P1_RCLWALK:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_qid(&cursor, &fcall->qid));
        break;
    case PLAN9P1_TOPEN:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u8(&cursor, &fcall->mode));
        break;
    case PLAN9P1_TCREATE:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get(&cursor, fcall->name, sizeof(fcall->name)));
        GET(cursor_get_u32(&cursor, &fcall->perm));
        GET(cursor_get_u8(&cursor, &fcall->mode));
        break;
    case PLAN9P1_TREAD:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u64(&cursor, &fcall->offset));
        GET(cursor_get_u16(&cursor, &fcall->count));
        if (fcall->count > PLAN9P1_MAX_DATA) {
            error_setg(errp, "9P1 data count %u exceeds maximum %u",
                       fcall->count, PLAN9P1_MAX_DATA);
            return -1;
        }
        break;
    case PLAN9P1_RREAD:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u16(&cursor, &fcall->count));
        GET(cursor_get_u8(&cursor, &pad));
        GET(cursor_get_slice(&cursor, fcall->count, &fcall->data));
        break;
    case PLAN9P1_TWRITE:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u64(&cursor, &fcall->offset));
        GET(cursor_get_u16(&cursor, &fcall->count));
        GET(cursor_get_u8(&cursor, &pad));
        GET(cursor_get_slice(&cursor, fcall->count, &fcall->data));
        break;
    case PLAN9P1_RWRITE:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u16(&cursor, &fcall->count));
        if (fcall->count > PLAN9P1_MAX_DATA) {
            error_setg(errp, "9P1 data count %u exceeds maximum %u",
                       fcall->count, PLAN9P1_MAX_DATA);
            return -1;
        }
        break;
    case PLAN9P1_RSTAT:
    case PLAN9P1_TWSTAT:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        if (!cursor_get_dir(&cursor, &fcall->dir)) {
            error_setg(errp, "invalid 9P1 directory record");
            return -1;
        }
        break;
    case PLAN9P1_TCLWALK:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_u16(&cursor, &fcall->newfid));
        GET(cursor_get(&cursor, fcall->name, sizeof(fcall->name)));
        break;
    case PLAN9P1_TAUTH:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get(&cursor, fcall->uname, sizeof(fcall->uname)));
        GET(cursor_get(&cursor, fcall->first_edition_challenge,
                       sizeof(fcall->first_edition_challenge)));
        break;
    case PLAN9P1_RAUTH:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get(&cursor, fcall->first_edition_reply,
                       sizeof(fcall->first_edition_reply)));
        break;
    case PLAN9P1_TSESSION:
        if (!fcall->first_edition) {
            GET(cursor_get(&cursor, fcall->challenge,
                           sizeof(fcall->challenge)));
        }
        break;
    case PLAN9P1_RSESSION:
        if (!fcall->first_edition) {
            GET(cursor_get(&cursor, fcall->challenge,
                           sizeof(fcall->challenge)));
            GET(cursor_get(&cursor, fcall->authid, sizeof(fcall->authid)));
            GET(cursor_get(&cursor, fcall->authdom, sizeof(fcall->authdom)));
        }
        break;
    case PLAN9P1_TATTACH:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get(&cursor, fcall->uname, sizeof(fcall->uname)));
        GET(cursor_get(&cursor, fcall->aname, sizeof(fcall->aname)));
        if (fcall->first_edition) {
            GET(cursor_get(&cursor, fcall->first_edition_auth,
                           sizeof(fcall->first_edition_auth)));
        } else {
            GET(cursor_get(&cursor, fcall->ticket, sizeof(fcall->ticket)));
            GET(cursor_get(&cursor, fcall->auth, sizeof(fcall->auth)));
        }
        break;
    case PLAN9P1_RATTACH:
        GET(cursor_get_u16(&cursor, &fcall->fid));
        GET(cursor_get_qid(&cursor, &fcall->qid));
        if (!fcall->first_edition) {
            GET(cursor_get(&cursor, fcall->auth, sizeof(fcall->auth)));
        }
        break;
    default:
        g_assert_not_reached();
    }
    if (cursor.p != cursor.end) {
        goto malformed;
    }
#undef GET
    return 0;

malformed:
#undef GET
    error_setg(errp, "invalid 9P1 message body");
    return -1;
}

static bool cursor_put(Plan9P1Cursor *cursor, const void *src, size_t length)
{
    if (!length) {
        return true;
    }
    if (length > (size_t)(cursor->end - cursor->p)) {
        return false;
    }
    memcpy(cursor->p, src, length);
    cursor->p += length;
    return true;
}

static bool cursor_put_u8(Plan9P1Cursor *cursor, uint8_t value)
{
    return cursor_put(cursor, &value, sizeof(value));
}

static bool cursor_put_u16(Plan9P1Cursor *cursor, uint16_t value)
{
    uint8_t bytes[2];

    store_u16(bytes, value);
    return cursor_put(cursor, bytes, sizeof(bytes));
}

static bool cursor_put_u32(Plan9P1Cursor *cursor, uint32_t value)
{
    uint8_t bytes[4];

    store_u32(bytes, value);
    return cursor_put(cursor, bytes, sizeof(bytes));
}

static bool cursor_put_u64(Plan9P1Cursor *cursor, uint64_t value)
{
    uint8_t bytes[8];

    store_u64(bytes, value);
    return cursor_put(cursor, bytes, sizeof(bytes));
}

static bool cursor_put_qid(Plan9P1Cursor *cursor, const Plan9P1Qid *qid)
{
    return cursor_put_u32(cursor, qid->path) &&
           cursor_put_u32(cursor, qid->vers);
}

static bool cursor_put_dir(Plan9P1Cursor *cursor, const Plan9P1Dir *dir)
{
    return cursor_put(cursor, dir->name, sizeof(dir->name)) &&
           cursor_put(cursor, dir->uid, sizeof(dir->uid)) &&
           cursor_put(cursor, dir->gid, sizeof(dir->gid)) &&
           cursor_put_qid(cursor, &dir->qid) &&
           cursor_put_u32(cursor, dir->mode) &&
           cursor_put_u32(cursor, dir->atime) &&
           cursor_put_u32(cursor, dir->mtime) &&
           cursor_put_u32(cursor, dir->length) &&
           cursor_put_u32(cursor, 0) &&
           cursor_put_u16(cursor, dir->type) &&
           cursor_put_u16(cursor, dir->dev);
}

static ssize_t encoded_length(const Plan9P1Fcall *fcall, Error **errp)
{
    int64_t type = fcall->type;
    size_t length = fixed_length(type);

    if (fcall->first_edition) {
        switch (fcall->type) {
        case PLAN9P1_TSESSION:
        case PLAN9P1_RSESSION:
            return 3;
        case PLAN9P1_TATTACH:
            return 89;
        case PLAN9P1_RATTACH:
            return 13;
        default:
            break;
        }
    }

    if (length == SIZE_MAX) {
        error_setg(errp, "unknown 9P1 message type %" PRId64, type);
        return -1;
    }
    if (fcall->count > PLAN9P1_MAX_DATA &&
        (fcall->type == PLAN9P1_TREAD ||
         fcall->type == PLAN9P1_RREAD ||
         fcall->type == PLAN9P1_TWRITE ||
         fcall->type == PLAN9P1_RWRITE)) {
        error_setg(errp, "9P1 data count %u exceeds maximum %u",
                   fcall->count, PLAN9P1_MAX_DATA);
        return -1;
    }
    if (fcall->type == PLAN9P1_RREAD) {
        return 8 + fcall->count;
    }
    if (fcall->type == PLAN9P1_TWRITE) {
        return 16 + fcall->count;
    }
    return length;
}

ssize_t plan9p1_encode(uint8_t *buf, size_t capacity,
                       const Plan9P1Fcall *fcall, Error **errp)
{
    Plan9P1Cursor cursor;
    ssize_t length;
    uint8_t type = fcall ? fcall->type : 0;

    if (!fcall) {
        error_setg(errp, "9P1 input fcall is NULL");
        return -1;
    }
    length = encoded_length(fcall, errp);
    if (length < 0) {
        return -1;
    }
    if ((fcall->type == PLAN9P1_RSTAT ||
         fcall->type == PLAN9P1_TWSTAT) && fcall->dir.length > UINT32_MAX) {
        error_setg(errp, "9P1 directory length exceeds 32 bits");
        return -1;
    }
    if ((fcall->type == PLAN9P1_RREAD ||
         fcall->type == PLAN9P1_TWRITE) && fcall->count && !fcall->data) {
        error_setg(errp, "9P1 data pointer is NULL for nonzero count");
        return -1;
    }
    if (!buf || capacity < (size_t)length) {
        error_setg(errp, "9P1 output capacity %zu is smaller than %zd",
                   capacity, length);
        return -1;
    }

    cursor.p = buf;
    cursor.end = buf + length;
    if (fcall->first_edition) {
        switch (fcall->type) {
        case PLAN9P1_TSESSION:
            type = PLAN9P1_1E_TSESSION;
            break;
        case PLAN9P1_RSESSION:
            type = PLAN9P1_1E_RSESSION;
            break;
        case PLAN9P1_TATTACH:
            type = PLAN9P1_1E_TATTACH;
            break;
        case PLAN9P1_RATTACH:
            type = PLAN9P1_1E_RATTACH;
            break;
        default:
            break;
        }
    }
    cursor_put_u8(&cursor, type);
    cursor_put_u16(&cursor, fcall->tag);
    switch (fcall->type) {
    case PLAN9P1_TNOP:
    case PLAN9P1_RNOP:
    case PLAN9P1_RFLUSH:
        break;
    case PLAN9P1_RERROR:
        cursor_put(&cursor, fcall->ename, sizeof(fcall->ename));
        break;
    case PLAN9P1_TFLUSH:
        cursor_put_u16(&cursor, fcall->oldtag);
        break;
    case PLAN9P1_TCLONE:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u16(&cursor, fcall->newfid);
        break;
    case PLAN9P1_RCLONE:
    case PLAN9P1_TCLUNK:
    case PLAN9P1_RCLUNK:
    case PLAN9P1_TREMOVE:
    case PLAN9P1_RREMOVE:
    case PLAN9P1_TSTAT:
    case PLAN9P1_RWSTAT:
        cursor_put_u16(&cursor, fcall->fid);
        break;
    case PLAN9P1_TWALK:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put(&cursor, fcall->name, sizeof(fcall->name));
        break;
    case PLAN9P1_RWALK:
    case PLAN9P1_ROPEN:
    case PLAN9P1_RCREATE:
    case PLAN9P1_RCLWALK:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_qid(&cursor, &fcall->qid);
        break;
    case PLAN9P1_TOPEN:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u8(&cursor, fcall->mode);
        break;
    case PLAN9P1_TCREATE:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put(&cursor, fcall->name, sizeof(fcall->name));
        cursor_put_u32(&cursor, fcall->perm);
        cursor_put_u8(&cursor, fcall->mode);
        break;
    case PLAN9P1_TREAD:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u64(&cursor, fcall->offset);
        cursor_put_u16(&cursor, fcall->count);
        break;
    case PLAN9P1_RREAD:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u16(&cursor, fcall->count);
        cursor_put_u8(&cursor, 0);
        cursor_put(&cursor, fcall->data, fcall->count);
        break;
    case PLAN9P1_TWRITE:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u64(&cursor, fcall->offset);
        cursor_put_u16(&cursor, fcall->count);
        cursor_put_u8(&cursor, 0);
        cursor_put(&cursor, fcall->data, fcall->count);
        break;
    case PLAN9P1_RWRITE:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u16(&cursor, fcall->count);
        break;
    case PLAN9P1_RSTAT:
    case PLAN9P1_TWSTAT:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_dir(&cursor, &fcall->dir);
        break;
    case PLAN9P1_TCLWALK:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_u16(&cursor, fcall->newfid);
        cursor_put(&cursor, fcall->name, sizeof(fcall->name));
        break;
    case PLAN9P1_TAUTH:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put(&cursor, fcall->uname, sizeof(fcall->uname));
        cursor_put(&cursor, fcall->first_edition_challenge,
                   sizeof(fcall->first_edition_challenge));
        break;
    case PLAN9P1_RAUTH:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put(&cursor, fcall->first_edition_reply,
                   sizeof(fcall->first_edition_reply));
        break;
    case PLAN9P1_TSESSION:
        if (!fcall->first_edition) {
            cursor_put(&cursor, fcall->challenge, sizeof(fcall->challenge));
        }
        break;
    case PLAN9P1_RSESSION:
        if (!fcall->first_edition) {
            cursor_put(&cursor, fcall->challenge, sizeof(fcall->challenge));
            cursor_put(&cursor, fcall->authid, sizeof(fcall->authid));
            cursor_put(&cursor, fcall->authdom, sizeof(fcall->authdom));
        }
        break;
    case PLAN9P1_TATTACH:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put(&cursor, fcall->uname, sizeof(fcall->uname));
        cursor_put(&cursor, fcall->aname, sizeof(fcall->aname));
        if (fcall->first_edition) {
            cursor_put(&cursor, fcall->first_edition_auth,
                       sizeof(fcall->first_edition_auth));
        } else {
            cursor_put(&cursor, fcall->ticket, sizeof(fcall->ticket));
            cursor_put(&cursor, fcall->auth, sizeof(fcall->auth));
        }
        break;
    case PLAN9P1_RATTACH:
        cursor_put_u16(&cursor, fcall->fid);
        cursor_put_qid(&cursor, &fcall->qid);
        if (!fcall->first_edition) {
            cursor_put(&cursor, fcall->auth, sizeof(fcall->auth));
        }
        break;
    default:
        g_assert_not_reached();
    }
    g_assert(cursor.p == cursor.end);
    return length;
}
