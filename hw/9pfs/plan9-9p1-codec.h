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

#ifndef HW_9PFS_PLAN9_9P1_CODEC_H
#define HW_9PFS_PLAN9_9P1_CODEC_H

#define PLAN9P1_NAMELEN 28
#define PLAN9P1_ERRLEN 64
#define PLAN9P1_DOMLEN 48
#define PLAN9P1_CHALLEN 8
#define PLAN9P1_TICKETLEN 72
#define PLAN9P1_AUTHLEN 13
#define PLAN9P1_DIRLEN 116
#define PLAN9P1_MAX_DATA 8192
#define PLAN9P1_MAX_FRAME (16 + PLAN9P1_MAX_DATA)

/* The First Edition placed session and attach messages in the initial gap. */
#define PLAN9P1_1E_TSESSION 52
#define PLAN9P1_1E_RSESSION 53
#define PLAN9P1_1E_TATTACH 58
#define PLAN9P1_1E_RATTACH 59

typedef struct Error Error;

typedef enum Plan9P1Type {
    PLAN9P1_TNOP = 50,
    PLAN9P1_RNOP = 51,
    PLAN9P1_RERROR = 55,
    PLAN9P1_TFLUSH = 56,
    PLAN9P1_RFLUSH = 57,
    PLAN9P1_TCLONE = 60,
    PLAN9P1_RCLONE = 61,
    PLAN9P1_TWALK = 62,
    PLAN9P1_RWALK = 63,
    PLAN9P1_TOPEN = 64,
    PLAN9P1_ROPEN = 65,
    PLAN9P1_TCREATE = 66,
    PLAN9P1_RCREATE = 67,
    PLAN9P1_TREAD = 68,
    PLAN9P1_RREAD = 69,
    PLAN9P1_TWRITE = 70,
    PLAN9P1_RWRITE = 71,
    PLAN9P1_TCLUNK = 72,
    PLAN9P1_RCLUNK = 73,
    PLAN9P1_TREMOVE = 74,
    PLAN9P1_RREMOVE = 75,
    PLAN9P1_TSTAT = 76,
    PLAN9P1_RSTAT = 77,
    PLAN9P1_TWSTAT = 78,
    PLAN9P1_RWSTAT = 79,
    PLAN9P1_TCLWALK = 80,
    PLAN9P1_RCLWALK = 81,
    PLAN9P1_TSESSION = 84,
    PLAN9P1_RSESSION = 85,
    PLAN9P1_TATTACH = 86,
    PLAN9P1_RATTACH = 87,
} Plan9P1Type;

typedef struct Plan9P1Qid {
    uint32_t path;
    uint32_t vers;
} Plan9P1Qid;

typedef struct Plan9P1Dir {
    uint8_t name[PLAN9P1_NAMELEN];
    uint8_t uid[PLAN9P1_NAMELEN];
    uint8_t gid[PLAN9P1_NAMELEN];
    Plan9P1Qid qid;
    uint32_t mode;
    uint32_t atime;
    uint32_t mtime;
    uint64_t length;
    uint16_t type;
    uint16_t dev;
} Plan9P1Dir;

typedef struct Plan9P1Fcall {
    Plan9P1Type type;
    bool first_edition;
    uint16_t tag;
    uint16_t fid;
    uint16_t newfid;
    uint16_t oldtag;
    uint8_t mode;
    uint32_t perm;
    uint64_t offset;
    uint16_t count;
    /* Points into the caller-owned input frame after decoding. */
    const uint8_t *data;
    uint8_t name[PLAN9P1_NAMELEN];
    uint8_t uname[PLAN9P1_NAMELEN];
    uint8_t aname[PLAN9P1_NAMELEN];
    uint8_t ename[PLAN9P1_ERRLEN];
    uint8_t challenge[PLAN9P1_CHALLEN];
    uint8_t ticket[PLAN9P1_TICKETLEN];
    uint8_t auth[PLAN9P1_AUTHLEN];
    uint8_t first_edition_auth[PLAN9P1_NAMELEN];
    uint8_t authid[PLAN9P1_NAMELEN];
    uint8_t authdom[PLAN9P1_DOMLEN];
    Plan9P1Qid qid;
    Plan9P1Dir dir;
} Plan9P1Fcall;

typedef struct Plan9P1Stream {
    uint8_t frame[PLAN9P1_MAX_FRAME];
    size_t used;
    bool failed;
    bool error_tag_valid;
    uint16_t error_tag;
} Plan9P1Stream;

typedef int (*Plan9P1FrameFn)(const uint8_t *frame, size_t length,
                              void *opaque, Error **errp);

/* Callback frames are borrowed from stream and valid only during the call. */
void plan9p1_stream_init(Plan9P1Stream *stream);
void plan9p1_stream_reset(Plan9P1Stream *stream);
int plan9p1_stream_feed(Plan9P1Stream *stream,
                        const uint8_t *buf, size_t len,
                        Plan9P1FrameFn emit, void *opaque,
                        Error **errp);
/* A framing failure poisons the stream until reset; known types retain tags. */
bool plan9p1_stream_error_tag(const Plan9P1Stream *stream, uint16_t *tag);

int plan9p1_decode(const uint8_t *buf, size_t len,
                   Plan9P1Fcall *fcall, Error **errp);
ssize_t plan9p1_encode(uint8_t *buf, size_t capacity,
                       const Plan9P1Fcall *fcall, Error **errp);

#endif /* HW_9PFS_PLAN9_9P1_CODEC_H */
