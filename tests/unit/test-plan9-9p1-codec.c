/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/9pfs/plan9-9p1-codec.h"
#include "qapi/error.h"

typedef struct WireCase {
    const char *name;
    const uint8_t *wire;
    size_t length;
} WireCase;

#define WIRE_CASE(n) { #n, wire_##n, sizeof(wire_##n) }

static const uint8_t wire_tnop[3] = { 0x32, 0x34, 0x12 };
static const uint8_t wire_rnop[3] = { 0x33, 0x35, 0x12 };
static const uint8_t wire_rerror[67] = {
    0x37, 0x36, 0x12, 'b', 'a', 'd',
};
static const uint8_t wire_tflush[5] = {
    0x38, 0x37, 0x12, 0xef, 0xbe,
};
static const uint8_t wire_rflush[3] = { 0x39, 0x38, 0x12 };
static const uint8_t wire_tclone[7] = {
    0x3c, 0x39, 0x12, 0x2a, 0x00, 0x2b, 0x00,
};
static const uint8_t wire_rclone[5] = {
    0x3d, 0x3a, 0x12, 0x2b, 0x00,
};
static const uint8_t wire_twalk[33] = {
    0x3e, 0x02, 0x00, 0x2a, 0x00, 'e', 't', 'c',
};
static const uint8_t wire_rwalk[13] = {
    0x3f, 0x3c, 0x12, 0x2a, 0x00,
    0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
};
static const uint8_t wire_topen[6] = {
    0x40, 0x3d, 0x12, 0x2a, 0x00, 0x03,
};
static const uint8_t wire_ropen[13] = {
    0x41, 0x3e, 0x12, 0x2a, 0x00,
    0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
};
static const uint8_t wire_tcreate[38] = {
    0x42, 0x3f, 0x12, 0x2a, 0x00, 't', 'm', 'p',
    [33] = 0xa4, [34] = 0x81, [37] = 0x01,
};
static const uint8_t wire_rcreate[13] = {
    0x43, 0x40, 0x12, 0x2a, 0x00,
    0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
};
static const uint8_t wire_tread[15] = {
    0x44, 0x03, 0x00, 0x2a, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02,
};
static const uint8_t wire_rread[11] = {
    0x45, 0x04, 0x00, 0x2a, 0x00, 0x03, 0x00, 0x00,
    'a', 'b', 'c',
};
static const uint8_t wire_rread_empty[8] = {
    0x45, 0x05, 0x00, 0x2a, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t wire_twrite[19] = {
    0x46, 0x04, 0x00, 0x2a, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 'a', 'b', 'c',
};
static const uint8_t wire_twrite_empty[16] = {
    0x46, 0x06, 0x00, 0x2a, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00,
};
static const uint8_t wire_rwrite[7] = {
    0x47, 0x44, 0x12, 0x2a, 0x00, 0x03, 0x00,
};
static const uint8_t wire_tclunk[5] = {
    0x48, 0x45, 0x12, 0x2a, 0x00,
};
static const uint8_t wire_rclunk[5] = {
    0x49, 0x46, 0x12, 0x2a, 0x00,
};
static const uint8_t wire_tremove[5] = {
    0x4a, 0x47, 0x12, 0x2a, 0x00,
};
static const uint8_t wire_rremove[5] = {
    0x4b, 0x48, 0x12, 0x2a, 0x00,
};
static const uint8_t wire_tstat[5] = {
    0x4c, 0x49, 0x12, 0x2a, 0x00,
};
static const uint8_t wire_rstat[121] = {
    0x4d, 0x4a, 0x12, 0x2a, 0x00,
    [5] = 'n', [33] = 'u', [61] = 'g',
    [89] = 0x44, [90] = 0x33, [91] = 0x22, [92] = 0x11,
    [93] = 0x88, [94] = 0x77, [95] = 0x66, [96] = 0x55,
    [97] = 0xa4, [98] = 0x81,
    [101] = 0x04, [102] = 0x03, [103] = 0x02, [104] = 0x01,
    [105] = 0x08, [106] = 0x07, [107] = 0x06, [108] = 0x05,
    [109] = 0x0d, [110] = 0x0c, [111] = 0x0b, [112] = 0x0a,
    [117] = 0x34, [118] = 0x12, [119] = 0x78, [120] = 0x56,
};
static const uint8_t wire_twstat[121] = {
    0x4e, 0x4b, 0x12, 0x2a, 0x00,
    [5] = 'n', [33] = 'u', [61] = 'g',
    [89] = 0x44, [90] = 0x33, [91] = 0x22, [92] = 0x11,
    [93] = 0x88, [94] = 0x77, [95] = 0x66, [96] = 0x55,
    [97] = 0xa4, [98] = 0x81,
    [101] = 0x04, [102] = 0x03, [103] = 0x02, [104] = 0x01,
    [105] = 0x08, [106] = 0x07, [107] = 0x06, [108] = 0x05,
    [109] = 0x0d, [110] = 0x0c, [111] = 0x0b, [112] = 0x0a,
    [117] = 0x34, [118] = 0x12, [119] = 0x78, [120] = 0x56,
};
static const uint8_t wire_rwstat[5] = {
    0x4f, 0x4c, 0x12, 0x2a, 0x00,
};
static const uint8_t wire_tclwalk[35] = {
    0x50, 0x4d, 0x12, 0x2a, 0x00, 0x2b, 0x00,
    'e', 't', 'c',
};
static const uint8_t wire_rclwalk[13] = {
    0x51, 0x4e, 0x12, 0x2b, 0x00,
    0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
};
static const uint8_t wire_tsession[11] = {
    0x54, 0x01, 0x00, 1, 2, 3, 4, 5, 6, 7, 8,
};
static const uint8_t wire_rsession[87] = {
    0x55, 0x50, 0x12, 1, 2, 3, 4, 5, 6, 7, 8,
    'a', 'u', 't', 'h',
    [39] = 'd', [40] = 'o', [41] = 'm',
};
static const uint8_t wire_tattach[146] = {
    0x56, 0x51, 0x12, 0x2a, 0x00,
    'g', 'l', 'e', 'n', [33] = '/',
    [61] = 0xa1, [132] = 0xa2, [133] = 0xb1, [145] = 0xb2,
};
static const uint8_t wire_rattach[26] = {
    0x57, 0x52, 0x12, 0x2a, 0x00,
    0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
    0xb1, [25] = 0xb2,
};

static const WireCase wire_cases[] = {
    WIRE_CASE(tnop), WIRE_CASE(rnop), WIRE_CASE(rerror),
    WIRE_CASE(tflush), WIRE_CASE(rflush),
    WIRE_CASE(tclone), WIRE_CASE(rclone),
    WIRE_CASE(twalk), WIRE_CASE(rwalk),
    WIRE_CASE(topen), WIRE_CASE(ropen),
    WIRE_CASE(tcreate), WIRE_CASE(rcreate),
    WIRE_CASE(tread), WIRE_CASE(rread),
    WIRE_CASE(twrite), WIRE_CASE(rwrite),
    WIRE_CASE(tclunk), WIRE_CASE(rclunk),
    WIRE_CASE(tremove), WIRE_CASE(rremove),
    WIRE_CASE(tstat), WIRE_CASE(rstat),
    WIRE_CASE(twstat), WIRE_CASE(rwstat),
    WIRE_CASE(tclwalk), WIRE_CASE(rclwalk),
    WIRE_CASE(tsession), WIRE_CASE(rsession),
    WIRE_CASE(tattach), WIRE_CASE(rattach),
};

typedef struct FrameCollector {
    GPtrArray *frames;
} FrameCollector;

static int collect_frame(const uint8_t *frame, size_t length,
                         void *opaque, Error **errp)
{
    FrameCollector *collector = opaque;

    g_ptr_array_add(collector->frames, g_bytes_new(frame, length));
    return 0;
}

static void assert_frame(FrameCollector *collector, size_t index,
                         const uint8_t *expected, size_t expected_length)
{
    GBytes *bytes = g_ptr_array_index(collector->frames, index);
    gsize actual_length;
    const void *actual = g_bytes_get_data(bytes, &actual_length);

    g_assert_cmpmem(actual, actual_length, expected, expected_length);
}

static void test_vectors_round_trip(void)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(wire_cases); i++) {
        const WireCase *test = &wire_cases[i];
        Error *err = NULL;
        Plan9P1Fcall fcall;
        uint8_t encoded[PLAN9P1_MAX_FRAME];
        ssize_t length;

        g_assert_cmpint(plan9p1_decode(test->wire, test->length,
                                      &fcall, &err), ==, 0);
        g_assert_null(err);
        length = plan9p1_encode(encoded, sizeof(encoded), &fcall, &err);
        g_assert_cmpint(length, ==, test->length);
        g_assert_null(err);
        g_assert_cmpmem(encoded, length, test->wire, test->length);
    }
}

static void test_golden_fields(void)
{
    Error *err = NULL;
    Plan9P1Fcall fcall;

    g_assert_cmpint(plan9p1_decode(wire_twalk, sizeof(wire_twalk),
                                  &fcall, &err), ==, 0);
    g_assert_cmpuint(fcall.type, ==, PLAN9P1_TWALK);
    g_assert_cmpuint(fcall.tag, ==, 2);
    g_assert_cmpuint(fcall.fid, ==, 0x2a);
    g_assert_cmpmem(fcall.name, 3, "etc", 3);

    g_assert_cmpint(plan9p1_decode(wire_tcreate, sizeof(wire_tcreate),
                                  &fcall, &err), ==, 0);
    g_assert_cmpuint(fcall.perm, ==, 0x81a4);
    g_assert_cmpuint(fcall.mode, ==, 1);

    g_assert_cmpint(plan9p1_decode(wire_tread, sizeof(wire_tread),
                                  &fcall, &err), ==, 0);
    g_assert_cmpuint(fcall.offset, ==, 0x100);
    g_assert_cmpuint(fcall.count, ==, 512);

    g_assert_cmpint(plan9p1_decode(wire_twrite, sizeof(wire_twrite),
                                  &fcall, &err), ==, 0);
    g_assert_cmpuint(fcall.count, ==, 3);
    g_assert_cmpmem(fcall.data, fcall.count, "abc", 3);

    g_assert_cmpint(plan9p1_decode(wire_rstat, sizeof(wire_rstat),
                                  &fcall, &err), ==, 0);
    g_assert_cmpuint(fcall.dir.qid.path, ==, 0x11223344);
    g_assert_cmpuint(fcall.dir.qid.vers, ==, 0x55667788);
    g_assert_cmpuint(fcall.dir.mode, ==, 0x81a4);
    g_assert_cmpuint(fcall.dir.atime, ==, 0x01020304);
    g_assert_cmpuint(fcall.dir.mtime, ==, 0x05060708);
    g_assert_cmpuint(fcall.dir.length, ==, 0x0a0b0c0d);
    g_assert_cmpuint(fcall.dir.type, ==, 0x1234);
    g_assert_cmpuint(fcall.dir.dev, ==, 0x5678);

    g_assert_cmpint(plan9p1_decode(wire_tattach, sizeof(wire_tattach),
                                  &fcall, &err), ==, 0);
    g_assert_cmpmem(fcall.uname, 4, "glen", 4);
    g_assert_cmpuint(fcall.ticket[0], ==, 0xa1);
    g_assert_cmpuint(fcall.ticket[PLAN9P1_TICKETLEN - 1], ==, 0xa2);
    g_assert_cmpuint(fcall.auth[0], ==, 0xb1);
    g_assert_cmpuint(fcall.auth[PLAN9P1_AUTHLEN - 1], ==, 0xb2);
}

static void test_fragment_splits(void)
{
    const WireCase *tests[] = { &wire_cases[13], &wire_cases[15] };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(tests); i++) {
        const WireCase *test = tests[i];
        size_t split;

        for (split = 0; split <= test->length; split++) {
            g_autoptr(GPtrArray) frames =
                g_ptr_array_new_with_free_func((GDestroyNotify)g_bytes_unref);
            FrameCollector collector = { .frames = frames };
            Error *err = NULL;
            Plan9P1Stream stream;

            plan9p1_stream_init(&stream);
            g_assert_cmpint(plan9p1_stream_feed(&stream, test->wire, split,
                                               collect_frame, &collector,
                                               &err), ==, 0);
            g_assert_cmpint(plan9p1_stream_feed(&stream, test->wire + split,
                                               test->length - split,
                                               collect_frame, &collector,
                                               &err), ==, 0);
            g_assert_null(err);
            g_assert_cmpuint(frames->len, ==, 1);
            assert_frame(&collector, 0, test->wire, test->length);
            plan9p1_stream_reset(&stream);
        }
    }
}

static void test_fragment_bytewise(void)
{
    g_autoptr(GPtrArray) frames =
        g_ptr_array_new_with_free_func((GDestroyNotify)g_bytes_unref);
    FrameCollector collector = { .frames = frames };
    Error *err = NULL;
    Plan9P1Stream stream;
    size_t i;

    plan9p1_stream_init(&stream);
    for (i = 0; i < sizeof(wire_rread); i++) {
        g_assert_cmpint(plan9p1_stream_feed(&stream, wire_rread + i, 1,
                                           collect_frame, &collector,
                                           &err), ==, 0);
    }
    g_assert_null(err);
    g_assert_cmpuint(frames->len, ==, 1);
    assert_frame(&collector, 0, wire_rread, sizeof(wire_rread));
    plan9p1_stream_reset(&stream);
}

static void test_coalesced_and_trailing(void)
{
    static const uint8_t combined[] = {
        0x32, 0x34, 0x12,
        0x38, 0x37, 0x12, 0xef, 0xbe,
        0x3c, 0x39, 0x12, 0x2a,
    };
    static const uint8_t clone_tail[] = { 0x00, 0x2b, 0x00 };
    g_autoptr(GPtrArray) frames =
        g_ptr_array_new_with_free_func((GDestroyNotify)g_bytes_unref);
    FrameCollector collector = { .frames = frames };
    Error *err = NULL;
    Plan9P1Stream stream;

    plan9p1_stream_init(&stream);
    g_assert_cmpint(plan9p1_stream_feed(&stream, combined, sizeof(combined),
                                       collect_frame, &collector, &err), ==, 0);
    g_assert_cmpuint(frames->len, ==, 2);
    assert_frame(&collector, 0, wire_tnop, sizeof(wire_tnop));
    assert_frame(&collector, 1, wire_tflush, sizeof(wire_tflush));

    g_assert_cmpint(plan9p1_stream_feed(&stream, clone_tail,
                                       sizeof(clone_tail), collect_frame,
                                       &collector, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(frames->len, ==, 3);
    assert_frame(&collector, 2, wire_tclone, sizeof(wire_tclone));
    plan9p1_stream_reset(&stream);
}

static void test_payload_boundaries(void)
{
    const size_t twrite_length = 16 + PLAN9P1_MAX_DATA;
    const size_t rread_length = 8 + PLAN9P1_MAX_DATA;
    g_autofree uint8_t *twrite = g_malloc0(twrite_length);
    g_autofree uint8_t *rread = g_malloc0(rread_length);
    g_autoptr(GPtrArray) frames =
        g_ptr_array_new_with_free_func((GDestroyNotify)g_bytes_unref);
    FrameCollector collector = { .frames = frames };
    Error *err = NULL;
    Plan9P1Fcall fcall;
    Plan9P1Stream stream;
    uint16_t tag;

    twrite[0] = 0x46;
    twrite[1] = 0x21;
    twrite[2] = 0x43;
    twrite[3] = 0x2a;
    twrite[13] = 0x00;
    twrite[14] = 0x20;
    twrite[16] = 0x5a;
    twrite[twrite_length - 1] = 0xa5;
    rread[0] = 0x45;
    rread[1] = 0x22;
    rread[2] = 0x43;
    rread[3] = 0x2a;
    rread[5] = 0x00;
    rread[6] = 0x20;
    rread[8] = 0x5a;
    rread[rread_length - 1] = 0xa5;

    g_assert_cmpint(plan9p1_decode(twrite, twrite_length, &fcall, &err), ==, 0);
    g_assert_cmpuint(fcall.count, ==, PLAN9P1_MAX_DATA);
    g_assert_cmpuint(fcall.data[0], ==, 0x5a);
    g_assert_cmpuint(fcall.data[PLAN9P1_MAX_DATA - 1], ==, 0xa5);
    g_assert_cmpint(plan9p1_decode(rread, rread_length, &fcall, &err), ==, 0);

    plan9p1_stream_init(&stream);
    g_assert_cmpint(plan9p1_stream_feed(&stream, twrite, twrite_length,
                                       collect_frame, &collector, &err), ==, 0);
    g_assert_cmpint(plan9p1_stream_feed(&stream, rread, rread_length,
                                       collect_frame, &collector, &err), ==, 0);
    g_assert_cmpuint(frames->len, ==, 2);
    assert_frame(&collector, 0, twrite, twrite_length);
    assert_frame(&collector, 1, rread, rread_length);
    plan9p1_stream_reset(&stream);

    twrite[13] = 0x01;
    twrite[14] = 0x20;
    g_assert_cmpint(plan9p1_stream_feed(&stream, twrite, 15, collect_frame,
                                       &collector, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_true(plan9p1_stream_error_tag(&stream, &tag));
    g_assert_cmpuint(tag, ==, 0x4321);
    plan9p1_stream_reset(&stream);
    error_free(g_steal_pointer(&err));

    rread[5] = 0x01;
    rread[6] = 0x20;
    g_assert_cmpint(plan9p1_stream_feed(&stream, rread, 7, collect_frame,
                                       &collector, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_true(plan9p1_stream_error_tag(&stream, &tag));
    g_assert_cmpuint(tag, ==, 0x4322);
    plan9p1_stream_reset(&stream);
    error_free(g_steal_pointer(&err));

    fcall.type = PLAN9P1_TWRITE;
    fcall.count = PLAN9P1_MAX_DATA + 1;
    g_assert_cmpint(plan9p1_encode(twrite, twrite_length, &fcall, &err),
                    ==, -1);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_empty_payloads(void)
{
    typedef struct EmptyWireCase {
        const uint8_t *wire;
        size_t length;
        Plan9P1Type type;
    } EmptyWireCase;
    const EmptyWireCase tests[] = {
        { wire_rread_empty, sizeof(wire_rread_empty), PLAN9P1_RREAD },
        { wire_twrite_empty, sizeof(wire_twrite_empty), PLAN9P1_TWRITE },
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(tests); i++) {
        const EmptyWireCase *test = &tests[i];
        Error *err = NULL;
        Plan9P1Fcall fcall;
        uint8_t encoded[sizeof(wire_twrite_empty)];

        g_assert_cmpint(plan9p1_decode(test->wire, test->length,
                                      &fcall, &err), ==, 0);
        g_assert_null(err);
        g_assert_cmpuint(fcall.type, ==, test->type);
        g_assert_cmpuint(fcall.count, ==, 0);

        fcall.data = NULL;
        g_assert_cmpint(plan9p1_encode(encoded, sizeof(encoded),
                                      &fcall, &err), ==, test->length);
        g_assert_null(err);
        g_assert_cmpmem(encoded, test->length, test->wire, test->length);
    }
}

static void assert_decode_fails(const uint8_t *wire, size_t length)
{
    Error *err = NULL;
    Plan9P1Fcall fcall;

    g_assert_cmpint(plan9p1_decode(wire, length, &fcall, &err), ==, -1);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_malformed_frames(void)
{
    uint8_t bad_write[sizeof(wire_twrite) + 1];
    uint8_t bad_read[sizeof(wire_rread) + 1];
    uint8_t bad_dir[sizeof(wire_rstat)];
    uint8_t bad_tread[sizeof(wire_tread)];
    uint8_t bad_rwrite[sizeof(wire_rwrite)];
    Error *err = NULL;
    Plan9P1Fcall fcall = { 0 };
    uint8_t encoded[PLAN9P1_MAX_FRAME];

    assert_decode_fails(wire_tnop, sizeof(wire_tnop) - 1);
    memcpy(bad_write, wire_twrite, sizeof(wire_twrite));
    bad_write[sizeof(wire_twrite)] = 0xff;
    assert_decode_fails(bad_write, sizeof(bad_write));
    assert_decode_fails(wire_twrite, sizeof(wire_twrite) - 1);
    memcpy(bad_read, wire_rread, sizeof(wire_rread));
    bad_read[sizeof(wire_rread)] = 0xff;
    assert_decode_fails(bad_read, sizeof(bad_read));
    assert_decode_fails(wire_rread, sizeof(wire_rread) - 1);
    memcpy(bad_dir, wire_rstat, sizeof(bad_dir));
    bad_dir[113] = 1;
    assert_decode_fails(bad_dir, sizeof(bad_dir));
    memcpy(bad_tread, wire_tread, sizeof(bad_tread));
    bad_tread[13] = 0x01;
    bad_tread[14] = 0x20;
    assert_decode_fails(bad_tread, sizeof(bad_tread));
    memcpy(bad_rwrite, wire_rwrite, sizeof(bad_rwrite));
    bad_rwrite[5] = 0x01;
    bad_rwrite[6] = 0x20;
    assert_decode_fails(bad_rwrite, sizeof(bad_rwrite));

    fcall.type = PLAN9P1_RSTAT;
    fcall.dir.length = UINT64_C(1) << 32;
    g_assert_cmpint(plan9p1_encode(encoded, sizeof(encoded), &fcall, &err),
                    ==, -1);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_unknown_fatal_and_reset(void)
{
    static const uint8_t unknown[] = { 0xff, 0x34, 0x12 };
    static const uint8_t overlong[] = { 0x32, 0x34, 0x12, 0xff };
    g_autoptr(GPtrArray) frames =
        g_ptr_array_new_with_free_func((GDestroyNotify)g_bytes_unref);
    FrameCollector collector = { .frames = frames };
    Error *err = NULL;
    Plan9P1Fcall fcall;
    Plan9P1Stream stream;
    uint8_t encoded[4];

    assert_decode_fails(unknown, sizeof(unknown));
    assert_decode_fails(overlong, sizeof(overlong));

    plan9p1_stream_init(&stream);
    g_assert_cmpint(plan9p1_stream_feed(&stream, unknown, sizeof(unknown),
                                       collect_frame, &collector, &err),
                    ==, -1);
    g_assert_nonnull(err);
    error_free(g_steal_pointer(&err));
    g_assert_cmpint(plan9p1_stream_feed(&stream, wire_tnop, sizeof(wire_tnop),
                                       collect_frame, &collector, &err),
                    ==, -1);
    g_assert_nonnull(err);
    g_assert_cmpuint(frames->len, ==, 0);
    error_free(g_steal_pointer(&err));

    plan9p1_stream_reset(&stream);
    g_assert_cmpint(plan9p1_stream_feed(&stream, wire_tnop, sizeof(wire_tnop),
                                       collect_frame, &collector, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(frames->len, ==, 1);
    plan9p1_stream_reset(&stream);

    memset(&fcall, 0, sizeof(fcall));
    fcall.type = 0xff;
    g_assert_cmpint(plan9p1_encode(encoded, sizeof(encoded), &fcall, &err),
                    ==, -1);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_wrapped_message_type(void)
{
    const Plan9P1Type bad_types[] = {
        PLAN9P1_TNOP + 256,
        PLAN9P1_RREAD + 256,
        (Plan9P1Type)-1,
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(bad_types); i++) {
        Error *err = NULL;
        Plan9P1Fcall fcall = { .type = bad_types[i] };
        uint8_t encoded[PLAN9P1_MAX_FRAME];

        g_assert_cmpint(plan9p1_encode(encoded, sizeof(encoded),
                                      &fcall, &err), ==, -1);
        g_assert_nonnull(err);
        error_free(err);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/plan9p1/vectors/round-trip", test_vectors_round_trip);
    g_test_add_func("/plan9p1/vectors/fields", test_golden_fields);
    g_test_add_func("/plan9p1/stream/splits", test_fragment_splits);
    g_test_add_func("/plan9p1/stream/bytewise", test_fragment_bytewise);
    g_test_add_func("/plan9p1/stream/coalesced", test_coalesced_and_trailing);
    g_test_add_func("/plan9p1/limits/payload", test_payload_boundaries);
    g_test_add_func("/plan9p1/limits/empty-payload", test_empty_payloads);
    g_test_add_func("/plan9p1/errors/malformed", test_malformed_frames);
    g_test_add_func("/plan9p1/errors/fatal-reset",
                    test_unknown_fatal_and_reset);
    g_test_add_func("/plan9p1/errors/wrapped-type",
                    test_wrapped_message_type);
    return g_test_run();
}
