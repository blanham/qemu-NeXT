/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Golden values were independently checked from the preserved Second Edition
 * sources in the Plan 9 rootfs: sys/src/libauth/{passtokey,convTR2M,
 * convM2TR,convT2M,convM2T,convA2M}.c and sys/man/2/encrypt.  The historical
 * DES block values
 * were cross-checked with OpenSSL DES/ECB after applying the documented
 * packed 56-bit-to-odd-parity 64-bit conversion; no real credential occurs
 * in these fixtures.
 */
#include "qemu/osdep.h"
#include "hw/9pfs/plan9-auth.h"
#include "qapi/error.h"

static void test_sizes(void)
{
    g_assert_cmpuint(PLAN9_AUTH_NAMELEN, ==, 28);
    g_assert_cmpuint(PLAN9_AUTH_DOMLEN, ==, 48);
    g_assert_cmpuint(PLAN9_AUTH_TICKET_REQUEST_LEN, ==, 141);
    g_assert_cmpuint(PLAN9_AUTH_TICKET_LEN, ==, 72);
    g_assert_cmpuint(PLAN9_AUTH_AUTHENTICATOR_LEN, ==, 13);
}

static void test_des56to64(void)
{
    static const uint8_t packed[PLAN9_AUTH_DES_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd,
    };
    static const uint8_t expected[8] = {
        0x01, 0x91, 0xd0, 0xad, 0x79, 0x4c, 0xae, 0x9b,
    };
    uint8_t expanded[8];

    plan9_auth_des56to64(packed, expanded);
    g_assert_cmpmem(expanded, sizeof(expanded), expected, sizeof(expected));
}

static void test_passtokey(void)
{
    static const uint8_t short_expected[PLAN9_AUTH_DES_KEY_LEN] = {
        0x61, 0xf1, 0x18, 0x00, 0x02, 0x81, 0x40,
    };
    static const uint8_t long_expected[PLAN9_AUTH_DES_KEY_LEN] = {
        0x26, 0x1b, 0x62, 0xdb, 0x16, 0x3c, 0x49,
    };
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];
    static const char password_27[] = "012345678901234567890123456";
    static const char password_28[] = "0123456789012345678901234567";
    static const char password_utf8_27[] =
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9" "a";
    static const char password_utf8_28[] =
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9";
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_passtokey(key, "", &err), ==, 0);
    g_assert_null(err);

    g_assert_cmpint(plan9_auth_passtokey(key, "abc", &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(key, sizeof(key), short_expected, sizeof(short_expected));

    g_assert_cmpint(plan9_auth_passtokey(key, "123456789", &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(key, sizeof(key), long_expected, sizeof(long_expected));

    g_assert_cmpuint(strlen(password_27), ==, PLAN9_AUTH_NAMELEN - 1);
    g_assert_cmpint(plan9_auth_passtokey(key, password_27, &err), ==, 0);
    g_assert_null(err);

    g_assert_cmpuint(strlen(password_28), ==, PLAN9_AUTH_NAMELEN);
    g_assert_cmpint(plan9_auth_passtokey(key, password_28, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    for (size_t i = 0; i < sizeof(key); i++) {
        g_assert_cmpuint(key[i], ==, 0);
    }

    g_assert_cmpuint(strlen(password_utf8_27), ==, PLAN9_AUTH_NAMELEN - 1);
    g_assert_cmpint(plan9_auth_passtokey(key, password_utf8_27, &err), ==, 0);
    g_assert_null(err);

    g_assert_cmpuint(strlen(password_utf8_28), ==, PLAN9_AUTH_NAMELEN);
    g_assert_cmpint(plan9_auth_passtokey(key, password_utf8_28, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    for (size_t i = 0; i < sizeof(key); i++) {
        g_assert_cmpuint(key[i], ==, 0);
    }
}

static void test_encrypt_vectors(void)
{
    static const uint8_t key[PLAN9_AUTH_DES_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd,
    };
    static const uint8_t plain8[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    static const uint8_t cipher8[] = {
        0x76, 0x3e, 0x78, 0xcf, 0xb5, 0x05, 0xed, 0xdd,
    };
    static const uint8_t cipher9[] = {
        0x76, 0x07, 0xa2, 0xbf, 0x54, 0x4c, 0x97, 0x1f, 0x1d,
    };
    static const uint8_t cipher14[] = {
        0x76, 0x3e, 0x78, 0xcf, 0xb5, 0x05, 0xd3,
        0x97, 0xa5, 0x76, 0xbe, 0x16, 0x81, 0xcd,
    };
    static const uint8_t cipher15[] = {
        0x76, 0x3e, 0x78, 0xcf, 0xb5, 0x05, 0xed, 0xd8,
        0x06, 0xdc, 0x3e, 0xb7, 0x81, 0xa4, 0x5c,
    };
    uint8_t buffer[sizeof(cipher15)];
    uint8_t wrong_key[PLAN9_AUTH_DES_KEY_LEN];
    Error *err = NULL;

    memcpy(buffer, plain8, sizeof(plain8));
    g_assert_cmpint(plan9_auth_encrypt(key, buffer, sizeof(plain8), &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(buffer, sizeof(plain8), cipher8, sizeof(cipher8));
    g_assert_cmpint(plan9_auth_decrypt(key, buffer, sizeof(plain8), &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(buffer, sizeof(plain8), plain8, sizeof(plain8));

    for (size_t len = 9; len <= sizeof(buffer); len++) {
        for (size_t i = 0; i < len; i++) {
            buffer[i] = i;
        }
        g_assert_cmpint(plan9_auth_encrypt(key, buffer, len, &err), ==, 0);
        g_assert_null(err);
        if (len == 9) {
            g_assert_cmpmem(buffer, len, cipher9, sizeof(cipher9));
        } else if (len == 14) {
            g_assert_cmpmem(buffer, len, cipher14, sizeof(cipher14));
        } else if (len == 15) {
            g_assert_cmpmem(buffer, len, cipher15, sizeof(cipher15));
        }
        g_assert_cmpint(plan9_auth_decrypt(key, buffer, len, &err), ==, 0);
        g_assert_null(err);
        for (size_t i = 0; i < len; i++) {
            g_assert_cmpuint(buffer[i], ==, i);
        }
    }

    g_assert_cmpint(plan9_auth_encrypt(key, buffer, 7, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;

    memcpy(buffer, cipher8, sizeof(cipher8));
    memcpy(wrong_key, key, sizeof(wrong_key));
    wrong_key[0] ^= 0x80;
    g_assert_cmpint(plan9_auth_decrypt(wrong_key, buffer, sizeof(cipher8),
                                       &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpint(memcmp(buffer, plain8, sizeof(plain8)), !=, 0);
}

static void test_ticket_request_codec(void)
{
    Plan9AuthTicketRequest request = {
        .type = PLAN9_AUTH_TREQ,
        .authid = "p9fs",
        .authdom = "nextlab",
        .challenge = { 1, 2, 3, 4, 5, 6, 7, 8 },
        .hostid = "tor",
        .uid = "tor",
    };
    Plan9AuthTicketRequest decoded;
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire, &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(wire[0], ==, PLAN9_AUTH_TREQ);
    g_assert_cmpmem(wire + 1, 4, "p9fs", 4);
    g_assert_cmpmem(wire + 29, 7, "nextlab", 7);
    g_assert_cmpmem(wire + 77, 8, request.challenge, 8);
    g_assert_cmpmem(wire + 85, 3, "tor", 3);
    g_assert_cmpmem(wire + 113, 3, "tor", 3);
    g_assert_cmpint(plan9_auth_ticket_request_decode(wire, sizeof(wire),
                                                      &decoded, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(decoded.type, ==, request.type);
    g_assert_cmpstr(decoded.authid, ==, request.authid);
    g_assert_cmpstr(decoded.authdom, ==, request.authdom);
    g_assert_cmpmem(decoded.challenge, sizeof(decoded.challenge),
                    request.challenge, sizeof(request.challenge));

    request.uid[0] = 'x';
    memset(request.uid + 1, 'x', PLAN9_AUTH_NAMELEN - 1);
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_ticket_and_authenticator_codecs(void)
{
    Plan9AuthTicket ticket = {
        .num = PLAN9_AUTH_TC,
        .challenge = { 8, 7, 6, 5, 4, 3, 2, 1 },
        .cuid = "tor",
        .suid = "p9fs",
        .key = { 1, 2, 3, 4, 5, 6, 7 },
    };
    Plan9AuthTicket ticket_decoded;
    Plan9AuthAuthenticator auth = {
        .num = PLAN9_AUTH_AC,
        .challenge = { 1, 3, 3, 7, 0, 0, 0, 1 },
        .id = 0x78563412,
    };
    Plan9AuthAuthenticator auth_decoded;
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN];
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN];
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_ticket_encode(&ticket, ticket_wire, &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(ticket_wire[0], ==, PLAN9_AUTH_TC);
    g_assert_cmpmem(ticket_wire + 1, 8, ticket.challenge, 8);
    g_assert_cmpmem(ticket_wire + 9, 3, "tor", 3);
    g_assert_cmpmem(ticket_wire + 37, 4, "p9fs", 4);
    g_assert_cmpmem(ticket_wire + 65, 7, ticket.key, 7);
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire, sizeof(ticket_wire),
                                              &ticket_decoded, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(ticket_decoded.cuid, ==, ticket.cuid);
    g_assert_cmpstr(ticket_decoded.suid, ==, ticket.suid);
    g_assert_cmpmem(ticket_decoded.key, sizeof(ticket_decoded.key),
                    ticket.key, sizeof(ticket.key));

    g_assert_cmpint(plan9_auth_authenticator_encode(&auth, auth_wire, &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(auth_wire[0], ==, PLAN9_AUTH_AC);
    g_assert_cmpmem(auth_wire + 1, 8, auth.challenge, 8);
    g_assert_cmpmem(auth_wire + 9, 4, "\x12\x34\x56\x78", 4);
    g_assert_cmpint(plan9_auth_authenticator_decode(auth_wire,
                                                     sizeof(auth_wire),
                                                     &auth_decoded,
                                                     &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(auth_decoded.id, ==, auth.id);

    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                              sizeof(ticket_wire) - 1,
                                              &ticket_decoded, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_rejections_wipe_ticket_output(void)
{
    Plan9AuthTicket ticket = {
        .num = PLAN9_AUTH_TC,
        .challenge = { 8, 7, 6, 5, 4, 3, 2, 1 },
        .suid = "p9fs",
        .key = { 1, 2, 3, 4, 5, 6, 7 },
    };
    uint8_t wire[PLAN9_AUTH_TICKET_LEN];
    Error *err = NULL;

    memset(ticket.cuid, 'x', PLAN9_AUTH_NAMELEN);
    memset(wire, 0xa5, sizeof(wire));
    g_assert_cmpint(plan9_auth_ticket_encode(&ticket, wire, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    for (size_t i = 0; i < sizeof(wire); i++) {
        g_assert_cmpuint(wire[i], ==, 0);
    }
}

static void test_invalid_arguments(void)
{
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_passtokey(NULL, "abc", &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(plan9_auth_ticket_decode(NULL, PLAN9_AUTH_TICKET_LEN,
                                             NULL, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void assert_zeroed(const void *data, size_t len)
{
    const uint8_t *bytes = data;

    for (size_t i = 0; i < len; i++) {
        g_assert_cmpuint(bytes[i], ==, 0);
    }
}

static void test_public_clear(void)
{
    uint8_t plaintext[PLAN9_AUTH_TICKET_LEN];
    Plan9AuthTicket ticket;

    memset(plaintext, 0xa5, sizeof(plaintext));
    plan9_auth_clear(plaintext, sizeof(plaintext));
    assert_zeroed(plaintext, sizeof(plaintext));

    memset(&ticket, 0xa5, sizeof(ticket));
    plan9_auth_ticket_clear(&ticket);
    assert_zeroed(&ticket, sizeof(ticket));
}

static void test_encode_failure_wipes_output(void)
{
    Plan9AuthTicketRequest request = {
        .authid = "p9fs",
        .authdom = "nextlab",
        .hostid = "tor",
        .uid = "tor",
    };
    Plan9AuthTicket ticket = {
        .cuid = "tor",
        .suid = "p9fs",
        .key = { 1, 2, 3, 4, 5, 6, 7 },
    };
    Plan9AuthAuthenticator auth = { 0 };
    uint8_t request_wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN];
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN];
    Error *err = NULL;

    memset(request_wire, 0xa5, sizeof(request_wire));
    g_assert_cmpint(plan9_auth_ticket_request_encode(NULL, request_wire, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(request_wire, sizeof(request_wire));
    err = NULL;

    memset(request.authid, 'x', PLAN9_AUTH_NAMELEN);
    memset(request_wire, 0xa5, sizeof(request_wire));
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, request_wire,
                                                      &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(request_wire, sizeof(request_wire));
    err = NULL;

    memset(ticket_wire, 0xa5, sizeof(ticket_wire));
    g_assert_cmpint(plan9_auth_ticket_encode(NULL, ticket_wire, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(ticket_wire, sizeof(ticket_wire));
    err = NULL;

    memset(auth_wire, 0xa5, sizeof(auth_wire));
    g_assert_cmpint(plan9_auth_authenticator_encode(NULL, auth_wire, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(auth_wire, sizeof(auth_wire));
    err = NULL;

    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, NULL, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(plan9_auth_ticket_encode(&ticket, NULL, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(plan9_auth_authenticator_encode(&auth, NULL, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_decode_failure_wipes_destination(void)
{
    uint8_t request_wire[PLAN9_AUTH_TICKET_REQUEST_LEN] = { 0 };
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN] = { 0 };
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN] = { 0 };
    Plan9AuthTicketRequest request;
    Plan9AuthTicket ticket;
    Plan9AuthAuthenticator auth;
    Error *err = NULL;

    memset(&request, 0xa5, sizeof(request));
    g_assert_cmpint(plan9_auth_ticket_request_decode(NULL, sizeof(request_wire),
                                                      &request, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&request, sizeof(request));
    err = NULL;

    memset(&request, 0xa5, sizeof(request));
    g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                      sizeof(request_wire) - 1,
                                                      &request, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&request, sizeof(request));
    err = NULL;

    memset(&request, 0xa5, sizeof(request));
    g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                      sizeof(request_wire) + 1,
                                                      &request, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&request, sizeof(request));
    err = NULL;

    memset(&ticket, 0xa5, sizeof(ticket));
    g_assert_cmpint(plan9_auth_ticket_decode(NULL, sizeof(ticket_wire),
                                              &ticket, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&ticket, sizeof(ticket));
    err = NULL;

    memset(&ticket, 0xa5, sizeof(ticket));
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                              sizeof(ticket_wire) - 1,
                                              &ticket, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&ticket, sizeof(ticket));
    err = NULL;

    memset(&ticket, 0xa5, sizeof(ticket));
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                              sizeof(ticket_wire) + 1,
                                              &ticket, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&ticket, sizeof(ticket));
    err = NULL;

    memset(&auth, 0xa5, sizeof(auth));
    g_assert_cmpint(plan9_auth_authenticator_decode(NULL, sizeof(auth_wire),
                                                     &auth, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&auth, sizeof(auth));
    err = NULL;

    memset(&auth, 0xa5, sizeof(auth));
    g_assert_cmpint(plan9_auth_authenticator_decode(auth_wire,
                                                     sizeof(auth_wire) - 1,
                                                     &auth, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&auth, sizeof(auth));
    err = NULL;

    memset(&auth, 0xa5, sizeof(auth));
    g_assert_cmpint(plan9_auth_authenticator_decode(auth_wire,
                                                     sizeof(auth_wire) + 1,
                                                     &auth, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&auth, sizeof(auth));
}

static void test_decode_fixed_string_compatibility(void)
{
    uint8_t request_wire[PLAN9_AUTH_TICKET_REQUEST_LEN] = { 0 };
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN] = { 0 };
    Plan9AuthTicketRequest request;
    Plan9AuthTicket ticket;
    Error *err = NULL;
    const size_t request_fields[] = { 1, 29, 85, 113 };
    const size_t request_sizes[] = {
        PLAN9_AUTH_NAMELEN, PLAN9_AUTH_DOMLEN,
        PLAN9_AUTH_NAMELEN, PLAN9_AUTH_NAMELEN,
    };
    const size_t ticket_fields[] = { 9, 37 };

    for (size_t i = 0; i < G_N_ELEMENTS(request_fields); i++) {
        memset(request_wire, 0, sizeof(request_wire));
        memset(request_wire + request_fields[i], 'x', request_sizes[i]);
        char *const dest[] = {
            request.authid, request.authdom, request.hostid, request.uid,
        };

        memset(&request, 0xa5, sizeof(request));
        g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                          sizeof(request_wire),
                                                          &request, &err),
                        ==, 0);
        g_assert_null(err);
        g_assert_cmpuint(dest[i][request_sizes[i] - 2], ==, 'x');
        g_assert_cmpuint(dest[i][request_sizes[i] - 1], ==, 0);
        g_assert_cmpuint(dest[i][request_sizes[i]], ==, 0);
    }

    for (size_t i = 0; i < G_N_ELEMENTS(ticket_fields); i++) {
        memset(ticket_wire, 0, sizeof(ticket_wire));
        memset(ticket_wire + ticket_fields[i], 'x', PLAN9_AUTH_NAMELEN);
        char *const dest[] = { ticket.cuid, ticket.suid };

        memset(&ticket, 0xa5, sizeof(ticket));
        g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                                  sizeof(ticket_wire),
                                                  &ticket, &err), ==, 0);
        g_assert_null(err);
        g_assert_cmpuint(dest[i][PLAN9_AUTH_NAMELEN - 2], ==, 'x');
        g_assert_cmpuint(dest[i][PLAN9_AUTH_NAMELEN - 1], ==, 0);
        g_assert_cmpuint(dest[i][PLAN9_AUTH_NAMELEN], ==, 0);
    }

    memset(request_wire, 0, sizeof(request_wire));
    request_wire[1] = 'a';
    request_wire[2] = 0;
    request_wire[3] = 0xa5;
    request_wire[29] = 'd';
    request_wire[30] = 0;
    request_wire[31] = 0xa5;
    request_wire[85] = 'h';
    request_wire[86] = 0;
    request_wire[87] = 0xa5;
    request_wire[113] = 'u';
    request_wire[114] = 0;
    request_wire[115] = 0xa5;
    g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                      sizeof(request_wire),
                                                      &request, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(request.authid, ==, "a");
    g_assert_cmpstr(request.authdom, ==, "d");
    g_assert_cmpstr(request.hostid, ==, "h");
    g_assert_cmpstr(request.uid, ==, "u");
    g_assert_cmpuint((uint8_t)request.authid[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)request.authdom[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)request.hostid[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)request.uid[2], ==, 0xa5);

    memset(ticket_wire, 0, sizeof(ticket_wire));
    ticket_wire[9] = 'c';
    ticket_wire[10] = 0;
    ticket_wire[11] = 0xa5;
    ticket_wire[37] = 's';
    ticket_wire[38] = 0;
    ticket_wire[39] = 0xa5;
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire, sizeof(ticket_wire),
                                              &ticket, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(ticket.cuid, ==, "c");
    g_assert_cmpstr(ticket.suid, ==, "s");
    g_assert_cmpuint((uint8_t)ticket.cuid[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)ticket.suid[2], ==, 0xa5);
    plan9_auth_ticket_clear(&ticket);
    assert_zeroed(&ticket, sizeof(ticket));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/plan9-auth/sizes", test_sizes);
    g_test_add_func("/plan9-auth/des56to64", test_des56to64);
    g_test_add_func("/plan9-auth/passtokey", test_passtokey);
    g_test_add_func("/plan9-auth/encrypt", test_encrypt_vectors);
    g_test_add_func("/plan9-auth/ticket-request", test_ticket_request_codec);
    g_test_add_func("/plan9-auth/ticket-authenticator",
                    test_ticket_and_authenticator_codecs);
    g_test_add_func("/plan9-auth/rejections-wipe-ticket-output",
                    test_rejections_wipe_ticket_output);
    g_test_add_func("/plan9-auth/invalid-arguments", test_invalid_arguments);
    g_test_add_func("/plan9-auth/public-clear", test_public_clear);
    g_test_add_func("/plan9-auth/encode-failure-wipes-output",
                    test_encode_failure_wipes_output);
    g_test_add_func("/plan9-auth/decode-failure-wipes-destination",
                    test_decode_failure_wipes_destination);
    g_test_add_func("/plan9-auth/decode-fixed-string-compatibility",
                    test_decode_fixed_string_compatibility);
    return g_test_run();
}
