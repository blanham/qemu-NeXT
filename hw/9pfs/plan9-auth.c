/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Second Edition compatibility notes:
 * - passtokey: sys/src/libauth/passtokey.c
 * - codecs: sys/src/libauth/conv{TR,T,A}2M.c and convM2{TR,T,A}.c
 * - overlapping encryption: sys/man/2/encrypt (the preserved crypt.c is an
 *   architecture object in sys/src/libc/port/crypt.{2,8,v}.save).
 */
#include "qemu/osdep.h"
#include "crypto/cipher.h"
#include "hw/9pfs/plan9-auth.h"
#include "qapi/error.h"

static void plan9_auth_wipe(void *ptr, size_t len)
{
    /* Volatile stores resist removal of secret erasure by the optimizer. */
    volatile uint8_t *p = ptr;

    while (len--) {
        *p++ = 0;
    }
}

static uint8_t odd_parity(uint8_t value)
{
    uint8_t bits = value & 0xfe;
    uint8_t parity = 0;

    for (uint8_t scan = bits; scan; scan >>= 1) {
        parity ^= scan & 1;
    }
    return bits | !parity;
}

void plan9_auth_des56to64(const uint8_t packed[PLAN9_AUTH_DES_KEY_LEN],
                          uint8_t expanded[PLAN9_AUTH_DES_BLOCK_LEN])
{
    uint64_t key = 0;

    for (size_t i = 0; i < PLAN9_AUTH_DES_KEY_LEN; i++) {
        key = (key << 8) | packed[i];
    }
    for (size_t i = 0; i < PLAN9_AUTH_DES_BLOCK_LEN; i++) {
        uint64_t group = key >> ((PLAN9_AUTH_DES_BLOCK_LEN - 1 - i) * 7);

        expanded[i] = odd_parity((group & 0x7f) << 1);
    }
}

static int plan9_auth_block(QCryptoCipher *cipher, uint8_t *block,
                            bool decrypt, Error **errp)
{
    uint8_t input[PLAN9_AUTH_DES_BLOCK_LEN];
    uint8_t output[PLAN9_AUTH_DES_BLOCK_LEN];
    int ret;

    memcpy(input, block, sizeof(input));
    if (decrypt) {
        ret = qcrypto_cipher_decrypt(cipher, input, output, sizeof(output),
                                     errp);
    } else {
        ret = qcrypto_cipher_encrypt(cipher, input, output, sizeof(output),
                                     errp);
    }
    if (!ret) {
        memcpy(block, output, sizeof(output));
    }
    plan9_auth_wipe(input, sizeof(input));
    plan9_auth_wipe(output, sizeof(output));
    return ret;
}

static int plan9_auth_crypt(const uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                            uint8_t *data, size_t len, bool decrypt,
                            Error **errp)
{
    QCryptoCipher *cipher = NULL;
    uint8_t expanded[PLAN9_AUTH_DES_BLOCK_LEN];
    uint8_t *buffer;
    size_t blocks, remainder;
    int ret = -1;

    if (!key || !data) {
        error_setg(errp, "Plan 9 authentication key and data are required");
        goto out;
    }
    if (len < PLAN9_AUTH_DES_BLOCK_LEN || len > PLAN9_AUTH_MAX_CRYPT_LEN) {
        error_setg(errp, "Plan 9 authentication data length %zu is invalid",
                   len);
        goto out;
    }
    if (!qcrypto_cipher_supports(QCRYPTO_CIPHER_ALGO_DES,
                                 QCRYPTO_CIPHER_MODE_ECB)) {
        error_setg(errp, "QEMU crypto backend does not support DES/ECB");
        goto out;
    }

    plan9_auth_des56to64(key, expanded);
    cipher = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_DES,
                                QCRYPTO_CIPHER_MODE_ECB, expanded,
                                sizeof(expanded), errp);
    if (!cipher) {
        goto out;
    }

    blocks = (len - 1) / 7;
    remainder = (len - 1) % 7;
    if (decrypt) {
        buffer = data + blocks * 7;
        if (remainder && plan9_auth_block(cipher, buffer - 7 + remainder,
                                          true, errp)) {
            goto out;
        }
        while (blocks--) {
            buffer -= 7;
            if (plan9_auth_block(cipher, buffer, true, errp)) {
                goto out;
            }
        }
    } else {
        buffer = data;
        for (size_t i = 0; i < blocks; i++, buffer += 7) {
            if (plan9_auth_block(cipher, buffer, false, errp)) {
                goto out;
            }
        }
        if (remainder && plan9_auth_block(cipher, buffer - 7 + remainder,
                                          false, errp)) {
            goto out;
        }
    }
    ret = 0;

out:
    qcrypto_cipher_free(cipher);
    plan9_auth_wipe(expanded, sizeof(expanded));
    return ret;
}

int plan9_auth_encrypt(const uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                       uint8_t *data, size_t len, Error **errp)
{
    return plan9_auth_crypt(key, data, len, false, errp);
}

int plan9_auth_decrypt(const uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                       uint8_t *data, size_t len, Error **errp)
{
    return plan9_auth_crypt(key, data, len, true, errp);
}

int plan9_auth_passtokey(uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                         const char *password, Error **errp)
{
    uint8_t password_buffer[PLAN9_AUTH_NAMELEN] = { 0 };
    uint8_t work_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint8_t *cursor;
    size_t length;
    int ret = -1;

    if (!key || !password) {
        error_setg(errp, "Plan 9 password and key are required");
        goto out;
    }
    length = strnlen(password, PLAN9_AUTH_NAMELEN + 1);
    if (length >= PLAN9_AUTH_NAMELEN) {
        error_setg(errp, "Plan 9 passwords must be at most %u bytes",
                   PLAN9_AUTH_NAMELEN - 1);
        goto out;
    }

    memset(password_buffer, ' ', PLAN9_AUTH_DES_BLOCK_LEN);
    memcpy(password_buffer, password, length);
    password_buffer[length] = 0;
    cursor = password_buffer;
    for (;;) {
        for (size_t i = 0; i < PLAN9_AUTH_DES_KEY_LEN; i++) {
            work_key[i] = (cursor[i] >> i) + (cursor[i + 1] << (7 - i));
        }
        if (length <= PLAN9_AUTH_DES_BLOCK_LEN) {
            memcpy(key, work_key, sizeof(work_key));
            ret = 0;
            goto out;
        }
        length -= PLAN9_AUTH_DES_BLOCK_LEN;
        cursor += PLAN9_AUTH_DES_BLOCK_LEN;
        if (length < PLAN9_AUTH_DES_BLOCK_LEN) {
            cursor -= PLAN9_AUTH_DES_BLOCK_LEN - length;
            length = PLAN9_AUTH_DES_BLOCK_LEN;
        }
        if (plan9_auth_encrypt(work_key, cursor, PLAN9_AUTH_DES_BLOCK_LEN,
                               errp)) {
            goto out;
        }
    }

out:
    if (ret && key) {
        memset(key, 0, PLAN9_AUTH_DES_KEY_LEN);
    }
    plan9_auth_wipe(work_key, sizeof(work_key));
    plan9_auth_wipe(password_buffer, sizeof(password_buffer));
    return ret;
}

static int put_string(const char *string, uint8_t *out, size_t len,
                      const char *field, Error **errp)
{
    size_t actual;

    if (!string) {
        error_setg(errp, "Plan 9 %s is required", field);
        return -1;
    }
    actual = strnlen(string, len);
    if (actual == len) {
        error_setg(errp, "Plan 9 %s is too long for its fixed field", field);
        return -1;
    }
    memset(out, 0, len);
    memcpy(out, string, actual);
    return 0;
}

static void get_string(char *out, const uint8_t *wire, size_t len)
{
    memcpy(out, wire, len);
    out[len] = 0;
}

static int check_length(size_t got, size_t expected, const char *name,
                        Error **errp)
{
    if (got != expected) {
        error_setg(errp, "Plan 9 %s length %zu is not %zu", name, got,
                   expected);
        return -1;
    }
    return 0;
}

int plan9_auth_ticket_request_encode(const Plan9AuthTicketRequest *request,
                                     uint8_t out[PLAN9_AUTH_TICKET_REQUEST_LEN],
                                     Error **errp)
{
    uint8_t *p = out;

    if (!request || !out) {
        error_setg(errp, "Plan 9 ticket request and output are required");
        return -1;
    }
    *p++ = request->type;
    if (put_string(request->authid, p, PLAN9_AUTH_NAMELEN, "auth-id", errp)) {
        goto error;
    }
    p += PLAN9_AUTH_NAMELEN;
    if (put_string(request->authdom, p, PLAN9_AUTH_DOMLEN, "auth-domain",
                   errp)) {
        goto error;
    }
    p += PLAN9_AUTH_DOMLEN;
    memcpy(p, request->challenge, PLAN9_AUTH_CHALLENGE_LEN);
    p += PLAN9_AUTH_CHALLENGE_LEN;
    if (put_string(request->hostid, p, PLAN9_AUTH_NAMELEN, "host-id", errp)) {
        goto error;
    }
    p += PLAN9_AUTH_NAMELEN;
    if (put_string(request->uid, p, PLAN9_AUTH_NAMELEN, "user-id", errp)) {
        goto error;
    }
    return 0;

error:
    plan9_auth_wipe(out, PLAN9_AUTH_TICKET_REQUEST_LEN);
    return -1;
}

int plan9_auth_ticket_request_decode(const uint8_t *wire, size_t len,
                                     Plan9AuthTicketRequest *request,
                                     Error **errp)
{
    const uint8_t *p = wire;

    if (!wire || !request) {
        error_setg(errp, "Plan 9 ticket request input and output are required");
        return -1;
    }
    if (check_length(len, PLAN9_AUTH_TICKET_REQUEST_LEN, "ticket request",
                     errp)) {
        return -1;
    }
    memset(request, 0, sizeof(*request));
    request->type = *p++;
    get_string(request->authid, p, PLAN9_AUTH_NAMELEN);
    p += PLAN9_AUTH_NAMELEN;
    get_string(request->authdom, p, PLAN9_AUTH_DOMLEN);
    p += PLAN9_AUTH_DOMLEN;
    memcpy(request->challenge, p, PLAN9_AUTH_CHALLENGE_LEN);
    p += PLAN9_AUTH_CHALLENGE_LEN;
    get_string(request->hostid, p, PLAN9_AUTH_NAMELEN);
    p += PLAN9_AUTH_NAMELEN;
    get_string(request->uid, p, PLAN9_AUTH_NAMELEN);
    return 0;
}

int plan9_auth_ticket_encode(const Plan9AuthTicket *ticket,
                             uint8_t out[PLAN9_AUTH_TICKET_LEN], Error **errp)
{
    uint8_t *p = out;

    if (!ticket || !out) {
        error_setg(errp, "Plan 9 ticket and output are required");
        return -1;
    }
    *p++ = ticket->num;
    memcpy(p, ticket->challenge, PLAN9_AUTH_CHALLENGE_LEN);
    p += PLAN9_AUTH_CHALLENGE_LEN;
    if (put_string(ticket->cuid, p, PLAN9_AUTH_NAMELEN, "ticket client",
                   errp)) {
        goto error;
    }
    p += PLAN9_AUTH_NAMELEN;
    if (put_string(ticket->suid, p, PLAN9_AUTH_NAMELEN, "ticket server",
                   errp)) {
        goto error;
    }
    p += PLAN9_AUTH_NAMELEN;
    memcpy(p, ticket->key, PLAN9_AUTH_DES_KEY_LEN);
    return 0;

error:
    plan9_auth_wipe(out, PLAN9_AUTH_TICKET_LEN);
    return -1;
}

int plan9_auth_ticket_decode(const uint8_t *wire, size_t len,
                             Plan9AuthTicket *ticket, Error **errp)
{
    const uint8_t *p = wire;

    if (!wire || !ticket) {
        error_setg(errp, "Plan 9 ticket input and output are required");
        return -1;
    }
    if (check_length(len, PLAN9_AUTH_TICKET_LEN, "ticket", errp)) {
        return -1;
    }
    memset(ticket, 0, sizeof(*ticket));
    ticket->num = *p++;
    memcpy(ticket->challenge, p, PLAN9_AUTH_CHALLENGE_LEN);
    p += PLAN9_AUTH_CHALLENGE_LEN;
    get_string(ticket->cuid, p, PLAN9_AUTH_NAMELEN);
    p += PLAN9_AUTH_NAMELEN;
    get_string(ticket->suid, p, PLAN9_AUTH_NAMELEN);
    p += PLAN9_AUTH_NAMELEN;
    memcpy(ticket->key, p, PLAN9_AUTH_DES_KEY_LEN);
    return 0;
}

int plan9_auth_authenticator_encode(const Plan9AuthAuthenticator *auth,
                                    uint8_t out[PLAN9_AUTH_AUTHENTICATOR_LEN],
                                    Error **errp)
{
    if (!auth || !out) {
        error_setg(errp, "Plan 9 authenticator and output are required");
        return -1;
    }
    out[0] = auth->num;
    memcpy(out + 1, auth->challenge, PLAN9_AUTH_CHALLENGE_LEN);
    out[9] = auth->id;
    out[10] = auth->id >> 8;
    out[11] = auth->id >> 16;
    out[12] = auth->id >> 24;
    return 0;
}

int plan9_auth_authenticator_decode(const uint8_t *wire, size_t len,
                                    Plan9AuthAuthenticator *auth,
                                    Error **errp)
{
    if (!wire || !auth) {
        error_setg(errp, "Plan 9 authenticator input and output are required");
        return -1;
    }
    if (check_length(len, PLAN9_AUTH_AUTHENTICATOR_LEN, "authenticator",
                     errp)) {
        return -1;
    }
    memset(auth, 0, sizeof(*auth));
    auth->num = wire[0];
    memcpy(auth->challenge, wire + 1, PLAN9_AUTH_CHALLENGE_LEN);
    auth->id = (uint32_t)wire[9] | ((uint32_t)wire[10] << 8) |
               ((uint32_t)wire[11] << 16) | ((uint32_t)wire[12] << 24);
    return 0;
}
