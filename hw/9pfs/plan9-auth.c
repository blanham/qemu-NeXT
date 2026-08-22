/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Second Edition compatibility notes:
 * - passtokey: sys/src/libauth/passtokey.c
 * - codecs: sys/src/libauth/conv{TR,T,A}2M.c and convM2{TR,T,A}.c
 * - overlapping encryption: sys/man/2/encrypt (the preserved crypt.c is an
 *   architecture object in sys/src/libc/port/crypt.{2,8,v}.save).
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "crypto/cipher.h"
#include "hw/9pfs/plan9-auth.h"
#include "qapi/error.h"

typedef struct Plan9AuthKeydbEntry {
    char name[PLAN9_AUTH_NAMELEN];
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];
    uint8_t status;
    uint8_t warnings;
    uint32_t expiry;
} Plan9AuthKeydbEntry;

struct Plan9AuthKeydb {
    size_t count;
    Plan9AuthKeydbEntry *entries;
};

static Plan9AuthKeydbReadHook keydb_read_hook;
static void *keydb_read_hook_opaque;

void plan9_auth_clear(void *ptr, size_t len)
{
    qcrypto_memzero(ptr, len);
}

void plan9_auth_ticket_clear(Plan9AuthTicket *ticket)
{
    if (ticket) {
        plan9_auth_clear(ticket, sizeof(*ticket));
    }
}

void plan9_auth_keydb_set_read_hook(Plan9AuthKeydbReadHook hook,
                                    void *opaque)
{
    keydb_read_hook = hook;
    keydb_read_hook_opaque = opaque;
}

static bool plan9_auth_keydb_metadata_equal(const struct stat *a,
                                            const struct stat *b)
{
    if (a->st_dev != b->st_dev || a->st_ino != b->st_ino ||
        a->st_mode != b->st_mode || a->st_size != b->st_size ||
        a->st_mtime != b->st_mtime || a->st_ctime != b->st_ctime) {
        return false;
    }
#if defined(CONFIG_DARWIN) || defined(CONFIG_FREEBSD)
    return a->st_mtimespec.tv_nsec == b->st_mtimespec.tv_nsec &&
           a->st_ctimespec.tv_nsec == b->st_ctimespec.tv_nsec;
#else
    return a->st_mtim.tv_nsec == b->st_mtim.tv_nsec &&
           a->st_ctim.tv_nsec == b->st_ctim.tv_nsec;
#endif
}

static Plan9AuthKeyStatus
plan9_auth_keydb_entry_status(const Plan9AuthKeydbEntry *entry, uint32_t now)
{
    if (entry->status == 1) {
        return PLAN9_AUTH_KEY_DISABLED;
    }
    if (entry->expiry && entry->expiry < now) {
        return PLAN9_AUTH_KEY_EXPIRED;
    }
    return PLAN9_AUTH_KEY_AVAILABLE;
}

void plan9_auth_keydb_free(Plan9AuthKeydb *keydb)
{
    if (!keydb) {
        return;
    }
    plan9_auth_clear(keydb->entries,
                    keydb->count * sizeof(*keydb->entries));
    g_free(keydb->entries);
    plan9_auth_clear(keydb, sizeof(*keydb));
    g_free(keydb);
}

Plan9AuthKeyStatus plan9_auth_keydb_lookup(const Plan9AuthKeydb *keydb,
                                           const char *name, uint32_t now,
                                           uint8_t key[
                                               PLAN9_AUTH_DES_KEY_LEN])
{
    if (key) {
        plan9_auth_clear(key, PLAN9_AUTH_DES_KEY_LEN);
    }
    if (!keydb || !name) {
        return PLAN9_AUTH_KEY_MISSING;
    }

    for (size_t i = 0; i < keydb->count; i++) {
        const Plan9AuthKeydbEntry *entry = &keydb->entries[i];
        Plan9AuthKeyStatus status;

        if (strcmp(entry->name, name)) {
            continue;
        }
        status = plan9_auth_keydb_entry_status(entry, now);
        if (status == PLAN9_AUTH_KEY_AVAILABLE && key) {
            memcpy(key, entry->key, PLAN9_AUTH_DES_KEY_LEN);
        }
        return status;
    }
    return PLAN9_AUTH_KEY_MISSING;
}

Plan9AuthKeydb *plan9_auth_keydb_load(const char *path,
                                      const uint8_t master_key[
                                          PLAN9_AUTH_DES_KEY_LEN],
                                      const char *server_id, uint32_t now,
                                      Error **errp)
{
    Plan9AuthKeydb *keydb = NULL;
    struct stat before, after, pathname;
    uint8_t *records = NULL;
    uint8_t eof;
    int fd = -1;
    size_t bytes = 0, count, offset = 0;
    ssize_t got;

    if (!path || !master_key || !server_id) {
        error_setg(errp,
                   "Plan 9 key database path, key, and server are required");
        return NULL;
    }

    fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
    if (fd < 0) {
        error_setg_errno(errp, errno, "cannot open Plan 9 key database");
        goto fail;
    }
    if (fstat(fd, &before) < 0) {
        error_setg_errno(errp, errno, "cannot stat Plan 9 key database");
        goto fail;
    }
    if (!S_ISREG(before.st_mode)) {
        error_setg(errp, "Plan 9 key database is not a regular file");
        goto fail;
    }
    if (before.st_size <= 0 ||
        before.st_size % PLAN9_AUTH_KEYDB_RECORD_LEN ||
        before.st_size / PLAN9_AUTH_KEYDB_RECORD_LEN >
            PLAN9_AUTH_KEYDB_MAX_RECORDS) {
        error_setg(errp, "Plan 9 key database has an invalid size");
        goto fail;
    }
    bytes = before.st_size;
    count = bytes / PLAN9_AUTH_KEYDB_RECORD_LEN;
    records = g_malloc(bytes);
    while (offset < bytes) {
        got = read(fd, records + offset, bytes - offset);
        if (got < 0 && errno == EINTR) {
            continue;
        }
        if (got <= 0) {
            if (got < 0) {
                error_setg_errno(errp, errno,
                                 "cannot read Plan 9 key database");
            } else {
                error_setg(errp, "Plan 9 key database changed during read");
            }
            goto fail;
        }
        offset += got;
    }
    do {
        got = read(fd, &eof, sizeof(eof));
    } while (got < 0 && errno == EINTR);
    if (got != 0) {
        if (got < 0) {
            error_setg_errno(errp, errno,
                             "cannot finish reading Plan 9 key database");
        } else {
            error_setg(errp, "Plan 9 key database changed during read");
        }
        goto fail;
    }

    if (keydb_read_hook) {
        keydb_read_hook(path, keydb_read_hook_opaque);
    }
    if (fstat(fd, &after) < 0 ||
        fstatat(AT_FDCWD, path, &pathname, AT_SYMLINK_NOFOLLOW) < 0) {
        error_setg_errno(errp, errno, "cannot revalidate Plan 9 key database");
        goto fail;
    }
    if (!plan9_auth_keydb_metadata_equal(&before, &after) ||
        !plan9_auth_keydb_metadata_equal(&before, &pathname)) {
        error_setg(errp, "Plan 9 key database changed during read");
        goto fail;
    }

    keydb = g_new0(Plan9AuthKeydb, 1);
    keydb->count = count;
    keydb->entries = g_new0(Plan9AuthKeydbEntry, count);
    for (size_t i = 0; i < count; i++) {
        uint8_t *record = records + i * PLAN9_AUTH_KEYDB_RECORD_LEN;
        Plan9AuthKeydbEntry *entry = &keydb->entries[i];

        if (plan9_auth_decrypt(master_key, record,
                               PLAN9_AUTH_KEYDB_RECORD_LEN, errp)) {
            goto fail;
        }
        memcpy(entry->name, record, PLAN9_AUTH_NAMELEN);
        /* Match passline(): historical full-width names lose byte 27. */
        entry->name[PLAN9_AUTH_NAMELEN - 1] = 0;
        memcpy(entry->key, record + PLAN9_AUTH_NAMELEN,
               PLAN9_AUTH_DES_KEY_LEN);
        entry->status = record[PLAN9_AUTH_NAMELEN + PLAN9_AUTH_DES_KEY_LEN];
        entry->warnings = record[PLAN9_AUTH_NAMELEN +
                                 PLAN9_AUTH_DES_KEY_LEN + 1];
        entry->expiry = ldl_le_p(record + PLAN9_AUTH_NAMELEN +
                                 PLAN9_AUTH_DES_KEY_LEN + 2);
        if (entry->status >= 2) {
            error_setg(errp, "Plan 9 key database record has invalid status");
            goto fail;
        }
        for (size_t other = 0; other < i; other++) {
            if (!strcmp(entry->name, keydb->entries[other].name)) {
                error_setg(errp, "Plan 9 key database has duplicate names");
                goto fail;
            }
        }
    }
    if (plan9_auth_keydb_lookup(keydb, server_id, now, NULL) !=
        PLAN9_AUTH_KEY_AVAILABLE) {
        error_setg(errp, "Plan 9 key database server is unavailable");
        goto fail;
    }

    close(fd);
    plan9_auth_clear(records, bytes);
    g_free(records);
    return keydb;

fail:
    if (fd >= 0) {
        close(fd);
    }
    plan9_auth_clear(records, bytes);
    g_free(records);
    plan9_auth_keydb_free(keydb);
    return NULL;
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
    uint64_t group = 0;

    for (size_t i = 0; i < PLAN9_AUTH_DES_KEY_LEN; i++) {
        key = (key << 8) | packed[i];
    }
    for (size_t i = 0; i < PLAN9_AUTH_DES_BLOCK_LEN; i++) {
        group = key >> ((PLAN9_AUTH_DES_BLOCK_LEN - 1 - i) * 7);

        expanded[i] = odd_parity((group & 0x7f) << 1);
    }
    plan9_auth_clear(&group, sizeof(group));
    plan9_auth_clear(&key, sizeof(key));
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
    plan9_auth_clear(input, sizeof(input));
    plan9_auth_clear(output, sizeof(output));
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
    plan9_auth_clear(expanded, sizeof(expanded));
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
    /*
     * Deliberately diverge from historical passtokey's truncation: helper
     * callers must not silently authenticate a different byte string.
     */
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
        plan9_auth_clear(key, PLAN9_AUTH_DES_KEY_LEN);
    }
    plan9_auth_clear(work_key, sizeof(work_key));
    plan9_auth_clear(password_buffer, sizeof(password_buffer));
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
    /* Match convM2*: a full wire field is valid but loses its final byte. */
    out[len - 1] = 0;
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
        if (out) {
            plan9_auth_clear(out, PLAN9_AUTH_TICKET_REQUEST_LEN);
        }
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
    plan9_auth_clear(out, PLAN9_AUTH_TICKET_REQUEST_LEN);
    return -1;
}

int plan9_auth_ticket_request_decode(const uint8_t *wire, size_t len,
                                     Plan9AuthTicketRequest *request,
                                     Error **errp)
{
    const uint8_t *p = wire;

    if (request) {
        plan9_auth_clear(request, sizeof(*request));
    }
    if (!wire || !request) {
        error_setg(errp, "Plan 9 ticket request input and output are required");
        return -1;
    }
    if (check_length(len, PLAN9_AUTH_TICKET_REQUEST_LEN, "ticket request",
                     errp)) {
        return -1;
    }
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
        if (out) {
            plan9_auth_clear(out, PLAN9_AUTH_TICKET_LEN);
        }
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
    plan9_auth_clear(out, PLAN9_AUTH_TICKET_LEN);
    return -1;
}

int plan9_auth_ticket_decode(const uint8_t *wire, size_t len,
                             Plan9AuthTicket *ticket, Error **errp)
{
    const uint8_t *p = wire;

    plan9_auth_ticket_clear(ticket);
    if (!wire || !ticket) {
        error_setg(errp, "Plan 9 ticket input and output are required");
        return -1;
    }
    if (check_length(len, PLAN9_AUTH_TICKET_LEN, "ticket", errp)) {
        return -1;
    }
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
        if (out) {
            plan9_auth_clear(out, PLAN9_AUTH_AUTHENTICATOR_LEN);
        }
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
    if (auth) {
        plan9_auth_clear(auth, sizeof(*auth));
    }
    if (!wire || !auth) {
        error_setg(errp, "Plan 9 authenticator input and output are required");
        return -1;
    }
    if (check_length(len, PLAN9_AUTH_AUTHENTICATOR_LEN, "authenticator",
                     errp)) {
        return -1;
    }
    auth->num = wire[0];
    memcpy(auth->challenge, wire + 1, PLAN9_AUTH_CHALLENGE_LEN);
    auth->id = (uint32_t)wire[9] | ((uint32_t)wire[10] << 8) |
               ((uint32_t)wire[11] << 16) | ((uint32_t)wire[12] << 24);
    return 0;
}
