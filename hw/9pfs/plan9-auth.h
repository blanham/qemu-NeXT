/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_9PFS_PLAN9_AUTH_H
#define HW_9PFS_PLAN9_AUTH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct Error Error;

enum {
    PLAN9_AUTH_NAMELEN = 28,
    PLAN9_AUTH_DOMLEN = 48,
    PLAN9_AUTH_DES_KEY_LEN = 7,
    PLAN9_AUTH_DES_BLOCK_LEN = 8,
    PLAN9_AUTH_CHALLENGE_LEN = 8,
    PLAN9_AUTH_TICKET_REQUEST_LEN = 141,
    PLAN9_AUTH_TICKET_LEN = 72,
    PLAN9_AUTH_TICKET_REPLY_LEN = 1 + 2 * PLAN9_AUTH_TICKET_LEN,
    PLAN9_AUTH_ERROR_LEN = 64, /* Historical libc ERRLEN. */
    PLAN9_AUTH_ERROR_REPLY_LEN = 1 + PLAN9_AUTH_ERROR_LEN,
    /* Enough for the boot checkkey + attach exchanges, with a DoS bound. */
    PLAN9_AUTH_TICKET_MAX_REQUESTS = 4,
    PLAN9_AUTH_AUTHENTICATOR_LEN = 13,
    PLAN9_AUTH_MAX_CRYPT_LEN = 4096,
    /* A 168 KiB database is already far beyond the original 512-user keyfs. */
    PLAN9_AUTH_KEYDB_MAX_RECORDS = 4096,
    PLAN9_AUTH_KEYDB_RECORD_LEN = 41,
};

typedef struct Plan9AuthKeydb Plan9AuthKeydb;
typedef struct Plan9AuthTicketService Plan9AuthTicketService;
typedef struct Plan9AuthTicketConnection Plan9AuthTicketConnection;

typedef enum Plan9AuthKeyStatus {
    PLAN9_AUTH_KEY_MISSING,
    PLAN9_AUTH_KEY_AVAILABLE,
    PLAN9_AUTH_KEY_DISABLED,
    PLAN9_AUTH_KEY_EXPIRED,
} Plan9AuthKeyStatus;

typedef void (*Plan9AuthKeydbReadHook)(const char *path, void *opaque);
typedef void (*Plan9AuthKeydbLookupHook)(size_t index, void *opaque);

typedef int (*Plan9AuthRandomBytes)(void *buf, size_t len, void *opaque,
                                    Error **errp);
typedef uint32_t (*Plan9AuthNowSeconds)(void *opaque);

typedef struct Plan9AuthTicketServiceConfig {
    /* Immutable and caller-owned; it must outlive the service/connections. */
    const Plan9AuthKeydb *keydb;
    /* NULL selects the host wall clock for expiry checks on every request. */
    Plan9AuthNowSeconds now_seconds;
    void *now_opaque;
    /* NULL selects qcrypto_random_bytes(). */
    Plan9AuthRandomBytes random_bytes;
    void *random_opaque;
} Plan9AuthTicketServiceConfig;

typedef struct Plan9AuthTicketTransportOps {
    /* Return len for atomic acceptance, -EAGAIN, or another fatal result. */
    int (*send_record)(const uint8_t *buf, size_t len, void *opaque);
    /* Request asynchronous transport close; it may free the connection. */
    void (*close)(void *opaque);
} Plan9AuthTicketTransportOps;

typedef enum Plan9AuthType {
    PLAN9_AUTH_TREQ = 1,
    PLAN9_AUTH_OK = 4,
    PLAN9_AUTH_ERR = 5,
    PLAN9_AUTH_TS = 64,
    PLAN9_AUTH_TC = 65,
    PLAN9_AUTH_AS = 66,
    PLAN9_AUTH_AC = 67,
} Plan9AuthType;

typedef struct Plan9AuthTicketRequest {
    uint8_t type;
    char authid[PLAN9_AUTH_NAMELEN + 1];
    char authdom[PLAN9_AUTH_DOMLEN + 1];
    uint8_t challenge[PLAN9_AUTH_CHALLENGE_LEN];
    char hostid[PLAN9_AUTH_NAMELEN + 1];
    char uid[PLAN9_AUTH_NAMELEN + 1];
} Plan9AuthTicketRequest;

typedef struct Plan9AuthTicket {
    uint8_t num;
    uint8_t challenge[PLAN9_AUTH_CHALLENGE_LEN];
    char cuid[PLAN9_AUTH_NAMELEN + 1];
    char suid[PLAN9_AUTH_NAMELEN + 1];
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];
} Plan9AuthTicket;

typedef struct Plan9AuthAuthenticator {
    uint8_t num;
    uint8_t challenge[PLAN9_AUTH_CHALLENGE_LEN];
    uint32_t id;
} Plan9AuthAuthenticator;

/*
 * Securely clear caller-owned secrets.  The implementation uses volatile
 * stores so the compiler cannot omit the erase.  Call after every successful
 * passtokey, decoded ticket, or plaintext ticket/authentication exchange.
 */
void plan9_auth_clear(void *ptr, size_t len);

/* A typed wrapper for a decoded ticket containing a conversation key. */
void plan9_auth_ticket_clear(Plan9AuthTicket *ticket);

/*
 * Load native Second Edition /adm/keys records encrypted independently with
 * master_key.  server_id must name an enabled, unexpired record at now.
 */
Plan9AuthKeydb *plan9_auth_keydb_load(const char *path,
                                      const uint8_t master_key[
                                          PLAN9_AUTH_DES_KEY_LEN],
                                      const char *server_id, uint32_t now,
                                      Error **errp);
void plan9_auth_keydb_free(Plan9AuthKeydb *keydb);
Plan9AuthKeyStatus plan9_auth_keydb_lookup(const Plan9AuthKeydb *keydb,
                                           const char *name, uint32_t now,
                                           uint8_t key[
                                               PLAN9_AUTH_DES_KEY_LEN]);

/*
 * Short-lived historical IL/566 ticket service.  Each connection accepts a
 * bounded sequence of complete AuthTreq records and sends one atomic AuthOK
 * reply for each.  Only one reply may be outstanding at a time.  Second
 * Edition boot performs two exchanges on the same connection.  Malformed or
 * unsupported records receive a fixed AuthErr record before transport close.
 * Connection free consumes the caller reference and is safe from transport
 * callbacks; the pointer must not be used afterward.
 */
Plan9AuthTicketService *plan9_auth_ticket_service_new(
    const Plan9AuthTicketServiceConfig *config, Error **errp);
void plan9_auth_ticket_service_free(Plan9AuthTicketService *service);
Plan9AuthTicketConnection *plan9_auth_ticket_connection_new(
    Plan9AuthTicketService *service,
    const Plan9AuthTicketTransportOps *ops, void *transport_opaque,
    Error **errp);
int plan9_auth_ticket_connection_receive_record(
    Plan9AuthTicketConnection *connection, const uint8_t *buf, size_t len,
    Error **errp);
void plan9_auth_ticket_connection_can_send(
    Plan9AuthTicketConnection *connection);
void plan9_auth_ticket_connection_free(
    Plan9AuthTicketConnection *connection);

/* Pack and encrypt one native 41-byte /adm/keys record. */
int plan9_auth_keydb_record_encode(
    uint8_t out[PLAN9_AUTH_KEYDB_RECORD_LEN],
    const uint8_t master_key[PLAN9_AUTH_DES_KEY_LEN], const char *name,
    const uint8_t key[PLAN9_AUTH_DES_KEY_LEN], uint8_t status,
    uint8_t warnings, uint32_t expiry, Error **errp);

/* Unit-test-only race injection point; production code never installs one. */
void plan9_auth_keydb_set_read_hook(Plan9AuthKeydbReadHook hook,
                                    void *opaque);
/* Unit-test-only visit instrumentation; production code never installs it. */
void plan9_auth_keydb_set_lookup_hook(Plan9AuthKeydbLookupHook hook,
                                      void *opaque);

/*
 * Passwords must be representable in a historical NAMELEN field (<= 27).
 * This deliberate helper constraint rejects too-long byte strings instead of
 * silently truncating them; the count is bytes, not Unicode code points.
 */
int plan9_auth_passtokey(uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                         const char *password, Error **errp);

/* Expand Plan 9's packed 56-bit DES key to an odd-parity DES key. */
void plan9_auth_des56to64(const uint8_t packed[PLAN9_AUTH_DES_KEY_LEN],
                          uint8_t expanded[PLAN9_AUTH_DES_BLOCK_LEN]);

/* Apply the historical overlapping-block Plan 9 transform in place. */
int plan9_auth_encrypt(const uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                       uint8_t *data, size_t len, Error **errp);
int plan9_auth_decrypt(const uint8_t key[PLAN9_AUTH_DES_KEY_LEN],
                       uint8_t *data, size_t len, Error **errp);

int plan9_auth_ticket_request_encode(const Plan9AuthTicketRequest *request,
                                     uint8_t out[PLAN9_AUTH_TICKET_REQUEST_LEN],
                                     Error **errp);
int plan9_auth_ticket_request_decode(const uint8_t *wire, size_t len,
                                     Plan9AuthTicketRequest *request,
                                     Error **errp);
int plan9_auth_ticket_encode(const Plan9AuthTicket *ticket,
                             uint8_t out[PLAN9_AUTH_TICKET_LEN], Error **errp);
int plan9_auth_ticket_decode(const uint8_t *wire, size_t len,
                             Plan9AuthTicket *ticket, Error **errp);
int plan9_auth_authenticator_encode(const Plan9AuthAuthenticator *auth,
                                    uint8_t out[PLAN9_AUTH_AUTHENTICATOR_LEN],
                                    Error **errp);
int plan9_auth_authenticator_decode(const uint8_t *wire, size_t len,
                                    Plan9AuthAuthenticator *auth,
                                    Error **errp);

#endif /* HW_9PFS_PLAN9_AUTH_H */
