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
    PLAN9_AUTH_AUTHENTICATOR_LEN = 13,
    PLAN9_AUTH_MAX_CRYPT_LEN = 4096,
};

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
