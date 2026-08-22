// SPDX-License-Identifier: NCSA
/*
 * Copyright (c) 2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimers.
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

#ifndef HW_9PFS_PLAN9_9P1_SERVER_H
#define HW_9PFS_PLAN9_9P1_SERVER_H

#include "hw/9pfs/9p-backend.h"
#include "hw/9pfs/plan9-auth.h"
#include "qom/object.h"

typedef struct Error Error;

#define TYPE_PLAN9P1_SERVER "plan9-9p1-server"
OBJECT_DECLARE_SIMPLE_TYPE(Plan9P1Server, PLAN9P1_SERVER)

typedef enum Plan9P1TransportKind {
    PLAN9P1_TRANSPORT_STREAM,
    PLAN9P1_TRANSPORT_RECORD,
} Plan9P1TransportKind;

typedef struct Plan9P1TransportOps {
    /* Stream delivery preserves its existing partial-write semantics. */
    size_t (*can_send)(void *opaque);
    int (*send)(const uint8_t *buf, size_t len, void *opaque);
    /*
     * Omitted initializers select STREAM.  In RECORD mode, can_send() is the
     * capacity for one complete record and send() must return len after
     * atomically accepting it, or -EAGAIN without consuming it.  Any other
     * result is a fatal transport failure.
     */
    Plan9P1TransportKind kind;
} Plan9P1TransportOps;

typedef struct Plan9P1ServerOptions {
    unsigned int max_devices;       /* 1..127; zero selects 127. */
    size_t max_queued_bytes;        /* zero selects the built-in bound. */
    size_t max_dir_cache_bytes;     /* zero selects the server-wide bound. */
    size_t max_qid_entries;         /* zero selects the built-in bound. */
} Plan9P1ServerOptions;

typedef struct Plan9P1AuthConfig {
    /* Immutable and caller-owned; it must outlive the server. */
    const Plan9AuthKeydb *keydb;
    const char *auth_id;
    const char *auth_domain;
    /* NULL callbacks select QEMU crypto randomness and host wall time. */
    Plan9AuthRandomBytes random_bytes;
    void *random_opaque;
    Plan9AuthNowSeconds now_seconds;
    void *now_opaque;
} Plan9P1AuthConfig;

typedef struct Plan9P1ReplayState {
    uint32_t low;
    uint32_t used;
} Plan9P1ReplayState;

/* Exact Second Edition unsigned 32-ID replay window, including wraparound. */
bool plan9p1_replay_accept(Plan9P1ReplayState *state, uint32_t id);

/*
 * All entry points run on the server's main AioContext.  receive() only
 * frames and queues input; it never enters a coroutine on the caller's stack.
 * The object embeds and owns the initialized V9fsBackend, and backend workers
 * hold QOM references until their main-context completion path retires.
 * Transport callbacks may reenter receive(), reset(), can_send(), or free();
 * destructive state changes are deferred until the callback returns.
 * A fatal protocol or transport failure invalidates only the current session;
 * a replacement connection must begin with Tsession before sending other
 * requests.
 */

Plan9P1Server *plan9p1_server_new(const char *fsdev_id,
                                  const Plan9P1TransportOps *ops,
                                  void *transport_opaque,
                                  const Plan9P1ServerOptions *options,
                                  Error **errp);

/* Split initialization lets the later UserCreatable wrapper unwind cleanly. */
int plan9p1_server_backend_init(Plan9P1Server *server,
                                const char *fsdev_id,
                                const Plan9P1ServerOptions *options,
                                Error **errp);
int plan9p1_server_start(Plan9P1Server *server,
                         const Plan9P1TransportOps *ops,
                         void *transport_opaque,
                         Error **errp);
/*
 * Must be called before start(); record transport alone does not enable auth.
 */
int plan9p1_server_configure_auth(Plan9P1Server *server,
                                  const Plan9P1AuthConfig *config,
                                  Error **errp);
int plan9p1_server_receive(Plan9P1Server *server,
                           const uint8_t *buf, size_t len,
                           Error **errp);
void plan9p1_server_can_send(Plan9P1Server *server);

/* A record transport must call this after every peer disconnect. */
void plan9p1_server_connection_closed(Plan9P1Server *server);

/* Reset is asynchronous when an operation or open fid needs worker cleanup. */
void plan9p1_server_reset(Plan9P1Server *server);
bool plan9p1_server_busy(const Plan9P1Server *server);

/* True after record-connection cleanup has fully retired. */
bool plan9p1_server_record_connection_ready(const Plan9P1Server *server);

/*
 * Stop accepting transport input and start asynchronous fid cleanup without
 * consuming a caller reference.  This is idempotent and is used by QOM
 * unparent as well as the consuming free() API.
 */
void plan9p1_server_begin_close(Plan9P1Server *server);

/*
 * Consumes the caller's QOM reference.  Each queued worker holds its own QOM
 * reference, so this is safe while backend work is pending.  A transport
 * callback may consume the caller reference this way; no later API use through
 * that caller-owned pointer is valid.
 */
void plan9p1_server_free(Plan9P1Server *server);

#endif /* HW_9PFS_PLAN9_9P1_SERVER_H */
