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
#include "qom/object.h"

typedef struct Error Error;

#define TYPE_PLAN9P1_SERVER "plan9-9p1-server"
OBJECT_DECLARE_SIMPLE_TYPE(Plan9P1Server, PLAN9P1_SERVER)

typedef struct Plan9P1TransportOps {
    size_t (*can_send)(void *opaque);
    int (*send)(const uint8_t *buf, size_t len, void *opaque);
} Plan9P1TransportOps;

typedef struct Plan9P1ServerOptions {
    unsigned int max_devices;       /* 1..127; zero selects 127. */
    size_t max_queued_bytes;        /* zero selects the built-in bound. */
    size_t max_dir_cache_bytes;     /* zero selects the server-wide bound. */
    size_t max_qid_entries;         /* zero selects the built-in bound. */
} Plan9P1ServerOptions;

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
int plan9p1_server_receive(Plan9P1Server *server,
                           const uint8_t *buf, size_t len,
                           Error **errp);
void plan9p1_server_can_send(Plan9P1Server *server);

/* Reset is asynchronous when an operation or open fid needs worker cleanup. */
void plan9p1_server_reset(Plan9P1Server *server);
bool plan9p1_server_busy(const Plan9P1Server *server);

/*
 * Consumes the caller's QOM reference.  Each queued worker holds its own QOM
 * reference, so this is safe while backend work is pending.  A transport
 * callback may consume the caller reference this way; no later API use through
 * that caller-owned pointer is valid.
 */
void plan9p1_server_free(Plan9P1Server *server);

#endif /* HW_9PFS_PLAN9_9P1_SERVER_H */
