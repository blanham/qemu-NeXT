/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef TESTS_UNIT_PLAN9_9P1_SLIRP_STUB_H
#define TESTS_UNIT_PLAN9_9P1_SLIRP_STUB_H

typedef struct Plan9P1SlirpStubConnection Plan9P1SlirpStubConnection;

void plan9p1_slirp_stub_enable(void);
void plan9p1_slirp_stub_disable(void);
void plan9p1_slirp_stub_set_deferred_close(bool enabled);
bool plan9p1_slirp_stub_had_duplicate_close(void);
unsigned int plan9p1_slirp_stub_listener_count(void);
uint16_t plan9p1_slirp_stub_listener_port(unsigned int index);
uint32_t plan9p1_slirp_stub_listener_address(unsigned int index);
uint32_t plan9p1_slirp_stub_bootp_file_address(void);
uint32_t plan9p1_slirp_stub_bootp_auth_address(void);
Plan9P1SlirpStubConnection *plan9p1_slirp_stub_open(uint16_t port);
bool plan9p1_slirp_stub_connection_accepted(
    const Plan9P1SlirpStubConnection *connection);
bool plan9p1_slirp_stub_connection_closed(
    const Plan9P1SlirpStubConnection *connection);
unsigned int plan9p1_slirp_stub_connection_close_requests(
    const Plan9P1SlirpStubConnection *connection);
void plan9p1_slirp_stub_close(Plan9P1SlirpStubConnection *connection);
void plan9p1_slirp_stub_deliver_close(Plan9P1SlirpStubConnection *connection);
void plan9p1_slirp_stub_deliver_record(Plan9P1SlirpStubConnection *connection,
                                      const uint8_t *data, size_t len);
GBytes *plan9p1_slirp_stub_pop_sent_record(
    Plan9P1SlirpStubConnection *connection);

#endif
