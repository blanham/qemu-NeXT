# SPDX-License-Identifier: GPL-2.0-or-later
system_ss.add(files('netinfo-xdr.c', 'netinfo-db.c'))
if slirp.found() and slirp_udp_service and slirp_udp_listen_full and \
   slirp_tcp_service
  system_ss.add(files('netinfo-server.c', 'netinfo-object.c'))
endif
