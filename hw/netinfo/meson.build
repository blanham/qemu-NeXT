# SPDX-License-Identifier: GPL-2.0-or-later
system_ss.add(files('netinfo-xdr.c', 'netinfo-db.c'))
system_ss.add(when: slirp, if_true: files('netinfo-server.c'))
