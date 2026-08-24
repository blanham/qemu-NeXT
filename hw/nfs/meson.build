system_ss.add(files('nfs2-xdr.c', 'nfs2-handle.c'))
system_ss.add(when: 'CONFIG_FSDEV_9P', if_true: files('nfs2-server.c'))
if have_virtfs and slirp.found() and slirp_udp_service and slirp_bootp_root
  system_ss.add(when: 'CONFIG_FSDEV_9P', if_true: files('nfs2-object.c'))
endif
