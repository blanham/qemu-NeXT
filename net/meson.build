system_ss.add(files(
  'announce.c',
  'checksum.c',
  'dgram.c',
  'dump.c',
  'eth.c',
  'filter-buffer.c',
  'filter-mirror.c',
  'filter.c',
  'hub.c',
  'net-hmp-cmds.c',
  'net.c',
  'queue.c',
  'socket.c',
  'stream.c',
  'stream_data.c',
  'util.c',
))

if get_option('replication').allowed() or \
    get_option('colo_proxy').allowed()
  system_ss.add(files('colo-compare.c'))
  system_ss.add(files('colo.c'))
else
  system_ss.add(files('colo-stubs.c'))
endif

if get_option('colo_proxy').allowed()
  system_ss.add(files('filter-rewriter.c'))
endif

system_ss.add(when: 'CONFIG_TCG', if_true: files('filter-replay.c'))

if have_l2tpv3
  system_ss.add(files('l2tpv3.c'))
endif
if enable_passt
  system_ss.add(files('passt.c'))
endif

# Do not infer IL support from a libslirp version: distributions can backport
# or omit it.  The feature macro and every public entry point must be present
# in the dependency we are about to link.
slirp_il = false
if slirp.found()
  if slirp.type_name() == 'internal'
    # The forced fallback is compiled from the pinned wrap below, whose
    # SLIRP_HAVE_IL guard remains the authoritative compiler check.
    slirp_il = true
  else
    slirp_il = cc.links('''
      #include <libslirp.h>
      #ifndef SLIRP_HAVE_IL
      #error libslirp was built without IL
      #endif
      int main(void)
      {
        SlirpILListener *(*listen)(Slirp *, struct in_addr, uint16_t,
                                   const SlirpILCallbacks *, void *) =
            slirp_il_listen;
        void (*remove_listener)(SlirpILListener *) = slirp_il_listener_remove;
        int (*send_record)(SlirpILConnection *, const uint8_t *, size_t) =
            slirp_il_send_record;
        void (*close_connection)(SlirpILConnection *) =
            slirp_il_connection_close;

        return !(listen && remove_listener && send_record && close_connection);
      }''', dependencies: slirp,
      name: 'libslirp IL public API')
  endif
endif

system_ss.add(files('slirp-il.c'))
system_ss.add(when: slirp, if_true: files('slirp.c', 'slirp-guestfwd.c'))
system_ss.add(when: vde, if_true: files('vde.c'))
if have_netmap
  system_ss.add(files('netmap.c'))
endif

system_ss.add(when: [libxdp, libbpf], if_true: files('af-xdp.c'))

if have_vhost_net_user
  system_ss.add(when: 'CONFIG_VIRTIO_NET', if_true: files('vhost-user.c'))
endif
stub_ss.add(files('vhost-user-stub.c'))

if host_os == 'windows'
  system_ss.add(files('tap-win32.c'))
elif host_os == 'linux'
  system_ss.add(files('tap.c', 'tap-linux.c'))
elif host_os in bsd_oses
  system_ss.add(files('tap.c', 'tap-bsd.c'))
elif host_os == 'sunos'
  system_ss.add(files('tap.c', 'tap-solaris.c'))
else
  system_ss.add(files('tap.c', 'tap-stub.c'))
endif
if have_vhost_net_vdpa
  system_ss.add(when: 'CONFIG_VIRTIO_NET', if_true: files('vhost-vdpa.c'))
endif
stub_ss.add(files('vhost-vdpa-stub.c'))

vmnet_files = files(
  'vmnet-common.m',
  'vmnet-bridged.m',
  'vmnet-host.c',
  'vmnet-shared.c'
)
system_ss.add(when: vmnet, if_true: vmnet_files)
subdir('can')
