
testblock = declare_dependency(dependencies: [block], sources: 'iothread.c')
nfs2_handle_testing = declare_dependency(
  compile_args: ['-DNFS2_HANDLE_TESTING'],
)

tests = {
  'check-block-qdict': [],
  'check-qdict': [],
  'check-qnum': [],
  'check-qstring': [],
  'check-qlist': [],
  'check-qnull': [],
  'check-qobject': [],
  'check-qjson': [],
  'check-json-parser': [],
  'check-qlit': [],
  'test-error-report': [],
  'test-qobject-output-visitor': [testqapi],
  'test-clone-visitor': [testqapi],
  'test-qobject-input-visitor': [testqapi],
  'test-forward-visitor': [testqapi],
  'test-string-input-visitor': [testqapi],
  'test-string-output-visitor': [testqapi],
  'test-visitor-serialization': [testqapi],
  'test-bitmap': [],
  'test-resv-mem': [],
  # all code tested by test-x86-topo is inside topology.h
  'test-x86-topo': [],
  'test-cutils': [],
  'test-div128': [],
  'test-shift128': [],
  'test-mul64': [],
  # all code tested by test-int128 is inside int128.h
  'test-int128': [],
  'rcutorture': [],
  'test-rcu-list': [],
  'test-rcu-simpleq': [],
  'test-rcu-tailq': [],
  'test-rcu-slist': [],
  'test-qdist': [],
  'test-qht': [],
  'test-qtree': [],
  'test-bitops': [],
  'test-bitcnt': [],
  'test-qgraph': ['../qtest/libqos/qgraph.c'],
  'check-qom-interface': [qom],
  'check-qom-proplist': [qom],
  'test-qemu-opts': [],
  'test-keyval': [testqapi],
  'test-logging': [],
  'test-qapi-util': [],
  'test-interval-tree': [],
  'test-fifo': [],
  'test-envlist': [],
  'test-icount-period': [],
  'test-plan9-9p1-codec': [
    meson.project_source_root() / 'hw/9pfs/plan9-9p1-codec.c',
  ],
  'test-onc-rpc': [
    meson.project_source_root() / 'net/onc-rpc-xdr.c',
  ],
  'test-nfs2-xdr': [
    meson.project_source_root() / 'hw/nfs/nfs2-xdr.c',
    meson.project_source_root() / 'net/onc-rpc-xdr.c',
  ],
  'test-nfs2-handle': [
    meson.project_source_root() / 'hw/nfs/nfs2-handle.c',
    crypto,
    nfs2_handle_testing,
  ],
  'test-nfs2-server': [
    meson.project_source_root() / 'hw/9pfs/9p-backend.c',
    meson.project_source_root() / 'hw/nfs/nfs2-xdr.c',
    meson.project_source_root() / 'net/onc-rpc-xdr.c',
    meson.project_source_root() / 'hw/nfs/nfs2-handle.c',
    meson.project_source_root() / 'hw/nfs/nfs2-server.c',
    crypto,
    testblock,
  ],
}

if host_os != 'windows'
  tests += {
    'test-plan9-auth': [
      meson.project_source_root() / 'hw/9pfs/plan9-auth.c',
      crypto,
      util,
    ],
  }
endif

if have_system or have_tools
  tests += {
    'test-qmp-event': [testqapi],
  }

  if seccomp.found()
    tests += {'test-seccomp': ['../../system/qemu-seccomp.c', seccomp]}
  endif
endif

if have_block
  tests += {
    'test-coroutine': [testblock],
    'test-aio': [testblock],
    'test-aio-multithread': [testblock],
    'test-throttle': [testblock],
    'test-thread-pool': [testblock],
    'test-hbitmap': [testblock],
    'test-bdrv-drain': [testblock],
    'test-bdrv-graph-mod': [testblock],
    'test-blockjob': [testblock],
    'test-blockjob-txn': [testblock],
    'test-block-backend': [testblock],
    'test-block-iothread': [testblock],
    'test-write-threshold': [testblock],
    'test-crypto-hash': [crypto],
    'test-crypto-hmac': [crypto],
    'test-crypto-cipher': [crypto],
    'test-crypto-akcipher': [crypto],
    'test-crypto-secret': [crypto, keyutils],
    'test-crypto-der': [crypto],
    'test-authz-simple': [authz],
    'test-authz-list': [authz],
    'test-authz-listfile': [authz],
    'test-io-task': [testblock],
    'test-io-channel-socket': ['socket-helpers.c', 'io-channel-helpers.c', io],
    'test-io-channel-file': ['io-channel-helpers.c', io],
    'test-io-channel-command': ['io-channel-helpers.c', io],
    'test-io-channel-buffer': ['io-channel-helpers.c', io],
    'test-io-channel-null': [io],
    'test-crypto-ivgen': [io],
    'test-crypto-afsplit': [io],
    'test-crypto-block': [io],
    'test-timed-average': [],
    'test-uuid': [],
  }
  if gnutls.found() and \
     tasn1.found() and \
     host_os != 'windows'
    tests += {
      'test-crypto-tlscredsx509': ['crypto-tls-x509-helpers.c',
                                   tasn1, crypto, gnutls],
      'test-crypto-tlssession': ['crypto-tls-x509-helpers.c', 'crypto-tls-psk-helpers.c',
                                 tasn1, crypto, gnutls],
      'test-io-channel-tls': ['io-channel-helpers.c', 'crypto-tls-x509-helpers.c',
                              tasn1, io, crypto, gnutls]}
  endif
  if pam.found()
    tests += {'test-authz-pam': [authz]}
  endif
  if host_os != 'windows'
    tests += {
      'test-image-locking': [testblock],
      'test-nested-aio-poll': [],
    }
  endif
  if config_host_data.get('CONFIG_REPLICATION')
    tests += {'test-replication': [testblock]}
  endif
  tests += {'test-crypto-pbkdf': [io]}
endif

if have_system and config_all_devices.has_key('CONFIG_NEXTCUBE')
  tests += {
    'test-next-nvram': [
      meson.project_source_root() / 'hw/nvram/next-nvram.c',
    ],
  }
endif

if have_system
  if have_virtfs
    plan9p1_local_test_sources = [
      meson.project_source_root() / 'hw/9pfs/9p-util-generic.c',
    ]
    if host_os == 'darwin'
      plan9p1_local_test_sources += meson.project_source_root() / 'hw/9pfs/9p-util-darwin.c'
    elif host_os == 'freebsd'
      plan9p1_local_test_sources += meson.project_source_root() / 'hw/9pfs/9p-util-freebsd.c'
    elif host_os == 'linux'
      plan9p1_local_test_sources += meson.project_source_root() / 'hw/9pfs/9p-util-linux.c'
    endif
    tests += {
      'test-9p-backend': [
        meson.project_source_root() / 'hw/9pfs/9p-backend.c',
      ],
      'test-plan9-9p1-server': [
        meson.project_source_root() / 'hw/core/reset.c',
        meson.project_source_root() / 'hw/9pfs/9p-backend.c',
        meson.project_source_root() / 'hw/9pfs/plan9-auth.c',
        meson.project_source_root() / 'hw/9pfs/plan9-9p1-codec.c',
        meson.project_source_root() / 'hw/9pfs/plan9-9p1-server.c',
        files('plan9-9p1-slirp-stub.c'),
        testblock,
        hwcore,
      ],
      'test-plan9-9p1-server-local': [
        meson.project_source_root() / 'hw/core/reset.c',
        meson.project_source_root() / 'hw/9pfs/9p-backend.c',
        meson.project_source_root() / 'hw/9pfs/plan9-auth.c',
        meson.project_source_root() / 'hw/9pfs/9p-local.c',
        meson.project_source_root() / 'hw/9pfs/9p-posix-acl.c',
        meson.project_source_root() / 'hw/9pfs/9p-xattr.c',
        meson.project_source_root() / 'hw/9pfs/9p-xattr-user.c',
        meson.project_source_root() / 'hw/9pfs/plan9-9p1-codec.c',
        meson.project_source_root() / 'hw/9pfs/plan9-9p1-server.c',
        files('plan9-9p1-slirp-stub.c'),
        plan9p1_local_test_sources,
        testblock,
        hwcore,
      ],
      'test-nfs2-server-local': [
        meson.project_source_root() / 'hw/9pfs/9p-backend.c',
        meson.project_source_root() / 'hw/9pfs/9p-local.c',
        meson.project_source_root() / 'hw/9pfs/9p-posix-acl.c',
        meson.project_source_root() / 'hw/9pfs/9p-xattr.c',
        meson.project_source_root() / 'hw/9pfs/9p-xattr-user.c',
        meson.project_source_root() / 'hw/nfs/nfs2-xdr.c',
        meson.project_source_root() / 'net/onc-rpc-xdr.c',
        meson.project_source_root() / 'hw/nfs/nfs2-handle.c',
        meson.project_source_root() / 'hw/nfs/nfs2-server.c',
        plan9p1_local_test_sources,
        crypto,
        testblock,
        hwcore,
      ],
    }
  endif

  if slirp.found()
  tests += {
    'test-slirp-stream': [
      meson.project_source_root() / 'net/slirp-stream.c',
    ],
    'test-slirp-udp': [
      meson.project_source_root() / 'net/slirp-bootp.c',
      meson.project_source_root() / 'net/slirp-udp.c',
    ],
    'test-slirp-plan9': [
      meson.project_source_root() / 'net/slirp-plan9.c',
    ],
    'test-slirp-guestfwd': [
      meson.project_source_root() / 'net/slirp-guestfwd.c',
    ],
    'test-slirp-il-integration': [
      meson.project_source_root() / 'net/slirp-bootp.c',
      meson.project_source_root() / 'net/slirp-il.c',
      meson.project_source_root() / 'net/slirp-guestfwd.c',
      meson.project_source_root() / 'net/slirp-plan9.c',
      meson.project_source_root() / 'net/slirp-stream.c',
      meson.project_source_root() / 'net/slirp-udp.c',
      slirp,
    ],
  }
  endif

  tests += {
    'test-slirp-il': [
      meson.project_source_root() / 'net/slirp-il.c',
    ],
  }

  tests += {
    'ptimer-test': ['ptimer-test-stubs.c', meson.project_source_root() / 'hw/core/ptimer.c'],
    'test-iov': [],
    'test-opts-visitor': [testqapi],
    'test-xs-node': [qom],
    'test-virtio-dmabuf': [meson.project_source_root() / 'hw/display/virtio-dmabuf.c'],
    'test-qmp-cmds': [testqapi],
    'test-xbzrle': [migration],
    'test-util-sockets': ['socket-helpers.c'],
    'test-base64': [],
    'test-bufferiszero': [],
    'test-smp-parse': [qom, meson.project_source_root() / 'hw/core/machine-smp.c'],
    'test-vmstate': [migration, io],
    'test-bt463': [
      migration,
      io,
      meson.project_source_root() / 'hw/display/bt463.c',
    ],
    'test-yank': ['socket-helpers.c', qom, io, chardev],
    'test-ram-discard-manager': [
      'test-ram-discard-manager.c',
      'test-ram-discard-manager-stubs.c',
      meson.project_source_root() / 'system/ram-discard-manager.c',
      genh, qemuutil, qom
    ],
  }
  if config_host_data.get('CONFIG_INOTIFY1')
    tests += {'test-util-filemonitor': []}
  endif

  # Some tests: test-char, test-qdev-global-props, and test-qga,
  # are not runnable under TSan due to a known issue.
  # https://github.com/google/sanitizers/issues/1116
  if not get_option('tsan')
    if host_os != 'windows'
        tests += {
          'test-char': ['socket-helpers.c', qom, io, chardev]
        }
    endif

    tests += {
      'test-qdev-global-props': [qom, hwcore]
    }
  endif
  tests += {'test-qdev': [qom, hwcore]}
endif

if have_ga and host_os == 'linux'
  tests += {'test-qga': ['../qtest/libqmp.c', '../qtest/libqtest.c']}
  test_deps += {'test-qga': qga}
endif

if host_os != 'windows'
  test_deps += {'test-plan9-auth': qemu_plan9_keydb_test}
endif

test_env = environment()
test_env.set('G_TEST_SRCDIR', meson.current_source_dir())
test_env.set('G_TEST_BUILDDIR', meson.current_build_dir())
if host_os != 'windows'
  test_env.set('QEMU_PLAN9_KEYDB', qemu_plan9_keydb_test.full_path())
endif

slow_tests = {
  'test-aio-multithread' : 120,
  'test-bufferiszero': 60,
  'test-crypto-block' : 300,
  'test-crypto-tlscredsx509': 90,
  'test-crypto-tlssession': 90,
  'test-replication': 60,
}

foreach test_name, extra: tests
  src = [test_name + '.c']
  deps = [qemuutil]
  if extra.length() > 0
    # use a sourceset to quickly separate sources and deps
    test_ss = ss.source_set()
    test_ss.add(extra)
    src += test_ss.all_sources()
    deps += test_ss.all_dependencies()
  endif
  if test_name == 'test-slirp-il-integration'
    exe = executable(test_name, src, genh, dependencies: deps,
                     c_args: ['-ffunction-sections'],
                     link_args: ['-Wl,--gc-sections'])
  else
    exe = executable(test_name, src, genh, dependencies: deps)
  endif

  test(test_name, exe,
       depends: test_deps.get(test_name, []),
       env: test_env,
       args: ['--tap', '-k'],
       protocol: 'tap',
       timeout: slow_tests.get(test_name, 30),
       priority: slow_tests.get(test_name, 30),
       suite: ['unit'])
endforeach

if have_system
  nfs2_handle_nm = find_program('nm')
  nfs2_handle_production_check = static_library(
    'nfs2-handle-production-check',
    meson.project_source_root() / 'hw/nfs/nfs2-handle.c',
    genh,
    build_by_default: false,
    dependencies: [glib],
  )
  test('test-nfs2-handle-production-symbols', python,
       args: [files('test-nfs2-handle-production-symbols.py'),
              nfs2_handle_nm.full_path(), nfs2_handle_production_check],
       depends: nfs2_handle_production_check,
       suite: ['unit'])
endif
