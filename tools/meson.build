if host_os != 'windows'
  qemu_plan9_keydb_sources = files('qemu-plan9-keydb.c') + [
    meson.project_source_root() / 'hw/9pfs/plan9-auth.c',
  ]
  qemu_plan9_keydb = executable(
    'qemu-plan9-keydb',
    qemu_plan9_keydb_sources,
    dependencies: [crypto, qemuutil],
    install: have_tools,
  )
  qemu_plan9_keydb_test = executable(
    'qemu-plan9-keydb-test',
    qemu_plan9_keydb_sources,
    c_args: '-DPLAN9_KEYDB_TESTING',
    dependencies: [crypto, qemuutil],
  )
endif
