qemu_plan9_keydb = executable(
  'qemu-plan9-keydb',
  files('qemu-plan9-keydb.c') +
    meson.project_source_root() / 'hw/9pfs/plan9-auth.c',
  dependencies: [crypto, qemuutil],
  install: have_tools,
)
