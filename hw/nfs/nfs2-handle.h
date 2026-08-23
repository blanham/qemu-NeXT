#ifndef HW_NFS_NFS2_HANDLE_H
#define HW_NFS_NFS2_HANDLE_H

#include "fsdev/file-op-9p.h"
#include "hw/nfs/nfs2-protocol.h"

typedef struct Nfs2HandleTable Nfs2HandleTable;
typedef struct Nfs2HandleAliasReservation Nfs2HandleAliasReservation;

typedef struct Nfs2HandlePathState {
    uint64_t id;
    uint32_t generation;
    bool present;
} Nfs2HandlePathState;

Nfs2HandleTable *nfs2_handle_table_new(const uint8_t *key, size_t key_length,
                                       Error **errp);
void nfs2_handle_table_clear(Nfs2HandleTable *table);
void nfs2_handle_table_free(Nfs2HandleTable *table);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(Nfs2HandleTable, nfs2_handle_table_free)

bool nfs2_handle_create(Nfs2HandleTable *table, uint64_t id,
                        const char *path, Nfs2FileHandle *handle,
                        Error **errp);
bool nfs2_handle_alias_reserve(Nfs2HandleTable *table,
                               const Nfs2FileHandle *handle,
                               const char *path,
                               Nfs2HandleAliasReservation **reservation,
                               Error **errp);
void nfs2_handle_alias_cancel(Nfs2HandleAliasReservation *reservation);
bool nfs2_handle_alias_commit(Nfs2HandleAliasReservation *reservation);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(Nfs2HandleAliasReservation,
                              nfs2_handle_alias_cancel)

bool nfs2_handle_resolve(Nfs2HandleTable *table,
                         const Nfs2FileHandle *handle, V9fsPath *path);
GPtrArray *nfs2_handle_paths_snapshot(Nfs2HandleTable *table,
                                      const Nfs2FileHandle *handle);
void nfs2_handle_path_state(Nfs2HandleTable *table, const char *path,
                            Nfs2HandlePathState *state);
bool nfs2_handle_path_state_allows(Nfs2HandleTable *table, const char *path,
                                   const Nfs2HandlePathState *state,
                                   uint64_t new_id);
bool nfs2_handle_rename(Nfs2HandleTable *table, const char *old_path,
                        const char *new_path, Error **errp);
bool nfs2_handle_remove(Nfs2HandleTable *table, const char *path,
                        Error **errp);
size_t nfs2_handle_table_record_count(const Nfs2HandleTable *table);

#ifdef NFS2_HANDLE_TESTING
/* Test-only fault/state injection. */
void nfs2_handle_table_set_next_generation_for_test(Nfs2HandleTable *table,
                                                     uint32_t generation);
void nfs2_handle_table_fail_next_mac_for_test(Nfs2HandleTable *table);
#endif

#endif
