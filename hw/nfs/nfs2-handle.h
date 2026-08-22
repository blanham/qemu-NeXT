#ifndef HW_NFS_NFS2_HANDLE_H
#define HW_NFS_NFS2_HANDLE_H

#include "fsdev/file-op-9p.h"
#include "hw/nfs/nfs2-protocol.h"

typedef struct Nfs2HandleTable Nfs2HandleTable;

Nfs2HandleTable *nfs2_handle_table_new(const uint8_t *key, size_t key_length,
                                       Error **errp);
void nfs2_handle_table_clear(Nfs2HandleTable *table);
void nfs2_handle_table_free(Nfs2HandleTable *table);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(Nfs2HandleTable, nfs2_handle_table_free)

bool nfs2_handle_create(Nfs2HandleTable *table, uint64_t id,
                        const char *path, Nfs2FileHandle *handle,
                        Error **errp);
bool nfs2_handle_resolve(Nfs2HandleTable *table,
                         const Nfs2FileHandle *handle, V9fsPath *path);
bool nfs2_handle_rename(Nfs2HandleTable *table, const char *old_path,
                        const char *new_path, Error **errp);
bool nfs2_handle_remove(Nfs2HandleTable *table, const char *path,
                        Error **errp);
size_t nfs2_handle_table_record_count(const Nfs2HandleTable *table);

#endif
