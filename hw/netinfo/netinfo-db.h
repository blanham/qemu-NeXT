/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_NETINFO_NETINFO_DB_H
#define HW_NETINFO_NETINFO_DB_H

#include <glib.h>

#include "hw/netinfo/netinfo-protocol.h"

typedef struct NetInfoDb NetInfoDb;

/* A parent-less node is the unique database root. */
typedef struct NetInfoDbNodeSpec {
    NiId id;
    bool has_parent;
    NiIndex parent;
    const NiPropertyList *properties;
} NetInfoDbNodeSpec;

NetInfoDb *netinfo_db_new(void);
NetInfoDb *netinfo_db_new_with_tag(const char *tag);
NetInfoDb *netinfo_db_new_default(void);
NetInfoDb *netinfo_db_default(void);
void netinfo_db_clear(NetInfoDb *db);
void netinfo_db_free(NetInfoDb *db);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(NetInfoDb, netinfo_db_free)

const char *netinfo_db_tag(const NetInfoDb *db);
size_t netinfo_db_node_count(const NetInfoDb *db);

/* Add all nodes before sealing; the source property list is deep-copied. */
NiStatus netinfo_db_add_node(NetInfoDb *db, const NiId *id,
                             bool has_parent, NiIndex parent,
                             const NiPropertyList *properties);
NiStatus netinfo_db_add(NetInfoDb *db, const NetInfoDbNodeSpec *spec);
NiStatus netinfo_db_seal(NetInfoDb *db);
NiStatus netinfo_db_finish(NetInfoDb *db);

/*
 * Initialize allocation-owning output values (NiIdList, NiNameList,
 * NiPropertyList, NiEntryList, and NiName) with their matching Task 2
 * ni_*_init helper before first use.  Successful operations replace existing
 * output contents and refresh input IDs to their canonical instances;
 * failures leave output values and input IDs unchanged.
 */
NiStatus netinfo_db_root(NetInfoDb *db, NiId *id);
NiStatus netinfo_db_self(NetInfoDb *db, NiId *id);
NiStatus netinfo_db_parent(NetInfoDb *db, NiId *id, NiId *parent);
NiStatus netinfo_db_parent_index(NetInfoDb *db, NiId *id, NiIndex *parent);
NiStatus netinfo_db_children(NetInfoDb *db, NiId *id, NiIdList *children);
NiStatus netinfo_db_read(NetInfoDb *db, NiId *id, NiPropertyList *properties);
NiStatus netinfo_db_lookup(NetInfoDb *db, NiId *id, const char *key,
                           const char *value, NiIdList *found);
NiStatus netinfo_db_list(NetInfoDb *db, NiId *id, const char *property,
                         NiEntryList *entries);
NiStatus netinfo_db_readprop(NetInfoDb *db, NiId *id, NiIndex property,
                             NiNameList *values);
NiStatus netinfo_db_listprops(NetInfoDb *db, NiId *id, NiNameList *names);
NiStatus netinfo_db_readname(NetInfoDb *db, NiId *id, NiIndex property,
                             NiIndex value, NiName *name);
NiStatus netinfo_db_lookup_read(NetInfoDb *db, NiId *id, const char *key,
                                const char *value, NiId *found,
                                NiPropertyList *properties);

/* Spelled-out aliases are convenient at the RPC boundary. */
NiStatus netinfo_db_read_prop(NetInfoDb *db, NiId *id, NiIndex property,
                              NiNameList *values);
NiStatus netinfo_db_list_props(NetInfoDb *db, NiId *id, NiNameList *names);
NiStatus netinfo_db_read_name(NetInfoDb *db, NiId *id, NiIndex property,
                              NiIndex value, NiName *name);

#endif /* HW_NETINFO_NETINFO_DB_H */
