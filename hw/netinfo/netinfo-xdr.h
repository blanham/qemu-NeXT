/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_NETINFO_NETINFO_XDR_H
#define HW_NETINFO_NETINFO_XDR_H

#include "net/onc-rpc.h"
#include "hw/netinfo/netinfo-protocol.h"

/* Every clear function accepts an initialized object and is idempotent. */
void ni_id_init(NiId *id);
void ni_id_clear(NiId *id);
void ni_name_init(NiName *name);
void ni_name_clear(NiName *name);
void ni_name_list_init(NiNameList *list);
void ni_name_list_clear(NiNameList *list);
void ni_property_init(NiProperty *property);
void ni_property_clear(NiProperty *property);
void ni_property_list_init(NiPropertyList *list);
void ni_property_list_clear(NiPropertyList *list);
void ni_id_list_init(NiIdList *list);
void ni_id_list_clear(NiIdList *list);
void ni_object_init(NiObject *object);
void ni_object_clear(NiObject *object);
void ni_entry_init(NiEntry *entry);
void ni_entry_clear(NiEntry *entry);
void ni_entry_list_init(NiEntryList *list);
void ni_entry_list_clear(NiEntryList *list);
void ni_bind_addr_info_init(NiBindAddrInfo *info);
void ni_bind_addr_info_clear(NiBindAddrInfo *info);
void ni_bind_registration_init(NiBindRegistration *registration);
void ni_bind_registration_clear(NiBindRegistration *registration);
void ni_bind_clone_args_init(NiBindCloneArgs *args);
void ni_bind_clone_args_clear(NiBindCloneArgs *args);
void ni_bind_args_init(NiBindArgs *args);
void ni_bind_args_clear(NiBindArgs *args);
void ni_bind_getregister_result_init(NiBindGetRegisterResult *result);
void ni_bind_getregister_result_clear(NiBindGetRegisterResult *result);
void ni_bind_listreg_result_init(NiBindListRegResult *result);
void ni_bind_listreg_result_clear(NiBindListRegResult *result);
void ni_id_result_init(NiIdResult *result);
void ni_id_result_clear(NiIdResult *result);
void ni_parent_stuff_init(NiParentStuff *stuff);
void ni_parent_stuff_clear(NiParentStuff *stuff);
void ni_parent_result_init(NiParentResult *result);
void ni_parent_result_clear(NiParentResult *result);
void ni_children_stuff_init(NiChildrenStuff *stuff);
void ni_children_stuff_clear(NiChildrenStuff *stuff);
void ni_children_result_init(NiChildrenResult *result);
void ni_children_result_clear(NiChildrenResult *result);
void ni_entry_stuff_init(NiEntryStuff *stuff);
void ni_entry_stuff_clear(NiEntryStuff *stuff);
void ni_list_result_init(NiListResult *result);
void ni_list_result_clear(NiListResult *result);
void ni_property_list_stuff_init(NiPropertyListStuff *stuff);
void ni_property_list_stuff_clear(NiPropertyListStuff *stuff);
void ni_create_args_init(NiCreateArgs *args);
void ni_create_args_clear(NiCreateArgs *args);
void ni_property_list_result_init(NiPropertyListResult *result);
void ni_property_list_result_clear(NiPropertyListResult *result);
void ni_create_stuff_init(NiCreateStuff *stuff);
void ni_create_stuff_clear(NiCreateStuff *stuff);
void ni_create_result_init(NiCreateResult *result);
void ni_create_result_clear(NiCreateResult *result);
void ni_destroy_args_init(NiDestroyArgs *args);
void ni_destroy_args_clear(NiDestroyArgs *args);
void ni_lookup_args_init(NiLookupArgs *args);
void ni_lookup_args_clear(NiLookupArgs *args);
void ni_lookup_stuff_init(NiLookupStuff *stuff);
void ni_lookup_stuff_clear(NiLookupStuff *stuff);
void ni_lookup_result_init(NiLookupResult *result);
void ni_lookup_result_clear(NiLookupResult *result);
void ni_name_args_init(NiNameArgs *args);
void ni_name_args_clear(NiNameArgs *args);
void ni_create_prop_args_init(NiCreatePropArgs *args);
void ni_create_prop_args_clear(NiCreatePropArgs *args);
void ni_write_prop_args_init(NiWritePropArgs *args);
void ni_write_prop_args_clear(NiWritePropArgs *args);
void ni_prop_args_init(NiPropArgs *args);
void ni_prop_args_clear(NiPropArgs *args);
void ni_name_list_stuff_init(NiNameListStuff *stuff);
void ni_name_list_stuff_clear(NiNameListStuff *stuff);
void ni_name_list_result_init(NiNameListResult *result);
void ni_name_list_result_clear(NiNameListResult *result);
void ni_prop_name_args_init(NiPropNameArgs *args);
void ni_prop_name_args_clear(NiPropNameArgs *args);
void ni_create_name_args_init(NiCreateNameArgs *args);
void ni_create_name_args_clear(NiCreateNameArgs *args);
void ni_name_index_args_init(NiNameIndexArgs *args);
void ni_name_index_args_clear(NiNameIndexArgs *args);
void ni_write_name_args_init(NiWriteNameArgs *args);
void ni_write_name_args_clear(NiWriteNameArgs *args);
void ni_read_name_stuff_init(NiReadNameStuff *stuff);
void ni_read_name_stuff_clear(NiReadNameStuff *stuff);
void ni_read_name_result_init(NiReadNameResult *result);
void ni_read_name_result_clear(NiReadNameResult *result);
void ni_binding_init(NiBinding *binding);
void ni_binding_clear(NiBinding *binding);
void ni_rparent_result_init(NiRParentResult *result);
void ni_rparent_result_clear(NiRParentResult *result);
void ni_object_node_init(NiObjectNode *node);
/* Clears the embedded object only; the caller retains ownership of node. */
void ni_object_node_clear(NiObjectNode *node);
void ni_object_list_init(NiObjectList *list);
void ni_object_list_clear(NiObjectList *list);
void ni_read_all_stuff_init(NiReadAllStuff *stuff);
void ni_read_all_stuff_clear(NiReadAllStuff *stuff);
void ni_read_all_result_init(NiReadAllResult *result);
void ni_read_all_result_clear(NiReadAllResult *result);
void ni_property_list_array_init(NiPropertyListArray *array);
void ni_property_list_array_clear(NiPropertyListArray *array);
void ni_list_all_stuff_init(NiListAllStuff *stuff);
void ni_list_all_stuff_clear(NiListAllStuff *stuff);
void ni_list_all_result_init(NiListAllResult *result);
void ni_list_all_result_clear(NiListAllResult *result);

/*
 * Reader/writer helpers.  Top-level decoders require reader exhaustion.
 * Destinations with a matching _init helper must be initialized before the
 * first decode or replacement.  Decoders
 * are transactional: malformed or trailing input leaves both reader and
 * destination unchanged; successful input clears the old destination and
 * replaces it with the decoded value.
 */
bool ni_xdr_encode_status(OncRpcXdrWriter *writer, NiStatus status);
bool ni_xdr_decode_status(OncRpcXdrReader *reader, NiStatus *status);
bool ni_xdr_encode_id(OncRpcXdrWriter *writer, const NiId *id);
bool ni_xdr_decode_id(OncRpcXdrReader *reader, NiId *id);
bool ni_xdr_encode_name(OncRpcXdrWriter *writer, const char *name);
bool ni_xdr_decode_name(OncRpcXdrReader *reader, NiName *name);
bool ni_xdr_encode_name_list(OncRpcXdrWriter *writer,
                             const NiNameList *list);
bool ni_xdr_decode_name_list(OncRpcXdrReader *reader, NiNameList *list);
bool ni_xdr_encode_property(OncRpcXdrWriter *writer,
                            const NiProperty *property);
bool ni_xdr_decode_property(OncRpcXdrReader *reader, NiProperty *property);
bool ni_xdr_encode_property_list(OncRpcXdrWriter *writer,
                                 const NiPropertyList *list);
bool ni_xdr_decode_property_list(OncRpcXdrReader *reader,
                                 NiPropertyList *list);
bool ni_xdr_encode_id_list(OncRpcXdrWriter *writer, const NiIdList *list);
bool ni_xdr_decode_id_list(OncRpcXdrReader *reader, NiIdList *list);
bool ni_xdr_encode_optional_id(OncRpcXdrWriter *writer, bool present,
                               const NiId *id);
bool ni_xdr_decode_optional_id(OncRpcXdrReader *reader, NiId *id,
                               bool *present);

bool ni_xdr_encode_bind_addr_info(OncRpcXdrWriter *writer,
                                  const NiBindAddrInfo *info);
bool ni_xdr_decode_bind_addr_info(OncRpcXdrReader *reader,
                                  NiBindAddrInfo *info);
bool ni_xdr_encode_bind_registration(OncRpcXdrWriter *writer,
                                     const NiBindRegistration *registration);
bool ni_xdr_decode_bind_registration(OncRpcXdrReader *reader,
                                     NiBindRegistration *registration);
bool ni_xdr_encode_bind_clone_args(OncRpcXdrWriter *writer,
                                   const NiBindCloneArgs *args);
bool ni_xdr_decode_bind_clone_args(OncRpcXdrReader *reader,
                                   NiBindCloneArgs *args);
bool ni_xdr_encode_bind_args(OncRpcXdrWriter *writer, const NiBindArgs *args);
bool ni_xdr_decode_bind_args(OncRpcXdrReader *reader, NiBindArgs *args);
bool ni_xdr_encode_bind_getregister_result(
    OncRpcXdrWriter *writer, const NiBindGetRegisterResult *result);
bool ni_xdr_decode_bind_getregister_result(
    OncRpcXdrReader *reader, NiBindGetRegisterResult *result);
bool ni_xdr_encode_bind_listreg_result(OncRpcXdrWriter *writer,
                                        const NiBindListRegResult *result);
bool ni_xdr_decode_bind_listreg_result(OncRpcXdrReader *reader,
                                       NiBindListRegResult *result);

bool ni_xdr_encode_id_result(OncRpcXdrWriter *writer,
                             const NiIdResult *result);
bool ni_xdr_decode_id_result(OncRpcXdrReader *reader, NiIdResult *result);
bool ni_xdr_encode_parent_result(OncRpcXdrWriter *writer,
                                 const NiParentResult *result);
bool ni_xdr_decode_parent_result(OncRpcXdrReader *reader,
                                 NiParentResult *result);
bool ni_xdr_encode_children_result(OncRpcXdrWriter *writer,
                                   const NiChildrenResult *result);
bool ni_xdr_decode_children_result(OncRpcXdrReader *reader,
                                   NiChildrenResult *result);
bool ni_xdr_encode_list_result(OncRpcXdrWriter *writer,
                               const NiListResult *result);
bool ni_xdr_decode_list_result(OncRpcXdrReader *reader, NiListResult *result);
bool ni_xdr_encode_property_list_stuff(OncRpcXdrWriter *writer,
                                       const NiPropertyListStuff *stuff);
bool ni_xdr_decode_property_list_stuff(OncRpcXdrReader *reader,
                                       NiPropertyListStuff *stuff);
bool ni_xdr_encode_create_args(OncRpcXdrWriter *writer,
                               const NiCreateArgs *args);
bool ni_xdr_decode_create_args(OncRpcXdrReader *reader, NiCreateArgs *args);
bool ni_xdr_encode_create_stuff(OncRpcXdrWriter *writer,
                                const NiCreateStuff *stuff);
bool ni_xdr_decode_create_stuff(OncRpcXdrReader *reader,
                                NiCreateStuff *stuff);
bool ni_xdr_encode_property_list_result(
    OncRpcXdrWriter *writer, const NiPropertyListResult *result);
bool ni_xdr_decode_property_list_result(OncRpcXdrReader *reader,
                                        NiPropertyListResult *result);
bool ni_xdr_encode_object(OncRpcXdrWriter *writer, const NiObject *object);
bool ni_xdr_decode_object(OncRpcXdrReader *reader, NiObject *object);
bool ni_xdr_encode_entry(OncRpcXdrWriter *writer, const NiEntry *entry);
bool ni_xdr_decode_entry(OncRpcXdrReader *reader, NiEntry *entry);
bool ni_xdr_encode_entry_list(OncRpcXdrWriter *writer,
                              const NiEntryList *list);
bool ni_xdr_decode_entry_list(OncRpcXdrReader *reader, NiEntryList *list);
bool ni_xdr_encode_parent_stuff(OncRpcXdrWriter *writer,
                                const NiParentStuff *stuff);
bool ni_xdr_decode_parent_stuff(OncRpcXdrReader *reader,
                                NiParentStuff *stuff);
bool ni_xdr_encode_children_stuff(OncRpcXdrWriter *writer,
                                  const NiChildrenStuff *stuff);
bool ni_xdr_decode_children_stuff(OncRpcXdrReader *reader,
                                  NiChildrenStuff *stuff);
bool ni_xdr_encode_entry_stuff(OncRpcXdrWriter *writer,
                               const NiEntryStuff *stuff);
bool ni_xdr_decode_entry_stuff(OncRpcXdrReader *reader, NiEntryStuff *stuff);
bool ni_xdr_encode_lookup_stuff(OncRpcXdrWriter *writer,
                                const NiLookupStuff *stuff);
bool ni_xdr_decode_lookup_stuff(OncRpcXdrReader *reader,
                                NiLookupStuff *stuff);
bool ni_xdr_encode_name_list_stuff(OncRpcXdrWriter *writer,
                                   const NiNameListStuff *stuff);
bool ni_xdr_decode_name_list_stuff(OncRpcXdrReader *reader,
                                   NiNameListStuff *stuff);
bool ni_xdr_encode_read_name_stuff(OncRpcXdrWriter *writer,
                                   const NiReadNameStuff *stuff);
bool ni_xdr_decode_read_name_stuff(OncRpcXdrReader *reader,
                                   NiReadNameStuff *stuff);
bool ni_xdr_encode_binding(OncRpcXdrWriter *writer,
                           const NiBinding *binding);
bool ni_xdr_decode_binding(OncRpcXdrReader *reader, NiBinding *binding);
bool ni_xdr_encode_list_all_stuff(OncRpcXdrWriter *writer,
                                  const NiListAllStuff *stuff);
bool ni_xdr_decode_list_all_stuff(OncRpcXdrReader *reader,
                                  NiListAllStuff *stuff);
bool ni_xdr_encode_create_result(OncRpcXdrWriter *writer,
                                 const NiCreateResult *result);
bool ni_xdr_decode_create_result(OncRpcXdrReader *reader,
                                 NiCreateResult *result);
bool ni_xdr_encode_destroy_args(OncRpcXdrWriter *writer,
                                const NiDestroyArgs *args);
bool ni_xdr_decode_destroy_args(OncRpcXdrReader *reader,
                                NiDestroyArgs *args);
bool ni_xdr_encode_lookup_args(OncRpcXdrWriter *writer,
                               const NiLookupArgs *args);
bool ni_xdr_decode_lookup_args(OncRpcXdrReader *reader,
                               NiLookupArgs *args);
bool ni_xdr_encode_lookup_result(OncRpcXdrWriter *writer,
                                 const NiLookupResult *result);
bool ni_xdr_decode_lookup_result(OncRpcXdrReader *reader,
                                 NiLookupResult *result);
bool ni_xdr_encode_name_args(OncRpcXdrWriter *writer, const NiNameArgs *args);
bool ni_xdr_decode_name_args(OncRpcXdrReader *reader, NiNameArgs *args);
bool ni_xdr_encode_create_prop_args(OncRpcXdrWriter *writer,
                                    const NiCreatePropArgs *args);
bool ni_xdr_decode_create_prop_args(OncRpcXdrReader *reader,
                                    NiCreatePropArgs *args);
bool ni_xdr_encode_write_prop_args(OncRpcXdrWriter *writer,
                                   const NiWritePropArgs *args);
bool ni_xdr_decode_write_prop_args(OncRpcXdrReader *reader,
                                   NiWritePropArgs *args);
bool ni_xdr_encode_prop_args(OncRpcXdrWriter *writer, const NiPropArgs *args);
bool ni_xdr_decode_prop_args(OncRpcXdrReader *reader, NiPropArgs *args);
bool ni_xdr_encode_name_list_result(OncRpcXdrWriter *writer,
                                    const NiNameListResult *result);
bool ni_xdr_decode_name_list_result(OncRpcXdrReader *reader,
                                    NiNameListResult *result);
bool ni_xdr_encode_prop_name_args(OncRpcXdrWriter *writer,
                                  const NiPropNameArgs *args);
bool ni_xdr_decode_prop_name_args(OncRpcXdrReader *reader,
                                  NiPropNameArgs *args);
bool ni_xdr_encode_create_name_args(OncRpcXdrWriter *writer,
                                    const NiCreateNameArgs *args);
bool ni_xdr_decode_create_name_args(OncRpcXdrReader *reader,
                                    NiCreateNameArgs *args);
bool ni_xdr_encode_name_index_args(OncRpcXdrWriter *writer,
                                   const NiNameIndexArgs *args);
bool ni_xdr_decode_name_index_args(OncRpcXdrReader *reader,
                                   NiNameIndexArgs *args);
bool ni_xdr_encode_write_name_args(OncRpcXdrWriter *writer,
                                   const NiWriteNameArgs *args);
bool ni_xdr_decode_write_name_args(OncRpcXdrReader *reader,
                                   NiWriteNameArgs *args);
bool ni_xdr_encode_read_name_result(OncRpcXdrWriter *writer,
                                    const NiReadNameResult *result);
bool ni_xdr_decode_read_name_result(OncRpcXdrReader *reader,
                                    NiReadNameResult *result);
bool ni_xdr_encode_rparent_result(OncRpcXdrWriter *writer,
                                  const NiRParentResult *result);
bool ni_xdr_decode_rparent_result(OncRpcXdrReader *reader,
                                  NiRParentResult *result);
bool ni_xdr_encode_object_list(OncRpcXdrWriter *writer,
                               const NiObjectList *list);
bool ni_xdr_decode_object_list(OncRpcXdrReader *reader, NiObjectList *list);
bool ni_xdr_encode_read_all_result(OncRpcXdrWriter *writer,
                                   const NiReadAllResult *result);
bool ni_xdr_decode_read_all_result(OncRpcXdrReader *reader,
                                   NiReadAllResult *result);
bool ni_xdr_encode_read_all_stuff(OncRpcXdrWriter *writer,
                                  const NiReadAllStuff *stuff);
bool ni_xdr_decode_read_all_stuff(OncRpcXdrReader *reader,
                                  NiReadAllStuff *stuff);
bool ni_xdr_encode_property_list_array(OncRpcXdrWriter *writer,
                                       const NiPropertyListArray *array);
bool ni_xdr_decode_property_list_array(OncRpcXdrReader *reader,
                                       NiPropertyListArray *array);
bool ni_xdr_encode_list_all_result(OncRpcXdrWriter *writer,
                                   const NiListAllResult *result);
bool ni_xdr_decode_list_all_result(OncRpcXdrReader *reader,
                                   NiListAllResult *result);

#endif /* HW_NETINFO_NETINFO_XDR_H */
