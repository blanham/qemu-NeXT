/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Clean-room NetInfo XDR codecs.
 *
 * The fixed wire declarations are published in the pinned NeXT sources:
 * https://github.com/johnsonjh/NeXTSrc/blob/ff846608a76ab2fbbb86e8a14c52ac85332f9786/libc-34.1/usr.include/netinfo/ni_prot.x
 * https://github.com/johnsonjh/NeXTSrc/blob/ff846608a76ab2fbbb86e8a14c52ac85332f9786/libc-34.1/usr.include/netinfo/nibind_prot.x
 * (SHA256 ni_prot.x:
 * 8dc940ec6da326fb5a949c53c8d7957be9dee3bc0ae5be9b13c22175532c77ce;
 * SHA256 nibind_prot.x:
 * 1b489f05f9842cc390e28321f93f4eb75c1d6d99a8e7cf98ab5b5882da583c1a).
 * They are cross-checked against Apple's Libinfo-221 sources:
 * https://github.com/apple-oss-distributions/Libinfo/blob/Libinfo-221/netinfo.subproj/ni_prot.x
 * https://github.com/apple-oss-distributions/Libinfo/blob/Libinfo-221/netinfo.subproj/nibind_prot.x
 *
 * Encoding rules are those of RFC 1831/RFC 1832:
 * https://www.rfc-editor.org/rfc/rfc1831.html
 * https://www.rfc-editor.org/rfc/rfc1832.html
 * This implementation is deliberately independent of rpcgen output and of
 * Apple's implementation.
 */
#include "qemu/osdep.h"

#include "hw/netinfo/netinfo-xdr.h"

static void reset_value(void *value, size_t size)
{
    if (value) {
        memset(value, 0, size);
    }
}

void ni_id_init(NiId *id)
{
    reset_value(id, sizeof(*id));
}

void ni_id_clear(NiId *id)
{
    reset_value(id, sizeof(*id));
}

void ni_name_init(NiName *name)
{
    if (name) {
        *name = NULL;
    }
}

void ni_name_clear(NiName *name)
{
    if (name) {
        g_free(*name);
        *name = NULL;
    }
}

void ni_name_list_init(NiNameList *list)
{
    reset_value(list, sizeof(*list));
}

void ni_name_list_clear(NiNameList *list)
{
    if (!list) {
        return;
    }
    for (size_t i = 0; i < list->count; i++) {
        ni_name_clear(&list->values[i]);
    }
    g_free(list->values);
    reset_value(list, sizeof(*list));
}

void ni_property_init(NiProperty *property)
{
    reset_value(property, sizeof(*property));
}

void ni_property_clear(NiProperty *property)
{
    if (!property) {
        return;
    }
    ni_name_clear(&property->name);
    ni_name_list_clear(&property->values);
    reset_value(property, sizeof(*property));
}

void ni_property_list_init(NiPropertyList *list)
{
    reset_value(list, sizeof(*list));
}

void ni_property_list_clear(NiPropertyList *list)
{
    if (!list) {
        return;
    }
    for (size_t i = 0; i < list->count; i++) {
        ni_property_clear(&list->properties[i]);
    }
    g_free(list->properties);
    reset_value(list, sizeof(*list));
}

void ni_id_list_init(NiIdList *list)
{
    reset_value(list, sizeof(*list));
}

void ni_id_list_clear(NiIdList *list)
{
    if (!list) {
        return;
    }
    g_free(list->values);
    reset_value(list, sizeof(*list));
}

void ni_object_init(NiObject *object)
{
    reset_value(object, sizeof(*object));
}

void ni_object_clear(NiObject *object)
{
    if (!object) {
        return;
    }
    ni_property_list_clear(&object->properties);
    ni_id_list_clear(&object->children);
    reset_value(object, sizeof(*object));
}

void ni_entry_init(NiEntry *entry)
{
    reset_value(entry, sizeof(*entry));
}

void ni_entry_clear(NiEntry *entry)
{
    if (!entry) {
        return;
    }
    ni_name_list_clear(&entry->names);
    reset_value(entry, sizeof(*entry));
}

void ni_entry_list_init(NiEntryList *list)
{
    reset_value(list, sizeof(*list));
}

void ni_entry_list_clear(NiEntryList *list)
{
    if (!list) {
        return;
    }
    for (size_t i = 0; i < list->count; i++) {
        ni_entry_clear(&list->entries[i]);
    }
    g_free(list->entries);
    reset_value(list, sizeof(*list));
}

void ni_bind_addr_info_init(NiBindAddrInfo *info)
{
    reset_value(info, sizeof(*info));
}

void ni_bind_addr_info_clear(NiBindAddrInfo *info)
{
    reset_value(info, sizeof(*info));
}

void ni_bind_registration_init(NiBindRegistration *registration)
{
    reset_value(registration, sizeof(*registration));
}

void ni_bind_registration_clear(NiBindRegistration *registration)
{
    if (!registration) {
        return;
    }
    ni_name_clear(&registration->tag);
    reset_value(registration, sizeof(*registration));
}

void ni_bind_clone_args_init(NiBindCloneArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_bind_clone_args_clear(NiBindCloneArgs *args)
{
    if (!args) {
        return;
    }
    ni_name_clear(&args->tag);
    ni_name_clear(&args->master_name);
    ni_name_clear(&args->master_tag);
    reset_value(args, sizeof(*args));
}

void ni_bind_args_init(NiBindArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_bind_args_clear(NiBindArgs *args)
{
    if (!args) {
        return;
    }
    ni_name_clear(&args->client_tag);
    ni_name_clear(&args->server_tag);
    reset_value(args, sizeof(*args));
}

void ni_bind_getregister_result_init(NiBindGetRegisterResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_bind_getregister_result_clear(NiBindGetRegisterResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_bind_listreg_result_init(NiBindListRegResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_bind_listreg_result_clear(NiBindListRegResult *result)
{
    if (!result) {
        return;
    }
    for (size_t i = 0; i < result->count; i++) {
        ni_bind_registration_clear(&result->registrations[i]);
    }
    g_free(result->registrations);
    reset_value(result, sizeof(*result));
}

void ni_id_result_init(NiIdResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_id_result_clear(NiIdResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_parent_stuff_init(NiParentStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_parent_stuff_clear(NiParentStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_parent_result_init(NiParentResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_parent_result_clear(NiParentResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_children_stuff_init(NiChildrenStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_children_stuff_clear(NiChildrenStuff *stuff)
{
    if (stuff) {
        ni_id_list_clear(&stuff->children);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_children_result_init(NiChildrenResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_children_result_clear(NiChildrenResult *result)
{
    if (result) {
        ni_children_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_entry_stuff_init(NiEntryStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_entry_stuff_clear(NiEntryStuff *stuff)
{
    if (stuff) {
        ni_entry_list_clear(&stuff->entries);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_list_result_init(NiListResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_list_result_clear(NiListResult *result)
{
    if (result) {
        ni_entry_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_property_list_stuff_init(NiPropertyListStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_property_list_stuff_clear(NiPropertyListStuff *stuff)
{
    if (stuff) {
        ni_property_list_clear(&stuff->props);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_create_args_init(NiCreateArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_create_args_clear(NiCreateArgs *args)
{
    if (args) {
        ni_property_list_clear(&args->props);
        reset_value(args, sizeof(*args));
    }
}

void ni_property_list_result_init(NiPropertyListResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_property_list_result_clear(NiPropertyListResult *result)
{
    if (result) {
        ni_property_list_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_create_stuff_init(NiCreateStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_create_stuff_clear(NiCreateStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_create_result_init(NiCreateResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_create_result_clear(NiCreateResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_destroy_args_init(NiDestroyArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_destroy_args_clear(NiDestroyArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_lookup_args_init(NiLookupArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_lookup_args_clear(NiLookupArgs *args)
{
    if (args) {
        ni_name_clear(&args->key);
        ni_name_clear(&args->value);
        reset_value(args, sizeof(*args));
    }
}

void ni_lookup_stuff_init(NiLookupStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_lookup_stuff_clear(NiLookupStuff *stuff)
{
    if (stuff) {
        ni_id_list_clear(&stuff->idlist);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_lookup_result_init(NiLookupResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_lookup_result_clear(NiLookupResult *result)
{
    if (result) {
        ni_lookup_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_name_args_init(NiNameArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_name_args_clear(NiNameArgs *args)
{
    if (args) {
        ni_name_clear(&args->name);
        reset_value(args, sizeof(*args));
    }
}

void ni_create_prop_args_init(NiCreatePropArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_create_prop_args_clear(NiCreatePropArgs *args)
{
    if (args) {
        ni_property_clear(&args->prop);
        reset_value(args, sizeof(*args));
    }
}

void ni_write_prop_args_init(NiWritePropArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_write_prop_args_clear(NiWritePropArgs *args)
{
    if (args) {
        ni_name_list_clear(&args->values);
        reset_value(args, sizeof(*args));
    }
}

void ni_prop_args_init(NiPropArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_prop_args_clear(NiPropArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_name_list_stuff_init(NiNameListStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_name_list_stuff_clear(NiNameListStuff *stuff)
{
    if (stuff) {
        ni_name_list_clear(&stuff->values);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_name_list_result_init(NiNameListResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_name_list_result_clear(NiNameListResult *result)
{
    if (result) {
        ni_name_list_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_prop_name_args_init(NiPropNameArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_prop_name_args_clear(NiPropNameArgs *args)
{
    if (args) {
        ni_name_clear(&args->name);
        reset_value(args, sizeof(*args));
    }
}

void ni_create_name_args_init(NiCreateNameArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_create_name_args_clear(NiCreateNameArgs *args)
{
    if (args) {
        ni_name_clear(&args->name);
        reset_value(args, sizeof(*args));
    }
}

void ni_name_index_args_init(NiNameIndexArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_name_index_args_clear(NiNameIndexArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_write_name_args_init(NiWriteNameArgs *args)
{
    reset_value(args, sizeof(*args));
}

void ni_write_name_args_clear(NiWriteNameArgs *args)
{
    if (args) {
        ni_name_clear(&args->name);
        reset_value(args, sizeof(*args));
    }
}

void ni_read_name_stuff_init(NiReadNameStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_read_name_stuff_clear(NiReadNameStuff *stuff)
{
    if (stuff) {
        ni_name_clear(&stuff->name);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_read_name_result_init(NiReadNameResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_read_name_result_clear(NiReadNameResult *result)
{
    if (result) {
        ni_read_name_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_binding_init(NiBinding *binding)
{
    reset_value(binding, sizeof(*binding));
}

void ni_binding_clear(NiBinding *binding)
{
    if (binding) {
        ni_name_clear(&binding->tag);
        reset_value(binding, sizeof(*binding));
    }
}

void ni_rparent_result_init(NiRParentResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_rparent_result_clear(NiRParentResult *result)
{
    if (result) {
        ni_binding_clear(&result->binding);
        reset_value(result, sizeof(*result));
    }
}

void ni_object_node_init(NiObjectNode *node)
{
    reset_value(node, sizeof(*node));
}

void ni_object_node_clear(NiObjectNode *node)
{
    /* Clear the embedded object; callers owning node must release it. */
    if (node) {
        ni_object_clear(&node->object);
        node->next = NULL;
    }
}

void ni_object_list_init(NiObjectList *list)
{
    reset_value(list, sizeof(*list));
}

void ni_object_list_clear(NiObjectList *list)
{
    if (!list) {
        return;
    }
    NiObjectNode *node = list->head;
    while (node) {
        NiObjectNode *next = node->next;
        ni_object_node_clear(node);
        g_free(node);
        node = next;
    }
    reset_value(list, sizeof(*list));
}

void ni_read_all_stuff_init(NiReadAllStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_read_all_stuff_clear(NiReadAllStuff *stuff)
{
    if (stuff) {
        ni_object_list_clear(&stuff->list);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_read_all_result_init(NiReadAllResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_read_all_result_clear(NiReadAllResult *result)
{
    if (result) {
        ni_read_all_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

void ni_property_list_array_init(NiPropertyListArray *array)
{
    reset_value(array, sizeof(*array));
}

void ni_property_list_array_clear(NiPropertyListArray *array)
{
    if (!array) {
        return;
    }
    for (size_t i = 0; i < array->count; i++) {
        ni_property_list_clear(&array->entries[i]);
    }
    g_free(array->entries);
    reset_value(array, sizeof(*array));
}

void ni_list_all_stuff_init(NiListAllStuff *stuff)
{
    reset_value(stuff, sizeof(*stuff));
}

void ni_list_all_stuff_clear(NiListAllStuff *stuff)
{
    if (stuff) {
        ni_property_list_array_clear(&stuff->entries);
        reset_value(stuff, sizeof(*stuff));
    }
}

void ni_list_all_result_init(NiListAllResult *result)
{
    reset_value(result, sizeof(*result));
}

void ni_list_all_result_clear(NiListAllResult *result)
{
    if (result) {
        ni_list_all_stuff_clear(&result->stuff);
        reset_value(result, sizeof(*result));
    }
}

static bool padded_size(size_t length, size_t *padded)
{
    if (!padded || length > SIZE_MAX - 3) {
        return false;
    }
    *padded = (length + 3) & ~(size_t)3;
    return true;
}

/* Unlike the shared transport helper, protocol strings reject dirty padding. */
static bool decode_blob(OncRpcXdrReader *reader, const uint8_t **data,
                        size_t *length, size_t maximum)
{
    OncRpcXdrReader tmp;
    uint32_t wire_length;
    size_t padded;

    if (!reader || !onc_rpc_xdr_reader_remaining(reader)) {
        return false;
    }
    tmp = *reader;
    if (!onc_rpc_xdr_u32(&tmp, &wire_length) ||
        wire_length > maximum || !padded_size(wire_length, &padded) ||
        onc_rpc_xdr_reader_remaining(&tmp) < padded) {
        return false;
    }
    for (size_t i = wire_length; i < padded; i++) {
        if (tmp.cursor[i] != 0) {
            return false;
        }
    }
    if (data) {
        *data = tmp.cursor;
    }
    if (length) {
        *length = wire_length;
    }
    tmp.cursor += padded;
    *reader = tmp;
    return true;
}

static bool encode_blob(OncRpcXdrWriter *writer, const void *data,
                        size_t length, size_t maximum)
{
    return onc_rpc_xdr_put_counted_opaque(writer, data, length, maximum);
}

static bool valid_status(NiStatus status)
{
    return (status >= NI_OK && status <= NI_NOUSER) || status == NI_FAILED;
}

static bool decode_status_inner(OncRpcXdrReader *reader, NiStatus *status)
{
    int32_t value;

    if (!onc_rpc_xdr_i32(reader, &value) ||
        (((value < NI_OK) || (value > NI_NOUSER)) && value != NI_FAILED)) {
        return false;
    }
    *status = (NiStatus)value;
    return true;
}

static bool encode_status_inner(OncRpcXdrWriter *writer, NiStatus status)
{
    return valid_status(status) && onc_rpc_xdr_put_i32(writer, status);
}

static bool decode_name_inner(OncRpcXdrReader *reader, NiName *name)
{
    const uint8_t *data;
    size_t length;
    NiName value;

    if (!name || !decode_blob(reader, &data, &length, NI_NAME_MAXLEN) ||
        memchr(data, '\0', length) || length == SIZE_MAX) {
        return false;
    }
    value = g_malloc(length + 1);
    memcpy(value, data, length);
    value[length] = '\0';
    *name = value;
    return true;
}

static bool encode_name_inner(OncRpcXdrWriter *writer, const char *name)
{
    size_t length;

    if (!name) {
        return false;
    }
    length = strnlen(name, NI_NAME_MAXLEN + 1U);
    if (length > NI_NAME_MAXLEN) {
        return false;
    }
    return encode_blob(writer, name, length, NI_NAME_MAXLEN);
}

static bool alloc_array(void **array, size_t count, size_t element_size)
{
    if (!array || (count && (!element_size ||
                             count > SIZE_MAX / element_size))) {
        return false;
    }
    *array = count ? g_malloc0(count * element_size) : NULL;
    return true;
}

/* Every counted element has at least this many bytes on the wire. */
static bool enough_for_items(const OncRpcXdrReader *reader, uint32_t count,
                             size_t minimum)
{
    size_t remaining = onc_rpc_xdr_reader_remaining(reader);

    return !minimum || count <= remaining / minimum;
}

static bool decode_name_list_inner(OncRpcXdrReader *reader,
                                   NiNameList *list)
{
    OncRpcXdrReader tmp = *reader;
    uint32_t wire_count;
    NiNameList value;

    ni_name_list_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &wire_count) ||
        wire_count > NI_NAMELIST_MAXLEN ||
        !enough_for_items(&tmp, wire_count, sizeof(uint32_t)) ||
        !alloc_array((void **)&value.values, wire_count,
                     sizeof(*value.values))) {
        return false;
    }
    value.count = wire_count;
    for (size_t i = 0; i < value.count; i++) {
        if (!decode_name_inner(&tmp, &value.values[i])) {
            ni_name_list_clear(&value);
            return false;
        }
    }
    *list = value;
    *reader = tmp;
    return true;
}

static bool encode_name_list_inner(OncRpcXdrWriter *writer,
                                   const NiNameList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer || !list || list->count > NI_NAMELIST_MAXLEN ||
        (list->count && !list->values) || list->count > UINT32_MAX) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, list->count)) {
        return false;
    }
    for (size_t i = 0; i < list->count; i++) {
        if (!encode_name_inner(&tmp, list->values[i])) {
            return false;
        }
    }
    *writer = tmp;
    return true;
}

static bool decode_property_inner(OncRpcXdrReader *reader,
                                  NiProperty *property)
{
    OncRpcXdrReader tmp = *reader;
    NiProperty value;

    ni_property_init(&value);
    if (!decode_name_inner(&tmp, &value.name) ||
        !decode_name_list_inner(&tmp, &value.values)) {
        ni_property_clear(&value);
        return false;
    }
    *property = value;
    *reader = tmp;
    return true;
}

static bool encode_property_inner(OncRpcXdrWriter *writer,
                                  const NiProperty *property)
{
    OncRpcXdrWriter tmp;

    if (!writer || !property) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_inner(&tmp, property->name) ||
        !encode_name_list_inner(&tmp, &property->values)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_property_list_inner(OncRpcXdrReader *reader,
                                       NiPropertyList *list)
{
    OncRpcXdrReader tmp = *reader;
    uint32_t wire_count;
    NiPropertyList value;

    ni_property_list_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &wire_count) ||
        wire_count > NI_PROPLIST_MAXLEN ||
        !enough_for_items(&tmp, wire_count, 2 * sizeof(uint32_t)) ||
        !alloc_array((void **)&value.properties, wire_count,
                     sizeof(*value.properties))) {
        return false;
    }
    value.count = wire_count;
    for (size_t i = 0; i < value.count; i++) {
        ni_property_init(&value.properties[i]);
        if (!decode_property_inner(&tmp, &value.properties[i])) {
            ni_property_list_clear(&value);
            return false;
        }
    }
    *list = value;
    *reader = tmp;
    return true;
}

static bool encode_property_list_inner(OncRpcXdrWriter *writer,
                                       const NiPropertyList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer || !list || list->count > NI_PROPLIST_MAXLEN ||
        (list->count && !list->properties) || list->count > UINT32_MAX) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, list->count)) {
        return false;
    }
    for (size_t i = 0; i < list->count; i++) {
        if (!encode_property_inner(&tmp, &list->properties[i])) {
            return false;
        }
    }
    *writer = tmp;
    return true;
}

static bool decode_id_inner(OncRpcXdrReader *reader, NiId *id)
{
    NiId value;

    if (!id || !onc_rpc_xdr_u32(reader, &value.nii_object) ||
        !onc_rpc_xdr_u32(reader, &value.nii_instance)) {
        return false;
    }
    *id = value;
    return true;
}

static bool encode_id_inner(OncRpcXdrWriter *writer, const NiId *id)
{
    OncRpcXdrWriter tmp;

    if (!writer || !id) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, id->nii_object) ||
        !onc_rpc_xdr_put_u32(&tmp, id->nii_instance)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_id_list_inner(OncRpcXdrReader *reader, NiIdList *list)
{
    OncRpcXdrReader tmp = *reader;
    uint32_t wire_count;
    NiIdList value;

    ni_id_list_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &wire_count) ||
        wire_count > NI_IDLIST_MAXLEN ||
        !enough_for_items(&tmp, wire_count, sizeof(uint32_t)) ||
        !alloc_array((void **)&value.values, wire_count,
                     sizeof(*value.values))) {
        return false;
    }
    value.count = wire_count;
    for (size_t i = 0; i < value.count; i++) {
        if (!onc_rpc_xdr_u32(&tmp, &value.values[i])) {
            ni_id_list_clear(&value);
            return false;
        }
    }
    *list = value;
    *reader = tmp;
    return true;
}

static bool encode_id_list_inner(OncRpcXdrWriter *writer,
                                 const NiIdList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer || !list || list->count > NI_IDLIST_MAXLEN ||
        (list->count && !list->values) || list->count > UINT32_MAX) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, list->count)) {
        return false;
    }
    for (size_t i = 0; i < list->count; i++) {
        if (!onc_rpc_xdr_put_u32(&tmp, list->values[i])) {
            return false;
        }
    }
    *writer = tmp;
    return true;
}

static bool decode_optional_id_inner(OncRpcXdrReader *reader, NiId *id,
                                     bool *present)
{
    OncRpcXdrReader tmp = *reader;
    bool has_id;
    NiId value = { 0 };

    if (!onc_rpc_xdr_bool(&tmp, &has_id) ||
        (has_id && !decode_id_inner(&tmp, &value))) {
        return false;
    }
    if (id) {
        *id = value;
    }
    if (present) {
        *present = has_id;
    }
    *reader = tmp;
    return true;
}

static bool encode_optional_id_inner(OncRpcXdrWriter *writer, bool present,
                                     const NiId *id)
{
    OncRpcXdrWriter tmp;

    if (!writer || (present && !id)) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_bool(&tmp, present) ||
        (present && !encode_id_inner(&tmp, id))) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_encode_status(OncRpcXdrWriter *writer, NiStatus status)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, status)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_status(OncRpcXdrReader *reader, NiStatus *status)
{
    OncRpcXdrReader tmp;
    NiStatus value;

    if (!reader || !status) {
        return false;
    }
    tmp = *reader;
    if (!decode_status_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *status = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_name(OncRpcXdrWriter *writer, const char *name)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_inner(&tmp, name)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_name(OncRpcXdrReader *reader, NiName *name)
{
    OncRpcXdrReader tmp;
    NiName value = NULL;

    if (!reader || !name) {
        return false;
    }
    tmp = *reader;
    if (!decode_name_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_name_clear(&value);
        return false;
    }
    ni_name_clear(name);
    *name = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_name_list(OncRpcXdrWriter *writer,
                             const NiNameList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_list_inner(&tmp, list)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_name_list(OncRpcXdrReader *reader, NiNameList *list)
{
    OncRpcXdrReader tmp;
    NiNameList value;

    if (!reader || !list) {
        return false;
    }
    ni_name_list_init(&value);
    tmp = *reader;
    if (!decode_name_list_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_name_list_clear(&value);
        return false;
    }
    ni_name_list_clear(list);
    *list = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_property(OncRpcXdrWriter *writer,
                            const NiProperty *property)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_property_inner(&tmp, property)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_property(OncRpcXdrReader *reader, NiProperty *property)
{
    OncRpcXdrReader tmp;
    NiProperty value;

    if (!reader || !property) {
        return false;
    }
    ni_property_init(&value);
    tmp = *reader;
    if (!decode_property_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_property_clear(&value);
        return false;
    }
    ni_property_clear(property);
    *property = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_property_list(OncRpcXdrWriter *writer,
                                 const NiPropertyList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_property_list_inner(&tmp, list)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_property_list(OncRpcXdrReader *reader,
                                 NiPropertyList *list)
{
    OncRpcXdrReader tmp;
    NiPropertyList value;

    if (!reader || !list) {
        return false;
    }
    ni_property_list_init(&value);
    tmp = *reader;
    if (!decode_property_list_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_property_list_clear(&value);
        return false;
    }
    ni_property_list_clear(list);
    *list = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_id(OncRpcXdrWriter *writer, const NiId *id)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_id(OncRpcXdrReader *reader, NiId *id)
{
    OncRpcXdrReader tmp;
    NiId value;

    if (!reader || !id) {
        return false;
    }
    tmp = *reader;
    if (!decode_id_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *id = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_id_list(OncRpcXdrWriter *writer, const NiIdList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_list_inner(&tmp, list)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_id_list(OncRpcXdrReader *reader, NiIdList *list)
{
    OncRpcXdrReader tmp;
    NiIdList value;

    if (!reader || !list) {
        return false;
    }
    ni_id_list_init(&value);
    tmp = *reader;
    if (!decode_id_list_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_id_list_clear(&value);
        return false;
    }
    ni_id_list_clear(list);
    *list = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_optional_id(OncRpcXdrWriter *writer, bool present,
                               const NiId *id)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_optional_id_inner(&tmp, present, id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_optional_id(OncRpcXdrReader *reader, NiId *id,
                               bool *present)
{
    OncRpcXdrReader tmp;
    NiId value;
    bool has_id;

    if (!reader) {
        return false;
    }
    tmp = *reader;
    if (!decode_optional_id_inner(&tmp, &value, &has_id) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    if (id) {
        *id = value;
    }
    if (present) {
        *present = has_id;
    }
    *reader = tmp;
    return true;
}

static bool decode_bind_addr_info_inner(OncRpcXdrReader *reader,
                                        NiBindAddrInfo *info)
{
    NiBindAddrInfo value;

    if (!info || !onc_rpc_xdr_u32(reader, &value.udp_port) ||
        !onc_rpc_xdr_u32(reader, &value.tcp_port)) {
        return false;
    }
    *info = value;
    return true;
}

static bool encode_bind_addr_info_inner(OncRpcXdrWriter *writer,
                                        const NiBindAddrInfo *info)
{
    OncRpcXdrWriter tmp;

    if (!writer || !info) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, info->udp_port) ||
        !onc_rpc_xdr_put_u32(&tmp, info->tcp_port)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_bind_registration_inner(OncRpcXdrReader *reader,
                                           NiBindRegistration *registration)
{
    OncRpcXdrReader tmp = *reader;
    NiBindRegistration value;

    ni_bind_registration_init(&value);
    if (!decode_name_inner(&tmp, &value.tag) ||
        !decode_bind_addr_info_inner(&tmp, &value.addrs)) {
        ni_bind_registration_clear(&value);
        return false;
    }
    *registration = value;
    *reader = tmp;
    return true;
}

static bool encode_bind_registration_inner(
    OncRpcXdrWriter *writer, const NiBindRegistration *registration)
{
    OncRpcXdrWriter tmp;

    if (!writer || !registration) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_inner(&tmp, registration->tag) ||
        !encode_bind_addr_info_inner(&tmp, &registration->addrs)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_bind_clone_args_inner(OncRpcXdrReader *reader,
                                         NiBindCloneArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiBindCloneArgs value;

    ni_bind_clone_args_init(&value);
    if (!decode_name_inner(&tmp, &value.tag) ||
        !decode_name_inner(&tmp, &value.master_name) ||
        !onc_rpc_xdr_u32(&tmp, &value.master_addr) ||
        !decode_name_inner(&tmp, &value.master_tag)) {
        ni_bind_clone_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_bind_clone_args_inner(OncRpcXdrWriter *writer,
                                         const NiBindCloneArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_inner(&tmp, args->tag) ||
        !encode_name_inner(&tmp, args->master_name) ||
        !onc_rpc_xdr_put_u32(&tmp, args->master_addr) ||
        !encode_name_inner(&tmp, args->master_tag)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_bind_args_inner(OncRpcXdrReader *reader, NiBindArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiBindArgs value;

    ni_bind_args_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &value.client_addr) ||
        !decode_name_inner(&tmp, &value.client_tag) ||
        !decode_name_inner(&tmp, &value.server_tag)) {
        ni_bind_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_bind_args_inner(OncRpcXdrWriter *writer,
                                   const NiBindArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, args->client_addr) ||
        !encode_name_inner(&tmp, args->client_tag) ||
        !encode_name_inner(&tmp, args->server_tag)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_bind_getregister_result_inner(
    OncRpcXdrReader *reader, NiBindGetRegisterResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiBindGetRegisterResult value;

    ni_bind_getregister_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_bind_addr_info_inner(&tmp, &value.addrs))) {
        ni_bind_getregister_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_bind_getregister_result_inner(
    OncRpcXdrWriter *writer, const NiBindGetRegisterResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_bind_addr_info_inner(&tmp, &result->addrs))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_bind_listreg_result_inner(OncRpcXdrReader *reader,
                                             NiBindListRegResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiBindListRegResult value;
    uint32_t wire_count;

    ni_bind_listreg_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status)) {
        return false;
    }
    if (value.status == NI_OK) {
        if (!onc_rpc_xdr_u32(&tmp, &wire_count) ||
            wire_count > NIBIND_MAXREGS ||
            !enough_for_items(&tmp, wire_count, 3 * sizeof(uint32_t)) ||
            !alloc_array((void **)&value.registrations, wire_count,
                         sizeof(*value.registrations))) {
            return false;
        }
        value.count = wire_count;
        for (size_t i = 0; i < value.count; i++) {
            ni_bind_registration_init(&value.registrations[i]);
            if (!decode_bind_registration_inner(&tmp,
                                                &value.registrations[i])) {
                ni_bind_listreg_result_clear(&value);
                return false;
            }
        }
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_bind_listreg_result_inner(
    OncRpcXdrWriter *writer, const NiBindListRegResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result || result->count > NIBIND_MAXREGS ||
        (result->count && !result->registrations) ||
        result->count > UINT32_MAX) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status)) {
        return false;
    }
    if (result->status == NI_OK) {
        if (!onc_rpc_xdr_put_u32(&tmp, result->count)) {
            return false;
        }
        for (size_t i = 0; i < result->count; i++) {
            if (!encode_bind_registration_inner(&tmp,
                                                &result->registrations[i])) {
                return false;
            }
        }
    } else if (result->count) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_encode_bind_addr_info(OncRpcXdrWriter *writer,
                                  const NiBindAddrInfo *info)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_bind_addr_info_inner(&tmp, info)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_bind_addr_info(OncRpcXdrReader *reader,
                                  NiBindAddrInfo *info)
{
    OncRpcXdrReader tmp;
    NiBindAddrInfo value;

    if (!reader || !info) {
        return false;
    }
    tmp = *reader;
    if (!decode_bind_addr_info_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        return false;
    }
    *info = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_bind_registration(OncRpcXdrWriter *writer,
                                     const NiBindRegistration *registration)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_bind_registration_inner(&tmp, registration)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_bind_registration(OncRpcXdrReader *reader,
                                     NiBindRegistration *registration)
{
    OncRpcXdrReader tmp;
    NiBindRegistration value;

    if (!reader || !registration) {
        return false;
    }
    ni_bind_registration_init(&value);
    tmp = *reader;
    if (!decode_bind_registration_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_bind_registration_clear(&value);
        return false;
    }
    ni_bind_registration_clear(registration);
    *registration = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_bind_clone_args(OncRpcXdrWriter *writer,
                                   const NiBindCloneArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_bind_clone_args_inner(&tmp, args)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_bind_clone_args(OncRpcXdrReader *reader,
                                   NiBindCloneArgs *args)
{
    OncRpcXdrReader tmp;
    NiBindCloneArgs value;

    if (!reader || !args) {
        return false;
    }
    ni_bind_clone_args_init(&value);
    tmp = *reader;
    if (!decode_bind_clone_args_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_bind_clone_args_clear(&value);
        return false;
    }
    ni_bind_clone_args_clear(args);
    *args = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_bind_args(OncRpcXdrWriter *writer, const NiBindArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_bind_args_inner(&tmp, args)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_bind_args(OncRpcXdrReader *reader, NiBindArgs *args)
{
    OncRpcXdrReader tmp;
    NiBindArgs value;

    if (!reader || !args) {
        return false;
    }
    ni_bind_args_init(&value);
    tmp = *reader;
    if (!decode_bind_args_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_bind_args_clear(&value);
        return false;
    }
    ni_bind_args_clear(args);
    *args = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_bind_getregister_result(
    OncRpcXdrWriter *writer, const NiBindGetRegisterResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_bind_getregister_result_inner(&tmp, result)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_bind_getregister_result(
    OncRpcXdrReader *reader, NiBindGetRegisterResult *result)
{
    OncRpcXdrReader tmp;
    NiBindGetRegisterResult value;

    if (!reader || !result) {
        return false;
    }
    ni_bind_getregister_result_init(&value);
    tmp = *reader;
    if (!decode_bind_getregister_result_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_bind_getregister_result_clear(&value);
        return false;
    }
    ni_bind_getregister_result_clear(result);
    *result = value;
    *reader = tmp;
    return true;
}

bool ni_xdr_encode_bind_listreg_result(OncRpcXdrWriter *writer,
                                       const NiBindListRegResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer) {
        return false;
    }
    tmp = *writer;
    if (!encode_bind_listreg_result_inner(&tmp, result)) {
        return false;
    }
    *writer = tmp;
    return true;
}

bool ni_xdr_decode_bind_listreg_result(OncRpcXdrReader *reader,
                                       NiBindListRegResult *result)
{
    OncRpcXdrReader tmp;
    NiBindListRegResult value;

    if (!reader || !result) {
        return false;
    }
    ni_bind_listreg_result_init(&value);
    tmp = *reader;
    if (!decode_bind_listreg_result_inner(&tmp, &value) ||
        !onc_rpc_xdr_reader_empty(&tmp)) {
        ni_bind_listreg_result_clear(&value);
        return false;
    }
    ni_bind_listreg_result_clear(result);
    *result = value;
    *reader = tmp;
    return true;
}

static bool decode_object_inner(OncRpcXdrReader *reader, NiObject *object)
{
    OncRpcXdrReader tmp = *reader;
    NiObject value;

    ni_object_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_property_list_inner(&tmp, &value.properties) ||
        !onc_rpc_xdr_u32(&tmp, &value.parent) ||
        !decode_id_list_inner(&tmp, &value.children)) {
        ni_object_clear(&value);
        return false;
    }
    *object = value;
    *reader = tmp;
    return true;
}

static bool encode_object_inner(OncRpcXdrWriter *writer,
                                const NiObject *object)
{
    OncRpcXdrWriter tmp;

    if (!writer || !object) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &object->id) ||
        !encode_property_list_inner(&tmp, &object->properties) ||
        !onc_rpc_xdr_put_u32(&tmp, object->parent) ||
        !encode_id_list_inner(&tmp, &object->children)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_entry_inner(OncRpcXdrReader *reader, NiEntry *entry)
{
    OncRpcXdrReader tmp = *reader;
    NiEntry value;

    ni_entry_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &value.id) ||
        !onc_rpc_xdr_bool(&tmp, &value.has_names) ||
        (value.has_names && !decode_name_list_inner(&tmp, &value.names))) {
        ni_entry_clear(&value);
        return false;
    }
    *entry = value;
    *reader = tmp;
    return true;
}

static bool encode_entry_inner(OncRpcXdrWriter *writer, const NiEntry *entry)
{
    OncRpcXdrWriter tmp;

    if (!writer || !entry || (!entry->has_names &&
                              (entry->names.count || entry->names.values))) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, entry->id) ||
        !onc_rpc_xdr_put_bool(&tmp, entry->has_names) ||
        (entry->has_names && !encode_name_list_inner(&tmp, &entry->names))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_entry_list_inner(OncRpcXdrReader *reader,
                                    NiEntryList *list)
{
    OncRpcXdrReader tmp = *reader;
    NiEntryList value;
    uint32_t wire_count;

    ni_entry_list_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &wire_count) ||
        wire_count > NI_IDLIST_MAXLEN ||
        !enough_for_items(&tmp, wire_count, 2 * sizeof(uint32_t)) ||
        !alloc_array((void **)&value.entries, wire_count,
                     sizeof(*value.entries))) {
        return false;
    }
    value.count = wire_count;
    for (size_t i = 0; i < value.count; i++) {
        ni_entry_init(&value.entries[i]);
        if (!decode_entry_inner(&tmp, &value.entries[i])) {
            ni_entry_list_clear(&value);
            return false;
        }
    }
    *list = value;
    *reader = tmp;
    return true;
}

static bool encode_entry_list_inner(OncRpcXdrWriter *writer,
                                    const NiEntryList *list)
{
    OncRpcXdrWriter tmp;

    if (!writer || !list || list->count > NI_IDLIST_MAXLEN ||
        list->count > UINT32_MAX || (list->count && !list->entries)) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, list->count)) {
        return false;
    }
    for (size_t i = 0; i < list->count; i++) {
        if (!encode_entry_inner(&tmp, &list->entries[i])) {
            return false;
        }
    }
    *writer = tmp;
    return true;
}

static bool decode_id_result_inner(OncRpcXdrReader *reader,
                                   NiIdResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiIdResult value;

    ni_id_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status)) {
        return false;
    }
    value.has_id = value.status == NI_OK;
    if (value.has_id && !decode_id_inner(&tmp, &value.id)) {
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_id_result_inner(OncRpcXdrWriter *writer,
                                   const NiIdResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK && !result->has_id) ||
        (result->status == NI_OK && !encode_id_inner(&tmp, &result->id))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_parent_stuff_inner(OncRpcXdrReader *reader,
                                      NiParentStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiParentStuff value;

    ni_parent_stuff_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &value.object_id) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_parent_stuff_inner(OncRpcXdrWriter *writer,
                                      const NiParentStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, stuff->object_id) ||
        !encode_id_inner(&tmp, &stuff->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_parent_result_inner(OncRpcXdrReader *reader,
                                       NiParentResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiParentResult value;

    ni_parent_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_parent_stuff_inner(&tmp, &value.stuff))) {
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_parent_result_inner(OncRpcXdrWriter *writer,
                                       const NiParentResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_parent_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_children_stuff_inner(OncRpcXdrReader *reader,
                                        NiChildrenStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiChildrenStuff value;

    ni_children_stuff_init(&value);
    if (!decode_id_list_inner(&tmp, &value.children) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        ni_children_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_children_stuff_inner(OncRpcXdrWriter *writer,
                                        const NiChildrenStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_list_inner(&tmp, &stuff->children) ||
        !encode_id_inner(&tmp, &stuff->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_children_result_inner(OncRpcXdrReader *reader,
                                         NiChildrenResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiChildrenResult value;

    ni_children_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_children_stuff_inner(&tmp, &value.stuff))) {
        ni_children_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_children_result_inner(OncRpcXdrWriter *writer,
                                         const NiChildrenResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_children_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_entry_stuff_inner(OncRpcXdrReader *reader,
                                     NiEntryStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiEntryStuff value;

    ni_entry_stuff_init(&value);
    if (!decode_entry_list_inner(&tmp, &value.entries) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        ni_entry_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_entry_stuff_inner(OncRpcXdrWriter *writer,
                                     const NiEntryStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_entry_list_inner(&tmp, &stuff->entries) ||
        !encode_id_inner(&tmp, &stuff->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_list_result_inner(OncRpcXdrReader *reader,
                                     NiListResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiListResult value;

    ni_list_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_entry_stuff_inner(&tmp, &value.stuff))) {
        ni_list_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_list_result_inner(OncRpcXdrWriter *writer,
                                     const NiListResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_entry_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_property_list_stuff_inner(
    OncRpcXdrReader *reader, NiPropertyListStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiPropertyListStuff value;

    ni_property_list_stuff_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_property_list_inner(&tmp, &value.props)) {
        ni_property_list_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_property_list_stuff_inner(
    OncRpcXdrWriter *writer, const NiPropertyListStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &stuff->id) ||
        !encode_property_list_inner(&tmp, &stuff->props)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_create_args_inner(OncRpcXdrReader *reader,
                                     NiCreateArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiCreateArgs value;

    ni_create_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_property_list_inner(&tmp, &value.props) ||
        !onc_rpc_xdr_u32(&tmp, &value.where) ||
        !decode_optional_id_inner(&tmp, &value.target_id,
                                  &value.has_target_id)) {
        ni_create_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_create_args_inner(OncRpcXdrWriter *writer,
                                     const NiCreateArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !encode_property_list_inner(&tmp, &args->props) ||
        !onc_rpc_xdr_put_u32(&tmp, args->where) ||
        !encode_optional_id_inner(&tmp, args->has_target_id,
                                  &args->target_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_property_list_result_inner(
    OncRpcXdrReader *reader, NiPropertyListResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiPropertyListResult value;

    ni_property_list_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_property_list_stuff_inner(&tmp, &value.stuff))) {
        ni_property_list_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_property_list_result_inner(
    OncRpcXdrWriter *writer, const NiPropertyListResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_property_list_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_create_stuff_inner(OncRpcXdrReader *reader,
                                      NiCreateStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiCreateStuff value;

    ni_create_stuff_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_create_stuff_inner(OncRpcXdrWriter *writer,
                                      const NiCreateStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &stuff->id) ||
        !encode_id_inner(&tmp, &stuff->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_create_result_inner(OncRpcXdrReader *reader,
                                       NiCreateResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiCreateResult value;

    ni_create_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_create_stuff_inner(&tmp, &value.stuff))) {
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_create_result_inner(OncRpcXdrWriter *writer,
                                       const NiCreateResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_create_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_destroy_args_inner(OncRpcXdrReader *reader,
                                      NiDestroyArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiDestroyArgs value;

    ni_destroy_args_init(&value);
    if (!decode_id_inner(&tmp, &value.parent_id) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_destroy_args_inner(OncRpcXdrWriter *writer,
                                      const NiDestroyArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->parent_id) ||
        !encode_id_inner(&tmp, &args->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_lookup_args_inner(OncRpcXdrReader *reader,
                                     NiLookupArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiLookupArgs value;

    ni_lookup_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_name_inner(&tmp, &value.key) ||
        !decode_name_inner(&tmp, &value.value)) {
        ni_lookup_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_lookup_args_inner(OncRpcXdrWriter *writer,
                                     const NiLookupArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !encode_name_inner(&tmp, args->key) ||
        !encode_name_inner(&tmp, args->value)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_lookup_stuff_inner(OncRpcXdrReader *reader,
                                      NiLookupStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiLookupStuff value;

    ni_lookup_stuff_init(&value);
    if (!decode_id_list_inner(&tmp, &value.idlist) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        ni_lookup_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_lookup_stuff_inner(OncRpcXdrWriter *writer,
                                      const NiLookupStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_list_inner(&tmp, &stuff->idlist) ||
        !encode_id_inner(&tmp, &stuff->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_lookup_result_inner(OncRpcXdrReader *reader,
                                       NiLookupResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiLookupResult value;

    ni_lookup_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_lookup_stuff_inner(&tmp, &value.stuff))) {
        ni_lookup_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_lookup_result_inner(OncRpcXdrWriter *writer,
                                       const NiLookupResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_lookup_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_name_args_inner(OncRpcXdrReader *reader, NiNameArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiNameArgs value;

    ni_name_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_name_inner(&tmp, &value.name)) {
        ni_name_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_name_args_inner(OncRpcXdrWriter *writer,
                                   const NiNameArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !encode_name_inner(&tmp, args->name)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_create_prop_args_inner(OncRpcXdrReader *reader,
                                          NiCreatePropArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiCreatePropArgs value;

    ni_create_prop_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_property_inner(&tmp, &value.prop) ||
        !onc_rpc_xdr_u32(&tmp, &value.where)) {
        ni_create_prop_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_create_prop_args_inner(OncRpcXdrWriter *writer,
                                          const NiCreatePropArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !encode_property_inner(&tmp, &args->prop) ||
        !onc_rpc_xdr_put_u32(&tmp, args->where)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_write_prop_args_inner(OncRpcXdrReader *reader,
                                         NiWritePropArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiWritePropArgs value;

    ni_write_prop_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !onc_rpc_xdr_u32(&tmp, &value.prop_index) ||
        !decode_name_list_inner(&tmp, &value.values)) {
        ni_write_prop_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_write_prop_args_inner(OncRpcXdrWriter *writer,
                                         const NiWritePropArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !onc_rpc_xdr_put_u32(&tmp, args->prop_index) ||
        !encode_name_list_inner(&tmp, &args->values)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_prop_args_inner(OncRpcXdrReader *reader, NiPropArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiPropArgs value;

    ni_prop_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !onc_rpc_xdr_u32(&tmp, &value.prop_index)) {
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_prop_args_inner(OncRpcXdrWriter *writer,
                                   const NiPropArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !onc_rpc_xdr_put_u32(&tmp, args->prop_index)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_name_list_stuff_inner(OncRpcXdrReader *reader,
                                         NiNameListStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiNameListStuff value;

    ni_name_list_stuff_init(&value);
    if (!decode_name_list_inner(&tmp, &value.values) ||
        !decode_id_inner(&tmp, &value.self_id)) {
        ni_name_list_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_name_list_stuff_inner(OncRpcXdrWriter *writer,
                                         const NiNameListStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_list_inner(&tmp, &stuff->values) ||
        !encode_id_inner(&tmp, &stuff->self_id)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_name_list_result_inner(OncRpcXdrReader *reader,
                                          NiNameListResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiNameListResult value;

    ni_name_list_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_name_list_stuff_inner(&tmp, &value.stuff))) {
        ni_name_list_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_name_list_result_inner(
    OncRpcXdrWriter *writer, const NiNameListResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_name_list_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_prop_name_args_inner(OncRpcXdrReader *reader,
                                        NiPropNameArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiPropNameArgs value;

    ni_prop_name_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !onc_rpc_xdr_u32(&tmp, &value.prop_index) ||
        !decode_name_inner(&tmp, &value.name)) {
        ni_prop_name_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_prop_name_args_inner(OncRpcXdrWriter *writer,
                                        const NiPropNameArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !onc_rpc_xdr_put_u32(&tmp, args->prop_index) ||
        !encode_name_inner(&tmp, args->name)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_create_name_args_inner(OncRpcXdrReader *reader,
                                          NiCreateNameArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiCreateNameArgs value;

    ni_create_name_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !onc_rpc_xdr_u32(&tmp, &value.prop_index) ||
        !decode_name_inner(&tmp, &value.name) ||
        !onc_rpc_xdr_u32(&tmp, &value.where)) {
        ni_create_name_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_create_name_args_inner(OncRpcXdrWriter *writer,
                                          const NiCreateNameArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !onc_rpc_xdr_put_u32(&tmp, args->prop_index) ||
        !encode_name_inner(&tmp, args->name) ||
        !onc_rpc_xdr_put_u32(&tmp, args->where)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_name_index_args_inner(OncRpcXdrReader *reader,
                                         NiNameIndexArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiNameIndexArgs value;

    ni_name_index_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !onc_rpc_xdr_u32(&tmp, &value.prop_index) ||
        !onc_rpc_xdr_u32(&tmp, &value.name_index)) {
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_name_index_args_inner(OncRpcXdrWriter *writer,
                                         const NiNameIndexArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !onc_rpc_xdr_put_u32(&tmp, args->prop_index) ||
        !onc_rpc_xdr_put_u32(&tmp, args->name_index)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_write_name_args_inner(OncRpcXdrReader *reader,
                                         NiWriteNameArgs *args)
{
    OncRpcXdrReader tmp = *reader;
    NiWriteNameArgs value;

    ni_write_name_args_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !onc_rpc_xdr_u32(&tmp, &value.prop_index) ||
        !onc_rpc_xdr_u32(&tmp, &value.name_index) ||
        !decode_name_inner(&tmp, &value.name)) {
        ni_write_name_args_clear(&value);
        return false;
    }
    *args = value;
    *reader = tmp;
    return true;
}

static bool encode_write_name_args_inner(OncRpcXdrWriter *writer,
                                         const NiWriteNameArgs *args)
{
    OncRpcXdrWriter tmp;

    if (!writer || !args) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &args->id) ||
        !onc_rpc_xdr_put_u32(&tmp, args->prop_index) ||
        !onc_rpc_xdr_put_u32(&tmp, args->name_index) ||
        !encode_name_inner(&tmp, args->name)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_read_name_stuff_inner(OncRpcXdrReader *reader,
                                         NiReadNameStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiReadNameStuff value;

    ni_read_name_stuff_init(&value);
    if (!decode_id_inner(&tmp, &value.id) ||
        !decode_name_inner(&tmp, &value.name)) {
        ni_read_name_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_read_name_stuff_inner(OncRpcXdrWriter *writer,
                                         const NiReadNameStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &stuff->id) ||
        !encode_name_inner(&tmp, stuff->name)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_read_name_result_inner(OncRpcXdrReader *reader,
                                          NiReadNameResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiReadNameResult value;

    ni_read_name_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_read_name_stuff_inner(&tmp, &value.stuff))) {
        ni_read_name_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_read_name_result_inner(
    OncRpcXdrWriter *writer, const NiReadNameResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_read_name_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_binding_inner(OncRpcXdrReader *reader, NiBinding *binding)
{
    OncRpcXdrReader tmp = *reader;
    NiBinding value;

    ni_binding_init(&value);
    if (!decode_name_inner(&tmp, &value.tag) ||
        !onc_rpc_xdr_u32(&tmp, &value.addr)) {
        ni_binding_clear(&value);
        return false;
    }
    *binding = value;
    *reader = tmp;
    return true;
}

static bool encode_binding_inner(OncRpcXdrWriter *writer,
                                 const NiBinding *binding)
{
    OncRpcXdrWriter tmp;

    if (!writer || !binding) {
        return false;
    }
    tmp = *writer;
    if (!encode_name_inner(&tmp, binding->tag) ||
        !onc_rpc_xdr_put_u32(&tmp, binding->addr)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_rparent_result_inner(OncRpcXdrReader *reader,
                                        NiRParentResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiRParentResult value;

    ni_rparent_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK && !decode_binding_inner(&tmp,
                                                         &value.binding))) {
        ni_rparent_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_rparent_result_inner(OncRpcXdrWriter *writer,
                                        const NiRParentResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK && !encode_binding_inner(&tmp,
                                                           &result->binding))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_object_list_inner(OncRpcXdrReader *reader,
                                     NiObjectList *list)
{
    OncRpcXdrReader tmp = *reader;
    NiObjectList value;
    NiObjectNode **tail;
    size_t count = 0;

    ni_object_list_init(&value);
    tail = &value.head;
    while (true) {
        bool present;
        NiObjectNode *node;

        if (!onc_rpc_xdr_bool(&tmp, &present)) {
            ni_object_list_clear(&value);
            return false;
        }
        if (!present) {
            break;
        }
        if (count == NI_OBJECT_LIST_MAXLEN) {
            ni_object_list_clear(&value);
            return false;
        }
        node = g_new0(NiObjectNode, 1);
        ni_object_node_init(node);
        if (!decode_object_inner(&tmp, &node->object)) {
            ni_object_node_clear(node);
            g_free(node);
            ni_object_list_clear(&value);
            return false;
        }
        *tail = node;
        tail = &node->next;
        count++;
    }
    value.count = count;
    *list = value;
    *reader = tmp;
    return true;
}

static bool encode_object_list_inner(OncRpcXdrWriter *writer,
                                     const NiObjectList *list)
{
    OncRpcXdrWriter tmp;
    const NiObjectNode *node;
    size_t count = 0;

    if (!writer || !list || list->count > NI_OBJECT_LIST_MAXLEN ||
        (list->count && !list->head)) {
        return false;
    }
    tmp = *writer;
    for (node = list->head; node; node = node->next) {
        if (count == NI_OBJECT_LIST_MAXLEN ||
            (list->count && count >= list->count) ||
            !onc_rpc_xdr_put_bool(&tmp, true) ||
            !encode_object_inner(&tmp, &node->object)) {
            return false;
        }
        count++;
    }
    if (count != list->count || !onc_rpc_xdr_put_bool(&tmp, false)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_read_all_stuff_inner(OncRpcXdrReader *reader,
                                        NiReadAllStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiReadAllStuff value;

    ni_read_all_stuff_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &value.checksum) ||
        !onc_rpc_xdr_u32(&tmp, &value.highestid) ||
        !decode_object_list_inner(&tmp, &value.list)) {
        ni_read_all_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_read_all_stuff_inner(OncRpcXdrWriter *writer,
                                        const NiReadAllStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, stuff->checksum) ||
        !onc_rpc_xdr_put_u32(&tmp, stuff->highestid) ||
        !encode_object_list_inner(&tmp, &stuff->list)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_read_all_result_inner(OncRpcXdrReader *reader,
                                         NiReadAllResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiReadAllResult value;

    ni_read_all_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_read_all_stuff_inner(&tmp, &value.stuff))) {
        ni_read_all_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_read_all_result_inner(OncRpcXdrWriter *writer,
                                         const NiReadAllResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_read_all_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_property_list_array_inner(
    OncRpcXdrReader *reader, NiPropertyListArray *array)
{
    OncRpcXdrReader tmp = *reader;
    NiPropertyListArray value;
    uint32_t wire_count;

    ni_property_list_array_init(&value);
    if (!onc_rpc_xdr_u32(&tmp, &wire_count) ||
        wire_count > NI_IDLIST_MAXLEN ||
        !enough_for_items(&tmp, wire_count, sizeof(uint32_t)) ||
        !alloc_array((void **)&value.entries, wire_count,
                     sizeof(*value.entries))) {
        return false;
    }
    value.count = wire_count;
    for (size_t i = 0; i < value.count; i++) {
        ni_property_list_init(&value.entries[i]);
        if (!decode_property_list_inner(&tmp, &value.entries[i])) {
            ni_property_list_array_clear(&value);
            return false;
        }
    }
    *array = value;
    *reader = tmp;
    return true;
}

static bool encode_property_list_array_inner(
    OncRpcXdrWriter *writer, const NiPropertyListArray *array)
{
    OncRpcXdrWriter tmp;

    if (!writer || !array || array->count > NI_IDLIST_MAXLEN ||
        array->count > UINT32_MAX || (array->count && !array->entries)) {
        return false;
    }
    tmp = *writer;
    if (!onc_rpc_xdr_put_u32(&tmp, array->count)) {
        return false;
    }
    for (size_t i = 0; i < array->count; i++) {
        if (!encode_property_list_inner(&tmp, &array->entries[i])) {
            return false;
        }
    }
    *writer = tmp;
    return true;
}

static bool decode_list_all_stuff_inner(OncRpcXdrReader *reader,
                                        NiListAllStuff *stuff)
{
    OncRpcXdrReader tmp = *reader;
    NiListAllStuff value;

    ni_list_all_stuff_init(&value);
    if (!decode_id_inner(&tmp, &value.self_id) ||
        !decode_property_list_array_inner(&tmp, &value.entries)) {
        ni_list_all_stuff_clear(&value);
        return false;
    }
    *stuff = value;
    *reader = tmp;
    return true;
}

static bool encode_list_all_stuff_inner(OncRpcXdrWriter *writer,
                                        const NiListAllStuff *stuff)
{
    OncRpcXdrWriter tmp;

    if (!writer || !stuff) {
        return false;
    }
    tmp = *writer;
    if (!encode_id_inner(&tmp, &stuff->self_id) ||
        !encode_property_list_array_inner(&tmp, &stuff->entries)) {
        return false;
    }
    *writer = tmp;
    return true;
}

static bool decode_list_all_result_inner(OncRpcXdrReader *reader,
                                         NiListAllResult *result)
{
    OncRpcXdrReader tmp = *reader;
    NiListAllResult value;

    ni_list_all_result_init(&value);
    if (!decode_status_inner(&tmp, &value.status) ||
        (value.status == NI_OK &&
         !decode_list_all_stuff_inner(&tmp, &value.stuff))) {
        ni_list_all_result_clear(&value);
        return false;
    }
    *result = value;
    *reader = tmp;
    return true;
}

static bool encode_list_all_result_inner(OncRpcXdrWriter *writer,
                                         const NiListAllResult *result)
{
    OncRpcXdrWriter tmp;

    if (!writer || !result) {
        return false;
    }
    tmp = *writer;
    if (!encode_status_inner(&tmp, result->status) ||
        (result->status == NI_OK &&
         !encode_list_all_stuff_inner(&tmp, &result->stuff))) {
        return false;
    }
    *writer = tmp;
    return true;
}

/*
 * Procedure-body codecs consume exactly one top-level value.  Each wrapper
 * decodes into a fresh temporary, rejects trailing bytes, and commits both the
 * reader and the destination only after the complete value is valid.
 */
#define DEFINE_NI_XDR_CODEC(name, type, init_fn, clear_fn, encode_fn, \
                             decode_fn) \
    bool ni_xdr_encode_##name(OncRpcXdrWriter *writer, \
                              const type * value) \
    { \
        OncRpcXdrWriter tmp; \
        if (!writer || !value) { \
            return false; \
        } \
        tmp = *writer; \
        if (!encode_fn(&tmp, value)) { \
            return false; \
        } \
        *writer = tmp; \
        return true; \
    } \
    bool ni_xdr_decode_##name(OncRpcXdrReader *reader, type *value) \
    { \
        OncRpcXdrReader tmp; \
        type decoded; \
        if (!reader || !value) { \
            return false; \
        } \
        init_fn(&decoded); \
        tmp = *reader; \
        if (!decode_fn(&tmp, &decoded) || \
            !onc_rpc_xdr_reader_empty(&tmp)) { \
            clear_fn(&decoded); \
            return false; \
        } \
        clear_fn(value); \
        *value = decoded; \
        *reader = tmp; \
        return true; \
    }

DEFINE_NI_XDR_CODEC(id_result, NiIdResult, ni_id_result_init,
                    ni_id_result_clear, encode_id_result_inner,
                    decode_id_result_inner)
DEFINE_NI_XDR_CODEC(parent_stuff, NiParentStuff, ni_parent_stuff_init,
                    ni_parent_stuff_clear, encode_parent_stuff_inner,
                    decode_parent_stuff_inner)
DEFINE_NI_XDR_CODEC(parent_result, NiParentResult, ni_parent_result_init,
                    ni_parent_result_clear, encode_parent_result_inner,
                    decode_parent_result_inner)
DEFINE_NI_XDR_CODEC(children_stuff, NiChildrenStuff, ni_children_stuff_init,
                    ni_children_stuff_clear, encode_children_stuff_inner,
                    decode_children_stuff_inner)
DEFINE_NI_XDR_CODEC(children_result, NiChildrenResult,
                    ni_children_result_init, ni_children_result_clear,
                    encode_children_result_inner, decode_children_result_inner)
DEFINE_NI_XDR_CODEC(entry, NiEntry, ni_entry_init, ni_entry_clear,
                    encode_entry_inner, decode_entry_inner)
DEFINE_NI_XDR_CODEC(entry_list, NiEntryList, ni_entry_list_init,
                    ni_entry_list_clear, encode_entry_list_inner,
                    decode_entry_list_inner)
DEFINE_NI_XDR_CODEC(entry_stuff, NiEntryStuff, ni_entry_stuff_init,
                    ni_entry_stuff_clear, encode_entry_stuff_inner,
                    decode_entry_stuff_inner)
DEFINE_NI_XDR_CODEC(list_result, NiListResult, ni_list_result_init,
                    ni_list_result_clear, encode_list_result_inner,
                    decode_list_result_inner)
DEFINE_NI_XDR_CODEC(property_list_stuff, NiPropertyListStuff,
                    ni_property_list_stuff_init, ni_property_list_stuff_clear,
                    encode_property_list_stuff_inner,
                    decode_property_list_stuff_inner)
DEFINE_NI_XDR_CODEC(create_args, NiCreateArgs, ni_create_args_init,
                    ni_create_args_clear, encode_create_args_inner,
                    decode_create_args_inner)
DEFINE_NI_XDR_CODEC(property_list_result, NiPropertyListResult,
                    ni_property_list_result_init,
                    ni_property_list_result_clear,
                    encode_property_list_result_inner,
                    decode_property_list_result_inner)
DEFINE_NI_XDR_CODEC(create_stuff, NiCreateStuff, ni_create_stuff_init,
                    ni_create_stuff_clear, encode_create_stuff_inner,
                    decode_create_stuff_inner)
DEFINE_NI_XDR_CODEC(create_result, NiCreateResult, ni_create_result_init,
                    ni_create_result_clear, encode_create_result_inner,
                    decode_create_result_inner)
DEFINE_NI_XDR_CODEC(destroy_args, NiDestroyArgs, ni_destroy_args_init,
                    ni_destroy_args_clear, encode_destroy_args_inner,
                    decode_destroy_args_inner)
DEFINE_NI_XDR_CODEC(lookup_args, NiLookupArgs, ni_lookup_args_init,
                    ni_lookup_args_clear, encode_lookup_args_inner,
                    decode_lookup_args_inner)
DEFINE_NI_XDR_CODEC(lookup_stuff, NiLookupStuff, ni_lookup_stuff_init,
                    ni_lookup_stuff_clear, encode_lookup_stuff_inner,
                    decode_lookup_stuff_inner)
DEFINE_NI_XDR_CODEC(lookup_result, NiLookupResult, ni_lookup_result_init,
                    ni_lookup_result_clear, encode_lookup_result_inner,
                    decode_lookup_result_inner)
DEFINE_NI_XDR_CODEC(name_args, NiNameArgs, ni_name_args_init,
                    ni_name_args_clear, encode_name_args_inner,
                    decode_name_args_inner)
DEFINE_NI_XDR_CODEC(create_prop_args, NiCreatePropArgs,
                    ni_create_prop_args_init, ni_create_prop_args_clear,
                    encode_create_prop_args_inner,
                    decode_create_prop_args_inner)
DEFINE_NI_XDR_CODEC(write_prop_args, NiWritePropArgs,
                    ni_write_prop_args_init, ni_write_prop_args_clear,
                    encode_write_prop_args_inner,
                    decode_write_prop_args_inner)
DEFINE_NI_XDR_CODEC(prop_args, NiPropArgs, ni_prop_args_init,
                    ni_prop_args_clear, encode_prop_args_inner,
                    decode_prop_args_inner)
DEFINE_NI_XDR_CODEC(name_list_stuff, NiNameListStuff,
                    ni_name_list_stuff_init, ni_name_list_stuff_clear,
                    encode_name_list_stuff_inner,
                    decode_name_list_stuff_inner)
DEFINE_NI_XDR_CODEC(name_list_result, NiNameListResult,
                    ni_name_list_result_init, ni_name_list_result_clear,
                    encode_name_list_result_inner,
                    decode_name_list_result_inner)
DEFINE_NI_XDR_CODEC(prop_name_args, NiPropNameArgs,
                    ni_prop_name_args_init, ni_prop_name_args_clear,
                    encode_prop_name_args_inner,
                    decode_prop_name_args_inner)
DEFINE_NI_XDR_CODEC(create_name_args, NiCreateNameArgs,
                    ni_create_name_args_init, ni_create_name_args_clear,
                    encode_create_name_args_inner,
                    decode_create_name_args_inner)
DEFINE_NI_XDR_CODEC(name_index_args, NiNameIndexArgs,
                    ni_name_index_args_init, ni_name_index_args_clear,
                    encode_name_index_args_inner,
                    decode_name_index_args_inner)
DEFINE_NI_XDR_CODEC(write_name_args, NiWriteNameArgs,
                    ni_write_name_args_init, ni_write_name_args_clear,
                    encode_write_name_args_inner,
                    decode_write_name_args_inner)
DEFINE_NI_XDR_CODEC(read_name_stuff, NiReadNameStuff,
                    ni_read_name_stuff_init, ni_read_name_stuff_clear,
                    encode_read_name_stuff_inner,
                    decode_read_name_stuff_inner)
DEFINE_NI_XDR_CODEC(read_name_result, NiReadNameResult,
                    ni_read_name_result_init, ni_read_name_result_clear,
                    encode_read_name_result_inner,
                    decode_read_name_result_inner)
DEFINE_NI_XDR_CODEC(binding, NiBinding, ni_binding_init, ni_binding_clear,
                    encode_binding_inner, decode_binding_inner)
DEFINE_NI_XDR_CODEC(rparent_result, NiRParentResult,
                    ni_rparent_result_init, ni_rparent_result_clear,
                    encode_rparent_result_inner,
                    decode_rparent_result_inner)
DEFINE_NI_XDR_CODEC(object, NiObject, ni_object_init, ni_object_clear,
                    encode_object_inner, decode_object_inner)
DEFINE_NI_XDR_CODEC(object_list, NiObjectList, ni_object_list_init,
                    ni_object_list_clear, encode_object_list_inner,
                    decode_object_list_inner)
DEFINE_NI_XDR_CODEC(read_all_stuff, NiReadAllStuff, ni_read_all_stuff_init,
                    ni_read_all_stuff_clear, encode_read_all_stuff_inner,
                    decode_read_all_stuff_inner)
DEFINE_NI_XDR_CODEC(read_all_result, NiReadAllResult, ni_read_all_result_init,
                    ni_read_all_result_clear, encode_read_all_result_inner,
                    decode_read_all_result_inner)
DEFINE_NI_XDR_CODEC(property_list_array, NiPropertyListArray,
                    ni_property_list_array_init,
                    ni_property_list_array_clear,
                    encode_property_list_array_inner,
                    decode_property_list_array_inner)
DEFINE_NI_XDR_CODEC(list_all_stuff, NiListAllStuff,
                    ni_list_all_stuff_init, ni_list_all_stuff_clear,
                    encode_list_all_stuff_inner,
                    decode_list_all_stuff_inner)
DEFINE_NI_XDR_CODEC(list_all_result, NiListAllResult,
                    ni_list_all_result_init, ni_list_all_result_clear,
                    encode_list_all_result_inner,
                    decode_list_all_result_inner)

#undef DEFINE_NI_XDR_CODEC
