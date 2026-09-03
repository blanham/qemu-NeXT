/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "hw/netinfo/netinfo-db.h"
#include "hw/netinfo/netinfo-xdr.h"

typedef struct NetInfoDbProperty {
    char *name;
    GPtrArray *values; /* char *, owned */
} NetInfoDbProperty;

typedef struct NetInfoDbNode {
    NiId id;
    bool has_parent;
    NiIndex parent;
    GPtrArray *properties; /* NetInfoDbProperty *, owned */
} NetInfoDbNode;

struct NetInfoDb {
    char *tag;
    GPtrArray *nodes; /* NetInfoDbNode *, insertion order */
    GHashTable *by_object; /* object ID -> NetInfoDbNode, lookup only */
    NetInfoDbNode *root;
    bool sealed;
};

static bool property_has_value(const NetInfoDbProperty *property,
                               const char *value);

static gpointer object_key(NiIndex object)
{
    return GUINT_TO_POINTER(object);
}

static void netinfo_db_property_free(gpointer opaque)
{
    NetInfoDbProperty *property = opaque;

    if (!property) {
        return;
    }
    g_free(property->name);
    g_ptr_array_free(property->values, true);
    g_free(property);
}

static void netinfo_db_node_free(gpointer opaque)
{
    NetInfoDbNode *node = opaque;

    if (!node) {
        return;
    }
    g_ptr_array_free(node->properties, true);
    g_free(node);
}

static NetInfoDbNode *netinfo_db_find(const NetInfoDb *db, NiIndex object)
{
    if (!db || !db->by_object) {
        return NULL;
    }
    return g_hash_table_lookup(db->by_object, object_key(object));
}

static bool valid_name(const char *name, bool allow_empty)
{
    size_t length;

    if (!name) {
        return false;
    }
    length = strnlen(name, NI_SERVICE_MAX_NAME + 1);
    return (allow_empty || length != 0) && length <= NI_SERVICE_MAX_NAME;
}

static NiStatus validate_name(const char *name, bool allow_empty)
{
    size_t length;

    if (!name) {
        return NI_SYSTEMERR;
    }
    length = strnlen(name, NI_SERVICE_MAX_NAME + 1);
    if (length > NI_SERVICE_MAX_NAME) {
        return NI_NOSPACE;
    }
    if (!allow_empty && !length) {
        return NI_NONAME;
    }
    return NI_OK;
}

static NiStatus validate_property_list(const NiPropertyList *properties)
{
    if (!properties) {
        return NI_OK;
    }
    if (properties->count > NI_SERVICE_MAX_PROPERTIES ||
        (properties->count && !properties->properties)) {
        return NI_NOSPACE;
    }
    for (size_t i = 0; i < properties->count; i++) {
        const NiProperty *property = &properties->properties[i];

        if (!valid_name(property->name, false)) {
            return validate_name(property->name, false);
        }
        if (property->values.count > NI_SERVICE_MAX_VALUES ||
            (property->values.count && !property->values.values)) {
            return NI_NOSPACE;
        }
        for (size_t j = 0; j < property->values.count; j++) {
            if (!valid_name(property->values.values[j], true)) {
                return validate_name(property->values.values[j], true);
            }
        }
    }
    return NI_OK;
}

static NetInfoDbProperty *netinfo_db_property_dup(const NiProperty *source)
{
    NetInfoDbProperty *property = g_new0(NetInfoDbProperty, 1);

    property->name = g_strdup(source->name);
    property->values = g_ptr_array_new_with_free_func(g_free);
    for (size_t i = 0; i < source->values.count; i++) {
        g_ptr_array_add(property->values,
                        g_strdup(source->values.values[i]));
    }
    return property;
}

static GPtrArray *netinfo_db_properties_dup(const NiPropertyList *source)
{
    GPtrArray *properties = g_ptr_array_new_with_free_func(
        netinfo_db_property_free);

    if (!source) {
        return properties;
    }
    for (size_t i = 0; i < source->count; i++) {
        g_ptr_array_add(properties,
                        netinfo_db_property_dup(&source->properties[i]));
    }
    return properties;
}

NetInfoDb *netinfo_db_new_with_tag(const char *tag)
{
    NetInfoDb *db;

    if (!valid_name(tag, false)) {
        return NULL;
    }
    db = g_new0(NetInfoDb, 1);
    db->tag = g_strdup(tag);
    db->nodes = g_ptr_array_new_with_free_func(netinfo_db_node_free);
    db->by_object = g_hash_table_new(g_direct_hash, g_direct_equal);
    return db;
}

NetInfoDb *netinfo_db_new(void)
{
    return netinfo_db_new_with_tag("network");
}

void netinfo_db_clear(NetInfoDb *db)
{
    if (!db) {
        return;
    }
    g_ptr_array_set_size(db->nodes, 0);
    g_hash_table_remove_all(db->by_object);
    db->root = NULL;
    db->sealed = false;
}

void netinfo_db_free(NetInfoDb *db)
{
    if (!db) {
        return;
    }
    g_ptr_array_free(db->nodes, true);
    g_hash_table_destroy(db->by_object);
    g_free(db->tag);
    g_free(db);
}

NetInfoDb *netinfo_db_new_default(void)
{
    static NiName root_name[] = { (char *)"/" };
    static NiName root_master[] = { (char *)"localhost/network" };
    static NiName machines_name[] = { (char *)"machines" };
    static NiName host_name[] = { (char *)"localhost" };
    static NiName host_ip[] = { (char *)"10.0.2.2" };
    static NiName host_serves[] = { (char *)"./network" };
    NiProperty root_properties[] = {
        { .name = (char *)"name",
          .values = { .count = 1, .values = root_name } },
        { .name = (char *)"master",
          .values = { .count = 1, .values = root_master } },
    };
    NiProperty machines_properties[] = {
        { .name = (char *)"name",
          .values = { .count = 1, .values = machines_name } },
    };
    NiProperty host_properties[] = {
        { .name = (char *)"name",
          .values = { .count = 1, .values = host_name } },
        { .name = (char *)"ip_address",
          .values = { .count = 1, .values = host_ip } },
        { .name = (char *)"serves",
          .values = { .count = 1, .values = host_serves } },
    };
    NiPropertyList root_list = {
        .count = G_N_ELEMENTS(root_properties), .properties = root_properties,
    };
    NiPropertyList machines_list = {
        .count = G_N_ELEMENTS(machines_properties),
        .properties = machines_properties,
    };
    NiPropertyList host_list = {
        .count = G_N_ELEMENTS(host_properties), .properties = host_properties,
    };
    NetInfoDb *db = netinfo_db_new();

    if (!db ||
        netinfo_db_add_node(db, &(NiId) { 0, 0x24 }, false, 0, &root_list) !=
            NI_OK ||
        netinfo_db_add_node(db, &(NiId) { 1, 1 }, true, 0,
                            &machines_list) !=
            NI_OK ||
        netinfo_db_add_node(db, &(NiId) { 2, 1 }, true, 1, &host_list) !=
            NI_OK || netinfo_db_seal(db) != NI_OK) {
        netinfo_db_free(db);
        return NULL;
    }
    return db;
}

NetInfoDb *netinfo_db_default(void)
{
    return netinfo_db_new_default();
}

const char *netinfo_db_tag(const NetInfoDb *db)
{
    return db ? db->tag : NULL;
}

size_t netinfo_db_node_count(const NetInfoDb *db)
{
    return db && db->nodes ? db->nodes->len : 0;
}

NiStatus netinfo_db_add_node(NetInfoDb *db, const NiId *id,
                             bool has_parent, NiIndex parent,
                             const NiPropertyList *properties)
{
    NiStatus status;
    NetInfoDbNode *node;

    if (!db || !id) {
        return NI_SYSTEMERR;
    }
    if (db->sealed) {
        return NI_RDONLY;
    }
    if (id->nii_object == UINT32_MAX ||
        netinfo_db_find(db, id->nii_object)) {
        return NI_BADID;
    }
    if (db->nodes->len >= NI_SERVICE_MAX_NODES) {
        return NI_NOSPACE;
    }
    if (has_parent && parent == UINT32_MAX) {
        return NI_UNRELATED;
    }
    status = validate_property_list(properties);
    if (status != NI_OK) {
        return status;
    }

    node = g_new0(NetInfoDbNode, 1);
    node->id = *id;
    node->has_parent = has_parent;
    node->parent = parent;
    node->properties = netinfo_db_properties_dup(properties);
    g_ptr_array_add(db->nodes, node);
    g_hash_table_insert(db->by_object, object_key(id->nii_object), node);
    return NI_OK;
}

NiStatus netinfo_db_add(NetInfoDb *db, const NetInfoDbNodeSpec *spec)
{
    if (!spec) {
        return NI_SYSTEMERR;
    }
    return netinfo_db_add_node(db, &spec->id, spec->has_parent,
                               spec->parent, spec->properties);
}

NiStatus netinfo_db_seal(NetInfoDb *db)
{
    size_t roots = 0;
    bool root_named = false;

    if (!db) {
        return NI_SYSTEMERR;
    }
    if (db->sealed) {
        return NI_OK;
    }
    if (!db->nodes->len) {
        return NI_NODIR;
    }
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *node = g_ptr_array_index(db->nodes, i);

        if (!node->has_parent) {
            roots++;
            db->root = node;
        }
    }
    if (roots != 1 || !db->root || db->root->id.nii_object != 0) {
        db->root = NULL;
        return NI_UNRELATED;
    }
    for (size_t i = 0; i < db->root->properties->len; i++) {
        NetInfoDbProperty *property = g_ptr_array_index(
            db->root->properties, i);
        if (strcmp(property->name, "name") == 0 &&
            property_has_value(property, "/")) {
            root_named = true;
            break;
        }
    }
    if (!root_named) {
        db->root = NULL;
        return NI_UNRELATED;
    }

    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *node = g_ptr_array_index(db->nodes, i);
        g_autoptr(GHashTable) seen = g_hash_table_new(g_direct_hash,
                                                      g_direct_equal);
        size_t depth = 0;

        while (node->has_parent) {
            if (depth++ >= NI_SERVICE_MAX_DEPTH) {
                db->root = NULL;
                return NI_NOSPACE;
            }
            if (g_hash_table_contains(seen, node)) {
                db->root = NULL;
                return NI_UNRELATED;
            }
            g_hash_table_add(seen, node);
            node = netinfo_db_find(db, node->parent);
            if (!node) {
                db->root = NULL;
                return NI_UNRELATED;
            }
        }
    }
    db->sealed = true;
    return NI_OK;
}

NiStatus netinfo_db_finish(NetInfoDb *db)
{
    return netinfo_db_seal(db);
}

static NiStatus ready_node(NetInfoDb *db, NiId *id, NetInfoDbNode **node)
{
    if (!db || !db->sealed || !id || !node) {
        return NI_SYSTEMERR;
    }
    *node = netinfo_db_find(db, id->nii_object);
    if (!*node) {
        return NI_BADID;
    }
    return NI_OK;
}

static void refresh_id(NiId *id, const NetInfoDbNode *node)
{
    id->nii_object = node->id.nii_object;
    id->nii_instance = node->id.nii_instance;
}

static NetInfoDbProperty *find_property(const NetInfoDbNode *node,
                                        const char *name)
{
    for (size_t i = 0; i < node->properties->len; i++) {
        NetInfoDbProperty *property = g_ptr_array_index(node->properties, i);

        if (strcmp(property->name, name) == 0) {
            return property;
        }
    }
    return NULL;
}

static bool property_has_value(const NetInfoDbProperty *property,
                               const char *value)
{
    for (size_t i = 0; i < property->values->len; i++) {
        if (strcmp(g_ptr_array_index(property->values, i), value) == 0) {
            return true;
        }
    }
    return false;
}

static void copy_name_list(const GPtrArray *source, NiNameList *destination)
{
    ni_name_list_init(destination);
    destination->count = source->len;
    if (!source->len) {
        return;
    }
    destination->values = g_new0(NiName, source->len);
    for (size_t i = 0; i < source->len; i++) {
        destination->values[i] = g_strdup(g_ptr_array_index(source, i));
    }
}

static void copy_properties(const NetInfoDbNode *node,
                            NiPropertyList *destination)
{
    ni_property_list_init(destination);
    destination->count = node->properties->len;
    if (!node->properties->len) {
        return;
    }
    destination->properties = g_new0(NiProperty, node->properties->len);
    for (size_t i = 0; i < node->properties->len; i++) {
        NetInfoDbProperty *source = g_ptr_array_index(node->properties, i);
        NiProperty *property = &destination->properties[i];

        ni_property_init(property);
        property->name = g_strdup(source->name);
        copy_name_list(source->values, &property->values);
    }
}

static void copy_ids(const NetInfoDb *db, const NetInfoDbNode *parent,
                     NiIdList *destination)
{
    size_t output = 0;

    ni_id_list_init(destination);
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *node = g_ptr_array_index(db->nodes, i);

        if (node->has_parent && node->parent == parent->id.nii_object) {
            destination->count++;
        }
    }
    if (!destination->count) {
        return;
    }
    destination->values = g_new0(NiIndex, destination->count);
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *node = g_ptr_array_index(db->nodes, i);

        if (node->has_parent && node->parent == parent->id.nii_object) {
            destination->values[output++] = node->id.nii_object;
        }
    }
}

NiStatus netinfo_db_root(NetInfoDb *db, NiId *id)
{
    if (!db || !db->sealed || !id) {
        return NI_SYSTEMERR;
    }
    refresh_id(id, db->root);
    return NI_OK;
}

NiStatus netinfo_db_self(NetInfoDb *db, NiId *id)
{
    NetInfoDbNode *node;
    NiStatus status = ready_node(db, id, &node);

    if (status == NI_OK) {
        refresh_id(id, node);
    }
    return status;
}

NiStatus netinfo_db_parent(NetInfoDb *db, NiId *id, NiId *parent)
{
    NetInfoDbNode *node;
    NetInfoDbNode *parent_node;
    NiStatus status;

    if (!parent) {
        return NI_SYSTEMERR;
    }
    status = ready_node(db, id, &node);
    if (status != NI_OK) {
        return status;
    }
    if (!node->has_parent) {
        /* Historical _NI_PARENT reports the root as its own parent. */
        refresh_id(id, node);
        refresh_id(parent, node);
        return NI_OK;
    }
    parent_node = netinfo_db_find(db, node->parent);
    if (!parent_node) {
        return NI_UNRELATED;
    }
    refresh_id(id, node);
    refresh_id(parent, parent_node);
    return NI_OK;
}

NiStatus netinfo_db_parent_index(NetInfoDb *db, NiId *id, NiIndex *parent)
{
    NiId parent_id;
    NiStatus status;

    if (!parent) {
        return NI_SYSTEMERR;
    }
    status = netinfo_db_parent(db, id, &parent_id);
    if (status == NI_OK) {
        *parent = parent_id.nii_object;
    }
    return status;
}

NiStatus netinfo_db_children(NetInfoDb *db, NiId *id, NiIdList *children)
{
    NetInfoDbNode *node;
    NiIdList result;
    NiStatus status;

    if (!children) {
        return NI_SYSTEMERR;
    }
    status = ready_node(db, id, &node);
    if (status != NI_OK) {
        return status;
    }
    copy_ids(db, node, &result);
    ni_id_list_clear(children);
    *children = result;
    refresh_id(id, node);
    return NI_OK;
}

NiStatus netinfo_db_read(NetInfoDb *db, NiId *id, NiPropertyList *properties)
{
    NetInfoDbNode *node;
    NiPropertyList result;
    NiStatus status;

    if (!properties) {
        return NI_SYSTEMERR;
    }
    status = ready_node(db, id, &node);
    if (status != NI_OK) {
        return status;
    }
    copy_properties(node, &result);
    ni_property_list_clear(properties);
    *properties = result;
    refresh_id(id, node);
    return NI_OK;
}

NiStatus netinfo_db_lookup(NetInfoDb *db, NiId *id, const char *key,
                           const char *value, NiIdList *found)
{
    NetInfoDbNode *parent;
    NiIdList result;
    NiStatus status;
    size_t output = 0;

    if (!key || !value || !found) {
        return NI_SYSTEMERR;
    }
    status = validate_name(key, false);
    if (status != NI_OK) {
        return status;
    }
    status = validate_name(value, true);
    if (status != NI_OK) {
        return status;
    }
    status = ready_node(db, id, &parent);
    if (status != NI_OK) {
        return status;
    }
    ni_id_list_init(&result);
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *child = g_ptr_array_index(db->nodes, i);
        NetInfoDbProperty *property;

        if (!child->has_parent || child->parent != parent->id.nii_object) {
            continue;
        }
        property = find_property(child, key);
        if (property && property_has_value(property, value)) {
            result.count++;
        }
    }
    if (!result.count) {
        ni_id_list_clear(&result);
        return NI_NODIR;
    }
    if (result.count) {
        result.values = g_new0(NiIndex, result.count);
        for (size_t i = 0; i < db->nodes->len; i++) {
            NetInfoDbNode *child = g_ptr_array_index(db->nodes, i);
            NetInfoDbProperty *property;

            if (!child->has_parent || child->parent != parent->id.nii_object) {
                continue;
            }
            property = find_property(child, key);
            if (property && property_has_value(property, value)) {
                result.values[output++] = child->id.nii_object;
            }
        }
    }
    ni_id_list_clear(found);
    *found = result;
    refresh_id(id, parent);
    return NI_OK;
}

NiStatus netinfo_db_list(NetInfoDb *db, NiId *id, const char *property,
                         NiEntryList *entries)
{
    NetInfoDbNode *parent;
    NiEntryList result;
    NiStatus status;
    NiEntry *entry;
    NetInfoDbProperty *projected;
    size_t output = 0;

    if (!property || !entries) {
        return NI_SYSTEMERR;
    }
    status = validate_name(property, false);
    if (status != NI_OK) {
        return status;
    }
    status = ready_node(db, id, &parent);
    if (status != NI_OK) {
        return status;
    }
    ni_entry_list_init(&result);
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *child = g_ptr_array_index(db->nodes, i);
        if (!child->has_parent || child->parent != parent->id.nii_object) {
            continue;
        }
        result.count++;
    }
    if (result.count) {
        result.entries = g_new0(NiEntry, result.count);
    }
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *child = g_ptr_array_index(db->nodes, i);

        if (!child->has_parent || child->parent != parent->id.nii_object) {
            continue;
        }
        entry = &result.entries[output++];
        ni_entry_init(entry);
        entry->id = child->id.nii_object;
        projected = find_property(child, property);
        if (projected && projected->values->len) {
            entry->has_names = true;
            copy_name_list(projected->values, &entry->names);
        }
    }
    ni_entry_list_clear(entries);
    *entries = result;
    refresh_id(id, parent);
    return NI_OK;
}

NiStatus netinfo_db_readprop(NetInfoDb *db, NiId *id, NiIndex property,
                             NiNameList *values)
{
    NetInfoDbNode *node;
    NetInfoDbProperty *source;
    NiNameList result;
    NiStatus status;

    if (!values) {
        return NI_SYSTEMERR;
    }
    status = ready_node(db, id, &node);
    if (status != NI_OK) {
        return status;
    }
    if (property >= node->properties->len) {
        return NI_NOPROP;
    }
    source = g_ptr_array_index(node->properties, property);
    copy_name_list(source->values, &result);
    ni_name_list_clear(values);
    *values = result;
    refresh_id(id, node);
    return NI_OK;
}

NiStatus netinfo_db_listprops(NetInfoDb *db, NiId *id, NiNameList *names)
{
    NetInfoDbNode *node;
    GPtrArray *source;
    GPtrArray *property_names;
    NiNameList result;
    NiStatus status;

    if (!names) {
        return NI_SYSTEMERR;
    }
    status = ready_node(db, id, &node);
    if (status != NI_OK) {
        return status;
    }
    source = node->properties;
    property_names = g_ptr_array_new();
    for (size_t i = 0; i < source->len; i++) {
        NetInfoDbProperty *property = g_ptr_array_index(source, i);
        g_ptr_array_add(property_names, property->name);
    }
    copy_name_list(property_names, &result);
    g_ptr_array_free(property_names, true);
    ni_name_list_clear(names);
    *names = result;
    refresh_id(id, node);
    return NI_OK;
}

NiStatus netinfo_db_readname(NetInfoDb *db, NiId *id, NiIndex property,
                             NiIndex value, NiName *name)
{
    NetInfoDbNode *node;
    NetInfoDbProperty *source;
    char *result;
    NiStatus status;

    if (!name) {
        return NI_SYSTEMERR;
    }
    status = ready_node(db, id, &node);
    if (status != NI_OK) {
        return status;
    }
    if (property >= node->properties->len) {
        return NI_NOPROP;
    }
    source = g_ptr_array_index(node->properties, property);
    if (value >= source->values->len) {
        return NI_NONAME;
    }
    result = g_strdup(g_ptr_array_index(source->values, value));
    g_free(*name);
    *name = result;
    refresh_id(id, node);
    return NI_OK;
}

NiStatus netinfo_db_lookup_read(NetInfoDb *db, NiId *id, const char *key,
                                const char *value, NiId *found,
                                NiPropertyList *properties)
{
    NetInfoDbNode *parent;
    NiStatus status;

    if (!key || !value || !found || !properties) {
        return NI_SYSTEMERR;
    }
    status = validate_name(key, false);
    if (status != NI_OK) {
        return status;
    }
    status = validate_name(value, true);
    if (status != NI_OK) {
        return status;
    }
    status = ready_node(db, id, &parent);
    if (status != NI_OK) {
        return status;
    }
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *child = g_ptr_array_index(db->nodes, i);
        NetInfoDbProperty *property;

        if (!child->has_parent || child->parent != parent->id.nii_object) {
            continue;
        }
        property = find_property(child, key);
        if (property && property_has_value(property, value)) {
            NiPropertyList result;

            copy_properties(child, &result);
            ni_property_list_clear(properties);
            *properties = result;
            refresh_id(id, parent);
            refresh_id(found, child);
            return NI_OK;
        }
    }
    return NI_NODIR;
}

NiStatus netinfo_db_read_prop(NetInfoDb *db, NiId *id, NiIndex property,
                              NiNameList *values)
{
    return netinfo_db_readprop(db, id, property, values);
}

NiStatus netinfo_db_list_props(NetInfoDb *db, NiId *id, NiNameList *names)
{
    return netinfo_db_listprops(db, id, names);
}

NiStatus netinfo_db_read_name(NetInfoDb *db, NiId *id, NiIndex property,
                              NiIndex value, NiName *name)
{
    return netinfo_db_readname(db, id, property, value, name);
}
