/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "qapi/error.h"
#include "hw/netinfo/netinfo-db.h"
#include "hw/netinfo/netinfo-xdr.h"
#include "qobject/qjson5.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"
#include "qobject/qnum.h"
#include "qobject/qstring.h"

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

#ifdef NETINFO_DB_TESTING
static NetInfoDbJson5ReadHook json5_read_hook;

void netinfo_db_test_set_json5_read_hook(NetInfoDbJson5ReadHook hook)
{
    json5_read_hook = hook;
}
#endif

static bool property_has_value(const NetInfoDbProperty *property,
                               const char *value);
static NetInfoDbProperty *find_property(const NetInfoDbNode *node,
                                        const char *name);

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

static bool json5_key_allowed(const char *key, const char *const *allowed,
                              size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (!strcmp(key, allowed[i])) {
            return true;
        }
    }
    return false;
}

#define JSON5_UNKNOWN_KEY_PREVIEW 64U

static void json5_set_unknown_key_error(const char *context, const char *key,
                                        Error **errp)
{
    size_t key_length = strlen(key);
    size_t preview_length = MIN(key_length,
                                (size_t)JSON5_UNKNOWN_KEY_PREVIEW);

    error_setg(errp, "%s has unknown key '%.*s%s' (%zu bytes)", context,
               (int)preview_length, key,
               preview_length < key_length ? "..." : "", key_length);
}

static bool json5_check_keys(const QDict *dict, const char *context,
                             const char *const *allowed, size_t count,
                             Error **errp)
{
    const QDictEntry *entry;

    if (!dict) {
        error_setg(errp, "%s must be an object", context);
        return false;
    }
    for (entry = qdict_ordered_first(dict); entry;
         entry = qdict_ordered_next(entry)) {
        const char *key = qdict_entry_key(entry);

        if (!json5_key_allowed(key, allowed, count)) {
            json5_set_unknown_key_error(context, key, errp);
            return false;
        }
    }
    for (size_t i = 0; i < count; i++) {
        if (!qdict_haskey(dict, allowed[i])) {
            error_setg(errp, "%s is missing key '%s'", context, allowed[i]);
            return false;
        }
    }
    return true;
}

static const char *json5_get_string(const QDict *dict, const char *key,
                                    const char *context, bool allow_empty,
                                    Error **errp)
{
    QString *string = qobject_to(QString, qdict_get(dict, key));
    const char *value;

    if (!string) {
        error_setg(errp, "%s.%s must be a string", context, key);
        return NULL;
    }
    value = qstring_get_str(string);
    if (!valid_name(value, allow_empty)) {
        if (!value || !value[0]) {
            error_setg(errp, "%s.%s must not be empty", context, key);
        } else {
            error_setg(errp, "%s.%s exceeds %u bytes", context, key,
                       NI_SERVICE_MAX_NAME);
        }
        return NULL;
    }
    return value;
}

static bool json5_get_u32(const QDict *dict, const char *key,
                          const char *context, uint32_t *value, Error **errp)
{
    QNum *number = qobject_to(QNum, qdict_get(dict, key));
    uint64_t unsigned_value;

    if (!number || !qnum_get_try_uint(number, &unsigned_value) ||
        unsigned_value > UINT32_MAX) {
        error_setg(errp, "%s.%s must be a finite non-negative u32 integer",
                   context, key);
        return false;
    }
    *value = (uint32_t)unsigned_value;
    return true;
}

static bool json5_parse_properties(const QDict *dict, const char *context,
                                   NiPropertyList *properties, Error **errp)
{
    const QDictEntry *entry;
    size_t property_index = 0;

    ni_property_list_init(properties);
    if (!dict) {
        error_setg(errp, "%s must be an object", context);
        return false;
    }
    if (qdict_size(dict) > NI_SERVICE_MAX_PROPERTIES) {
        error_setg(errp, "%s has more than %u properties", context,
                   NI_SERVICE_MAX_PROPERTIES);
        return false;
    }
    properties->count = qdict_size(dict);
    if (properties->count) {
        properties->properties = g_new0(NiProperty, properties->count);
    }
    for (entry = qdict_ordered_first(dict); entry;
         entry = qdict_ordered_next(entry), property_index++) {
        const char *property_name = qdict_entry_key(entry);
        QList *value_list;
        const QListEntry *value_entry;
        NiProperty *property = &properties->properties[property_index];
        size_t value_index = 0;
        g_autofree char *property_context = NULL;
        size_t property_name_length;

        ni_property_init(property);
        property_name_length = strnlen(property_name, NI_SERVICE_MAX_NAME + 1);
        if (!property_name_length) {
            error_setg(errp, "%s property name is empty", context);
            goto fail;
        }
        if (property_name_length > NI_SERVICE_MAX_NAME) {
            error_setg(errp, "%s property name exceeds %u bytes", context,
                       NI_SERVICE_MAX_NAME);
            goto fail;
        }
        property->name = g_strdup(property_name);
        value_list = qobject_to(QList, qdict_entry_value(entry));
        property_context = g_strdup_printf("%s.%s", context, property_name);
        if (!value_list) {
            error_setg(errp, "%s must be an array of strings",
                       property_context);
            goto fail;
        }
        if (qlist_size(value_list) > NI_SERVICE_MAX_VALUES) {
            error_setg(errp, "%s has more than %u values", property_context,
                       NI_SERVICE_MAX_VALUES);
            goto fail;
        }
        property->values.count = qlist_size(value_list);
        if (property->values.count) {
            property->values.values = g_new0(NiName,
                                              property->values.count);
        }
        for (value_entry = qlist_first(value_list); value_entry;
             value_entry = qlist_next(value_entry), value_index++) {
            QString *value_string = qobject_to(QString,
                                                qlist_entry_obj(value_entry));
            const char *value;

            if (!value_string) {
                error_setg(errp, "%s[%zu] must be a string",
                           property_context, value_index);
                goto fail;
            }
            value = qstring_get_str(value_string);
            if (!valid_name(value, true)) {
                error_setg(errp, "%s[%zu] exceeds %u bytes",
                           property_context, value_index,
                           NI_SERVICE_MAX_NAME);
                goto fail;
            }
            property->values.values[value_index] = g_strdup(value);
        }
    }
    return true;

fail:
    ni_property_list_clear(properties);
    return false;
}

static bool json5_parse_node(NetInfoDb *db, const QDict *dict, size_t index,
                             Error **errp)
{
    static const char *const node_keys[] = {
        "id", "instance", "parent", "properties",
    };
    g_autofree char *context = g_strdup_printf("nodes[%zu]", index);
    QDict *properties_dict;
    QObject *parent_object;
    NiPropertyList properties;
    NiId id;
    NiIndex parent = 0;
    bool has_parent;
    NiStatus status;

    if (!json5_check_keys(dict, context, node_keys,
                          G_N_ELEMENTS(node_keys), errp)) {
        return false;
    }
    if (!json5_get_u32(dict, "id", context, &id.nii_object, errp) ||
        !json5_get_u32(dict, "instance", context, &id.nii_instance, errp)) {
        return false;
    }
    if (id.nii_object == UINT32_MAX) {
        error_setg(errp, "%s.id uses the reserved null object value", context);
        return false;
    }

    parent_object = qdict_get(dict, "parent");
    if (parent_object && qobject_type(parent_object) == QTYPE_QNULL) {
        has_parent = false;
    } else {
        has_parent = true;
        if (!json5_get_u32(dict, "parent", context, &parent, errp)) {
            return false;
        }
        if (parent == UINT32_MAX) {
            error_setg(errp, "%s.parent uses the reserved null object value",
                       context);
            return false;
        }
    }

    properties_dict = qobject_to(QDict, qdict_get(dict, "properties"));
    if (!properties_dict) {
        error_setg(errp, "%s.properties must be an object", context);
        return false;
    }
    if (!json5_parse_properties(properties_dict, "node.properties",
                                &properties, errp)) {
        return false;
    }
    status = netinfo_db_add_node(db, &id, has_parent, parent, &properties);
    ni_property_list_clear(&properties);
    if (status != NI_OK) {
        if (status == NI_BADID) {
            error_setg(errp, "%s.id duplicates an earlier node", context);
        } else if (status == NI_NOSPACE) {
            error_setg(errp, "%s exceeds a NetInfo service limit", context);
        } else {
            error_setg(errp, "%s failed validation (status %d)", context,
                       status);
        }
        return false;
    }
    return true;
}

static bool json5_validate_graph(NetInfoDb *db, const char *domain_name,
                                 Error **errp)
{
    size_t roots = 0;

    if (!db->nodes->len) {
        error_setg(errp, "domain.nodes must contain at least one node");
        return false;
    }
    /* Check parent reachability, cycles, and depth before root diagnostics. */
    for (size_t i = 0; i < db->nodes->len; i++) {
        NetInfoDbNode *node = g_ptr_array_index(db->nodes, i);
        NetInfoDbNode *cursor = node;
        g_autoptr(GHashTable) seen = g_hash_table_new(g_direct_hash,
                                                      g_direct_equal);
        size_t depth = 0;

        if (!node->has_parent) {
            roots++;
            continue;
        }
        while (cursor->has_parent) {
            if (depth >= NI_SERVICE_MAX_DEPTH) {
                error_setg(errp, "node %u exceeds parent depth limit %u",
                           node->id.nii_object, NI_SERVICE_MAX_DEPTH);
                return false;
            }
            if (g_hash_table_contains(seen, cursor)) {
                error_setg(errp, "node %u participates in a parent cycle",
                           node->id.nii_object);
                return false;
            }
            g_hash_table_add(seen, cursor);
            {
                NiIndex missing_node = cursor->id.nii_object;
                NiIndex missing_parent = cursor->parent;

                cursor = netinfo_db_find(db, missing_parent);
                if (!cursor) {
                    error_setg(errp, "node %u has dangling parent %u",
                               missing_node, missing_parent);
                    return false;
                }
            }
            depth++;
        }
    }
    if (roots != 1) {
        error_setg(errp, "domain must contain exactly one root, found %zu",
                   roots);
        return false;
    }
    if (!db->root) {
        for (size_t i = 0; i < db->nodes->len; i++) {
            NetInfoDbNode *node = g_ptr_array_index(db->nodes, i);

            if (!node->has_parent) {
                db->root = node;
                break;
            }
        }
    }
    if (!db->root || db->root->id.nii_object != 0) {
        error_setg(errp, "domain root must have object id 0");
        return false;
    }
    if (strcmp(domain_name, "/") != 0) {
        error_setg(errp, "domain.name must be '/' for the network root");
        return false;
    }
    {
        NetInfoDbProperty *name = find_property(db->root, "name");

        if (!name || name->values->len != 1 ||
            strcmp(g_ptr_array_index(name->values, 0), domain_name) != 0) {
            error_setg(errp,
                       "domain.name does not match the root name property");
            return false;
        }
    }
    return true;
}

static char *json5_read_file_bounded(const char *path, size_t *length,
                                     Error **errp)
{
    Error *open_error = NULL;
    char *contents = NULL;
    struct stat file_stat;
    size_t used = 0;
    int fd;

    fd = qemu_open(path, O_RDONLY | O_BINARY, &open_error);
    if (fd < 0) {
        error_propagate(errp, open_error);
        return NULL;
    }
    if (fstat(fd, &file_stat) < 0) {
        error_setg_errno(errp, errno, "failed to stat NetInfo JSON5 '%s'",
                         path);
        qemu_close(fd);
        return NULL;
    }
    if (!S_ISREG(file_stat.st_mode)) {
        error_setg(errp, "NetInfo JSON5 '%s' is not a regular file", path);
        qemu_close(fd);
        return NULL;
    }
    if (file_stat.st_size < 0 ||
        (uintmax_t)file_stat.st_size > QJSON5_MAX_INPUT_SIZE) {
        error_setg(errp, "NetInfo JSON5 '%s' exceeds %u bytes", path,
                   QJSON5_MAX_INPUT_SIZE);
        qemu_close(fd);
        return NULL;
    }

    /* Read one byte beyond the parser limit so growth after fstat is safe. */
    contents = g_malloc((size_t)QJSON5_MAX_INPUT_SIZE + 1);
    while (used <= QJSON5_MAX_INPUT_SIZE) {
        ssize_t read_count;

        do {
#ifdef NETINFO_DB_TESTING
            read_count = json5_read_hook ?
                json5_read_hook(fd, contents + used,
                                QJSON5_MAX_INPUT_SIZE + 1 - used) :
                read(fd, contents + used,
                     QJSON5_MAX_INPUT_SIZE + 1 - used);
#else
            read_count = read(fd, contents + used,
                              QJSON5_MAX_INPUT_SIZE + 1 - used);
#endif
        } while (read_count < 0 && errno == EINTR);
        if (read_count > 0) {
            used += read_count;
            if (used > QJSON5_MAX_INPUT_SIZE) {
                error_setg(errp, "NetInfo JSON5 '%s' exceeds %u bytes", path,
                           QJSON5_MAX_INPUT_SIZE);
                g_free(contents);
                qemu_close(fd);
                return NULL;
            }
            continue;
        }
        if (read_count == 0) {
            break;
        }
        error_setg_errno(errp, errno, "failed to read NetInfo JSON5 '%s'",
                         path);
        g_free(contents);
        qemu_close(fd);
        return NULL;
    }
    qemu_close(fd);
    contents[used] = '\0';
    *length = used;
    return contents;
}

NetInfoDb *netinfo_db_load_json5(const char *path, bool defaults,
                                 Error **errp)
{
    static const char *const root_keys[] = { "version", "domain", "nodes" };
    static const char *const domain_keys[] = { "tag", "name" };
    g_autoptr(NetInfoDb) db = NULL;
    g_autofree char *contents = NULL;
    QObject *document = NULL;
    QDict *root;
    QDict *domain;
    QList *nodes;
    const char *domain_tag;
    const char *domain_name;
    Error *parse_error = NULL;
    size_t length = 0;

    if (!path) {
        if (!defaults) {
            error_setg(errp,
                       "NetInfo JSON5 path is required when defaults are disabled");
            return NULL;
        }
        db = netinfo_db_new_default();
        if (!db) {
            error_setg(errp, "failed to construct the safe NetInfo default");
        }
        return g_steal_pointer(&db);
    }
    contents = json5_read_file_bounded(path, &length, errp);
    if (!contents) {
        return NULL;
    }
    document = qobject_from_json5(contents, length, &parse_error);
    if (!document) {
        if (parse_error) {
            error_setg(errp, "failed to parse NetInfo JSON5 '%s': %s", path,
                       error_get_pretty(parse_error));
            error_free(parse_error);
        } else {
            error_setg(errp, "failed to parse NetInfo JSON5 '%s'", path);
        }
        return NULL;
    }
    root = qobject_to(QDict, document);
    if (!root || !json5_check_keys(root, "domain file", root_keys,
                                   G_N_ELEMENTS(root_keys), errp)) {
        if (!root) {
            error_setg(errp, "NetInfo JSON5 '%s' must contain an object", path);
        }
        goto fail;
    }
    {
        QNum *version = qobject_to(QNum, qdict_get(root, "version"));
        uint64_t version_value;

        if (!version || !qnum_get_try_uint(version, &version_value) ||
            version_value != 1) {
            error_setg(errp, "unsupported NetInfo JSON5 version (expected 1)");
            goto fail;
        }
    }
    domain = qobject_to(QDict, qdict_get(root, "domain"));
    if (!domain) {
        error_setg(errp, "domain must be an object");
        goto fail;
    }
    if (!json5_check_keys(domain, "domain", domain_keys,
                          G_N_ELEMENTS(domain_keys), errp)) {
        goto fail;
    }
    domain_tag = json5_get_string(domain, "tag", "domain", false, errp);
    if (!domain_tag) {
        goto fail;
    }
    domain_name = json5_get_string(domain, "name", "domain", true, errp);
    if (!domain_name) {
        goto fail;
    }
    if (strcmp(domain_name, "/") != 0) {
        error_setg(errp, "domain.name must be '/' for the network root");
        goto fail;
    }
    db = netinfo_db_new_with_tag(domain_tag);
    if (!db) {
        error_setg(errp, "domain.tag is invalid");
        goto fail;
    }
    nodes = qobject_to(QList, qdict_get(root, "nodes"));
    if (!nodes) {
        error_setg(errp, "domain file.nodes must be an array");
        goto fail;
    }
    if (qlist_size(nodes) > NI_SERVICE_MAX_NODES) {
        error_setg(errp, "domain file.nodes has more than %u nodes",
                   NI_SERVICE_MAX_NODES);
        goto fail;
    }
    {
        const QListEntry *entry;
        size_t index = 0;

        for (entry = qlist_first(nodes); entry;
             entry = qlist_next(entry), index++) {
            QDict *node = qobject_to(QDict, qlist_entry_obj(entry));

            if (!node || !json5_parse_node(db, node, index, errp)) {
                if (!node) {
                    error_setg(errp, "nodes[%zu] must be an object", index);
                }
                goto fail;
            }
        }
    }
    if (!json5_validate_graph(db, domain_name, errp)) {
        goto fail;
    }
    if (netinfo_db_seal(db) != NI_OK) {
        error_setg(errp, "failed to seal the NetInfo domain");
        goto fail;
    }
    qobject_unref(document);
    return g_steal_pointer(&db);

fail:
    qobject_unref(document);
    return NULL;
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
