/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "hw/netinfo/netinfo-db.h"
#include "hw/netinfo/netinfo-xdr.h"

static NiPropertyList props(NiProperty *properties, size_t count)
{
    return (NiPropertyList) {
        .count = count,
        .properties = properties,
    };
}

static NiProperty property(char *name, NiName *values, size_t count)
{
    return (NiProperty) {
        .name = name,
        .values = {
            .count = count,
            .values = values,
        },
    };
}

static NetInfoDb *build_test_db(void)
{
    static NiName root_name[] = { (char *)"/" };
    static NiName root_master[] = { (char *)"localhost/network" };
    static NiName machines_name[] = { (char *)"machines" };
    static NiName host_name[] = { (char *)"localhost" };
    static NiName host_ip[] = { (char *)"10.0.2.2" };
    static NiName host_serves[] = { (char *)"./network" };
    static NiProperty root_properties[2];
    static NiProperty machines_properties[1];
    static NiProperty host_properties[3];
    static bool initialized;
    NiPropertyList root_list;
    NiPropertyList machines_list;
    NiPropertyList host_list;
    g_autoptr(NetInfoDb) db = NULL;

    if (!initialized) {
        root_properties[0] = property((char *)"name", root_name, 1);
        root_properties[1] = property((char *)"master", root_master, 1);
        machines_properties[0] = property((char *)"name", machines_name, 1);
        host_properties[0] = property((char *)"name", host_name, 1);
        host_properties[1] = property((char *)"ip_address", host_ip, 1);
        host_properties[2] = property((char *)"serves", host_serves, 1);
        initialized = true;
    }

    root_list = props(root_properties, G_N_ELEMENTS(root_properties));
    machines_list = props(machines_properties,
                          G_N_ELEMENTS(machines_properties));
    host_list = props(host_properties, G_N_ELEMENTS(host_properties));

    db = netinfo_db_new();
    g_assert_nonnull(db);
    g_assert_cmpint(netinfo_db_add_node(
                        db, &(NiId) { .nii_object = 0, .nii_instance = 1 },
                        false, 0, &root_list),
                    ==, NI_OK);
    g_assert_cmpint(netinfo_db_add_node(
                        db, &(NiId) { .nii_object = 1, .nii_instance = 1 },
                        true, 0, &machines_list),
                    ==, NI_OK);
    g_assert_cmpint(netinfo_db_add_node(
                        db, &(NiId) { .nii_object = 2, .nii_instance = 1 },
                        true, 1, &host_list),
                    ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);
    return g_steal_pointer(&db);
}

static void test_root_self_parent_children_read(void)
{
    g_autoptr(NetInfoDb) db = build_test_db();
    NiId id = { .nii_object = 99, .nii_instance = 99 };
    NiId parent = { 0 };
    NiIdList children;
    NiPropertyList result;

    ni_id_list_init(&children);
    ni_property_list_init(&result);

    g_assert_cmpint(netinfo_db_root(db, &id), ==, NI_OK);
    g_assert_cmpuint(id.nii_object, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 1);

    id = (NiId) { .nii_object = 2, .nii_instance = 0xfeed };
    g_assert_cmpint(netinfo_db_self(db, &id), ==, NI_OK);
    g_assert_cmpuint(id.nii_instance, ==, 1);

    g_assert_cmpint(netinfo_db_parent(db, &id, &parent), ==, NI_OK);
    g_assert_cmpuint(parent.nii_object, ==, 1);
    g_assert_cmpuint(parent.nii_instance, ==, 1);
    g_assert_cmpuint(id.nii_object, ==, 2);
    g_assert_cmpuint(id.nii_instance, ==, 1);

    id = (NiId) { .nii_object = 1, .nii_instance = 42 };
    g_assert_cmpint(netinfo_db_children(db, &id, &children), ==, NI_OK);
    g_assert_cmpuint(id.nii_instance, ==, 1);
    g_assert_cmpuint(children.count, ==, 1);
    g_assert_cmpuint(children.values[0], ==, 2);

    id = (NiId) { .nii_object = 2, .nii_instance = 0 };
    g_assert_cmpint(netinfo_db_read(db, &id, &result), ==, NI_OK);
    g_assert_cmpuint(result.count, ==, 3);
    g_assert_cmpstr(result.properties[0].name, ==, "name");
    g_assert_cmpstr(result.properties[0].values.values[0], ==, "localhost");
    g_assert_cmpstr(result.properties[1].name, ==, "ip_address");
    g_assert_cmpstr(result.properties[2].name, ==, "serves");

    ni_id_list_clear(&children);
    ni_property_list_clear(&result);
}

static void test_lookup_and_lookupread_are_exact_and_ordered(void)
{
    static NiName root_name[] = { (char *)"/" };
    static NiName child_a_name[] = { (char *)"a" };
    static NiName child_b_name[] = { (char *)"b" };
    static NiName duplicate_values[] = { (char *)"same", (char *)"same" };
    NiProperty root_prop = property((char *)"name", root_name, 1);
    NiProperty child_a_props[] = {
        property((char *)"name", child_a_name, 1),
        property((char *)"alias", duplicate_values,
                 G_N_ELEMENTS(duplicate_values)),
    };
    NiProperty child_b_props[] = {
        property((char *)"name", child_b_name, 1),
        property((char *)"alias", duplicate_values,
                 G_N_ELEMENTS(duplicate_values)),
    };
    NiPropertyList root_list = props(&root_prop, 1);
    NiPropertyList a_list = props(child_a_props, G_N_ELEMENTS(child_a_props));
    NiPropertyList b_list = props(child_b_props, G_N_ELEMENTS(child_b_props));
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id = { .nii_object = 0, .nii_instance = 1 };
    NiId found = { 0 };
    NiIdList ids;
    NiPropertyList read;

    ni_id_list_init(&ids);
    ni_property_list_init(&read);
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root_list), ==,
                    NI_OK);
    id = (NiId) { .nii_object = 4, .nii_instance = 7 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0, &a_list), ==,
                    NI_OK);
    id = (NiId) { .nii_object = 3, .nii_instance = 8 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0, &b_list), ==,
                    NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);

    id = (NiId) { .nii_object = 0, .nii_instance = 999 };
    g_assert_cmpint(netinfo_db_lookup(db, &id, "alias", "same", &ids),
                    ==, NI_OK);
    g_assert_cmpuint(ids.count, ==, 2);
    g_assert_cmpuint(ids.values[0], ==, 4);
    g_assert_cmpuint(ids.values[1], ==, 3);
    g_assert_cmpuint(id.nii_instance, ==, 1);

    ni_id_list_clear(&ids);
    id = (NiId) { .nii_object = 4, .nii_instance = 0 };
    g_assert_cmpint(netinfo_db_read(db, &id, &read), ==, NI_OK);
    g_assert_cmpuint(read.properties[1].values.count, ==, 2);
    g_assert_cmpstr(read.properties[1].values.values[0], ==, "same");
    g_assert_cmpstr(read.properties[1].values.values[1], ==, "same");
    ni_property_list_clear(&read);

    id = (NiId) { .nii_object = 0, .nii_instance = 999 };
    g_assert_cmpint(netinfo_db_lookup_read(db, &id, "name", "b", &found,
                                           &read), ==, NI_OK);
    g_assert_cmpuint(found.nii_object, ==, 3);
    g_assert_cmpuint(found.nii_instance, ==, 8);
    g_assert_cmpuint(read.count, ==, 2);
    g_assert_cmpstr(read.properties[0].name, ==, "name");
    g_assert_cmpstr(read.properties[0].values.values[0], ==, "b");
    g_assert_cmpstr(read.properties[1].name, ==, "alias");

    ni_property_list_clear(&read);
}

static void test_list_and_property_name_projections(void)
{
    g_autoptr(NetInfoDb) db = build_test_db();
    NiId id = { .nii_object = 1, .nii_instance = 0 };
    NiEntryList entries;
    NiNameList names;
    NiNameList values;
    NiName value = NULL;

    ni_entry_list_init(&entries);
    ni_name_list_init(&names);
    ni_name_list_init(&values);

    g_assert_cmpint(netinfo_db_list(db, &id, "name", &entries), ==, NI_OK);
    g_assert_cmpuint(entries.count, ==, 1);
    g_assert_true(entries.entries[0].has_names);
    g_assert_cmpuint(entries.entries[0].names.count, ==, 1);
    g_assert_cmpstr(entries.entries[0].names.values[0], ==, "localhost");
    g_assert_cmpuint(id.nii_instance, ==, 1);
    ni_entry_list_clear(&entries);

    g_assert_cmpint(netinfo_db_list(db, &id, "ip_address", &entries), ==,
                    NI_OK);
    g_assert_true(entries.entries[0].has_names);
    g_assert_cmpuint(entries.entries[0].names.count, ==, 1);
    g_assert_cmpstr(entries.entries[0].names.values[0], ==, "10.0.2.2");
    ni_entry_list_clear(&entries);

    id = (NiId) { .nii_object = 1, .nii_instance = 0 };
    g_assert_cmpint(netinfo_db_list(db, &id, "not_present", &entries), ==,
                    NI_OK);
    g_assert_cmpuint(entries.count, ==, 1);
    g_assert_false(entries.entries[0].has_names);
    ni_entry_list_clear(&entries);

    id = (NiId) { .nii_object = 2, .nii_instance = 99 };
    g_assert_cmpint(netinfo_db_listprops(db, &id, &names), ==, NI_OK);
    g_assert_cmpuint(names.count, ==, 3);
    g_assert_cmpstr(names.values[0], ==, "name");
    g_assert_cmpstr(names.values[1], ==, "ip_address");
    g_assert_cmpstr(names.values[2], ==, "serves");
    g_assert_cmpuint(id.nii_instance, ==, 1);

    id = (NiId) { .nii_object = 2, .nii_instance = 99 };
    g_assert_cmpint(netinfo_db_readprop(db, &id, 1, &values), ==, NI_OK);
    g_assert_cmpuint(values.count, ==, 1);
    g_assert_cmpstr(values.values[0], ==, "10.0.2.2");
    g_assert_cmpuint(id.nii_instance, ==, 1);
    id = (NiId) { .nii_object = 2, .nii_instance = 99 };
    g_assert_cmpint(netinfo_db_readname(db, &id, 1, 0, &value), ==, NI_OK);
    g_assert_cmpstr(value, ==, "10.0.2.2");
    g_assert_cmpuint(id.nii_instance, ==, 1);

    g_free(value);
    ni_entry_list_clear(&entries);
    ni_name_list_clear(&names);
    ni_name_list_clear(&values);
}

static void test_missing_ids_properties_and_names(void)
{
    g_autoptr(NetInfoDb) db = build_test_db();
    NiId id = { .nii_object = 99, .nii_instance = 1 };
    NiId found = { 0 };
    NiId parent;
    NiIdList children;
    NiPropertyList properties;
    NiNameList names;
    NiName value = NULL;

    ni_id_list_init(&children);
    ni_property_list_init(&properties);
    ni_name_list_init(&names);
    g_assert_cmpint(netinfo_db_self(db, &id), ==, NI_BADID);
    g_assert_cmpint(netinfo_db_parent(db, &id, &parent), ==, NI_BADID);
    g_assert_cmpint(netinfo_db_children(db, &id, &children), ==, NI_BADID);
    g_assert_cmpint(netinfo_db_read(db, &id, &properties), ==, NI_BADID);

    id = (NiId) { .nii_object = 2, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_readprop(db, &id, 99, &names), ==, NI_NOPROP);
    g_assert_cmpint(netinfo_db_readname(db, &id, 99, 0, &value), ==,
                    NI_NOPROP);
    g_assert_cmpint(netinfo_db_readname(db, &id, 1, 99, &value), ==,
                    NI_NONAME);
    g_assert_cmpint(netinfo_db_lookup(db, &id, "missing", "x", &children),
                    ==, NI_NODIR);
    id = (NiId) { .nii_object = 0, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_lookup(db, &id, "name", "not-there",
                                      &children), ==, NI_NODIR);
    g_assert_cmpint(netinfo_db_lookup_read(db, &id, "name", "not-there",
                                           &found, &properties), ==, NI_NODIR);

    ni_id_list_clear(&children);
    ni_property_list_clear(&properties);
    ni_name_list_clear(&names);
    g_free(value);
}

static void test_root_and_parent_errors(void)
{
    g_autoptr(NetInfoDb) db = build_test_db();
    NiId id = { .nii_object = 0, .nii_instance = 99 };
    NiId parent = { .nii_object = 99, .nii_instance = 98 };
    NiIndex parent_index = 99;

    g_assert_cmpint(netinfo_db_parent(db, &id, &parent), ==, NI_OK);
    g_assert_cmpuint(id.nii_object, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 1);
    g_assert_cmpuint(parent.nii_object, ==, 0);
    g_assert_cmpuint(parent.nii_instance, ==, 1);

    id = (NiId) { .nii_object = 0, .nii_instance = 99 };
    g_assert_cmpint(netinfo_db_parent_index(db, &id, &parent_index), ==,
                    NI_OK);
    g_assert_cmpuint(parent_index, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 1);
}

static void init_idlist_sentinel(NiIdList *list)
{
    ni_id_list_init(list);
    list->count = 1;
    list->values = g_new(NiIndex, 1);
    list->values[0] = 77;
}

static void init_namelist_sentinel(NiNameList *list)
{
    ni_name_list_init(list);
    list->count = 1;
    list->values = g_new0(NiName, 1);
    list->values[0] = g_strdup("keep");
}

static void init_property_list_sentinel(NiPropertyList *list)
{
    ni_property_list_init(list);
    list->count = 1;
    list->properties = g_new0(NiProperty, 1);
    ni_property_init(&list->properties[0]);
    list->properties[0].name = g_strdup("sentinel");
    list->properties[0].values.count = 1;
    list->properties[0].values.values = g_new0(NiName, 1);
    list->properties[0].values.values[0] = g_strdup("keep");
}

static void init_entry_list_sentinel(NiEntryList *list)
{
    ni_entry_list_init(list);
    list->count = 1;
    list->entries = g_new0(NiEntry, 1);
    ni_entry_init(&list->entries[0]);
    list->entries[0].id = 77;
    list->entries[0].has_names = true;
    list->entries[0].names.count = 1;
    list->entries[0].names.values = g_new0(NiName, 1);
    list->entries[0].names.values[0] = g_strdup("keep");
}

static void assert_idlist_sentinel(const NiIdList *list)
{
    g_assert_cmpuint(list->count, ==, 1);
    g_assert_cmpuint(list->values[0], ==, 77);
}

static void assert_namelist_sentinel(const NiNameList *list)
{
    g_assert_cmpuint(list->count, ==, 1);
    g_assert_cmpstr(list->values[0], ==, "keep");
}

static void assert_property_list_sentinel(const NiPropertyList *list)
{
    g_assert_cmpuint(list->count, ==, 1);
    g_assert_cmpstr(list->properties[0].name, ==, "sentinel");
    g_assert_cmpuint(list->properties[0].values.count, ==, 1);
    g_assert_cmpstr(list->properties[0].values.values[0], ==, "keep");
}

static void assert_entry_list_sentinel(const NiEntryList *list)
{
    g_assert_cmpuint(list->count, ==, 1);
    g_assert_cmpuint(list->entries[0].id, ==, 77);
    g_assert_true(list->entries[0].has_names);
    g_assert_cmpuint(list->entries[0].names.count, ==, 1);
    g_assert_cmpstr(list->entries[0].names.values[0], ==, "keep");
}

static void test_errors_preserve_prepopulated_outputs(void)
{
    g_autoptr(NetInfoDb) db = build_test_db();
    NiId id;
    NiId found;
    NiId parent;
    NiIdList ids;
    NiNameList names;
    NiPropertyList properties;
    NiEntryList entries;
    NiName name;

    init_idlist_sentinel(&ids);
    init_namelist_sentinel(&names);
    init_property_list_sentinel(&properties);
    init_entry_list_sentinel(&entries);
    ni_name_init(&name);
    name = g_strdup("keep");

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    parent = (NiId) { .nii_object = 88, .nii_instance = 87 };
    g_assert_cmpint(netinfo_db_parent(db, &id, &parent), ==, NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    g_assert_cmpuint(parent.nii_object, ==, 88);
    g_assert_cmpuint(parent.nii_instance, ==, 87);

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_children(db, &id, &ids), ==, NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_idlist_sentinel(&ids);

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_read(db, &id, &properties), ==, NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_property_list_sentinel(&properties);

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_list(db, &id, "name", &entries), ==,
                    NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_entry_list_sentinel(&entries);

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_listprops(db, &id, &names), ==, NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_namelist_sentinel(&names);

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_lookup(db, &id, "name", "x", &ids), ==,
                    NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_idlist_sentinel(&ids);

    id = (NiId) { .nii_object = 99, .nii_instance = 98 };
    found = (NiId) { .nii_object = 88, .nii_instance = 87 };
    g_assert_cmpint(netinfo_db_lookup_read(db, &id, "name", "x", &found,
                                           &properties), ==, NI_BADID);
    g_assert_cmpuint(id.nii_object, ==, 99);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    g_assert_cmpuint(found.nii_object, ==, 88);
    g_assert_cmpuint(found.nii_instance, ==, 87);
    assert_property_list_sentinel(&properties);

    id = (NiId) { .nii_object = 2, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_readprop(db, &id, 99, &names), ==, NI_NOPROP);
    g_assert_cmpuint(id.nii_object, ==, 2);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_namelist_sentinel(&names);

    id = (NiId) { .nii_object = 2, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_readname(db, &id, 99, 0, &name), ==,
                    NI_NOPROP);
    g_assert_cmpuint(id.nii_object, ==, 2);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    g_assert_cmpstr(name, ==, "keep");

    id = (NiId) { .nii_object = 2, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_readname(db, &id, 1, 99, &name), ==,
                    NI_NONAME);
    g_assert_cmpuint(id.nii_object, ==, 2);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    g_assert_cmpstr(name, ==, "keep");

    id = (NiId) { .nii_object = 0, .nii_instance = 98 };
    g_assert_cmpint(netinfo_db_lookup(db, &id, "name", "x", &ids), ==,
                    NI_NODIR);
    g_assert_cmpuint(id.nii_object, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    assert_idlist_sentinel(&ids);

    found = (NiId) { .nii_object = 88, .nii_instance = 87 };
    g_assert_cmpint(netinfo_db_lookup_read(db, &id, "name", "x", &found,
                                           &properties), ==, NI_NODIR);
    g_assert_cmpuint(id.nii_object, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 98);
    g_assert_cmpuint(found.nii_object, ==, 88);
    g_assert_cmpuint(found.nii_instance, ==, 87);
    assert_property_list_sentinel(&properties);

    ni_id_list_clear(&ids);
    ni_name_list_clear(&names);
    ni_property_list_clear(&properties);
    ni_entry_list_clear(&entries);
    ni_name_clear(&name);
}

static void test_construction_is_strict_and_transactional(void)
{
    static NiName root_name[] = { (char *)"/" };
    static NiProperty root_prop = {
        .name = (char *)"name",
        .values = { .count = 1, .values = root_name },
    };
    NiPropertyList root = props(&root_prop, 1);
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    g_autoptr(NetInfoDb) tagged = NULL;
    g_autofree char *long_tag = g_malloc0(NI_SERVICE_MAX_NAME + 2);
    NiId id = { .nii_object = 0, .nii_instance = 1 };

    g_assert_null(netinfo_db_new_with_tag(NULL));
    g_assert_null(netinfo_db_new_with_tag(""));
    memset(long_tag, 'x', NI_SERVICE_MAX_NAME + 1);
    g_assert_null(netinfo_db_new_with_tag(long_tag));
    tagged = netinfo_db_new_with_tag("custom");
    g_assert_nonnull(tagged);
    g_assert_cmpstr(netinfo_db_tag(tagged), ==, "custom");

    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==, NI_OK);
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==,
                    NI_BADID);
    id = (NiId) { .nii_object = 1, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 99, &root), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);
}

static void test_default_network_domain(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new_default();
    NiId id = { 0 };
    NiIdList children;
    NiPropertyList properties;

    ni_id_list_init(&children);
    ni_property_list_init(&properties);
    g_assert_nonnull(db);
    g_assert_cmpint(netinfo_db_root(db, &id), ==, NI_OK);
    g_assert_cmpuint(id.nii_object, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 0x24);
    g_assert_cmpint(netinfo_db_read(db, &id, &properties), ==, NI_OK);
    g_assert_cmpuint(properties.count, ==, 2);
    g_assert_cmpstr(properties.properties[0].name, ==, "name");
    g_assert_cmpstr(properties.properties[0].values.values[0], ==, "/");
    g_assert_cmpstr(properties.properties[1].name, ==, "master");
    g_assert_cmpstr(properties.properties[1].values.values[0], ==,
                    "localhost/network");
    g_assert_cmpint(netinfo_db_children(db, &id, &children), ==, NI_OK);
    g_assert_cmpuint(children.count, ==, 1);
    g_assert_cmpuint(children.values[0], ==, 1);

    id = (NiId) { .nii_object = 0, .nii_instance = 0 };
    g_assert_cmpint(netinfo_db_lookup(db, &id, "name", "machines",
                                      &children), ==, NI_OK);
    g_assert_cmpuint(children.count, ==, 1);
    g_assert_cmpuint(children.values[0], ==, 1);
    id = (NiId) { .nii_object = 1, .nii_instance = 0 };
    g_assert_cmpint(netinfo_db_lookup(db, &id, "name", "localhost",
                                      &children), ==, NI_OK);
    g_assert_cmpuint(children.count, ==, 1);
    g_assert_cmpuint(children.values[0], ==, 2);
    id = (NiId) { .nii_object = 2, .nii_instance = 0 };
    ni_property_list_clear(&properties);
    g_assert_cmpint(netinfo_db_read(db, &id, &properties), ==, NI_OK);
    g_assert_cmpuint(properties.count, ==, 3);
    g_assert_cmpstr(properties.properties[1].name, ==, "ip_address");
    g_assert_cmpstr(properties.properties[1].values.values[0], ==,
                    "10.0.2.2");
    g_assert_cmpstr(properties.properties[2].name, ==, "serves");
    g_assert_cmpstr(properties.properties[2].values.values[0], ==,
                    "./network");
    ni_id_list_clear(&children);
    ni_property_list_clear(&properties);
}

static void test_input_ownership_and_service_limits(void)
{
    char property_name[] = "name";
    char root_value[] = "/";
    NiName values[] = { root_value };
    NiProperty root_property = {
        .name = property_name,
        .values = { .count = 1, .values = values },
    };
    NiPropertyList root = props(&root_property, 1);
    NiId root_id = { .nii_object = 0, .nii_instance = 1 };
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    g_autofree char *long_name = g_malloc0(NI_SERVICE_MAX_NAME + 2);
    NiProperty long_property;
    NiPropertyList long_list;
    NiProperty *many_properties;
    NiPropertyList many_property_list;
    NiName *many_values;
    NiProperty many_values_property;
    NiPropertyList many_values_list;
    NiPropertyList read;
    NiId id = root_id;

    g_assert_cmpint(netinfo_db_add_node(db, &root_id, false, 0, &root), ==,
                    NI_OK);
    property_name[0] = 'x';
    root_value[0] = 'x';
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);
    ni_property_list_init(&read);
    g_assert_cmpint(netinfo_db_read(db, &id, &read), ==, NI_OK);
    g_assert_cmpstr(read.properties[0].name, ==, "name");
    g_assert_cmpstr(read.properties[0].values.values[0], ==, "/");
    ni_property_list_clear(&read);

    memset(long_name, 'x', NI_SERVICE_MAX_NAME + 1);
    long_property = property(long_name, NULL, 0);
    long_list = props(&long_property, 1);
    netinfo_db_free(g_steal_pointer(&db));
    db = netinfo_db_new();
    g_assert_cmpint(netinfo_db_add_node(db, &root_id, false, 0, &long_list),
                    ==, NI_NOSPACE);

    many_properties = g_new0(NiProperty, NI_SERVICE_MAX_PROPERTIES + 1);
    many_property_list = props(many_properties,
                               NI_SERVICE_MAX_PROPERTIES + 1);
    g_assert_cmpint(netinfo_db_add_node(db, &root_id, false, 0,
                                        &many_property_list), ==, NI_NOSPACE);
    /* A distinct object exercises the property-count limit itself. */
    id = (NiId) { .nii_object = 1, .nii_instance = 1 };
    for (size_t i = 0; i < NI_SERVICE_MAX_PROPERTIES + 1; i++) {
        many_properties[i].name = (char *)"p";
    }
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0,
                                        &many_property_list), ==, NI_NOSPACE);
    g_free(many_properties);

    many_values = g_new0(NiName, NI_SERVICE_MAX_VALUES + 1);
    for (size_t i = 0; i < NI_SERVICE_MAX_VALUES + 1; i++) {
        many_values[i] = (char *)"v";
    }
    many_values_property = property((char *)"name", many_values,
                                    NI_SERVICE_MAX_VALUES + 1);
    many_values_list = props(&many_values_property, 1);
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0,
                                        &many_values_list), ==, NI_NOSPACE);
    g_free(many_values);
}

static void test_parent_validation_and_sealed_mutation(void)
{
    static NiName root_name[] = { (char *)"/" };
    static NiProperty root_property = {
        .name = (char *)"name",
        .values = { .count = 1, .values = root_name },
    };
    NiPropertyList root = props(&root_property, 1);
    NiId id;
    g_autoptr(NetInfoDb) db = netinfo_db_new();

    id = (NiId) { .nii_object = 0, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==, NI_OK);
    id = (NiId) { .nii_object = 1, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 99, &root), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);

    netinfo_db_clear(db);
    id = (NiId) { .nii_object = 0, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==, NI_OK);
    id = (NiId) { .nii_object = 1, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 2, &root), ==, NI_OK);
    id = (NiId) { .nii_object = 2, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 1, &root), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);

    netinfo_db_clear(db);
    id = (NiId) { .nii_object = 0, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);
    id = (NiId) { .nii_object = 1, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0, &root), ==,
                    NI_RDONLY);
}

static NiPropertyList strict_root_properties(void)
{
    static NiName root_name[] = { (char *)"/" };
    static NiProperty root_property = {
        .name = (char *)"name",
        .values = { .count = 1, .values = root_name },
    };

    return props(&root_property, 1);
}

static void add_strict_root(NetInfoDb *db, NiIndex instance)
{
    NiPropertyList root = strict_root_properties();
    NiId id = { .nii_object = 0, .nii_instance = instance };

    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==,
                    NI_OK);
}

static void test_strict_seal_rejects_empty_database(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();

    g_assert_cmpint(netinfo_db_seal(db), ==, NI_NODIR);
    g_assert_cmpuint(netinfo_db_node_count(db), ==, 0);

    add_strict_root(db, 1);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);
}

static void test_strict_seal_rejects_database_without_root(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id = { .nii_object = 1, .nii_instance = 1 };

    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0, NULL), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);
}

static void test_strict_seal_rejects_multiple_roots(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id = { .nii_object = 1, .nii_instance = 1 };

    add_strict_root(db, 1);
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, NULL), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);
}

static void test_strict_seal_rejects_nonzero_object_root(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiPropertyList root = strict_root_properties();
    NiId id = { .nii_object = 1, .nii_instance = 1 };

    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, &root), ==,
                    NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);
}

static void test_strict_seal_rejects_malformed_root_name(void)
{
    static NiName malformed_name[] = { (char *)"not-root" };
    static NiProperty malformed_property = {
        .name = (char *)"name",
        .values = { .count = 1, .values = malformed_name },
    };
    NiPropertyList malformed_root = props(&malformed_property, 1);
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id = { .nii_object = 0, .nii_instance = 1 };

    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0,
                                        &malformed_root), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);
}

static void test_strict_add_rejects_null_object_sentinel(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id = { .nii_object = UINT32_MAX, .nii_instance = 1 };

    /* NiIndex's all-ones value is the historical NI_INDEX_NULL sentinel. */
    g_assert_cmpuint((NiIndex) -1, ==, UINT32_MAX);
    g_assert_cmpint(netinfo_db_add_node(db, &id, false, 0, NULL), ==,
                    NI_BADID);
    g_assert_cmpuint(netinfo_db_node_count(db), ==, 0);
}

static void add_depth_chain(NetInfoDb *db, size_t last_object)
{
    NiId id;

    add_strict_root(db, 1);
    for (size_t object = 1; object <= last_object; object++) {
        id = (NiId) {
            .nii_object = object,
            .nii_instance = 1,
        };
        g_assert_cmpint(netinfo_db_add_node(db, &id, true, object - 1,
                                            NULL), ==, NI_OK);
    }
}

static void test_strict_seal_enforces_depth_limit(void)
{
    g_autoptr(NetInfoDb) exact = netinfo_db_new();
    g_autoptr(NetInfoDb) over = netinfo_db_new();

    add_depth_chain(exact, NI_SERVICE_MAX_DEPTH);
    g_assert_cmpint(netinfo_db_seal(exact), ==, NI_OK);

    add_depth_chain(over, NI_SERVICE_MAX_DEPTH + 1);
    g_assert_cmpint(netinfo_db_seal(over), ==, NI_NOSPACE);
}

static void test_strict_seal_retry_after_adding_missing_parent(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id;
    NiIdList children;

    add_strict_root(db, 1);
    id = (NiId) { .nii_object = 1, .nii_instance = 1 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 77, NULL), ==,
                    NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_UNRELATED);

    id = (NiId) { .nii_object = 77, .nii_instance = 2 };
    g_assert_cmpint(netinfo_db_add_node(db, &id, true, 0, NULL), ==, NI_OK);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);

    ni_id_list_init(&children);
    id = (NiId) { .nii_object = 0, .nii_instance = 99 };
    g_assert_cmpint(netinfo_db_children(db, &id, &children), ==, NI_OK);
    g_assert_cmpuint(id.nii_instance, ==, 1);
    g_assert_cmpuint(children.count, ==, 1);
    g_assert_cmpuint(children.values[0], ==, 77);
    ni_id_list_clear(&children);
}

static void test_strict_clear_reuses_sealed_database(void)
{
    g_autoptr(NetInfoDb) db = netinfo_db_new();
    NiId id;

    add_strict_root(db, 1);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);
    netinfo_db_clear(db);
    g_assert_cmpuint(netinfo_db_node_count(db), ==, 0);

    add_strict_root(db, 2);
    g_assert_cmpint(netinfo_db_seal(db), ==, NI_OK);
    id = (NiId) { .nii_object = 0, .nii_instance = 99 };
    g_assert_cmpint(netinfo_db_root(db, &id), ==, NI_OK);
    g_assert_cmpuint(id.nii_object, ==, 0);
    g_assert_cmpuint(id.nii_instance, ==, 2);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/netinfo-db/tree/root-self-parent-children-read",
                    test_root_self_parent_children_read);
    g_test_add_func("/netinfo-db/tree/lookup-lookupread",
                    test_lookup_and_lookupread_are_exact_and_ordered);
    g_test_add_func("/netinfo-db/tree/list-projections",
                    test_list_and_property_name_projections);
    g_test_add_func("/netinfo-db/errors/missing-values",
                    test_missing_ids_properties_and_names);
    g_test_add_func("/netinfo-db/errors/root-parent",
                    test_root_and_parent_errors);
    g_test_add_func("/netinfo-db/errors/preserve-prepopulated-outputs",
                    test_errors_preserve_prepopulated_outputs);
    g_test_add_func("/netinfo-db/construction/strict",
                    test_construction_is_strict_and_transactional);
    g_test_add_func("/netinfo-db/default/network-domain",
                    test_default_network_domain);
    g_test_add_func("/netinfo-db/construction/ownership-limits",
                    test_input_ownership_and_service_limits);
    g_test_add_func("/netinfo-db/construction/parent-validation",
                    test_parent_validation_and_sealed_mutation);
    g_test_add_func("/netinfo-db/construction/strict-empty",
                    test_strict_seal_rejects_empty_database);
    g_test_add_func("/netinfo-db/construction/strict-no-root",
                    test_strict_seal_rejects_database_without_root);
    g_test_add_func("/netinfo-db/construction/strict-multiple-roots",
                    test_strict_seal_rejects_multiple_roots);
    g_test_add_func("/netinfo-db/construction/strict-nonzero-root",
                    test_strict_seal_rejects_nonzero_object_root);
    g_test_add_func("/netinfo-db/construction/strict-malformed-root",
                    test_strict_seal_rejects_malformed_root_name);
    g_test_add_func("/netinfo-db/construction/strict-null-object",
                    test_strict_add_rejects_null_object_sentinel);
    g_test_add_func("/netinfo-db/construction/strict-depth",
                    test_strict_seal_enforces_depth_limit);
    g_test_add_func("/netinfo-db/construction/strict-seal-retry",
                    test_strict_seal_retry_after_adding_missing_parent);
    g_test_add_func("/netinfo-db/construction/strict-clear-reuse",
                    test_strict_clear_reuses_sealed_database);
    return g_test_run();
}
