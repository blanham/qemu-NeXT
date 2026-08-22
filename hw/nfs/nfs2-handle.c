#include "qemu/osdep.h"

#include "crypto/hmac.h"
#include "crypto/hash.h"
#include "crypto/random.h"
#include "hw/nfs/nfs2-handle.h"
#include "qapi/error.h"
#include "qemu/bswap.h"

#define NFS2_HANDLE_KEY_SIZE 32
#define NFS2_HANDLE_MAC_SIZE 12
#define NFS2_HANDLE_AUTHENTICATED_SIZE 20
#define NFS2_HANDLE_VERSION 1

typedef struct Nfs2HandleRecord {
    uint64_t id;
    uint32_t generation;
    GPtrArray *aliases;
} Nfs2HandleRecord;

typedef struct Nfs2RenameChange {
    V9fsPath *alias;
    Nfs2HandleRecord *record;
    char *new_path;
} Nfs2RenameChange;

struct Nfs2HandleTable {
    uint8_t key[NFS2_HANDLE_KEY_SIZE];
    GHashTable *by_id;
    GHashTable *by_path;
    size_t alias_count;
    uint32_t next_generation;
    bool generation_exhausted;
#ifdef NFS2_HANDLE_TESTING
    bool fail_next_mac;
#endif
    bool active;
};

static void secure_clear(void *data, size_t length)
{
    /* Volatile prevents the compiler from eliding secret-data erasure. */
    volatile uint8_t *p = data;

    while (length--) {
        *p++ = 0;
    }
}

static void path_free(gpointer opaque)
{
    V9fsPath *path = opaque;

    if (path) {
        g_free(path->data);
        g_free(path);
    }
}

static V9fsPath *path_new(const char *value)
{
    V9fsPath *path = g_new0(V9fsPath, 1);

    path->data = g_strdup(value);
    path->size = strlen(value) + 1;
    return path;
}

static void record_free(gpointer opaque)
{
    Nfs2HandleRecord *record = opaque;

    if (record) {
        g_ptr_array_free(record->aliases, true);
        g_free(record);
    }
}

static void rename_change_free(gpointer opaque)
{
    Nfs2RenameChange *change = opaque;

    if (change) {
        g_free(change->new_path);
        g_free(change);
    }
}

static bool path_valid(const char *path, Error **errp)
{
    size_t length;

    if (!path || path[0] != '/') {
        error_setg(errp, "NFS handle path must be absolute");
        return false;
    }
    length = strlen(path);
    if (length > NFS2_MAX_PATH) {
        error_setg(errp, "NFS handle path exceeds %u bytes", NFS2_MAX_PATH);
        return false;
    }
    return true;
}

static bool calculate_mac(Nfs2HandleTable *table, const uint8_t *data,
                          uint8_t mac[NFS2_HANDLE_MAC_SIZE], Error **errp)
{
    g_autoptr(QCryptoHmac) hmac = NULL;
    uint8_t digest[QCRYPTO_HASH_DIGEST_LEN_SHA256];
    uint8_t *digest_ptr = digest;
    size_t digest_length = sizeof(digest);

#ifdef NFS2_HANDLE_TESTING
    if (table->fail_next_mac) {
        table->fail_next_mac = false;
        error_setg(errp, "injected NFS handle HMAC failure");
        return false;
    }
#endif
    hmac = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA256, table->key,
                            sizeof(table->key), errp);
    if (!hmac || qcrypto_hmac_bytes(hmac, (const char *)data,
                                    NFS2_HANDLE_AUTHENTICATED_SIZE,
                                    &digest_ptr, &digest_length, errp) < 0) {
        secure_clear(digest, sizeof(digest));
        return false;
    }
    memcpy(mac, digest, NFS2_HANDLE_MAC_SIZE);
    secure_clear(digest, sizeof(digest));
    return true;
}

static bool mac_equal(const uint8_t *left, const uint8_t *right, size_t length)
{
    uint8_t difference = 0;

    for (size_t i = 0; i < length; i++) {
        difference |= left[i] ^ right[i];
    }
    return difference == 0;
}

static bool encode_handle(Nfs2HandleTable *table,
                          const Nfs2HandleRecord *record,
                          Nfs2FileHandle *handle, Error **errp)
{
    memcpy(handle->bytes, "QN2F", 4);
    stl_be_p(handle->bytes + 4, NFS2_HANDLE_VERSION);
    stq_be_p(handle->bytes + 8, record->id);
    stl_be_p(handle->bytes + 16, record->generation);
    return calculate_mac(table, handle->bytes, handle->bytes + 20, errp);
}

Nfs2HandleTable *nfs2_handle_table_new(const uint8_t *key, size_t key_length,
                                       Error **errp)
{
    Nfs2HandleTable *table = g_new0(Nfs2HandleTable, 1);
    g_autoptr(QCryptoHmac) probe = NULL;

    if (key) {
        if (key_length != sizeof(table->key)) {
            error_setg(errp, "NFS handle HMAC key must be %zu bytes",
                       sizeof(table->key));
            g_free(table);
            return NULL;
        }
        memcpy(table->key, key, sizeof(table->key));
    } else {
        if (key_length != 0) {
            error_setg(errp, "NFS handle HMAC key is NULL with nonzero length");
            g_free(table);
            return NULL;
        }
        if (qcrypto_random_bytes(table->key, sizeof(table->key), errp) < 0) {
            secure_clear(table, sizeof(*table));
            g_free(table);
            return NULL;
        }
    }

    probe = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA256, table->key,
                             sizeof(table->key), errp);
    if (!probe) {
        secure_clear(table, sizeof(*table));
        g_free(table);
        return NULL;
    }

    table->by_id = g_hash_table_new_full(g_int64_hash, g_int64_equal,
                                         NULL, record_free);
    table->by_path = g_hash_table_new(g_str_hash, g_str_equal);
    table->next_generation = 1;
    table->active = true;
    return table;
}

void nfs2_handle_table_clear(Nfs2HandleTable *table)
{
    if (!table) {
        return;
    }
    g_hash_table_remove_all(table->by_path);
    g_hash_table_remove_all(table->by_id);
    table->alias_count = 0;
    secure_clear(table->key, sizeof(table->key));
    table->active = false;
}

void nfs2_handle_table_free(Nfs2HandleTable *table)
{
    if (!table) {
        return;
    }
    nfs2_handle_table_clear(table);
    g_hash_table_destroy(table->by_path);
    g_hash_table_destroy(table->by_id);
    secure_clear(table, sizeof(*table));
    g_free(table);
}

bool nfs2_handle_create(Nfs2HandleTable *table, uint64_t id,
                        const char *path, Nfs2FileHandle *handle,
                        Error **errp)
{
    Nfs2HandleRecord *record;
    Nfs2HandleRecord *path_record;
    V9fsPath *alias;
    Nfs2FileHandle encoded;
    bool new_record = false;

    if (!table || !table->active || !handle) {
        error_setg(errp, "NFS handle table is not active");
        return false;
    }
    if (!path_valid(path, errp)) {
        return false;
    }

    path_record = g_hash_table_lookup(table->by_path, path);
    if (path_record && path_record->id == id) {
        if (!encode_handle(table, path_record, &encoded, errp)) {
            return false;
        }
        *handle = encoded;
        return true;
    }
    if (!path_record && table->alias_count >= NFS2_MAX_HANDLE_RECORDS) {
        error_setg(errp, "NFS handle table is full");
        return false;
    }

    record = g_hash_table_lookup(table->by_id, &id);
    if (!record) {
        if (table->generation_exhausted) {
            error_setg(errp, "NFS handle generation exhausted");
            return false;
        }
        record = g_new0(Nfs2HandleRecord, 1);
        record->id = id;
        record->generation = table->next_generation;
        record->aliases = g_ptr_array_new_with_free_func(path_free);
        new_record = true;
    }

    if (!encode_handle(table, record, &encoded, errp)) {
        if (new_record) {
            record_free(record);
        }
        return false;
    }
    if (new_record) {
        if (table->next_generation == UINT32_MAX) {
            table->generation_exhausted = true;
        } else {
            table->next_generation++;
        }
        g_hash_table_insert(table->by_id, &record->id, record);
    }

    if (path_record) {
        for (size_t i = 0; i < path_record->aliases->len; i++) {
            V9fsPath *old_alias = g_ptr_array_index(path_record->aliases, i);

            if (strcmp(old_alias->data, path) == 0) {
                g_hash_table_remove(table->by_path, old_alias->data);
                g_ptr_array_remove_index(path_record->aliases, i);
                table->alias_count--;
                break;
            }
        }
        if (path_record->aliases->len == 0) {
            g_hash_table_remove(table->by_id, &path_record->id);
        }
    }

    alias = path_new(path);
    g_ptr_array_add(record->aliases, alias);
    g_hash_table_insert(table->by_path, alias->data, record);
    table->alias_count++;
    *handle = encoded;
    return true;
}

static Nfs2HandleRecord *resolve_record(Nfs2HandleTable *table,
                                        const Nfs2FileHandle *handle)
{
    uint8_t expected[NFS2_HANDLE_MAC_SIZE];
    uint64_t id;
    uint32_t generation;
    Nfs2HandleRecord *record;

    if (!table || !table->active || !handle ||
        !calculate_mac(table, handle->bytes, expected, NULL) ||
        !mac_equal(expected, handle->bytes + 20, sizeof(expected)) ||
        memcmp(handle->bytes, "QN2F", 4) != 0 ||
        ldl_be_p(handle->bytes + 4) != NFS2_HANDLE_VERSION) {
        secure_clear(expected, sizeof(expected));
        return NULL;
    }
    secure_clear(expected, sizeof(expected));

    id = ldq_be_p(handle->bytes + 8);
    generation = ldl_be_p(handle->bytes + 16);
    record = g_hash_table_lookup(table->by_id, &id);
    if (!record || record->generation != generation) {
        return NULL;
    }
    return record;
}

bool nfs2_handle_resolve(Nfs2HandleTable *table,
                         const Nfs2FileHandle *handle, V9fsPath *path)
{
    Nfs2HandleRecord *record = resolve_record(table, handle);
    V9fsPath *canonical;

    if (!record || !path || record->aliases->len == 0) {
        return false;
    }

    canonical = g_ptr_array_index(record->aliases, 0);
    g_free(path->data);
    path->data = g_memdup2(canonical->data, canonical->size);
    path->size = canonical->size;
    return true;
}

GPtrArray *nfs2_handle_paths_snapshot(Nfs2HandleTable *table,
                                      const Nfs2FileHandle *handle)
{
    Nfs2HandleRecord *record = resolve_record(table, handle);
    GPtrArray *paths;

    if (!record || record->aliases->len == 0) {
        return NULL;
    }
    paths = g_ptr_array_new_with_free_func(path_free);
    for (size_t i = 0; i < record->aliases->len; i++) {
        V9fsPath *alias = g_ptr_array_index(record->aliases, i);

        g_ptr_array_add(paths, path_new(alias->data));
    }
    return paths;
}

void nfs2_handle_path_state(Nfs2HandleTable *table, const char *path,
                            Nfs2HandlePathState *state)
{
    Nfs2HandleRecord *record;

    if (!state) {
        return;
    }
    memset(state, 0, sizeof(*state));
    if (!table || !table->active || !path) {
        return;
    }
    record = g_hash_table_lookup(table->by_path, path);
    if (record) {
        state->id = record->id;
        state->generation = record->generation;
        state->present = true;
    }
}

bool nfs2_handle_path_state_allows(Nfs2HandleTable *table, const char *path,
                                   const Nfs2HandlePathState *state,
                                   uint64_t new_id)
{
    Nfs2HandleRecord *record;

    if (!table || !table->active || !path || !state) {
        return false;
    }
    record = g_hash_table_lookup(table->by_path, path);
    if (record && record->id == new_id) {
        return true;
    }
    return state->present ?
           record && record->id == state->id &&
           record->generation == state->generation : !record;
}

static bool path_has_prefix(const char *path, const char *prefix)
{
    size_t prefix_length = strlen(prefix);

    if (strcmp(path, prefix) == 0) {
        return true;
    }
    if (prefix_length == 1 && prefix[0] == '/') {
        return path[0] == '/';
    }
    return strncmp(path, prefix, prefix_length) == 0 &&
           path[prefix_length] == '/';
}

bool nfs2_handle_rename(Nfs2HandleTable *table, const char *old_path,
                        const char *new_path, Error **errp)
{
    g_autoptr(GPtrArray) changes =
        g_ptr_array_new_with_free_func(rename_change_free);
    g_autoptr(GHashTable) affected = g_hash_table_new(g_direct_hash,
                                                       g_direct_equal);
    g_autoptr(GHashTable) destinations = g_hash_table_new(g_str_hash,
                                                           g_str_equal);
    g_autoptr(GPtrArray) replacements =
        g_ptr_array_new_with_free_func(rename_change_free);
    g_autoptr(GHashTable) replaced = g_hash_table_new(g_direct_hash,
                                                       g_direct_equal);
    GHashTableIter iter;
    gpointer value;
    Nfs2HandleRecord *old_record;
    Nfs2HandleRecord *new_record;
    size_t old_length;

    if (!table || !table->active || !path_valid(old_path, errp) ||
        !path_valid(new_path, errp)) {
        return false;
    }
    if (strcmp(old_path, new_path) == 0) {
        return true;
    }
    old_record = g_hash_table_lookup(table->by_path, old_path);
    new_record = g_hash_table_lookup(table->by_path, new_path);
    if (old_record && old_record == new_record) {
        return true;
    }

    old_length = strlen(old_path);
    g_hash_table_iter_init(&iter, table->by_id);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        Nfs2HandleRecord *record = value;

        for (size_t i = 0; i < record->aliases->len; i++) {
            V9fsPath *alias = g_ptr_array_index(record->aliases, i);
            Nfs2RenameChange *change;
            const char *suffix;

            if (!path_has_prefix(alias->data, old_path)) {
                continue;
            }
            if (old_length == 1 && strcmp(alias->data, "/") != 0) {
                suffix = alias->data;
            } else {
                suffix = alias->data + old_length;
            }
            change = g_new0(Nfs2RenameChange, 1);
            change->alias = alias;
            change->record = record;
            change->new_path = g_strconcat(new_path, suffix, NULL);
            if (strlen(change->new_path) > NFS2_MAX_PATH) {
                error_setg(errp, "renamed NFS handle path exceeds %u bytes",
                           NFS2_MAX_PATH);
                rename_change_free(change);
                return false;
            }
            if (g_hash_table_contains(destinations, change->new_path)) {
                error_setg(errp, "rename creates duplicate NFS handle paths");
                rename_change_free(change);
                return false;
            }
            g_hash_table_add(destinations, change->new_path);
            g_hash_table_add(affected, alias);
            g_ptr_array_add(changes, change);
        }
    }

    g_hash_table_iter_init(&iter, table->by_id);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        Nfs2HandleRecord *record = value;

        for (size_t i = 0; i < record->aliases->len; i++) {
            V9fsPath *alias = g_ptr_array_index(record->aliases, i);

            if (path_has_prefix(alias->data, new_path) &&
                !g_hash_table_contains(affected, alias)) {
                Nfs2RenameChange *replacement =
                    g_new0(Nfs2RenameChange, 1);

                replacement->alias = alias;
                replacement->record = record;
                g_ptr_array_add(replacements, replacement);
                g_hash_table_add(replaced, alias);
            }
        }
    }

    for (size_t i = 0; i < changes->len; i++) {
        Nfs2RenameChange *change = g_ptr_array_index(changes, i);
        Nfs2HandleRecord *existing =
            g_hash_table_lookup(table->by_path, change->new_path);
        V9fsPath *existing_alias;

        if (!existing) {
            continue;
        }
        existing_alias = NULL;
        for (size_t j = 0; j < existing->aliases->len; j++) {
            V9fsPath *candidate = g_ptr_array_index(existing->aliases, j);

            if (strcmp(candidate->data, change->new_path) == 0) {
                existing_alias = candidate;
                break;
            }
        }
        if (!g_hash_table_contains(affected, existing_alias) &&
            !g_hash_table_contains(replaced, existing_alias)) {
            error_setg(errp, "rename collides with an NFS handle path");
            return false;
        }
    }

    for (size_t i = 0; i < changes->len; i++) {
        Nfs2RenameChange *change = g_ptr_array_index(changes, i);

        g_hash_table_remove(table->by_path, change->alias->data);
    }
    for (size_t i = 0; i < replacements->len; i++) {
        Nfs2RenameChange *replacement = g_ptr_array_index(replacements, i);

        g_hash_table_remove(table->by_path, replacement->alias->data);
    }
    for (size_t i = 0; i < replacements->len; i++) {
        Nfs2RenameChange *replacement = g_ptr_array_index(replacements, i);
        Nfs2HandleRecord *record = replacement->record;

        g_assert(g_ptr_array_remove(record->aliases, replacement->alias));
        table->alias_count--;
        if (record->aliases->len == 0) {
            g_hash_table_remove(table->by_id, &record->id);
        }
    }
    for (size_t i = 0; i < changes->len; i++) {
        Nfs2RenameChange *change = g_ptr_array_index(changes, i);

        g_free(change->alias->data);
        change->alias->data = g_steal_pointer(&change->new_path);
        change->alias->size = strlen(change->alias->data) + 1;
        g_hash_table_insert(table->by_path, change->alias->data,
                            change->record);
    }
    return true;
}

bool nfs2_handle_remove(Nfs2HandleTable *table, const char *path,
                        Error **errp)
{
    Nfs2HandleRecord *record;

    if (!table || !table->active || !path_valid(path, errp)) {
        return false;
    }
    record = g_hash_table_lookup(table->by_path, path);
    if (!record) {
        error_setg(errp, "NFS handle path is not registered");
        return false;
    }

    for (size_t i = 0; i < record->aliases->len; i++) {
        V9fsPath *alias = g_ptr_array_index(record->aliases, i);

        if (strcmp(alias->data, path) == 0) {
            g_hash_table_remove(table->by_path, alias->data);
            g_ptr_array_remove_index(record->aliases, i);
            table->alias_count--;
            break;
        }
    }
    if (record->aliases->len == 0) {
        g_hash_table_remove(table->by_id, &record->id);
    }
    return true;
}

size_t nfs2_handle_table_record_count(const Nfs2HandleTable *table)
{
    return table ? g_hash_table_size(table->by_id) : 0;
}

#ifdef NFS2_HANDLE_TESTING
void nfs2_handle_table_set_next_generation_for_test(Nfs2HandleTable *table,
                                                     uint32_t generation)
{
    g_assert(table && table->active);
    g_assert(generation != 0);
    table->next_generation = generation;
    table->generation_exhausted = false;
}

void nfs2_handle_table_fail_next_mac_for_test(Nfs2HandleTable *table)
{
    g_assert(table && table->active);
    table->fail_next_mac = true;
}
#endif
