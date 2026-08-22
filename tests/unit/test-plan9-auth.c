/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Golden values were independently checked from the preserved Second Edition
 * sources in the Plan 9 rootfs: sys/src/libauth/{passtokey,convTR2M,
 * convM2TR,convT2M,convM2T,convA2M}.c and sys/man/2/encrypt.  The historical
 * DES block values
 * were cross-checked with OpenSSL DES/ECB after applying the documented
 * packed 56-bit-to-odd-parity 64-bit conversion; no real credential occurs
 * in these fixtures.
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/base64.h"
#include "hw/9pfs/plan9-auth.h"
#include "qapi/error.h"

#include <termios.h>

#ifdef HAVE_OPENPTY
#include <pty.h>
#include <sys/ioctl.h>
#endif

static void assert_zeroed(const void *data, size_t len);

typedef struct KeydbFixtureRecord {
    const char *name;
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];
    uint8_t status;
    uint8_t warnings;
    uint32_t expiry;
} KeydbFixtureRecord;

static const uint8_t keydb_master_key[PLAN9_AUTH_DES_KEY_LEN] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd,
};

static char *keydb_tempdir(void)
{
    GError *error = NULL;
    char *dir = g_dir_make_tmp("qemu-plan9-keydb-XXXXXX", &error);

    g_assert_no_error(error);
    g_assert_nonnull(dir);
    return dir;
}

static void keydb_remove_tree(const char *dir)
{
    g_autofree char *path = g_build_filename(dir, "keys", NULL);

    unlink(path);
    rmdir(dir);
}

typedef struct KeydbToolResult {
    int status;
    char *out;
    char *err;
    char *terminal;
#ifdef HAVE_OPENPTY
    struct termios termios_before;
    struct termios termios_after;
    bool have_termios;
#endif
} KeydbToolResult;

typedef struct KeydbToolBarrier {
    GMutex lock;
    GCond ready;
    unsigned int count;
} KeydbToolBarrier;

#ifdef HAVE_OPENPTY
typedef struct KeydbToolChildSetup {
    int slave;
} KeydbToolChildSetup;

static void keydb_tool_child_setup(void *opaque)
{
    KeydbToolChildSetup *setup = opaque;

    if (setsid() < 0 || ioctl(setup->slave, TIOCSCTTY, 0) < 0) {
        _exit(127);
    }
}
#endif

static char *read_all_fd(int fd)
{
    GString *text = g_string_new(NULL);
    char buf[256];
    ssize_t got;

    while ((got = read(fd, buf, sizeof(buf))) > 0) {
        g_string_append_len(text, buf, got);
    }
    return g_string_free(text, false);
}

static bool read_until_fd(int fd, GString *text, const char *needle)
{
    char byte;

    while (!strstr(text->str, needle)) {
        if (read(fd, &byte, 1) != 1) {
            return false;
        }
        g_string_append_c(text, byte);
    }
    return true;
}

static KeydbToolResult keydb_tool_run_barrier(
    const char *keydb, const char *secret, const char *server_id,
    const char *password_input, bool tty, KeydbToolBarrier *barrier,
    const char *late_collision, int signal_after_prompt,
    const char *stop_stage)
{
    const char *tool = g_getenv("QEMU_PLAN9_KEYDB");
    char *argv[] = {
        (char *)tool, (char *)"create", (char *)"--keydb", (char *)keydb,
        (char *)"--secret", (char *)secret, (char *)"--server-id",
        (char *)server_id, NULL,
    };
    KeydbToolResult result = { .status = -1 };
    GError *gerr = NULL;
    g_auto(GStrv) envp = g_get_environ();
    GPid pid = 0;
    GSpawnChildSetupFunc child_setup_func = NULL;
    void *child_setup_data = NULL;
    int out_pipe[2], err_pipe[2];
    int input = -1;
    GString *early_err = g_string_new(NULL);
#ifdef HAVE_OPENPTY
    int master = -1, slave = -1;
    int observe = -1;
    GString *early_terminal = g_string_new(NULL);
    KeydbToolChildSetup child_setup;
#endif

    g_assert_nonnull(tool);
    if (stop_stage) {
        envp = g_environ_setenv(envp, "QEMU_PLAN9_KEYDB_TEST_STAGE",
                                stop_stage, true);
    }
    g_assert_cmpint(pipe(out_pipe), ==, 0);
    g_assert_cmpint(pipe(err_pipe), ==, 0);
#ifdef HAVE_OPENPTY
    if (tty) {
        g_assert_cmpint(openpty(&master, &slave, NULL, NULL, NULL), ==, 0);
        observe = dup(slave);
        g_assert_cmpint(observe, >=, 0);
        g_assert_cmpint(tcgetattr(observe, &result.termios_before), ==, 0);
        input = slave;
        child_setup.slave = slave;
        child_setup_func = keydb_tool_child_setup;
        child_setup_data = &child_setup;
    } else
#endif
    {
        input = open("/dev/null", O_RDONLY | O_CLOEXEC);
        g_assert_cmpint(input, >=, 0);
    }
    g_assert_true(g_spawn_async_with_fds(NULL, argv, envp,
                                         G_SPAWN_DO_NOT_REAP_CHILD,
                                         child_setup_func, child_setup_data,
                                         &pid, input,
                                         out_pipe[1], err_pipe[1], &gerr));
    g_assert_no_error(gerr);
    close(input);
    close(out_pipe[1]);
    close(err_pipe[1]);
#ifdef HAVE_OPENPTY
    if (tty) {
        const char *newline;

        g_assert_nonnull(password_input);
        newline = strchr(password_input, '\n');
        g_assert_nonnull(newline);
        if (read_until_fd(master, early_terminal, "tor password: ")) {
            if (signal_after_prompt) {
                g_assert_cmpint(kill(pid, signal_after_prompt), ==, 0);
                goto wait_for_child;
            }
            if (late_collision) {
                g_assert_true(g_file_set_contents(late_collision,
                                                  "late-secret", 11, NULL));
            }
            if (barrier) {
                g_mutex_lock(&barrier->lock);
                barrier->count++;
                g_cond_broadcast(&barrier->ready);
                while (barrier->count < 2) {
                    g_cond_wait(&barrier->ready, &barrier->lock);
                }
                g_mutex_unlock(&barrier->lock);
            }
            g_assert_cmpint(write(master, password_input,
                                  newline - password_input + 1), ==,
                            newline - password_input + 1);
            if ((size_t)(newline - password_input) < PLAN9_AUTH_NAMELEN &&
                read_until_fd(master, early_terminal,
                              "confirm tor password: ")) {
                newline++;
                g_assert_cmpint(write(master, newline, strlen(newline)), ==,
                                strlen(newline));
            }
            if (stop_stage &&
                (!strcmp(stop_stage, "after-password") ||
                 !strcmp(stop_stage, "after-temp") ||
                 !strcmp(stop_stage, "between-publish"))) {
                int stopped;

                g_assert_cmpint(waitpid(pid, &stopped, WUNTRACED), ==, pid);
                g_assert_true(WIFSTOPPED(stopped));
                g_assert_cmpint(WSTOPSIG(stopped), ==, SIGSTOP);
                if (!strcmp(stop_stage, "between-publish")) {
                    g_assert_true(g_file_test(keydb,
                                              G_FILE_TEST_IS_REGULAR));
                    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
                } else {
                    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
                    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
                }
                g_assert_cmpint(kill(pid, SIGTERM), ==, 0);
                g_assert_cmpint(kill(pid, SIGCONT), ==, 0);
            }
        }
    }
#else
    g_assert_false(tty);
#endif
wait_for_child:
    g_assert_cmpint(waitpid(pid, &result.status, 0), ==, pid);
    g_spawn_close_pid(pid);
    result.out = read_all_fd(out_pipe[0]);
    {
        g_autofree char *remaining = read_all_fd(err_pipe[0]);

        g_string_append(early_err, remaining);
        result.err = g_string_free(early_err, false);
        early_err = NULL;
    }
    close(out_pipe[0]);
    close(err_pipe[0]);
#ifdef HAVE_OPENPTY
    if (tty) {
        int flags = fcntl(master, F_GETFL);

        g_assert_cmpint(tcgetattr(observe, &result.termios_after), ==, 0);
        result.have_termios = true;
        close(observe);
        g_assert_cmpint(fcntl(master, F_SETFL, flags | O_NONBLOCK), ==, 0);
        {
            g_autofree char *remaining = read_all_fd(master);

            g_string_append(early_terminal, remaining);
            result.terminal = g_string_free(early_terminal, false);
            early_terminal = NULL;
        }
        close(master);
    }
    if (early_terminal) {
        g_string_free(early_terminal, true);
    }
#endif
    if (early_err) {
        g_string_free(early_err, true);
    }
    return result;
}

static KeydbToolResult keydb_tool_run(const char *keydb, const char *secret,
                                      const char *server_id,
                                      const char *password_input, bool tty)
{
    return keydb_tool_run_barrier(keydb, secret, server_id, password_input,
                                  tty, NULL, NULL, 0, NULL);
}

static void keydb_tool_result_clear(KeydbToolResult *result)
{
    g_free(result->out);
    g_free(result->err);
    g_free(result->terminal);
}

#ifdef HAVE_OPENPTY
static void assert_termios_restored(const KeydbToolResult *result)
{
    g_assert_true(result->have_termios);
    g_assert_cmpuint(result->termios_after.c_iflag, ==,
                     result->termios_before.c_iflag);
    g_assert_cmpuint(result->termios_after.c_oflag, ==,
                     result->termios_before.c_oflag);
    g_assert_cmpuint(result->termios_after.c_cflag, ==,
                     result->termios_before.c_cflag);
    g_assert_cmpuint(result->termios_after.c_lflag, ==,
                     result->termios_before.c_lflag);
    g_assert_cmpmem(result->termios_after.c_cc,
                    sizeof(result->termios_after.c_cc),
                    result->termios_before.c_cc,
                    sizeof(result->termios_before.c_cc));
}
#endif

static void assert_no_keydb_temps(const char *dir, size_t expected_files)
{
    g_autoptr(GDir) stream = g_dir_open(dir, 0, NULL);
    const char *name;
    size_t count = 0;

    g_assert_nonnull(stream);
    while ((name = g_dir_read_name(stream))) {
        g_assert_null(strstr(name, ".qemu-plan9-keydb-"));
        count++;
    }
    g_assert_cmpuint(count, ==, expected_files);
}

static void keydb_write(const char *path, const KeydbFixtureRecord *records,
                        size_t count)
{
    g_autofree uint8_t *ciphertext = g_new0(uint8_t,
        count * PLAN9_AUTH_KEYDB_RECORD_LEN);
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);

    g_assert_cmpint(fd, >=, 0);
    for (size_t i = 0; i < count; i++) {
        uint8_t *record = ciphertext + i * PLAN9_AUTH_KEYDB_RECORD_LEN;
        size_t name_len = strnlen(records[i].name, PLAN9_AUTH_NAMELEN);
        Error *err = NULL;

        g_assert_cmpuint(name_len, <=, PLAN9_AUTH_NAMELEN);
        memcpy(record, records[i].name, name_len);
        memcpy(record + PLAN9_AUTH_NAMELEN, records[i].key,
               PLAN9_AUTH_DES_KEY_LEN);
        record[PLAN9_AUTH_NAMELEN + PLAN9_AUTH_DES_KEY_LEN] =
            records[i].status;
        record[PLAN9_AUTH_NAMELEN + PLAN9_AUTH_DES_KEY_LEN + 1] =
            records[i].warnings;
        stl_le_p(record + PLAN9_AUTH_NAMELEN + PLAN9_AUTH_DES_KEY_LEN + 2,
                 records[i].expiry);
        g_assert_cmpint(plan9_auth_encrypt(keydb_master_key, record,
                                            PLAN9_AUTH_KEYDB_RECORD_LEN,
                                            &err), ==, 0);
        g_assert_null(err);
    }
    g_assert_cmpint(write(fd, ciphertext,
                          count * PLAN9_AUTH_KEYDB_RECORD_LEN), ==,
                    count * PLAN9_AUTH_KEYDB_RECORD_LEN);
    close(fd);
    plan9_auth_clear(ciphertext, count * PLAN9_AUTH_KEYDB_RECORD_LEN);
}

static Plan9AuthKeydb *keydb_load(const char *path, const char *server,
                                   uint32_t now)
{
    Error *err = NULL;
    Plan9AuthKeydb *keydb = plan9_auth_keydb_load(path, keydb_master_key,
                                                   server, now, &err);

    g_assert_null(err);
    g_assert_nonnull(keydb);
    return keydb;
}

static void keydb_expect_rejected(const char *path, const char *server,
                                  uint32_t now)
{
    Error *err = NULL;

    g_assert_null(plan9_auth_keydb_load(path, keydb_master_key, server, now,
                                        &err));
    g_assert_nonnull(err);
    error_free(err);
}

static void test_keydb_golden_and_lookup(void)
{
    /*
     * Independently made with OpenSSL DES/ECB and the Second Edition
     * sys/man/2/encrypt overlap sequence at offsets 0,7,14,21,28,33, using
     * the historical 56-to-64 expansion of keydb_master_key.  This is one
     * native 41-byte /adm/keys record: p9fs, key 01..07, status ok, warning
     * 9, never expires.  It is deliberately not generated by the code under
     * test.
     */
    static const uint8_t golden[] = {
        0x75, 0x6f, 0x1a, 0x43, 0x45, 0x3a, 0x6d, 0xb7, 0xf1, 0x32,
        0x56, 0x77, 0x3c, 0x36, 0x05, 0xb9, 0x68, 0xaf, 0xca, 0xd1,
        0x57, 0x13, 0xa7, 0x3c, 0x8f, 0xab, 0xcf, 0x37, 0xd7, 0xd2,
        0x12, 0xeb, 0x03, 0xbd, 0x8d, 0x02, 0xe1, 0x88, 0x1c, 0x9f,
        0x2b,
    };
    static const KeydbFixtureRecord records[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 9, 0 },
        { "tor", { 7, 6, 5, 4, 3, 2, 1 }, 0, 3, 100 },
        { "disabled", { 3, 3, 3, 3, 3, 3, 3 }, 1, 0, 0 },
        { "old", { 4, 4, 4, 4, 4, 4, 4 }, 0, 0, 99 },
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    Plan9AuthKeydb *keydb;
    int fd;

    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(write(fd, golden, sizeof(golden)), ==, sizeof(golden));
    close(fd);
    keydb = keydb_load(path, "p9fs", 100);
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "p9fs", 100, key), ==,
                    PLAN9_AUTH_KEY_AVAILABLE);
    g_assert_cmpmem(key, sizeof(key), records[0].key, sizeof(key));
    plan9_auth_keydb_free(keydb);

    keydb_write(path, records, G_N_ELEMENTS(records));
    keydb = keydb_load(path, "p9fs", 100);
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "tor", 100, key), ==,
                    PLAN9_AUTH_KEY_AVAILABLE);
    g_assert_cmpmem(key, sizeof(key), records[1].key, sizeof(key));
    memset(key, 0xa5, sizeof(key));
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "disabled", 100, key),
                    ==, PLAN9_AUTH_KEY_DISABLED);
    assert_zeroed(key, sizeof(key));
    memset(key, 0xa5, sizeof(key));
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "old", 100, key), ==,
                    PLAN9_AUTH_KEY_EXPIRED);
    assert_zeroed(key, sizeof(key));
    memset(key, 0xa5, sizeof(key));
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "nope", 100, key), ==,
                    PLAN9_AUTH_KEY_MISSING);
    assert_zeroed(key, sizeof(key));
    plan9_auth_keydb_free(keydb);
    keydb_remove_tree(dir);
}

typedef struct KeydbLookupVisits {
    size_t visits;
} KeydbLookupVisits;

static void keydb_lookup_visit(size_t index, void *opaque)
{
    KeydbLookupVisits *visits = opaque;

    g_assert_cmpuint(index, ==, visits->visits);
    visits->visits++;
}

static void test_keydb_lookup_full_scan(void)
{
    static const KeydbFixtureRecord records[] = {
        { "first", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
        { "middle", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
        { "disabled", { 3, 3, 3, 3, 3, 3, 3 }, 1, 0, 0 },
        { "old", { 4, 4, 4, 4, 4, 4, 4 }, 0, 0, 99 },
        { "p9fs", { 5, 5, 5, 5, 5, 5, 5 }, 0, 0, 0 },
    };
    static const struct {
        const char *name;
        Plan9AuthKeyStatus status;
    } cases[] = {
        { "first", PLAN9_AUTH_KEY_AVAILABLE },
        { "middle", PLAN9_AUTH_KEY_AVAILABLE },
        { "disabled", PLAN9_AUTH_KEY_DISABLED },
        { "old", PLAN9_AUTH_KEY_EXPIRED },
        { "missing", PLAN9_AUTH_KEY_MISSING },
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    Plan9AuthKeydb *keydb;
    KeydbLookupVisits visits = { 0 };
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];

    keydb_write(path, records, G_N_ELEMENTS(records));
    keydb = keydb_load(path, "p9fs", 100);
    plan9_auth_keydb_set_lookup_hook(keydb_lookup_visit, &visits);
    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        visits.visits = 0;
        g_assert_cmpint(plan9_auth_keydb_lookup(keydb, cases[i].name, 100,
                                                key), ==, cases[i].status);
        g_assert_cmpuint(visits.visits, ==, G_N_ELEMENTS(records));
    }
    plan9_auth_keydb_set_lookup_hook(NULL, NULL);
    plan9_auth_clear(key, sizeof(key));
    plan9_auth_keydb_free(keydb);
    keydb_remove_tree(dir);
}

static void test_keydb_record_encoder(void)
{
    static const uint8_t expected[PLAN9_AUTH_KEYDB_RECORD_LEN] = {
        0x75, 0x6f, 0x1a, 0x43, 0x45, 0x3a, 0x6d, 0xb7, 0xf1, 0x32,
        0x56, 0x77, 0x3c, 0x36, 0x05, 0xb9, 0x68, 0xaf, 0xca, 0xd1,
        0x57, 0x13, 0xa7, 0x3c, 0x8f, 0xab, 0xcf, 0x37, 0xd7, 0xd2,
        0x12, 0xeb, 0x03, 0xbd, 0x8d, 0x02, 0xe1, 0x88, 0x1c, 0x9f,
        0x2b,
    };
    static const uint8_t record_key[PLAN9_AUTH_DES_KEY_LEN] = {
        1, 2, 3, 4, 5, 6, 7,
    };
    uint8_t record[PLAN9_AUTH_KEYDB_RECORD_LEN];
    uint8_t plain[PLAN9_AUTH_KEYDB_RECORD_LEN];
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_keydb_record_encode(record, keydb_master_key,
                                                    "p9fs", record_key,
                                                    0, 9, 0, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(record, sizeof(record), expected, sizeof(expected));

    memcpy(plain, record, sizeof(plain));
    g_assert_cmpint(plan9_auth_decrypt(keydb_master_key, plain,
                                      sizeof(plain), &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(plain, 4, "p9fs", 4);
    assert_zeroed(plain + 4, PLAN9_AUTH_NAMELEN - 4);
    g_assert_cmpmem(plain + PLAN9_AUTH_NAMELEN,
                    PLAN9_AUTH_DES_KEY_LEN, record_key,
                    PLAN9_AUTH_DES_KEY_LEN);
    g_assert_cmpuint(plain[PLAN9_AUTH_NAMELEN + PLAN9_AUTH_DES_KEY_LEN],
                     ==, 0);
    g_assert_cmpuint(plain[PLAN9_AUTH_NAMELEN + PLAN9_AUTH_DES_KEY_LEN + 1],
                     ==, 9);
    g_assert_cmpuint(ldl_le_p(plain + PLAN9_AUTH_NAMELEN +
                              PLAN9_AUTH_DES_KEY_LEN + 2), ==, 0);

    memset(record, 0xa5, sizeof(record));
    g_assert_cmpint(plan9_auth_keydb_record_encode(record, keydb_master_key,
                                                    "", record_key,
                                                    0, 0, 0, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    assert_zeroed(record, sizeof(record));

    memset(record, 0xa5, sizeof(record));
    g_assert_cmpint(plan9_auth_keydb_record_encode(record, keydb_master_key,
                                                    "1234567890123456789012345678",
                                                    record_key, 0, 0, 0,
                                                    &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    assert_zeroed(record, sizeof(record));

    memset(record, 0xa5, sizeof(record));
    g_assert_cmpint(plan9_auth_keydb_record_encode(record, keydb_master_key,
                                                    "p9fs", record_key,
                                                    2, 0, 0, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(record, sizeof(record));
    plan9_auth_clear(plain, sizeof(plain));
}

static void test_keydb_tool_rejects_non_tty(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolResult result = keydb_tool_run(keydb, secret, "p9fs", NULL,
                                            false);

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    g_assert_null(strstr(result.out, "password"));
    assert_no_keydb_temps(dir, 0);
    keydb_tool_result_clear(&result);
    rmdir(dir);
}

static void test_keydb_tool_rejects_malformed_command(void)
{
    const char *tool = g_getenv("QEMU_PLAN9_KEYDB");
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    g_autofree char *out = NULL;
    g_autofree char *err = NULL;
    char *argv[] = {
        (char *)tool, (char *)"create", (char *)"--keydb", keydb,
        (char *)"--keydb", secret, (char *)"--server-id",
        (char *)"p9fs", NULL,
    };
    GError *gerr = NULL;
    int status;

    g_assert_nonnull(tool);
    g_assert_true(g_spawn_sync(NULL, argv, NULL, 0, NULL, NULL, &out, &err,
                               &status, &gerr));
    g_assert_no_error(gerr);
    g_assert_true(WIFEXITED(status));
    g_assert_cmpint(WEXITSTATUS(status), !=, 0);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    assert_no_keydb_temps(dir, 0);
    rmdir(dir);
}

#ifdef HAVE_OPENPTY
static void test_keydb_tool_success(void)
{
    static const char fixture_password[] = "historical-fixture";
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb_path = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret_path = g_build_filename(dir, "secret", NULL);
    g_autofree char *secret_text = NULL;
    g_autofree char *keydb_text = NULL;
    gsize secret_text_len = 0;
    gsize keydb_text_len = 0;
    g_autofree uint8_t *master_key = NULL;
    size_t master_len = 0;
    uint8_t tor_key[PLAN9_AUTH_DES_KEY_LEN];
    uint8_t expected_tor_key[PLAN9_AUTH_DES_KEY_LEN];
    uint8_t server_key[PLAN9_AUTH_DES_KEY_LEN];
    struct stat st;
    Error *err = NULL;
    Plan9AuthKeydb *keydb;
    KeydbToolResult result = keydb_tool_run(
        keydb_path, secret_path, "p9fs",
        "historical-fixture\nhistorical-fixture\n", true);

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), ==, 0);
    assert_termios_restored(&result);
    g_assert_cmpstr(result.out, ==, "");
    g_assert_cmpstr(result.err, ==, "");
    g_assert_null(strstr(result.err, fixture_password));
    g_assert_null(strstr(result.terminal, fixture_password));
    g_assert_cmpint(stat(keydb_path, &st), ==, 0);
    g_assert_cmpint(st.st_size, ==, 2 * PLAN9_AUTH_KEYDB_RECORD_LEN);
    g_assert_cmpuint(st.st_mode & 0777, ==, 0600);
    g_assert_cmpint(stat(secret_path, &st), ==, 0);
    g_assert_cmpint(st.st_size, ==, 12);
    g_assert_cmpuint(st.st_mode & 0777, ==, 0600);
    g_assert_true(g_file_get_contents(secret_path, &secret_text,
                                      &secret_text_len, NULL));
    g_assert_cmpuint(secret_text_len, ==, 12);
    g_assert_null(memchr(secret_text, '\n', secret_text_len));
    master_key = qbase64_decode(secret_text, secret_text_len, &master_len,
                                &err);
    g_assert_null(err);
    g_assert_cmpuint(master_len, ==, PLAN9_AUTH_DES_KEY_LEN);
    g_assert_true(g_file_get_contents(keydb_path, &keydb_text,
                                      &keydb_text_len, NULL));
    g_assert_cmpuint(keydb_text_len, ==, 2 * PLAN9_AUTH_KEYDB_RECORD_LEN);
    g_assert_cmpint(plan9_auth_decrypt(master_key, (uint8_t *)keydb_text,
                                      PLAN9_AUTH_KEYDB_RECORD_LEN,
                                      &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(keydb_text, ==, "tor");
    g_assert_cmpint(plan9_auth_decrypt(
                        master_key,
                        (uint8_t *)keydb_text + PLAN9_AUTH_KEYDB_RECORD_LEN,
                        PLAN9_AUTH_KEYDB_RECORD_LEN, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(keydb_text + PLAN9_AUTH_KEYDB_RECORD_LEN, ==, "p9fs");
    keydb = plan9_auth_keydb_load(keydb_path, master_key, "p9fs", 0, &err);
    g_assert_null(err);
    g_assert_nonnull(keydb);
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "tor", 0, tor_key), ==,
                    PLAN9_AUTH_KEY_AVAILABLE);
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "tor", UINT32_MAX,
                                            NULL), ==,
                    PLAN9_AUTH_KEY_AVAILABLE);
    g_assert_cmpint(plan9_auth_passtokey(expected_tor_key, fixture_password,
                                         &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(tor_key, sizeof(tor_key), expected_tor_key,
                    sizeof(expected_tor_key));
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "p9fs", 0, server_key),
                    ==, PLAN9_AUTH_KEY_AVAILABLE);
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "p9fs", UINT32_MAX,
                                            NULL), ==,
                    PLAN9_AUTH_KEY_AVAILABLE);
    g_assert_cmpint(memcmp(master_key, server_key, sizeof(server_key)), !=, 0);
    g_assert_cmpint(memcmp(tor_key, server_key, sizeof(server_key)), !=, 0);
    plan9_auth_keydb_free(keydb);
    plan9_auth_clear(tor_key, sizeof(tor_key));
    plan9_auth_clear(expected_tor_key, sizeof(expected_tor_key));
    plan9_auth_clear(server_key, sizeof(server_key));
    plan9_auth_clear(master_key, master_len);
    plan9_auth_clear(secret_text, secret_text_len);
    plan9_auth_clear(keydb_text, keydb_text_len);
    assert_no_keydb_temps(dir, 2);
    keydb_tool_result_clear(&result);
    unlink(secret_path);
    unlink(keydb_path);
    rmdir(dir);
}

static void test_keydb_tool_password_mismatch(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolResult result = keydb_tool_run(keydb, secret, "p9fs",
                                            "first\nsecond\n", true);

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    assert_termios_restored(&result);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    g_assert_null(strstr(result.out, "first"));
    g_assert_null(strstr(result.err, "first"));
    g_assert_null(strstr(result.terminal, "first"));
    assert_no_keydb_temps(dir, 0);
    keydb_tool_result_clear(&result);
    rmdir(dir);
}

static void test_keydb_tool_rejects_unrepresentable(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    static const char too_long[] = "0123456789012345678901234567";
    KeydbToolResult result = keydb_tool_run(
        keydb, secret, "p9fs",
        "0123456789012345678901234567\n"
        "0123456789012345678901234567\n", true);

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    assert_termios_restored(&result);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    g_assert_null(strstr(result.err, too_long));
    g_assert_null(strstr(result.terminal, too_long));
    assert_no_keydb_temps(dir, 0);
    keydb_tool_result_clear(&result);

    result = keydb_tool_run(keydb, secret, "tor", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    keydb_tool_result_clear(&result);
    result = keydb_tool_run(keydb, secret,
                            "1234567890123456789012345678",
                            NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    keydb_tool_result_clear(&result);
    result = keydb_tool_run(keydb, keydb, "p9fs", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    keydb_tool_result_clear(&result);
    assert_no_keydb_temps(dir, 0);
    rmdir(dir);
}

static void test_keydb_tool_signal_restores_terminal(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolResult result = keydb_tool_run_barrier(
        keydb, secret, "p9fs", "unused\nunused\n", true, NULL, NULL,
        SIGTERM, NULL);

    g_assert_true(WIFSIGNALED(result.status));
    g_assert_cmpint(WTERMSIG(result.status), ==, SIGTERM);
    assert_termios_restored(&result);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    assert_no_keydb_temps(dir, 0);
    keydb_tool_result_clear(&result);
    rmdir(dir);
}

static void test_keydb_tool_signal_transaction_stages(void)
{
    static const char *const stages[] = {
        "after-password", "after-temp", "between-publish",
    };

    for (size_t i = 0; i < G_N_ELEMENTS(stages); i++) {
        g_autofree char *dir = keydb_tempdir();
        g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
        g_autofree char *secret = g_build_filename(dir, "secret", NULL);
        KeydbToolResult result = keydb_tool_run_barrier(
            keydb, secret, "p9fs", "pw\npw\n", true, NULL, NULL, 0,
            stages[i]);

        g_assert_true(WIFSIGNALED(result.status));
        g_assert_cmpint(WTERMSIG(result.status), ==, SIGTERM);
        assert_termios_restored(&result);
        g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
        g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
        assert_no_keydb_temps(dir, 0);
        keydb_tool_result_clear(&result);
        rmdir(dir);
    }
}

static void test_keydb_tool_restore_error_is_combined(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolResult result = keydb_tool_run_barrier(
        keydb, secret, "p9fs", "first\nsecond\n", true, NULL, NULL, 0,
        "restore-error");

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    assert_termios_restored(&result);
    g_assert_nonnull(strstr(result.err, "confirmation does not match"));
    g_assert_nonnull(strstr(result.err, "Additionally, cannot restore"));
    g_assert_null(strstr(result.out, "first"));
    g_assert_null(strstr(result.err, "first"));
    g_assert_null(strstr(result.terminal, "first"));
    assert_no_keydb_temps(dir, 0);
    keydb_tool_result_clear(&result);
    rmdir(dir);
}

static void test_keydb_tool_temp_fstat_failure_cleans(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolResult result = keydb_tool_run_barrier(
        keydb, secret, "p9fs", "pw\npw\n", true, NULL, NULL, 0,
        "temp-fstat-error");

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    assert_termios_restored(&result);
    g_assert_nonnull(strstr(result.err, "inspect temporary output"));
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    assert_no_keydb_temps(dir, 0);
    keydb_tool_result_clear(&result);
    rmdir(dir);
}

static void test_keydb_tool_collision_refusal(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    g_autofree char *contents = NULL;
    gsize len;
    KeydbToolResult result;

    g_assert_true(g_file_set_contents(keydb, "keep", 4, NULL));
    result = keydb_tool_run(keydb, secret, "p9fs", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_true(g_file_get_contents(keydb, &contents, &len, NULL));
    g_assert_cmpmem(contents, len, "keep", 4);
    g_assert_nonnull(strstr(result.err, "incomplete"));
    g_assert_nonnull(strstr(result.err, "remove it manually"));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    assert_no_keydb_temps(dir, 1);
    keydb_tool_result_clear(&result);
    unlink(keydb);

    g_assert_cmpint(symlink("missing", keydb), ==, 0);
    result = keydb_tool_run(keydb, secret, "p9fs", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_true(g_file_test(keydb, G_FILE_TEST_IS_SYMLINK));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    assert_no_keydb_temps(dir, 1);
    keydb_tool_result_clear(&result);
    unlink(keydb);

    g_assert_true(g_file_set_contents(secret, "keep-secret", 11, NULL));
    result = keydb_tool_run(keydb, secret, "p9fs", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_clear_pointer(&contents, g_free);
    g_assert_true(g_file_get_contents(secret, &contents, &len, NULL));
    g_assert_cmpmem(contents, len, "keep-secret", 11);
    assert_no_keydb_temps(dir, 1);
    keydb_tool_result_clear(&result);
    unlink(secret);
    rmdir(dir);
}

static void test_keydb_tool_rejects_insecure_parent(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolResult result;

    g_assert_cmpint(chmod(dir, 0750), ==, 0);
    result = keydb_tool_run(keydb, secret, "p9fs", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_nonnull(strstr(result.err, "0700"));
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    keydb_tool_result_clear(&result);
    g_assert_cmpint(chmod(dir, 0700), ==, 0);
    rmdir(dir);
}

static void test_keydb_tool_rejects_parent_symlink(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *real = g_build_filename(dir, "real", NULL);
    g_autofree char *alias = g_build_filename(dir, "alias", NULL);
    g_autofree char *keydb = g_build_filename(alias, "keys", NULL);
    g_autofree char *secret = g_build_filename(alias, "secret", NULL);
    KeydbToolResult result;

    g_assert_cmpint(mkdir(real, 0700), ==, 0);
    g_assert_cmpint(symlink("real", alias), ==, 0);
    result = keydb_tool_run(keydb, secret, "p9fs", NULL, false);
    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_false(g_file_test(secret, G_FILE_TEST_EXISTS));
    keydb_tool_result_clear(&result);
    unlink(alias);
    rmdir(real);
    rmdir(dir);
}

typedef struct KeydbToolThread {
    const char *keydb;
    const char *secret;
    KeydbToolBarrier *barrier;
    KeydbToolResult result;
} KeydbToolThread;

static gpointer keydb_tool_thread(gpointer opaque)
{
    KeydbToolThread *thread = opaque;

    thread->result = keydb_tool_run_barrier(
        thread->keydb, thread->secret, "p9fs", "pw\npw\n", true,
        thread->barrier, NULL, 0, NULL);
    return NULL;
}

static void test_keydb_tool_concurrent_winner(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    KeydbToolBarrier barrier;
    KeydbToolThread workers[2] = {
        { .keydb = keydb, .secret = secret, .barrier = &barrier },
        { .keydb = keydb, .secret = secret, .barrier = &barrier },
    };
    GThread *threads[2];
    unsigned int successes = 0;

    g_mutex_init(&barrier.lock);
    g_cond_init(&barrier.ready);
    barrier.count = 0;
    for (size_t i = 0; i < G_N_ELEMENTS(threads); i++) {
        threads[i] = g_thread_new("plan9-keydb", keydb_tool_thread,
                                  &workers[i]);
    }
    for (size_t i = 0; i < G_N_ELEMENTS(threads); i++) {
        g_thread_join(threads[i]);
        g_assert_true(WIFEXITED(workers[i].result.status));
        successes += WEXITSTATUS(workers[i].result.status) == 0;
        keydb_tool_result_clear(&workers[i].result);
    }
    g_assert_cmpuint(successes, ==, 1);
    g_assert_true(g_file_test(keydb, G_FILE_TEST_IS_REGULAR));
    g_assert_true(g_file_test(secret, G_FILE_TEST_IS_REGULAR));
    assert_no_keydb_temps(dir, 2);
    g_cond_clear(&barrier.ready);
    g_mutex_clear(&barrier.lock);
    unlink(secret);
    unlink(keydb);
    rmdir(dir);
}

static void test_keydb_tool_late_collision_rollback(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *keydb = g_build_filename(dir, "keys", NULL);
    g_autofree char *secret = g_build_filename(dir, "secret", NULL);
    g_autofree char *contents = NULL;
    gsize len;
    KeydbToolResult result = keydb_tool_run_barrier(
        keydb, secret, "p9fs", "pw\npw\n", true, NULL, secret, 0, NULL);

    g_assert_true(WIFEXITED(result.status));
    g_assert_cmpint(WEXITSTATUS(result.status), !=, 0);
    g_assert_false(g_file_test(keydb, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_get_contents(secret, &contents, &len, NULL));
    g_assert_cmpmem(contents, len, "late-secret", 11);
    assert_no_keydb_temps(dir, 1);
    keydb_tool_result_clear(&result);
    unlink(secret);
    rmdir(dir);
}
#endif

static void test_keydb_rejections(void)
{
    static const KeydbFixtureRecord valid[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
    };
    static const KeydbFixtureRecord duplicate[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
        { "p9fs", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
    };
    static const KeydbFixtureRecord canonical_duplicate[] = {
        { "123456789012345678901234567a", { 1, 2, 3, 4, 5, 6, 7 },
          0, 0, 0 },
        { "123456789012345678901234567b", { 7, 6, 5, 4, 3, 2, 1 },
          0, 0, 0 },
    };
    static const KeydbFixtureRecord invalid[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 2, 0, 0 },
    };
    static const KeydbFixtureRecord disabled[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 1, 0, 0 },
    };
    static const KeydbFixtureRecord expired[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 99 },
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    int fd;

    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    keydb_expect_rejected(path, "p9fs", 100);

    keydb_write(path, valid, 1);
    fd = open(path, O_WRONLY | O_APPEND);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(write(fd, "x", 1), ==, 1);
    close(fd);
    keydb_expect_rejected(path, "p9fs", 100);

    keydb_write(path, duplicate, G_N_ELEMENTS(duplicate));
    keydb_expect_rejected(path, "p9fs", 100);
    keydb_write(path, canonical_duplicate, G_N_ELEMENTS(canonical_duplicate));
    keydb_expect_rejected(path, "123456789012345678901234567", 100);
    keydb_write(path, invalid, 1);
    keydb_expect_rejected(path, "p9fs", 100);
    keydb_write(path, disabled, 1);
    keydb_expect_rejected(path, "p9fs", 100);
    keydb_write(path, expired, 1);
    keydb_expect_rejected(path, "p9fs", 100);
    keydb_write(path, valid, 1);
    keydb_expect_rejected(path, "missing", 100);

    unlink(path);
    g_assert_cmpint(symlink("elsewhere", path), ==, 0);
    keydb_expect_rejected(path, "p9fs", 100);
    unlink(path);
    g_assert_cmpint(mkdir(path, 0700), ==, 0);
    keydb_expect_rejected(path, "p9fs", 100);
    rmdir(path);
    keydb_remove_tree(dir);
}

static void test_keydb_max_and_immutable_copy(void)
{
    KeydbFixtureRecord *records = g_new0(KeydbFixtureRecord,
                                         PLAN9_AUTH_KEYDB_MAX_RECORDS);
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    Plan9AuthKeydb *keydb;
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];

    for (size_t i = 0; i < PLAN9_AUTH_KEYDB_MAX_RECORDS; i++) {
        records[i].name = g_strdup_printf("u%zu", i);
        memset(records[i].key, i, sizeof(records[i].key));
    }
    g_free((char *)records[0].name);
    records[0].name = "p9fs";
    keydb_write(path, records, PLAN9_AUTH_KEYDB_MAX_RECORDS);
    keydb = keydb_load(path, "p9fs", 0);
    unlink(path);
    g_assert_cmpint(plan9_auth_keydb_lookup(keydb, "u1", 0, key), ==,
                    PLAN9_AUTH_KEY_AVAILABLE);
    for (size_t i = 0; i < sizeof(key); i++) {
        g_assert_cmpuint(key[i], ==, 1);
    }
    plan9_auth_keydb_free(keydb);
    keydb_remove_tree(dir);
    for (size_t i = 1; i < PLAN9_AUTH_KEYDB_MAX_RECORDS; i++) {
        g_free((char *)records[i].name);
    }
    g_free(records);
}

static void test_keydb_over_limit(void)
{
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    uint8_t byte = 0;

    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(lseek(fd, PLAN9_AUTH_KEYDB_RECORD_LEN *
                          PLAN9_AUTH_KEYDB_MAX_RECORDS, SEEK_SET), ==,
                    PLAN9_AUTH_KEYDB_RECORD_LEN * PLAN9_AUTH_KEYDB_MAX_RECORDS);
    g_assert_cmpint(write(fd, &byte, 1), ==, 1);
    close(fd);
    keydb_expect_rejected(path, "p9fs", 0);
    keydb_remove_tree(dir);
}

typedef struct KeydbRace {
    const char *path;
    const char *replacement;
} KeydbRace;

static void keydb_replace_after_read(const char *path, void *opaque)
{
    KeydbRace *race = opaque;

    g_assert_cmpstr(path, ==, race->path);
    g_assert_cmpint(rename(race->replacement, path), ==, 0);
}

static void test_keydb_changed_during_read(void)
{
    static const KeydbFixtureRecord original[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
    };
    static const KeydbFixtureRecord replacement[] = {
        { "p9fs", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    g_autofree char *replacement_path = g_build_filename(dir, "replacement",
                                                          NULL);
    KeydbRace race = { path, replacement_path };

    keydb_write(path, original, 1);
    keydb_write(replacement_path, replacement, 1);
    plan9_auth_keydb_set_read_hook(keydb_replace_after_read, &race);
    keydb_expect_rejected(path, "p9fs", 0);
    plan9_auth_keydb_set_read_hook(NULL, NULL);
    keydb_remove_tree(dir);
}

static void test_sizes(void)
{
    g_assert_cmpuint(PLAN9_AUTH_NAMELEN, ==, 28);
    g_assert_cmpuint(PLAN9_AUTH_DOMLEN, ==, 48);
    g_assert_cmpuint(PLAN9_AUTH_TICKET_REQUEST_LEN, ==, 141);
    g_assert_cmpuint(PLAN9_AUTH_TICKET_LEN, ==, 72);
    g_assert_cmpuint(PLAN9_AUTH_AUTHENTICATOR_LEN, ==, 13);
}

static void test_des56to64(void)
{
    static const uint8_t packed[PLAN9_AUTH_DES_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd,
    };
    static const uint8_t expected[8] = {
        0x01, 0x91, 0xd0, 0xad, 0x79, 0x4c, 0xae, 0x9b,
    };
    uint8_t expanded[8];

    plan9_auth_des56to64(packed, expanded);
    g_assert_cmpmem(expanded, sizeof(expanded), expected, sizeof(expected));
}

static void test_passtokey(void)
{
    static const uint8_t short_expected[PLAN9_AUTH_DES_KEY_LEN] = {
        0x61, 0xf1, 0x18, 0x00, 0x02, 0x81, 0x40,
    };
    static const uint8_t long_expected[PLAN9_AUTH_DES_KEY_LEN] = {
        0x26, 0x1b, 0x62, 0xdb, 0x16, 0x3c, 0x49,
    };
    uint8_t key[PLAN9_AUTH_DES_KEY_LEN];
    static const char password_27[] = "012345678901234567890123456";
    static const char password_28[] = "0123456789012345678901234567";
    static const char password_utf8_27[] =
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9" "a";
    static const char password_utf8_28[] =
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9"
        "\xc3\xa9\xc3\xa9\xc3\xa9\xc3\xa9";
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_passtokey(key, "", &err), ==, 0);
    g_assert_null(err);

    g_assert_cmpint(plan9_auth_passtokey(key, "abc", &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(key, sizeof(key), short_expected, sizeof(short_expected));

    g_assert_cmpint(plan9_auth_passtokey(key, "123456789", &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(key, sizeof(key), long_expected, sizeof(long_expected));

    g_assert_cmpuint(strlen(password_27), ==, PLAN9_AUTH_NAMELEN - 1);
    g_assert_cmpint(plan9_auth_passtokey(key, password_27, &err), ==, 0);
    g_assert_null(err);

    g_assert_cmpuint(strlen(password_28), ==, PLAN9_AUTH_NAMELEN);
    g_assert_cmpint(plan9_auth_passtokey(key, password_28, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    for (size_t i = 0; i < sizeof(key); i++) {
        g_assert_cmpuint(key[i], ==, 0);
    }

    g_assert_cmpuint(strlen(password_utf8_27), ==, PLAN9_AUTH_NAMELEN - 1);
    g_assert_cmpint(plan9_auth_passtokey(key, password_utf8_27, &err), ==, 0);
    g_assert_null(err);

    g_assert_cmpuint(strlen(password_utf8_28), ==, PLAN9_AUTH_NAMELEN);
    g_assert_cmpint(plan9_auth_passtokey(key, password_utf8_28, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    for (size_t i = 0; i < sizeof(key); i++) {
        g_assert_cmpuint(key[i], ==, 0);
    }
}

static void test_encrypt_vectors(void)
{
    static const uint8_t key[PLAN9_AUTH_DES_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd,
    };
    static const uint8_t plain8[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    static const uint8_t cipher8[] = {
        0x76, 0x3e, 0x78, 0xcf, 0xb5, 0x05, 0xed, 0xdd,
    };
    static const uint8_t cipher9[] = {
        0x76, 0x07, 0xa2, 0xbf, 0x54, 0x4c, 0x97, 0x1f, 0x1d,
    };
    static const uint8_t cipher14[] = {
        0x76, 0x3e, 0x78, 0xcf, 0xb5, 0x05, 0xd3,
        0x97, 0xa5, 0x76, 0xbe, 0x16, 0x81, 0xcd,
    };
    static const uint8_t cipher15[] = {
        0x76, 0x3e, 0x78, 0xcf, 0xb5, 0x05, 0xed, 0xd8,
        0x06, 0xdc, 0x3e, 0xb7, 0x81, 0xa4, 0x5c,
    };
    uint8_t buffer[sizeof(cipher15)];
    uint8_t wrong_key[PLAN9_AUTH_DES_KEY_LEN];
    Error *err = NULL;

    memcpy(buffer, plain8, sizeof(plain8));
    g_assert_cmpint(plan9_auth_encrypt(key, buffer, sizeof(plain8), &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(buffer, sizeof(plain8), cipher8, sizeof(cipher8));
    g_assert_cmpint(plan9_auth_decrypt(key, buffer, sizeof(plain8), &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpmem(buffer, sizeof(plain8), plain8, sizeof(plain8));

    for (size_t len = 9; len <= sizeof(buffer); len++) {
        for (size_t i = 0; i < len; i++) {
            buffer[i] = i;
        }
        g_assert_cmpint(plan9_auth_encrypt(key, buffer, len, &err), ==, 0);
        g_assert_null(err);
        if (len == 9) {
            g_assert_cmpmem(buffer, len, cipher9, sizeof(cipher9));
        } else if (len == 14) {
            g_assert_cmpmem(buffer, len, cipher14, sizeof(cipher14));
        } else if (len == 15) {
            g_assert_cmpmem(buffer, len, cipher15, sizeof(cipher15));
        }
        g_assert_cmpint(plan9_auth_decrypt(key, buffer, len, &err), ==, 0);
        g_assert_null(err);
        for (size_t i = 0; i < len; i++) {
            g_assert_cmpuint(buffer[i], ==, i);
        }
    }

    g_assert_cmpint(plan9_auth_encrypt(key, buffer, 7, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;

    memcpy(buffer, cipher8, sizeof(cipher8));
    memcpy(wrong_key, key, sizeof(wrong_key));
    wrong_key[0] ^= 0x80;
    g_assert_cmpint(plan9_auth_decrypt(wrong_key, buffer, sizeof(cipher8),
                                       &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpint(memcmp(buffer, plain8, sizeof(plain8)), !=, 0);
}

static void test_ticket_request_codec(void)
{
    Plan9AuthTicketRequest request = {
        .type = PLAN9_AUTH_TREQ,
        .authid = "p9fs",
        .authdom = "nextlab",
        .challenge = { 1, 2, 3, 4, 5, 6, 7, 8 },
        .hostid = "tor",
        .uid = "tor",
    };
    Plan9AuthTicketRequest decoded;
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire, &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(wire[0], ==, PLAN9_AUTH_TREQ);
    g_assert_cmpmem(wire + 1, 4, "p9fs", 4);
    g_assert_cmpmem(wire + 29, 7, "nextlab", 7);
    g_assert_cmpmem(wire + 77, 8, request.challenge, 8);
    g_assert_cmpmem(wire + 85, 3, "tor", 3);
    g_assert_cmpmem(wire + 113, 3, "tor", 3);
    g_assert_cmpint(plan9_auth_ticket_request_decode(wire, sizeof(wire),
                                                      &decoded, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(decoded.type, ==, request.type);
    g_assert_cmpstr(decoded.authid, ==, request.authid);
    g_assert_cmpstr(decoded.authdom, ==, request.authdom);
    g_assert_cmpmem(decoded.challenge, sizeof(decoded.challenge),
                    request.challenge, sizeof(request.challenge));

    request.uid[0] = 'x';
    memset(request.uid + 1, 'x', PLAN9_AUTH_NAMELEN - 1);
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_ticket_and_authenticator_codecs(void)
{
    Plan9AuthTicket ticket = {
        .num = PLAN9_AUTH_TC,
        .challenge = { 8, 7, 6, 5, 4, 3, 2, 1 },
        .cuid = "tor",
        .suid = "p9fs",
        .key = { 1, 2, 3, 4, 5, 6, 7 },
    };
    Plan9AuthTicket ticket_decoded;
    Plan9AuthAuthenticator auth = {
        .num = PLAN9_AUTH_AC,
        .challenge = { 1, 3, 3, 7, 0, 0, 0, 1 },
        .id = 0x78563412,
    };
    Plan9AuthAuthenticator auth_decoded;
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN];
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN];
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_ticket_encode(&ticket, ticket_wire, &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(ticket_wire[0], ==, PLAN9_AUTH_TC);
    g_assert_cmpmem(ticket_wire + 1, 8, ticket.challenge, 8);
    g_assert_cmpmem(ticket_wire + 9, 3, "tor", 3);
    g_assert_cmpmem(ticket_wire + 37, 4, "p9fs", 4);
    g_assert_cmpmem(ticket_wire + 65, 7, ticket.key, 7);
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire, sizeof(ticket_wire),
                                              &ticket_decoded, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(ticket_decoded.cuid, ==, ticket.cuid);
    g_assert_cmpstr(ticket_decoded.suid, ==, ticket.suid);
    g_assert_cmpmem(ticket_decoded.key, sizeof(ticket_decoded.key),
                    ticket.key, sizeof(ticket.key));

    g_assert_cmpint(plan9_auth_authenticator_encode(&auth, auth_wire, &err),
                    ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(auth_wire[0], ==, PLAN9_AUTH_AC);
    g_assert_cmpmem(auth_wire + 1, 8, auth.challenge, 8);
    g_assert_cmpmem(auth_wire + 9, 4, "\x12\x34\x56\x78", 4);
    g_assert_cmpint(plan9_auth_authenticator_decode(auth_wire,
                                                     sizeof(auth_wire),
                                                     &auth_decoded,
                                                     &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(auth_decoded.id, ==, auth.id);

    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                              sizeof(ticket_wire) - 1,
                                              &ticket_decoded, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_rejections_wipe_ticket_output(void)
{
    Plan9AuthTicket ticket = {
        .num = PLAN9_AUTH_TC,
        .challenge = { 8, 7, 6, 5, 4, 3, 2, 1 },
        .suid = "p9fs",
        .key = { 1, 2, 3, 4, 5, 6, 7 },
    };
    uint8_t wire[PLAN9_AUTH_TICKET_LEN];
    Error *err = NULL;

    memset(ticket.cuid, 'x', PLAN9_AUTH_NAMELEN);
    memset(wire, 0xa5, sizeof(wire));
    g_assert_cmpint(plan9_auth_ticket_encode(&ticket, wire, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    for (size_t i = 0; i < sizeof(wire); i++) {
        g_assert_cmpuint(wire[i], ==, 0);
    }
}

static void test_invalid_arguments(void)
{
    Error *err = NULL;

    g_assert_cmpint(plan9_auth_passtokey(NULL, "abc", &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(plan9_auth_ticket_decode(NULL, PLAN9_AUTH_TICKET_LEN,
                                             NULL, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void assert_zeroed(const void *data, size_t len)
{
    const uint8_t *bytes = data;

    for (size_t i = 0; i < len; i++) {
        g_assert_cmpuint(bytes[i], ==, 0);
    }
}

static void test_public_clear(void)
{
    uint8_t plaintext[PLAN9_AUTH_TICKET_LEN];
    Plan9AuthTicket ticket;

    memset(plaintext, 0xa5, sizeof(plaintext));
    plan9_auth_clear(plaintext, sizeof(plaintext));
    assert_zeroed(plaintext, sizeof(plaintext));

    memset(&ticket, 0xa5, sizeof(ticket));
    plan9_auth_ticket_clear(&ticket);
    assert_zeroed(&ticket, sizeof(ticket));
}

static void test_encode_failure_wipes_output(void)
{
    Plan9AuthTicketRequest request = {
        .authid = "p9fs",
        .authdom = "nextlab",
        .hostid = "tor",
        .uid = "tor",
    };
    Plan9AuthTicket ticket = {
        .cuid = "tor",
        .suid = "p9fs",
        .key = { 1, 2, 3, 4, 5, 6, 7 },
    };
    Plan9AuthAuthenticator auth = { 0 };
    uint8_t request_wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN];
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN];
    Error *err = NULL;

    memset(request_wire, 0xa5, sizeof(request_wire));
    g_assert_cmpint(plan9_auth_ticket_request_encode(NULL, request_wire, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(request_wire, sizeof(request_wire));
    err = NULL;

    memset(request.authid, 'x', PLAN9_AUTH_NAMELEN);
    memset(request_wire, 0xa5, sizeof(request_wire));
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, request_wire,
                                                      &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(request_wire, sizeof(request_wire));
    err = NULL;

    memset(ticket_wire, 0xa5, sizeof(ticket_wire));
    g_assert_cmpint(plan9_auth_ticket_encode(NULL, ticket_wire, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(ticket_wire, sizeof(ticket_wire));
    err = NULL;

    memset(auth_wire, 0xa5, sizeof(auth_wire));
    g_assert_cmpint(plan9_auth_authenticator_encode(NULL, auth_wire, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(auth_wire, sizeof(auth_wire));
    err = NULL;

    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, NULL, &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(plan9_auth_ticket_encode(&ticket, NULL, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_cmpint(plan9_auth_authenticator_encode(&auth, NULL, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_decode_failure_wipes_destination(void)
{
    uint8_t request_wire[PLAN9_AUTH_TICKET_REQUEST_LEN] = { 0 };
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN] = { 0 };
    uint8_t auth_wire[PLAN9_AUTH_AUTHENTICATOR_LEN] = { 0 };
    Plan9AuthTicketRequest request;
    Plan9AuthTicket ticket;
    Plan9AuthAuthenticator auth;
    Error *err = NULL;

    memset(&request, 0xa5, sizeof(request));
    g_assert_cmpint(plan9_auth_ticket_request_decode(NULL, sizeof(request_wire),
                                                      &request, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&request, sizeof(request));
    err = NULL;

    memset(&request, 0xa5, sizeof(request));
    g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                      sizeof(request_wire) - 1,
                                                      &request, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&request, sizeof(request));
    err = NULL;

    memset(&request, 0xa5, sizeof(request));
    g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                      sizeof(request_wire) + 1,
                                                      &request, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&request, sizeof(request));
    err = NULL;

    memset(&ticket, 0xa5, sizeof(ticket));
    g_assert_cmpint(plan9_auth_ticket_decode(NULL, sizeof(ticket_wire),
                                              &ticket, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&ticket, sizeof(ticket));
    err = NULL;

    memset(&ticket, 0xa5, sizeof(ticket));
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                              sizeof(ticket_wire) - 1,
                                              &ticket, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&ticket, sizeof(ticket));
    err = NULL;

    memset(&ticket, 0xa5, sizeof(ticket));
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                              sizeof(ticket_wire) + 1,
                                              &ticket, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&ticket, sizeof(ticket));
    err = NULL;

    memset(&auth, 0xa5, sizeof(auth));
    g_assert_cmpint(plan9_auth_authenticator_decode(NULL, sizeof(auth_wire),
                                                     &auth, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&auth, sizeof(auth));
    err = NULL;

    memset(&auth, 0xa5, sizeof(auth));
    g_assert_cmpint(plan9_auth_authenticator_decode(auth_wire,
                                                     sizeof(auth_wire) - 1,
                                                     &auth, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&auth, sizeof(auth));
    err = NULL;

    memset(&auth, 0xa5, sizeof(auth));
    g_assert_cmpint(plan9_auth_authenticator_decode(auth_wire,
                                                     sizeof(auth_wire) + 1,
                                                     &auth, &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    assert_zeroed(&auth, sizeof(auth));
}

static void test_decode_fixed_string_compatibility(void)
{
    uint8_t request_wire[PLAN9_AUTH_TICKET_REQUEST_LEN] = { 0 };
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN] = { 0 };
    Plan9AuthTicketRequest request;
    Plan9AuthTicket ticket;
    Error *err = NULL;
    const size_t request_fields[] = { 1, 29, 85, 113 };
    const size_t request_sizes[] = {
        PLAN9_AUTH_NAMELEN, PLAN9_AUTH_DOMLEN,
        PLAN9_AUTH_NAMELEN, PLAN9_AUTH_NAMELEN,
    };
    const size_t ticket_fields[] = { 9, 37 };

    for (size_t i = 0; i < G_N_ELEMENTS(request_fields); i++) {
        memset(request_wire, 0, sizeof(request_wire));
        memset(request_wire + request_fields[i], 'x', request_sizes[i]);
        char *const dest[] = {
            request.authid, request.authdom, request.hostid, request.uid,
        };

        memset(&request, 0xa5, sizeof(request));
        g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                          sizeof(request_wire),
                                                          &request, &err),
                        ==, 0);
        g_assert_null(err);
        g_assert_cmpuint(dest[i][request_sizes[i] - 2], ==, 'x');
        g_assert_cmpuint(dest[i][request_sizes[i] - 1], ==, 0);
        g_assert_cmpuint(dest[i][request_sizes[i]], ==, 0);
    }

    for (size_t i = 0; i < G_N_ELEMENTS(ticket_fields); i++) {
        memset(ticket_wire, 0, sizeof(ticket_wire));
        memset(ticket_wire + ticket_fields[i], 'x', PLAN9_AUTH_NAMELEN);
        char *const dest[] = { ticket.cuid, ticket.suid };

        memset(&ticket, 0xa5, sizeof(ticket));
        g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                                  sizeof(ticket_wire),
                                                  &ticket, &err), ==, 0);
        g_assert_null(err);
        g_assert_cmpuint(dest[i][PLAN9_AUTH_NAMELEN - 2], ==, 'x');
        g_assert_cmpuint(dest[i][PLAN9_AUTH_NAMELEN - 1], ==, 0);
        g_assert_cmpuint(dest[i][PLAN9_AUTH_NAMELEN], ==, 0);
    }

    memset(request_wire, 0, sizeof(request_wire));
    request_wire[1] = 'a';
    request_wire[2] = 0;
    request_wire[3] = 0xa5;
    request_wire[29] = 'd';
    request_wire[30] = 0;
    request_wire[31] = 0xa5;
    request_wire[85] = 'h';
    request_wire[86] = 0;
    request_wire[87] = 0xa5;
    request_wire[113] = 'u';
    request_wire[114] = 0;
    request_wire[115] = 0xa5;
    g_assert_cmpint(plan9_auth_ticket_request_decode(request_wire,
                                                      sizeof(request_wire),
                                                      &request, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(request.authid, ==, "a");
    g_assert_cmpstr(request.authdom, ==, "d");
    g_assert_cmpstr(request.hostid, ==, "h");
    g_assert_cmpstr(request.uid, ==, "u");
    g_assert_cmpuint((uint8_t)request.authid[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)request.authdom[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)request.hostid[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)request.uid[2], ==, 0xa5);

    memset(ticket_wire, 0, sizeof(ticket_wire));
    ticket_wire[9] = 'c';
    ticket_wire[10] = 0;
    ticket_wire[11] = 0xa5;
    ticket_wire[37] = 's';
    ticket_wire[38] = 0;
    ticket_wire[39] = 0xa5;
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire, sizeof(ticket_wire),
                                              &ticket, &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpstr(ticket.cuid, ==, "c");
    g_assert_cmpstr(ticket.suid, ==, "s");
    g_assert_cmpuint((uint8_t)ticket.cuid[2], ==, 0xa5);
    g_assert_cmpuint((uint8_t)ticket.suid[2], ==, 0xa5);
    plan9_auth_ticket_clear(&ticket);
    assert_zeroed(&ticket, sizeof(ticket));
}

typedef struct TicketServiceTransport {
    GByteArray *output;
    GByteArray *last_attempt;
    Plan9AuthTicketConnection **connection_slot;
    const uint8_t *reentrant_record;
    size_t reentrant_record_len;
    Error *reentrant_error;
    int reentrant_result;
    unsigned int send_calls;
    unsigned int close_calls;
    bool would_block;
    size_t short_send;
    int fatal_result;
    bool free_on_send;
    bool free_on_close;
    bool receive_on_send;
    bool can_send_on_send;
    unsigned int ready_then_eagain;
    bool ready_then_eagain_forever;
} TicketServiceTransport;

typedef struct TicketServiceRandom {
    uint8_t next;
    unsigned int calls;
    bool fail;
} TicketServiceRandom;

static int ticket_service_random(void *buf, size_t len, void *opaque,
                                 Error **errp)
{
    TicketServiceRandom *random = opaque;
    uint8_t *bytes = buf;

    random->calls++;
    if (random->fail) {
        error_setg(errp, "injected ticket random failure");
        return -1;
    }
    for (size_t i = 0; i < len; i++) {
        bytes[i] = random->next++;
    }
    return 0;
}

static uint32_t ticket_service_now(void *opaque)
{
    return *(uint32_t *)opaque;
}

static int ticket_service_send(const uint8_t *buf, size_t len, void *opaque)
{
    TicketServiceTransport *transport = opaque;
    size_t sent = transport->short_send ? MIN(len, transport->short_send) :
                                          len;

    transport->send_calls++;
    g_byte_array_set_size(transport->last_attempt, 0);
    g_byte_array_append(transport->last_attempt, buf, len);
    if (transport->ready_then_eagain ||
        transport->ready_then_eagain_forever) {
        if (transport->ready_then_eagain) {
            transport->ready_then_eagain--;
        }
        plan9_auth_ticket_connection_can_send(*transport->connection_slot);
        return -EAGAIN;
    }
    if (transport->would_block) {
        return -EAGAIN;
    }
    if (transport->fatal_result) {
        return transport->fatal_result;
    }
    g_byte_array_append(transport->output, buf, sent);
    if (transport->receive_on_send) {
        transport->receive_on_send = false;
        transport->reentrant_result =
            plan9_auth_ticket_connection_receive_record(
                *transport->connection_slot, transport->reentrant_record,
                transport->reentrant_record_len,
                &transport->reentrant_error);
    }
    if (transport->can_send_on_send) {
        transport->can_send_on_send = false;
        plan9_auth_ticket_connection_can_send(*transport->connection_slot);
    }
    if (transport->free_on_send) {
        plan9_auth_ticket_connection_free(*transport->connection_slot);
        *transport->connection_slot = NULL;
    }
    return sent;
}

static void ticket_service_close(void *opaque)
{
    TicketServiceTransport *transport = opaque;

    transport->close_calls++;
    if (transport->free_on_close) {
        plan9_auth_ticket_connection_free(*transport->connection_slot);
        *transport->connection_slot = NULL;
    }
}

static const Plan9AuthTicketTransportOps ticket_service_transport_ops = {
    .send_record = ticket_service_send,
    .close = ticket_service_close,
};

static void ticket_service_transport_init(TicketServiceTransport *transport)
{
    transport->output = g_byte_array_new();
    transport->last_attempt = g_byte_array_new();
}

static void ticket_service_transport_clear(TicketServiceTransport *transport)
{
    g_byte_array_unref(transport->output);
    g_byte_array_unref(transport->last_attempt);
}

static void test_ticket_service_success(void)
{
    static const KeydbFixtureRecord records[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
        { "tor", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
    };
    Plan9AuthTicketRequest request = {
        .type = PLAN9_AUTH_TREQ,
        .authid = "p9fs",
        .authdom = "cs.bell-labs.com",
        .challenge = { 0x10, 0x20, 0x30, 0x40,
                       0x50, 0x60, 0x70, 0x80 },
        .hostid = "tor",
        .uid = "tor",
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t client_wire[PLAN9_AUTH_TICKET_LEN];
    uint8_t server_wire[PLAN9_AUTH_TICKET_LEN];
    TicketServiceTransport transport = { 0 };
    TicketServiceRandom random = { .next = 0x20 };
    uint32_t now = 100;
    Plan9AuthTicketServiceConfig config;
    Plan9AuthTicketService *service;
    Plan9AuthTicketConnection *connection;
    Plan9AuthTicket client, server;
    uint8_t first_conversation[PLAN9_AUTH_DES_KEY_LEN];
    Plan9AuthKeydb *keydb;

    ticket_service_transport_init(&transport);
    keydb_write(path, records, G_N_ELEMENTS(records));
    keydb = keydb_load(path, "p9fs", 100);
    config = (Plan9AuthTicketServiceConfig) {
        .keydb = keydb,
        .now_seconds = ticket_service_now,
        .now_opaque = &now,
        .random_bytes = ticket_service_random,
        .random_opaque = &random,
    };
    service = plan9_auth_ticket_service_new(&config, &error_abort);
    connection = plan9_auth_ticket_connection_new(
        service, &ticket_service_transport_ops, &transport, &error_abort);
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire,
                                                      &error_abort), ==, 0);

    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        connection, wire, sizeof(wire), &error_abort), ==, 0);
    g_assert_cmpuint(transport.send_calls, ==, 1);
    g_assert_cmpuint(transport.close_calls, ==, 0);
    g_assert_cmpuint(transport.output->len, ==,
                     PLAN9_AUTH_TICKET_REPLY_LEN);
    g_assert_cmpuint(transport.output->data[0], ==, PLAN9_AUTH_OK);

    memcpy(client_wire, transport.output->data + 1, sizeof(client_wire));
    memcpy(server_wire, transport.output->data + 1 + sizeof(client_wire),
           sizeof(server_wire));
    g_assert_cmpint(plan9_auth_decrypt(records[1].key, client_wire,
                                        sizeof(client_wire), &error_abort),
                    ==, 0);
    g_assert_cmpint(plan9_auth_decrypt(records[0].key, server_wire,
                                        sizeof(server_wire), &error_abort),
                    ==, 0);
    g_assert_cmpint(plan9_auth_ticket_decode(client_wire, sizeof(client_wire),
                                              &client, &error_abort), ==, 0);
    g_assert_cmpint(plan9_auth_ticket_decode(server_wire, sizeof(server_wire),
                                              &server, &error_abort), ==, 0);
    g_assert_cmpuint(client.num, ==, PLAN9_AUTH_TC);
    g_assert_cmpuint(server.num, ==, PLAN9_AUTH_TS);
    g_assert_cmpmem(client.challenge, sizeof(client.challenge),
                    request.challenge, sizeof(request.challenge));
    g_assert_cmpmem(server.challenge, sizeof(server.challenge),
                    request.challenge, sizeof(request.challenge));
    g_assert_cmpstr(client.cuid, ==, "tor");
    g_assert_cmpstr(client.suid, ==, "tor");
    g_assert_cmpstr(server.cuid, ==, "tor");
    g_assert_cmpstr(server.suid, ==, "tor");
    g_assert_cmpmem(client.key, sizeof(client.key), server.key,
                    sizeof(server.key));
    g_assert_cmpuint(random.calls, ==, 3);
    memcpy(first_conversation, client.key, sizeof(first_conversation));

    request.challenge[0]++;
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire,
                                                      &error_abort), ==, 0);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        connection, wire, sizeof(wire), &error_abort), ==, 0);
    g_assert_cmpuint(transport.send_calls, ==, 2);
    g_assert_cmpuint(transport.close_calls, ==, 0);
    g_assert_cmpuint(transport.output->len, ==,
                     2 * PLAN9_AUTH_TICKET_REPLY_LEN);
    memcpy(client_wire,
           transport.output->data + PLAN9_AUTH_TICKET_REPLY_LEN + 1,
           sizeof(client_wire));
    memcpy(server_wire,
           transport.output->data + PLAN9_AUTH_TICKET_REPLY_LEN + 1 +
               PLAN9_AUTH_TICKET_LEN,
           sizeof(server_wire));
    g_assert_cmpint(plan9_auth_decrypt(records[1].key, client_wire,
                                        sizeof(client_wire), &error_abort),
                    ==, 0);
    g_assert_cmpint(plan9_auth_decrypt(records[0].key, server_wire,
                                        sizeof(server_wire), &error_abort),
                    ==, 0);
    g_assert_cmpint(plan9_auth_ticket_decode(client_wire, sizeof(client_wire),
                                              &client, &error_abort), ==, 0);
    g_assert_cmpint(plan9_auth_ticket_decode(server_wire, sizeof(server_wire),
                                              &server, &error_abort), ==, 0);
    g_assert_cmpmem(client.key, sizeof(client.key), server.key,
                    sizeof(server.key));
    g_assert_cmpmem(client.challenge, sizeof(client.challenge),
                    request.challenge, sizeof(request.challenge));
    g_assert_false(!memcmp(client.key, first_conversation,
                           sizeof(first_conversation)));
    g_assert_cmpuint(random.calls, ==, 6);

    plan9_auth_ticket_clear(&client);
    plan9_auth_ticket_clear(&server);
    plan9_auth_clear(client_wire, sizeof(client_wire));
    plan9_auth_clear(server_wire, sizeof(server_wire));
    plan9_auth_clear(first_conversation, sizeof(first_conversation));
    plan9_auth_ticket_connection_free(connection);
    plan9_auth_ticket_service_free(service);
    plan9_auth_keydb_free(keydb);
    ticket_service_transport_clear(&transport);
    keydb_remove_tree(dir);
}

static void test_ticket_service_rejects_speaks_for(void)
{
    static const KeydbFixtureRecord records[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
        { "tor", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
    };
    Plan9AuthTicketRequest request = {
        .type = PLAN9_AUTH_TREQ,
        .authid = "p9fs",
        .hostid = "tor",
        .uid = "alice",
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN];
    TicketServiceTransport transport = { 0 };
    TicketServiceRandom random = { .next = 0x40 };
    uint32_t now = 100;
    Plan9AuthKeydb *keydb;
    Plan9AuthTicketService *service;
    Plan9AuthTicketConnection *connection;
    Plan9AuthTicket ticket;

    ticket_service_transport_init(&transport);
    keydb_write(path, records, G_N_ELEMENTS(records));
    keydb = keydb_load(path, "p9fs", 100);
    service = plan9_auth_ticket_service_new(
        &(Plan9AuthTicketServiceConfig) {
            .keydb = keydb,
            .now_seconds = ticket_service_now,
            .now_opaque = &now,
            .random_bytes = ticket_service_random,
            .random_opaque = &random,
        }, &error_abort);
    connection = plan9_auth_ticket_connection_new(
        service, &ticket_service_transport_ops, &transport, &error_abort);
    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire,
                                                      &error_abort), ==, 0);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        connection, wire, sizeof(wire), &error_abort), ==, 0);
    memcpy(ticket_wire, transport.output->data + 1, sizeof(ticket_wire));
    g_assert_cmpint(plan9_auth_decrypt(records[1].key, ticket_wire,
                                        sizeof(ticket_wire), &error_abort),
                    ==, 0);
    g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire, sizeof(ticket_wire),
                                              &ticket, &error_abort), ==, 0);
    g_assert_cmpstr(ticket.cuid, ==, "alice");
    g_assert_cmpstr(ticket.suid, ==, "none");

    plan9_auth_ticket_clear(&ticket);
    plan9_auth_clear(ticket_wire, sizeof(ticket_wire));
    plan9_auth_ticket_connection_free(connection);
    plan9_auth_ticket_service_free(service);
    plan9_auth_keydb_free(keydb);
    ticket_service_transport_clear(&transport);
    keydb_remove_tree(dir);
}

static void test_ticket_service_privacy_substitution(void)
{
    static const KeydbFixtureRecord records[] = {
        { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
        { "tor", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
        { "disabled", { 3, 3, 3, 3, 3, 3, 3 }, 1, 0, 0 },
        { "old", { 4, 4, 4, 4, 4, 4, 4 }, 0, 0, 99 },
    };
    static const struct {
        const char *authid;
        const char *hostid;
        bool client_substitute;
        bool server_substitute;
    } cases[] = {
        { "p9fs", "missing", true, false },
        { "p9fs", "disabled", true, false },
        { "p9fs", "old", true, false },
        { "missing", "tor", false, true },
        { "disabled", "tor", false, true },
        { "old", "tor", false, true },
    };
    g_autofree char *dir = keydb_tempdir();
    g_autofree char *path = g_build_filename(dir, "keys", NULL);
    TicketServiceRandom random = { .next = 0x20 };
    uint32_t now = 100;
    Plan9AuthKeydb *keydb;
    Plan9AuthTicketService *service;

    keydb_write(path, records, G_N_ELEMENTS(records));
    keydb = keydb_load(path, "p9fs", 100);
    service = plan9_auth_ticket_service_new(
        &(Plan9AuthTicketServiceConfig) {
            .keydb = keydb,
            .now_seconds = ticket_service_now,
            .now_opaque = &now,
            .random_bytes = ticket_service_random,
            .random_opaque = &random,
        }, &error_abort);

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        Plan9AuthTicketRequest request = {
            .type = PLAN9_AUTH_TREQ,
            .uid = "tor",
        };
        TicketServiceTransport transport = { 0 };
        Plan9AuthTicketConnection *connection;
        uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
        uint8_t ticket_wire[PLAN9_AUTH_TICKET_LEN];
        uint8_t client_key[PLAN9_AUTH_DES_KEY_LEN];
        uint8_t server_key[PLAN9_AUTH_DES_KEY_LEN];
        Plan9AuthTicket ticket;
        uint8_t first_random = random.next;
        unsigned int calls = random.calls;

        g_strlcpy(request.authid, cases[i].authid, sizeof(request.authid));
        g_strlcpy(request.hostid, cases[i].hostid, sizeof(request.hostid));
        for (size_t n = 0; n < sizeof(client_key); n++) {
            client_key[n] = first_random + n;
            server_key[n] = first_random + sizeof(client_key) + n;
        }
        if (!cases[i].client_substitute) {
            memcpy(client_key, records[1].key, sizeof(client_key));
        }
        if (!cases[i].server_substitute) {
            memcpy(server_key, records[0].key, sizeof(server_key));
        }

        ticket_service_transport_init(&transport);
        connection = plan9_auth_ticket_connection_new(
            service, &ticket_service_transport_ops, &transport,
            &error_abort);
        g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire,
                                                          &error_abort),
                        ==, 0);
        g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                            connection, wire, sizeof(wire), &error_abort),
                        ==, 0);
        g_assert_cmpuint(random.calls, ==, calls + 3);
        g_assert_cmpuint(transport.output->len, ==,
                         PLAN9_AUTH_TICKET_REPLY_LEN);
        g_assert_cmpuint(transport.output->data[0], ==, PLAN9_AUTH_OK);
        g_assert_cmpuint(transport.close_calls, ==, 0);

        memcpy(ticket_wire, transport.output->data + 1,
               sizeof(ticket_wire));
        g_assert_cmpint(plan9_auth_decrypt(client_key, ticket_wire,
                                            sizeof(ticket_wire),
                                            &error_abort), ==, 0);
        g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                                  sizeof(ticket_wire),
                                                  &ticket, &error_abort),
                        ==, 0);
        g_assert_cmpuint(ticket.num, ==, PLAN9_AUTH_TC);
        plan9_auth_ticket_clear(&ticket);

        memcpy(ticket_wire,
               transport.output->data + 1 + PLAN9_AUTH_TICKET_LEN,
               sizeof(ticket_wire));
        g_assert_cmpint(plan9_auth_decrypt(server_key, ticket_wire,
                                            sizeof(ticket_wire),
                                            &error_abort), ==, 0);
        g_assert_cmpint(plan9_auth_ticket_decode(ticket_wire,
                                                  sizeof(ticket_wire),
                                                  &ticket, &error_abort),
                        ==, 0);
        g_assert_cmpuint(ticket.num, ==, PLAN9_AUTH_TS);

        plan9_auth_ticket_clear(&ticket);
        plan9_auth_clear(ticket_wire, sizeof(ticket_wire));
        plan9_auth_clear(client_key, sizeof(client_key));
        plan9_auth_clear(server_key, sizeof(server_key));
        plan9_auth_ticket_connection_free(connection);
        ticket_service_transport_clear(&transport);
    }

    plan9_auth_ticket_service_free(service);
    plan9_auth_keydb_free(keydb);
    keydb_remove_tree(dir);
}

static const KeydbFixtureRecord ticket_service_records[] = {
    { "p9fs", { 1, 2, 3, 4, 5, 6, 7 }, 0, 0, 0 },
    { "tor", { 7, 6, 5, 4, 3, 2, 1 }, 0, 0, 0 },
    { "disabled", { 3, 3, 3, 3, 3, 3, 3 }, 1, 0, 0 },
    { "old", { 4, 4, 4, 4, 4, 4, 4 }, 0, 0, 99 },
};

typedef struct TicketServiceHarness {
    char *dir;
    char *path;
    Plan9AuthKeydb *keydb;
    uint32_t now;
    TicketServiceRandom random;
    TicketServiceTransport transport;
    Plan9AuthTicketService *service;
    Plan9AuthTicketConnection *connection;
} TicketServiceHarness;

static void ticket_service_harness_init(TicketServiceHarness *harness)
{
    harness->dir = keydb_tempdir();
    harness->path = g_build_filename(harness->dir, "keys", NULL);
    harness->random.next = 0x20;
    harness->now = 100;
    ticket_service_transport_init(&harness->transport);
    harness->transport.connection_slot = &harness->connection;
    keydb_write(harness->path, ticket_service_records,
                G_N_ELEMENTS(ticket_service_records));
    harness->keydb = keydb_load(harness->path, "p9fs", 100);
    harness->service = plan9_auth_ticket_service_new(
        &(Plan9AuthTicketServiceConfig) {
            .keydb = harness->keydb,
            .now_seconds = ticket_service_now,
            .now_opaque = &harness->now,
            .random_bytes = ticket_service_random,
            .random_opaque = &harness->random,
        }, &error_abort);
    harness->connection = plan9_auth_ticket_connection_new(
        harness->service, &ticket_service_transport_ops,
        &harness->transport, &error_abort);
}

static void ticket_service_harness_clear(TicketServiceHarness *harness)
{
    plan9_auth_ticket_connection_free(harness->connection);
    plan9_auth_ticket_service_free(harness->service);
    plan9_auth_keydb_free(harness->keydb);
    ticket_service_transport_clear(&harness->transport);
    keydb_remove_tree(harness->dir);
    g_free(harness->path);
    g_free(harness->dir);
}

static void ticket_service_request_wire(
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN], uint8_t challenge)
{
    Plan9AuthTicketRequest request = {
        .type = PLAN9_AUTH_TREQ,
        .authid = "p9fs",
        .challenge = { challenge },
        .hostid = "tor",
        .uid = "tor",
    };

    g_assert_cmpint(plan9_auth_ticket_request_encode(&request, wire,
                                                      &error_abort), ==, 0);
}

static void ticket_service_expect_bad_record(const uint8_t *wire, size_t len)
{
    TicketServiceHarness harness = { 0 };
    uint8_t expected[PLAN9_AUTH_ERROR_REPLY_LEN] = { PLAN9_AUTH_ERR };

    memcpy(expected + 1, "protocol botch", strlen("protocol botch"));
    ticket_service_harness_init(&harness);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, len, &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.send_calls, ==, 1);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    g_assert_cmpuint(harness.transport.output->len, ==, sizeof(expected));
    g_assert_cmpmem(harness.transport.output->data,
                    harness.transport.output->len,
                    expected, sizeof(expected));
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_malformed_records(void)
{
    uint8_t wire[2 * PLAN9_AUTH_TICKET_REQUEST_LEN] = { 0 };

    ticket_service_expect_bad_record(NULL, 0);
    ticket_service_request_wire(wire, 1);
    ticket_service_expect_bad_record(wire,
                                     PLAN9_AUTH_TICKET_REQUEST_LEN - 1);
    ticket_service_expect_bad_record(wire,
                                     PLAN9_AUTH_TICKET_REQUEST_LEN + 1);
    memcpy(wire + PLAN9_AUTH_TICKET_REQUEST_LEN, wire,
           PLAN9_AUTH_TICKET_REQUEST_LEN);
    ticket_service_expect_bad_record(wire, sizeof(wire));
    wire[0] = PLAN9_AUTH_AC;
    ticket_service_expect_bad_record(wire,
                                     PLAN9_AUTH_TICKET_REQUEST_LEN);
}

static void test_ticket_service_backpressure(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t first_attempt[PLAN9_AUTH_TICKET_REPLY_LEN];

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 1);
    harness.transport.would_block = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.send_calls, ==, 1);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);
    g_assert_cmpuint(harness.transport.output->len, ==, 0);
    g_assert_cmpuint(harness.transport.last_attempt->len, ==,
                     sizeof(first_attempt));
    memcpy(first_attempt, harness.transport.last_attempt->data,
           sizeof(first_attempt));

    harness.transport.would_block = false;
    plan9_auth_ticket_connection_can_send(harness.connection);
    g_assert_cmpuint(harness.transport.send_calls, ==, 2);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_REPLY_LEN);
    g_assert_cmpmem(harness.transport.output->data,
                    harness.transport.output->len,
                    first_attempt, sizeof(first_attempt));
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);

    ticket_service_request_wire(wire, 2);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     2 * PLAN9_AUTH_TICKET_REPLY_LEN);
    plan9_auth_clear(first_attempt, sizeof(first_attempt));
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_error_backpressure(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t first_attempt[PLAN9_AUTH_ERROR_REPLY_LEN];

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 1);
    wire[0] = PLAN9_AUTH_AC;
    harness.transport.would_block = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.send_calls, ==, 1);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);
    g_assert_cmpuint(harness.transport.output->len, ==, 0);
    g_assert_cmpuint(harness.transport.last_attempt->len, ==,
                     sizeof(first_attempt));
    memcpy(first_attempt, harness.transport.last_attempt->data,
           sizeof(first_attempt));

    harness.transport.would_block = false;
    plan9_auth_ticket_connection_can_send(harness.connection);
    g_assert_cmpuint(harness.transport.send_calls, ==, 2);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_ERROR_REPLY_LEN);
    g_assert_cmpmem(harness.transport.output->data,
                    harness.transport.output->len,
                    first_attempt, sizeof(first_attempt));
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    plan9_auth_ticket_connection_can_send(harness.connection);
    g_assert_cmpuint(harness.transport.send_calls, ==, 2);

    plan9_auth_clear(first_attempt, sizeof(first_attempt));
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_rejects_record_while_reply_pending(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 1);
    harness.transport.would_block = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    ticket_service_request_wire(wire, 2);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    harness.transport.would_block = false;
    plan9_auth_ticket_connection_can_send(harness.connection);
    g_assert_cmpuint(harness.transport.send_calls, ==, 1);
    g_assert_cmpuint(harness.transport.output->len, ==, 0);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_request_bound(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    for (unsigned int i = 0; i < PLAN9_AUTH_TICKET_MAX_REQUESTS; i++) {
        ticket_service_request_wire(wire, i);
        g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                            harness.connection, wire, sizeof(wire),
                            &error_abort), ==, 0);
    }
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_MAX_REQUESTS *
                         PLAN9_AUTH_TICKET_REPLY_LEN);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);
    ticket_service_request_wire(wire, PLAN9_AUTH_TICKET_MAX_REQUESTS);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    g_assert_cmpuint(harness.random.calls, ==,
                     3 * PLAN9_AUTH_TICKET_MAX_REQUESTS);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_transport_failures(void)
{
    for (unsigned int short_send = 0; short_send < 2; short_send++) {
        TicketServiceHarness harness = { 0 };
        uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
        Error *err = NULL;

        ticket_service_harness_init(&harness);
        ticket_service_request_wire(wire, 1);
        if (short_send) {
            harness.transport.short_send = PLAN9_AUTH_TICKET_REPLY_LEN - 1;
        } else {
            harness.transport.fatal_result = -EIO;
        }
        g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                            harness.connection, wire, sizeof(wire), &err),
                        <, 0);
        g_assert_nonnull(err);
        error_free(err);
        g_assert_cmpuint(harness.transport.close_calls, ==, 1);
        ticket_service_harness_clear(&harness);
    }
}

static void test_ticket_service_random_failure(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 1);
    harness.random.fail = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpuint(harness.transport.send_calls, ==, 0);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    g_assert_cmpuint(harness.transport.output->len, ==, 0);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_reentrant_cleanup(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 1);
    harness.transport.free_on_send = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_null(harness.connection);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_REPLY_LEN);
    ticket_service_harness_clear(&harness);

    memset(&harness, 0, sizeof(harness));
    ticket_service_harness_init(&harness);
    harness.transport.free_on_close = true;
    err = NULL;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, NULL, 0, &err), ==, 0);
    g_assert_null(err);
    g_assert_null(harness.connection);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_reentrant_record(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t first[PLAN9_AUTH_TICKET_REQUEST_LEN];
    uint8_t second[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(first, 1);
    ticket_service_request_wire(second, 2);
    harness.transport.receive_on_send = true;
    harness.transport.reentrant_record = second;
    harness.transport.reentrant_record_len = sizeof(second);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, first, sizeof(first), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpint(harness.transport.reentrant_result, <, 0);
    g_assert_nonnull(harness.transport.reentrant_error);
    error_free(harness.transport.reentrant_error);
    harness.transport.reentrant_error = NULL;
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);

    err = NULL;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, second, sizeof(second), &err),
                    <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_reentrant_can_send(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 0);
    harness.transport.can_send_on_send = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), ==, 0);
    g_assert_null(err);
    g_assert_cmpuint(harness.transport.send_calls, ==, 1);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_REPLY_LEN);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);

    for (unsigned int i = 1; i < PLAN9_AUTH_TICKET_MAX_REQUESTS; i++) {
        ticket_service_request_wire(wire, i);
        g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                            harness.connection, wire, sizeof(wire),
                            &error_abort), ==, 0);
    }
    g_assert_cmpuint(harness.transport.send_calls, ==,
                     PLAN9_AUTH_TICKET_MAX_REQUESTS);
    ticket_service_request_wire(wire, PLAN9_AUTH_TICKET_MAX_REQUESTS);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_deferred_can_send(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];
    Error *err = NULL;

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 0);
    harness.transport.ready_then_eagain = 1;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.send_calls, ==, 2);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_REPLY_LEN);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);

    for (unsigned int i = 1; i < PLAN9_AUTH_TICKET_MAX_REQUESTS; i++) {
        ticket_service_request_wire(wire, i);
        g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                            harness.connection, wire, sizeof(wire),
                            &error_abort), ==, 0);
    }
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_MAX_REQUESTS *
                         PLAN9_AUTH_TICKET_REPLY_LEN);
    ticket_service_request_wire(wire, PLAN9_AUTH_TICKET_MAX_REQUESTS);
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire), &err), <, 0);
    g_assert_nonnull(err);
    error_free(err);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_deferred_error_can_send(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 0);
    wire[0] = PLAN9_AUTH_AC;
    harness.transport.ready_then_eagain = 1;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.send_calls, ==, 2);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_ERROR_REPLY_LEN);
    g_assert_cmpuint(harness.transport.close_calls, ==, 1);
    ticket_service_harness_clear(&harness);
}

static void test_ticket_service_deferred_can_send_bounded(void)
{
    TicketServiceHarness harness = { 0 };
    uint8_t wire[PLAN9_AUTH_TICKET_REQUEST_LEN];

    ticket_service_harness_init(&harness);
    ticket_service_request_wire(wire, 0);
    harness.transport.ready_then_eagain_forever = true;
    g_assert_cmpint(plan9_auth_ticket_connection_receive_record(
                        harness.connection, wire, sizeof(wire),
                        &error_abort), ==, 0);
    g_assert_cmpuint(harness.transport.send_calls, ==, 2);
    g_assert_cmpuint(harness.transport.output->len, ==, 0);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);

    plan9_auth_ticket_connection_can_send(harness.connection);
    g_assert_cmpuint(harness.transport.send_calls, ==, 4);
    g_assert_cmpuint(harness.transport.output->len, ==, 0);

    harness.transport.ready_then_eagain_forever = false;
    plan9_auth_ticket_connection_can_send(harness.connection);
    g_assert_cmpuint(harness.transport.send_calls, ==, 5);
    g_assert_cmpuint(harness.transport.output->len, ==,
                     PLAN9_AUTH_TICKET_REPLY_LEN);
    g_assert_cmpuint(harness.transport.close_calls, ==, 0);
    ticket_service_harness_clear(&harness);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/plan9-auth/sizes", test_sizes);
    g_test_add_func("/plan9-auth/des56to64", test_des56to64);
    g_test_add_func("/plan9-auth/passtokey", test_passtokey);
    g_test_add_func("/plan9-auth/encrypt", test_encrypt_vectors);
    g_test_add_func("/plan9-auth/ticket-request", test_ticket_request_codec);
    g_test_add_func("/plan9-auth/ticket-authenticator",
                    test_ticket_and_authenticator_codecs);
    g_test_add_func("/plan9-auth/rejections-wipe-ticket-output",
                    test_rejections_wipe_ticket_output);
    g_test_add_func("/plan9-auth/invalid-arguments", test_invalid_arguments);
    g_test_add_func("/plan9-auth/public-clear", test_public_clear);
    g_test_add_func("/plan9-auth/encode-failure-wipes-output",
                    test_encode_failure_wipes_output);
    g_test_add_func("/plan9-auth/decode-failure-wipes-destination",
                    test_decode_failure_wipes_destination);
    g_test_add_func("/plan9-auth/decode-fixed-string-compatibility",
                    test_decode_fixed_string_compatibility);
    g_test_add_func("/plan9-auth/keydb-golden-and-lookup",
                    test_keydb_golden_and_lookup);
    g_test_add_func("/plan9-auth/keydb-lookup-full-scan",
                    test_keydb_lookup_full_scan);
    g_test_add_func("/plan9-auth/keydb-record-encoder",
                    test_keydb_record_encoder);
    g_test_add_func("/plan9-auth/keydb-tool/non-tty",
                    test_keydb_tool_rejects_non_tty);
    g_test_add_func("/plan9-auth/keydb-tool/malformed-command",
                    test_keydb_tool_rejects_malformed_command);
#ifdef HAVE_OPENPTY
    g_test_add_func("/plan9-auth/keydb-tool/success",
                    test_keydb_tool_success);
    g_test_add_func("/plan9-auth/keydb-tool/password-mismatch",
                    test_keydb_tool_password_mismatch);
    g_test_add_func("/plan9-auth/keydb-tool/rejects-unrepresentable",
                    test_keydb_tool_rejects_unrepresentable);
    g_test_add_func("/plan9-auth/keydb-tool/signal-restores-terminal",
                    test_keydb_tool_signal_restores_terminal);
    g_test_add_func("/plan9-auth/keydb-tool/signal-transaction-stages",
                    test_keydb_tool_signal_transaction_stages);
    g_test_add_func("/plan9-auth/keydb-tool/restore-error-combined",
                    test_keydb_tool_restore_error_is_combined);
    g_test_add_func("/plan9-auth/keydb-tool/temp-fstat-cleanup",
                    test_keydb_tool_temp_fstat_failure_cleans);
    g_test_add_func("/plan9-auth/keydb-tool/collision-refusal",
                    test_keydb_tool_collision_refusal);
    g_test_add_func("/plan9-auth/keydb-tool/insecure-parent",
                    test_keydb_tool_rejects_insecure_parent);
    g_test_add_func("/plan9-auth/keydb-tool/parent-symlink",
                    test_keydb_tool_rejects_parent_symlink);
    g_test_add_func("/plan9-auth/keydb-tool/concurrent-winner",
                    test_keydb_tool_concurrent_winner);
    g_test_add_func("/plan9-auth/keydb-tool/late-collision-rollback",
                    test_keydb_tool_late_collision_rollback);
#endif
    g_test_add_func("/plan9-auth/keydb-rejections", test_keydb_rejections);
    g_test_add_func("/plan9-auth/keydb-max-and-immutable-copy",
                    test_keydb_max_and_immutable_copy);
    g_test_add_func("/plan9-auth/keydb-over-limit", test_keydb_over_limit);
    g_test_add_func("/plan9-auth/keydb-changed-during-read",
                    test_keydb_changed_during_read);
    g_test_add_func("/plan9-auth/ticket-service/success",
                    test_ticket_service_success);
    g_test_add_func("/plan9-auth/ticket-service/speaks-for-none",
                    test_ticket_service_rejects_speaks_for);
    g_test_add_func("/plan9-auth/ticket-service/privacy-substitution",
                    test_ticket_service_privacy_substitution);
    g_test_add_func("/plan9-auth/ticket-service/malformed-records",
                    test_ticket_service_malformed_records);
    g_test_add_func("/plan9-auth/ticket-service/backpressure",
                    test_ticket_service_backpressure);
    g_test_add_func("/plan9-auth/ticket-service/error-backpressure",
                    test_ticket_service_error_backpressure);
    g_test_add_func("/plan9-auth/ticket-service/pending-record",
                    test_ticket_service_rejects_record_while_reply_pending);
    g_test_add_func("/plan9-auth/ticket-service/request-bound",
                    test_ticket_service_request_bound);
    g_test_add_func("/plan9-auth/ticket-service/transport-failures",
                    test_ticket_service_transport_failures);
    g_test_add_func("/plan9-auth/ticket-service/random-failure",
                    test_ticket_service_random_failure);
    g_test_add_func("/plan9-auth/ticket-service/reentrant-cleanup",
                    test_ticket_service_reentrant_cleanup);
    g_test_add_func("/plan9-auth/ticket-service/reentrant-record",
                    test_ticket_service_reentrant_record);
    g_test_add_func("/plan9-auth/ticket-service/reentrant-can-send",
                    test_ticket_service_reentrant_can_send);
    g_test_add_func("/plan9-auth/ticket-service/deferred-can-send",
                    test_ticket_service_deferred_can_send);
    g_test_add_func("/plan9-auth/ticket-service/deferred-error-can-send",
                    test_ticket_service_deferred_error_can_send);
    g_test_add_func("/plan9-auth/ticket-service/deferred-can-send-bounded",
                    test_ticket_service_deferred_can_send_bounded);
    return g_test_run();
}
