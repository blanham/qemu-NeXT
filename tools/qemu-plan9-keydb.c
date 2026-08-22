/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include <termios.h>
#ifdef CONFIG_LINUX
#include <sys/syscall.h>
#endif

#include "crypto/init.h"
#include "crypto/random.h"
#include "hw/9pfs/plan9-auth.h"
#include "qapi/error.h"
#include "qemu/error-report.h"

#ifndef RENAME_NOREPLACE
#define RENAME_NOREPLACE (1U << 0)
#endif

typedef struct ProvisionOutput {
    char *path;
    char *dirpath;
    char *base;
    char *temp;
    int dirfd;
    int fd;
    struct stat inode;
    bool inode_valid;
    bool target_exists;
    bool temp_exists;
    bool published;
} ProvisionOutput;

typedef struct ProvisionSignalScope {
    struct sigaction old[4];
    size_t installed;
} ProvisionSignalScope;

static const int provision_signals[] = { SIGINT, SIGTERM, SIGHUP, SIGQUIT };

static volatile sig_atomic_t interrupted_signal;

static void provision_signal_handler(int sig)
{
    interrupted_signal = sig;
}

static int provision_signal_scope_enter(ProvisionSignalScope *scope,
                                        Error **errp)
{
    struct sigaction action = { 0 };

    interrupted_signal = 0;
    action.sa_handler = provision_signal_handler;
    sigemptyset(&action.sa_mask);
    while (scope->installed < G_N_ELEMENTS(provision_signals)) {
        size_t i = scope->installed;

        if (sigaction(provision_signals[i], &action, &scope->old[i]) < 0) {
            error_setg_errno(errp, errno, "cannot install signal handler");
            return -1;
        }
        scope->installed++;
    }
    return 0;
}

static void provision_signal_scope_leave(ProvisionSignalScope *scope)
{
    while (scope->installed) {
        scope->installed--;
        sigaction(provision_signals[scope->installed],
                  &scope->old[scope->installed], NULL);
    }
}

static int provision_check_interrupted(Error **errp)
{
    if (!interrupted_signal) {
        return 0;
    }
    if (!*errp) {
        error_setg(errp, "provisioning interrupted by signal %d",
                   interrupted_signal);
    }
    return -1;
}

static void error_set_or_append_errno(Error **errp, int os_errno,
                                      const char *message)
{
    if (*errp) {
        error_append_hint(errp, "Additionally, %s: %s\n", message,
                          strerror(os_errno));
    } else {
        error_setg_errno(errp, os_errno, "%s", message);
    }
}

static void provision_test_pause(const char *stage)
{
#ifdef PLAN9_KEYDB_TESTING
    if (!g_strcmp0(g_getenv("QEMU_PLAN9_KEYDB_TEST_STAGE"), stage)) {
        raise(SIGSTOP);
    }
#endif
}

static bool same_inode(const struct stat *a, const struct stat *b)
{
    return a->st_dev == b->st_dev && a->st_ino == b->st_ino;
}

static void output_init_empty(ProvisionOutput *output)
{
    memset(output, 0, sizeof(*output));
    output->dirfd = -1;
    output->fd = -1;
}

static int output_prepare(ProvisionOutput *output, const char *path,
                          Error **errp)
{
    struct stat parent;
    struct stat target;

    output->path = g_strdup(path);
    output->dirpath = g_path_get_dirname(path);
    output->base = g_path_get_basename(path);
    if (!path[0] || !strcmp(output->base, ".") ||
        !strcmp(output->base, "..")) {
        error_setg(errp, "invalid output path");
        return -1;
    }
    output->dirfd = open(output->dirpath,
                         O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (output->dirfd < 0) {
        error_setg_errno(errp, errno, "cannot open output directory");
        return -1;
    }
    if (fstat(output->dirfd, &parent) < 0) {
        error_setg_errno(errp, errno, "cannot inspect output directory");
        return -1;
    }
    if (!S_ISDIR(parent.st_mode) || parent.st_uid != geteuid() ||
        (parent.st_mode & 07777) != 0700) {
        error_setg(errp, "output directory must be owned by the effective "
                   "user and have mode 0700: %s", output->dirpath);
        return -1;
    }
    if (fstatat(output->dirfd, output->base, &target,
                AT_SYMLINK_NOFOLLOW) == 0) {
        output->target_exists = true;
        return 0;
    }
    if (errno != ENOENT) {
        error_setg_errno(errp, errno, "cannot inspect output path");
        return -1;
    }
    return 0;
}

static bool outputs_equal(const ProvisionOutput *a, const ProvisionOutput *b)
{
    struct stat adir, bdir;

    return !strcmp(a->base, b->base) &&
           fstat(a->dirfd, &adir) == 0 && fstat(b->dirfd, &bdir) == 0 &&
           same_inode(&adir, &bdir);
}

static int output_create_temp(ProvisionOutput *output, Error **errp)
{
    uint8_t random[8];
    char suffix[sizeof(random) * 2 + 1];

    for (unsigned int attempt = 0; attempt < 128; attempt++) {
        if (provision_check_interrupted(errp)) {
            goto fail;
        }
        if (qcrypto_random_bytes(random, sizeof(random), errp) < 0) {
            goto fail;
        }
        for (size_t i = 0; i < sizeof(random); i++) {
            snprintf(suffix + i * 2, 3, "%02x", random[i]);
        }
        g_free(output->temp);
        output->temp = g_strdup_printf(".qemu-plan9-keydb-%s", suffix);
        output->fd = openat(output->dirfd, output->temp,
                            O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC |
                            O_NOFOLLOW, 0600);
        if (output->fd >= 0) {
            output->temp_exists = true;
            if (provision_check_interrupted(errp)) {
                goto fail;
            }
#ifdef PLAN9_KEYDB_TESTING
            if (!g_strcmp0(g_getenv("QEMU_PLAN9_KEYDB_TEST_STAGE"),
                           "temp-fstat-error")) {
                errno = EIO;
                error_setg_errno(errp, errno,
                                 "cannot inspect temporary output");
                goto fail;
            }
#endif
            if (fstat(output->fd, &output->inode) < 0) {
                error_setg_errno(errp, errno,
                                 "cannot inspect temporary output");
                goto fail;
            }
            output->inode_valid = true;
            if (fchmod(output->fd, 0600) < 0) {
                error_setg_errno(errp, errno,
                                 "cannot secure temporary output");
                goto fail;
            }
            if (provision_check_interrupted(errp)) {
                goto fail;
            }
            plan9_auth_clear(random, sizeof(random));
            plan9_auth_clear(suffix, sizeof(suffix));
            return 0;
        }
        if (provision_check_interrupted(errp)) {
            goto fail;
        }
        if (errno != EEXIST) {
            error_setg_errno(errp, errno, "cannot create temporary output");
            goto fail;
        }
    }
    error_setg(errp, "cannot allocate a unique temporary output");

fail:
    plan9_auth_clear(random, sizeof(random));
    plan9_auth_clear(suffix, sizeof(suffix));
    return -1;
}

static int output_write_close(ProvisionOutput *output, const void *data,
                              size_t len, Error **errp)
{
    if (provision_check_interrupted(errp)) {
        return -1;
    }
    if (qemu_write_full(output->fd, data, len) != len) {
        error_setg_errno(errp, errno, "cannot write temporary output");
        return -1;
    }
    if (provision_check_interrupted(errp)) {
        return -1;
    }
    if (fsync(output->fd) < 0) {
        error_setg_errno(errp, errno, "cannot sync temporary output");
        return -1;
    }
    if (provision_check_interrupted(errp)) {
        return -1;
    }
    if (close(output->fd) < 0) {
        output->fd = -1;
        error_setg_errno(errp, errno, "cannot close temporary output");
        return -1;
    }
    output->fd = -1;
    return 0;
}

static int output_publish(ProvisionOutput *output, Error **errp)
{
    struct stat source;

    if (provision_check_interrupted(errp)) {
        return -1;
    }
    /*
     * Revalidate at the last possible point.  The containing directory is an
     * euid-owned 0700 directory opened without following its final symlink,
     * so a different user cannot replace this name between the check and the
     * namespace operation.
     */
    if (!output->inode_valid ||
        fstatat(output->dirfd, output->temp, &source,
                AT_SYMLINK_NOFOLLOW) < 0 ||
        !S_ISREG(source.st_mode) || !same_inode(&source, &output->inode)) {
        error_setg(errp, "temporary output changed before publication");
        return -1;
    }
#if defined(CONFIG_LINUX) && defined(SYS_renameat2)
    if (syscall(SYS_renameat2, output->dirfd, output->temp,
                output->dirfd, output->base, RENAME_NOREPLACE) == 0) {
        output->temp_exists = false;
        output->published = true;
        goto sync;
    }
    if (errno != ENOSYS && errno != EINVAL && errno != EOPNOTSUPP) {
        error_setg_errno(errp, errno, "cannot publish output without replace");
        return -1;
    }
    if (provision_check_interrupted(errp) ||
        fstatat(output->dirfd, output->temp, &source,
                AT_SYMLINK_NOFOLLOW) < 0 ||
        !S_ISREG(source.st_mode) || !same_inode(&source, &output->inode)) {
        if (!*errp) {
            error_setg(errp, "temporary output changed before publication");
        }
        return -1;
    }
#endif
    if (linkat(output->dirfd, output->temp, output->dirfd, output->base,
               0) < 0) {
        error_setg_errno(errp, errno, "cannot publish output without replace");
        return -1;
    }
    output->published = true;
    if (provision_check_interrupted(errp)) {
        return -1;
    }
    if (unlinkat(output->dirfd, output->temp, 0) < 0) {
        error_setg_errno(errp, errno, "cannot remove temporary output link");
        return -1;
    }
    output->temp_exists = false;

sync:
    if (fsync(output->dirfd) < 0) {
        error_setg_errno(errp, errno, "cannot sync output directory");
        return -1;
    }
    if (provision_check_interrupted(errp)) {
        return -1;
    }
    return 0;
}

static void output_unlink_verified(ProvisionOutput *output, bool published)
{
    const char *name = published ? output->base : output->temp;
    struct stat current;

    if (!name || !output->inode_valid ||
        fstatat(output->dirfd, name, &current,
                         AT_SYMLINK_NOFOLLOW) < 0 ||
        !S_ISREG(current.st_mode) || !same_inode(&current, &output->inode)) {
        return;
    }
    if (unlinkat(output->dirfd, name, 0) == 0) {
        fsync(output->dirfd);
    }
}

static void output_cleanup(ProvisionOutput *output, bool rollback)
{
    if (output->temp_exists && !output->inode_valid && output->fd >= 0) {
#ifdef PLAN9_KEYDB_TESTING
        bool inject_failure = !g_strcmp0(
            g_getenv("QEMU_PLAN9_KEYDB_TEST_STAGE"), "temp-fstat-error");
#else
        bool inject_failure = false;
#endif

        if (!inject_failure && fstat(output->fd, &output->inode) == 0) {
            output->inode_valid = true;
        }
    }
    /*
     * POSIX has no compare-and-unlink primitive.  The immediately adjacent
     * inode check is safe against other users because output_prepare() holds
     * an euid-owned mode-0700 directory fd.  If the first fstat() itself
     * failed, the still-open O_EXCL file has never been published and its
     * random name can be unlinked within that same private directory.  A
     * process with the same uid has equivalent authority over both this
     * process and the directory.
     */
    if (output->temp_exists) {
        if (output->inode_valid) {
            output_unlink_verified(output, false);
        } else if (output->fd >= 0 &&
                   unlinkat(output->dirfd, output->temp, 0) == 0) {
            fsync(output->dirfd);
        }
        output->temp_exists = false;
    }
    if (output->fd >= 0) {
        close(output->fd);
        output->fd = -1;
    }
    if (rollback && output->published) {
        output_unlink_verified(output, true);
        output->published = false;
    }
    if (output->dirfd >= 0) {
        close(output->dirfd);
    }
    g_free(output->path);
    g_free(output->dirpath);
    g_free(output->base);
    g_free(output->temp);
    output_init_empty(output);
}

static int write_prompt(int ttyfd, const char *prompt, Error **errp)
{
    size_t len = strlen(prompt);

    if (provision_check_interrupted(errp)) {
        return -1;
    }
    if (qemu_write_full(ttyfd, prompt, len) != len) {
        error_setg_errno(errp, errno, "cannot write password prompt");
        return -1;
    }
    return 0;
}

static int read_password_line(int ttyfd,
                              char password[PLAN9_AUTH_NAMELEN + 1],
                              Error **errp)
{
    size_t len = 0;
    bool overflow = false;
    char byte;
    ssize_t got;

    for (;;) {
        if (interrupted_signal) {
            error_setg(errp, "password input interrupted");
            return -1;
        }
        got = read(ttyfd, &byte, 1);
        if (got < 0 && errno == EINTR) {
            if (interrupted_signal) {
                error_setg(errp, "password input interrupted");
                return -1;
            }
            continue;
        }
        if (got <= 0) {
            error_setg(errp, "cannot read password from terminal");
            return -1;
        }
        if (byte == '\n' || byte == '\r') {
            break;
        }
        if (byte == '\0') {
            overflow = true;
            continue;
        }
        if (len < PLAN9_AUTH_NAMELEN) {
            password[len++] = byte;
        } else {
            overflow = true;
        }
    }
    password[MIN(len, (size_t)PLAN9_AUTH_NAMELEN)] = 0;
    if (overflow || len >= PLAN9_AUTH_NAMELEN) {
        error_setg(errp, "password is not representable in Plan 9");
        return -1;
    }
    return 0;
}

static int read_password_pair(char password[PLAN9_AUTH_NAMELEN + 1],
                              Error **errp)
{
    struct termios saved, hidden;
    char confirmation[PLAN9_AUTH_NAMELEN + 1] = { 0 };
    bool changed = false;
    int ttyfd = -1;
    int ret = -1;

    ttyfd = open("/dev/tty", O_RDWR | O_CLOEXEC | O_NOCTTY);
    if (ttyfd < 0 || !isatty(ttyfd) || tcgetattr(ttyfd, &saved) < 0) {
        error_setg(errp, "password input requires a terminal");
        goto out;
    }
    hidden = saved;
    hidden.c_lflag &= ~(ECHO | ECHONL);
    if (tcsetattr(ttyfd, TCSANOW, &hidden) < 0) {
        error_setg_errno(errp, errno, "cannot disable terminal echo");
        goto out;
    }
    changed = true;
    if (write_prompt(ttyfd, "tor password: ", errp) ||
        read_password_line(ttyfd, password, errp) ||
        write_prompt(ttyfd, "\nconfirm tor password: ", errp) ||
        read_password_line(ttyfd, confirmation, errp) ||
        write_prompt(ttyfd, "\n", errp)) {
        goto out;
    }
    if (strcmp(password, confirmation)) {
        error_setg(errp, "password confirmation does not match");
        goto out;
    }
    ret = 0;

out:
    if (changed) {
        int restore_ret = tcsetattr(ttyfd, TCSANOW, &saved);
        int saved_errno = errno;

#ifdef PLAN9_KEYDB_TESTING
        if (!g_strcmp0(g_getenv("QEMU_PLAN9_KEYDB_TEST_STAGE"),
                       "restore-error")) {
            restore_ret = -1;
            saved_errno = EIO;
        }
#endif
        if (restore_ret < 0) {
            error_set_or_append_errno(errp, saved_errno,
                                      "cannot restore terminal state");
            ret = -1;
        }
    }
    if (ttyfd >= 0 && close(ttyfd) < 0) {
        int saved_errno = errno;

        error_set_or_append_errno(errp, saved_errno,
                                  "cannot close password terminal");
        ret = -1;
    }
    plan9_auth_clear(confirmation, sizeof(confirmation));
    if (ret < 0) {
        plan9_auth_clear(password, PLAN9_AUTH_NAMELEN + 1);
    }
    return ret;
}

static void usage(void)
{
    error_report("usage: qemu-plan9-keydb create --keydb PATH "
                 "--secret PATH --server-id ID");
    error_report("Both parent directories must be owned by the effective "
                 "user with mode 0700.");
    error_report("Secret is the commit marker. An uncatchable signal or "
                 "power loss can leave only the key database; inspect and "
                 "remove that file manually before retrying.");
}

int main(int argc, char **argv)
{
    const char *keydb_path = NULL, *secret_path = NULL, *server_id = NULL;
    ProvisionOutput keydb, secret;
    uint8_t master_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint8_t server_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint8_t tor_key[PLAN9_AUTH_DES_KEY_LEN] = { 0 };
    uint8_t records[2 * PLAN9_AUTH_KEYDB_RECORD_LEN] = { 0 };
    char password[PLAN9_AUTH_NAMELEN + 1] = { 0 };
    char *base64 = NULL;
    Error *err = NULL;
    ProvisionSignalScope signal_scope = { 0 };
    int ret = EXIT_FAILURE;
    bool rollback = true;

    error_init(argv[0]);
    qcrypto_init(&error_fatal);
    output_init_empty(&keydb);
    output_init_empty(&secret);

    if (argc != 8 || strcmp(argv[1], "create") ||
        strcmp(argv[2], "--keydb") || strcmp(argv[4], "--secret") ||
        strcmp(argv[6], "--server-id")) {
        usage();
        goto out;
    }
    keydb_path = argv[3];
    secret_path = argv[5];
    server_id = argv[7];
    if (!keydb_path[0] || !secret_path[0] || !server_id[0] ||
        strnlen(server_id, PLAN9_AUTH_NAMELEN) >= PLAN9_AUTH_NAMELEN ||
        !strcmp(server_id, "tor")) {
        usage();
        goto out;
    }
    if (provision_signal_scope_enter(&signal_scope, &err) ||
        output_prepare(&keydb, keydb_path, &err) ||
        output_prepare(&secret, secret_path, &err)) {
        goto out;
    }
    if (outputs_equal(&keydb, &secret)) {
        error_setg(&err, "key database and Secret outputs must differ");
        goto out;
    }
    if (keydb.target_exists && !secret.target_exists) {
        error_setg(&err, "incomplete Plan 9 key database transaction: key "
                   "database exists but Secret is absent; verify that the "
                   "key database is safe to remove, remove it manually, "
                   "then rerun; no files were changed");
        goto out;
    }
    if (keydb.target_exists || secret.target_exists) {
        error_setg(&err, "output already exists: %s",
                   keydb.target_exists ? keydb.path : secret.path);
        goto out;
    }
    if (provision_check_interrupted(&err)) {
        goto out;
    }
    if (read_password_pair(password, &err) ||
        plan9_auth_passtokey(tor_key, password, &err)) {
        goto out;
    }
    plan9_auth_clear(password, sizeof(password));
    provision_test_pause("after-password");
    if (provision_check_interrupted(&err)) {
        goto out;
    }
    if (qcrypto_random_bytes(master_key, sizeof(master_key), &err) < 0 ||
        qcrypto_random_bytes(server_key, sizeof(server_key), &err) < 0) {
        goto out;
    }
    if (provision_check_interrupted(&err)) {
        goto out;
    }
    if (plan9_auth_keydb_record_encode(records, master_key, "tor", tor_key,
                                        0, 0, 0, &err) ||
        plan9_auth_keydb_record_encode(records + PLAN9_AUTH_KEYDB_RECORD_LEN,
                                        master_key, server_id, server_key,
                                        0, 0, 0, &err)) {
        goto out;
    }
    if (provision_check_interrupted(&err)) {
        goto out;
    }
    base64 = g_base64_encode(master_key, sizeof(master_key));
    if (!base64 || strlen(base64) != 12) {
        error_setg(&err, "cannot encode Plan 9 master key");
        goto out;
    }
    if (output_create_temp(&keydb, &err) ||
        output_create_temp(&secret, &err) ||
        output_write_close(&keydb, records, sizeof(records), &err) ||
        output_write_close(&secret, base64, 12, &err)) {
        goto out;
    }
    provision_test_pause("after-temp");
    if (output_publish(&keydb, &err)) {
        goto out;
    }
    provision_test_pause("between-publish");
    if (provision_check_interrupted(&err) ||
        output_publish(&secret, &err)) {
        goto out;
    }
    rollback = false;
    ret = EXIT_SUCCESS;

out:
    output_cleanup(&secret, rollback);
    output_cleanup(&keydb, rollback);
    plan9_auth_clear(password, sizeof(password));
    plan9_auth_clear(tor_key, sizeof(tor_key));
    plan9_auth_clear(server_key, sizeof(server_key));
    plan9_auth_clear(master_key, sizeof(master_key));
    plan9_auth_clear(records, sizeof(records));
    if (base64) {
        plan9_auth_clear(base64, strlen(base64));
        g_free(base64);
    }
    provision_signal_scope_leave(&signal_scope);
    if (err) {
        error_report_err(err);
    }
    if (interrupted_signal) {
        int sig = interrupted_signal;

        signal(sig, SIG_DFL);
        raise(sig);
    }
    return ret;
}
