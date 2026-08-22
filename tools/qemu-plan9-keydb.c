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
    bool temp_exists;
    bool published;
} ProvisionOutput;

static volatile sig_atomic_t interrupted_signal;

static void provision_signal_handler(int sig)
{
    interrupted_signal = sig;
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
                         O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (output->dirfd < 0) {
        error_setg_errno(errp, errno, "cannot open output directory");
        return -1;
    }
    if (fstatat(output->dirfd, output->base, &target,
                AT_SYMLINK_NOFOLLOW) == 0) {
        error_setg(errp, "output already exists: %s", path);
        return -1;
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
            if (fstat(output->fd, &output->inode) < 0) {
                error_setg_errno(errp, errno,
                                 "cannot inspect temporary output");
                goto fail;
            }
            output->temp_exists = true;
            if (fchmod(output->fd, 0600) < 0) {
                error_setg_errno(errp, errno,
                                 "cannot secure temporary output");
                goto fail;
            }
            plan9_auth_clear(random, sizeof(random));
            plan9_auth_clear(suffix, sizeof(suffix));
            return 0;
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
    if (qemu_write_full(output->fd, data, len) != len) {
        error_setg_errno(errp, errno, "cannot write temporary output");
        return -1;
    }
    if (fsync(output->fd) < 0) {
        error_setg_errno(errp, errno, "cannot sync temporary output");
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
#endif
    if (linkat(output->dirfd, output->temp, output->dirfd, output->base,
               0) < 0) {
        error_setg_errno(errp, errno, "cannot publish output without replace");
        return -1;
    }
    output->published = true;
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
    return 0;
}

static void output_unlink_verified(ProvisionOutput *output, bool published)
{
    const char *name = published ? output->base : output->temp;
    struct stat current;

    if (!name || fstatat(output->dirfd, name, &current,
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
    if (output->fd >= 0) {
        close(output->fd);
        output->fd = -1;
    }
    if (output->temp_exists) {
        output_unlink_verified(output, false);
        output->temp_exists = false;
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

static int write_prompt(const char *prompt, Error **errp)
{
    size_t len = strlen(prompt);

    if (qemu_write_full(STDERR_FILENO, prompt, len) != len) {
        error_setg_errno(errp, errno, "cannot write password prompt");
        return -1;
    }
    return 0;
}

static int read_password_line(char password[PLAN9_AUTH_NAMELEN + 1],
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
        got = read(STDIN_FILENO, &byte, 1);
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
    static const int signals[] = { SIGINT, SIGTERM, SIGHUP, SIGQUIT };
    struct sigaction old[G_N_ELEMENTS(signals)];
    struct sigaction action = { 0 };
    struct termios saved, hidden;
    char confirmation[PLAN9_AUTH_NAMELEN + 1] = { 0 };
    size_t installed = 0;
    bool changed = false;
    int ret = -1;

    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved) < 0) {
        error_setg(errp, "password input requires a terminal");
        goto out;
    }
    action.sa_handler = provision_signal_handler;
    sigemptyset(&action.sa_mask);
    for (; installed < G_N_ELEMENTS(signals); installed++) {
        if (sigaction(signals[installed], &action, &old[installed]) < 0) {
            error_setg_errno(errp, errno, "cannot protect terminal state");
            goto out;
        }
    }
    hidden = saved;
    hidden.c_lflag &= ~(ECHO | ECHONL);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &hidden) < 0) {
        error_setg_errno(errp, errno, "cannot disable terminal echo");
        goto out;
    }
    changed = true;
    if (write_prompt("tor password: ", errp) ||
        read_password_line(password, errp) ||
        write_prompt("\nconfirm tor password: ", errp) ||
        read_password_line(confirmation, errp) ||
        write_prompt("\n", errp)) {
        goto out;
    }
    if (strcmp(password, confirmation)) {
        error_setg(errp, "password confirmation does not match");
        goto out;
    }
    ret = 0;

out:
    if (changed && tcsetattr(STDIN_FILENO, TCSANOW, &saved) < 0 && !*errp) {
        error_setg_errno(errp, errno, "cannot restore terminal state");
        ret = -1;
    }
    while (installed) {
        installed--;
        sigaction(signals[installed], &old[installed], NULL);
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
    if (output_prepare(&keydb, keydb_path, &err) ||
        output_prepare(&secret, secret_path, &err)) {
        goto out;
    }
    if (outputs_equal(&keydb, &secret)) {
        error_setg(&err, "key database and Secret outputs must differ");
        goto out;
    }
    if (read_password_pair(password, &err) ||
        plan9_auth_passtokey(tor_key, password, &err)) {
        goto out;
    }
    plan9_auth_clear(password, sizeof(password));
    if (qcrypto_random_bytes(master_key, sizeof(master_key), &err) < 0 ||
        qcrypto_random_bytes(server_key, sizeof(server_key), &err) < 0) {
        goto out;
    }
    if (plan9_auth_keydb_record_encode(records, master_key, "tor", tor_key,
                                        0, 0, 0, &err) ||
        plan9_auth_keydb_record_encode(records + PLAN9_AUTH_KEYDB_RECORD_LEN,
                                        master_key, server_id, server_key,
                                        0, 0, 0, &err)) {
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
        output_write_close(&secret, base64, 12, &err) ||
        output_publish(&keydb, &err) ||
        output_publish(&secret, &err)) {
        goto out;
    }
    rollback = false;
    ret = EXIT_SUCCESS;

out:
    if (err) {
        error_report_err(err);
    }
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
    if (interrupted_signal) {
        int sig = interrupted_signal;

        signal(sig, SIG_DFL);
        raise(sig);
    }
    return ret;
}
