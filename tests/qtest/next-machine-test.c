/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/units.h"
#include "libqtest.h"

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#endif

#define NEXT_SCR1             0x0200c000
#define NEXT_RAM_BASE         0x04000000
#define NEXT_ROM_SIZE         (128 * KiB)
#define NEXT_FB_RANGE         \
    "000000000b000000-000000000b1cb0ff"
#define TEST_TIMEOUT          (5 * G_USEC_PER_SEC)
#define NEXT_030_ROM_SHA256   \
    "bdccecc045c1af09d0962e02e30e737e8571a81ec6a4458be63d57189d79eb92"
#define NEXT_030_PASS_SHA256  \
    "9e3c391867f05f3e7a14263ec569c997cd5924cd1ab156f13c813b9e851db3b9"
#define NEXT_030_V41_DEADLINE_USEC (40 * G_USEC_PER_SEC)
#define NEXT_030_V41_POLL_USEC     (500 * G_TIME_SPAN_MILLISECOND)
#define NEXT_030_V41_SHUTDOWN_USEC (2 * G_USEC_PER_SEC)

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct ExpectedSCR1 {
    const char *machine;
    uint32_t value;
    uint8_t dma_revision;
    uint8_t machine_type;
    uint8_t board_revision;
    uint8_t cpu_clock;
} ExpectedSCR1;

typedef struct MachineTest {
    const char *machine;
    const char *product_name;
    uint64_t ram_size;
    bool has_mono_framebuffer;
} MachineTest;

typedef struct ByteDeviceMappingTest {
    const char *machine;
    const char * const *ranges;
} ByteDeviceMappingTest;

static const char * const next_computer_byte_device_ranges[] = {
    "0000000002006000-000000000200600f (prio 0, i/o): next.mb8795",
    "0000000002006010-0000000002006014 (prio 0, i/o): next.memctl",
    "0000000002008000-0000000002008007 (prio 0, i/o): next.dsp",
    "0000000002012000-000000000201201f (prio 0, i/o): next-optical",
    "0000000002014000-000000000201400f (prio 0, i/o): esp-regs",
    "0000000002014100-0000000002014107 (prio 0, i/o): fdc",
    "0000000002014108-0000000002014108 (prio 0, i/o): next-floppy-ctrl",
    "0000000002016000-0000000002016004 (prio 0, i/o): next.system-timer",
    "0000000002018000-0000000002018003 (prio 0, i/o): escc",
    "000000000201a000-000000000201a003 (prio 0, i/o): next.event-counter",
    NULL,
};

static const char * const next_cube_byte_device_ranges[] = {
    "0000000002106000-000000000210600f (prio 0, i/o): next.mb8795",
    "0000000002106010-0000000002106014 (prio 0, i/o): next.memctl",
    "0000000002108000-0000000002108007 (prio 0, i/o): next.dsp",
    "0000000002112000-000000000211201f (prio 0, i/o): next-optical",
    "0000000002114000-000000000211400f (prio 0, i/o): esp-regs",
    "0000000002114100-0000000002114107 (prio 0, i/o): fdc",
    "0000000002114108-0000000002114108 (prio 0, i/o): next-floppy-ctrl",
    "0000000002116000-0000000002116004 (prio 0, i/o): next.system-timer",
    "0000000002118000-0000000002118003 (prio 0, i/o): escc",
    "000000000211a000-000000000211a003 (prio 0, i/o): next.event-counter",
    NULL,
};

static const char * const next_station_byte_device_ranges[] = {
    "0000000002106000-000000000210600f (prio 0, i/o): next.mb8795",
    "0000000002106010-0000000002106014 (prio 0, i/o): next.memctl",
    "0000000002108000-0000000002108007 (prio 0, i/o): next.dsp",
    "0000000002110000-000000000211000f (prio -10000, i/o): empty-slot",
    "0000000002114000-000000000211400f (prio 0, i/o): esp-regs",
    "0000000002114100-0000000002114107 (prio 0, i/o): fdc",
    "0000000002114108-0000000002114108 (prio 0, i/o): next-floppy-ctrl",
    "0000000002116000-0000000002116004 (prio 0, i/o): next.system-timer",
    "0000000002118000-0000000002118003 (prio 0, i/o): escc",
    "000000000211a000-000000000211a003 (prio 0, i/o): next.event-counter",
    NULL,
};

static const char * const next_station_color_byte_device_ranges[] = {
    "0000000002106000-000000000210600f (prio 0, i/o): next.mb8795",
    "0000000002106010-0000000002106014 (prio 0, i/o): next.memctl",
    "0000000002108000-0000000002108007 (prio 0, i/o): next.dsp",
    "0000000002110000-000000000211000f (prio -10000, i/o): empty-slot",
    "0000000002114000-000000000211400f (prio 0, i/o): esp-regs",
    "0000000002114100-0000000002114107 (prio 0, i/o): fdc",
    "0000000002114108-0000000002114108 (prio 0, i/o): next-floppy-ctrl",
    "0000000002116000-0000000002116004 (prio 0, i/o): next.system-timer",
    "0000000002118000-0000000002118003 (prio 0, i/o): escc",
    "000000000211a000-000000000211a003 (prio 0, i/o): next.event-counter",
    NULL,
};

static void cleanup_test_rom(void *opaque)
{
    TestROM *rom = opaque;

    qtest_remove_abrt_handler(rom);
    if (rom->fd >= 0) {
        close(rom->fd);
    }
    if (rom->path) {
        g_unlink(rom->path);
        g_free(rom->path);
    }
    g_free(rom);
}

static TestROM *create_test_rom(void)
{
    TestROM *rom = g_new0(TestROM, 1);

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-machine-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    return rom;
}

static QTestState *next_machine_start_with_rom(
    TestROM *rom, const char *machine, const char *args)
{
    g_autofree char *quoted_rom_path = g_shell_quote(rom->path);

    return qtest_initf("-machine %s -bios %s %s",
                       machine, quoted_rom_path, args ?: "");
}

static QTestState *next_machine_start(const char *machine, const char *args)
{
    return next_machine_start_with_rom(create_test_rom(), machine, args);
}

static void test_machine_registration(void)
{
    create_test_rom();

    g_assert_true(qtest_has_machine("next-cube"));
    g_assert_true(qtest_has_machine("next-station"));
    g_assert_true(qtest_has_machine("next-station-color"));
    g_assert_true(qtest_has_machine("next-computer"));
}

static void test_scr1(gconstpointer opaque)
{
    const ExpectedSCR1 *expected = opaque;
    QTestState *qts = next_machine_start(expected->machine, NULL);
    uint32_t scr1 = qtest_readl(qts, NEXT_SCR1);

    g_assert_cmphex(scr1, ==, expected->value);
    g_assert_cmphex(extract32(scr1, 16, 8), ==, expected->dma_revision);
    g_assert_cmphex(extract32(scr1, 12, 4), ==, expected->machine_type);
    g_assert_cmphex(extract32(scr1, 8, 4), ==, expected->board_revision);
    g_assert_cmphex(extract32(scr1, 0, 2), ==, expected->cpu_clock);

    qtest_quit(qts);
}

static void test_ram_and_framebuffer(gconstpointer opaque)
{
    const MachineTest *test = opaque;
    g_autofree char *args =
        g_strdup_printf("-m %" PRIu64 "M", test->ram_size / MiB);
    QTestState *qts = next_machine_start(test->machine, args);
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");
    const uint64_t last_word =
        NEXT_RAM_BASE + test->ram_size - sizeof(uint32_t);

    qtest_writel(qts, NEXT_RAM_BASE, 0x01234567);
    qtest_writel(qts, last_word, 0x89abcdef);
    g_assert_cmphex(qtest_readl(qts, NEXT_RAM_BASE), ==, 0x01234567);
    g_assert_cmphex(qtest_readl(qts, last_word), ==, 0x89abcdef);
    if (test->has_mono_framebuffer) {
        g_assert_nonnull(strstr(flatview, NEXT_FB_RANGE));
    } else {
        g_assert_null(strstr(flatview, NEXT_FB_RANGE));
    }

    qtest_quit(qts);
}

static void test_byte_device_mapping(gconstpointer opaque)
{
    const ByteDeviceMappingTest *test = opaque;
    g_autofree char *mtree = NULL;
    QTestState *qts = next_machine_start(test->machine, NULL);
    size_t i;

    mtree = qtest_hmp(qts, "info mtree -f");
    for (i = 0; test->ranges[i]; i++) {
        g_assert_nonnull(strstr(mtree, test->ranges[i]));
    }

    qtest_quit(qts);
}

static void test_ram_rejection(gconstpointer opaque)
{
    const MachineTest *test = opaque;
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        g_autofree char *args =
            g_strdup_printf("-m %" PRIu64 "M",
                            test->ram_size / MiB + 1);
        QTestState *qts =
            next_machine_start_with_rom(rom, test->machine, args);

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    {
        g_autofree char *pattern =
            g_strdup_printf("*%s supports at most %" PRIu64
                            " MiB of RAM*",
                            test->product_name, test->ram_size / MiB);

        g_test_trap_assert_stderr(pattern);
    }
}

static void test_invalid_cpu(void)
{
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        QTestState *qts =
            next_machine_start_with_rom(rom, "next-cube", "-cpu m68030");

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    g_test_trap_assert_stderr("*Invalid CPU model*");
}

static void test_valid_cpu(void)
{
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        QTestState *qts =
            next_machine_start_with_rom(rom, "next-cube", "-cpu m68040");

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_passed();
}

static void test_default_cpu(void)
{
    QTestState *qts = next_machine_start("next-computer", NULL);
    g_autofree char *cpus = qtest_hmp(qts, "info cpus");

    g_assert_nonnull(strstr(cpus, "model=m68030"));
    qtest_quit(qts);
}

static void test_computer_invalid_cpu(void)
{
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        QTestState *qts =
            next_machine_start_with_rom(rom, "next-computer", "-cpu m68040");

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    g_test_trap_assert_stderr("*Invalid CPU model*");
}

static void test_missing_firmware(void)
{
    create_test_rom();

    if (g_test_subprocess()) {
        const char *qemu_binary = g_getenv("QTEST_QEMU_BINARY");
        const char *argv[] = {
            qemu_binary,
            "-machine", "next-cube",
            "-display", "none",
            "-audio", "none",
            "-bios", "/definitely/missing/next.rom",
            NULL
        };
        g_autofree char *stderr_data = NULL;
        int wait_status = 0;

        g_assert_nonnull(qemu_binary);
        g_assert_true(g_spawn_sync(NULL, (char **)argv, NULL,
                                  G_SPAWN_STDOUT_TO_DEV_NULL,
                                  NULL, NULL, NULL, &stderr_data,
                                  &wait_status, NULL));
        g_assert_cmpint(wait_status, !=, 0);
        g_assert_nonnull(strstr(stderr_data, "Could not load ROM image"));
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_passed();
}

#ifndef _WIN32
typedef struct Next030V41Child {
    GPid pid;
    int monitor_fd;
    int wait_status;
    int wait_errno;
    bool reaped;
    bool wait_failed;
    bool pid_closed;
} Next030V41Child;

static void next_030_v41_close_monitor(Next030V41Child *child)
{
    if (child->monitor_fd >= 0) {
        close(child->monitor_fd);
        child->monitor_fd = -1;
    }
}

static void next_030_v41_mark_reaped(Next030V41Child *child)
{
    child->reaped = true;
    if (!child->pid_closed) {
        g_spawn_close_pid(child->pid);
        child->pid_closed = true;
    }
}

static bool next_030_v41_write_monitor(int fd, const char *command)
{
    const char *data = command;
    size_t remaining = strlen(command);

    while (remaining) {
        ssize_t written;

#ifdef MSG_NOSIGNAL
        written = send(fd, data, remaining, MSG_NOSIGNAL);
#else
        written = write(fd, data, remaining);
#endif
        if (written > 0) {
            data += written;
            remaining -= written;
        } else if (written < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

static bool next_030_v41_screen_matches(const char *screen_path)
{
    struct stat before;
    struct stat after;
    g_autofree char *screen_data = NULL;
    g_autofree char *screen_hash = NULL;
    gsize screen_size;

    if (stat(screen_path, &before) < 0 || !S_ISREG(before.st_mode) ||
        before.st_size <= 0) {
        return false;
    }
    if (!g_file_get_contents(screen_path, &screen_data, &screen_size, NULL)) {
        return false;
    }
    if (stat(screen_path, &after) < 0 ||
        before.st_size != after.st_size ||
        before.st_mtime != after.st_mtime ||
        (off_t)screen_size != before.st_size) {
        return false;
    }

    screen_hash = g_compute_checksum_for_data(
        G_CHECKSUM_SHA256, (const guchar *)screen_data, screen_size);
    return g_str_equal(screen_hash, NEXT_030_PASS_SHA256);
}

static bool next_030_v41_poll_child(Next030V41Child *child)
{
    pid_t result;

    do {
        result = waitpid(child->pid, &child->wait_status, WNOHANG);
    } while (result < 0 && errno == EINTR);

    if (result == child->pid) {
        next_030_v41_mark_reaped(child);
        return true;
    }
    if (result < 0) {
        child->wait_errno = errno;
        child->wait_failed = true;
        g_test_message("waitpid(%" G_PID_FORMAT "): %s", child->pid,
                       g_strerror(child->wait_errno));
        return true;
    }
    return false;
}

static bool next_030_v41_wait_child(Next030V41Child *child,
                                    gint64 timeout_usec)
{
    const gint64 deadline = g_get_monotonic_time() + timeout_usec;

    while (!child->reaped && !child->wait_failed) {
        gint64 remaining;

        if (next_030_v41_poll_child(child)) {
            break;
        }
        remaining = deadline - g_get_monotonic_time();
        if (remaining <= 0) {
            break;
        }
        g_usleep(MIN((gint64)NEXT_030_V41_POLL_USEC, remaining));
    }
    return child->reaped;
}

static bool next_030_v41_stop_child(Next030V41Child *child)
{
    pid_t result;

    if (child->pid <= 0) {
        return true;
    }
    if (!child->reaped && !child->wait_failed) {
        next_030_v41_wait_child(child, NEXT_030_V41_SHUTDOWN_USEC);
    }
    if (!child->reaped) {
        if (kill(child->pid, SIGTERM) < 0 && errno != ESRCH) {
            g_test_message("could not terminate v41 QEMU (pid %" G_PID_FORMAT
                           "): %s", child->pid, g_strerror(errno));
        }
        if (!child->wait_failed) {
            next_030_v41_wait_child(child, NEXT_030_V41_SHUTDOWN_USEC);
        }
    }
    if (!child->reaped) {
        if (kill(child->pid, SIGKILL) < 0 && errno != ESRCH) {
            g_test_message("could not kill v41 QEMU (pid %" G_PID_FORMAT
                           "): %s", child->pid, g_strerror(errno));
        }
        do {
            result = waitpid(child->pid, &child->wait_status, 0);
        } while (result < 0 && errno == EINTR);
        if (result == child->pid) {
            next_030_v41_mark_reaped(child);
        } else if (result < 0) {
            child->wait_errno = errno;
            child->wait_failed = true;
            g_test_message("waitpid(%" G_PID_FORMAT ") after kill: %s",
                           child->pid, g_strerror(child->wait_errno));
        }
    }
    if (!child->pid_closed) {
        g_spawn_close_pid(child->pid);
        child->pid_closed = true;
    }
    return child->reaped;
}

static bool next_030_v41_child_exited_ok(const Next030V41Child *child)
{
    if (!child->reaped) {
        return false;
    }
    if (WIFEXITED(child->wait_status)) {
        if (WEXITSTATUS(child->wait_status) == 0) {
            return true;
        }
        g_test_message("v41 QEMU exited with status %d",
                       WEXITSTATUS(child->wait_status));
    } else if (WIFSIGNALED(child->wait_status)) {
        g_test_message("v41 QEMU terminated by signal %d",
                       WTERMSIG(child->wait_status));
    } else {
        g_test_message("v41 QEMU returned unexpected wait status 0x%x",
                       child->wait_status);
    }
    return false;
}

static bool next_030_v41_spawn_child(const char *qemu_binary,
                                     const char *rom_path,
                                     Next030V41Child *child)
{
    int monitor_pair[2] = { -1, -1 };
    int dev_null = -1;
    GError *error = NULL;
    char *argv[] = {
        (char *)qemu_binary,
        (char *)"-M", (char *)"next-computer",
        (char *)"-cpu", (char *)"m68030",
        (char *)"-m", (char *)"64M",
        (char *)"-bios", (char *)rom_path,
        (char *)"-display", (char *)"none",
        (char *)"-audio", (char *)"none",
        (char *)"-no-reboot",
        (char *)"-monitor", (char *)"stdio",
        (char *)"-icount", (char *)"shift=8,align=on,sleep=on",
        NULL,
    };

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, monitor_pair) < 0) {
        g_test_message("socketpair for v41 QEMU monitor failed: %s",
                       g_strerror(errno));
        return false;
    }
#ifdef SO_NOSIGPIPE
    {
        int no_sigpipe = 1;

        setsockopt(monitor_pair[1], SOL_SOCKET, SO_NOSIGPIPE,
                   &no_sigpipe, sizeof(no_sigpipe));
    }
#endif
    dev_null = open("/dev/null", O_WRONLY);
    if (dev_null < 0) {
        g_test_message("open /dev/null for v41 QEMU failed: %s",
                       g_strerror(errno));
        goto fail;
    }

    if (!g_spawn_async_with_fds(NULL, argv, NULL,
                                G_SPAWN_DO_NOT_REAP_CHILD,
                                NULL, NULL, &child->pid,
                                monitor_pair[0], dev_null, dev_null,
                                &error)) {
        g_test_message("could not spawn v41 QEMU: %s", error->message);
        g_clear_error(&error);
        goto fail;
    }

    close(monitor_pair[0]);
    close(dev_null);
    child->monitor_fd = monitor_pair[1];
    monitor_pair[0] = -1;
    monitor_pair[1] = -1;
    dev_null = -1;
    return true;

fail:
    if (monitor_pair[0] >= 0) {
        close(monitor_pair[0]);
    }
    if (monitor_pair[1] >= 0) {
        close(monitor_pair[1]);
    }
    if (dev_null >= 0) {
        close(dev_null);
    }
    return false;
}

static bool next_030_v41_run(const char *qemu_binary, const char *rom_path,
                             const char *screen_path)
{
    g_autofree char *screendump_command =
        g_strdup_printf("screendump %s\n", screen_path);
    Next030V41Child child = {
        .pid = -1,
        .monitor_fd = -1,
    };
    const gint64 deadline =
        g_get_monotonic_time() + NEXT_030_V41_DEADLINE_USEC;
    gint64 next_screendump = 0;
    bool screen_matches = false;

    if (!next_030_v41_spawn_child(qemu_binary, rom_path, &child)) {
        return false;
    }

    while (g_get_monotonic_time() < deadline) {
        const gint64 now = g_get_monotonic_time();
        gint64 remaining;

        if (next_030_v41_screen_matches(screen_path)) {
            screen_matches = true;
            break;
        }
        if (next_030_v41_poll_child(&child)) {
            if (child.reaped) {
                g_test_message("v41 QEMU exited before the expected screen "
                               "hash appeared");
            }
            break;
        }
        if (now >= next_screendump) {
            if (!next_030_v41_write_monitor(child.monitor_fd,
                                             screendump_command)) {
                g_test_message("could not send v41 screendump command");
                break;
            }
            next_screendump = now + NEXT_030_V41_POLL_USEC;
        }
        remaining = deadline - g_get_monotonic_time();
        if (remaining > 0) {
            g_usleep(MIN((gint64)NEXT_030_V41_POLL_USEC, remaining));
        }
    }

    if (!screen_matches && g_get_monotonic_time() >= deadline) {
        g_test_message("v41 screen hash deadline expired after %d seconds",
                       NEXT_030_V41_DEADLINE_USEC / G_USEC_PER_SEC);
    }

    if (child.monitor_fd >= 0) {
        if (!next_030_v41_write_monitor(child.monitor_fd, "quit\n") &&
            screen_matches) {
            g_test_message("could not send v41 QEMU quit command");
            screen_matches = false;
        }
        next_030_v41_close_monitor(&child);
    }
    next_030_v41_stop_child(&child);
    return screen_matches && next_030_v41_child_exited_ok(&child);
}
#endif

static void test_next_computer_v41_rom(void)
{
#ifdef _WIN32
    g_test_skip("v41 firmware smoke test requires POSIX process APIs");
#else
    const char *rom_path = g_getenv("QTEST_NEXT_030_ROM");
    const char *qemu_binary = g_getenv("QTEST_QEMU_BINARY");
    g_autofree char *rom_data = NULL;
    g_autofree char *rom_hash = NULL;
    TestROM *screen = NULL;
    gsize rom_size;
    bool passed;

    if (!rom_path || !rom_path[0]) {
        g_test_skip("set QTEST_NEXT_030_ROM to run the Rev 1.0 v41 smoke test");
        return;
    }

    g_assert_true(g_file_get_contents(rom_path, &rom_data, &rom_size, NULL));
    rom_hash = g_compute_checksum_for_data(G_CHECKSUM_SHA256,
                                            (const guchar *)rom_data,
                                            rom_size);
    g_assert_cmpstr(rom_hash, ==, NEXT_030_ROM_SHA256);
    g_assert_nonnull(qemu_binary);

    screen = g_new0(TestROM, 1);
    screen->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, screen);
    g_test_queue_destroy(cleanup_test_rom, screen);
    screen->fd = g_file_open_tmp("next-v41-XXXXXX", &screen->path, NULL);
    g_assert_cmpint(screen->fd, >=, 0);
    close(screen->fd);
    screen->fd = -1;
    g_assert_cmpint(g_unlink(screen->path), ==, 0);

    passed = next_030_v41_run(qemu_binary, rom_path, screen->path);
    if (g_unlink(screen->path) < 0 && errno != ENOENT) {
        g_test_message("could not remove v41 screenshot %s: %s",
                       screen->path, g_strerror(errno));
        passed = false;
    }
    g_assert_true(passed);
#endif
}

int main(int argc, char **argv)
{
    static ExpectedSCR1 scr1_tests[] = {
        {
            .machine = "next-cube",
            .value = 0x00012002,
            .dma_revision = 1,
            .machine_type = 2,
            .board_revision = 0,
            .cpu_clock = 2,
        }, {
            .machine = "next-station",
            .value = 0x00011002,
            .dma_revision = 1,
            .machine_type = 1,
            .board_revision = 0,
            .cpu_clock = 2,
        }, {
            .machine = "next-station-color",
            .value = 0x00013002,
            .dma_revision = 1,
            .machine_type = 3,
            .board_revision = 0,
            .cpu_clock = 2,
        }, {
            .machine = "next-computer",
            .value = 0x00010102,
            .dma_revision = 1,
            .machine_type = 0,
            .board_revision = 1,
            .cpu_clock = 2,
        },
    };
    static MachineTest machine_tests[] = {
        {
            .machine = "next-cube",
            .product_name = "NeXTcube (68040, X15)",
            .ram_size = 64 * MiB,
            .has_mono_framebuffer = true,
        }, {
            .machine = "next-station",
            .product_name = "NeXTstation (Warp 9)",
            .ram_size = 64 * MiB,
            .has_mono_framebuffer = true,
        }, {
            .machine = "next-station-color",
            .product_name = "NeXTstation Color (Warp 9C)",
            .ram_size = 32 * MiB,
            .has_mono_framebuffer = false,
        }, {
            .machine = "next-computer",
            .product_name = "NeXT Computer (68030)",
            .ram_size = 64 * MiB,
            .has_mono_framebuffer = true,
        },
    };
    static ByteDeviceMappingTest byte_device_mapping_tests[] = {
        {
            .machine = "next-computer",
            .ranges = next_computer_byte_device_ranges,
        }, {
            .machine = "next-cube",
            .ranges = next_cube_byte_device_ranges,
        }, {
            .machine = "next-station",
            .ranges = next_station_byte_device_ranges,
        }, {
            .machine = "next-station-color",
            .ranges = next_station_color_byte_device_ranges,
        },
    };
    size_t i;

    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-machine/registration", test_machine_registration);
    for (i = 0; i < ARRAY_SIZE(scr1_tests); i++) {
        g_autofree char *path =
            g_strdup_printf("/next-machine/%s/scr1", scr1_tests[i].machine);

        qtest_add_data_func_full(path, &scr1_tests[i], test_scr1, NULL);
    }
    for (i = 0; i < ARRAY_SIZE(machine_tests); i++) {
        g_autofree char *ram_path =
            g_strdup_printf("/next-machine/%s/ram-and-framebuffer",
                            machine_tests[i].machine);
        g_autofree char *reject_path =
            g_strdup_printf("/next-machine/%s/reject-%" PRIu64 "m",
                            machine_tests[i].machine,
                            machine_tests[i].ram_size / MiB + 1);

        qtest_add_data_func_full(ram_path, &machine_tests[i],
                                 test_ram_and_framebuffer, NULL);
        qtest_add_data_func_full(reject_path, &machine_tests[i],
                                 test_ram_rejection, NULL);
    }
    for (i = 0; i < ARRAY_SIZE(byte_device_mapping_tests); i++) {
        g_autofree char *path =
            g_strdup_printf("/next-machine/%s/byte-device-mapping",
                            byte_device_mapping_tests[i].machine);

        qtest_add_data_func_full(path, &byte_device_mapping_tests[i],
                                 test_byte_device_mapping, NULL);
    }
    qtest_add_func("/next-machine/next-cube/reject-m68030",
                   test_invalid_cpu);
    qtest_add_func("/next-machine/next-cube/accept-m68040", test_valid_cpu);
    qtest_add_func("/next-machine/next-computer/default-m68030",
                   test_default_cpu);
    qtest_add_func("/next-machine/next-computer/reject-m68040",
                   test_computer_invalid_cpu);
    qtest_add_func("/next-machine/next-cube/missing-firmware",
                   test_missing_firmware);
    qtest_add_func("/next-machine/next-computer/v41-rom",
                   test_next_computer_v41_rom);

    return g_test_run();
}
