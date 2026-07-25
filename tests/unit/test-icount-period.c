/*
 * Exact icount period tests
 *
 * Copyright 2026 Bryce Lanham
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "exec/icount.h"

static void test_period_range(void)
{
    g_assert_false(icount_period_valid(0));
    g_assert_true(icount_period_valid(1));
    g_assert_true(icount_period_valid(MAX_ICOUNT_PERIOD_NS));
    g_assert_false(icount_period_valid(MAX_ICOUNT_PERIOD_NS + 1ULL));
}

static void test_period_conversion(void)
{
    g_assert_cmpint(icount_period_to_ns(6984, 143), ==, 998712);
    g_assert_cmpint(icount_period_to_ns(6984, 128), ==, 893952);
}

static void test_period_rounding(void)
{
    const int64_t count = 6984;
    const int64_t exact = count * 143;

    g_assert_cmpint(icount_period_round(exact, 143), ==, count);
    g_assert_cmpint(icount_period_round(exact + 1, 143), ==, count + 1);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/icount/period/range", test_period_range);
    g_test_add_func("/icount/period/conversion", test_period_conversion);
    g_test_add_func("/icount/period/rounding", test_period_rounding);
    return g_test_run();
}
