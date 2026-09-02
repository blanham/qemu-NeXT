/*
 * Bounded JSON5 parser unit tests.
 *
 * Copyright (C) 2026, QEMU contributors
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"

#include <math.h>

#include "qapi/error.h"
#include "qobject/qbool.h"
#include "qobject/qdict.h"
#include "qobject/qjson5.h"
#include "qobject/qlist.h"
#include "qobject/qnum.h"
#include "qobject/qstring.h"

static QObject *parse_ok(const char *text, size_t length)
{
    Error *err = NULL;
    QObject *obj;

    obj = qobject_from_json5(text, length, &err);
    g_assert_null(err);
    g_assert_nonnull(obj);
    return obj;
}

static void parse_fail(const char *text, size_t length)
{
    Error *err = NULL;
    QObject *obj;

    obj = qobject_from_json5(text, length, &err);
    g_assert_null(obj);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_json5_literals(void)
{
    static const struct {
        const char *name;
        const char *text;
        QType type;
    } cases[] = {
        { "true", "true", QTYPE_QBOOL },
        { "false", "false", QTYPE_QBOOL },
        { "null", "null", QTYPE_QNULL },
        { "commented-object",
          "// leading\n{ /* middle */ answer: 42, // trailing\n}",
          QTYPE_QDICT },
        { "commented-array", "[1, /* two */ 2, // three\n 3,]", QTYPE_QLIST },
        { "line-comment-at-eof", "1 // comment without a final line ending",
          QTYPE_QNUM },
        { "nonascii-comments", "// caf\xC3\xA9\n/* \xCF\x80 */ 1", QTYPE_QNUM },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        g_test_message("literal JSON5 case: %s", cases[i].name);
        QObject *obj = parse_ok(cases[i].text, strlen(cases[i].text));

        g_assert_cmpint(qobject_type(obj), ==, cases[i].type);
        qobject_unref(obj);
    }
}

static void test_json5_strings(void)
{
    static const struct {
        const char *name;
        const char *text;
        const char *expected;
    } cases[] = {
        { "single", "'single quoted'", "single quoted" },
        { "double", "\"double quoted\"", "double quoted" },
        { "quotes", "'double \" and escaped \\' quote'",
          "double \" and escaped ' quote" },
        { "escapes", "'\\b\\f\\n\\r\\t\\v\\0\\\\\\/\\\'\\\"'",
          "\b\f\n\r\t\v\xC0\x80\\/'\"" },
        { "hex-escape", "'A\\x42\\x43'", "ABC" },
        { "unicode-escape", "'A\\u0042\\u20ac'", "AB\xE2\x82\xAC" },
        { "surrogate-pair", "'\\uD834\\uDD1E'", "\xF0\x9D\x84\x9E" },
        { "line-continuation", "'first\\\nsecond'", "firstsecond" },
        { "crlf-continuation", "'first\\\r\nsecond'", "firstsecond" },
        { "line-separator-continuation", "'first\\\xE2\x80\xA8"
          "second'", "firstsecond" },
        { "paragraph-separator-continuation", "'first\\\xE2\x80\xA9"
          "second'", "firstsecond" },
        { "raw-line-separator", "'a\xE2\x80\xA8" "b'",
          "a\xE2\x80\xA8" "b" },
        { "raw-paragraph-separator", "'a\xE2\x80\xA9" "b'",
          "a\xE2\x80\xA9" "b" },
        { "unescaped-double-in-single", "'a \" b'", "a \" b" },
        { "unescaped-single-in-double", "\"a ' b\"", "a ' b" },
        { "identity-escapes", "'\\A\\C\\/\\D\\C'", "AC/DC" },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        g_test_message("string JSON5 case: %s", cases[i].name);
        QObject *obj = parse_ok(cases[i].text, strlen(cases[i].text));
        QString *str = qobject_to(QString, obj);

        g_assert_nonnull(str);
        g_assert_cmpstr(qstring_get_str(str), ==, cases[i].expected);
        qobject_unref(obj);
    }
}

static void test_json5_numbers(void)
{
    static const struct {
        const char *name;
        const char *text;
        QNumKind kind;
        int64_t i64;
        uint64_t u64;
        double dbl;
    } cases[] = {
        { "zero", "0", QNUM_I64, 0, 0, 0.0 },
        { "signed-positive", "+42", QNUM_I64, 42, 0, 0.0 },
        { "signed-negative", "-42", QNUM_I64, -42, 0, 0.0 },
        { "hex", "0xffffffff", QNUM_I64, UINT32_MAX, 0, 0.0 },
        { "hex-upper", "+0XFFFFFFFF", QNUM_I64, UINT32_MAX, 0, 0.0 },
        { "negative-hex-min", "-0x8000000000000000", QNUM_I64,
          INT64_MIN, 0, 0.0 },
        { "int64-max", "9223372036854775807", QNUM_I64, INT64_MAX, 0, 0.0 },
        { "uint64-max", "18446744073709551615", QNUM_U64, 0, UINT64_MAX, 0.0 },
        { "zero-fraction", "0.5", QNUM_DOUBLE, 0, 0, 0.5 },
        { "zero-exponent", "0e1", QNUM_DOUBLE, 0, 0, 0.0 },
        { "signed-zero-fraction", "+0.5", QNUM_DOUBLE, 0, 0, 0.5 },
        { "signed-zero-exponent", "-0e1", QNUM_DOUBLE, 0, 0, 0.0 },
        { "leading-dot", ".5", QNUM_DOUBLE, 0, 0, 0.5 },
        { "trailing-dot", "1.", QNUM_DOUBLE, 0, 0, 1.0 },
        { "exponent", "1.25e+2", QNUM_DOUBLE, 0, 0, 125.0 },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        QObject *obj = parse_ok(cases[i].text, strlen(cases[i].text));
        QNum *num = qobject_to(QNum, obj);

        g_assert_nonnull(num);
        g_assert_cmpint(num->kind, ==, cases[i].kind);
        switch (num->kind) {
        case QNUM_I64:
            g_assert_cmpint(num->u.i64, ==, cases[i].i64);
            break;
        case QNUM_U64:
            g_assert_cmpuint(num->u.u64, ==, cases[i].u64);
            break;
        case QNUM_DOUBLE:
            g_assert_cmpfloat(num->u.dbl, ==, cases[i].dbl);
            break;
        }
        qobject_unref(obj);
    }
}

static void test_json5_nonfinite(void)
{
    static const struct {
        const char *name;
        const char *text;
        bool nan;
        double expected;
    } cases[] = {
        { "nan", "NaN", true, 0.0 },
        { "positive-nan", "+NaN", true, 0.0 },
        { "negative-nan", "-NaN", true, 0.0 },
        { "infinity", "Infinity", false, INFINITY },
        { "positive-infinity", "+Infinity", false, INFINITY },
        { "negative-infinity", "-Infinity", false, -INFINITY },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        QObject *obj = parse_ok(cases[i].text, strlen(cases[i].text));
        QNum *num = qobject_to(QNum, obj);

        g_assert_nonnull(num);
        g_assert_cmpint(num->kind, ==, QNUM_DOUBLE);
        if (cases[i].nan) {
            g_assert_true(isnan(num->u.dbl));
        } else {
            g_assert_cmpfloat(num->u.dbl, ==, cases[i].expected);
        }
        qobject_unref(obj);
    }
}

static void test_json5_collections(void)
{
    QObject *obj = parse_ok(
        "{unquoted: [1, {single: 'value',}], quoted: \"ok\",}",
        strlen("{unquoted: [1, {single: 'value',}], quoted: \"ok\",}"));
    QDict *dict = qobject_to(QDict, obj);
    QList *list;
    QDict *nested;

    g_assert_nonnull(dict);
    g_assert_cmpuint(qdict_size(dict), ==, 2);
    list = qobject_to(QList, qdict_get(dict, "unquoted"));
    g_assert_nonnull(list);
    g_assert_cmpuint(qlist_size(list), ==, 2);
    g_assert_cmpint(qnum_get_int(qobject_to(QNum, qlist_peek(list))), ==, 1);
    nested = qobject_to(QDict, qlist_entry_obj(qlist_next(qlist_first(list))));
    g_assert_nonnull(nested);
    g_assert_cmpstr(qdict_get_str(nested, "single"), ==, "value");
    g_assert_cmpstr(qdict_get_str(dict, "quoted"), ==, "ok");
    qobject_unref(obj);
}

static void test_json5_source_order(void)
{
    static const char *const root_keys[] = { "zeta", "alpha", "middle" };
    static const char *const nested_keys[] = { "two", "one" };
    QObject *obj = parse_ok(
        "{zeta: 1, alpha: {two: 2, one: 1}, middle: 3}",
        strlen("{zeta: 1, alpha: {two: 2, one: 1}, middle: 3}"));
    QDict *dict = qobject_to(QDict, obj);
    QDict *nested = qdict_get_qdict(dict, "alpha");
    const QDictEntry *entry;
    size_t i;

    for (entry = qdict_ordered_first(dict), i = 0;
         entry;
         entry = qdict_ordered_next(entry), i++) {
        g_assert_cmpuint(i, <, G_N_ELEMENTS(root_keys));
        g_assert_cmpstr(qdict_entry_key(entry), ==, root_keys[i]);
    }
    g_assert_cmpuint(i, ==, G_N_ELEMENTS(root_keys));

    for (entry = qdict_ordered_first(nested), i = 0;
         entry;
         entry = qdict_ordered_next(entry), i++) {
        g_assert_cmpuint(i, <, G_N_ELEMENTS(nested_keys));
        g_assert_cmpstr(qdict_entry_key(entry), ==, nested_keys[i]);
    }
    g_assert_cmpuint(i, ==, G_N_ELEMENTS(nested_keys));
    qobject_unref(obj);
}

static void test_json5_identifier_keys(void)
{
    QObject *obj = parse_ok(
        "{_leading: 1, dollar$9: 2, alphabet123: 3, true: 4, π: 5}",
        strlen("{_leading: 1, dollar$9: 2, alphabet123: 3, true: 4, π: 5}"));
    QDict *dict = qobject_to(QDict, obj);

    g_assert_cmpuint(qdict_size(dict), ==, 5);
    g_assert_cmpint(qdict_get_int(dict, "_leading"), ==, 1);
    g_assert_cmpint(qdict_get_int(dict, "dollar$9"), ==, 2);
    g_assert_cmpint(qdict_get_int(dict, "alphabet123"), ==, 3);
    g_assert_cmpint(qdict_get_int(dict, "true"), ==, 4);
    g_assert_cmpint(qdict_get_int(dict, "π"), ==, 5);
    qobject_unref(obj);
}

static void test_json5_escaped_identifier_keys(void)
{
    QObject *obj = parse_ok(
        "{\\u0061: 1, a\\u203Fb: 2, c\xE2\x80\xBF" "d: 3, "
        "e\\u200Cf\\u200Dg: 4, h\\uD801\\uDC00: 5, "
        "m\\u0301n: 6, o\xE0\xA4\xBE: 7, $value: 8}",
        strlen("{\\u0061: 1, a\\u203Fb: 2, c\xE2\x80\xBF"
                "d: 3, e\\u200Cf\\u200Dg: 4, h\\uD801\\uDC00: 5, "
                "m\\u0301n: 6, o\xE0\xA4\xBE: 7, $value: 8}"));
    QDict *dict = qobject_to(QDict, obj);

    g_assert_cmpuint(qdict_size(dict), ==, 8);
    g_assert_cmpint(qdict_get_int(dict, "a"), ==, 1);
    g_assert_cmpint(qdict_get_int(dict, "a\xE2\x80\xBF" "b"), ==, 2);
    g_assert_cmpint(qdict_get_int(dict, "c\xE2\x80\xBF" "d"), ==, 3);
    g_assert_cmpint(qdict_get_int(dict, "e\xE2\x80\x8C" "f\xE2\x80\x8D"
                                  "g"), ==, 4);
    g_assert_cmpint(qdict_get_int(dict, "h\xF0\x90\x90\x80"), ==, 5);
    g_assert_cmpint(qdict_get_int(dict, "m\xCC\x81n"), ==, 6);
    g_assert_cmpint(qdict_get_int(dict, "o\xE0\xA4\xBE"), ==, 7);
    g_assert_cmpint(qdict_get_int(dict, "$value"), ==, 8);
    qobject_unref(obj);
}

static void test_json5_identifier_mark_categories(void)
{
    static const struct {
        const char *name;
        const char *text;
    } cases[] = {
        { "raw-enclosing-mark", "{a\xE2\x83\x9D: 1}" },
        { "escaped-enclosing-mark", "{a\\u20DD: 1}" },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        g_test_message("invalid identifier mark case: %s", cases[i].name);
        parse_fail(cases[i].text, strlen(cases[i].text));
    }
}

static void test_json5_leading_zero_invalid(void)
{
    static const struct {
        const char *name;
        const char *text;
    } cases[] = {
        { "integer", "00" },
        { "integer-long", "0123" },
        { "fraction", "00.5" },
        { "fraction-trailing-dot", "00." },
        { "exponent", "00e1" },
        { "signed-positive", "+00" },
        { "signed-positive-long", "+0123" },
        { "signed-negative", "-00" },
        { "signed-negative-long", "-0123" },
        { "signed-fraction", "+00.5" },
        { "signed-exponent", "-00e1" },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        g_test_message("invalid leading-zero case: %s", cases[i].name);
        parse_fail(cases[i].text, strlen(cases[i].text));
    }
}

static void test_json5_invalid_table(void)
{
    static const struct {
        const char *name;
        const char *text;
    } cases[] = {
        { "duplicate-unquoted", "{a: 1, a: 2}" },
        { "duplicate-quoted-unquoted", "{'a': 1, a: 2}" },
        { "duplicate-escaped-unquoted", "{a: 1, \\u0061: 2}" },
        { "escaped-invalid-start", "{\\u0031: 1}" },
        { "escaped-invalid-number-category", "{\\u00B2: 1}" },
        { "escaped-invalid-continue", "{a\\u002d: 1}" },
        { "raw-invalid-enclosing-mark", "{a\xE2\x83\x9D: 1}" },
        { "escaped-invalid-enclosing-mark", "{a\\u20DD: 1}" },
        { "escaped-unpaired-surrogate", "{\\uD800: 1}" },
        { "escaped-malformed-unicode", "{\\u12G4: 1}" },
        { "leading-zero-integer", "00" },
        { "leading-zero-integer-long", "0123" },
        { "leading-zero-fraction", "00.5" },
        { "leading-zero-exponent", "00e1" },
        { "signed-leading-zero", "+00" },
        { "signed-leading-zero-long", "-0123" },
        { "signed-leading-zero-fraction", "+00.5" },
        { "signed-leading-zero-exponent", "-00e1" },
        { "integer-overflow", "18446744073709551616" },
        { "integer-underflow", "-9223372036854775809" },
        { "hex-overflow", "0x10000000000000000" },
        { "negative-hex-overflow", "-0x8000000000000001" },
        { "missing-exponent-digits", "1e+" },
        { "missing-hex-digits", "0x" },
        { "dot-without-digits", "." },
        { "plus-dot-without-digits", "+." },
        { "trailing-number", "1 2" },
        { "trailing-object", "{}{}" },
        { "trailing-identifier", "{a: 1} junk" },
        { "unterminated-string", "'unterminated" },
        { "unterminated-block-comment", "/" "* unterminated" },
        { "invalid-decimal-escape", "'\\1'" },
        { "invalid-decimal-escape-nine", "'\\9'" },
        { "invalid-hex-escape", "'\\x1'" },
        { "invalid-unicode-escape", "'\\u12G4'" },
        { "unpaired-leading-surrogate", "'\\uD800'" },
        { "unpaired-trailing-surrogate", "'\\uDC00'" },
        { "bad-surrogate-pair", "'\\uD800\\u0041'" },
        { "raw-overlong-utf8", "'\xC0\xAF'" },
        { "raw-surrogate-utf8", "'\xED\xA0\x80'" },
        { "raw-out-of-range-utf8", "'\xF4\x90\x80\x80'" },
        { "raw-modified-nul", "'\xC0\x80'" },
        { "raw-control", "'\n'" },
        { "line-comment-overlong-utf8", "// \xC0\xAF\n0" },
        { "line-comment-truncated-utf8", "// \xE2\x82\n0" },
        { "block-comment-surrogate-utf8", "/* \xED\xA0\x80 */ 0" },
        { "block-comment-out-of-range-utf8", "/* \xF4\x90\x80\x80 */ 0" },
        { "missing-colon", "{a 1}" },
        { "missing-value", "{a:}" },
        { "missing-separator", "[1 2]" },
        { "empty-array-item", "[,]" },
        { "empty-object-key", "{: 1}" },
        { "invalid-key-character", "{a-b: 1}" },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(cases); i++) {
        g_test_message("invalid JSON5 case: %s", cases[i].name);
        parse_fail(cases[i].text, strlen(cases[i].text));
    }
}

static void test_json5_nesting_limit(void)
{
    GString *text = g_string_sized_new(QJSON5_MAX_NESTING * 2 + 2);
    QObject *obj;

    for (size_t i = 0; i < QJSON5_MAX_NESTING; i++) {
        g_string_append_c(text, '[');
    }
    g_string_append_c(text, '0');
    for (size_t i = 0; i < QJSON5_MAX_NESTING; i++) {
        g_string_append_c(text, ']');
    }
    obj = parse_ok(text->str, text->len);
    qobject_unref(obj);
    g_string_free(text, true);

    text = g_string_sized_new((QJSON5_MAX_NESTING + 1) * 2 + 2);
    for (size_t i = 0; i < QJSON5_MAX_NESTING + 1; i++) {
        g_string_append_c(text, '[');
    }
    g_string_append_c(text, '0');
    for (size_t i = 0; i < QJSON5_MAX_NESTING + 1; i++) {
        g_string_append_c(text, ']');
    }
    parse_fail(text->str, text->len);
    g_string_free(text, true);
}

static void test_json5_input_limit(void)
{
    char *text = g_malloc(QJSON5_MAX_INPUT_SIZE);
    QObject *obj;

    memset(text, ' ', QJSON5_MAX_INPUT_SIZE);
    text[QJSON5_MAX_INPUT_SIZE - 1] = '0';
    obj = parse_ok(text, QJSON5_MAX_INPUT_SIZE);
    qobject_unref(obj);

    text = g_realloc(text, QJSON5_MAX_INPUT_SIZE + 1);
    text[QJSON5_MAX_INPUT_SIZE] = ' ';
    parse_fail(text, QJSON5_MAX_INPUT_SIZE + 1);
    g_free(text);
}

static void test_json5_token_limit(void)
{
    GString *text;
    QObject *obj;

    text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE);
    g_string_append_c(text, '\'');
    for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 2; i++) {
        g_string_append_c(text, 'x');
    }
    g_string_append_c(text, '\'');
    g_assert_cmpuint(text->len, ==, QJSON5_MAX_TOKEN_SIZE);
    obj = parse_ok(text->str, text->len);
    qobject_unref(obj);
    g_string_free(text, true);

    text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 1);
    g_string_append_c(text, '\'');
    for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 1; i++) {
        g_string_append_c(text, 'x');
    }
    g_string_append_c(text, '\'');
    g_assert_cmpuint(text->len, ==, QJSON5_MAX_TOKEN_SIZE + 1);
    parse_fail(text->str, text->len);
    g_string_free(text, true);

    text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 3);
    g_string_append(text, "//");
    for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 2; i++) {
        g_string_append_c(text, 'x');
    }
    g_string_append(text, "\n0");
    obj = parse_ok(text->str, text->len);
    qobject_unref(obj);
    g_string_free(text, true);

    text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 4);
    g_string_append(text, "//");
    for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 1; i++) {
        g_string_append_c(text, 'x');
    }
    g_string_append(text, "\n0");
    parse_fail(text->str, text->len);
    g_string_free(text, true);

    text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 4);
    g_string_append_c(text, '{');
    g_string_append_c(text, 'a');
    for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 1; i++) {
        g_string_append_c(text, 'x');
    }
    g_string_append(text, ":0}");
    obj = parse_ok(text->str, text->len);
    qobject_unref(obj);
    g_string_free(text, true);

    text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 5);
    g_string_append_c(text, '{');
    g_string_append_c(text, 'a');
    for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE; i++) {
        g_string_append_c(text, 'x');
    }
    g_string_append(text, ":0}");
    parse_fail(text->str, text->len);
    g_string_free(text, true);
}

static void test_json5_line_comment_terminator_limits(void)
{
    static const char *const terminators[] = {
        "\xE2\x80\xA8",
        "\xE2\x80\xA9",
    };

    for (size_t terminator = 0; terminator < G_N_ELEMENTS(terminators);
         terminator++) {
        GString *text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 8);
        QObject *obj;

        g_string_append(text, "//");
        for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 2; i++) {
            g_string_append_c(text, 'x');
        }
        g_string_append(text, terminators[terminator]);
        g_string_append_c(text, '0');
        g_assert_cmpuint(text->len, ==,
                         QJSON5_MAX_TOKEN_SIZE + strlen(terminators[terminator])
                         + 1);
        obj = parse_ok(text->str, text->len);
        qobject_unref(obj);
        g_string_free(text, true);

        text = g_string_sized_new(QJSON5_MAX_TOKEN_SIZE + 8);
        g_string_append(text, "//");
        for (size_t i = 0; i < QJSON5_MAX_TOKEN_SIZE - 1; i++) {
            g_string_append_c(text, 'x');
        }
        g_string_append(text, terminators[terminator]);
        g_string_append_c(text, '0');
        g_assert_cmpuint(text->len, ==,
                         QJSON5_MAX_TOKEN_SIZE + strlen(terminators[terminator])
                         + 2);
        parse_fail(text->str, text->len);
        g_string_free(text, true);
    }
}

static void test_json5_length_and_nul(void)
{
    static const char embedded_nul[] = { '1', '\0', '2' };
    static const char bounded_text[] = {
        '1', '\0', 'g', 'a', 'r', 'b', 'a', 'g', 'e'
    };
    QObject *obj;

    obj = parse_ok(bounded_text, 1);
    qobject_unref(obj);
    parse_fail(embedded_nul, sizeof(embedded_nul));
    parse_fail(embedded_nul, 2);
    parse_fail(NULL, 0);
    parse_fail(NULL, 1);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/qjson5/literals", test_json5_literals);
    g_test_add_func("/qjson5/strings", test_json5_strings);
    g_test_add_func("/qjson5/numbers", test_json5_numbers);
    g_test_add_func("/qjson5/nonfinite", test_json5_nonfinite);
    g_test_add_func("/qjson5/collections", test_json5_collections);
    g_test_add_func("/qjson5/source-order", test_json5_source_order);
    g_test_add_func("/qjson5/identifier-keys", test_json5_identifier_keys);
    g_test_add_func("/qjson5/escaped-identifier-keys",
                    test_json5_escaped_identifier_keys);
    g_test_add_func("/qjson5/identifier-mark-categories",
                    test_json5_identifier_mark_categories);
    g_test_add_func("/qjson5/numbers/leading-zero-invalid",
                    test_json5_leading_zero_invalid);
    g_test_add_func("/qjson5/invalid", test_json5_invalid_table);
    g_test_add_func("/qjson5/limits/nesting", test_json5_nesting_limit);
    g_test_add_func("/qjson5/limits/input", test_json5_input_limit);
    g_test_add_func("/qjson5/limits/token", test_json5_token_limit);
    g_test_add_func("/qjson5/limits/comment-terminators",
                    test_json5_line_comment_terminator_limits);
    g_test_add_func("/qjson5/length-and-nul", test_json5_length_and_nul);

    return g_test_run();
}
