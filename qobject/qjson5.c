/*
 * Bounded JSON5 QObject parser.
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
#include "qemu/cutils.h"
#include "qemu/unicode.h"
#include "qobject/qbool.h"
#include "qobject/qdict.h"
#include "qobject/qjson5.h"
#include "qobject/qlist.h"
#include "qobject/qnull.h"
#include "qobject/qnum.h"
#include "qobject/qstring.h"

typedef struct QJson5Parser {
    const char *text;
    size_t length;
    size_t pos;
    size_t depth;
    Error *err;
} QJson5Parser;

static void qjson5_error(QJson5Parser *parser, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

static void qjson5_error(QJson5Parser *parser, const char *fmt, ...)
{
    va_list ap;
    g_autofree char *message = NULL;

    if (parser->err) {
        return;
    }

    va_start(ap, fmt);
    message = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    error_setg(&parser->err, "%s at byte %zu", message, parser->pos);
}

static bool qjson5_token_too_long(QJson5Parser *parser, size_t start)
{
    if (parser->pos - start > QJSON5_MAX_TOKEN_SIZE) {
        qjson5_error(parser, "JSON5 token exceeds %u bytes",
                     QJSON5_MAX_TOKEN_SIZE);
        return true;
    }
    return false;
}

static bool qjson5_is_ascii_ident_start(unsigned char c)
{
    return (c == '_' || c == '$' ||
            (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'));
}

static bool qjson5_decode_utf8_at(const QJson5Parser *parser, size_t pos,
                                  int *codepoint, size_t *width)
{
    char *end;
    int cp;

    if (pos >= parser->length) {
        return false;
    }
    if ((unsigned char)parser->text[pos] < 0x80) {
        if (parser->text[pos] == 0) {
            return false;
        }
        *codepoint = (unsigned char)parser->text[pos];
        *width = 1;
        return true;
    }

    cp = mod_utf8_codepoint(parser->text + pos,
                            MIN(parser->length - pos, (size_t)6), &end);
    if (cp < 0 || cp == 0) {
        return false;
    }
    *codepoint = cp;
    *width = end - (parser->text + pos);
    return true;
}

static bool qjson5_identifier_start_codepoint(int codepoint)
{
    GUnicodeType type;

    if (codepoint == '$' || codepoint == '_') {
        return true;
    }
    type = g_unichar_type(codepoint);
    if (g_unichar_isalpha(codepoint) || type == G_UNICODE_LETTER_NUMBER) {
        return true;
    }
    switch (codepoint) {
    case 0x1885:
    case 0x1886:
    case 0x2118:
    case 0x212e:
    case 0x309b:
    case 0x309c:
        return true;
    default:
        return false;
    }
}

static bool qjson5_identifier_continue_codepoint(int codepoint)
{
    GUnicodeType type;

    if (qjson5_identifier_start_codepoint(codepoint) ||
        codepoint == 0x200c || codepoint == 0x200d) {
        return true;
    }
    type = g_unichar_type(codepoint);
    if (type == G_UNICODE_NON_SPACING_MARK ||
        type == G_UNICODE_SPACING_MARK ||
        type == G_UNICODE_DECIMAL_NUMBER ||
        type == G_UNICODE_CONNECT_PUNCTUATION) {
        return true;
    }
    switch (codepoint) {
    case 0x00b7:
    case 0x0387:
    case 0x1369:
    case 0x136a:
    case 0x136b:
    case 0x136c:
    case 0x136d:
    case 0x136e:
    case 0x136f:
    case 0x1370:
    case 0x1371:
    case 0x19da:
        return true;
    default:
        return false;
    }
}

static bool qjson5_identifier_codepoint_at(const QJson5Parser *parser,
                                           size_t pos, bool start,
                                           size_t *width)
{
    int codepoint;

    if (!qjson5_decode_utf8_at(parser, pos, &codepoint, width)) {
        return false;
    }
    return start ? qjson5_identifier_start_codepoint(codepoint) :
                   qjson5_identifier_continue_codepoint(codepoint);
}

static bool qjson5_is_unicode_space(int codepoint)
{
    switch (codepoint) {
    case 0x00a0: /* NO-BREAK SPACE */
    case 0x1680: /* OGHAM SPACE MARK */
    case 0x2028: /* LINE SEPARATOR */
    case 0x2029: /* PARAGRAPH SEPARATOR */
    case 0x202f: /* NARROW NO-BREAK SPACE */
    case 0x205f: /* MEDIUM MATHEMATICAL SPACE */
    case 0x3000: /* IDEOGRAPHIC SPACE */
    case 0xfeff: /* ZERO WIDTH NO-BREAK SPACE */
        return true;
    default:
        return codepoint >= 0x2000 && codepoint <= 0x200a;
    }
}

static bool qjson5_skip_space(QJson5Parser *parser)
{
    while (parser->pos < parser->length) {
        unsigned char c = parser->text[parser->pos];

        switch (c) {
        case ' ':
        case '\t':
        case '\n':
        case '\r':
        case '\v':
        case '\f':
            parser->pos++;
            continue;
        case 0:
            qjson5_error(parser, "embedded NUL is not valid JSON5 input");
            return false;
        case '/':
            if (parser->pos + 1 >= parser->length) {
                return true;
            }
            if (parser->text[parser->pos + 1] == '/') {
                size_t comment_start = parser->pos;

                parser->pos += 2;
                if (qjson5_token_too_long(parser, comment_start)) {
                    return false;
                }
                while (parser->pos < parser->length) {
                    int codepoint;
                    size_t width;

                    if (parser->text[parser->pos] == 0) {
                        qjson5_error(parser,
                                     "embedded NUL is not valid JSON5 input");
                        return false;
                    }
                    if (parser->text[parser->pos] == '\n' ||
                        parser->text[parser->pos] == '\r') {
                        break;
                    }
                    if ((unsigned char)parser->text[parser->pos] < 0x80) {
                        parser->pos++;
                        if (qjson5_token_too_long(parser, comment_start)) {
                            return false;
                        }
                        continue;
                    }
                    if (!qjson5_decode_utf8_at(parser, parser->pos,
                                               &codepoint, &width)) {
                        qjson5_error(parser,
                                     "invalid UTF-8 sequence in comment");
                        return false;
                    }
                    parser->pos += width;
                    if (codepoint == 0x2028 || codepoint == 0x2029) {
                        break;
                    }
                    if (qjson5_token_too_long(parser, comment_start)) {
                        return false;
                    }
                }
                continue;
            }
            if (parser->text[parser->pos + 1] == '*') {
                bool closed = false;
                size_t comment_start = parser->pos;

                parser->pos += 2;
                if (qjson5_token_too_long(parser, comment_start)) {
                    return false;
                }
                while (parser->pos < parser->length) {
                    int codepoint;
                    size_t width;

                    if (parser->text[parser->pos] == 0) {
                        qjson5_error(parser,
                                     "embedded NUL is not valid JSON5 input");
                        return false;
                    }
                    if (parser->pos + 1 < parser->length &&
                        parser->text[parser->pos] == '*' &&
                        parser->text[parser->pos + 1] == '/') {
                        parser->pos += 2;
                        if (qjson5_token_too_long(parser, comment_start)) {
                            return false;
                        }
                        closed = true;
                        break;
                    }
                    if ((unsigned char)parser->text[parser->pos] < 0x80) {
                        parser->pos++;
                        if (qjson5_token_too_long(parser, comment_start)) {
                            return false;
                        }
                        continue;
                    }
                    if (!qjson5_decode_utf8_at(parser, parser->pos,
                                               &codepoint, &width)) {
                        qjson5_error(parser,
                                     "invalid UTF-8 sequence in comment");
                        return false;
                    }
                    parser->pos += width;
                    if (qjson5_token_too_long(parser, comment_start)) {
                        return false;
                    }
                }
                if (!closed) {
                    qjson5_error(parser, "unterminated block comment");
                    return false;
                }
                continue;
            }
            return true;
        default:
            break;
        }

        if (c >= 0x80) {
            char *end;
            int codepoint = mod_utf8_codepoint(parser->text + parser->pos,
                                               MIN(parser->length - parser->pos,
                                                   (size_t)6), &end);
            if (codepoint >= 0 && qjson5_is_unicode_space(codepoint)) {
                parser->pos = end - parser->text;
                continue;
            }
        }
        break;
    }
    return !parser->err;
}

static int qjson5_hex_value(unsigned char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static bool qjson5_parse_hex(QJson5Parser *parser, size_t digits,
                             uint32_t *value)
{
    uint32_t result = 0;

    if (parser->length - parser->pos < digits) {
        qjson5_error(parser, "truncated hexadecimal escape");
        return false;
    }
    for (size_t i = 0; i < digits; i++) {
        int digit = qjson5_hex_value(parser->text[parser->pos + i]);

        if (digit < 0) {
            qjson5_error(parser, "invalid hexadecimal escape");
            return false;
        }
        result = (result << 4) | digit;
    }
    parser->pos += digits;
    *value = result;
    return true;
}

static bool qjson5_append_codepoint(QJson5Parser *parser, GString *string,
                                    int codepoint)
{
    char encoded[5];
    ssize_t length;

    length = mod_utf8_encode(encoded, sizeof(encoded), codepoint);
    if (length < 0) {
        qjson5_error(parser, "invalid Unicode codepoint");
        return false;
    }
    g_string_append_len(string, encoded, length);
    return true;
}

static QString *qjson5_parse_string(QJson5Parser *parser)
{
    GString *string = g_string_new(NULL);
    size_t start = parser->pos;
    unsigned char quote = parser->text[parser->pos++];

    while (parser->pos < parser->length) {
        unsigned char c = parser->text[parser->pos++];

        if (qjson5_token_too_long(parser, start)) {
            goto fail;
        }
        if (c == quote) {
            return qstring_from_gstring(string);
        }
        if (c == 0) {
            qjson5_error(parser, "embedded NUL is not valid in a string");
            goto fail;
        }
        if (c == '\\') {
            uint32_t codepoint;

            if (parser->pos >= parser->length) {
                qjson5_error(parser, "unterminated escape sequence");
                goto fail;
            }
            c = parser->text[parser->pos++];
            if (c == '\n') {
                continue;
            }
            if (c == '\r') {
                if (parser->pos < parser->length &&
                    parser->text[parser->pos] == '\n') {
                    parser->pos++;
                }
                continue;
            }
            if (c == 0xe2 && parser->pos + 1 < parser->length &&
                (unsigned char)parser->text[parser->pos] == 0x80 &&
                ((unsigned char)parser->text[parser->pos + 1] == 0xa8 ||
                 (unsigned char)parser->text[parser->pos + 1] == 0xa9)) {
                parser->pos += 2;
                continue;
            }

            switch (c) {
            case '\'':
                if (!qjson5_append_codepoint(parser, string, '\'')) {
                    goto fail;
                }
                break;
            case '"':
                if (!qjson5_append_codepoint(parser, string, '"')) {
                    goto fail;
                }
                break;
            case '\\':
                if (!qjson5_append_codepoint(parser, string, '\\')) {
                    goto fail;
                }
                break;
            case '/':
                if (!qjson5_append_codepoint(parser, string, '/')) {
                    goto fail;
                }
                break;
            case 'b':
                if (!qjson5_append_codepoint(parser, string, '\b')) {
                    goto fail;
                }
                break;
            case 'f':
                if (!qjson5_append_codepoint(parser, string, '\f')) {
                    goto fail;
                }
                break;
            case 'n':
                if (!qjson5_append_codepoint(parser, string, '\n')) {
                    goto fail;
                }
                break;
            case 'r':
                if (!qjson5_append_codepoint(parser, string, '\r')) {
                    goto fail;
                }
                break;
            case 't':
                if (!qjson5_append_codepoint(parser, string, '\t')) {
                    goto fail;
                }
                break;
            case 'v':
                if (!qjson5_append_codepoint(parser, string, '\v')) {
                    goto fail;
                }
                break;
            case '0':
                if (parser->pos < parser->length &&
                    parser->text[parser->pos] >= '0' &&
                    parser->text[parser->pos] <= '9') {
                    qjson5_error(parser,
                                 "\\0 escape cannot be followed by a digit");
                    goto fail;
                }
                if (!qjson5_append_codepoint(parser, string, 0)) {
                    goto fail;
                }
                break;
            case 'x':
                if (!qjson5_parse_hex(parser, 2, &codepoint) ||
                    !qjson5_append_codepoint(parser, string, codepoint)) {
                    goto fail;
                }
                break;
            case 'u':
                if (!qjson5_parse_hex(parser, 4, &codepoint)) {
                    goto fail;
                }
                if (codepoint >= 0xd800 && codepoint <= 0xdbff) {
                    uint32_t trailing;

                    if (parser->length - parser->pos < 6 ||
                        parser->text[parser->pos] != '\\' ||
                        parser->text[parser->pos + 1] != 'u') {
                        qjson5_error(parser, "unpaired Unicode surrogate");
                        goto fail;
                    }
                    parser->pos += 2;
                    if (!qjson5_parse_hex(parser, 4, &trailing) ||
                        trailing < 0xdc00 || trailing > 0xdfff) {
                        qjson5_error(parser, "invalid Unicode surrogate pair");
                        goto fail;
                    }
                    codepoint = 0x10000 +
                        ((codepoint - 0xd800) << 10) + (trailing - 0xdc00);
                } else if (codepoint >= 0xdc00 && codepoint <= 0xdfff) {
                    qjson5_error(parser, "unpaired Unicode surrogate");
                    goto fail;
                }
                if (!qjson5_append_codepoint(parser, string, codepoint)) {
                    goto fail;
                }
                break;
            default:
                if (c >= '0' && c <= '9') {
                    qjson5_error(parser,
                                 "decimal digit is not valid after escape");
                    goto fail;
                }
                if (c < 0x20) {
                    qjson5_error(parser, "invalid escape sequence in string");
                    goto fail;
                }
                if (c < 0x80) {
                    g_string_append_c(string, c);
                } else {
                    int escaped_codepoint;
                    size_t escaped_width;

                    if (!qjson5_decode_utf8_at(parser, parser->pos - 1,
                                               &escaped_codepoint,
                                               &escaped_width)) {
                        qjson5_error(parser,
                                     "invalid UTF-8 sequence in escape");
                        goto fail;
                    }
                    parser->pos = parser->pos - 1 + escaped_width;
                    if (!qjson5_append_codepoint(parser, string,
                                                 escaped_codepoint)) {
                        goto fail;
                    }
                }
            }
            continue;
        }
        if (c < 0x20) {
            qjson5_error(parser, "control character in string");
            goto fail;
        }
        if (c < 0x80) {
            g_string_append_c(string, c);
        } else {
            int codepoint;
            size_t width;

            if (!qjson5_decode_utf8_at(parser, parser->pos - 1,
                                       &codepoint, &width)) {
                qjson5_error(parser, "invalid UTF-8 sequence in string");
                goto fail;
            }
            parser->pos = parser->pos - 1 + width;
            if (!qjson5_append_codepoint(parser, string, codepoint)) {
                goto fail;
            }
        }
    }

    qjson5_error(parser, "unterminated string");

fail:
    g_string_free(string, true);
    return NULL;
}

static bool qjson5_scan_identifier(QJson5Parser *parser, bool allow_escapes,
                                   GString **decoded)
{
    size_t begin = parser->pos;
    GString *string = g_string_new(NULL);
    bool first = true;

    while (parser->pos < parser->length) {
        int codepoint;
        size_t width;
        bool escaped_identifier = false;

        if (parser->text[parser->pos] == '\\') {
            uint32_t escaped;

            if (!allow_escapes) {
                break;
            }
            escaped_identifier = true;
            parser->pos++;
            if (parser->pos >= parser->length ||
                parser->text[parser->pos] != 'u') {
                qjson5_error(parser, "invalid Unicode escape in identifier");
                goto fail;
            }
            parser->pos++;
            if (!qjson5_parse_hex(parser, 4, &escaped)) {
                goto fail;
            }
            if (escaped >= 0xd800 && escaped <= 0xdbff) {
                uint32_t trailing;

                if (parser->length - parser->pos < 6 ||
                    parser->text[parser->pos] != '\\' ||
                    parser->text[parser->pos + 1] != 'u') {
                    qjson5_error(parser, "unpaired Unicode surrogate");
                    goto fail;
                }
                parser->pos += 2;
                if (!qjson5_parse_hex(parser, 4, &trailing) ||
                    trailing < 0xdc00 || trailing > 0xdfff) {
                    qjson5_error(parser, "invalid Unicode surrogate pair");
                    goto fail;
                }
                escaped = 0x10000 + ((escaped - 0xd800) << 10) +
                    (trailing - 0xdc00);
            } else if (escaped >= 0xdc00 && escaped <= 0xdfff) {
                qjson5_error(parser, "unpaired Unicode surrogate");
                goto fail;
            }
            codepoint = escaped;
        } else if (!qjson5_decode_utf8_at(parser, parser->pos,
                                          &codepoint, &width)) {
            if ((unsigned char)parser->text[parser->pos] >= 0x80) {
                qjson5_error(parser,
                             "invalid UTF-8 sequence in identifier");
                goto fail;
            }
            break;
        }

        if (first ? !qjson5_identifier_start_codepoint(codepoint) :
                    !qjson5_identifier_continue_codepoint(codepoint)) {
            if (escaped_identifier) {
                qjson5_error(parser, "invalid codepoint in identifier");
                goto fail;
            }
            break;
        }
        if (!escaped_identifier) {
            parser->pos += width;
        }
        if (!qjson5_append_codepoint(parser, string, codepoint)) {
            goto fail;
        }
        if (qjson5_token_too_long(parser, begin) ||
            string->len > QJSON5_MAX_TOKEN_SIZE) {
            goto fail;
        }
        first = false;
    }

    if (first) {
        qjson5_error(parser, "expecting identifier");
        goto fail;
    }
    *decoded = string;
    return true;

fail:
    g_string_free(string, true);
    return false;
}

static QString *qjson5_parse_identifier_key(QJson5Parser *parser)
{
    GString *string;

    if (!qjson5_scan_identifier(parser, true, &string)) {
        return NULL;
    }
    return qstring_from_gstring(string);
}

static int qjson5_digit_value(unsigned char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static QObject *qjson5_make_integer(QJson5Parser *parser, size_t end,
                                    bool negative,
                                    unsigned base, size_t digits_start)
{
    uint64_t value = 0;
    uint64_t limit = negative ? (uint64_t)INT64_MAX + 1 : UINT64_MAX;

    for (size_t pos = digits_start; pos < end; pos++) {
        int digit = qjson5_digit_value(parser->text[pos]);

        if (digit < 0 || (unsigned)digit >= base) {
            qjson5_error(parser, "invalid digit in integer");
            return NULL;
        }
        if (value > (limit - digit) / base) {
            qjson5_error(parser, "integer is out of range");
            return NULL;
        }
        value = value * base + digit;
    }

    if (negative) {
        if (value == (uint64_t)INT64_MAX + 1) {
            return QOBJECT(qnum_from_int(INT64_MIN));
        }
        return QOBJECT(qnum_from_int(-(int64_t)value));
    }
    if (value <= INT64_MAX) {
        return QOBJECT(qnum_from_int((int64_t)value));
    }
    return QOBJECT(qnum_from_uint(value));
}

static QObject *qjson5_parse_number(QJson5Parser *parser)
{
    size_t start = parser->pos;
    size_t digits_start;
    size_t integer_digits_start;
    bool negative = false;
    bool has_fraction = false;
    bool has_exponent = false;
    bool has_digits_before = false;
    bool has_digits_after = false;
    unsigned char c;

    if (parser->text[parser->pos] == '+' ||
        parser->text[parser->pos] == '-') {
        negative = parser->text[parser->pos] == '-';
        parser->pos++;
        if (parser->pos >= parser->length) {
            qjson5_error(parser, "sign must be followed by a number");
            return NULL;
        }
        if (qjson5_is_ascii_ident_start(parser->text[parser->pos])) {
            GString *identifier;

            if (!qjson5_scan_identifier(parser, false, &identifier)) {
                return NULL;
            }
            if (qjson5_token_too_long(parser, start)) {
                g_string_free(identifier, true);
                return NULL;
            }
            if (!strcmp(identifier->str, "Infinity")) {
                double infinity = negative ? -INFINITY : INFINITY;

                g_string_free(identifier, true);
                return QOBJECT(qnum_from_double(infinity));
            }
            if (!strcmp(identifier->str, "NaN")) {
                g_string_free(identifier, true);
                return QOBJECT(qnum_from_double(NAN));
            }
            g_string_free(identifier, true);
            qjson5_error(parser, "invalid signed numeric literal");
            return NULL;
        }
    }

    if (parser->pos + 1 < parser->length && parser->text[parser->pos] == '0' &&
        (parser->text[parser->pos + 1] == 'x' ||
         parser->text[parser->pos + 1] == 'X')) {
        parser->pos += 2;
        digits_start = parser->pos;
        while (parser->pos < parser->length &&
               qjson5_digit_value(parser->text[parser->pos]) >= 0 &&
               qjson5_digit_value(parser->text[parser->pos]) < 16) {
            parser->pos++;
        }
        if (parser->pos == digits_start) {
            qjson5_error(parser, "hexadecimal literal has no digits");
            return NULL;
        }
        if (qjson5_token_too_long(parser, start)) {
            return NULL;
        }
        return qjson5_make_integer(parser, parser->pos, negative, 16,
                                   digits_start);
    }

    integer_digits_start = parser->pos;
    c = parser->text[parser->pos];
    if (c == '.') {
        has_fraction = true;
        parser->pos++;
    } else if (c >= '0' && c <= '9') {
        has_digits_before = true;
        do {
            parser->pos++;
        } while (parser->pos < parser->length &&
                 parser->text[parser->pos] >= '0' &&
                 parser->text[parser->pos] <= '9');
        if (parser->text[integer_digits_start] == '0' &&
            parser->pos - integer_digits_start > 1) {
            qjson5_error(parser, "leading zero in decimal literal");
            return NULL;
        }
        if (parser->pos < parser->length && parser->text[parser->pos] == '.') {
            has_fraction = true;
            parser->pos++;
        }
    } else {
        qjson5_error(parser, "expecting number");
        return NULL;
    }

    digits_start = parser->pos;
    while (parser->pos < parser->length &&
           parser->text[parser->pos] >= '0' &&
           parser->text[parser->pos] <= '9') {
        has_digits_after = true;
        parser->pos++;
    }
    if (!has_digits_before && !has_digits_after) {
        qjson5_error(parser, "decimal point must be followed by digits");
        return NULL;
    }

    if (parser->pos < parser->length &&
        (parser->text[parser->pos] == 'e' ||
         parser->text[parser->pos] == 'E')) {
        has_exponent = true;
        parser->pos++;
        if (parser->pos < parser->length &&
            (parser->text[parser->pos] == '+' ||
             parser->text[parser->pos] == '-')) {
            parser->pos++;
        }
        digits_start = parser->pos;
        while (parser->pos < parser->length &&
               parser->text[parser->pos] >= '0' &&
               parser->text[parser->pos] <= '9') {
            parser->pos++;
        }
        if (parser->pos == digits_start) {
            qjson5_error(parser, "exponent has no digits");
            return NULL;
        }
    }
    if (qjson5_token_too_long(parser, start)) {
        return NULL;
    }

    if (!has_fraction && !has_exponent) {
        digits_start = start;
        if (parser->text[digits_start] == '+' ||
            parser->text[digits_start] == '-') {
            digits_start++;
        }
        return qjson5_make_integer(parser, parser->pos, negative, 10,
                                   digits_start);
    }

    {
        g_autofree char *number = g_strndup(parser->text + start,
                                            parser->pos - start);
        const char *end;
        double value;
        int ret = qemu_strtod(number, &end, &value);

        if ((ret != 0 && ret != -ERANGE) ||
            end != number + (parser->pos - start)) {
            qjson5_error(parser, "invalid decimal literal");
            return NULL;
        }
        return QOBJECT(qnum_from_double(value));
    }
}

static QObject *qjson5_parse_value(QJson5Parser *parser);

static bool qjson5_enter_container(QJson5Parser *parser)
{
    if (parser->depth >= QJSON5_MAX_NESTING) {
        qjson5_error(parser, "JSON5 nesting exceeds %u levels",
                     QJSON5_MAX_NESTING);
        return false;
    }
    parser->depth++;
    return true;
}

static QObject *qjson5_parse_array(QJson5Parser *parser)
{
    QList *list;

    if (!qjson5_enter_container(parser)) {
        return NULL;
    }
    parser->pos++;
    list = qlist_new();
    if (!qjson5_skip_space(parser)) {
        goto fail;
    }
    if (parser->pos < parser->length && parser->text[parser->pos] == ']') {
        parser->pos++;
        parser->depth--;
        return QOBJECT(list);
    }

    while (parser->pos < parser->length) {
        QObject *value = qjson5_parse_value(parser);

        if (!value) {
            goto fail;
        }
        qlist_append_obj(list, value);
        if (!qjson5_skip_space(parser)) {
            goto fail;
        }
        if (parser->pos >= parser->length) {
            qjson5_error(parser, "unterminated array");
            goto fail;
        }
        if (parser->text[parser->pos] == ']') {
            parser->pos++;
            parser->depth--;
            return QOBJECT(list);
        }
        if (parser->text[parser->pos] != ',') {
            qjson5_error(parser, "expecting ',' or ']' in array");
            goto fail;
        }
        parser->pos++;
        if (!qjson5_skip_space(parser)) {
            goto fail;
        }
        if (parser->pos < parser->length && parser->text[parser->pos] == ']') {
            parser->pos++;
            parser->depth--;
            return QOBJECT(list);
        }
    }

    qjson5_error(parser, "unterminated array");

fail:
    parser->depth--;
    qobject_unref(list);
    return NULL;
}

static QObject *qjson5_parse_object(QJson5Parser *parser)
{
    QDict *dict;

    if (!qjson5_enter_container(parser)) {
        return NULL;
    }
    parser->pos++;
    dict = qdict_new();
    if (!qjson5_skip_space(parser)) {
        goto fail;
    }
    if (parser->pos < parser->length && parser->text[parser->pos] == '}') {
        parser->pos++;
        parser->depth--;
        return QOBJECT(dict);
    }

    while (parser->pos < parser->length) {
        QString *key;
        const char *key_text;
        QObject *value;

        if (parser->text[parser->pos] == '\'' ||
            parser->text[parser->pos] == '"') {
            key = qjson5_parse_string(parser);
        } else {
            key = qjson5_parse_identifier_key(parser);
        }
        if (!key) {
            goto fail;
        }
        key_text = qstring_get_str(key);
        if (!qjson5_skip_space(parser)) {
            qobject_unref(key);
            goto fail;
        }
        if (parser->pos >= parser->length || parser->text[parser->pos] != ':') {
            qjson5_error(parser, "expecting ':' after object key");
            qobject_unref(key);
            goto fail;
        }
        parser->pos++;
        if (qdict_haskey(dict, key_text)) {
            qjson5_error(parser, "duplicate object key '%s'", key_text);
            qobject_unref(key);
            goto fail;
        }
        value = qjson5_parse_value(parser);
        if (!value) {
            qobject_unref(key);
            goto fail;
        }
        qdict_put_obj(dict, key_text, value);
        qobject_unref(key);

        if (!qjson5_skip_space(parser)) {
            goto fail;
        }
        if (parser->pos >= parser->length) {
            qjson5_error(parser, "unterminated object");
            goto fail;
        }
        if (parser->text[parser->pos] == '}') {
            parser->pos++;
            parser->depth--;
            return QOBJECT(dict);
        }
        if (parser->text[parser->pos] != ',') {
            qjson5_error(parser, "expecting ',' or '}' in object");
            goto fail;
        }
        parser->pos++;
        if (!qjson5_skip_space(parser)) {
            goto fail;
        }
        if (parser->pos < parser->length && parser->text[parser->pos] == '}') {
            parser->pos++;
            parser->depth--;
            return QOBJECT(dict);
        }
    }

    qjson5_error(parser, "unterminated object");

fail:
    parser->depth--;
    qobject_unref(dict);
    return NULL;
}

static QObject *qjson5_parse_identifier_value(QJson5Parser *parser)
{
    GString *identifier;
    QObject *value = NULL;

    if (!qjson5_scan_identifier(parser, false, &identifier)) {
        return NULL;
    }
    if (!strcmp(identifier->str, "true")) {
        value = QOBJECT(qbool_from_bool(true));
    } else if (!strcmp(identifier->str, "false")) {
        value = QOBJECT(qbool_from_bool(false));
    } else if (!strcmp(identifier->str, "null")) {
        value = QOBJECT(qnull());
    } else if (!strcmp(identifier->str, "NaN")) {
        value = QOBJECT(qnum_from_double(NAN));
    } else if (!strcmp(identifier->str, "Infinity")) {
        value = QOBJECT(qnum_from_double(INFINITY));
    }
    if (!value) {
        qjson5_error(parser, "unknown identifier '%s'", identifier->str);
    }
    g_string_free(identifier, true);
    return value;
}

static QObject *qjson5_parse_value(QJson5Parser *parser)
{
    if (!qjson5_skip_space(parser)) {
        return NULL;
    }
    if (parser->pos >= parser->length) {
        qjson5_error(parser, "expecting JSON5 value");
        return NULL;
    }
    switch (parser->text[parser->pos]) {
    case '{':
        return qjson5_parse_object(parser);
    case '[':
        return qjson5_parse_array(parser);
    case '\'':
    case '"':
        return QOBJECT(qjson5_parse_string(parser));
    case '+':
    case '-':
    case '.':
        return qjson5_parse_number(parser);
    default:
        if ((parser->text[parser->pos] >= '0' &&
             parser->text[parser->pos] <= '9')) {
            return qjson5_parse_number(parser);
        }
        {
            size_t width;

            if (qjson5_identifier_codepoint_at(parser, parser->pos, true,
                                               &width)) {
                return qjson5_parse_identifier_value(parser);
            }
        }
        qjson5_error(parser, "unexpected character in JSON5 value");
        return NULL;
    }
}

QObject *qobject_from_json5(const char *text, size_t length, Error **errp)
{
    QJson5Parser parser = {
        .text = text,
        .length = length,
    };
    QObject *result;

    if (length > QJSON5_MAX_INPUT_SIZE) {
        error_setg(errp, "JSON5 input exceeds %u bytes",
                   QJSON5_MAX_INPUT_SIZE);
        return NULL;
    }
    if (!text) {
        error_setg(errp, "JSON5 input is NULL");
        return NULL;
    }

    result = qjson5_parse_value(&parser);
    if (result && qjson5_skip_space(&parser) && parser.pos != parser.length) {
        qjson5_error(&parser, "trailing input after JSON5 value");
    }
    if (parser.err) {
        qobject_unref(result);
        result = NULL;
    }
    error_propagate(errp, parser.err);
    return result;
}
