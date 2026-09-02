/*
 * Bounded JSON5 QObject parser.
 *
 * This parser is intentionally separate from QEMU's QMP JSON parser.  It is
 * intended for trusted, versioned configuration files and accepts a small
 * JSON5 grammar while retaining explicit resource bounds.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QOBJECT_QJSON5_H
#define QOBJECT_QJSON5_H

#include "qapi/error.h"
#include "qobject/qobject.h"

/* Limits are inclusive: an input/token of exactly the limit is accepted. */
#define QJSON5_MAX_INPUT_SIZE  (4U * 1024U * 1024U)
#define QJSON5_MAX_NESTING     128U
#define QJSON5_MAX_TOKEN_SIZE  (1U * 1024U * 1024U)

/*
 * Parse exactly @length bytes of JSON5 input.
 *
 * The input does not need to be NUL terminated.  A NUL byte in the supplied
 * range is rejected, including in comments; a NUL after @length is ignored.
 * On success the returned QObject is a strong reference owned by the caller.
 */
QObject *qobject_from_json5(const char *text, size_t length, Error **errp);

#endif /* QOBJECT_QJSON5_H */
