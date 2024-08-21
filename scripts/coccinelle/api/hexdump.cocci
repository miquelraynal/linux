// SPDX-License-Identifier: GPL-2.0-only
///
/// Convert callers of print_hex_dump(_debug) to the new prototype avoiding the
/// use of a boolean for the ascii parameter and changing it to a flags entry.
///
// Confidence: Low
// Copyright: (C) 2024, Miquèl Raynal, Bootlin.
// Options:
//
// Keywords: print_hex_dump, dump
// Version min: 6.10.0
//

virtual context
virtual patch
virtual org
virtual report


@ depends on !context && patch && !org && !report @
expression E;
@@

(
print_hex_dump
|
print_hex_dump_debug
|
dynamic_hex_dump
)
 (E,...,
(
- true
+ DUMP_FLAG_ASCII
|
- 1
+ DUMP_FLAG_ASCII
|
- false
+ 0
)
 );
