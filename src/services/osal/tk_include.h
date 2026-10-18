/** @file tk_include.h
 *
 * @brief Include the uT-Kernel headers next to the standard C headers.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The kernel header tk/syslib.h defines its own size_t, which clashes with
 * the one from <stddef.h>. PROHIBIT_DEF_SIZE_T (a switch the kernel header
 * provides) turns the kernel copy off.
 */

#ifndef TK_INCLUDE_H
#define TK_INCLUDE_H

#include <stddef.h>

#define PROHIBIT_DEF_SIZE_T

#include <tk/tkernel.h>

#endif /* TK_INCLUDE_H */

/*** end of file ***/
