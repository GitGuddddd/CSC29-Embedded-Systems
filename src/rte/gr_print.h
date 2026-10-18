/** @file gr_print.h
 *
 * @brief One print function for the target (tm_printf) and the PC (printf).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * NOTE: tm_printf writes to the USB console of the uT-Kernel template. It
 * understands %d %u %x %s %c with the l qualifier, which is all the radar
 * code uses.
 */

#ifndef GR_PRINT_H
#define GR_PRINT_H

#if defined(__arm__)
#include "tk_include.h"
#include <tm/tmonitor.h>
#define gr_printf(...)    ((void) tm_printf((const UB *) __VA_ARGS__))
#else
#include <stdio.h>
#define gr_printf(...)    ((void) printf(__VA_ARGS__))
#endif

#endif /* GR_PRINT_H */

/*** end of file ***/
