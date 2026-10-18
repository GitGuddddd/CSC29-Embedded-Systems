/** @file hal_time.c
 *
 * @brief Millisecond time base from the uT-Kernel operating time.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdint.h>

#include "tk_include.h"

#include "hal_time.h"

/*!
 * @brief Milliseconds since boot. Wraps after about 49 days.
 *
 * NOTE: Callers compare times by subtracting, so the wrap is harmless.
 * tk_get_otm counts milliseconds from kernel start (it is not affected by
 * tk_set_tim).
 *
 * @return Milliseconds since boot.
 */
uint32_t
hal_time_now_ms (void)
{
    SYSTIM tim;

    (void) tk_get_otm(&tim);

    return ((uint32_t) tim.lo);
}

/*** end of file ***/
