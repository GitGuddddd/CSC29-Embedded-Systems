#ifndef ALERT_H
#define ALERT_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/types.h"

/* Fault sources, one bit each. IMU outranks radar when both are active. */
typedef enum
{
    ALERT_FAULT_IMU   = 0x01,
    ALERT_FAULT_RADAR = 0x02
} alert_fault_t;

/* BRING-UP EDIT (Putra, 8 Nov): FR9 cue byte (prof feedback: tell the user
 * what to do, not just how close). Same bit layout as rte_type.h's
 * RTE_CUE_*; defined again here, not included, so this file stays free of
 * the RTE types (same reason alert_set_obstacle takes a plain int32_t).
 * Brian owns the final value; this is a bring-up placeholder (see alert.c). */
#define ALERT_CUE_STOP        (0x01u)  /* continuous tone, no pulsing */
#define ALERT_CUE_SIDE_SHIFT  (1u)
#define ALERT_CUE_SIDE_MASK   (0x03u)
#define ALERT_CUE_SIDE_CENTER (0u)
#define ALERT_CUE_SIDE_LEFT   (1u)
#define ALERT_CUE_SIDE_RIGHT  (2u)

void alert_init (uint pin);
void alert_update (uint32_t now_ms);                 /* call every loop, never blocks */
void alert_set_fault (alert_fault_t source, bool active);
void alert_set_obstacle (int32_t dist_mm);           /* <= 0 or > 2000 means silent */
void alert_set_cue (uint8_t cue);          /* ALERT_CUE_* bits; 0 = none */

#endif

/*** end of file ***/
