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

void alert_init (uint pin);
void alert_update (uint32_t now_ms);                 /* call every loop, never blocks */
void alert_set_fault (alert_fault_t source, bool active);
void alert_set_obstacle (int32_t dist_mm);           /* <= 0 or > 2000 means silent */

#endif
