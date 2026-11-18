/**
 * @file    alert.h
 * @brief   Audible alerts of the glasses node: fault tones and obstacle
 *          beeps.
 */

#ifndef ALERT_H
#define ALERT_H

#include <stdbool.h>
#include <stdint.h>

/* Fault sources, one bit each. The IMU outranks the radar when both are
 * active. The values are the fault flag bits of protocol.h.
 * NOTE: The TOF and BLE_LOSS values are proposals that still have to be
 * agreed with the rest of the team. */
typedef enum
{
    ALERT_FAULT_IMU      = 0x01,
    ALERT_FAULT_RADAR    = 0x02,
    ALERT_FAULT_TOF      = 0x04,
    ALERT_FAULT_BLE_LOSS = 0x08
} alert_fault_t;

/* Sets up the buzzer on the given GPIO pin and silences it. */
void alert_init (uint32_t pin);

/* Updates the tone. Call it every loop. It never blocks. */
void alert_update (uint32_t now_ms);

/* Sets or clears one fault source. Safe to call from a callback. */
void alert_set_fault (alert_fault_t source, bool b_active);

/* Sets the obstacle distance in mm. A value of 0 or less, or above 2000,
 * means silent. The value is stored in 16 bits. */
void alert_set_obstacle (int32_t dist_mm);

/* Returns the active fault sources as a bit mask of alert_fault_t values. */
uint8_t alert_get_fault_mask (void);

#endif /* ALERT_H */

/* end of alert.h */

