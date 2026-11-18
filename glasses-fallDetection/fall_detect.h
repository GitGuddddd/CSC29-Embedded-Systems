/**
 * @file    fall_detect.h
 * @brief   Head fall detector: free fall, impact, stillness and tilt check.
 */

#ifndef FALL_DETECT_H
#define FALL_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#include "mpu9250.h"

/* Feeds one sample (every 10 ms) into the detector. Returns true exactly
 * once when a fall is confirmed. */
bool fall_detect_update (const imu_sample_t * p_sample, uint32_t now_ms);

/* Returns the name of the current detector state, for logging. */
const char * fall_detect_state_name (void);

/* Returns the detector to its idle state. */
void fall_detect_reset (void);

#endif /* FALL_DETECT_H */

/* end of fall_detect.h */

