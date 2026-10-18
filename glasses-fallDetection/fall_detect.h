#ifndef FALL_DETECT_H
#define FALL_DETECT_H

#include <stdbool.h>
#include <stdint.h>
#include "mpu9250.h"

// Returns true exactly once when a fall is confirmed
bool fall_detect_update(const imu_sample_t *s, uint32_t now_ms);
const char *fall_detect_state_name(void);
void fall_detect_reset(void);
#endif