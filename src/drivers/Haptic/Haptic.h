#ifndef HAPTIC_H
#define HAPTIC_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file Haptic.h
 * \brief Cane vibration motor driver interface (sole writer of GP15)
 *
 * Each source posts the cue it wants with Haptic_Request(); Haptic_PlayOnce()
 * adds a short cue that plays a single time (calibration feedback).
 * Haptic_Step() picks the highest-priority cue and plays it without blocking.
 * Priority: STEP_DOWN > STEP_UP > calibration feedback > caution > LINK_LOSS
 * > calibration prompt. Step warnings pre-empting LINK_LOSS was confirmed at
 * Tribunal 1 (12:50PM). The motor is silent while the cane is not gripped.
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t gpio_pin;
} Haptic_Config_t;

typedef enum
{
    HAPTIC_SRC_GROUND = 0U,
    HAPTIC_SRC_LINK,
    HAPTIC_SRC_COUNT
} Haptic_Source_t;

extern const Haptic_Config_t Haptic_Config_Cane;

void Haptic_Init(const Haptic_Config_t * const p_cfg);
void Haptic_Request(Haptic_Source_t source, Alert_Type_t pattern_type);
void Haptic_PlayOnce(Alert_Type_t pattern_type, uint32_t now_ms);
void Haptic_SetGripHeld(bool held);
void Haptic_Step(uint32_t now_ms);
Alert_Type_t Haptic_GetActivePattern(void);
bool Haptic_IsMotorOn(void);

#endif /* HAPTIC_H */
