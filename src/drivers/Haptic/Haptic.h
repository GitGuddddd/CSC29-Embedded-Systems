#ifndef HAPTIC_H
#define HAPTIC_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file Haptic.h
 * \brief Cane vibration motor driver interface
 * Priority preemption: physical hazard aborts link loss warning
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t gpio_pin;
} Haptic_Config_t;

extern const Haptic_Config_t Haptic_Config_Cane;

void Haptic_Init(const Haptic_Config_t * const p_cfg);
void Haptic_PlayPattern(Alert_Type_t pattern_type);
void Haptic_Abort(void);
bool Haptic_IsAborted(void);
void Haptic_Step(uint32_t now_ms);

#endif /* HAPTIC_H */
