#ifndef TOUCHSENSOR_H
#define TOUCHSENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file TouchSensor.h
 * \brief Capacitive touch grip driver with 20ms debounce (FR1, FR2)
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t gpio_pin;
} TouchSensor_Config_t;

extern const TouchSensor_Config_t TouchSensor_Config_Cane;

void TouchSensor_Init(const TouchSensor_Config_t * const p_cfg);
bool TouchSensor_Update(const TouchSensor_Config_t * const p_cfg, bool raw_pin_level, Grip_State_t * const p_state_out);

#endif /* TOUCHSENSOR_H */
