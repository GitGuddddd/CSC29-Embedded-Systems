#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Vl53l1x.h
 * \brief VL53L1X ToF driver interface
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t i2c_port;
    uint8_t dev_addr;
    uint8_t xshut_pin;
} Vl53l1x_Config_t;

extern const Vl53l1x_Config_t Vl53l1x_Config_Cane;

bool Vl53l1x_Init(const Vl53l1x_Config_t * const p_cfg);
bool Vl53l1x_ReadDistanceMm(const Vl53l1x_Config_t * const p_cfg, uint16_t * const p_dist_mm);

#endif /* VL53L1X_H */
