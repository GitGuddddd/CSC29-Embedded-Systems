#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Vl53l0x.h
 * \brief VL53L0X / VL53L0XV2 ToF sensor driver interface
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t i2c_port;
    uint8_t dev_addr;
    uint8_t xshut_pin;
} Vl53l0x_Config_t;

extern const Vl53l0x_Config_t Vl53l0x_Config_Cane;

bool Vl53l0x_Init(const Vl53l0x_Config_t * const p_cfg);
bool Vl53l0x_ReadDistanceMm(const Vl53l0x_Config_t * const p_cfg, uint16_t * const p_dist_mm);

#endif /* VL53L0X_H */
