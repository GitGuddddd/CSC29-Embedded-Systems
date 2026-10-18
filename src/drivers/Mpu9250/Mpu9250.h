#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file Mpu9250.h
 * \brief MPU-9250 / MPU-6500 IMU driver interface
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t i2c_port;
    uint8_t dev_addr;
    float bias_gx;
    float bias_gy;
    float bias_gz;
} Mpu9250_Config_t;

extern const Mpu9250_Config_t Mpu9250_Config_Cane;
extern const Mpu9250_Config_t Mpu9250_Config_Glasses;

bool Mpu9250_Init(const Mpu9250_Config_t * const p_cfg);
uint8_t Mpu9250_ReadWhoAmI(const Mpu9250_Config_t * const p_cfg);
bool Mpu9250_ReadSample(const Mpu9250_Config_t * const p_cfg, Imu_Sample_t * const p_sample);
void Mpu9250_CalibrateGyro(const Mpu9250_Config_t * const p_cfg, uint32_t n_samples);

#endif /* MPU9250_H */
