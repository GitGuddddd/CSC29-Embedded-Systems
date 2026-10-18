#include "Mpu9250.h"

/*
 * \file Mpu9250_Lcfg.c
 * \brief Link-time configuration for Cane vs Glasses IMU
 * Compliance: Barr-C:2018
 */

const Mpu9250_Config_t Mpu9250_Config_Cane = {
    .i2c_port = 0U,
    .dev_addr = 0x68U,
    .bias_gx  = 0.0f,
    .bias_gy  = 0.0f,
    .bias_gz  = 0.0f
};

const Mpu9250_Config_t Mpu9250_Config_Glasses = {
    .i2c_port = 1U,
    .dev_addr = 0x68U,
    .bias_gx  = 0.0f,
    .bias_gy  = 0.0f,
    .bias_gz  = 0.0f
};
