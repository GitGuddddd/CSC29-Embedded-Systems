#ifndef MPU9250_CFG_H
#define MPU9250_CFG_H

/*
 * \file Mpu9250_Cfg.h
 * \brief Compile-time configuration for Mpu9250 / Mpu6500 driver
 * Compliance: Barr-C:2018
 */

#define MPU9250_CFG_SAMPLE_RATE_HZ     (100U)
#define MPU9250_CFG_DLPF_CONFIG        (0x03U) /* ~44 Hz bandwidth */
#define MPU9250_CFG_GYRO_RANGE_1000DPS (0x10U)
#define MPU9250_CFG_ACCEL_RANGE_8G     (0x10U)

#endif /* MPU9250_CFG_H */
