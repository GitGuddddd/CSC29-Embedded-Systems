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

/* Non-blocking gyro offset calibration (Mpu9250_GyroCalStep), one sample per
 * 10 ms call. The window is accepted only if the device stayed still.
 */
#define MPU9250_CFG_GYRO_CAL_SAMPLES   (100U)  /* 1 s at 100 Hz, max 127 */
#define MPU9250_CFG_GYRO_CAL_SPREAD_DPS (4.0f) /* max - min per gyro axis */
#define MPU9250_CFG_GYRO_CAL_BIAS_DPS  (10.0f) /* largest believable offset */
#define MPU9250_CFG_GYRO_CAL_ACC_TOL_G (0.05f) /* accel drift per axis */

#endif /* MPU9250_CFG_H */
