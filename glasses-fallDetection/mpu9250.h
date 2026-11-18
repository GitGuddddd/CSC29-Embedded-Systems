/**
 * @file    mpu9250.h
 * @brief   I2C driver for the MPU-6500 / MPU-9250 accelerometer and gyro.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

/* One sensor reading. */
typedef struct
{
    float accel_x;   /* Acceleration along X in g. */
    float accel_y;   /* Acceleration along Y in g. */
    float accel_z;   /* Acceleration along Z in g. */
    float gyro_x;    /* Angular rate around X in degrees per second. */
    float gyro_y;    /* Angular rate around Y in degrees per second. */
    float gyro_z;    /* Angular rate around Z in degrees per second. */
} imu_sample_t;

/* Result of the gyro calibration. */
typedef enum
{
    MPU9250_CAL_OK = 0,      /* Bias measured and applied. */
    MPU9250_CAL_BUS_ERROR,   /* Too many failed reads, bias left at zero. */
    MPU9250_CAL_MOVING       /* Sensor was not still, bias left at zero. */
} mpu9250_cal_t;

/* Reads the WHO_AM_I register. Returns 0 if the read fails. */
uint8_t mpu9250_whoami (i2c_inst_t * p_i2c, uint8_t addr);

/* Checks WHO_AM_I and configures the sensor: +-8 g, +-1000 dps and a
 * low-pass filter of about 44 Hz. Returns false if it fails. */
bool mpu9250_init (i2c_inst_t * p_i2c, uint8_t addr);

/* Reads one sample. Returns false if the bus read fails. */
bool mpu9250_read (imu_sample_t * p_sample);

/* Measures the gyro zero-rate bias over num_samples readings, 5 ms apart.
 * Rejects the measurement if the readings moved too much. The largest
 * max-min spread of any axis is written to p_range_dps (may be NULL), to
 * help tune the threshold. */
mpu9250_cal_t mpu9250_calibrate_gyro (uint32_t  num_samples,
                                      float   * p_range_dps);

#endif /* MPU9250_H */

/* end of mpu9250.h */

