#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

typedef struct
{
    float ax, ay, az;   /* acceleration in g */
    float gx, gy, gz;   /* angular rate in degrees/s */
} imu_sample_t;

typedef enum
{
    MPU9250_CAL_OK = 0,       /* bias measured and applied */
    MPU9250_CAL_BUS_ERROR,    /* too many failed reads, bias left at zero */
    MPU9250_CAL_MOVING        /* sensor was not still, bias left at zero */
} mpu9250_cal_t;

uint8_t mpu9250_whoami (i2c_inst_t *i2c, uint8_t addr);
bool    mpu9250_init (i2c_inst_t *i2c, uint8_t addr);
bool    mpu9250_read (imu_sample_t *s);

/* Measures the gyro zero-rate bias over n_samples (5 ms apart). Rejects the
 * measurement if the readings moved too much. range_dps (may be NULL) receives
 * the largest max-min spread of any axis, to help tune the threshold. */
mpu9250_cal_t mpu9250_calibrate_gyro (int n_samples, float *range_dps);

#endif
