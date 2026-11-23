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

typedef enum
{
    MPU9250_INIT_IDENTITY = 0U,
    MPU9250_INIT_RESET,
    MPU9250_INIT_WAKE,
    MPU9250_INIT_DLPF,
    MPU9250_INIT_GYRO,
    MPU9250_INIT_ACCEL,
    MPU9250_INIT_ACCEL_DLPF,
    MPU9250_INIT_DONE
} Mpu9250_InitStep_t;

#define MPU9250_INIT_RESET_WAIT_MS (50U)

/* One bounded register action, no delay/loop. The caller waits RESET_WAIT_MS
 * after RESET completes before WAKE. Completion uses the boot-time epoch;
 * request_ms supplies the equivalent time for host-only I/O.
 */
bool Mpu9250_RunInitStep(const Mpu9250_Config_t * const p_cfg, Mpu9250_InitStep_t step,
                         uint32_t request_ms, uint32_t * const p_completed_ms);
/* Legacy synchronous bring-up; not for the cyclic perception/recovery path. */
bool Mpu9250_Init(const Mpu9250_Config_t * const p_cfg);
/* PWR_MGMT_1 hardware sleep. After waking, the caller must wait for sensor settling. */
bool Mpu9250_SetSleep(const Mpu9250_Config_t * const p_cfg, bool sleep_enabled);
bool Mpu9250_SetSleepTimed(const Mpu9250_Config_t * const p_cfg, bool sleep_enabled,
                           uint32_t request_ms, uint32_t * const p_completed_ms);
uint8_t Mpu9250_ReadWhoAmI(const Mpu9250_Config_t * const p_cfg);
bool Mpu9250_ReadSample(const Mpu9250_Config_t * const p_cfg, Imu_Sample_t * const p_sample);
void Mpu9250_CalibrateGyro(const Mpu9250_Config_t * const p_cfg, uint32_t n_samples);

/* Non-blocking gyro offset calibration. Feed every new sample (one call per
 * 10 ms cycle). After MPU9250_CFG_GYRO_CAL_SAMPLES samples with the device
 * still, the mean gyro reading becomes the offset subtracted by
 * Mpu9250_ReadSample. A window with movement is discarded and a new one starts.
 */
typedef enum
{
    MPU9250_GYRO_CAL_COLLECTING = 0U, /* window still filling */
    MPU9250_GYRO_CAL_APPLIED,         /* window was still: offset set now */
    MPU9250_GYRO_CAL_REJECTED,        /* device moved: offset unchanged */
    MPU9250_GYRO_CAL_DONE             /* already calibrated: nothing to do */
} Mpu9250_GyroCalStatus_t;

typedef struct
{
    float sum_gx;
    float sum_gy;
    float sum_gz;
    float min_gx;
    float max_gx;
    float min_gy;
    float max_gy;
    float min_gz;
    float max_gz;
    float first_ax;
    float first_ay;
    float first_az;
    uint8_t count : 7;    /* samples in the current window, 0..127 */
    uint8_t done : 1;     /* offset applied */
} Mpu9250_GyroCal_t;

/* Starts a fresh calibration and clears the applied gyro offset to 0. */
void Mpu9250_GyroCalInit(Mpu9250_GyroCal_t * const p_cal);
Mpu9250_GyroCalStatus_t Mpu9250_GyroCalStep(
    Mpu9250_GyroCal_t * const p_cal, const Imu_Sample_t * const p_sample);
void Mpu9250_GetGyroOffset(float * const p_gx, float * const p_gy,
                           float * const p_gz);

#endif /* MPU9250_H */
