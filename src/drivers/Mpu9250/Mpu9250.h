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

#endif /* MPU9250_H */
