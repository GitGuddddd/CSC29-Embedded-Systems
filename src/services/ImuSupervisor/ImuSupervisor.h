#ifndef IMUSUPERVISOR_H
#define IMUSUPERVISOR_H

#include "drivers/Mpu9250/Mpu9250.h"

typedef enum
{
    IMU_SUPERVISOR_STARTING = 0U,
    IMU_SUPERVISOR_RUNNING,
    IMU_SUPERVISOR_ASLEEP,
    IMU_SUPERVISOR_FAULT
} ImuSupervisor_State_t;

typedef struct
{
    const Mpu9250_Config_t * p_cfg;
    uint32_t transition_ms;
    Imu_Sample_t sample;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t state : 2;
    uint8_t failures : 2;
    uint8_t successes : 2;
    uint8_t fault : 1;
    uint8_t sample_valid : 1;
    uint8_t init_step : 3;
    uint8_t reserved : 5;
} ImuSupervisor_t;

void ImuSupervisor_Init(ImuSupervisor_t * const p_s, const Mpu9250_Config_t * const p_cfg,
                        uint8_t sda_pin, uint8_t scl_pin, uint32_t now_ms);
/* One bounded initialization action/acquisition/verified power transition per call.
 * Reset/settling waits perform no I/O. ASLEEP performs no bus transactions.
 */
void ImuSupervisor_Step(ImuSupervisor_t * const p_s, bool sleep_requested, uint32_t now_ms);

#endif /* IMUSUPERVISOR_H */
