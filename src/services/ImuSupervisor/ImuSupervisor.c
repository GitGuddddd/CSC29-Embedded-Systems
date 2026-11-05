#include "ImuSupervisor.h"
#include "ImuSupervisor_Cfg.h"
#include "hal/Hal_I2c.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(IMUSUPERVISOR_FAILURE_COUNT <= 3U, "Failure counter needs more bits");
_Static_assert(IMUSUPERVISOR_RECOVERY_COUNT <= 3U, "Recovery counter needs more bits");
_Static_assert(MPU9250_INIT_DONE <= 7U, "Initialization phase needs more bits");

static void Enter_Fault(ImuSupervisor_t * const p_s, uint32_t now_ms)
{
    p_s->state = IMU_SUPERVISOR_FAULT;
    p_s->fault = true;
    p_s->sample_valid = false;
    p_s->successes = 0U;
    p_s->init_step = MPU9250_INIT_DONE;
    p_s->transition_ms = now_ms;
}

static void Start_Settling(ImuSupervisor_t * const p_s, uint32_t now_ms)
{
    p_s->state = IMU_SUPERVISOR_STARTING;
    p_s->transition_ms = now_ms;
    p_s->failures = 0U;
    p_s->successes = 0U;
    p_s->sample_valid = false;
    p_s->init_step = MPU9250_INIT_DONE;
}

void ImuSupervisor_Init(ImuSupervisor_t * const p_s, const Mpu9250_Config_t * const p_cfg,
                        uint8_t sda_pin, uint8_t scl_pin, uint32_t now_ms)
{
    if (p_s == NULL)
    {
        return;
    }
    memset(p_s, 0, sizeof(*p_s));
    p_s->p_cfg = p_cfg;
    p_s->sda_pin = sda_pin;
    p_s->scl_pin = scl_pin;
    if (p_cfg != NULL)
    {
        /* Software-only startup: the first cyclic calls perform the register
         * sequence, allowing retained alarms/BLE/touch to run immediately.
         */
        p_s->state = IMU_SUPERVISOR_STARTING;
        p_s->init_step = MPU9250_INIT_IDENTITY;
        p_s->transition_ms = now_ms;
    }
    else
    {
        Enter_Fault(p_s, now_ms);
    }
}

void ImuSupervisor_Step(ImuSupervisor_t * const p_s, bool sleep_requested, uint32_t now_ms)
{
    if ((p_s == NULL) || (p_s->p_cfg == NULL))
    {
        return;
    }
    p_s->sample_valid = false;
    if (p_s->init_step != MPU9250_INIT_DONE)
    {
        if (sleep_requested || ((p_s->init_step == MPU9250_INIT_WAKE) &&
            ((uint32_t)(now_ms - p_s->transition_ms) < MPU9250_INIT_RESET_WAIT_MS)))
        {
            return;
        }
        Mpu9250_InitStep_t step = (Mpu9250_InitStep_t)p_s->init_step;
        uint32_t completed_ms = now_ms;
        if (!Mpu9250_RunInitStep(p_s->p_cfg, step, now_ms, &completed_ms))
        {
            Enter_Fault(p_s, completed_ms);
        }
        else
        {
            p_s->init_step = (uint8_t)((p_s->init_step + 1U) & 0x07U);
            if (step == MPU9250_INIT_RESET) p_s->transition_ms = completed_ms;
            if (p_s->init_step == MPU9250_INIT_DONE) Start_Settling(p_s, completed_ms);
        }
        return; /* Never read samples or perform a second action this cycle. */
    }
    if (p_s->state == IMU_SUPERVISOR_FAULT)
    {
        if (!sleep_requested &&
            ((uint32_t)(now_ms - p_s->transition_ms) >= IMUSUPERVISOR_RETRY_MS))
        {
            /* The owning task is the only user of this IMU bus. Restore its mux
             * after bus clear, then verify identity/configuration again.
             */
            Hal_I2c_BusReset(p_s->sda_pin, p_s->scl_pin);
            Hal_I2c_Init(p_s->p_cfg->i2c_port, p_s->sda_pin, p_s->scl_pin, IMUSUPERVISOR_I2C_HZ);
            p_s->init_step = MPU9250_INIT_IDENTITY;
            p_s->failures = 0U;
            p_s->successes = 0U;
        }
        return;
    }
    if (sleep_requested)
    {
        if (p_s->state != IMU_SUPERVISOR_ASLEEP)
        {
            uint32_t completed_ms = now_ms;
            if (Mpu9250_SetSleepTimed(p_s->p_cfg, true, now_ms, &completed_ms))
            {
                p_s->state = IMU_SUPERVISOR_ASLEEP;
            }
            else
            {
                Enter_Fault(p_s, completed_ms);
            }
        }
        return;
    }
    if (p_s->state == IMU_SUPERVISOR_ASLEEP)
    {
        uint32_t completed_ms = now_ms;
        if (Mpu9250_SetSleepTimed(p_s->p_cfg, false, now_ms, &completed_ms))
        {
            Start_Settling(p_s, completed_ms);
        }
        else
        {
            Enter_Fault(p_s, completed_ms);
        }
        return;
    }
    if ((p_s->state == IMU_SUPERVISOR_STARTING) &&
        ((uint32_t)(now_ms - p_s->transition_ms) < IMUSUPERVISOR_SETTLE_MS))
    {
        return;
    }

    Imu_Sample_t sample;
    bool valid = Mpu9250_ReadSample(p_s->p_cfg, &sample);
    valid = valid && isfinite(sample.ax) && isfinite(sample.ay) && isfinite(sample.az) &&
            isfinite(sample.gx) && isfinite(sample.gy) && isfinite(sample.gz);
    if (!valid)
    {
        p_s->successes = 0U;
        if (p_s->failures < IMUSUPERVISOR_FAILURE_COUNT)
        {
            ++p_s->failures;
        }
        if (p_s->failures >= IMUSUPERVISOR_FAILURE_COUNT)
        {
            Enter_Fault(p_s, now_ms);
        }
        return;
    }
    p_s->failures = 0U;
    if (p_s->successes < IMUSUPERVISOR_RECOVERY_COUNT)
    {
        ++p_s->successes;
    }
    if (p_s->successes >= IMUSUPERVISOR_RECOVERY_COUNT)
    {
        p_s->state = IMU_SUPERVISOR_RUNNING;
        p_s->fault = false;
        p_s->sample = sample;
        p_s->sample.timestamp_ms = now_ms;
        p_s->sample_valid = true;
    }
}
