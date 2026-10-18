#include "Mpu9250.h"
#include "Mpu9250_Cfg.h"
#include "hal/Hal_I2c.h"
#include <stddef.h>

#ifdef PICO_ON_DEVICE
#include "pico/time.h"
#endif

#define REG_PWR_MGMT_1    (0x6BU)
#define REG_CONFIG        (0x1AU)
#define REG_GYRO_CONFIG   (0x1BU)
#define REG_ACCEL_CONFIG  (0x1CU)
#define REG_ACCEL_CONFIG2 (0x1DU)
#define REG_ACCEL_XOUT_H  (0x3BU)
#define REG_WHO_AM_I      (0x75U)

#define WHO_AM_I_9250     (0x71U)
#define WHO_AM_I_9255     (0x73U)
#define WHO_AM_I_6500     (0x70U)

#define ACC_SENS_8G       (0.000244140625f) /* 8g range / 32768 */
#define GYR_SENS_1000     (0.030517578125f) /* 1000dps range / 32768 */

static float s_cal_gx = 0.0f;
static float s_cal_gy = 0.0f;
static float s_cal_gz = 0.0f;

uint8_t Mpu9250_ReadWhoAmI(const Mpu9250_Config_t * const p_cfg)
{
    if (p_cfg == NULL)
    {
        return 0U;
    }
    uint8_t val = 0U;
    if (Hal_I2c_ReadRegs(p_cfg->i2c_port, p_cfg->dev_addr, REG_WHO_AM_I, &val, 1U))
    {
        return val;
    }
    return 0U;
}

bool Mpu9250_Init(const Mpu9250_Config_t * const p_cfg)
{
    if (p_cfg == NULL)
    {
        return false;
    }

    uint8_t who = Mpu9250_ReadWhoAmI(p_cfg);
    if ((who != WHO_AM_I_9250) && (who != WHO_AM_I_9255) && (who != WHO_AM_I_6500))
    {
        return false;
    }

    /* Reset device */
    if (!Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, REG_PWR_MGMT_1, 0x80U))
    {
        return false;
    }

#ifdef PICO_ON_DEVICE
    sleep_ms(50);
#endif

    bool ok = true;
    ok = ok && Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, REG_PWR_MGMT_1, 0x01U);
    ok = ok && Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, REG_CONFIG, MPU9250_CFG_DLPF_CONFIG);
    ok = ok && Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, REG_GYRO_CONFIG, MPU9250_CFG_GYRO_RANGE_1000DPS);
    ok = ok && Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, REG_ACCEL_CONFIG, MPU9250_CFG_ACCEL_RANGE_8G);
    ok = ok && Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, REG_ACCEL_CONFIG2, MPU9250_CFG_DLPF_CONFIG);

    return ok;
}

bool Mpu9250_ReadSample(const Mpu9250_Config_t * const p_cfg, Imu_Sample_t * const p_sample)
{
    if ((p_cfg == NULL) || (p_sample == NULL))
    {
        return false;
    }

    uint8_t buf[14];
    if (!Hal_I2c_ReadRegs(p_cfg->i2c_port, p_cfg->dev_addr, REG_ACCEL_XOUT_H, buf, 14U))
    {
        return false;
    }

    int16_t ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    int16_t gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    p_sample->ax = (float)ax * ACC_SENS_8G;
    p_sample->ay = (float)ay * ACC_SENS_8G;
    p_sample->az = (float)az * ACC_SENS_8G;
    p_sample->gx = ((float)gx * GYR_SENS_1000) - s_cal_gx;
    p_sample->gy = ((float)gy * GYR_SENS_1000) - s_cal_gy;
    p_sample->gz = ((float)gz * GYR_SENS_1000) - s_cal_gz;
    p_sample->timestamp_ms = 0U;

    return true;
}

void Mpu9250_CalibrateGyro(const Mpu9250_Config_t * const p_cfg, uint32_t n_samples)
{
    if ((p_cfg == NULL) || (n_samples == 0U))
    {
        return;
    }

    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;
    Imu_Sample_t s;

    for (uint32_t i = 0U; i < n_samples; i++)
    {
        if (Mpu9250_ReadSample(p_cfg, &s))
        {
            sx += s.gx;
            sy += s.gy;
            sz += s.gz;
        }
    }

    s_cal_gx = sx / (float)n_samples;
    s_cal_gy = sy / (float)n_samples;
    s_cal_gz = sz / (float)n_samples;
}
