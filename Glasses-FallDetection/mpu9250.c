#include "mpu9250.h"
#include <math.h>
#include "pico/stdlib.h"

// Register addresses (MPU-9250 register map, RM-MPU-9250A-00)
#define REG_PWR_MGMT_1   0x6B
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H 0x3B   // accel(6) + temp(2) + gyro(6) = 14 bytes, big-endian
#define REG_WHO_AM_I     0x75
#define WHO_AM_I_9250    0x71
#define WHO_AM_I_9255    0x73
#define WHO_AM_I_6500    0x70

#define ACC_SENS_8G      0.000244f  // g per LSB at +-8 g   (65536 / 16 g)
#define GYR_SENS_1000    0.030487f  // dps per LSB at +-1000 dps (65536 / 2000 dps)

#define I2C_TIMEOUT_US   3000u      // a stuck bus must not freeze the 100 Hz loop

#define CAL_MAX_RANGE_DPS  4.0f     // max-min spread allowed on any gyro axis while calibrating
#define CAL_MAX_BIAS_DPS   10.0f    // a larger mean offset means the sensor was moving or faulty

static i2c_inst_t *s_i2c;
static uint8_t s_addr;
static float bias_gx, bias_gy, bias_gz;

static bool write_reg (uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return (2 == i2c_write_timeout_us(s_i2c, s_addr, buf, 2, false, I2C_TIMEOUT_US));
}

static bool read_regs (uint8_t reg, uint8_t *dst, size_t n)
{
    if (1 != i2c_write_timeout_us(s_i2c, s_addr, &reg, 1, true, I2C_TIMEOUT_US))
    {
        return false;
    }
    return ((int)n == i2c_read_timeout_us(s_i2c, s_addr, dst, n, false, I2C_TIMEOUT_US));
}

uint8_t mpu9250_whoami(i2c_inst_t *i2c, uint8_t addr) {
    s_i2c = i2c;
    s_addr = addr;
    uint8_t v = 0;
    read_regs(REG_WHO_AM_I, &v, 1);
    return v;
}

bool mpu9250_init(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t who = mpu9250_whoami(i2c, addr);
    if (who != WHO_AM_I_9250 && who != WHO_AM_I_9255 && who != WHO_AM_I_6500) return false;

    write_reg(REG_PWR_MGMT_1, 0x80);       // reset
    sleep_ms(100);

    bool ok = write_reg(REG_PWR_MGMT_1, 0x01)     // wake, clock = gyro PLL
           && write_reg(REG_CONFIG, 0x03)          // DLPF ~44 Hz
           && write_reg(REG_GYRO_CONFIG, 0x10)     // +-1000 dps
           && write_reg(REG_ACCEL_CONFIG, 0x10)    // +-8 g
           && write_reg(REG_ACCEL_CONFIG2, 0x03);  // accel DLPF ~44 Hz
    sleep_ms(50);
    return ok;
}

bool mpu9250_read(imu_sample_t *s) {
    uint8_t b[14];
    if (!read_regs(REG_ACCEL_XOUT_H, b, 14)) return false;

    int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
    int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
    int16_t az = (int16_t)((b[4]  << 8) | b[5]);
    // b[6],b[7] = temperature, skipped
    int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    s->ax = ax * ACC_SENS_8G;
    s->ay = ay * ACC_SENS_8G;
    s->az = az * ACC_SENS_8G;
    s->gx = gx * GYR_SENS_1000 - bias_gx;
    s->gy = gy * GYR_SENS_1000 - bias_gy;
    s->gz = gz * GYR_SENS_1000 - bias_gz;
    return true;
}

mpu9250_cal_t mpu9250_calibrate_gyro (int n_samples, float *range_dps)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    float lo[3]  = { 1.0e9f, 1.0e9f, 1.0e9f };
    float hi[3]  = { -1.0e9f, -1.0e9f, -1.0e9f };
    float worst = 0.0f;
    int good = 0;
    imu_sample_t s = { 0 };

    /* Start from zero bias so the readings below are raw, not already corrected. */
    bias_gx = 0.0f;
    bias_gy = 0.0f;
    bias_gz = 0.0f;

    for (int i = 0; i < n_samples; i++)
    {
        if (mpu9250_read(&s))
        {
            float v[3] = { s.gx, s.gy, s.gz };

            for (int k = 0; k < 3; k++)
            {
                sum[k] += v[k];
                if (v[k] < lo[k]) { lo[k] = v[k]; }
                if (v[k] > hi[k]) { hi[k] = v[k]; }
            }
            good++;
        }
        sleep_ms(5);
    }

    if ((0 == good) || (good < (n_samples / 2)))
    {
        return MPU9250_CAL_BUS_ERROR;
    }

    for (int k = 0; k < 3; k++)
    {
        if ((hi[k] - lo[k]) > worst)
        {
            worst = hi[k] - lo[k];
        }
    }
    if (NULL != range_dps)
    {
        *range_dps = worst;
    }

    /* NOTE: thresholds are starting values, tune them from the printed range. */
    if ((worst > CAL_MAX_RANGE_DPS) ||
        (fabsf(sum[0] / (float)good) > CAL_MAX_BIAS_DPS) ||
        (fabsf(sum[1] / (float)good) > CAL_MAX_BIAS_DPS) ||
        (fabsf(sum[2] / (float)good) > CAL_MAX_BIAS_DPS))
    {
        return MPU9250_CAL_MOVING;
    }

    bias_gx = sum[0] / (float)good;
    bias_gy = sum[1] / (float)good;
    bias_gz = sum[2] / (float)good;
    return MPU9250_CAL_OK;
}
