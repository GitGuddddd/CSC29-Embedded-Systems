/**
 * @file    mpu9250.c
 * @brief   I2C driver for the MPU-6500 / MPU-9250 accelerometer and gyro.
 *
 * Register addresses come from the MPU-9250 register map (RM-MPU-9250A-00).
 * All bit operations are done on unsigned values.
 */

#include "mpu9250.h"

#include <math.h>
#include <stddef.h>

#include "pico/stdlib.h"

/* Register addresses. */
#define REG_PWR_MGMT_1     0x6Bu
#define REG_CONFIG         0x1Au
#define REG_GYRO_CONFIG    0x1Bu
#define REG_ACCEL_CONFIG   0x1Cu
#define REG_ACCEL_CONFIG2  0x1Du
#define REG_ACCEL_XOUT_H   0x3Bu   /* Accel, temperature and gyro follow. */
#define REG_WHO_AM_I       0x75u

/* Values of the WHO_AM_I register that are accepted. */
#define WHO_AM_I_9250      0x71u
#define WHO_AM_I_9255      0x73u
#define WHO_AM_I_6500      0x70u

/* Register values. */
#define PWR_RESET          0x80u
#define PWR_WAKE_GYRO_PLL  0x01u   /* Wake up, clock from the gyro PLL. */
#define CONFIG_DLPF_44HZ   0x03u
#define GYRO_FS_1000DPS    0x10u
#define ACCEL_FS_8G        0x10u
#define ACCEL_DLPF_44HZ    0x03u

/* Delays. */
#define RESET_DELAY_MS     100u
#define CONFIG_DELAY_MS    50u
#define CAL_GAP_MS         5u

/* Sensitivity. */
#define ACC_SENS_8G        0.000244f   /* g per LSB at +-8 g. */
#define GYR_SENS_1000      0.030487f   /* dps per LSB at +-1000 dps. */

/* A stuck bus must not freeze the 100 Hz loop. */
#define I2C_TIMEOUT_US     3000u

/* Layout of the 14-byte burst read, big-endian 16-bit words. */
#define SAMPLE_BYTES       14u
#define OFFSET_AX          0u
#define OFFSET_AY          2u
#define OFFSET_AZ          4u
#define OFFSET_GX          8u
#define OFFSET_GY          10u
#define OFFSET_GZ          12u
#define BITS_PER_BYTE      8u

/* Number of bytes of one register write: register address and value. */
#define WRITE_BYTES        2

#define AXIS_COUNT         3u
#define CONFIG_ENTRIES     5u

/* Gyro calibration limits. They are starting values, tune them with the
 * range that the calibration prints. */
#define CAL_MAX_RANGE_DPS  4.0f    /* Largest max-min spread on any axis. */
#define CAL_MAX_BIAS_DPS   10.0f   /* A larger mean offset means movement. */
#define CAL_START_LOW      1.0e9f
#define CAL_START_HIGH     (-1.0e9f)

typedef struct
{
    uint8_t reg;
    uint8_t value;
} config_entry_t;

static const config_entry_t g_config_table[CONFIG_ENTRIES] =
{
    { REG_PWR_MGMT_1,    PWR_WAKE_GYRO_PLL },
    { REG_CONFIG,        CONFIG_DLPF_44HZ },
    { REG_GYRO_CONFIG,   GYRO_FS_1000DPS },
    { REG_ACCEL_CONFIG,  ACCEL_FS_8G },
    { REG_ACCEL_CONFIG2, ACCEL_DLPF_44HZ }
};

static i2c_inst_t * gp_i2c    = NULL;
static uint8_t      g_addr    = 0u;
static float        g_bias_gx = 0.0f;
static float        g_bias_gy = 0.0f;
static float        g_bias_gz = 0.0f;

static bool    write_reg (uint8_t reg, uint8_t value);
static bool    read_regs (uint8_t reg, uint8_t * p_dst, uint32_t count);
static int16_t word_at (const uint8_t * p_raw, uint32_t offset);

/**
 * @brief  Reads the WHO_AM_I register.
 * @param  p_i2c  I2C instance of the sensor.
 * @param  addr   I2C address of the sensor.
 * @return The register value, or 0 if the read fails.
 */
uint8_t mpu9250_whoami (i2c_inst_t * p_i2c, uint8_t addr)
{
    uint8_t who = 0u;

    gp_i2c = p_i2c;
    g_addr = addr;

    if (!read_regs(REG_WHO_AM_I, &who, 1u))
    {
        who = 0u;
    }

    return who;
}

/**
 * @brief  Checks WHO_AM_I and configures the sensor.
 * @param  p_i2c  I2C instance of the sensor.
 * @param  addr   I2C address of the sensor.
 * @return true if the sensor answered and accepted the configuration.
 */
bool mpu9250_init (i2c_inst_t * p_i2c, uint8_t addr)
{
    uint8_t  who  = mpu9250_whoami(p_i2c, addr);
    bool     b_ok = false;
    uint32_t idx;

    if ((WHO_AM_I_9250 == who) || (WHO_AM_I_9255 == who) ||
        (WHO_AM_I_6500 == who))
    {
        /* The result of the reset write is not used: the sensor restarts
         * and the writes below show whether it is working. */
        write_reg(REG_PWR_MGMT_1, PWR_RESET);
        sleep_ms(RESET_DELAY_MS);

        b_ok = true;
        for (idx = 0u; b_ok && (idx < CONFIG_ENTRIES); idx++)
        {
            b_ok = write_reg(g_config_table[idx].reg,
                             g_config_table[idx].value);
        }
        sleep_ms(CONFIG_DELAY_MS);
    }

    return b_ok;
}

/**
 * @brief  Reads one sample from the sensor.
 * @param  p_sample  Output sample, written only if the read succeeds.
 * @return true if the bus read succeeded.
 */
bool mpu9250_read (imu_sample_t * p_sample)
{
    uint8_t raw[SAMPLE_BYTES];
    bool    b_ok = read_regs(REG_ACCEL_XOUT_H, raw, SAMPLE_BYTES);

    if (b_ok)
    {
        /* The two temperature bytes are skipped. */
        p_sample->accel_x = word_at(raw, OFFSET_AX) * ACC_SENS_8G;
        p_sample->accel_y = word_at(raw, OFFSET_AY) * ACC_SENS_8G;
        p_sample->accel_z = word_at(raw, OFFSET_AZ) * ACC_SENS_8G;
        p_sample->gyro_x  = (word_at(raw, OFFSET_GX) * GYR_SENS_1000) -
                            g_bias_gx;
        p_sample->gyro_y  = (word_at(raw, OFFSET_GY) * GYR_SENS_1000) -
                            g_bias_gy;
        p_sample->gyro_z  = (word_at(raw, OFFSET_GZ) * GYR_SENS_1000) -
                            g_bias_gz;
    }

    return b_ok;
}

/**
 * @brief  Measures the gyro zero-rate bias while the sensor is still.
 * @param  num_samples   Number of readings, 5 ms apart.
 * @param  p_range_dps   Receives the largest max-min spread, may be NULL.
 * @return Result of the calibration. The bias is changed only on success.
 */
mpu9250_cal_t mpu9250_calibrate_gyro (uint32_t  num_samples,
                                      float   * p_range_dps)
{
    mpu9250_cal_t result = MPU9250_CAL_OK;
    imu_sample_t  sample = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float         sum[AXIS_COUNT]  = { 0.0f, 0.0f, 0.0f };
    float         low[AXIS_COUNT]  = { CAL_START_LOW, CAL_START_LOW,
                                       CAL_START_LOW };
    float         high[AXIS_COUNT] = { CAL_START_HIGH, CAL_START_HIGH,
                                       CAL_START_HIGH };
    float         rate[AXIS_COUNT];
    float         mean[AXIS_COUNT];
    float         worst = 0.0f;
    float         good_count;
    uint32_t      good = 0u;
    uint32_t      idx;
    uint32_t      axis;
    bool          b_moving;

    /* Start from zero bias so the readings below are raw, not corrected. */
    g_bias_gx = 0.0f;
    g_bias_gy = 0.0f;
    g_bias_gz = 0.0f;

    for (idx = 0u; idx < num_samples; idx++)
    {
        if (mpu9250_read(&sample))
        {
            rate[0] = sample.gyro_x;
            rate[1] = sample.gyro_y;
            rate[2] = sample.gyro_z;

            for (axis = 0u; axis < AXIS_COUNT; axis++)
            {
                sum[axis] += rate[axis];
                if (rate[axis] < low[axis])
                {
                    low[axis] = rate[axis];
                }
                if (rate[axis] > high[axis])
                {
                    high[axis] = rate[axis];
                }
            }
            good++;
        }
        sleep_ms(CAL_GAP_MS);
    }

    if ((0u == good) || (good < (num_samples / 2u)))
    {
        result = MPU9250_CAL_BUS_ERROR;
    }
    else
    {
        /* Cast: good is at most num_samples, so a float holds it exactly. */
        good_count = (float)good;
        b_moving   = false;

        for (axis = 0u; axis < AXIS_COUNT; axis++)
        {
            mean[axis] = sum[axis] / good_count;
            if ((high[axis] - low[axis]) > worst)
            {
                worst = high[axis] - low[axis];
            }
            if (fabsf(mean[axis]) > CAL_MAX_BIAS_DPS)
            {
                b_moving = true;
            }
        }

        if (NULL != p_range_dps)
        {
            *p_range_dps = worst;
        }

        if ((worst > CAL_MAX_RANGE_DPS) || b_moving)
        {
            result = MPU9250_CAL_MOVING;
        }
        else
        {
            g_bias_gx = mean[0];
            g_bias_gy = mean[1];
            g_bias_gz = mean[2];
        }
    }

    return result;
}

/**
 * @brief  Writes one sensor register.
 * @param  reg    Register address.
 * @param  value  Value to write.
 * @return true if both bytes were accepted.
 */
static bool write_reg (uint8_t reg, uint8_t value)
{
    uint8_t buf[WRITE_BYTES];
    int32_t written;

    buf[0] = reg;
    buf[1] = value;

    written = i2c_write_timeout_us(gp_i2c, g_addr, buf, WRITE_BYTES, false,
                                   I2C_TIMEOUT_US);

    return (WRITE_BYTES == written);
}

/**
 * @brief  Reads a block of consecutive registers.
 * @param  reg    Address of the first register.
 * @param  p_dst  Output buffer.
 * @param  count  Number of bytes to read.
 * @return true if all bytes were read.
 */
static bool read_regs (uint8_t reg, uint8_t * p_dst, uint32_t count)
{
    bool    b_ok = false;
    int32_t got;

    if (1 == i2c_write_timeout_us(gp_i2c, g_addr, &reg, 1u, true,
                                  I2C_TIMEOUT_US))
    {
        got = i2c_read_timeout_us(gp_i2c, g_addr, p_dst, count, false,
                                  I2C_TIMEOUT_US);
        if (got >= 0)
        {
            /* Cast: got is not negative here, so it keeps its value. */
            b_ok = (count == (uint32_t)got);
        }
    }

    return b_ok;
}

/**
 * @brief  Joins two bytes of a big-endian register pair into a signed value.
 * @param  p_raw   Raw bytes read from the sensor.
 * @param  offset  Index of the high byte.
 * @return The 16-bit two's complement value.
 */
static int16_t word_at (const uint8_t * p_raw, uint32_t offset)
{
    uint32_t high = p_raw[offset];
    uint32_t low  = p_raw[offset + 1u];
    uint32_t word = (high << BITS_PER_BYTE) | low;

    /* Cast: the sensor sends a 16-bit two's complement number, so the 16
     * bits in word are reinterpreted as a signed value. */
    return (int16_t)word;
}

/* end of mpu9250.c */

