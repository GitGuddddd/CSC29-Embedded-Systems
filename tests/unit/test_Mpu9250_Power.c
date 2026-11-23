#include "unity.h"
#include "drivers/Mpu9250/Mpu9250.h"
#include "drivers/Mpu9250/Mpu9250_Cfg.h"
#include "hal/Hal_I2c.h"
#include <string.h>

static uint8_t s_registers[256];
static bool s_read_ok;
static bool s_write_ok;
static bool s_ignore_write;
static uint32_t s_writes;
static uint32_t s_reads;
static const Mpu9250_Config_t s_cfg = { .i2c_port = 0U, .dev_addr = 0x68U };

bool Hal_I2c_ReadRegs(uint8_t port, uint8_t addr, uint8_t reg, uint8_t * const p_dst, uint32_t len)
{
    TEST_ASSERT_EQUAL_UINT8(0U, port);
    TEST_ASSERT_EQUAL_HEX8(0x68U, addr);
    ++s_reads;
    if (!s_read_ok) { return false; }
    memcpy(p_dst, &s_registers[reg], len);
    return true;
}

bool Hal_I2c_WriteReg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t value)
{
    TEST_ASSERT_EQUAL_UINT8(0U, port);
    TEST_ASSERT_EQUAL_HEX8(0x68U, addr);
    ++s_writes;
    if (!s_write_ok) { return false; }
    if (!s_ignore_write) { s_registers[reg] = value; }
    return true;
}

void setUp(void)
{
    memset(s_registers, 0, sizeof(s_registers));
    s_registers[0x6BU] = 0x01U;
    s_read_ok = true;
    s_write_ok = true;
    s_ignore_write = false;
    s_writes = 0U;
    s_reads = 0U;
}

void tearDown(void) { }

void test_RealSleepBit_PreservesClockAndConfiguredRanges(void)
{
    s_registers[0x1BU] = 0x10U;
    s_registers[0x1CU] = 0x10U;
    TEST_ASSERT_TRUE(Mpu9250_SetSleep(&s_cfg, true));
    TEST_ASSERT_EQUAL_HEX8(0x41U, s_registers[0x6BU]);
    TEST_ASSERT_TRUE(Mpu9250_SetSleep(&s_cfg, false));
    TEST_ASSERT_EQUAL_HEX8(0x01U, s_registers[0x6BU]);
    TEST_ASSERT_EQUAL_HEX8(0x10U, s_registers[0x1BU]);
    TEST_ASSERT_EQUAL_HEX8(0x10U, s_registers[0x1CU]);
}

void test_Wake_ExitsCycleAndGyroStandby(void)
{
    s_registers[0x6BU] = 0x79U;
    TEST_ASSERT_TRUE(Mpu9250_SetSleep(&s_cfg, false));
    TEST_ASSERT_EQUAL_HEX8(0x09U, s_registers[0x6BU]);
}

void test_BusErrors_AndMismatchedReadback_DoNotClaimSleep(void)
{
    s_read_ok = false;
    TEST_ASSERT_FALSE(Mpu9250_SetSleep(&s_cfg, true));
    TEST_ASSERT_EQUAL_UINT32(0U, s_writes);
    s_read_ok = true;
    s_write_ok = false;
    TEST_ASSERT_FALSE(Mpu9250_SetSleep(&s_cfg, true));
    s_write_ok = true;
    s_ignore_write = true;
    TEST_ASSERT_FALSE(Mpu9250_SetSleep(&s_cfg, true));
}

void test_NullOrResettingDevice_IsRejected(void)
{
    TEST_ASSERT_FALSE(Mpu9250_SetSleep(NULL, true));
    s_registers[0x6BU] = 0x80U;
    TEST_ASSERT_FALSE(Mpu9250_SetSleep(&s_cfg, true));
    TEST_ASSERT_EQUAL_UINT32(0U, s_writes);
}

void test_InitializationStepsPerformOneRegisterActionWithUnchangedValues(void)
{
    s_registers[0x75U] = 0x70U; /* MPU-6500. */
    static const uint8_t registers[] = {0x6BU, 0x6BU, 0x1AU, 0x1BU, 0x1CU, 0x1DU};
    static const uint8_t values[] = {0x80U, 0x01U, 0x03U, 0x10U, 0x10U, 0x03U};
    uint32_t completed = 0U;
    TEST_ASSERT_TRUE(Mpu9250_RunInitStep(&s_cfg, MPU9250_INIT_IDENTITY, 100U, &completed));
    TEST_ASSERT_EQUAL_UINT32(100U, completed);
    TEST_ASSERT_EQUAL_UINT32(1U, s_reads);
    TEST_ASSERT_EQUAL_UINT32(0U, s_writes);
    for (uint8_t step = MPU9250_INIT_RESET; step < MPU9250_INIT_DONE; ++step)
    {
        uint32_t before = s_reads + s_writes;
        TEST_ASSERT_TRUE(Mpu9250_RunInitStep(&s_cfg, (Mpu9250_InitStep_t)step, 200U, &completed));
        TEST_ASSERT_EQUAL_UINT32(before + 1U, s_reads + s_writes);
        TEST_ASSERT_EQUAL_HEX8(values[step - 1U], s_registers[registers[step - 1U]]);
    }
}

void test_BadIdentityOrInitializationStepDoesNotProceed(void)
{
    uint32_t completed;
    s_registers[0x75U] = 0xFFU;
    TEST_ASSERT_FALSE(Mpu9250_RunInitStep(&s_cfg, MPU9250_INIT_IDENTITY, 10U, &completed));
    TEST_ASSERT_EQUAL_UINT32(0U, s_writes);
    TEST_ASSERT_FALSE(Mpu9250_RunInitStep(NULL, MPU9250_INIT_RESET, 20U, &completed));
    TEST_ASSERT_FALSE(Mpu9250_RunInitStep(&s_cfg, MPU9250_INIT_DONE, 20U, &completed));
    TEST_ASSERT_FALSE(Mpu9250_RunInitStep(&s_cfg, (Mpu9250_InitStep_t)255, 20U, &completed));
    TEST_ASSERT_EQUAL_UINT32(0U, s_writes);
}

void test_TimedSleepRetainsVerifiedHardwareBehavior(void)
{
    uint32_t completed;
    TEST_ASSERT_TRUE(Mpu9250_SetSleepTimed(&s_cfg, true, 123U, &completed));
    TEST_ASSERT_EQUAL_UINT32(123U, completed);
    TEST_ASSERT_EQUAL_HEX8(0x41U, s_registers[0x6BU]);
    s_ignore_write = true;
    TEST_ASSERT_FALSE(Mpu9250_SetSleepTimed(&s_cfg, false, 234U, &completed));
    TEST_ASSERT_EQUAL_UINT32(234U, completed);
}

/* Feeds n identical samples; returns the status of the last call. */
static Mpu9250_GyroCalStatus_t Feed_Cal(Mpu9250_GyroCal_t * const p_cal,
                                        float gx, float gy, float gz,
                                        float az, uint32_t n)
{
    Imu_Sample_t s = { 0 };
    Mpu9250_GyroCalStatus_t status = MPU9250_GYRO_CAL_COLLECTING;

    s.az = az;
    for (uint32_t i = 0U; i < n; i++)
    {
        /* Small alternating noise, well inside the 4 dps spread limit. */
        float noise = ((i % 2U) == 0U) ? 0.4f : -0.4f;
        s.gx = gx + noise;
        s.gy = gy - noise;
        s.gz = gz + noise;
        status = Mpu9250_GyroCalStep(p_cal, &s);
    }
    return status;
}

void test_GyroCal_StillWindow_AppliesOffset(void)
{
    Mpu9250_GyroCal_t cal;
    float ox = 0.0f;
    float oy = 0.0f;
    float oz = 0.0f;

    Mpu9250_GyroCalInit(&cal);
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_COLLECTING,
                      Feed_Cal(&cal, 3.0f, -1.5f, 0.8f, 1.0f,
                               MPU9250_CFG_GYRO_CAL_SAMPLES - 1U));
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_APPLIED,
                      Feed_Cal(&cal, 3.0f, -1.5f, 0.8f, 1.0f, 1U));
    Mpu9250_GetGyroOffset(&ox, &oy, &oz);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, ox);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -1.5f, oy);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.8f, oz);

    /* Calibrated once: later samples change nothing. */
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_DONE,
                      Feed_Cal(&cal, 9.0f, 9.0f, 9.0f, 1.0f, 5U));
    Mpu9250_GetGyroOffset(&ox, &oy, &oz);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, ox);
}

void test_GyroCal_OffsetIsSubtractedFromReadSample(void)
{
    Mpu9250_GyroCal_t cal;
    Imu_Sample_t s = { 0 };

    Mpu9250_GyroCalInit(&cal);
    (void)Feed_Cal(&cal, 3.0f, 0.0f, 0.0f, 1.0f,
                   MPU9250_CFG_GYRO_CAL_SAMPLES);

    /* Raw gyro X = 98 counts = about 2.99 dps at +-1000 dps. */
    s_registers[0x43U] = 0x00U;
    s_registers[0x44U] = 98U;
    TEST_ASSERT_TRUE(Mpu9250_ReadSample(&s_cfg, &s));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, s.gx);
}

void test_GyroCal_RotatingWindow_IsRejected(void)
{
    Mpu9250_GyroCal_t cal;
    Imu_Sample_t s = { 0 };
    Mpu9250_GyroCalStatus_t status = MPU9250_GYRO_CAL_COLLECTING;
    float ox = 1.0f;
    float oy = 1.0f;
    float oz = 1.0f;

    Mpu9250_GyroCalInit(&cal);
    s.az = 1.0f;
    for (uint32_t i = 0U; i < MPU9250_CFG_GYRO_CAL_SAMPLES; i++)
    {
        /* Cast: small loop index, exact as a float. Spread of 9.9 dps. */
        s.gx = (float)i * 0.1f;
        status = Mpu9250_GyroCalStep(&cal, &s);
    }
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_REJECTED, status);
    Mpu9250_GetGyroOffset(&ox, &oy, &oz);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ox);
}

void test_GyroCal_SteadyFastTurn_IsRejected(void)
{
    Mpu9250_GyroCal_t cal;
    float ox = 1.0f;
    float oy = 1.0f;
    float oz = 1.0f;

    /* A steady 15 dps turn: small spread, but not a believable offset. */
    Mpu9250_GyroCalInit(&cal);
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_REJECTED,
                      Feed_Cal(&cal, 15.0f, 0.0f, 0.0f, 1.0f,
                               MPU9250_CFG_GYRO_CAL_SAMPLES));
    Mpu9250_GetGyroOffset(&ox, &oy, &oz);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ox);
}

void test_GyroCal_TiltDuringWindow_IsRejectedAndRetries(void)
{
    Mpu9250_GyroCal_t cal;
    float ox = 0.0f;
    float oy = 0.0f;
    float oz = 0.0f;

    Mpu9250_GyroCalInit(&cal);
    (void)Feed_Cal(&cal, 2.0f, 0.0f, 0.0f, 1.0f, 10U);
    /* Gravity shifts by 0.1 g: the glasses were tilted. */
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_REJECTED,
                      Feed_Cal(&cal, 2.0f, 0.0f, 0.0f, 0.9f, 1U));

    /* Still again: a fresh window succeeds. */
    TEST_ASSERT_EQUAL(MPU9250_GYRO_CAL_APPLIED,
                      Feed_Cal(&cal, 2.0f, 0.0f, 0.0f, 0.9f,
                               MPU9250_CFG_GYRO_CAL_SAMPLES));
    Mpu9250_GetGyroOffset(&ox, &oy, &oz);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, ox);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_RealSleepBit_PreservesClockAndConfiguredRanges);
    RUN_TEST(test_Wake_ExitsCycleAndGyroStandby);
    RUN_TEST(test_BusErrors_AndMismatchedReadback_DoNotClaimSleep);
    RUN_TEST(test_NullOrResettingDevice_IsRejected);
    RUN_TEST(test_InitializationStepsPerformOneRegisterActionWithUnchangedValues);
    RUN_TEST(test_BadIdentityOrInitializationStepDoesNotProceed);
    RUN_TEST(test_TimedSleepRetainsVerifiedHardwareBehavior);
    RUN_TEST(test_GyroCal_StillWindow_AppliesOffset);
    RUN_TEST(test_GyroCal_OffsetIsSubtractedFromReadSample);
    RUN_TEST(test_GyroCal_RotatingWindow_IsRejected);
    RUN_TEST(test_GyroCal_SteadyFastTurn_IsRejected);
    RUN_TEST(test_GyroCal_TiltDuringWindow_IsRejectedAndRetries);
    return UNITY_END();
}
