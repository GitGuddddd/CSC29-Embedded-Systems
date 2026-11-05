#include "unity.h"
#include "drivers/Mpu9250/Mpu9250.h"
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
    return UNITY_END();
}
