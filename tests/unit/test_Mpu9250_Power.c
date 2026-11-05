#include "unity.h"
#include "drivers/Mpu9250/Mpu9250.h"
#include "hal/Hal_I2c.h"
#include <string.h>

static uint8_t s_registers[256];
static bool s_read_ok;
static bool s_write_ok;
static bool s_ignore_write;
static uint32_t s_writes;
static const Mpu9250_Config_t s_cfg = { .i2c_port = 0U, .dev_addr = 0x68U };

bool Hal_I2c_ReadRegs(uint8_t port, uint8_t addr, uint8_t reg, uint8_t * const p_dst, uint32_t len)
{
    TEST_ASSERT_EQUAL_UINT8(0U, port);
    TEST_ASSERT_EQUAL_HEX8(0x68U, addr);
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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_RealSleepBit_PreservesClockAndConfiguredRanges);
    RUN_TEST(test_Wake_ExitsCycleAndGyroStandby);
    RUN_TEST(test_BusErrors_AndMismatchedReadback_DoNotClaimSleep);
    RUN_TEST(test_NullOrResettingDevice_IsRejected);
    return UNITY_END();
}
