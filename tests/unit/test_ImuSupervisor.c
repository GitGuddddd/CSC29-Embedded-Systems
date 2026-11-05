#include "unity.h"
#include "services/ImuSupervisor/ImuSupervisor.h"
#include "services/ImuSupervisor/ImuSupervisor_Cfg.h"
#include "hal/Hal_I2c.h"
#include <math.h>

static ImuSupervisor_t s_imu;
static const Mpu9250_Config_t s_cfg = { .i2c_port = 0U, .dev_addr = 0x68U };
static bool s_init_ok;
static bool s_sleep_ok;
static bool s_read_ok;
static bool s_nan;
static uint32_t s_inits;
static uint32_t s_reads;
static uint32_t s_sleeps;
static uint32_t s_wakes;
static uint32_t s_bus_resets;
static uint32_t s_now;

bool Mpu9250_Init(const Mpu9250_Config_t * const p_cfg)
{
    TEST_ASSERT_EQUAL_PTR(&s_cfg, p_cfg);
    ++s_inits;
    return s_init_ok;
}

bool Mpu9250_SetSleep(const Mpu9250_Config_t * const p_cfg, bool enabled)
{
    TEST_ASSERT_EQUAL_PTR(&s_cfg, p_cfg);
    if (enabled) { ++s_sleeps; } else { ++s_wakes; }
    return s_sleep_ok;
}

bool Mpu9250_ReadSample(const Mpu9250_Config_t * const p_cfg, Imu_Sample_t * const p_sample)
{
    TEST_ASSERT_EQUAL_PTR(&s_cfg, p_cfg);
    ++s_reads;
    *p_sample = (Imu_Sample_t){ .az = s_nan ? NAN : 1.0f };
    return s_read_ok;
}

void Hal_I2c_BusReset(uint8_t sda, uint8_t scl)
{
    TEST_ASSERT_EQUAL_UINT8(4U, sda);
    TEST_ASSERT_EQUAL_UINT8(5U, scl);
    ++s_bus_resets;
}

void Hal_I2c_Init(uint8_t port, uint8_t sda, uint8_t scl, uint32_t hz)
{
    TEST_ASSERT_EQUAL_UINT8(0U, port);
    TEST_ASSERT_EQUAL_UINT8(4U, sda);
    TEST_ASSERT_EQUAL_UINT8(5U, scl);
    TEST_ASSERT_EQUAL_UINT32(400000U, hz);
}

static void Tick(uint32_t ms, bool sleep_requested)
{
    s_now += ms;
    ImuSupervisor_Step(&s_imu, sleep_requested, s_now);
}

static void Ready(void)
{
    Tick(100U, false);
    Tick(10U, false);
    Tick(10U, false);
    TEST_ASSERT_EQUAL(IMU_SUPERVISOR_RUNNING, s_imu.state);
    TEST_ASSERT_TRUE(s_imu.sample_valid);
}

void setUp(void)
{
    s_init_ok = true;
    s_sleep_ok = true;
    s_read_ok = true;
    s_nan = false;
    s_inits = 0U;
    s_reads = 0U;
    s_sleeps = 0U;
    s_wakes = 0U;
    s_bus_resets = 0U;
    s_now = 1000U;
    ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
}

void tearDown(void) { }

void test_Startup_WaitsForSettlingAndThreeFreshSamples(void)
{
    Tick(90U, false);
    TEST_ASSERT_EQUAL_UINT32(0U, s_reads);
    Tick(10U, false);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
    Tick(10U, false);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
    Tick(10U, false);
    TEST_ASSERT_TRUE(s_imu.sample_valid);
    TEST_ASSERT_EQUAL_UINT32(s_now, s_imu.sample.timestamp_ms);
}

void test_Sleep_PerformsNoPolling_AndWakeDoesNotResetCalibration(void)
{
    Ready();
    Tick(10U, true);
    TEST_ASSERT_EQUAL(IMU_SUPERVISOR_ASLEEP, s_imu.state);
    uint32_t reads = s_reads;
    for (uint32_t i = 0U; i < 100U; ++i) { Tick(10U, true); }
    TEST_ASSERT_EQUAL_UINT32(reads, s_reads);
    TEST_ASSERT_EQUAL_UINT32(1U, s_sleeps);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
    Tick(10U, false);
    TEST_ASSERT_EQUAL(IMU_SUPERVISOR_STARTING, s_imu.state);
    TEST_ASSERT_EQUAL_UINT32(1U, s_wakes);
    TEST_ASSERT_EQUAL_UINT32(1U, s_inits);
    Ready();
}

void test_ThreeReadFailures_EnterFault_AndRecoveryNeedsFreshSamples(void)
{
    Ready();
    s_read_ok = false;
    Tick(10U, false);
    Tick(10U, false);
    TEST_ASSERT_FALSE(s_imu.fault);
    Tick(10U, false);
    TEST_ASSERT_TRUE(s_imu.fault);
    Tick(990U, false);
    TEST_ASSERT_EQUAL_UINT32(0U, s_bus_resets);
    s_read_ok = true;
    Tick(10U, false);
    TEST_ASSERT_EQUAL_UINT32(1U, s_bus_resets);
    TEST_ASSERT_TRUE(s_imu.fault);
    Ready();
    TEST_ASSERT_FALSE(s_imu.fault);
}

void test_IntermittentFailures_DoNotAccumulateAsConsecutive(void)
{
    Ready();
    for (uint32_t i = 0U; i < 10U; ++i)
    {
        s_read_ok = false;
        Tick(10U, false);
        s_read_ok = true;
        Tick(10U, false);
    }
    TEST_ASSERT_FALSE(s_imu.fault);
}

void test_FailedSleepOrWake_IsReportedAsFault(void)
{
    Ready();
    s_sleep_ok = false;
    Tick(10U, true);
    TEST_ASSERT_EQUAL(IMU_SUPERVISOR_FAULT, s_imu.state);
    TEST_ASSERT_TRUE(s_imu.fault);
    s_sleep_ok = true;
    Tick(1000U, false);
    Ready();
    Tick(10U, true);
    s_sleep_ok = false;
    Tick(10U, false);
    TEST_ASSERT_TRUE(s_imu.fault);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
}

void test_InitFailure_AndCorruptSamples_NeverReportReady(void)
{
    s_init_ok = false;
    ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
    TEST_ASSERT_TRUE(s_imu.fault);
    s_init_ok = true;
    Tick(1000U, false);
    s_nan = true;
    Tick(100U, false);
    Tick(10U, false);
    Tick(10U, false);
    TEST_ASSERT_TRUE(s_imu.fault);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Startup_WaitsForSettlingAndThreeFreshSamples);
    RUN_TEST(test_Sleep_PerformsNoPolling_AndWakeDoesNotResetCalibration);
    RUN_TEST(test_ThreeReadFailures_EnterFault_AndRecoveryNeedsFreshSamples);
    RUN_TEST(test_IntermittentFailures_DoNotAccumulateAsConsecutive);
    RUN_TEST(test_FailedSleepOrWake_IsReportedAsFault);
    RUN_TEST(test_InitFailure_AndCorruptSamples_NeverReportReady);
    return UNITY_END();
}
