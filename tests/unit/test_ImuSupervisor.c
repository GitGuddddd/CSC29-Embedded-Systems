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
static uint32_t s_actions;
static uint32_t s_io_cost_ms;
static int s_failed_step;

bool Mpu9250_Init(const Mpu9250_Config_t * const p_cfg)
{
    (void)p_cfg;
    TEST_FAIL_MESSAGE("Cyclic supervisor must not call synchronous IMU initialization");
    return false;
}

bool Mpu9250_RunInitStep(const Mpu9250_Config_t * const p_cfg, Mpu9250_InitStep_t step,
                         uint32_t request_ms, uint32_t * const p_completed_ms)
{
    TEST_ASSERT_EQUAL_PTR(&s_cfg, p_cfg);
    TEST_ASSERT_EQUAL_UINT32(s_now, request_ms);
    if (step == MPU9250_INIT_IDENTITY) ++s_inits;
    ++s_actions;
    s_now += s_io_cost_ms;
    *p_completed_ms = s_now;
    return s_init_ok && ((int)step != s_failed_step);
}

bool Mpu9250_SetSleep(const Mpu9250_Config_t * const p_cfg, bool enabled)
{
    TEST_ASSERT_EQUAL_PTR(&s_cfg, p_cfg);
    if (enabled) { ++s_sleeps; } else { ++s_wakes; }
    return s_sleep_ok;
}

bool Mpu9250_SetSleepTimed(const Mpu9250_Config_t * const p_cfg, bool enabled,
                           uint32_t request_ms, uint32_t * const p_completed_ms)
{
    TEST_ASSERT_EQUAL_UINT32(s_now, request_ms);
    bool ok = Mpu9250_SetSleep(p_cfg, enabled);
    s_now += s_io_cost_ms;
    *p_completed_ms = s_now;
    return ok;
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
    uint32_t before = s_actions + s_reads + s_sleeps + s_wakes;
    ImuSupervisor_Step(&s_imu, sleep_requested, s_now);
    TEST_ASSERT_TRUE((s_actions + s_reads + s_sleeps + s_wakes - before) <= 1U);
}

static void Configure(void)
{
    for (uint32_t i = 0U; (i < 30U) && (s_imu.init_step != MPU9250_INIT_DONE); ++i)
    {
        Tick(10U, false);
    }
    TEST_ASSERT_EQUAL_UINT8(MPU9250_INIT_DONE, s_imu.init_step);
}

static void Ready(void)
{
    Configure();
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
    s_actions = 0U;
    s_io_cost_ms = 0U;
    s_failed_step = -1;
    ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
}

void tearDown(void) { }

void test_Startup_WaitsForSettlingAndThreeFreshSamples(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, s_actions); /* Init is software-only. */
    Tick(10U, false); /* Identity. */
    Tick(10U, false); /* Reset. */
    TEST_ASSERT_EQUAL_UINT32(2U, s_actions);
    Tick(49U, false);
    TEST_ASSERT_EQUAL_UINT32(2U, s_actions); /* No wake before reset settling. */
    Tick(1U, false);
    TEST_ASSERT_EQUAL_UINT32(3U, s_actions);
    Configure();
    TEST_ASSERT_EQUAL_UINT32(7U, s_actions);
    Tick(99U, false);
    TEST_ASSERT_EQUAL_UINT32(0U, s_reads);
    Tick(1U, false);
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
    TEST_ASSERT_FALSE(s_imu.fault); /* Not yet probed; configuration is asynchronous. */
    Tick(10U, false);
    TEST_ASSERT_TRUE(s_imu.fault);
    s_init_ok = true;
    Tick(1000U, false);
    Configure();
    s_nan = true;
    Tick(100U, false);
    Tick(10U, false);
    Tick(10U, false);
    TEST_ASSERT_TRUE(s_imu.fault);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
}

void test_ResetAndWakeSettlingStartAfterIoCompletion(void)
{
    s_io_cost_ms = 5U;
    Tick(10U, false);
    Tick(10U, false);
    TEST_ASSERT_EQUAL_UINT32(s_now, s_imu.transition_ms);
    s_io_cost_ms = 0U;
    uint32_t actions = s_actions;
    Tick(49U, false);
    TEST_ASSERT_EQUAL_UINT32(actions, s_actions);
    Tick(1U, false);
    TEST_ASSERT_EQUAL_UINT32(actions + 1U, s_actions);
    Ready();
    Tick(10U, true);
    s_io_cost_ms = 7U;
    Tick(10U, false);
    TEST_ASSERT_EQUAL_UINT32(s_now, s_imu.transition_ms);
    s_io_cost_ms = 0U;
    uint32_t reads = s_reads;
    Tick(99U, false);
    TEST_ASSERT_EQUAL_UINT32(reads, s_reads);
    Tick(1U, false);
    TEST_ASSERT_EQUAL_UINT32(reads + 1U, s_reads);
}

void test_ResetWaitWrapsAndSleepRequestPausesConfiguration(void)
{
    s_now = UINT32_MAX - 30U;
    ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
    Tick(10U, false);
    Tick(10U, false);
    Tick(49U, false);
    TEST_ASSERT_EQUAL_UINT8(MPU9250_INIT_WAKE, s_imu.init_step);
    Tick(1U, false);
    TEST_ASSERT_EQUAL_UINT8(MPU9250_INIT_DLPF, s_imu.init_step);
    uint32_t actions = s_actions;
    Tick(1000U, true);
    TEST_ASSERT_EQUAL_UINT32(actions, s_actions);
    TEST_ASSERT_NOT_EQUAL(IMU_SUPERVISOR_ASLEEP, s_imu.state);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
    Tick(1U, false);
    TEST_ASSERT_EQUAL_UINT32(actions + 1U, s_actions);
}

void test_EveryConfigurationFailureStopsAndDefersRetry(void)
{
    for (int failed = MPU9250_INIT_IDENTITY; failed < MPU9250_INIT_DONE; ++failed)
    {
        s_failed_step = failed;
        ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
        Configure();
        TEST_ASSERT_TRUE(s_imu.fault);
        TEST_ASSERT_EQUAL(IMU_SUPERVISOR_FAULT, s_imu.state);
        uint32_t actions = s_actions;
        uint32_t resets = s_bus_resets;
        Tick(999U, false);
        TEST_ASSERT_EQUAL_UINT32(actions, s_actions);
        TEST_ASSERT_EQUAL_UINT32(resets, s_bus_resets);
        Tick(1U, false);
        TEST_ASSERT_EQUAL_UINT32(resets + 1U, s_bus_resets);
        TEST_ASSERT_EQUAL_UINT32(actions, s_actions); /* Bus clear, no register action yet. */
    }
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
    RUN_TEST(test_ResetAndWakeSettlingStartAfterIoCompletion);
    RUN_TEST(test_ResetWaitWrapsAndSleepRequestPausesConfiguration);
    RUN_TEST(test_EveryConfigurationFailureStopsAndDefersRetry);
    return UNITY_END();
}
