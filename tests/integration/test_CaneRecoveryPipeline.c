#include "unity.h"
#include "app/CaneMode/CaneMode.h"
#include "app/FallDetection/FallDetection.h"
#include "app/AlertManager/AlertManager.h"
#include "drivers/TouchSensor/TouchSensor.h"
#include "services/ImuSupervisor/ImuSupervisor.h"
#include "services/BleComm/BleComm.h"
#include "hal/Hal_I2c.h"
#include <string.h>

/* Real policy, debounce, fall algorithm, supervisor, audio and packet codec;
 * only sensor/bus I/O is substituted. Advance the same 10 ms cycle as the app.
 */
static const Mpu9250_Config_t s_cfg = { .dev_addr = 0x68U };
static CaneMode_t s_mode;
static CaneMode_Output_t s_out;
static ImuSupervisor_t s_imu;
static Alert_Output_t s_audio;
static Ble_AdvancedThreatPacket_t s_decoded;
static Imu_Sample_t s_sensor;
static bool s_touch;
static bool s_sleep_ok;
static uint32_t s_reads;
static uint32_t s_now;
static uint8_t s_incidents;

bool Mpu9250_Init(const Mpu9250_Config_t * const p_cfg)
{
    (void)p_cfg;
    TEST_FAIL_MESSAGE("Perception pipeline must not call synchronous IMU init");
    return false;
}

bool Mpu9250_RunInitStep(const Mpu9250_Config_t * const p_cfg, Mpu9250_InitStep_t step,
                         uint32_t request_ms, uint32_t * const p_completed_ms)
{
    (void)p_cfg;
    (void)step;
    *p_completed_ms = request_ms;
    return true;
}

bool Mpu9250_SetSleep(const Mpu9250_Config_t * const p_cfg, bool enabled)
{
    (void)p_cfg;
    (void)enabled;
    return s_sleep_ok;
}

bool Mpu9250_SetSleepTimed(const Mpu9250_Config_t * const p_cfg, bool enabled,
                           uint32_t request_ms, uint32_t * const p_completed_ms)
{
    *p_completed_ms = request_ms;
    return Mpu9250_SetSleep(p_cfg, enabled);
}

bool Mpu9250_ReadSample(const Mpu9250_Config_t * const p_cfg, Imu_Sample_t * const p_sample)
{
    (void)p_cfg;
    ++s_reads;
    *p_sample = s_sensor;
    return true;
}

void Hal_I2c_BusReset(uint8_t sda, uint8_t scl) { (void)sda; (void)scl; }
void Hal_I2c_Init(uint8_t port, uint8_t sda, uint8_t scl, uint32_t hz)
{
    (void)port; (void)sda; (void)scl; (void)hz;
}

static void Cycle(void)
{
    s_now += 10U;
    Grip_State_t grip;
    (void)TouchSensor_Update(NULL, s_touch, s_now, &grip);
    ImuSupervisor_Step(&s_imu, s_out.imu_sleep_requested, s_now);
    bool fall = false;
    if (s_imu.sample_valid && s_out.fall_detection_enabled)
    {
        fall = FallDetection_Step(&s_imu.sample, s_now);
    }
    else if (!s_imu.sample_valid)
    {
        FallDetection_Reset();
    }
    if (fall) { ++s_incidents; }
    CaneMode_Input_t input = {
        .grip = grip,
        .imu_ready = (s_imu.state == IMU_SUPERVISOR_RUNNING),
        .imu_sleeping = (s_imu.state == IMU_SUPERVISOR_ASLEEP),
        .imu_fault = s_imu.fault,
        .sample_valid = s_imu.sample_valid,
        .sample = s_imu.sample,
        .local_fall = fall,
        .fall_pending = (FallDetection_GetState() != FALL_STATE_IDLE)
    };
    CaneMode_Step(&s_mode, &input, s_now, &s_out);
    if (s_out.reset_fall_detection) { FallDetection_Reset(); }
    AlertManager_SetFallState(s_out.fall_latched, s_out.alarm_paused, s_now);
    AlertManager_Arbitrate(ALERT_NONE, 0U, s_now, &s_audio);
    Ble_AdvancedThreatPacket_t packet = {
        .node_type = BLE_NODE_TYPE_CANE,
        .pair_id = BLE_DEFAULT_PAIR_ID,
        .state_flags = s_out.status_flags,
        .rolling_fall_seq = s_incidents,
        .nearest_dist_mm = 0xFFFFU,
        .ttc_20ms = BLE_TTC_NO_THREAT
    };
    Ble_RawPacket_t raw;
    BleComm_Encode(&packet, &raw);
    TEST_ASSERT_TRUE(BleComm_Decode(&raw, BLE_DEFAULT_PAIR_ID, &s_decoded));
}

static void Run(uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += 10U) { Cycle(); }
}

static void Drop_AndAcknowledge(void)
{
    s_touch = false;
    s_sensor = (Imu_Sample_t){0};
    Run(50U);
    s_sensor.ax = 3.0f;
    Run(10U);
    s_sensor.ax = 1.0f;
    Run(1010U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    TEST_ASSERT_EQUAL_UINT8(1U, s_incidents);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_FALL_ALERT, s_decoded.state_flags);
    s_touch = true;
    Run(30U);
    TEST_ASSERT_TRUE(s_out.alarm_paused);
    TEST_ASSERT_TRUE(s_audio.fall_latched);
    TEST_ASSERT_FALSE(s_audio.buzzer_active);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_ALARM_PAUSED | BLE_FLAG_FALL_ALERT, s_decoded.state_flags);
}

void setUp(void)
{
    s_now = 0U;
    s_reads = 0U;
    s_incidents = 0U;
    s_touch = true;
    s_sleep_ok = true;
    s_sensor = (Imu_Sample_t){ .az = 1.0f };
    memset(&s_out, 0, sizeof(s_out));
    CaneMode_Init(&s_mode, s_now);
    TouchSensor_Init(NULL);
    FallDetection_Init(&FallConfig_Cane);
    AlertManager_Init();
    BleComm_Init(&BleComm_Config_Cane);
    ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
    Run(300U);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
}

void tearDown(void) { }

void test_FullDrop_Pause_IntentionalPark_AndTouchWake(void)
{
    Drop_AndAcknowledge();
    Run(3000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    s_touch = false;
    Run(30U);
    s_touch = true;
    Run(3040U);
    TEST_ASSERT_EQUAL(SYS_STATE_STANDBY, s_out.state);
    TEST_ASSERT_EQUAL(IMU_SUPERVISOR_ASLEEP, s_imu.state);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_STANDBY | BLE_FLAG_PARKED_BY_USER, s_decoded.state_flags);
    TEST_ASSERT_FALSE(s_audio.fall_latched);
    uint32_t reads = s_reads;
    Run(3000U);
    TEST_ASSERT_EQUAL_UINT32(reads, s_reads);
    TEST_ASSERT_EQUAL_UINT8(1U, s_decoded.rolling_fall_seq);

    s_touch = false;
    Run(30U);
    s_touch = true;
    Run(130U);
    TEST_ASSERT_EQUAL(SYS_STATE_INIT, s_out.state);
    s_sensor = (Imu_Sample_t){ .az = 1.0f };
    Run(200U);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
    TEST_ASSERT_TRUE(s_out.fall_detection_enabled);
    TEST_ASSERT_FALSE(s_audio.fall_latched);
}

void test_FailedPowerTransition_CannotClearIncident_AndRealarmKeepsItsSequence(void)
{
    Drop_AndAcknowledge();
    s_touch = false;
    Run(30U);
    s_touch = true;
    s_sleep_ok = false;
    Run(3040U);
    TEST_ASSERT_TRUE(s_imu.fault);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_TRUE(s_out.alarm_paused);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_SENSOR_FAULT | BLE_FLAG_FALL_ALERT, s_decoded.state_flags);
    s_sleep_ok = true;
    while (s_out.alarm_paused && (s_now < 20000U)) { Cycle(); }
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    TEST_ASSERT_TRUE(s_audio.buzzer_active);
    TEST_ASSERT_EQUAL_UINT8(1U, s_decoded.rolling_fall_seq);
}

void test_RetainedAlarmAndGripAreProcessedDuringResetWait(void)
{
    ImuSupervisor_Init(&s_imu, &s_cfg, 4U, 5U, s_now);
    CaneMode_RestoreIncident(&s_mode);
    s_sensor = (Imu_Sample_t){.ax = 1.0f};
    s_touch = false;
    Run(30U);
    TEST_ASSERT_EQUAL_UINT8(MPU9250_INIT_WAKE, s_imu.init_step);
    TEST_ASSERT_FALSE(s_imu.sample_valid);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_TRUE(s_audio.buzzer_active);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_FALL_ALERT, s_decoded.state_flags);
    s_touch = true;
    Run(30U);
    TEST_ASSERT_EQUAL_UINT8(MPU9250_INIT_WAKE, s_imu.init_step);
    TEST_ASSERT_TRUE(s_out.alarm_paused); /* Debounced regrip still runs during reset wait. */
    TEST_ASSERT_FALSE(s_audio.buzzer_active);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_FALL_ALERT | BLE_FLAG_ALARM_PAUSED, s_decoded.state_flags);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_FullDrop_Pause_IntentionalPark_AndTouchWake);
    RUN_TEST(test_FailedPowerTransition_CannotClearIncident_AndRealarmKeepsItsSequence);
    RUN_TEST(test_RetainedAlarmAndGripAreProcessedDuringResetWait);
    return UNITY_END();
}
