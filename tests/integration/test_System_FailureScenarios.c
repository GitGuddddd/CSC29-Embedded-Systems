#include "unity.h"
#include "app/FallDetection/FallDetection.h"
#include "app/GroundProjection/GroundProjection.h"
#include "app/GroundProjection/GroundProjection_Cfg.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "app/AlertManager/AlertManager.h"
#include "services/BleComm/BleComm.h"
#include "services/Watchdog/Watchdog.h"
#include "drivers/Ld2450/Ld2450.h"
#include "drivers/Buzzer/Buzzer.h"
#include "services/SafetyDelivery/SafetyDelivery.h"
#include "app/AlertManager/AlertManager_Cfg.h"
#include <string.h>
#include <math.h>

#include "hal/Hal_Pwm.c"
#include "drivers/Buzzer/Buzzer.c"
#include "services/SafetyDelivery/SafetyDelivery.c"

/*
 * \file test_System_FailureScenarios.c
 * \brief Fault Injection, Edge Cases, Sensor Failure, and Auto-Recovery Test Suite
 * Standard: IEC 62304 / ISO 26262 Fault Injection Verification
 * Compliance: Barr-C:2018
 */

void setUp(void)
{
    AlertManager_Init();
    ThreatAssessment_Init();
    FallDetection_Init(&FallConfig_Head);
    Watchdog_Init(&Watchdog_Config_Default);
}

void tearDown(void)
{
}

/*
 * SCENARIO 1: Sensor Malfunction / Corrupt Floating Point Data (NaN / Inf injection)
 * Objective: Verify that if an IMU hardware glitch returns NaN or Infinity,
 * the Fall Detection algorithm does NOT get permanently stuck, divide by zero, or falsely trigger.
 * It must safely discard/absorb and auto-recover when healthy data resumes.
 */
void test_Scenario_ImuGlitch_NaN_And_AutoRecovery(void)
{
    Imu_Sample_t corrupt_sample;
    corrupt_sample.ax = 0.0f / 0.0f; /* NaN */
    corrupt_sample.ay = 1.0f / 0.0f; /* +Inf */
    corrupt_sample.az = -1.0f / 0.0f; /* -Inf */
    corrupt_sample.gx = 0.0f;
    corrupt_sample.gy = 0.0f;
    corrupt_sample.gz = 0.0f;
    corrupt_sample.timestamp_ms = 100U;

    /* Feed corrupt sample */
    bool fall_triggered = FallDetection_Step(&corrupt_sample, 100U);
    TEST_ASSERT_FALSE(fall_triggered);

    /* Verify system didn't crash or trigger false confirmed state */
    TEST_ASSERT_NOT_EQUAL(FALL_STATE_CHECK_STILL, FallDetection_GetState());

    /* Healthy sensor data resumes: normal walking/upright (1g on Z) */
    Imu_Sample_t healthy_sample;
    memset(&healthy_sample, 0, sizeof(healthy_sample));
    healthy_sample.az = 1.0f;

    for (uint32_t i = 0U; i < 20U; i++)
    {
        fall_triggered = FallDetection_Step(&healthy_sample, 200U + (i * 10U));
        TEST_ASSERT_FALSE(fall_triggered);
    }

    /* System state must be stable IDLE */
    TEST_ASSERT_EQUAL(FALL_STATE_IDLE, FallDetection_GetState());
}

/*
 * SCENARIO 2: Abrupt Optical Blackout / Out-of-Range Spike on ToF Sensor
 * Objective: Verify that if sunlight blinding or complete laser blackout occurs
 * (reading > 3000mm or < 100mm), the Ground Projection engine treats it as INVALID,
 * clears temporary voting spikes without falsely tripping a step-down cliff warning,
 * and recovers gracefully when valid ground returns resume.
 */
void test_Scenario_Tof_Glitch_Spike_Rejection_And_Recovery(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    /* Calibrate healthy 1000mm ground baseline */
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, i * 10U);
    }
    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);

    uint32_t t = GP_CAL_SAMPLES * 10U;

    /* Single isolated corrupt sunlight spike (e.g. 5000mm or 10mm) */
    GroundProjection_Feed(&d, GP_RANGE_VALID, 65000U, t + 10U); /* Triggers out-of-bounds INVALID */
    TEST_ASSERT_NOT_EQUAL(GP_STATE_STEP_DOWN, d.state);
    TEST_ASSERT_EQUAL(GP_STATE_UNKNOWN, d.state);

    /* Ground returns to normal 1000mm: median filter and voting rebuilds back to LEVEL */
    for (uint32_t i = 0U; i < 10U; i++)
    {
        t += 10U;
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, t);
    }
    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);
}

/*
 * SCENARIO 3: Hardware Bus Hang / Stale Sensor Data Timeout (500ms deadman)
 * Objective: Verify that if the ToF sensor physically disconnects or the I2C bus hangs,
 * the 500ms stale timeout enters SENSOR_FAULT. When the sensor is plugged back in,
 * it safely transitions back to UNKNOWN -> LEVEL without requiring a full system reboot.
 */
void test_Scenario_Tof_BusHang_Timeout_And_Hotplug_Recovery(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, i * 10U);
    }
    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);

    /* Sensor freezes: no new readings arrive for 600ms */
    uint32_t freeze_start = GP_CAL_SAMPLES * 10U;
    GroundProjection_Tick(&d, freeze_start + 600U);

    /* Must detect stale sensor and enter SENSOR_FAULT */
    TEST_ASSERT_EQUAL(GP_STATE_SENSOR_FAULT, d.state);

    /* Sensor hotplug / recovery: healthy readings resume */
    uint32_t resume_time = freeze_start + 1000U;
    for (uint32_t i = 0U; i < 10U; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, resume_time + (i * 10U));
    }
    /* Auto-recovers to LEVEL */
    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);
}

/*
 * SCENARIO 4: RF Packet Loss & Replay Attack / Sequence Inversion
 * Objective: Under heavy RF multipath interference or delayed packets,
 * stale or out-of-order packets must be rejected. When fresh packets resume,
 * monotonic sequence tracking must immediately synchronize.
 */
void test_Scenario_Ble_PacketCorruption_And_MonotonicRecovery(void)
{
    BleComm_Config_t cfg = {
        .role = BLE_ROLE_OBSERVER_CANE,
        .device_id = 0x01U
    };
    BleComm_Init(&cfg);

    Ble_AdvancedThreatPacket_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    /* Packet 1 arrives: sequence 10 */
    pkt.seq_num = 10U;
    TEST_ASSERT_TRUE(BleComm_ReceivePacket(&pkt));

    /* Delayed/Replayed stale packet: sequence 8 (must be rejected) */
    pkt.seq_num = 8U;
    TEST_ASSERT_FALSE(BleComm_ReceivePacket(&pkt));

    /* Duplicate packet: sequence 10 (must be rejected) */
    pkt.seq_num = 10U;
    TEST_ASSERT_FALSE(BleComm_ReceivePacket(&pkt));

    /* Link recovers: next monotonically increasing packet 11 accepted */
    pkt.seq_num = 11U;
    TEST_ASSERT_TRUE(BleComm_ReceivePacket(&pkt));
}

/*
 * SCENARIO 5: Torso Attenuation Link Loss & Deadman Watchdog Degraded Mode
 * Objective: When BLE packets are blocked by the user's torso for > 500ms,
 * the cane enters LINK_LOSS degraded mode (2 long-pulse warning).
 * Then, if a real cliff hazard occurs concurrently, verify that the physical hazard
 * strictly preempts the link-loss warning without deadlock or silence.
 */
void test_Scenario_Watchdog_LinkLoss_Preempted_By_CliffHazard(void)
{
    Watchdog_Config_t wcfg = { .deadman_timeout_ms = 500U };
    Watchdog_Init(&wcfg);

    uint32_t now = 0U;
    Watchdog_KickBleDeadman(now);

    /* Under 500ms: no link loss */
    now += 400U;
    TEST_ASSERT_FALSE(Watchdog_CheckBleLinkLoss(now));

    /* 550ms: Link Loss detected */
    now += 150U;
    TEST_ASSERT_TRUE(Watchdog_CheckBleLinkLoss(now));

    /* Feed link loss into AlertManager */
    Alert_Output_t alert_out;
    AlertManager_Arbitrate(ALERT_LINK_LOSS, 0U, now, &alert_out);
    TEST_ASSERT_EQUAL(ALERT_LINK_LOSS, alert_out.active_alert);
    TEST_ASSERT_TRUE(alert_out.haptic_active);

    /* While in link loss, severe cliff hazard occurs! */
    AlertManager_Arbitrate(ALERT_CLIFF_WARNING, 0U, now + 10U, &alert_out);
    /* Physical cliff hazard must override link loss */
    TEST_ASSERT_EQUAL(ALERT_CLIFF_WARNING, alert_out.active_alert);
    TEST_ASSERT_TRUE(alert_out.haptic_active);

    /* Link recovers: heartbeat received */
    Watchdog_KickBleDeadman(now + 20U);
    TEST_ASSERT_FALSE(Watchdog_CheckBleLinkLoss(now + 25U));
}

/*
 * SCENARIO 6: An unacknowledged incident does not silently expire.
 * Audio pause is explicit and bounded by the mode manager; the latch remains
 * set through pulse gaps, technical faults and an acknowledged pause.
 */
void test_Scenario_Incapacitated_Fall_LatchedUntilExplicitRecovery(void)
{
    AlertManager_Init();

    Alert_Output_t out;
    uint32_t t = 1000U;

    /* Fall triggers alarm */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, t, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* During a rhythmic silent gap the incident is still active. */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, t + 29900U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* After 30 seconds there is no automatic indefinite mute. */
    AlertManager_Arbitrate(ALERT_NONE, 0U, t + 30100U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* Even if other minor alerts arrive, latched fall prevents them from clearing */
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 500U, t + 35000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* Explicit, verified recovery resets the system. */
    AlertManager_AcknowledgeReset();
    AlertManager_Arbitrate(ALERT_NONE, 0U, t + 36000U, &out);
    TEST_ASSERT_FALSE(out.fall_latched);
}

/*
 * SCENARIO 7: Cane Buzzer Hardware Failure & Automatic Failover to Glasses
 * Objective: Verify that when Cane buzzer fails, Cane buzzer is silenced,
 * BLE reports BLE_FLAG_BUZZER_FAULT, SafetyDelivery triggers fallback immediately,
 * and Glasses automatically takes over audio output. When Cane buzzer recovers,
 * Glasses buzzer turns silent and Cane buzzer resumes normal playback.
 */
void test_Scenario_CaneBuzzerFailure_AutomaticFailoverToGlasses(void)
{
    /* Initialize subsystem drivers */
    Buzzer_Init(NULL);
    TEST_ASSERT_TRUE(Buzzer_IsHealthy());
    AlertManager_Init();

    SafetyDelivery_t delivery;
    SafetyDelivery_Init(&delivery, 0x4321U);

    /* 1. Normal state: Head-fall latches on Glasses at t = 1000 ms */
    uint32_t t = 1000U;
    SafetyDelivery_Start(&delivery, 0x4321U, 2U, t);

    /* Cane is healthy, receives head fall over BLE, and sends ACK with healthy buzzer */
    Ble_AdvancedThreatPacket_t cane_pkt = {
        .preamble = 0xAAU,
        .device_id = 0x01U,
        .pair_id = BLE_DEFAULT_PAIR_ID,
        .node_type = BLE_NODE_TYPE_CANE,
        .protocol_version = BLE_PROTOCOL_VERSION,
        .session_id = 0x8888U,
        .peer_ack_valid = Buzzer_IsHealthy(),
        .peer_ack_session_id = 0x4321U,
        .peer_ack_event_seq = 2U,
        .state_flags = 0U
    };
    Ble_RawFrame_t frame;
    BleComm_EncodeFrame(&cane_pkt, &frame);

    Ble_AdvancedThreatPacket_t rx_pkt;
    TEST_ASSERT_TRUE(BleComm_DecodeFrame(&frame, BLE_DEFAULT_PAIR_ID, &rx_pkt));
    SafetyDelivery_ObservePeer(&delivery, &rx_pkt, t + 50U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&delivery));
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(&delivery, t + 50U));

    /* Dual-node audio arbitration in healthy state */
    /* Glasses buzzer must remain SILENT by default (played ONLY on Cane) */
    Alert_SyncInput_t sync_glasses = {
        .is_glasses_node = true,
        .local_fall_latched = true,
        .local_alarm_paused = false,
        .peer_fall_active = false,
        .delivery_fallback = SafetyDelivery_NeedsFallback(&delivery, t + 50U),
        .cane_link_fresh = true,
        .cane_buzzer_fault = ((rx_pkt.state_flags & BLE_FLAG_BUZZER_FAULT) != 0U),
        .obstacle_active = false,
        .obstacle_dist_mm = 0U,
        .sensor_fault = false
    };
    Alert_Output_t out_glasses;
    AlertManager_ArbitrateSynced(&sync_glasses, t + 50U, &out_glasses);
    TEST_ASSERT_FALSE(out_glasses.buzzer_active);

    /* Cane buzzer plays the emergency alarm */
    Alert_SyncInput_t sync_cane = {
        .is_glasses_node = false,
        .local_fall_latched = false,
        .local_alarm_paused = false,
        .peer_fall_active = true,
        .delivery_fallback = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = !Buzzer_IsHealthy(),
        .obstacle_active = false,
        .obstacle_dist_mm = 0U,
        .sensor_fault = false
    };
    Alert_Output_t out_cane;
    AlertManager_ArbitrateSynced(&sync_cane, t + 50U, &out_cane);
    TEST_ASSERT_TRUE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_cane.active_alert);
    Buzzer_PlayToneAtDuty(out_cane.generated_freq_hz, out_cane.generated_duty_percent);
    TEST_ASSERT_EQUAL_UINT16(out_cane.generated_freq_hz, Buzzer_GetActiveFrequencyHz());
    TEST_ASSERT_EQUAL_UINT8(out_cane.generated_duty_percent, Buzzer_GetActiveDutyPercent());

    /* 2. FAULT INJECTION: Cane buzzer hardware fails! */
    t = 1100U;
    Buzzer_InjectFault(true);
    TEST_ASSERT_FALSE(Buzzer_IsHealthy());

    /* Cane buzzer stops immediately */
    Buzzer_PlayToneAtDuty(out_cane.generated_freq_hz, out_cane.generated_duty_percent);
    TEST_ASSERT_EQUAL_UINT16(0U, Buzzer_GetActiveFrequencyHz());
    TEST_ASSERT_EQUAL_UINT8(0U, Buzzer_GetActiveDutyPercent());

    /* Cane updates BLE packet: peer_ack_valid is false, BLE_FLAG_BUZZER_FAULT is set */
    cane_pkt.peer_ack_valid = Buzzer_IsHealthy();
    cane_pkt.state_flags |= BLE_FLAG_BUZZER_FAULT;
    BleComm_EncodeFrame(&cane_pkt, &frame);

    /* Glasses decodes frame and observes buzzer fault */
    TEST_ASSERT_TRUE(BleComm_DecodeFrame(&frame, BLE_DEFAULT_PAIR_ID, &rx_pkt));
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_BUZZER_FAULT, rx_pkt.state_flags);
    SafetyDelivery_ObservePeer(&delivery, &rx_pkt, t);

    /* Immediate fallback without waiting 750 ms */
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&delivery));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(&delivery, t));

    /* Arbitration under Cane buzzer fault */
    /* Cane buzzer is forced off */
    sync_cane.cane_buzzer_fault = !Buzzer_IsHealthy();
    AlertManager_ArbitrateSynced(&sync_cane, t, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

    /* Glasses automatically takes over audio output! */
    sync_glasses.cane_buzzer_fault = ((rx_pkt.state_flags & BLE_FLAG_BUZZER_FAULT) != 0U);
    sync_glasses.delivery_fallback = SafetyDelivery_NeedsFallback(&delivery, t);
    AlertManager_ArbitrateSynced(&sync_glasses, t, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_glasses.active_alert);

    /* 3. RECOVERY: Cane buzzer recovers! */
    t = 1200U;
    Buzzer_InjectFault(false);
    TEST_ASSERT_TRUE(Buzzer_IsHealthy());

    cane_pkt.peer_ack_valid = Buzzer_IsHealthy();
    cane_pkt.state_flags &= ~BLE_FLAG_BUZZER_FAULT;
    BleComm_EncodeFrame(&cane_pkt, &frame);

    TEST_ASSERT_TRUE(BleComm_DecodeFrame(&frame, BLE_DEFAULT_PAIR_ID, &rx_pkt));
    TEST_ASSERT_BITS_LOW(BLE_FLAG_BUZZER_FAULT, rx_pkt.state_flags);
    SafetyDelivery_ObservePeer(&delivery, &rx_pkt, t);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&delivery));
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(&delivery, t));

    /* Glasses buzzer returns to silent */
    sync_glasses.cane_buzzer_fault = ((rx_pkt.state_flags & BLE_FLAG_BUZZER_FAULT) != 0U);
    sync_glasses.delivery_fallback = SafetyDelivery_NeedsFallback(&delivery, t);
    AlertManager_ArbitrateSynced(&sync_glasses, t, &out_glasses);
    TEST_ASSERT_FALSE(out_glasses.buzzer_active);

    /* Cane buzzer resumes playing */
    sync_cane.cane_buzzer_fault = !Buzzer_IsHealthy();
    AlertManager_ArbitrateSynced(&sync_cane, t, &out_cane);
    TEST_ASSERT_TRUE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_cane.active_alert);
    Buzzer_PlayToneAtDuty(out_cane.generated_freq_hz, out_cane.generated_duty_percent);
    TEST_ASSERT_EQUAL_UINT16(out_cane.generated_freq_hz, Buzzer_GetActiveFrequencyHz());
    TEST_ASSERT_EQUAL_UINT8(out_cane.generated_duty_percent, Buzzer_GetActiveDutyPercent());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Scenario_ImuGlitch_NaN_And_AutoRecovery);
    RUN_TEST(test_Scenario_Tof_Glitch_Spike_Rejection_And_Recovery);
    RUN_TEST(test_Scenario_Tof_BusHang_Timeout_And_Hotplug_Recovery);
    RUN_TEST(test_Scenario_Ble_PacketCorruption_And_MonotonicRecovery);
    RUN_TEST(test_Scenario_Watchdog_LinkLoss_Preempted_By_CliffHazard);
    RUN_TEST(test_Scenario_Incapacitated_Fall_LatchedUntilExplicitRecovery);
    RUN_TEST(test_Scenario_CaneBuzzerFailure_AutomaticFailoverToGlasses);
    return UNITY_END();
}
