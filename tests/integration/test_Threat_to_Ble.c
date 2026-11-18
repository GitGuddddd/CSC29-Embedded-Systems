#include "unity.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "services/BleComm/BleComm.h"
#include "mock_BleRadio.h"
#include "app/AlertManager/AlertManager.h"
#include "app/AlertManager/AlertManager_Cfg.h"
#include "drivers/Buzzer/Buzzer.h"
#include "services/SafetyDelivery/SafetyDelivery.h"

#include "hal/Hal_Pwm.c"
#include "drivers/Buzzer/Buzzer.c"
#include "services/SafetyDelivery/SafetyDelivery.c"
#include "app/AlertManager/AlertManager.c"

void setUp(void)
{
    MockBleRadio_Reset();
    BleComm_Init(&BleComm_Config_Glasses);
    ThreatAssessment_Init();
}

void tearDown(void)
{
}

static Ble_AdvancedThreatPacket_t RoundTrip(const Threat_Result_t * const p_threat)
{
    Ble_AdvancedThreatPacket_t source = {
        .pair_id = BLE_DEFAULT_PAIR_ID,
        .node_type = BLE_NODE_TYPE_GLASSES,
        .session_id = BleComm_GetSessionId(),
        .state_flags = p_threat->is_threat_active ? BLE_FLAG_OBSTACLE_ACTIVE : 0U,
        .nearest_dist_mm = p_threat->nearest_distance_mm,
        .ttc_20ms = p_threat->ttc_20ms
    };
    Ble_RawFrame_t frame;
    Ble_AdvancedThreatPacket_t decoded;
    BleComm_EncodeFrame(&source, &frame);
    TEST_ASSERT_TRUE(BleComm_DecodeFrame(&frame, BLE_DEFAULT_PAIR_ID, &decoded));
    return decoded;
}

void test_TtcPipelineQuantizesOnceWithoutChangingWireFormat(void)
{
    Radar_Target_t target = {.distance_mm = 1150U, .speed_cms = 20, .is_valid = true};
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    Ble_AdvancedThreatPacket_t decoded = RoundTrip(&result);
    TEST_ASSERT_EQUAL_UINT16(287U, result.ttc_20ms); /* 5750 ms -> 5740 ms on the existing wire. */
    TEST_ASSERT_EQUAL_UINT16(result.ttc_20ms, decoded.ttc_20ms);
    TEST_ASSERT_EQUAL_UINT32(12U, sizeof(Ble_RawFrame_t));
}

void test_MaximumValidTtcDoesNotWrapIntoAnUrgentSmallValue(void)
{
    Radar_Target_t target = {.distance_mm = 4094U, .speed_cms = 5, .is_valid = true};
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    Ble_AdvancedThreatPacket_t decoded = RoundTrip(&result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_MAX_TICKS, decoded.ttc_20ms);
    TEST_ASSERT_EQUAL_UINT32(81880U, (uint32_t)decoded.ttc_20ms * BLE_TTC_STEP_MS);
    Ble_RawFrame_t frame;
    BleComm_EncodeFrame(&decoded, &frame);
    static const uint8_t expected[12] = {
        0xA1U, 0x00U, 0x00U, 0xFFU, 0x7EU, 0xFFU,
        0xE2U, 0x02U, 0x29U, 0x00U, 0x00U, 0x00U
    }; /* Shared maximum-TTC vector with Android. */
    TEST_ASSERT_EQUAL_MEMORY(expected, &frame, sizeof(expected));
}

void test_LongTtcIsUnavailableButObstacleFlagRemainsActive(void)
{
    Radar_Target_t target = {.distance_mm = 2000U, .speed_cms = 1, .is_valid = true};
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    Ble_AdvancedThreatPacket_t decoded = RoundTrip(&result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, decoded.ttc_20ms);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_OBSTACLE_ACTIVE, decoded.state_flags);
}

void test_AllTwelveBitTtcValuesRoundTripExactly(void)
{
    Threat_Result_t result = {.nearest_distance_mm = 1000U};
    for (uint16_t ticks = 0U; ticks <= BLE_TTC_NO_THREAT; ++ticks)
    {
        result.ttc_20ms = ticks & 0x0FFFU;
        Ble_AdvancedThreatPacket_t decoded = RoundTrip(&result);
        TEST_ASSERT_EQUAL_UINT16(ticks, decoded.ttc_20ms);
    }
}

void test_NoTargetDoesNotBecomeAZeroMillisecondCollision(void)
{
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(NULL, 0U, GRIP_STATE_HELD, &result);
    Ble_AdvancedThreatPacket_t decoded = RoundTrip(&result);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, decoded.nearest_dist_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, decoded.ttc_20ms);
    TEST_ASSERT_EQUAL_UINT8(0U, decoded.state_flags);
}

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

    /* Dual-node audio arbitration in healthy state for a Cane-detected fall */
    /* Glasses buzzer must remain SILENT when the fall is Cane-detected */
    Alert_SyncInput_t sync_glasses = {
        .is_glasses_node = true,
        .local_fall_latched = false,
        .local_alarm_paused = false,
        .peer_fall_active = true,
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

    /* Cane buzzer plays the Cane-detected emergency alarm */
    Alert_SyncInput_t sync_cane = {
        .is_glasses_node = false,
        .local_fall_latched = true,
        .local_alarm_paused = false,
        .peer_fall_active = false,
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
    RUN_TEST(test_TtcPipelineQuantizesOnceWithoutChangingWireFormat);
    RUN_TEST(test_MaximumValidTtcDoesNotWrapIntoAnUrgentSmallValue);
    RUN_TEST(test_LongTtcIsUnavailableButObstacleFlagRemainsActive);
    RUN_TEST(test_AllTwelveBitTtcValuesRoundTripExactly);
    RUN_TEST(test_NoTargetDoesNotBecomeAZeroMillisecondCollision);
    RUN_TEST(test_Scenario_CaneBuzzerFailure_AutomaticFailoverToGlasses);
    return UNITY_END();
}
