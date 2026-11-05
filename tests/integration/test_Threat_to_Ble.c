#include "unity.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "services/BleComm/BleComm.h"
#include "mock_BleRadio.h"

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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_TtcPipelineQuantizesOnceWithoutChangingWireFormat);
    RUN_TEST(test_MaximumValidTtcDoesNotWrapIntoAnUrgentSmallValue);
    RUN_TEST(test_LongTtcIsUnavailableButObstacleFlagRemainsActive);
    RUN_TEST(test_AllTwelveBitTtcValuesRoundTripExactly);
    RUN_TEST(test_NoTargetDoesNotBecomeAZeroMillisecondCollision);
    return UNITY_END();
}
