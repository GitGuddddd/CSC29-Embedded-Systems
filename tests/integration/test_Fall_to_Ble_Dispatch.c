#include "unity.h"
#include "app/FallDetection/FallDetection.h"
#include "services/BleComm/BleComm.h"

void setUp(void)
{
    FallDetection_Init(&FallConfig_Head);
    BleComm_Init(&BleComm_Config_Glasses);
}

void tearDown(void)
{
}

void test_Integration_FallToBleDispatch(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall_detected = false;

    /* Upright standing baseline */
    s.ax = 0.0f; s.ay = 0.0f; s.az = 1.0f;
    FallDetection_Step(&s, now);

    /* Freefall: 0g */
    now += 10U;
    s.az = 0.0f;
    FallDetection_Step(&s, now);

    /* Maintain freefall > 50ms */
    now += 60U;
    FallDetection_Step(&s, now);

    /* Impact: 3.0g */
    now += 10U;
    s.ax = 3.0f;
    FallDetection_Step(&s, now);

    /* Settle for 800ms */
    now += 800U;
    FallDetection_Step(&s, now);

    /* Still on ground, tilted 90 degrees */
    s.ax = 1.0f; s.ay = 0.0f; s.az = 0.0f;
    for (uint8_t i = 0U; i < 50U; i++)
    {
        now += 10U;
        fall_detected = FallDetection_Step(&s, now);
    }

    TEST_ASSERT_TRUE(fall_detected);

    /* Verify BLE Dispatch payload */
    Ble_AdvancedThreatPacket_t pkt = {0};
    pkt.preamble = 0xAAU;
    pkt.device_id = 0x02U;
    pkt.pair_id = BLE_DEFAULT_PAIR_ID;
    pkt.node_type = BLE_NODE_TYPE_GLASSES;
    pkt.heartbeat_seq = 1U;
    pkt.rolling_fall_seq = 1U;
    pkt.seq_num = 1U;
    pkt.state_flags = (fall_detected) ? 0x01U : 0x00U;
    pkt.nearest_dist_mm = 0xFFFFU;
    pkt.closing_speed_cms = 0;
    pkt.ttc_ms = 0xFFFFU;

    bool tx_ok = BleComm_SendPacket(&pkt);
    TEST_ASSERT_TRUE(tx_ok);

    /* Verify Receiver side accepts first packet */
    bool rx_ok = BleComm_ReceivePacket(&pkt);
    TEST_ASSERT_TRUE(rx_ok);

    /* Verify packet can be popped from RX queue */
    Ble_AdvancedThreatPacket_t popped_pkt;
    bool pop_ok = BleComm_PopReceivedPacket(&popped_pkt, NULL);
    TEST_ASSERT_TRUE(pop_ok);
    TEST_ASSERT_EQUAL_HEX8(0x02U, popped_pkt.device_id);
    TEST_ASSERT_EQUAL_HEX8(0x01U, popped_pkt.state_flags & 0x01U);

    /* Queue should now be empty */
    TEST_ASSERT_FALSE(BleComm_PopReceivedPacket(&popped_pkt, NULL));
}

void test_BleComm_EncodeDecode_6ByteRawPacket(void)
{
    TEST_ASSERT_EQUAL_UINT32(6U, sizeof(Ble_RawPacket_t));

    Ble_AdvancedThreatPacket_t src = {
        .preamble = 0xAAU,
        .device_id = 0x02U,
        .pair_id = 0x05U,
        .node_type = BLE_NODE_TYPE_GLASSES,
        .heartbeat_seq = 3U,
        .rolling_fall_seq = 7U,
        .seq_num = 3U,
        .state_flags = BLE_FLAG_FALL_ALERT | BLE_FLAG_OBSTACLE_ACTIVE,
        .nearest_dist_mm = 1200U, /* 1200 mm */
        .closing_speed_cms = 0,
        .ttc_ms = 8000U           /* 8000 ms */
    };

    Ble_RawPacket_t raw;
    BleComm_Encode(&src, &raw);

    /* Check Byte 0: sync=0xA, pair=0x5 -> 0xA5 */
    TEST_ASSERT_EQUAL_HEX8(0xA5U, raw.preamble_pair_id);

    /* Check Byte 1: type=0, hb=3, fall=7 -> (0<<7)|(3<<4)|7 = 0x37 */
    TEST_ASSERT_EQUAL_HEX8(0x37U, raw.type_seq_fall);

    /* Check Byte 2: state flags */
    TEST_ASSERT_EQUAL_HEX8(src.state_flags, raw.state_flags);

    /* Decode with correct pair ID */
    Ble_AdvancedThreatPacket_t decoded = {0};
    bool dec_ok = BleComm_Decode(&raw, 0x05U, &decoded);
    TEST_ASSERT_TRUE(dec_ok);
    TEST_ASSERT_EQUAL_HEX8(0x05U, decoded.pair_id);
    TEST_ASSERT_EQUAL_INT(BLE_NODE_TYPE_GLASSES, decoded.node_type);
    TEST_ASSERT_EQUAL_UINT8(3U, decoded.heartbeat_seq);
    TEST_ASSERT_EQUAL_UINT8(7U, decoded.rolling_fall_seq);
    TEST_ASSERT_EQUAL_HEX8(src.state_flags, decoded.state_flags);
    TEST_ASSERT_EQUAL_UINT16(1200U, decoded.nearest_dist_mm);
    TEST_ASSERT_EQUAL_UINT16(8000U, decoded.ttc_ms);

    /* Decode with wrong pair ID must be rejected */
    Ble_AdvancedThreatPacket_t wrong = {0};
    bool reject_ok = BleComm_Decode(&raw, 0x09U, &wrong);
    TEST_ASSERT_FALSE(reject_ok);
}

void test_BleComm_WrapSafe_RollingFallCount(void)
{
    /* Delta = 0: Duplicate, no new fall */
    TEST_ASSERT_FALSE(BleComm_CheckNewFall(5U, 5U));

    /* Normal increment: 5 -> 6 */
    TEST_ASSERT_TRUE(BleComm_CheckNewFall(6U, 5U));

    /* Several packets dropped: 5 -> 8 */
    TEST_ASSERT_TRUE(BleComm_CheckNewFall(8U, 5U));

    /* Maximum window: 5 -> 12 (delta = 7) */
    TEST_ASSERT_TRUE(BleComm_CheckNewFall(12U, 5U));

    /* Beyond window: delta = 8 -> rejected as stale */
    TEST_ASSERT_FALSE(BleComm_CheckNewFall(13U, 5U));

    /* Wrap-around: 14 -> 1 (delta = (1 - 14) & 15 = 3) -> valid new fall */
    TEST_ASSERT_TRUE(BleComm_CheckNewFall(1U, 14U));

    /* Wrap-around: 15 -> 0 (delta = 1) -> valid new fall */
    TEST_ASSERT_TRUE(BleComm_CheckNewFall(0U, 15U));

    /* Wrap-around stale: 1 -> 14 (delta = (14 - 1) & 15 = 13) -> rejected */
    TEST_ASSERT_FALSE(BleComm_CheckNewFall(14U, 1U));
}

void test_PeerFall_IsDetectedOnFirstContact_AndNotReplayedAfterAcknowledgement(void)
{
    BleComm_PeerFall_t tracker = {0};
    Ble_AdvancedThreatPacket_t packet = {
        .rolling_fall_seq = 15U,
        .state_flags = BLE_FLAG_FALL_ALERT
    };
    TEST_ASSERT_TRUE(BleComm_TrackPeerFall(&tracker, &packet));
    TEST_ASSERT_FALSE(BleComm_TrackPeerFall(&tracker, &packet));
    packet.state_flags |= BLE_FLAG_ALARM_PAUSED;
    TEST_ASSERT_FALSE(BleComm_TrackPeerFall(&tracker, &packet));
    packet.state_flags = BLE_FLAG_FALL_ALERT;
    TEST_ASSERT_FALSE(BleComm_TrackPeerFall(&tracker, &packet));
    /* Stale incident counters must not move the baseline backwards. */
    packet.rolling_fall_seq = 14U;
    TEST_ASSERT_FALSE(BleComm_TrackPeerFall(&tracker, &packet));
    packet.rolling_fall_seq = 15U;
    TEST_ASSERT_FALSE(BleComm_TrackPeerFall(&tracker, &packet));
    packet.rolling_fall_seq = 0U;
    TEST_ASSERT_TRUE(BleComm_TrackPeerFall(&tracker, &packet));
    packet.state_flags = 0U;
    TEST_ASSERT_FALSE(BleComm_TrackPeerFall(&tracker, &packet));
    packet.state_flags = BLE_FLAG_FALL_ALERT;
    TEST_ASSERT_TRUE(BleComm_TrackPeerFall(&tracker, &packet));
}

void test_PausedAndParkedStatus_RoundTripsWithoutChangingPacketSize(void)
{
    Ble_AdvancedThreatPacket_t packet = {
        .node_type = BLE_NODE_TYPE_CANE,
        .pair_id = BLE_DEFAULT_PAIR_ID,
        .state_flags = BLE_FLAG_FALL_ALERT | BLE_FLAG_ALARM_PAUSED,
        .nearest_dist_mm = 0xFFFFU,
        .ttc_ms = 0xFFFFU
    };
    Ble_RawPacket_t raw;
    Ble_AdvancedThreatPacket_t decoded;
    BleComm_Encode(&packet, &raw);
    TEST_ASSERT_TRUE(BleComm_Decode(&raw, BLE_DEFAULT_PAIR_ID, &decoded));
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_FALL_ALERT | BLE_FLAG_ALARM_PAUSED, decoded.state_flags);
    packet.state_flags = BLE_FLAG_STANDBY | BLE_FLAG_PARKED_BY_USER | BLE_FLAG_CANE_HELD;
    BleComm_Encode(&packet, &raw);
    TEST_ASSERT_TRUE(BleComm_Decode(&raw, BLE_DEFAULT_PAIR_ID, &decoded));
    TEST_ASSERT_EQUAL_HEX8(packet.state_flags, decoded.state_flags);
    TEST_ASSERT_EQUAL_UINT32(6U, sizeof(raw));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Integration_FallToBleDispatch);
    RUN_TEST(test_BleComm_EncodeDecode_6ByteRawPacket);
    RUN_TEST(test_BleComm_WrapSafe_RollingFallCount);
    RUN_TEST(test_PeerFall_IsDetectedOnFirstContact_AndNotReplayedAfterAcknowledgement);
    RUN_TEST(test_PausedAndParkedStatus_RoundTripsWithoutChangingPacketSize);
    return UNITY_END();
}
