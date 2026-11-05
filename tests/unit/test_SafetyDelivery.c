#include "unity.h"
#include "services/SafetyDelivery/SafetyDelivery.h"
#include "services/SafetyDelivery/SafetyDelivery_Cfg.h"

static SafetyDelivery_t s_delivery;

void setUp(void)
{
    SafetyDelivery_Init(&s_delivery, 0x1234U);
}

void tearDown(void)
{
}

static Ble_AdvancedThreatPacket_t PeerReceipt(uint16_t peer_session,
                                               uint16_t origin_session,
                                               uint8_t event_seq)
{
    Ble_AdvancedThreatPacket_t packet = {0};
    packet.node_type = BLE_NODE_TYPE_CANE;
    packet.device_id = 0x01U;
    packet.protocol_version = BLE_PROTOCOL_VERSION;
    packet.session_id = peer_session;
    packet.peer_ack_valid = true;
    packet.peer_ack_session_id = origin_session;
    packet.peer_ack_event_seq = event_seq & BLE_FALL_SEQ_MASK;
    return packet;
}

void test_FallbackStartsOnlyAfterBoundedReceiptDeadline(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 7U, 100U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS - 1U));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS));
}

void test_OnlyExactApplicationReceiptConfirmsCurrentEvent(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 7U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x9999U, 7U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.peer_ack_session_id = 0x1234U;
    packet.peer_ack_event_seq = 6U;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 200U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.peer_ack_event_seq = 7U;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 300U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(&s_delivery, 300U));
}

void test_ConfirmedEventFallsBackWhenPeerHeartbeatExpires(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 1U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x1234U, 1U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_PEER_TIMEOUT_MS - 1U));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_PEER_TIMEOUT_MS));
}

void test_PeerRebootInvalidatesOldReceipt(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 1U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x1234U, 1U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.session_id = 0xA002U;
    packet.peer_ack_valid = false;
    SafetyDelivery_ObservePeer(&s_delivery, &packet,
                               SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS + 1U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS + 1U));
}

void test_LegacyDowngradeInvalidatesOldReceipt(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 1U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x1234U, 1U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.protocol_version = 0U;
    packet.session_id = 0U;
    packet.peer_ack_valid = false;
    SafetyDelivery_ObservePeer(&s_delivery, &packet,
                               SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS + 1U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS + 1U));
}

void test_ClearStopsFallbackAndTimerArithmeticWraps(void)
{
    uint32_t start = UINT32_MAX - 100U;
    SafetyDelivery_Start(&s_delivery, 0x1234U, 3U, start);
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, start + SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS));
    SafetyDelivery_Clear(&s_delivery);
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(&s_delivery, UINT32_MAX));
}

void test_ReusedNibbleRequiresFreshEpochReceipt(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 0U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x1234U, 0U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    SafetyDelivery_Start(&s_delivery, 0x1235U, 0U, 1000U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 1100U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.peer_ack_session_id = 0x1235U;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 1200U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
}

void test_WithdrawnReceiptRearmsFallbackWithoutClearingIncident(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 2U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x1234U, 2U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.peer_ack_valid = false;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 1000U);
    TEST_ASSERT_TRUE(s_delivery.active);
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(&s_delivery, 1000U));
}

void test_PreEventAndOldVersionReceiptsCannotConfirm(void)
{
    SafetyDelivery_Start(&s_delivery, 0x1234U, 2U, 1000U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA001U, 0x1234U, 2U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 999U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.protocol_version = 1U;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 1100U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.protocol_version = BLE_PROTOCOL_VERSION;
    packet.node_type = BLE_NODE_TYPE_GLASSES;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 1200U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_FallbackStartsOnlyAfterBoundedReceiptDeadline);
    RUN_TEST(test_OnlyExactApplicationReceiptConfirmsCurrentEvent);
    RUN_TEST(test_ConfirmedEventFallsBackWhenPeerHeartbeatExpires);
    RUN_TEST(test_PeerRebootInvalidatesOldReceipt);
    RUN_TEST(test_LegacyDowngradeInvalidatesOldReceipt);
    RUN_TEST(test_ClearStopsFallbackAndTimerArithmeticWraps);
    RUN_TEST(test_ReusedNibbleRequiresFreshEpochReceipt);
    RUN_TEST(test_WithdrawnReceiptRearmsFallbackWithoutClearingIncident);
    RUN_TEST(test_PreEventAndOldVersionReceiptsCannotConfirm);
    return UNITY_END();
}
