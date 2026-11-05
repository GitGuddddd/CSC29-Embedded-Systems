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

static Ble_AdvancedThreatPacket_t PeerReceipt(uint32_t peer_session,
                                               uint32_t origin_session,
                                               uint32_t event_id)
{
    Ble_AdvancedThreatPacket_t packet = {0};
    packet.protocol_version = BLE_PROTOCOL_VERSION;
    packet.session_id = peer_session;
    packet.peer_ack_valid = true;
    packet.peer_ack_session_id = origin_session;
    packet.peer_ack_event_id = event_id;
    return packet;
}

void test_FallbackStartsOnlyAfterBoundedReceiptDeadline(void)
{
    SafetyDelivery_Start(&s_delivery, 0x10000007U, 100U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS - 1U));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS));
}

void test_OnlyExactApplicationReceiptConfirmsCurrentEvent(void)
{
    SafetyDelivery_Start(&s_delivery, 0x10000007U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA0000001U, 0x99990000U,
                                                     0x10000007U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.peer_ack_session_id = 0x1234U;
    packet.peer_ack_event_id = 0x10000006U;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 200U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.peer_ack_event_id = 0x10000007U;
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 300U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(&s_delivery, 300U));
}

void test_ConfirmedEventFallsBackWhenPeerHeartbeatExpires(void)
{
    SafetyDelivery_Start(&s_delivery, 1U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA0000001U, 0x1234U, 1U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_PEER_TIMEOUT_MS - 1U));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, 100U + SAFETYDELIVERY_CFG_PEER_TIMEOUT_MS));
}

void test_PeerRebootInvalidatesOldReceipt(void)
{
    SafetyDelivery_Start(&s_delivery, 1U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA0000001U, 0x1234U, 1U);
    SafetyDelivery_ObservePeer(&s_delivery, &packet, 100U);
    TEST_ASSERT_TRUE(SafetyDelivery_IsConfirmed(&s_delivery));
    packet.session_id = 0xA0000002U;
    packet.peer_ack_valid = false;
    SafetyDelivery_ObservePeer(&s_delivery, &packet,
                               SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS + 1U);
    TEST_ASSERT_FALSE(SafetyDelivery_IsConfirmed(&s_delivery));
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS + 1U));
}

void test_LegacyDowngradeInvalidatesOldReceipt(void)
{
    SafetyDelivery_Start(&s_delivery, 1U, 0U);
    Ble_AdvancedThreatPacket_t packet = PeerReceipt(0xA0000001U, 0x1234U, 1U);
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
    SafetyDelivery_Start(&s_delivery, 3U, start);
    TEST_ASSERT_TRUE(SafetyDelivery_NeedsFallback(
        &s_delivery, start + SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS));
    SafetyDelivery_Clear(&s_delivery);
    TEST_ASSERT_FALSE(SafetyDelivery_NeedsFallback(&s_delivery, UINT32_MAX));
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
    return UNITY_END();
}
