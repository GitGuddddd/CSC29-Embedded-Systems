#include "unity.h"
#include "services/BleComm/BleComm.h"
#include "services/BleComm/BleComm_Cfg.h"
#include "services/Watchdog/Watchdog.h"
#include "mock_BleRadio.h"
#include <string.h>

static Ble_AdvancedThreatPacket_t s_packet;

void setUp(void)
{
    MockBleRadio_Reset();
    BleComm_Init(&BleComm_Config_Cane);
    Watchdog_Init(&Watchdog_Config_Default);
    s_packet = (Ble_AdvancedThreatPacket_t){
        .node_type = BLE_NODE_TYPE_CANE,
        .pair_id = BLE_DEFAULT_PAIR_ID,
        .nearest_dist_mm = 0xFFFFU,
        .ttc_ms = 0xFFFFU
    };
}

void tearDown(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, MockBleRadio_GetState()->wrong_context_count);
}

static void Service(uint32_t now_ms)
{
    MockBleRadio_SetTime(now_ms);
    (void)BleComm_Core1_Task();
}

void test_AllSystemStates_HaveExplicitRadioPolicies(void)
{
    static const BleComm_ProfileId_t expected[] = {
        BLECOMM_PROFILE_FAST,   /* INIT */
        BLECOMM_PROFILE_QUIET,  /* IDLE */
        BLECOMM_PROFILE_ACTIVE,
        BLECOMM_PROFILE_FAST,   /* WATCH */
        BLECOMM_PROFILE_FAST,   /* FALL_CONFIRMED */
        BLECOMM_PROFILE_FAST,   /* FALL_MUTED */
        BLECOMM_PROFILE_FAST,   /* HARD_FAULT */
        BLECOMM_PROFILE_QUIET,  /* STANDBY */
        BLECOMM_PROFILE_FAST,   /* UNGRIPPED_WATCH */
        BLECOMM_PROFILE_FAST    /* FAULT_DEGRADED */
    };
    TEST_ASSERT_EQUAL_UINT32(SYS_STATE_COUNT, sizeof(expected) / sizeof(expected[0]));
    for (uint32_t i = 0U; i < (uint32_t)SYS_STATE_COUNT; ++i)
    {
        TEST_ASSERT_EQUAL(expected[i], BleComm_SelectProfile((Sys_State_t)i, 0U, false));
        TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST, BleComm_SelectProfile((Sys_State_t)i, 0U, true));
        TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST,
                          BleComm_SelectProfile((Sys_State_t)i, BLE_FLAG_FALL_ALERT, false));
    }
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST, BleComm_SelectProfile((Sys_State_t)255U, 0U, false));
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST, BleComm_SelectProfile(SYS_STATE_STANDBY, BLE_FLAG_ALARM_PAUSED, false));
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST, BleComm_SelectProfile(SYS_STATE_STANDBY, BLE_FLAG_SENSOR_FAULT, false));
}

void test_PolicyEscalatesImmediately_AndDownshiftIsWrapSafe(void)
{
    BleComm_Policy_t policy;
    BleComm_PolicyInit(&policy);
    uint32_t now = UINT32_MAX - 100U;
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_QUIET, BleComm_PolicyStep(&policy, BLECOMM_PROFILE_QUIET, now));
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST, BleComm_PolicyStep(&policy, BLECOMM_PROFILE_FAST, now + 10U));
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_FAST, BleComm_PolicyStep(&policy, BLECOMM_PROFILE_QUIET, now + 1009U));
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_QUIET, BleComm_PolicyStep(&policy, BLECOMM_PROFILE_QUIET, now + 1010U));
    TEST_ASSERT_EQUAL(BLECOMM_PROFILE_ACTIVE, BleComm_PolicyStep(&policy, BLECOMM_PROFILE_ACTIVE, now + 1020U));
}

void test_ControllerStartupCachesLatestState_AndProgramsRealIntervalUnits(void)
{
    TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_STANDBY, false));
    Service(0U);
    TEST_ASSERT_EQUAL_UINT32(0U, MockBleRadio_GetState()->data_count);
    MockBleRadio_SetReady(true);
    TEST_ASSERT_EQUAL_HEX16(0x0140U, MockBleRadio_GetState()->settings.advertising_units);
    TEST_ASSERT_EQUAL_HEX16(0x0080U, MockBleRadio_GetState()->settings.scan_window_units);

    s_packet.state_flags = BLE_FLAG_FALL_ALERT;
    TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_FALL_CONFIRMED, false));
    Service(10U);
    TEST_ASSERT_EQUAL_HEX16(0x00A0U, MockBleRadio_GetState()->settings.advertising_units);
    TEST_ASSERT_EQUAL_HEX16(0x00A0U, MockBleRadio_GetState()->settings.scan_window_units);

    s_packet.state_flags = 0U;
    TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_ACTIVE, false));
    Service(20U);
    TEST_ASSERT_EQUAL_HEX16(0x00A0U, MockBleRadio_GetState()->settings.advertising_units);
    Service(1010U);
    TEST_ASSERT_EQUAL_HEX16(0x00F0U, MockBleRadio_GetState()->settings.advertising_units);
}

void test_PayloadUpdatesDoNotReconfigureRadio_AndRestartRestoresConfiguration(void)
{
    TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_ACTIVE, false));
    Service(0U);
    MockBleRadio_SetReady(true);
    uint32_t configurations = MockBleRadio_GetState()->configure_count;
    s_packet.heartbeat_seq = 1U;
    TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_ACTIVE, false));
    Service(50U);
    TEST_ASSERT_EQUAL_UINT32(configurations, MockBleRadio_GetState()->configure_count);
    MockBleRadio_SetReady(false);
    MockBleRadio_SetReady(true);
    TEST_ASSERT_EQUAL_UINT32(configurations + 1U, MockBleRadio_GetState()->configure_count);
}

void test_FullQueueReportsBackpressure_AndWorkerCoalescesCompleteSnapshots(void)
{
    for (uint8_t i = 0U; i < (BLECOMM_CFG_TX_QUEUE_SIZE - 1U); ++i)
    {
        s_packet.heartbeat_seq = i;
        TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_STANDBY, false));
    }
    TEST_ASSERT_FALSE(BleComm_Publish(&s_packet, SYS_STATE_ACTIVE, false));
    Service(0U);
    MockBleRadio_SetReady(true);
    const MockBleRadio_State_t * const p_radio = MockBleRadio_GetState();
    TEST_ASSERT_EQUAL_UINT32(1U, p_radio->data_count);
    Ble_RawPacket_t raw;
    memcpy(&raw, &p_radio->data[p_radio->data_len - sizeof(raw)], sizeof(raw));
    Ble_AdvancedThreatPacket_t decoded;
    TEST_ASSERT_TRUE(BleComm_Decode(&raw, BLE_DEFAULT_PAIR_ID, &decoded));
    TEST_ASSERT_EQUAL_UINT8(BLECOMM_CFG_TX_QUEUE_SIZE - 2U, decoded.heartbeat_seq);
    TEST_ASSERT_TRUE(BleComm_Publish(&s_packet, SYS_STATE_ACTIVE, false));
}

void test_FrozenPeerAdvertisementsDoNotRefreshDeadman_AfterSequenceRebase(void)
{
    s_packet.seq_num = 1U;
    TEST_ASSERT_TRUE(BleComm_ReceivePacketAt(&s_packet, 100U));
    s_packet.seq_num = 7U; /* Delayed reception crosses the three-bit ordering window. */
    TEST_ASSERT_FALSE(BleComm_ReceivePacketAt(&s_packet, 110U));
    TEST_ASSERT_TRUE(BleComm_ReceivePacketAt(&s_packet, 400U));
    TEST_ASSERT_FALSE(BleComm_ReceivePacketAt(&s_packet, 1000U));
    TEST_ASSERT_FALSE(BleComm_ReceivePacketAt(&s_packet, 2000U));
    Ble_AdvancedThreatPacket_t received;
    uint32_t received_ms;
    TEST_ASSERT_TRUE(BleComm_PopReceivedPacket(&received, &received_ms));
    TEST_ASSERT_EQUAL_UINT32(100U, received_ms);
    TEST_ASSERT_TRUE(BleComm_PopReceivedPacket(&received, &received_ms));
    TEST_ASSERT_EQUAL_UINT32(400U, received_ms);
    Watchdog_KickBleDeadman(received_ms);
    TEST_ASSERT_FALSE(Watchdog_CheckBleLinkLoss(899U));
    TEST_ASSERT_TRUE(Watchdog_CheckBleLinkLoss(900U));
}

void test_RxQueueFull_DoesNotConsumeSequenceBeforeDelivery(void)
{
    for (uint16_t i = 0U; i < (BLECOMM_CFG_RX_QUEUE_SIZE - 1U); ++i)
    {
        s_packet.seq_num = (uint16_t)(20U + i);
        TEST_ASSERT_TRUE(BleComm_ReceivePacketAt(&s_packet, i * 100U));
    }
    s_packet.seq_num = 100U;
    TEST_ASSERT_FALSE(BleComm_ReceivePacketAt(&s_packet, 1000U));
    Ble_AdvancedThreatPacket_t received;
    TEST_ASSERT_TRUE(BleComm_PopReceivedPacket(&received, NULL));
    TEST_ASSERT_TRUE(BleComm_ReceivePacketAt(&s_packet, 1010U));
}

void test_WatchdogRequiresActualWorkerAndPerceptionProgress(void)
{
    Service(0U);
    MockBleRadio_SetReady(true);
    Watchdog_SetCore0Alive();
    TEST_ASSERT_TRUE(BleComm_Core1_Task());
    TEST_ASSERT_TRUE(Watchdog_FeedHardware());
    TEST_ASSERT_FALSE(Watchdog_FeedHardware());
    MockBleRadio_SetWorkerRunning(false);
    Watchdog_SetCore0Alive();
    TEST_ASSERT_FALSE(BleComm_Core1_Task());
    MockBleRadio_SetWorkerRunning(true);
    TEST_ASSERT_TRUE(BleComm_Core1_Task());
    TEST_ASSERT_TRUE(Watchdog_FeedHardware());
}

void test_AdvertisingParserRejectsTruncationWrongPairAndOwnNode(void)
{
    Service(0U);
    uint8_t ad[10] = {9U, 0xFFU, 0x29U, 0U};
    Ble_RawPacket_t raw;
    BleComm_Encode(&s_packet, &raw);
    memcpy(&ad[4], &raw, sizeof(raw));
    MockBleRadio_Receive(ad, sizeof(ad));
    Ble_AdvancedThreatPacket_t received;
    TEST_ASSERT_FALSE(BleComm_PopReceivedPacket(&received, NULL)); /* own node */
    s_packet.node_type = BLE_NODE_TYPE_GLASSES;
    s_packet.pair_id = 2U;
    BleComm_Encode(&s_packet, &raw);
    memcpy(&ad[4], &raw, sizeof(raw));
    MockBleRadio_Receive(ad, sizeof(ad));
    TEST_ASSERT_FALSE(BleComm_PopReceivedPacket(&received, NULL));
    s_packet.pair_id = BLE_DEFAULT_PAIR_ID;
    BleComm_Encode(&s_packet, &raw);
    memcpy(&ad[4], &raw, sizeof(raw));
    MockBleRadio_Receive(ad, 9U);
    TEST_ASSERT_FALSE(BleComm_PopReceivedPacket(&received, NULL));
    MockBleRadio_SetTime(1234U);
    MockBleRadio_Receive(ad, sizeof(ad));
    uint32_t time;
    TEST_ASSERT_TRUE(BleComm_PopReceivedPacket(&received, &time));
    TEST_ASSERT_EQUAL_UINT32(1234U, time);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_AllSystemStates_HaveExplicitRadioPolicies);
    RUN_TEST(test_PolicyEscalatesImmediately_AndDownshiftIsWrapSafe);
    RUN_TEST(test_ControllerStartupCachesLatestState_AndProgramsRealIntervalUnits);
    RUN_TEST(test_PayloadUpdatesDoNotReconfigureRadio_AndRestartRestoresConfiguration);
    RUN_TEST(test_FullQueueReportsBackpressure_AndWorkerCoalescesCompleteSnapshots);
    RUN_TEST(test_FrozenPeerAdvertisementsDoNotRefreshDeadman_AfterSequenceRebase);
    RUN_TEST(test_RxQueueFull_DoesNotConsumeSequenceBeforeDelivery);
    RUN_TEST(test_WatchdogRequiresActualWorkerAndPerceptionProgress);
    RUN_TEST(test_AdvertisingParserRejectsTruncationWrongPairAndOwnNode);
    return UNITY_END();
}
