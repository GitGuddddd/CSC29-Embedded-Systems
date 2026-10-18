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
    Ble_AdvancedThreatPacket_t pkt;
    pkt.preamble = 0xAAU;
    pkt.device_id = 0x02U;
    pkt.seq_num = 1U;
    pkt.state_flags = (fall_detected) ? 0x01U : 0x00U;
    pkt.nearest_dist_mm = 0U;
    pkt.closing_speed_cms = 0;
    pkt.ttc_ms = 0U;

    bool tx_ok = BleComm_SendPacket(&pkt);
    TEST_ASSERT_TRUE(tx_ok);

    /* Verify Receiver side drops duplicate sequence numbers */
    bool rx_ok = BleComm_ReceivePacket(&pkt);
    TEST_ASSERT_TRUE(rx_ok);

    /* Verify packet can be popped from RX queue */
    Ble_AdvancedThreatPacket_t popped_pkt;
    bool pop_ok = BleComm_PopReceivedPacket(&popped_pkt);
    TEST_ASSERT_TRUE(pop_ok);
    TEST_ASSERT_EQUAL_HEX8(0x02U, popped_pkt.device_id);
    TEST_ASSERT_EQUAL_HEX8(0x01U, popped_pkt.state_flags & 0x01U);

    /* Queue should now be empty */
    TEST_ASSERT_FALSE(BleComm_PopReceivedPacket(&popped_pkt));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Integration_FallToBleDispatch);
    return UNITY_END();
}
