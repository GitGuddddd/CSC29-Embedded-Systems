#include "unity.h"
#include "drivers/Ld2450/Ld2450.h"
#include "drivers/Ld2450/Ld2450_Cfg.h"
#include "hal/Hal_Uart.h"
#include "hal/Hal_Uart_Cfg.h"
#include <string.h>

void setUp(void)
{
    TEST_ASSERT_TRUE(Ld2450_Init(&Ld2450_Config_Glasses));
}

void tearDown(void)
{
}

void test_Ld2450_ParseValidFrame(void)
{
    /* 30-byte valid frame: Header AA FF 03 00, 3 targets of 8 bytes, Footer 55 CC */
    uint8_t frame[30] = {0};
    frame[0] = 0xAAU;
    frame[1] = 0xFFU;
    frame[2] = 0x03U;
    frame[3] = 0x00U;

    /* Target 1: x=0, y=+1500mm (0x8000 | 1500), speed=+25 cm/s (0x8000 | 25), dist_res=0 */
    frame[4] = 0x00; frame[5] = 0x00; /* x */
    uint16_t y_raw = (uint16_t)(0x8000U | 1500U);
    uint16_t s_raw = (uint16_t)(0x8000U | 25U);
    frame[6] = (uint8_t)(y_raw & 0xFFU); frame[7] = (uint8_t)(y_raw >> 8); /* y = +1500 */
    frame[8] = (uint8_t)(s_raw & 0xFFU); frame[9] = (uint8_t)(s_raw >> 8); /* speed = +25 */
    frame[10] = 0; frame[11] = 0;

    /* Footer at 28, 29 */
    frame[28] = 0x55U;
    frame[29] = 0xCCU;

    Radar_Target_t targets[3];
    uint8_t count = 0U;

    bool ok = Ld2450_ParseFrame(frame, 30U, targets, &count);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(3U, count); /* every radar slot is returned */
    TEST_ASSERT_TRUE(targets[0].is_valid);
    TEST_ASSERT_EQUAL_UINT16(1500U, targets[0].distance_mm);
    TEST_ASSERT_EQUAL_INT16(25, targets[0].speed_cms);
    TEST_ASSERT_FALSE(targets[1].is_valid);
    TEST_ASSERT_FALSE(targets[2].is_valid);
}

void test_Ld2450_RejectsCorruptFooter(void)
{
    uint8_t frame[30] = {0};
    frame[0] = 0xAAU; frame[1] = 0xFFU; frame[2] = 0x03U; frame[3] = 0x00U;
    frame[28] = 0x00U; frame[29] = 0x00U; /* Bad footer */

    Radar_Target_t targets[3];
    uint8_t count = 0U;

    bool ok = Ld2450_ParseFrame(frame, 30U, targets, &count);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT8(0U, count);
}

static void MakeFrame(uint8_t * const p_frame, uint16_t distance_mm)
{
    memset(p_frame, 0, LD2450_CFG_FRAME_BYTES);
    p_frame[0] = 0xAAU;
    p_frame[1] = 0xFFU;
    p_frame[2] = 0x03U;
    uint16_t y_raw = (distance_mm > 0U) ? (uint16_t)(0x8000U | distance_mm) : 0U;
    p_frame[6] = (uint8_t)(y_raw & 0xFFU);
    p_frame[7] = (uint8_t)(y_raw >> 8);
    p_frame[28] = 0x55U;
    p_frame[29] = 0xCCU;
}

void test_StreamWaitsForCompleteFrame_AndReturnsLatestFrame(void)
{
    uint8_t frame[30];
    MakeFrame(frame, 1500U);
    Radar_Target_t targets[3];
    uint8_t count;
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, 10U));
    TEST_ASSERT_FALSE(Ld2450_ProcessStream(targets, &count, 20U));
    TEST_ASSERT_EQUAL_UINT8(0U, count);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(&frame[10], 20U));
    MakeFrame(frame, 900U);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 40U));
    TEST_ASSERT_EQUAL_UINT8(3U, count);
    TEST_ASSERT_TRUE(targets[0].is_valid);
    TEST_ASSERT_EQUAL_UINT16(900U, targets[0].distance_mm);
    MakeFrame(frame, 0U); /* A complete no-target frame is healthy data, not a timeout. */
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 60U));
    TEST_ASSERT_EQUAL_UINT8(3U, count);
    TEST_ASSERT_FALSE(targets[0].is_valid);
    TEST_ASSERT_FALSE(targets[1].is_valid);
    TEST_ASSERT_FALSE(targets[2].is_valid);
    TEST_ASSERT_FALSE(Ld2450_HasRxFault());
}

void test_StreamResynchronizesAfterNoise_AndAcrossBufferBoundary(void)
{
    uint8_t noise[LD2450_CFG_CIRC_BUF_SIZE - 4U] = {0};
    Radar_Target_t targets[3];
    uint8_t count;
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(noise, sizeof(noise)));
    (void)Ld2450_ProcessStream(targets, &count, 20U);
    (void)Ld2450_ProcessStream(targets, &count, 40U);
    (void)Ld2450_ProcessStream(targets, &count, 60U);
    uint8_t frame[30];
    MakeFrame(frame, 1234U);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 80U));
    TEST_ASSERT_EQUAL_UINT16(1234U, targets[0].distance_mm);
    TEST_ASSERT_FALSE(Ld2450_HasRxFault());
}

void test_OverrunIsNotMistakenForEmpty_AndFreshFramesRecover(void)
{
    uint8_t noise[((LD2450_CFG_CIRC_BUF_SIZE / HAL_UART_DMA_BLOCK_BYTES) + 1U) * HAL_UART_DMA_BLOCK_BYTES] = {0};
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(noise, sizeof(noise)));
    Radar_Target_t targets[3];
    uint8_t count;
    TEST_ASSERT_FALSE(Ld2450_ProcessStream(targets, &count, 20U));
    TEST_ASSERT_TRUE(Ld2450_HasRxFault());
    TEST_ASSERT_EQUAL_UINT32(1U, Ld2450_GetOverrunCount());
    uint8_t frame[30];
    MakeFrame(frame, 1000U);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 40U));
    TEST_ASSERT_FALSE(Ld2450_HasRxFault());
}

void test_ProducedCounterWrapIsIndependentOfRingWrap(void)
{
    Hal_Uart_TestSetProduced(UINT32_MAX - 9U);
    Radar_Target_t targets[3];
    uint8_t count;
    (void)Ld2450_ProcessStream(targets, &count, 20U); /* Drop the simulated old backlog. */
    uint8_t frame[30];
    MakeFrame(frame, 1400U);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    Hal_Uart_RxSnapshot_t rx;
    TEST_ASSERT_TRUE(Hal_Uart_GetRxSnapshot(&rx));
    TEST_ASSERT_EQUAL_UINT32(20U, rx.produced_bytes);
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 40U));
    TEST_ASSERT_EQUAL_UINT16(1400U, targets[0].distance_mm);
}

void test_UnalignedFrameWaitsForCommittedDmaTail(void)
{
    uint8_t noise = 0U;
    uint8_t frame[30];
    uint8_t padding[29] = {0};
    MakeFrame(frame, 1700U);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(&noise, 1U));
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    Radar_Target_t targets[3];
    uint8_t count;
    TEST_ASSERT_FALSE(Ld2450_ProcessStream(targets, &count, 20U));
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(padding, sizeof(padding)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 40U));
    TEST_ASSERT_EQUAL_UINT16(1700U, targets[0].distance_mm);
}

void test_LatePollingDoesNotRefreshThreatWithStaleBufferedFrame(void)
{
    Radar_Target_t targets[3];
    uint8_t count;
    (void)Ld2450_ProcessStream(targets, &count, 0U);
    uint8_t frame[30];
    MakeFrame(frame, 800U);
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_FALSE(Ld2450_ProcessStream(targets, &count, 200U));
    TEST_ASSERT_TRUE(Ld2450_HasRxFault());
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 220U));
}

void test_DmaFaultAndInvalidConfigurationAreReported(void)
{
    _Alignas(HAL_UART_DMA_RING_BYTES) uint8_t buffer[HAL_UART_DMA_RING_BYTES];
    TEST_ASSERT_FALSE(Hal_Uart_DmaInit(0U, NULL, sizeof(buffer)));
    TEST_ASSERT_FALSE(Hal_Uart_DmaInit(0U, &buffer[1], sizeof(buffer)));
    TEST_ASSERT_FALSE(Hal_Uart_DmaInit(0U, buffer, 256U));
    TEST_ASSERT_FALSE(Hal_Uart_DmaInit(2U, buffer, sizeof(buffer)));
    Hal_Uart_TestSetDmaFault(true);
    Radar_Target_t targets[3];
    uint8_t count;
    TEST_ASSERT_FALSE(Ld2450_ProcessStream(targets, &count, 20U));
    TEST_ASSERT_TRUE(Ld2450_HasRxFault());
    TEST_ASSERT_TRUE(Ld2450_Init(&Ld2450_Config_Glasses));
    TEST_ASSERT_TRUE(Ld2450_HasRxFault()); /* Configuration success alone is not a fresh frame. */
}

void test_PartialHeadersAndBadFooterCannotPublishAnUninitializedBody(void)
{
    uint8_t stream[180] = {0};
    for (uint32_t offset = 0U; offset < 90U; offset += 5U)
    {
        stream[offset] = 0xAAU;
        stream[offset + 1U] = 0xFFU;
        stream[offset + 2U] = 0x03U;
        stream[offset + 3U] = 0x01U; /* Almost a header: never read as a frame body. */
    }
    MakeFrame(&stream[90], 600U);
    stream[118] = 0U; /* Valid header with corrupt footer still needs full validation. */
    MakeFrame(&stream[120], 1300U);
    Radar_Target_t targets[3];
    uint8_t count;
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(stream, sizeof(stream)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 20U));
    TEST_ASSERT_EQUAL_UINT8(3U, count);
    TEST_ASSERT_TRUE(targets[0].is_valid);
    TEST_ASSERT_EQUAL_UINT16(1300U, targets[0].distance_mm);
    TEST_ASSERT_FALSE(Ld2450_HasRxFault());
    TEST_ASSERT_EQUAL_UINT32(0U, Ld2450_GetOverrunCount());
}

void test_Ld2450_DecodesLateralXAndEuclideanDistance(void)
{
    uint8_t frame[30] = {0};
    frame[0] = 0xAAU;
    frame[1] = 0xFFU;
    frame[2] = 0x03U;
    frame[3] = 0x00U;

    /* Target 1: x = -300 mm (300U), y = +400 mm (0x8000 | 400) -> dist = 500 mm, speed = +25 cm/s (0x8000 | 25) */
    uint16_t x1_raw = 300U;
    uint16_t y1_raw = (uint16_t)(0x8000U | 400U);
    uint16_t s1_raw = (uint16_t)(0x8000U | 25U);
    frame[4] = (uint8_t)(x1_raw & 0xFFU);
    frame[5] = (uint8_t)(x1_raw >> 8);
    frame[6] = (uint8_t)(y1_raw & 0xFFU);
    frame[7] = (uint8_t)(y1_raw >> 8);
    frame[8] = (uint8_t)(s1_raw & 0xFFU);
    frame[9] = (uint8_t)(s1_raw >> 8);

    /* Target 2: x = +600 mm (0x8000 | 600), y = +800 mm (0x8000 | 800) -> dist = 1000 mm, speed = -15 cm/s (15U) */
    uint16_t x2_raw = (uint16_t)(0x8000U | 600U);
    uint16_t y2_raw = (uint16_t)(0x8000U | 800U);
    uint16_t s2_raw = 15U;
    frame[12] = (uint8_t)(x2_raw & 0xFFU);
    frame[13] = (uint8_t)(x2_raw >> 8);
    frame[14] = (uint8_t)(y2_raw & 0xFFU);
    frame[15] = (uint8_t)(y2_raw >> 8);
    frame[16] = (uint8_t)(s2_raw & 0xFFU);
    frame[17] = (uint8_t)(s2_raw >> 8);

    frame[28] = 0x55U;
    frame[29] = 0xCCU;

    Radar_Target_t targets[3];
    uint8_t count = 0U;
    TEST_ASSERT_TRUE(Ld2450_ParseFrame(frame, sizeof(frame), targets, &count));
    TEST_ASSERT_EQUAL_UINT8(3U, count);
    TEST_ASSERT_FALSE(targets[2].is_valid);

    TEST_ASSERT_TRUE(targets[0].is_valid);
    TEST_ASSERT_EQUAL_INT16(-300, targets[0].x_mm);
    TEST_ASSERT_EQUAL_UINT16(500U, targets[0].distance_mm);
    TEST_ASSERT_EQUAL_INT16(25, targets[0].speed_cms);

    TEST_ASSERT_TRUE(targets[1].is_valid);
    TEST_ASSERT_EQUAL_INT16(600, targets[1].x_mm);
    TEST_ASSERT_EQUAL_UINT16(1000U, targets[1].distance_mm);
    TEST_ASSERT_EQUAL_INT16(-15, targets[1].speed_cms);
}

/* An empty radar slot must not shift later targets down: ThreatAssessment keeps
 * per-slot tracking history, so a shifted target would inherit another person's. */
void test_Ld2450_EmptySlotKeepsLaterTargetsInTheirOwnSlot(void)
{
    uint8_t frame[30];
    MakeFrame(frame, 0U); /* slot 0 empty */
    const uint16_t y1_raw = (uint16_t)(0x8000U | 2400U);
    const uint16_t x1_raw = (uint16_t)(0x8000U | 900U);
    frame[12] = (uint8_t)(x1_raw & 0xFFU); frame[13] = (uint8_t)(x1_raw >> 8);
    frame[14] = (uint8_t)(y1_raw & 0xFFU); frame[15] = (uint8_t)(y1_raw >> 8);

    Radar_Target_t targets[3];
    uint8_t count = 0U;
    TEST_ASSERT_TRUE(Ld2450_ParseFrame(frame, sizeof(frame), targets, &count));
    TEST_ASSERT_EQUAL_UINT8(3U, count);
    TEST_ASSERT_FALSE(targets[0].is_valid);
    TEST_ASSERT_TRUE(targets[1].is_valid);
    TEST_ASSERT_EQUAL_INT16(900, targets[1].x_mm);
    TEST_ASSERT_FALSE(targets[2].is_valid);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Ld2450_ParseValidFrame);
    RUN_TEST(test_Ld2450_RejectsCorruptFooter);
    RUN_TEST(test_StreamWaitsForCompleteFrame_AndReturnsLatestFrame);
    RUN_TEST(test_StreamResynchronizesAfterNoise_AndAcrossBufferBoundary);
    RUN_TEST(test_OverrunIsNotMistakenForEmpty_AndFreshFramesRecover);
    RUN_TEST(test_ProducedCounterWrapIsIndependentOfRingWrap);
    RUN_TEST(test_UnalignedFrameWaitsForCommittedDmaTail);
    RUN_TEST(test_LatePollingDoesNotRefreshThreatWithStaleBufferedFrame);
    RUN_TEST(test_DmaFaultAndInvalidConfigurationAreReported);
    RUN_TEST(test_PartialHeadersAndBadFooterCannotPublishAnUninitializedBody);
    RUN_TEST(test_Ld2450_DecodesLateralXAndEuclideanDistance);
    RUN_TEST(test_Ld2450_EmptySlotKeepsLaterTargetsInTheirOwnSlot);
    return UNITY_END();
}
