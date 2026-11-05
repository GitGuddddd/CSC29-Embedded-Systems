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

    /* Target 1: x=0, y=1500mm, speed=25 cm/s, dist_res=0 */
    frame[4] = 0x00; frame[5] = 0x00; /* x */
    frame[6] = (uint8_t)(1500 & 0xFF); frame[7] = (uint8_t)((1500 >> 8) & 0xFF); /* y = 1500 */
    frame[8] = 25; frame[9] = 0; /* speed = 25 */
    frame[10] = 0; frame[11] = 0;

    /* Footer at 28, 29 */
    frame[28] = 0x55U;
    frame[29] = 0xCCU;

    Radar_Target_t targets[3];
    uint8_t count = 0U;

    bool ok = Ld2450_ParseFrame(frame, 30U, targets, &count);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(1U, count);
    TEST_ASSERT_EQUAL_UINT16(1500U, targets[0].distance_mm);
    TEST_ASSERT_EQUAL_INT16(25, targets[0].speed_cms);
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
    p_frame[6] = (uint8_t)(distance_mm & 0xFFU);
    p_frame[7] = (uint8_t)(distance_mm >> 8);
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
    TEST_ASSERT_EQUAL_UINT8(1U, count);
    TEST_ASSERT_EQUAL_UINT16(900U, targets[0].distance_mm);
    MakeFrame(frame, 0U); /* A complete no-target frame is healthy data, not a timeout. */
    TEST_ASSERT_TRUE(Hal_Uart_TestInjectRx(frame, sizeof(frame)));
    TEST_ASSERT_TRUE(Ld2450_ProcessStream(targets, &count, 60U));
    TEST_ASSERT_EQUAL_UINT8(0U, count);
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
    return UNITY_END();
}
