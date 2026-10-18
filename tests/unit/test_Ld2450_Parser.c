#include "unity.h"
#include "drivers/Ld2450/Ld2450.h"

void setUp(void)
{
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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Ld2450_ParseValidFrame);
    RUN_TEST(test_Ld2450_RejectsCorruptFooter);
    return UNITY_END();
}
