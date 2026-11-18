#include "unity.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "app/ThreatAssessment/ThreatAssessment_Cfg.h"

void setUp(void)
{
    ThreatAssessment_Init();
}

void tearDown(void)
{
}

void test_ThreatAssessment_IgnoresDepartingTargets(void)
{
    Radar_Target_t targets[1] = {0};
    targets[0].is_valid = true;
    targets[0].distance_mm = 1500U;
    targets[0].speed_cms = -20; /* Departing fast (-20 < -10) */

    Threat_Result_t res;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);

    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_Hysteresis_EnterAndClear(void)
{
    Radar_Target_t targets[1] = {0};
    targets[0].is_valid = true;
    targets[0].speed_cms = 15; /* Approaching */

    Threat_Result_t res;

    /* 1. At 2050mm (Outside enter threshold of 2000mm) -> Inactive */
    targets[0].distance_mm = 2050U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);

    /* 2. Moves closer: 1950mm (< 2000mm) -> Active */
    targets[0].distance_mm = 1950U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* 3. Moves back to 2050mm (In hysteresis zone: > 2000mm but <= 2100mm) -> Remains Active! */
    targets[0].distance_mm = 2050U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* 4. Moves past clear threshold: 2150mm (> 2100mm) -> Inactive */
    targets[0].distance_mm = 2150U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_FR6_GripGating(void)
{
    Radar_Target_t targets[1] = {0};
    targets[0].is_valid = true;
    targets[0].distance_mm = 1200U;
    targets[0].speed_cms = 10;

    Threat_Result_t res;

    /* Active while held */
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* Suppressed when explicitly confirmed RELEASED */
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_RELEASED, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);

    /* Fail-safe active when UNKNOWN (link dropout) */
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_UNKNOWN, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
}

void test_ThreatAssessment_TtcIsBoundedInWireUnits(void)
{
    Radar_Target_t target = {.is_valid = true, .distance_mm = 2000U, .speed_cms = 1};
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_TRUE(result.is_threat_active); /* Unavailable TTC does not suppress the obstacle. */
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);

    target.distance_mm = 4094U;
    target.speed_cms = 5;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_MAX_TICKS, result.ttc_20ms);
    TEST_ASSERT_EQUAL_UINT32(81880U, (uint32_t)result.ttc_20ms * BLE_TTC_STEP_MS);

    target.distance_mm = 819U;
    target.speed_cms = 1;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms); /* 4095 is reserved. */
}

void test_ThreatAssessment_TtcMatchesWideArithmeticAcrossBoundaryInputs(void)
{
    static const uint16_t distances[] = {1U, 819U, 1000U, 2000U, 4094U, 4095U, 6000U, 65534U};
    static const int16_t speeds[] = {0, 1, 2, 5, 10, 100, 32767};
    for (uint32_t d = 0U; d < sizeof(distances) / sizeof(distances[0]); ++d)
    {
        for (uint32_t s = 0U; s < sizeof(speeds) / sizeof(speeds[0]); ++s)
        {
            Radar_Target_t target = {.is_valid = true, .distance_mm = distances[d], .speed_cms = speeds[s]};
            Threat_Result_t result;
            ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
            uint16_t expected = BLE_TTC_NO_THREAT;
            if (speeds[s] > 0)
            {
                uint64_t milliseconds = ((uint64_t)distances[d] * 100U) / (uint16_t)speeds[s];
                uint64_t ticks = milliseconds / BLE_TTC_STEP_MS;
                if (ticks <= BLE_TTC_MAX_TICKS) expected = (uint16_t)ticks;
            }
            TEST_ASSERT_EQUAL_UINT16(expected, result.ttc_20ms);
        }
    }
}

void test_ThreatAssessment_ClearUsesUnavailableSentinels(void)
{
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(NULL, 0U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_FALSE(result.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, result.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);
    Radar_Target_t target = {.is_valid = true, .distance_mm = 1500U, .speed_cms = 10};
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_RELEASED, &result);
    TEST_ASSERT_FALSE(result.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);
}

void test_ThreatAssessment_ThreeLaneOccupancy_CorridorAndSideHysteresis(void)
{
    Threat_Result_t res;

    /* 1. Corridor gate: targets outside +/- 1500 mm lateral X are ignored. */
    Radar_Target_t outside[2] = {
        { .distance_mm = 1000U, .speed_cms = 20, .x_mm = -1501, .is_valid = true },
        { .distance_mm = 1000U, .speed_cms = 20, .x_mm =  1501, .is_valid = true }
    };
    ThreatAssessment_ProcessTargetsAt(outside, 2U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT8(0U, res.open_direction_cue);

    /* 2. Target enters LEFT lane (x = -426 < -425), while CENTER is clear. */
    Radar_Target_t targets[2] = {
        { .distance_mm = 1500U, .speed_cms = 10, .x_mm = -426, .is_valid = true },
        { .distance_mm = 1600U, .speed_cms = 10, .x_mm =    0, .is_valid = true }
    };
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 20U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    /* Left + Center blocked -> open direction is RIGHT */
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);

    /* 3. Slot 0 drifts inward to x = -400 (between -425 and -350 hysteresis band).
     *    Side hysteresis keeps slot 0 in LEFT; both LEFT and CENTER remain blocked.
     */
    targets[0].x_mm = -400;
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 40U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);

    /* 4. Slot 0 crosses inner release boundary x = -349 (> -350) -> moves to CENTER.
     *    After 200 ms hold expires on LEFT, only CENTER is blocked -> cue defaults LEFT.
     */
    targets[0].x_mm = -349;
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 60U, &res);
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 260U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_LEFT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);
}

void test_ThreatAssessment_LaneHoldBridgesBriefDropout(void)
{
    Threat_Result_t res;
    Radar_Target_t target = { .distance_mm = 1500U, .speed_cms = 20, .x_mm = 0, .is_valid = true };

    /* Frame 1: target inside enter threshold -> lane blocked */
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1500U, res.nearest_distance_mm);

    /* Frame 2 at t=50 ms: target vanishes -> hold starts, lane stays blocked */
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 50U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1500U, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);

    /* Frame 3 at t=249 ms (199 ms elapsed): still held */
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 249U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1500U, res.nearest_distance_mm);

    /* Frame 4 at t=250 ms (200 ms elapsed): hold expires, lane clears */
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 250U, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);

    /* Reappearance within hold cancels pending clear and restarts timer */
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 300U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 350U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    target.distance_mm = 1600U;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 400U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1600U, res.nearest_distance_mm);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 550U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 749U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 750U, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_TtcHoldSmoothsBriefSpeedDropout(void)
{
    Threat_Result_t res;
    Radar_Target_t target = { .distance_mm = 1500U, .speed_cms = 20, .x_mm = 0, .is_valid = true };

    /* Live closing speed 20 cm/s -> TTC = 1500 * 5 / 20 = 375 ticks */
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(375U, res.ttc_20ms);

    /* Speed drops to 0 cm/s at t=50 while target remains live at 1400 mm -> reuse 20 cm/s */
    target.distance_mm = 1400U;
    target.speed_cms = 0;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 50U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1400U, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_INT16(20, res.closing_speed_cms);
    TEST_ASSERT_EQUAL_UINT16(350U, res.ttc_20ms);

    /* At t=250 ms (200 ms elapsed <= 200 ms): still smoothed */
    target.distance_mm = 1300U;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 250U, &res);
    TEST_ASSERT_EQUAL_UINT16(325U, res.ttc_20ms);

    /* At t=251 ms (201 ms elapsed > 200 ms): speed hold expires -> TTC unavailable */
    target.distance_mm = 1280U;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 251U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);

    /* Full occupancy gap clears held speed so reappearance with speed=0 does not reuse it */
    target.distance_mm = 1500U;
    target.speed_cms = 20;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 300U, &res);
    TEST_ASSERT_EQUAL_UINT16(375U, res.ttc_20ms);

    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 350U, &res);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);

    target.distance_mm = 1400U;
    target.speed_cms = 0;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 400U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);
}

void test_ThreatAssessment_OpenDirectionCueTruthTable(void)
{
    typedef struct
    {
        bool left;
        bool center;
        bool right;
        bool expect_active;
        uint8_t expect_cue;
    } CueRow_t;

    static const CueRow_t rows[8] = {
        { false, false, false, false, 0U },
        { true,  false, false, true,  0U },
        { false, false, true,  true,  0U },
        { true,  false, true,  true,  0U },
        { false, true,  false, true,  (uint8_t)(THREAT_LANE_LEFT  << THREAT_CUE_SIDE_SHIFT) },
        { false, true,  true,  true,  (uint8_t)(THREAT_LANE_LEFT  << THREAT_CUE_SIDE_SHIFT) },
        { true,  true,  false, true,  (uint8_t)(THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT) },
        { true,  true,  true,  true,  THREAT_CUE_STOP }
    };

    for (uint8_t r = 0U; r < 8U; r++)
    {
        ThreatAssessment_Init();
        Radar_Target_t targets[3] = {
            { .distance_mm = rows[r].left   ? 1500U : 2500U, .speed_cms = 0, .x_mm = -600, .is_valid = true },
            { .distance_mm = rows[r].center ? 1500U : 2500U, .speed_cms = 0, .x_mm =    0, .is_valid = true },
            { .distance_mm = rows[r].right  ? 1500U : 2500U, .speed_cms = 0, .x_mm =  600, .is_valid = true }
        };
        Threat_Result_t res;
        ThreatAssessment_ProcessTargetsAt(targets, 3U, GRIP_STATE_HELD, 0U, &res);
        TEST_ASSERT_EQUAL(rows[r].expect_active, (bool)res.is_threat_active);
        TEST_ASSERT_EQUAL_UINT8(rows[r].expect_cue, (uint8_t)res.open_direction_cue);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ThreatAssessment_IgnoresDepartingTargets);
    RUN_TEST(test_ThreatAssessment_Hysteresis_EnterAndClear);
    RUN_TEST(test_ThreatAssessment_FR6_GripGating);
    RUN_TEST(test_ThreatAssessment_TtcIsBoundedInWireUnits);
    RUN_TEST(test_ThreatAssessment_TtcMatchesWideArithmeticAcrossBoundaryInputs);
    RUN_TEST(test_ThreatAssessment_ClearUsesUnavailableSentinels);
    RUN_TEST(test_ThreatAssessment_ThreeLaneOccupancy_CorridorAndSideHysteresis);
    RUN_TEST(test_ThreatAssessment_LaneHoldBridgesBriefDropout);
    RUN_TEST(test_ThreatAssessment_TtcHoldSmoothsBriefSpeedDropout);
    RUN_TEST(test_ThreatAssessment_OpenDirectionCueTruthTable);
    return UNITY_END();
}
