#include "unity.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "app/ThreatAssessment/ThreatAssessment_Cfg.h"

/* Approaching speed in the radar's own sign convention (see ThreatAssessment_Cfg.h). */
/* x in the user's frame: negative = left, whatever way the radar is mounted. */
#define USER_X(v) ((int16_t)(THREAT_MIRROR_X ? -(v) : (v)))
#define APPROACHING(v) ((int16_t)(THREAT_NEGATIVE_SPEED_IS_APPROACHING ? -(v) : (v)))

void setUp(void)
{
    ThreatAssessment_Init();
}

void tearDown(void)
{
}

void test_ThreatAssessment_DepartingTargetStillOccupiesLane(void)
{
    /* putra-branch OPEN-22: nothing is excluded for moving away. */
    Radar_Target_t targets[1] = {0};
    targets[0].is_valid = true;
    targets[0].distance_mm = 1500U;
    targets[0].speed_cms = APPROACHING(-20); /* departing at 20 cm/s */

    Threat_Result_t res;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);

    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1500U, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms); /* not closing -> no TTC */
}

void test_ThreatAssessment_SpeedSignConvention_PositiveIsApproaching(void)
{
    Threat_Result_t res;
    Radar_Target_t target = { .distance_mm = 1600U, .speed_cms = APPROACHING(16), .x_mm = 0, .is_valid = true };

    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_INT16(16, res.closing_speed_cms);
    TEST_ASSERT_EQUAL_UINT16(500U, res.ttc_20ms); /* 1600 * 5 / 16 */

    /* The opposite sign is a departing reading: no TTC, but the lane stays blocked. */
    ThreatAssessment_Init();
    target.speed_cms = APPROACHING(-16);
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);
}

void test_ThreatAssessment_Hysteresis_EnterAndClear(void)
{
    Radar_Target_t targets[1] = {0};
    targets[0].is_valid = true;
    targets[0].speed_cms = APPROACHING(15);

    Threat_Result_t res;

    /* 1. Just outside the enter threshold -> Inactive */
    targets[0].distance_mm = (uint16_t)(THREAT_ALERT_ENTER_DIST_MM + 50U);
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);

    /* 2. Moves closer: at the enter threshold -> Active */
    targets[0].distance_mm = THREAT_ALERT_ENTER_DIST_MM;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* 3. Back inside the hysteresis band (> enter, <= clear) -> Remains Active */
    targets[0].distance_mm = (uint16_t)(THREAT_ALERT_ENTER_DIST_MM + 50U);
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* 4. Past the clear threshold -> Inactive */
    targets[0].distance_mm = (uint16_t)(THREAT_ALERT_CLEAR_DIST_MM + 50U);
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_FR6_GripGating(void)
{
    Radar_Target_t targets[1] = {0};
    targets[0].is_valid = true;
    targets[0].distance_mm = 1200U;
    targets[0].speed_cms = APPROACHING(10);

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

void test_ThreatAssessment_ReleasedKeepsLaneOccupancyCurrent(void)
{
    /* putra-branch FR6: a confirmed release silences the alert, but the lanes
     * keep scanning, so regripping inside the hysteresis band resumes at once. */
    Radar_Target_t target = { .distance_mm = 2900U, .speed_cms = 0, .x_mm = 0, .is_valid = true };
    Threat_Result_t res;

    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    target.distance_mm = (uint16_t)(THREAT_ALERT_ENTER_DIST_MM + 50U); /* between the enter and clear distances */
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_RELEASED, 20U, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, res.nearest_distance_mm);

    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 40U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)(THREAT_ALERT_ENTER_DIST_MM + 50U), res.nearest_distance_mm);
}

void test_ThreatAssessment_TtcIsBoundedInWireUnits(void)
{
    Radar_Target_t target = {.is_valid = true, .distance_mm = 2000U, .speed_cms = APPROACHING(1)};
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_TRUE(result.is_threat_active); /* Unavailable TTC does not suppress the obstacle. */
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);

    target.distance_mm = 4094U;
    target.speed_cms = APPROACHING(5);
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_MAX_TICKS, result.ttc_20ms);
    TEST_ASSERT_EQUAL_UINT32(81880U, (uint32_t)result.ttc_20ms * BLE_TTC_STEP_MS);

    target.distance_mm = 819U;
    target.speed_cms = APPROACHING(1);
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
            Radar_Target_t target = {.is_valid = true, .distance_mm = distances[d], .speed_cms = APPROACHING(speeds[s])};
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
    Radar_Target_t target = {.is_valid = true, .distance_mm = 2000U, .speed_cms = APPROACHING(10)};
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_RELEASED, &result);
    TEST_ASSERT_FALSE(result.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);
}

void test_ThreatAssessment_ThreeLaneOccupancy_CorridorAndSideHysteresis(void)
{
    Threat_Result_t res;

    /* 1. Corridor gate: targets outside +/- 1500 mm lateral X are ignored. */
    Radar_Target_t outside[2] = {
        { .distance_mm = 1000U, .speed_cms = APPROACHING(20), .x_mm = USER_X(-1501), .is_valid = true },
        { .distance_mm = 1000U, .speed_cms = APPROACHING(20), .x_mm =  1501, .is_valid = true }
    };
    ThreatAssessment_ProcessTargetsAt(outside, 2U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT8(0U, res.open_direction_cue);

    /* 2. Target enters LEFT lane (x = -426 < -425), while CENTER is clear. */
    Radar_Target_t targets[2] = {
        { .distance_mm = 2000U, .speed_cms = APPROACHING(10), .x_mm = USER_X(-426), .is_valid = true },
        { .distance_mm = 2100U, .speed_cms = APPROACHING(10), .x_mm =    0, .is_valid = true }
    };
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 20U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    /* Left + Center blocked -> open direction is RIGHT */
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);

    /* 3. Slot 0 drifts inward to x = -400 (inside the hysteresis band below the deadzone).
     *    Side hysteresis keeps slot 0 in LEFT; both LEFT and CENTER remain blocked.
     */
    targets[0].x_mm = USER_X(-400);
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 40U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);

    /* 4. Slot 0 crosses the inner release boundary -> moves to CENTER.
     *    After the lane hold expires on LEFT, only CENTER is blocked. LEFT has only
     *    just cleared, so the confirmed-free RIGHT lane is offered first ...
     */
    targets[0].x_mm = USER_X(-(THREAT_SIDE_DEADZONE_MM - THREAT_SIDE_HYSTERESIS_MM - 1));
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, 60U, &res);
    /* The target was standing still in LEFT, so LEFT gets the still-person
     * hold. */
    const uint32_t left_cleared_ms = 60U + THREAT_STILL_LANE_HOLD_MS;
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD, left_cleared_ms, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);
    /* ... and once LEFT has stayed clear long enough, the cue defaults to LEFT. */
    ThreatAssessment_ProcessTargetsAt(targets, 2U, GRIP_STATE_HELD,
                                      left_cleared_ms + THREAT_FREE_LANE_CLEAR_MS, &res);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_LEFT << THREAT_CUE_SIDE_SHIFT),
                            res.open_direction_cue);
}

void test_ThreatAssessment_LaneHoldBridgesBriefDropout(void)
{
    Threat_Result_t res;
    Radar_Target_t target = { .distance_mm = 2000U, .speed_cms = APPROACHING(20), .x_mm = 0, .is_valid = true };

    /* Frame 1: target inside enter threshold -> lane blocked */
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(2000U, res.nearest_distance_mm);

    /* Frame 2 at t=50 ms: target vanishes -> hold starts, lane stays blocked */
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 50U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(2000U, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);

    /* Frame 3 one ms before the hold expires: still held */
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 50U + THREAT_LANE_HOLD_MS - 1U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(2000U, res.nearest_distance_mm);

    /* Frame 4: hold expires, lane clears */
    const uint32_t hold_end_ms = 50U + THREAT_LANE_HOLD_MS;
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, hold_end_ms, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, res.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, res.ttc_20ms);

    /* Reappearance within hold cancels pending clear and restarts timer */
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, hold_end_ms + 50U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, hold_end_ms + 100U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    target.distance_mm = 2100U;
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 400U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(2100U, res.nearest_distance_mm);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 550U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 550U + THREAT_LANE_HOLD_MS - 1U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    ThreatAssessment_ProcessTargetsAt(NULL, 0U, GRIP_STATE_HELD, 550U + THREAT_LANE_HOLD_MS, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_TtcHoldSmoothsBriefSpeedDropout(void)
{
    Threat_Result_t res;
    Radar_Target_t target = { .distance_mm = 1500U, .speed_cms = APPROACHING(20), .x_mm = 0, .is_valid = true };

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
    target.speed_cms = APPROACHING(20);
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

void test_ThreatAssessment_TtcHoldTreatsDepartingReadingAsDropout(void)
{
    Threat_Result_t res;
    Radar_Target_t target = { .distance_mm = 1500U, .speed_cms = APPROACHING(20), .x_mm = 0, .is_valid = true };

    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_EQUAL_UINT16(375U, res.ttc_20ms);

    /* One frame reads as departing (wrong sign) mid-approach: held speed is reused. */
    target.distance_mm = 1400U;
    target.speed_cms = APPROACHING(-5);
    ThreatAssessment_ProcessTargetsAt(&target, 1U, GRIP_STATE_HELD, 90U, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(350U, res.ttc_20ms);
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
        { true,  true,  true,  true,  (uint8_t)(THREAT_CUE_SIDE_ALL_BLOCKED << THREAT_CUE_SIDE_SHIFT) } /* all blocked, far: no STOP */
    };

    for (uint8_t r = 0U; r < 8U; r++)
    {
        ThreatAssessment_Init();
        Radar_Target_t targets[3] = {
            { .distance_mm = rows[r].left   ? 2000U : 3500U, .speed_cms = 0, .x_mm = USER_X(-600), .is_valid = true },
            { .distance_mm = rows[r].center ? 2000U : 3500U, .speed_cms = 0, .x_mm =    0, .is_valid = true },
            { .distance_mm = rows[r].right  ? 2000U : 3500U, .speed_cms = 0, .x_mm = USER_X(600), .is_valid = true }
        };
        Threat_Result_t res;
        ThreatAssessment_ProcessTargetsAt(targets, 3U, GRIP_STATE_HELD, 0U, &res);
        TEST_ASSERT_EQUAL(rows[r].expect_active, (bool)res.is_threat_active);
        TEST_ASSERT_EQUAL_UINT8(rows[r].expect_cue, (uint8_t)res.open_direction_cue);
    }
}

static uint8_t Run_Cue(const Radar_Target_t * p_targets, uint8_t n, uint32_t t_ms)
{
    Threat_Result_t res;
    ThreatAssessment_ProcessTargetsAt(p_targets, n, GRIP_STATE_HELD, t_ms, &res);
    return (uint8_t)res.open_direction_cue;
}

void test_ThreatAssessment_StopWhenCenterLaneBlockedAndTooClose(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 2000U, .speed_cms = 0, .x_mm = 0, .is_valid = true };
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);      /* 1500 mm: not yet */
    t.distance_mm = THREAT_STOP_ENTER_DIST_MM;
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP, Run_Cue(&t, 1U, 100U) & THREAT_CUE_STOP);
    t.distance_mm = (uint16_t)(THREAT_STOP_ENTER_DIST_MM + 50U);             /* inside hysteresis band */
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP, Run_Cue(&t, 1U, 200U) & THREAT_CUE_STOP);
    t.distance_mm = (uint16_t)(THREAT_STOP_CLEAR_DIST_MM + 50U);
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 300U) & THREAT_CUE_STOP);
}

void test_ThreatAssessment_NoStopForSideObstacleOnlyEvenWhenClose(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 400U, .speed_cms = 0, .x_mm = USER_X(-600), .is_valid = true };
    const uint8_t cue = Run_Cue(&t, 1U, 0U);
    TEST_ASSERT_EQUAL_UINT8(0U, cue); /* centre free: plain centre pattern, no STOP */
}

void test_ThreatAssessment_StopWhenAllLanesBlockedFollowsCentreDistance(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t[3] = {
        { .distance_mm = 450U,  .speed_cms = 0, .x_mm = USER_X(-600), .is_valid = true }, /* left, very close */
        { .distance_mm = 2000U, .speed_cms = 0, .x_mm =    0, .is_valid = true }, /* centre, far */
        { .distance_mm = 2000U, .speed_cms = 0, .x_mm = USER_X(600), .is_valid = true }
    };
    /* All blocked, but the user's own lane is not close: no STOP. */
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(t, 3U, 0U) & THREAT_CUE_STOP);
    t[1].distance_mm = 900U; /* the centre obstacle is now about to be hit */
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP, Run_Cue(t, 3U, 100U) & THREAT_CUE_STOP);
}

void test_ThreatAssessment_JustClearedLaneIsNotOfferedAsFree(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t[3] = {
        { .distance_mm = 2000U, .speed_cms = 0, .x_mm = USER_X(-600), .is_valid = true }, /* left */
        { .distance_mm = 2000U, .speed_cms = 0, .x_mm =    0, .is_valid = true }, /* centre */
        { .distance_mm = 2000U, .speed_cms = 0, .x_mm = USER_X(600), .is_valid = true }  /* right */
    };
    const uint8_t no_free = (uint8_t)(THREAT_CUE_SIDE_ALL_BLOCKED << THREAT_CUE_SIDE_SHIFT);
    const uint8_t left_free = (uint8_t)(THREAT_LANE_LEFT << THREAT_CUE_SIDE_SHIFT);

    TEST_ASSERT_EQUAL_UINT8(no_free, Run_Cue(t, 3U, 0U));
    /* The left target (standing still) is lost; the still-person hold then
     * keeps the lane blocked for a while. */
    t[0].is_valid = false;
    (void)Run_Cue(t, 3U, 100U);
    const uint32_t cleared_ms = 100U + THREAT_STILL_LANE_HOLD_MS;
    TEST_ASSERT_EQUAL_UINT8(no_free, Run_Cue(t, 3U, cleared_ms - 1U));
    TEST_ASSERT_EQUAL_UINT8(no_free, Run_Cue(t, 3U, cleared_ms));   /* just cleared: not trusted */
    TEST_ASSERT_EQUAL_UINT8(no_free, Run_Cue(t, 3U, cleared_ms + THREAT_FREE_LANE_CLEAR_MS - 1U));
    TEST_ASSERT_EQUAL_UINT8(left_free, Run_Cue(t, 3U, cleared_ms + THREAT_FREE_LANE_CLEAR_MS)); /* clear long enough */
}

void test_ThreatAssessment_NeverBlockedLaneIsFreeImmediately(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 2000U, .speed_cms = 0, .x_mm = 0, .is_valid = true };
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(THREAT_LANE_LEFT << THREAT_CUE_SIDE_SHIFT), Run_Cue(&t, 1U, 0U));
}

void test_ThreatAssessment_CueStaysSteadyWhenCentrePointDropsOutBriefly(void)
{
    /* Regression for the bench log: a slow/still person's centre point drops out for
     * 0.3-0.4 s at a time while a side point stays. The cue must not toggle. */
    ThreatAssessment_Init();
    Radar_Target_t t[2] = {
        { .distance_mm = 2300U, .speed_cms = 0, .x_mm = 0, .is_valid = true },
        { .distance_mm = 2400U, .speed_cms = 0, .x_mm = USER_X(700), .is_valid = true }
    };
    Threat_Result_t res;
    uint16_t first_cue = 0xFFFFU;
    for (uint32_t now = 0U; now <= 4000U; now += 20U)
    {
        const uint32_t phase = now % 1000U;
        t[0].is_valid = (phase >= 400U); /* centre point missing 400 ms of every second */
        ThreatAssessment_ProcessTargetsAt(t, 2U, GRIP_STATE_HELD, now, &res);
        if (now == 400U)
        {
            first_cue = res.open_direction_cue; /* the centre point has just appeared */
        }
        if (now >= 400U)
        {
            TEST_ASSERT_EQUAL_UINT16(first_cue, res.open_direction_cue);
        }
    }
}

void test_ThreatAssessment_SideEdgeJitterDoesNotToggleLanes(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 2300U, .speed_cms = 0, .x_mm = USER_X(-(THREAT_SIDE_DEADZONE_MM + 10)), .is_valid = true };
    Threat_Result_t res;
    ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, 0U, &res); /* settles in LEFT */
    const uint16_t settled = res.open_direction_cue;
    for (uint32_t i = 1U; i < 60U; i++)
    {
        /* x wobbles +/-100 mm around the deadzone edge, still inside the hysteresis band */
        const int16_t wobble = (int16_t)(((i % 2U) == 0U) ? 100 : -100);
        t.x_mm = USER_X(-(THREAT_SIDE_DEADZONE_MM + 10) + wobble);
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, i * 20U, &res);
        TEST_ASSERT_EQUAL_UINT16(settled, res.open_direction_cue);
    }
}

/* Walks a target along a straight line and reports whether any frame raised an alert. */
static bool Walk_RaisesAlert(int32_t x0_mm, int32_t y0_mm, int32_t vx_mmps, int32_t vy_mmps, uint32_t duration_ms)
{
    ThreatAssessment_Init();
    bool any = false;
    for (uint32_t now = 0U; now <= duration_ms; now += 100U)
    {
        const int32_t x = x0_mm + ((vx_mmps * (int32_t)now) / 1000);
        const int32_t y = y0_mm + ((vy_mmps * (int32_t)now) / 1000);
        if (y <= 0)
        {
            break;
        }
        const int32_t sq = (x * x) + (y * y);
        int32_t d = y + (x < 0 ? -x : x); /* start above the root, then Newton */
        for (uint8_t i = 0U; i < 20U; i++)
        {
            d = (d + (sq / d)) / 2;
        }
        Radar_Target_t t = { .distance_mm = (uint16_t)d, .speed_cms = 0, .x_mm = USER_X((int16_t)x), .is_valid = true };
        Threat_Result_t res;
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
        any = any || (res.is_threat_active != 0U);
    }
    return any;
}

void test_ThreatAssessment_PersonCrossingBeyondStopZoneIsIgnored(void)
{
    /* About 1.3 m/s sideways, 2 m ahead, coming from outside the corridor. */
    TEST_ASSERT_FALSE(Walk_RaisesAlert(-2000, 2000, 1300, 0, 3500U));
    TEST_ASSERT_FALSE(Walk_RaisesAlert(2000, 2800, -1300, 0, 3500U));
}

void test_ThreatAssessment_ApproachingTargetsStillAlert(void)
{
    TEST_ASSERT_TRUE(Walk_RaisesAlert(0, 3000, 0, -1000, 3000U));       /* straight at the user */
    TEST_ASSERT_TRUE(Walk_RaisesAlert(-1200, 3000, 300, -1000, 3000U)); /* mostly forward, a little sideways */
    TEST_ASSERT_TRUE(Walk_RaisesAlert(-1800, 2000, 400, 0, 6000U));     /* slow drift across: not a crossing */
}

void test_ThreatAssessment_CrossingInsideStopZoneStillCounts(void)
{
    TEST_ASSERT_TRUE(Walk_RaisesAlert(-2000, 1200, 1300, 0, 3500U));
}

void test_ThreatAssessment_CrossingTrackerSurvivesExtremeFrames(void)
{
    /* Corrupt or jumping frames: full-scale x, huge range, 1 ms and 300 ms steps. The
     * result only has to stay well formed (no overflow, no crash); run under UBSan in CI. */
    static const int16_t xs[] = { -32767, -1, 0, 1, 32767 };
    static const uint16_t ds[] = { 1U, 300U, 1500U, 3000U, 46340U, 65535U };
    static const uint32_t steps[] = { 1U, 20U, 100U, 300U, 301U };
    uint32_t now = 0U;
    uint32_t seed = 12345U;
    ThreatAssessment_Init();
    for (uint32_t n = 0U; n < 4000U; n++)
    {
        seed = (seed * 1664525U) + 1013904223U;
        Radar_Target_t t = { .distance_mm = ds[(seed >> 8) % 6U], .speed_cms = (int16_t)(seed >> 16),
                             .x_mm = xs[(seed >> 4) % 5U], .is_valid = true };
        Threat_Result_t res;
        now += steps[(seed >> 12) % 5U];
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
        TEST_ASSERT_TRUE(res.open_direction_cue <= 7U);
    }
}

/* Speed-scaled STOP: same obstacle distance, different closing speed: only the
 * faster approach is in STOP. 1.0 m/s gives 500 + 850 + 625 = 1975 mm. */
void test_ThreatAssessment_StopDistanceGrowsWithClosingSpeed(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 1900U,
                         .speed_cms = APPROACHING(50),
                         .x_mm = 0,
                         .is_valid = true };
    /* 0.5 m/s: floor 1500 mm */
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);

    ThreatAssessment_Init();
    t.speed_cms = APPROACHING(100);
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);

    ThreatAssessment_Init();
    /* just outside 1975 mm */
    t.distance_mm = 2000U;
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);
}

void test_ThreatAssessment_StopDistanceIsFlooredAndCapped(void)
{
    /* Slow: the floor. 0.75 m/s computes 1226 mm, floored to 1500 mm. */
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 1600U,
                         .speed_cms = APPROACHING(75),
                         .x_mm = 0,
                         .is_valid = true };
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);
    t.distance_mm = THREAT_STOP_ENTER_DIST_MM;
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 100U) & THREAT_CUE_STOP);

    /* Moving away: the floor, not a bigger distance. */
    ThreatAssessment_Init();
    t.distance_mm = 1500U;
    t.speed_cms = APPROACHING(-100);
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);
    ThreatAssessment_Init();
    t.distance_mm = 1700U;
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);

    /* Very fast: capped at THREAT_STOP_MAX_DIST_MM (the parser already rejects
     * > 10 m/s). */
    ThreatAssessment_Init();
    t.speed_cms = APPROACHING(900);
    t.distance_mm = (uint16_t)(THREAT_STOP_MAX_DIST_MM + 50U);
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);
    t.distance_mm = THREAT_STOP_MAX_DIST_MM;
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 100U) & THREAT_CUE_STOP);
}

void test_ThreatAssessment_StopDistanceIsFrozenWhileLatched(void)
{
    ThreatAssessment_Init();
    /* 1.2 m/s: 500 + 1020 + 900 = 2420 mm, so STOP clears above 2520 mm. */
    Radar_Target_t t = { .distance_mm = 2400U,
                         .speed_cms = APPROACHING(120),
                         .x_mm = 0,
                         .is_valid = true };
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);
    /* The approach slows to a stop: STOP must not drop just because the speed
     * fell. */
    t.speed_cms = 0;
    t.distance_mm = 2450U;
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 300U) & THREAT_CUE_STOP);
    t.distance_mm = 2500U;
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 400U) & THREAT_CUE_STOP);
    t.distance_mm = 2560U;
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 500U) & THREAT_CUE_STOP);
    /* After clearing, the next STOP uses the new (zero) speed again: floor
     * 1500 mm. */
    t.distance_mm = 2000U;
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 600U) & THREAT_CUE_STOP);
}

/* A jogger at 3 m/s gets STOP at the 2800 mm ceiling, well before the 1500 mm
 * floor. */
void test_ThreatAssessment_JoggerGetsStopAtTheCeiling(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 2900U,
                         .speed_cms = APPROACHING(300),
                         .x_mm = 0,
                         .is_valid = true };
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 0U) & THREAT_CUE_STOP);
    t.distance_mm = 2600U;
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            Run_Cue(&t, 1U, 100U) & THREAT_CUE_STOP);
}

/* A fast target in a side lane is not in the user's path: no STOP. */
void test_ThreatAssessment_FastSideApproachDoesNotStop(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t = { .distance_mm = 2900U, .speed_cms = APPROACHING(300),
                         .x_mm = USER_X(-800), .is_valid = true };
    (void)Run_Cue(&t, 1U, 0U);
    t.distance_mm = 2600U;
    (void)Run_Cue(&t, 1U, 100U);
    t.distance_mm = 1200U;
    TEST_ASSERT_EQUAL_UINT8(0U, Run_Cue(&t, 1U, 200U) & THREAT_CUE_STOP);
}

/* The LD2450 re-orders its slots: when a still person drops out, the person in
 * the next slot moves into their slot. That jump must be read as a different
 * target, not as fast sideways motion, otherwise the crossing filter hides the
 * person and the lane clears (bench log: a still person 2.5 m ahead plus one
 * beside them). */
void test_ThreatAssessment_RadarSlotReorderDoesNotClearLane(void)
{
    ThreatAssessment_Init();
    const Radar_Target_t centre = { .distance_mm = 2500U,
                                    .speed_cms = 0,
                                    .x_mm = USER_X(-100),
                                    .is_valid = true };
    const Radar_Target_t right = { .distance_mm = 2570U,
                                   .speed_cms = 0,
                                   .x_mm = USER_X(900),
                                   .is_valid = true };
    const Radar_Target_t empty = {
        .distance_mm = 0U, .speed_cms = 0, .x_mm = 0, .is_valid = false
    };
    Threat_Result_t res;
    for (uint32_t t = 0U; t < 6000U; t += 100U)
    {
        Radar_Target_t frame[3];
        if ((t % 1000U) < 400U)
        {
            /* centre person dropped out, slots shifted */
            frame[0] = right; frame[1] = empty; frame[2] = empty;
        }
        else
        {
            frame[0] = centre; frame[1] = right; frame[2] = empty;
        }
        ThreatAssessment_ProcessTargetsAt(frame, 3U, GRIP_STATE_HELD, t, &res);
        if (t >= 500U)
        {
            /* centre stays blocked */
            TEST_ASSERT_EQUAL_UINT8(0x01U,
                                    ThreatAssessment_GetBlockedMask() & 0x01U);
            /* right stays blocked */
            TEST_ASSERT_EQUAL_UINT8(0x04U,
                                    ThreatAssessment_GetBlockedMask() & 0x04U);
        }
    }
}

/* A departing obstacle keeps its lane blocked but reports "receding" so the
 * glasses can keep the buzzer quiet; it beeps again once it stops. STOP is
 * never made quiet. */
void test_ThreatAssessment_DepartingObstacleIsReceding(void)
{
    ThreatAssessment_Init();
    Threat_Result_t res;
    Radar_Target_t t = { .distance_mm = 1800U,
                         .speed_cms = APPROACHING(-40),
                         .x_mm = USER_X(800),
                         .is_valid = true };
    uint32_t now = 0U;
    /* 40 mm per frame: quiet once the gap has grown THREAT_RECEDE_MIN_GAIN_MM,
     * not before. */
    const uint16_t start = t.distance_mm;
    while ((uint32_t)t.distance_mm <
           ((uint32_t)start + THREAT_RECEDE_MIN_GAIN_MM))
    {
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
        TEST_ASSERT_FALSE(ThreatAssessment_IsReceding());
        t.distance_mm = (uint16_t)(t.distance_mm + 40U);
        now += 100U;
    }
    ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
    now += 100U;
    TEST_ASSERT_TRUE(res.is_threat_active);         /* lane still blocked */
    TEST_ASSERT_TRUE(ThreatAssessment_IsReceding());

    t.speed_cms = 0;                                 /* stops walking away */
    ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
    TEST_ASSERT_FALSE(ThreatAssessment_IsReceding());

    /* STOP with the obstacle moving away beyond THREAT_STOP_RECEDE_MIN_MM: the
     * STOP cue stays set (lanes, BLE) but it reports receding, so the tone can
     * end early. */
    ThreatAssessment_Init();
    Radar_Target_t c = { .distance_mm = 1000U,
                         .speed_cms = APPROACHING(-40),
                         .x_mm = 0,
                         .is_valid = true };
    for (uint32_t k = 0U; k < 6U; k++)
    {
        ThreatAssessment_ProcessTargetsAt(&c, 1U, GRIP_STATE_HELD, k * 100U,
                                          &res);
        c.distance_mm = (uint16_t)(c.distance_mm + 40U);
    }
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            res.open_direction_cue & THREAT_CUE_STOP);
    TEST_ASSERT_TRUE(ThreatAssessment_IsReceding());

    /* ... it turns back: the next frame is no longer receding, so STOP sounds
     * again. */
    c.speed_cms = APPROACHING(40);
    ThreatAssessment_ProcessTargetsAt(&c, 1U, GRIP_STATE_HELD, 600U, &res);
    TEST_ASSERT_FALSE(ThreatAssessment_IsReceding());

    /* Within THREAT_STOP_RECEDE_MIN_MM: never quiet, even when moving away. */
    ThreatAssessment_Init();
    Radar_Target_t n = { .distance_mm =
                             (uint16_t)(THREAT_STOP_RECEDE_MIN_MM - 200U),
                         .speed_cms = APPROACHING(-40),
                         .x_mm = 0,
                         .is_valid = true };
    for (uint32_t k = 0U; k < 5U; k++)
    {
        ThreatAssessment_ProcessTargetsAt(&n, 1U, GRIP_STATE_HELD, k * 100U,
                                          &res);
        n.distance_mm = (uint16_t)(n.distance_mm + 20U);
    }
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP,
                            res.open_direction_cue & THREAT_CUE_STOP);
    TEST_ASSERT_FALSE(ThreatAssessment_IsReceding());
}

/* Bench log: a person standing ~2 m ahead, then walking slowly towards the
 * radar, reads -16..-24 cm/s for a few frames at a time while the distance does
 * not grow. That must keep beeping. */
void test_ThreatAssessment_SpeedJitterOnStillPersonIsNotReceding(void)
{
    static const int16_t jitter[12] = { -16, -24, -16, 8,   -16, -16,
                                        -24, 0,   -24, -16, -16, 8 };
    static const int16_t drift[12] = { 0,   10, -15, 20,  -5,  15,
                                       -20, 5,  10,  -30, -40, -50 };
    for (uint8_t lane = 0U; lane < 2U; lane++)
    {
        ThreatAssessment_Init();
        Threat_Result_t res;
        Radar_Target_t t = { .distance_mm = 2060U,
                             .speed_cms = 0,
                             .x_mm = (lane == 0U) ? 0 : USER_X(800),
                             .is_valid = true };
        for (uint32_t k = 0U; k < 60U; k++)
        {
            t.speed_cms = APPROACHING(jitter[k % 12U]);
            /* slow approach */
            t.distance_mm =
                (uint16_t)((int32_t)2060 + drift[k % 12U] - (int32_t)(k * 3U));
            ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, k * 100U,
                                              &res);
            TEST_ASSERT_TRUE(res.is_threat_active);
            TEST_ASSERT_FALSE(ThreatAssessment_IsReceding());
        }
    }
}

/* An approaching obstacle is never receding. */
void test_ThreatAssessment_ApproachingObstacleIsNotReceding(void)
{
    ThreatAssessment_Init();
    Threat_Result_t res;
    Radar_Target_t t = { .distance_mm = 2800U,
                         .speed_cms = APPROACHING(40),
                         .x_mm = USER_X(-800),
                         .is_valid = true };
    for (uint32_t k = 0U; k < 10U; k++)
    {
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, k * 100U,
                                          &res);
        TEST_ASSERT_FALSE(ThreatAssessment_IsReceding());
        t.distance_mm = (uint16_t)(t.distance_mm - 40U);
    }
}

/* Bench log, three people standing 1.5-3 m ahead: the radar kept losing one of
 * them for more than the normal lane hold, so the guidance switched left/right.
 * A lane whose person was standing still is held for
 * THREAT_STILL_LANE_HOLD_MS. */
void test_ThreatAssessment_StillPersonDropoutKeepsLaneBlocked(void)
{
    ThreatAssessment_Init();
    Radar_Target_t t[3] = {
        { .distance_mm = 2200U,
          .speed_cms = 0,
          .x_mm = USER_X(-900),
          .is_valid = true },
        { .distance_mm = 2300U, .speed_cms = 0, .x_mm = 0, .is_valid = true },
        { .distance_mm = 2200U,
          .speed_cms = 0,
          .x_mm = USER_X(900),
          .is_valid = true }
    };
    const uint8_t no_free =
        (uint8_t)(THREAT_CUE_SIDE_ALL_BLOCKED << THREAT_CUE_SIDE_SHIFT);
    for (uint32_t now = 0U; now < 1000U; now += 100U)
    {
        (void)Run_Cue(t, 3U, now);
    }
    t[0].is_valid = false; /* the left person is lost for 1.2 s */
    for (uint32_t now = 1000U; now < 2200U; now += 100U)
    {
        TEST_ASSERT_EQUAL_UINT8(no_free, Run_Cue(t, 3U, now));
    }
    t[0].is_valid = true;
    TEST_ASSERT_EQUAL_UINT8(no_free, Run_Cue(t, 3U, 2200U));
}

/* Someone who walks out of a lane is moving when last seen, so the lane clears
 * after the normal hold, not the still-person one. */
void test_ThreatAssessment_PersonWalkingOutOfLaneClearsAfterNormalHold(void)
{
    /* sideways out of the corridor at 1 m/s */
    ThreatAssessment_Init();
    Threat_Result_t res;
    Radar_Target_t t = { .distance_mm = 2000U,
                         .speed_cms = 0,
                         .x_mm = USER_X(800),
                         .is_valid = true };
    uint32_t now = 0U;
    for (; now < 1000U; now += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
    }
    for (int16_t x = 900; x <= THREAT_CORRIDOR_HALF_MM; x = (int16_t)(x + 100))
    {
        t.x_mm = USER_X(x);
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
        now += 100U;
    }
    const uint32_t left_at = now;
    t.is_valid = false;
    for (; now < (left_at + THREAT_LANE_HOLD_MS + 100U); now += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&t, 1U, GRIP_STATE_HELD, now, &res);
    }
    TEST_ASSERT_EQUAL_UINT8(0U, ThreatAssessment_GetBlockedMask() & 0x04U);

    /* approaching, then gone */
    ThreatAssessment_Init();
    Radar_Target_t a = { .distance_mm = 2500U,
                         .speed_cms = APPROACHING(40),
                         .x_mm = USER_X(-800),
                         .is_valid = true };
    for (now = 0U; now < 500U; now += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&a, 1U, GRIP_STATE_HELD, now, &res);
        a.distance_mm = (uint16_t)(a.distance_mm - 40U);
    }
    a.is_valid = false;
    const uint32_t gone_at = now;
    for (; now <= (gone_at + THREAT_LANE_HOLD_MS); now += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&a, 1U, GRIP_STATE_HELD, now, &res);
    }
    TEST_ASSERT_EQUAL_UINT8(0U, ThreatAssessment_GetBlockedMask());
}

/* One bit per filtering step, set only on the frame where that step acted. */
void test_ThreatAssessment_FilterFlagsMarkEachFilter(void)
{
    Threat_Result_t res;
    const Radar_Target_t empty = { .distance_mm = 0U, .speed_cms = 0,
                                   .x_mm = 0, .is_valid = false };

    /* A plain obstacle ahead: no filter acted. */
    ThreatAssessment_Init();
    Radar_Target_t ahead = { .distance_mm = 2000U, .speed_cms = 0,
                             .x_mm = 0, .is_valid = true };
    ThreatAssessment_ProcessTargetsAt(&ahead, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_EQUAL_UINT8(0U, ThreatAssessment_GetFilterFlags());

    /* Outside the corridor. */
    ThreatAssessment_Init();
    Radar_Target_t wide = { .distance_mm = 2500U, .speed_cms = 0,
                            .x_mm = USER_X(2000), .is_valid = true };
    ThreatAssessment_ProcessTargetsAt(&wide, 1U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_EQUAL_UINT8(THREAT_FLT_OUTSIDE,
                            ThreatAssessment_GetFilterFlags());
    /* ... cleared on the next frame without it. */
    ThreatAssessment_ProcessTargetsAt(&ahead, 1U, GRIP_STATE_HELD, 100U, &res);
    TEST_ASSERT_EQUAL_UINT8(0U, ThreatAssessment_GetFilterFlags() &
                                    THREAT_FLT_OUTSIDE);

    /* Crossing at 1.5 m/s, 2.5 m ahead. */
    ThreatAssessment_Init();
    Radar_Target_t cross = { .distance_mm = 2500U, .speed_cms = 0,
                             .x_mm = USER_X(-1200), .is_valid = true };
    bool seen = false;
    for (uint32_t t = 0U; t <= 1000U; t += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&cross, 1U, GRIP_STATE_HELD, t, &res);
        seen = seen || ((ThreatAssessment_GetFilterFlags() &
                         THREAT_FLT_CROSSING) != 0U);
        cross.x_mm = (int16_t)(cross.x_mm + USER_X(150));
    }
    TEST_ASSERT_TRUE(seen);

    /* The radar hands slot 0 to a different person 1.5 m to the side. */
    ThreatAssessment_Init();
    Radar_Target_t slot0 = ahead;
    ThreatAssessment_ProcessTargetsAt(&slot0, 1U, GRIP_STATE_HELD, 0U, &res);
    slot0.x_mm = USER_X(1000);
    slot0.distance_mm = 2800U;
    ThreatAssessment_ProcessTargetsAt(&slot0, 1U, GRIP_STATE_HELD, 100U, &res);
    TEST_ASSERT_EQUAL_UINT8(THREAT_FLT_SLOT_JUMP,
                            ThreatAssessment_GetFilterFlags() &
                                THREAT_FLT_SLOT_JUMP);

    /* Moving away. */
    ThreatAssessment_Init();
    Radar_Target_t away = { .distance_mm = 1800U, .speed_cms = APPROACHING(-40),
                            .x_mm = USER_X(800), .is_valid = true };
    for (uint32_t t = 0U; t < 800U; t += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&away, 1U, GRIP_STATE_HELD, t, &res);
        away.distance_mm = (uint16_t)(away.distance_mm + 40U);
    }
    TEST_ASSERT_EQUAL_UINT8(THREAT_FLT_RECEDING,
                            ThreatAssessment_GetFilterFlags());

    /* A still person lost: flagged once the normal hold would have cleared
     * the lane, and only while the still-person hold keeps it blocked. */
    ThreatAssessment_Init();
    for (uint32_t t = 0U; t < 500U; t += 100U)
    {
        ThreatAssessment_ProcessTargetsAt(&ahead, 1U, GRIP_STATE_HELD, t, &res);
    }
    ThreatAssessment_ProcessTargetsAt(&empty, 1U, GRIP_STATE_HELD, 500U, &res);
    TEST_ASSERT_EQUAL_UINT8(0U, ThreatAssessment_GetFilterFlags());
    ThreatAssessment_ProcessTargetsAt(&empty, 1U, GRIP_STATE_HELD,
                                      500U + THREAT_LANE_HOLD_MS, &res);
    TEST_ASSERT_EQUAL_UINT8(THREAT_FLT_STILL_HOLD,
                            ThreatAssessment_GetFilterFlags());
    ThreatAssessment_ProcessTargetsAt(&empty, 1U, GRIP_STATE_HELD,
                                      500U + THREAT_STILL_LANE_HOLD_MS, &res);
    TEST_ASSERT_EQUAL_UINT8(0U, ThreatAssessment_GetFilterFlags());
}

/* The debug view's per-slot status follows what the decision did. */
void test_ThreatAssessment_SlotStatusShowsWhatHappenedToEachSlot(void)
{
    ThreatAssessment_Init();
    Threat_Result_t res;
    Radar_Target_t t[3] = {
        { .distance_mm = 2000U, .speed_cms = 0, .x_mm = USER_X(-800),
          .is_valid = true },
        { .distance_mm = 0U, .speed_cms = 0, .x_mm = 0, .is_valid = false },
        { .distance_mm = 2500U, .speed_cms = 0, .x_mm = USER_X(2000),
          .is_valid = true }
    };
    ThreatAssessment_ProcessTargetsAt(t, 3U, GRIP_STATE_HELD, 0U, &res);
    TEST_ASSERT_EQUAL_UINT8(THREAT_SLOT_LEFT,
                            ThreatAssessment_GetSlotStatus(0U));
    TEST_ASSERT_EQUAL_UINT8(THREAT_SLOT_EMPTY,
                            ThreatAssessment_GetSlotStatus(1U));
    TEST_ASSERT_EQUAL_UINT8(THREAT_SLOT_OUTSIDE,
                            ThreatAssessment_GetSlotStatus(2U));
    TEST_ASSERT_EQUAL_UINT8(THREAT_SLOT_EMPTY,
                            ThreatAssessment_GetSlotStatus(3U));

    /* Debug conversions match the decision's: user frame, forward distance,
     * closing speed. */
    TEST_ASSERT_EQUAL_INT16(-800, ThreatAssessment_UserLateralMm(USER_X(-800)));
    TEST_ASSERT_EQUAL_UINT16(1600U, ThreatAssessment_ForwardMm(2000U, -1200));
    TEST_ASSERT_EQUAL_INT16(40, ThreatAssessment_ClosingCms(APPROACHING(40)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ThreatAssessment_DepartingTargetStillOccupiesLane);
    RUN_TEST(test_ThreatAssessment_SpeedSignConvention_PositiveIsApproaching);
    RUN_TEST(test_ThreatAssessment_Hysteresis_EnterAndClear);
    RUN_TEST(test_ThreatAssessment_FR6_GripGating);
    RUN_TEST(test_ThreatAssessment_ReleasedKeepsLaneOccupancyCurrent);
    RUN_TEST(test_ThreatAssessment_TtcIsBoundedInWireUnits);
    RUN_TEST(test_ThreatAssessment_TtcMatchesWideArithmeticAcrossBoundaryInputs);
    RUN_TEST(test_ThreatAssessment_ClearUsesUnavailableSentinels);
    RUN_TEST(test_ThreatAssessment_ThreeLaneOccupancy_CorridorAndSideHysteresis);
    RUN_TEST(test_ThreatAssessment_LaneHoldBridgesBriefDropout);
    RUN_TEST(test_ThreatAssessment_TtcHoldSmoothsBriefSpeedDropout);
    RUN_TEST(test_ThreatAssessment_TtcHoldTreatsDepartingReadingAsDropout);
    RUN_TEST(test_ThreatAssessment_OpenDirectionCueTruthTable);
    RUN_TEST(test_ThreatAssessment_StopWhenCenterLaneBlockedAndTooClose);
    RUN_TEST(test_ThreatAssessment_NoStopForSideObstacleOnlyEvenWhenClose);
    RUN_TEST(test_ThreatAssessment_StopWhenAllLanesBlockedFollowsCentreDistance);
    RUN_TEST(test_ThreatAssessment_JustClearedLaneIsNotOfferedAsFree);
    RUN_TEST(test_ThreatAssessment_NeverBlockedLaneIsFreeImmediately);
    RUN_TEST(test_ThreatAssessment_CueStaysSteadyWhenCentrePointDropsOutBriefly);
    RUN_TEST(test_ThreatAssessment_SideEdgeJitterDoesNotToggleLanes);
    RUN_TEST(test_ThreatAssessment_PersonCrossingBeyondStopZoneIsIgnored);
    RUN_TEST(test_ThreatAssessment_ApproachingTargetsStillAlert);
    RUN_TEST(test_ThreatAssessment_CrossingInsideStopZoneStillCounts);
    RUN_TEST(test_ThreatAssessment_CrossingTrackerSurvivesExtremeFrames);
    RUN_TEST(test_ThreatAssessment_StopDistanceGrowsWithClosingSpeed);
    RUN_TEST(test_ThreatAssessment_StopDistanceIsFlooredAndCapped);
    RUN_TEST(test_ThreatAssessment_StopDistanceIsFrozenWhileLatched);
    RUN_TEST(test_ThreatAssessment_JoggerGetsStopAtTheCeiling);
    RUN_TEST(test_ThreatAssessment_FastSideApproachDoesNotStop);
    RUN_TEST(test_ThreatAssessment_RadarSlotReorderDoesNotClearLane);
    RUN_TEST(test_ThreatAssessment_DepartingObstacleIsReceding);
    RUN_TEST(test_ThreatAssessment_ApproachingObstacleIsNotReceding);
    RUN_TEST(test_ThreatAssessment_SpeedJitterOnStillPersonIsNotReceding);
    RUN_TEST(test_ThreatAssessment_StillPersonDropoutKeepsLaneBlocked);
    RUN_TEST(test_ThreatAssessment_PersonWalkingOutOfLaneClearsAfterNormalHold);
    RUN_TEST(test_ThreatAssessment_FilterFlagsMarkEachFilter);
    RUN_TEST(test_ThreatAssessment_SlotStatusShowsWhatHappenedToEachSlot);
    return UNITY_END();
}
