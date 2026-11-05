#include "unity.h"
#include "app/CaneMode/CaneMode.h"
#include "app/CaneMode/CaneMode_Cfg.h"
#include <math.h>
#include <string.h>

static CaneMode_t s_mode;
static CaneMode_Input_t s_in;
static CaneMode_Output_t s_out;
static uint32_t s_now;

static void Step(void)
{
    s_in.sample.timestamp_ms = s_now;
    CaneMode_Step(&s_mode, &s_in, s_now, &s_out);
}

static void Advance(uint32_t ms)
{
    for (uint32_t elapsed = 0U; elapsed < ms; elapsed += 10U)
    {
        s_now += 10U;
        Step();
    }
}

static void Grip(Grip_State_t grip)
{
    s_in.grip = grip;
    Advance(10U);
}

static void Horizontal(void)
{
    s_in.sample.ax = 1.0f;
    s_in.sample.az = 0.0f;
}

static void Upright(void)
{
    s_in.sample.ax = 0.0f;
    s_in.sample.az = 1.0f;
}

static void Fall(void)
{
    Horizontal();
    s_in.local_fall = true;
    Advance(10U);
    s_in.local_fall = false;
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
}

static void Pause(void)
{
    Grip(GRIP_STATE_RELEASED);
    Fall();
    Grip(GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_MUTED, s_out.state);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_FALL_ALERT | BLE_FLAG_ALARM_PAUSED, s_out.status_flags);
}

static void Request_Override(void)
{
    Pause();
    Grip(GRIP_STATE_RELEASED);
    Advance(100U);
    Grip(GRIP_STATE_HELD);
    Advance(3000U);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested);
    TEST_ASSERT_TRUE(s_out.fall_latched); /* Wait for the I2C sleep acknowledgement. */
}

static void Confirm_Sleep(void)
{
    s_in.imu_sleeping = true;
    s_in.imu_ready = false;
    s_in.sample_valid = false;
    Advance(10U);
    TEST_ASSERT_EQUAL(SYS_STATE_STANDBY, s_out.state);
    TEST_ASSERT_FALSE(s_out.fall_latched);
}

void setUp(void)
{
    memset(&s_in, 0, sizeof(s_in));
    s_now = 1000U;
    s_in.grip = GRIP_STATE_HELD;
    s_in.imu_ready = true;
    s_in.sample_valid = true;
    Upright();
    CaneMode_Init(&s_mode, s_now);
    Step();
}

void tearDown(void) { }

void test_NormalParking_RequiresContinuousUprightStillness(void)
{
    Grip(GRIP_STATE_RELEASED);
    TEST_ASSERT_EQUAL(SYS_STATE_UNGRIPPED_WATCH, s_out.state);
    Advance(2990U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    Advance(10U);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested);
    TEST_ASSERT_FALSE(s_out.fall_latched);
    Confirm_Sleep();
    TEST_ASSERT_FALSE(s_out.fall_detection_enabled);
    TEST_ASSERT_FALSE(s_out.ground_enabled);
}

void test_Regrip_CancelsParkingWithoutCancellingFallEvidence(void)
{
    Grip(GRIP_STATE_RELEASED);
    Advance(2990U);
    s_in.fall_pending = true;
    Grip(GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    TEST_ASSERT_FALSE(s_out.reset_fall_detection);
}

void test_ReleaseOrTiltAlone_IsNotAFall_AndPendingFallCannotSleep(void)
{
    Grip(GRIP_STATE_RELEASED);
    Horizontal();
    Advance(10000U);
    TEST_ASSERT_EQUAL(SYS_STATE_UNGRIPPED_WATCH, s_out.state);
    TEST_ASSERT_FALSE(s_out.fall_latched);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    Upright();
    s_in.fall_pending = true;
    Advance(5000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_Motion_AndUpsideDownPose_CannotPark(void)
{
    Grip(GRIP_STATE_RELEASED);
    Advance(2000U);
    s_in.sample.gx = 20.0f;
    Advance(10U);
    s_in.sample.gx = 0.0f;
    Advance(1000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    s_in.sample.az = -1.0f;
    Advance(4000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_Alarm_PrioritizesNewFallOverSameCycleGripAndParking(void)
{
    Grip(GRIP_STATE_RELEASED);
    Advance(2990U);
    s_in.grip = GRIP_STATE_HELD;
    Fall();
    TEST_ASSERT_FALSE(s_out.alarm_paused);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    Advance(4000U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
}

void test_PinnedGrip_CannotAcknowledgeOrInvokeOverride(void)
{
    Fall();
    Advance(20000U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    Grip(GRIP_STATE_RELEASED);
    Grip(GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_MUTED, s_out.state);
    Advance(4000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_Pause_IsBoundedEvenWhenGripRemainsHeld(void)
{
    Pause();
    Advance(14990U);
    TEST_ASSERT_TRUE(s_out.alarm_paused);
    Advance(10U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    Advance(2000U);
    TEST_ASSERT_FALSE(s_out.alarm_paused);
}

void test_UnrecoveredRelease_RealarmAtThreeSeconds_NotStandby(void)
{
    Pause();
    Grip(GRIP_STATE_RELEASED);
    Advance(2990U);
    TEST_ASSERT_TRUE(s_out.alarm_paused);
    Advance(10U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    Advance(60000U);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_UprightRecovery_RequiresHeldAndOneContinuousSecond(void)
{
    Pause();
    Upright();
    Advance(10U);
    Advance(990U);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    Advance(10U);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
    TEST_ASSERT_FALSE(s_out.fall_latched);
    TEST_ASSERT_TRUE(s_out.reset_fall_detection);
    Grip(GRIP_STATE_RELEASED);
    Advance(3000U);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested);
}

void test_UprightWithoutGrip_DoesNotClearAnUnresolvedIncident(void)
{
    Pause();
    Upright();
    Grip(GRIP_STATE_RELEASED);
    Advance(4000U);
    TEST_ASSERT_TRUE(s_out.alarm_paused);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_Override_RequiresSecondHoldAndConfirmedHardwareSleep(void)
{
    Request_Override();
    Confirm_Sleep();
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_STANDBY | BLE_FLAG_PARKED_BY_USER, s_out.status_flags);
    Advance(10000U);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested); /* Same held grip cannot wake. */
    Grip(GRIP_STATE_RELEASED);
    Grip(GRIP_STATE_HELD);
    Advance(90U);
    TEST_ASSERT_EQUAL(SYS_STATE_STANDBY, s_out.state);
    Advance(10U);
    TEST_ASSERT_EQUAL(SYS_STATE_INIT, s_out.state);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    Advance(200U);
    TEST_ASSERT_EQUAL(SYS_STATE_INIT, s_out.state); /* Still waiting for actual IMU readiness. */
    s_in.imu_sleeping = false;
    s_in.imu_ready = true;
    s_in.sample_valid = true;
    Advance(10U);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
}

void test_InterruptedOverrideHold_RestartsQualification(void)
{
    Pause();
    Grip(GRIP_STATE_RELEASED);
    Grip(GRIP_STATE_HELD);
    Advance(2500U);
    Grip(GRIP_STATE_RELEASED);
    Grip(GRIP_STATE_HELD);
    Advance(500U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_SleepFailure_PreservesIncidentAndPauseDeadline(void)
{
    Request_Override();
    s_in.imu_fault = true;
    s_in.sample_valid = false;
    Advance(10U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_TRUE(s_out.degraded);
    Advance(15000U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
}

void test_TechnicalFault_CannotOverwriteFallOrClaimUprightRecovery(void)
{
    Pause();
    s_in.imu_fault = true;
    Upright();
    Advance(2000U);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_SENSOR_FAULT | BLE_FLAG_FALL_ALERT, s_out.status_flags);
    s_in.imu_fault = false;
    Advance(1010U);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
}

void test_StaleInvalidOrMissingSamples_CannotQualifyParking(void)
{
    Grip(GRIP_STATE_RELEASED);
    Advance(2000U);
    s_now += 1000U;
    CaneMode_Step(&s_mode, &s_in, s_now, &s_out); /* stale timestamp and observation gap */
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    s_in.sample.az = NAN;
    Advance(4000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    Upright();
    s_in.sample_valid = false;
    Advance(4000U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_TimerWrap_AndLongNormalGrip(void)
{
    Advance(60000U);
    TEST_ASSERT_EQUAL(SYS_STATE_ACTIVE, s_out.state);
    TEST_ASSERT_FALSE(s_out.degraded);
    s_now = UINT32_MAX - 100U;
    CaneMode_Init(&s_mode, s_now);
    Step();
    Grip(GRIP_STATE_RELEASED);
    Advance(3000U);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested);
}

void test_PeerAlarmDuringStandby_DoesNotNeedImuMotionWake(void)
{
    Grip(GRIP_STATE_RELEASED);
    Advance(3000U);
    Confirm_Sleep();
    s_in.peer_fall = true;
    Advance(10U);
    s_in.peer_fall = false;
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    Grip(GRIP_STATE_HELD);
    Advance(100U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    TEST_ASSERT_TRUE(s_out.alarm_paused);
}

void test_ExpiredPause_CannotBeClearedByALateSleepAcknowledgement(void)
{
    Request_Override();
    s_now = s_mode.pause_start_ms + 15000U;
    s_in.imu_sleeping = true;
    s_in.imu_ready = false;
    s_in.sample_valid = false;
    Step();
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
}

void test_TofFault_DoesNotPreventImuVerifiedRecovery(void)
{
    Pause();
    s_in.technical_fault = true;
    Upright();
    Advance(1010U);
    TEST_ASSERT_FALSE(s_out.fall_latched);
    TEST_ASSERT_EQUAL(SYS_STATE_FAULT_DEGRADED, s_out.state);
    TEST_ASSERT_TRUE(s_out.degraded);
    Grip(GRIP_STATE_RELEASED);
    Advance(3000U);
    Confirm_Sleep();
    TEST_ASSERT_BITS_HIGH(BLE_FLAG_SENSOR_FAULT, s_out.status_flags);
}

void test_MovingAgain_CancelsUncommittedNormalParking(void)
{
    Grip(GRIP_STATE_RELEASED);
    Advance(3000U);
    TEST_ASSERT_TRUE(s_out.imu_sleep_requested);
    s_in.sample.gy = 100.0f;
    Advance(10U);
    TEST_ASSERT_FALSE(s_out.imu_sleep_requested);
    TEST_ASSERT_TRUE(s_out.fall_detection_enabled);
}

void test_RestoredIncidentResumesAudibleAndRequiresNormalGuardedClearance(void)
{
    CaneMode_RestoreIncident(&s_mode);
    Step();
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    TEST_ASSERT_TRUE(s_out.fall_latched);
    TEST_ASSERT_FALSE(s_out.alarm_paused);
    TEST_ASSERT_FALSE(s_out.fall_detection_enabled);

    /* A held grip at reboot is not treated as an acknowledgement edge. */
    Advance(100U);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_CONFIRMED, s_out.state);
    Grip(GRIP_STATE_RELEASED);
    Grip(GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL(SYS_STATE_FALL_MUTED, s_out.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_NormalParking_RequiresContinuousUprightStillness);
    RUN_TEST(test_Regrip_CancelsParkingWithoutCancellingFallEvidence);
    RUN_TEST(test_ReleaseOrTiltAlone_IsNotAFall_AndPendingFallCannotSleep);
    RUN_TEST(test_Motion_AndUpsideDownPose_CannotPark);
    RUN_TEST(test_Alarm_PrioritizesNewFallOverSameCycleGripAndParking);
    RUN_TEST(test_PinnedGrip_CannotAcknowledgeOrInvokeOverride);
    RUN_TEST(test_Pause_IsBoundedEvenWhenGripRemainsHeld);
    RUN_TEST(test_UnrecoveredRelease_RealarmAtThreeSeconds_NotStandby);
    RUN_TEST(test_UprightRecovery_RequiresHeldAndOneContinuousSecond);
    RUN_TEST(test_UprightWithoutGrip_DoesNotClearAnUnresolvedIncident);
    RUN_TEST(test_Override_RequiresSecondHoldAndConfirmedHardwareSleep);
    RUN_TEST(test_InterruptedOverrideHold_RestartsQualification);
    RUN_TEST(test_SleepFailure_PreservesIncidentAndPauseDeadline);
    RUN_TEST(test_TechnicalFault_CannotOverwriteFallOrClaimUprightRecovery);
    RUN_TEST(test_StaleInvalidOrMissingSamples_CannotQualifyParking);
    RUN_TEST(test_TimerWrap_AndLongNormalGrip);
    RUN_TEST(test_PeerAlarmDuringStandby_DoesNotNeedImuMotionWake);
    RUN_TEST(test_ExpiredPause_CannotBeClearedByALateSleepAcknowledgement);
    RUN_TEST(test_TofFault_DoesNotPreventImuVerifiedRecovery);
    RUN_TEST(test_MovingAgain_CancelsUncommittedNormalParking);
    RUN_TEST(test_RestoredIncidentResumesAudibleAndRequiresNormalGuardedClearance);
    return UNITY_END();
}
