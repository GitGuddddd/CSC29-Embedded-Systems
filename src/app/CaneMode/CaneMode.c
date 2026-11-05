#include "CaneMode.h"
#include "CaneMode_Cfg.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool Qualify(CaneMode_Timer_t * const p_timer, bool condition,
                    uint32_t duration_ms, uint32_t now_ms)
{
    if (!condition)
    {
        p_timer->running = false;
        return false;
    }
    if (!p_timer->running)
    {
        p_timer->running = true;
        p_timer->since_ms = now_ms;
    }
    return ((uint32_t)(now_ms - p_timer->since_ms) >= duration_ms);
}

static void Reset_Qualifications(CaneMode_t * const p_mode)
{
    p_mode->park.running = false;
    p_mode->recovery.running = false;
    p_mode->release.running = false;
    p_mode->hold.running = false;
}

static bool Get_StillPose(const CaneMode_Input_t * const p_in, uint32_t now_ms,
                          bool * const p_park_upright, bool * const p_recovery_upright)
{
    *p_park_upright = false;
    *p_recovery_upright = false;
    const Imu_Sample_t * const p_s = &p_in->sample;
    if ((!p_in->sample_valid) || (!p_in->imu_ready) || p_in->imu_fault ||
        ((uint32_t)(now_ms - p_s->timestamp_ms) > CANEMODE_MAX_SAMPLE_AGE_MS) ||
        !isfinite(p_s->ax) || !isfinite(p_s->ay) || !isfinite(p_s->az) ||
        !isfinite(p_s->gx) || !isfinite(p_s->gy) || !isfinite(p_s->gz))
    {
        return false;
    }

    float a_sq = (p_s->ax * p_s->ax) + (p_s->ay * p_s->ay) + (p_s->az * p_s->az);
    float g_sq = (p_s->gx * p_s->gx) + (p_s->gy * p_s->gy) + (p_s->gz * p_s->gz);
    if ((a_sq < CANEMODE_STILL_ACC_MIN_SQ) || (a_sq > CANEMODE_STILL_ACC_MAX_SQ) ||
        (g_sq > CANEMODE_STILL_GYRO_MAX_SQ))
    {
        return false;
    }

    float dot = (p_s->ax * CANEMODE_UPRIGHT_X) + (p_s->ay * CANEMODE_UPRIGHT_Y) +
                (p_s->az * CANEMODE_UPRIGHT_Z);
    float ref_sq = (CANEMODE_UPRIGHT_X * CANEMODE_UPRIGHT_X) +
                   (CANEMODE_UPRIGHT_Y * CANEMODE_UPRIGHT_Y) +
                   (CANEMODE_UPRIGHT_Z * CANEMODE_UPRIGHT_Z);
    /* Signed dot product rejects an upside-down cane; no acosf/sqrtf is needed. */
    if ((dot > 0.0f) && (ref_sq > 0.0f))
    {
        *p_park_upright = ((dot * dot) >= (a_sq * ref_sq * CANEMODE_PARK_COS_SQ));
        *p_recovery_upright = ((dot * dot) >= (a_sq * ref_sq * CANEMODE_RECOVERY_COS_SQ));
    }
    return true;
}

static void Latch_Alarm(CaneMode_t * const p_mode, Grip_State_t grip)
{
    p_mode->alarm = CANE_ALARM_LATCHED;
    p_mode->acknowledge_armed = (grip == GRIP_STATE_RELEASED);
    p_mode->override_armed = false;
    p_mode->override_holding = false;
    p_mode->park_requested = false;
    p_mode->park_override = false;
    Reset_Qualifications(p_mode);
    p_mode->wake.running = false;
}

void CaneMode_Init(CaneMode_t * const p_mode, uint32_t now_ms)
{
    if (p_mode != NULL)
    {
        memset(p_mode, 0, sizeof(*p_mode));
        p_mode->power = CANE_POWER_STARTING;
        p_mode->alarm = CANE_ALARM_CLEAR;
        p_mode->previous_grip = GRIP_STATE_UNKNOWN;
        p_mode->last_step_ms = now_ms;
    }
}

void CaneMode_Step(CaneMode_t * const p_mode, const CaneMode_Input_t * const p_in,
                   uint32_t now_ms, CaneMode_Output_t * const p_out)
{
    if ((p_mode == NULL) || (p_in == NULL) || (p_out == NULL))
    {
        return;
    }
    memset(p_out, 0, sizeof(*p_out));
    if ((uint32_t)(now_ms - p_mode->last_step_ms) > CANEMODE_MAX_STEP_GAP_MS)
    {
        /* Missing observations cannot count as continuous stillness or a long hold. */
        Reset_Qualifications(p_mode);
        p_mode->wake.running = false;
        p_mode->override_holding = false;
        p_mode->park_requested = false;
        p_mode->park_override = false;
    }
    p_mode->last_step_ms = now_ms;

    bool held = (p_in->grip == GRIP_STATE_HELD);
    bool released = (p_in->grip == GRIP_STATE_RELEASED);
    bool rising = held && (p_mode->previous_grip == GRIP_STATE_RELEASED);
    bool park_upright;
    bool recovery_upright;
    bool still = Get_StillPose(p_in, now_ms, &park_upright, &recovery_upright);
    bool pose_fault = p_in->imu_fault || (p_in->grip == GRIP_STATE_UNKNOWN);
    bool fault = pose_fault || p_in->technical_fault;
    bool new_fall = p_in->local_fall || p_in->peer_fall;

    /* A new incident wins over acknowledgement, recovery and a pending sleep. */
    if (new_fall)
    {
        Latch_Alarm(p_mode, p_in->grip);
    }
    if (pose_fault)
    {
        p_mode->park_requested = false;
        p_mode->park_override = false;
        p_mode->override_armed = false;
        p_mode->override_holding = false;
        p_mode->hold.running = false;
    }
    if (p_mode->park_requested && !p_in->imu_sleeping &&
        ((!p_mode->park_override && (!released || !park_upright || p_in->fall_pending)) ||
         (p_mode->park_override && (!held || !still || p_in->fall_pending))))
    {
        p_mode->park_requested = false;
        p_mode->park_override = false;
    }
    if (p_mode->park_requested && p_mode->park_override &&
        ((uint32_t)(now_ms - p_mode->pause_start_ms) >= CANEMODE_AUDIO_PAUSE_MS))
    {
        Latch_Alarm(p_mode, p_in->grip);
    }
    if (p_mode->park_requested && p_in->imu_sleeping && !new_fall)
    {
        /* Commit standby (and the explicit override) only after the driver confirms sleep. */
        p_mode->power = CANE_POWER_STANDBY;
        p_mode->parked_by_user = p_mode->park_override;
        p_mode->park_requested = false;
        p_mode->park_override = false;
        p_mode->alarm = CANE_ALARM_CLEAR;
        p_mode->wake_armed = released;
        Reset_Qualifications(p_mode);
        p_mode->wake.running = false;
        p_out->reset_fall_detection = true;
    }

    if ((p_mode->power == CANE_POWER_MONITORING) && p_in->imu_sleeping)
    {
        /* A cancelled sleep may already have reached the sensor. Require wake validation. */
        p_mode->power = CANE_POWER_STARTING;
    }
    if (p_mode->power == CANE_POWER_STANDBY)
    {
        if (released)
        {
            p_mode->wake_armed = true;
        }
        /* The override's held touch is consumed; release and hold again to wake. */
        if (Qualify(&p_mode->wake, held && p_mode->wake_armed,
                    CANEMODE_WAKE_HOLD_MS, now_ms))
        {
            p_mode->power = CANE_POWER_STARTING;
            p_mode->parked_by_user = false;
            p_out->reset_fall_detection = true;
            Reset_Qualifications(p_mode);
        }
    }
    else if ((p_mode->power == CANE_POWER_STARTING) && p_in->imu_ready &&
             p_in->sample_valid && !p_in->imu_fault && !p_in->imu_sleeping)
    {
        p_mode->power = CANE_POWER_MONITORING;
        p_out->reset_fall_detection = true;
    }

    if (!new_fall && (p_mode->alarm != CANE_ALARM_CLEAR))
    {
        if (p_mode->alarm == CANE_ALARM_LATCHED)
        {
            if (released)
            {
                p_mode->acknowledge_armed = true;
            }
            if (rising && p_mode->acknowledge_armed)
            {
                p_mode->alarm = CANE_ALARM_PAUSED;
                p_mode->pause_start_ms = now_ms;
                p_mode->override_armed = false;
                p_mode->override_holding = false;
                Reset_Qualifications(p_mode);
            }
        }

        if (Qualify(&p_mode->recovery, held && recovery_upright && !pose_fault &&
                    (p_mode->power == CANE_POWER_MONITORING), CANEMODE_RECOVERY_MS, now_ms))
        {
            p_mode->alarm = CANE_ALARM_CLEAR;
            p_mode->override_armed = false;
            p_mode->override_holding = false;
            p_out->reset_fall_detection = true;
            Reset_Qualifications(p_mode);
        }
        else if (p_mode->alarm == CANE_ALARM_PAUSED)
        {
            bool pause_expired = ((uint32_t)(now_ms - p_mode->pause_start_ms) >=
                                  CANEMODE_AUDIO_PAUSE_MS);
            bool released_unrecovered = Qualify(&p_mode->release,
                                                released && !recovery_upright,
                                                CANEMODE_RELEASE_REALARM_MS, now_ms);
            if (pause_expired || released_unrecovered)
            {
                Latch_Alarm(p_mode, p_in->grip);
            }
            else
            {
                if (released)
                {
                    p_mode->override_armed = still && !pose_fault;
                    p_mode->override_holding = false;
                }
                if (rising && p_mode->override_armed)
                {
                    p_mode->override_holding = true;
                    p_mode->override_armed = false;
                }
                if (Qualify(&p_mode->hold, held && p_mode->override_holding && still &&
                            !pose_fault && !p_in->fall_pending, CANEMODE_PARK_HOLD_MS, now_ms))
                {
                    p_mode->park_requested = true;
                    p_mode->park_override = true;
                }
            }
        }
    }
    else if (!new_fall && (p_mode->power == CANE_POWER_MONITORING))
    {
        /* Elapsed release time alone never disables a pending fall detector. */
        if (Qualify(&p_mode->park, released && park_upright && !pose_fault && !p_in->fall_pending,
                    CANEMODE_PARK_MS, now_ms))
        {
            p_mode->park_requested = true;
            p_mode->park_override = false;
        }
    }

    p_out->fall_latched = (p_mode->alarm != CANE_ALARM_CLEAR);
    p_out->alarm_paused = (p_mode->alarm == CANE_ALARM_PAUSED);
    p_out->degraded = fault;
    p_out->imu_sleep_requested = p_mode->park_requested || (p_mode->power == CANE_POWER_STANDBY);
    p_out->fall_detection_enabled = (p_mode->power == CANE_POWER_MONITORING) &&
                                    !p_out->fall_latched && p_in->imu_ready && !p_in->imu_fault &&
                                    !p_out->imu_sleep_requested;
    p_out->ground_enabled = (p_mode->power == CANE_POWER_MONITORING) && held &&
                            !p_out->fall_latched && !p_out->imu_sleep_requested;

    if (p_out->fall_latched)
    {
        p_out->state = p_out->alarm_paused ? SYS_STATE_FALL_MUTED : SYS_STATE_FALL_CONFIRMED;
        p_out->status_flags |= BLE_FLAG_FALL_ALERT;
    }
    else if (p_mode->power == CANE_POWER_STANDBY)
    {
        p_out->state = SYS_STATE_STANDBY;
    }
    else if (fault)
    {
        p_out->state = SYS_STATE_FAULT_DEGRADED;
    }
    else if (p_mode->power == CANE_POWER_STARTING)
    {
        p_out->state = SYS_STATE_INIT;
    }
    else if (!held)
    {
        p_out->state = SYS_STATE_UNGRIPPED_WATCH;
    }
    else
    {
        p_out->state = p_in->ground_hazard ? SYS_STATE_WATCH : SYS_STATE_ACTIVE;
    }
    if (held)
    {
        p_out->status_flags |= BLE_FLAG_CANE_HELD;
    }
    if (fault)
    {
        p_out->status_flags |= BLE_FLAG_SENSOR_FAULT;
    }
    if (p_out->alarm_paused)
    {
        p_out->status_flags |= BLE_FLAG_ALARM_PAUSED;
    }
    if (p_mode->power == CANE_POWER_STANDBY)
    {
        p_out->status_flags |= BLE_FLAG_STANDBY;
    }
    if (p_mode->parked_by_user)
    {
        p_out->status_flags |= BLE_FLAG_PARKED_BY_USER;
    }
    p_mode->previous_grip = p_in->grip;
}

const char * CaneMode_GetStateName(Sys_State_t state)
{
    switch (state)
    {
        case SYS_STATE_INIT: return "INIT / IMU STARTING";
        case SYS_STATE_ACTIVE: return "ACTIVE";
        case SYS_STATE_WATCH: return "WATCH (GROUND HAZARD)";
        case SYS_STATE_UNGRIPPED_WATCH: return "UNGRIPPED WATCH";
        case SYS_STATE_FALL_CONFIRMED: return "FALL CONFIRMED";
        case SYS_STATE_FALL_MUTED: return "FALL PAUSED";
        case SYS_STATE_STANDBY: return "STANDBY (IMU ASLEEP)";
        case SYS_STATE_FAULT_DEGRADED: return "FAULT DEGRADED";
        default: return "UNKNOWN";
    }
}
