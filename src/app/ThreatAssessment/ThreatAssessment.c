#include "ThreatAssessment.h"
#include "ThreatAssessment_Cfg.h"
#include <stddef.h>

/*
 * \file ThreatAssessment.c
 * \brief 3-lane obstacle threat assessment with corridor gating, side hysteresis,
 *        lane occupancy hold (THREAT_LANE_HOLD_MS), and TTC speed-dropout smoothing (THREAT_SPEED_HOLD_MS) (FR8, FR13)
 *        Logic ported from putra-branch (OPEN-22 to OPEN-26): every valid target
 *        in the corridor is evaluated into its lane, none is excluded for moving
 *        away, and closing speed follows THREAT_NEGATIVE_SPEED_IS_APPROACHING.
 * Compliance: Barr-C:2018
 */

/* Per-lane state. Widest members first and the two flags as 1-bit fields so the
 * struct has no padding holes (28 -> 20 bytes per lane). Only the radar task calls
 * ThreatAssessment_*, so the shared flag byte is never written from two contexts. */
typedef struct
{
    uint32_t hold_pending_ms;
    uint32_t speed_hold_pending_ms;
    uint32_t cleared_at_ms;
    uint16_t last_dist_mm;
    int16_t held_closing_cms;
    uint8_t blocked : 1;
    uint8_t ever_blocked : 1;
} Threat_LaneState_t;

static Threat_LaneState_t s_lanes[THREAT_LANE_COUNT];
/* Per radar slot: last lateral position and smoothed sideways speed (crossing filter). */
typedef struct
{
    uint32_t prev_ms;
    int16_t vx_mmps; /* clamped to +/-THREAT_CROSSING_SPEED_CLAMP_MMPS */
    int16_t vy_mmps;
    int16_t prev_x_mm;
    uint16_t prev_y_mm;
    uint8_t has_prev : 1;
} Threat_Track_t;

_Static_assert(sizeof(Threat_Track_t) == 16U, "Crossing tracker must stay at 16 bytes per slot");

static Threat_Track_t s_tracks[3];
static Threat_Lane_t s_target_last_side[3];
static uint32_t s_synthetic_ms = 0U;
static bool s_stop_latched = false;
static uint8_t s_fast_frames = 0U; /* consecutive frames of a fast centre approach */

_Static_assert(sizeof(Threat_LaneState_t) == 20U, "Lane state must stay free of padding holes");
_Static_assert(sizeof(Threat_Result_t) == 6U, "Compact threat result must remain six bytes");

static void Clear_Result(Threat_Result_t * const p_result)
{
    *p_result = (Threat_Result_t){
        .nearest_distance_mm = UINT16_MAX,
        .closing_speed_cms = 0,
        .ttc_20ms = BLE_TTC_NO_THREAT,
        .is_threat_active = 0U,
        .open_direction_cue = 0U
    };
}

static void Reset_Lane(Threat_LaneState_t * const p_lane)
{
    p_lane->blocked = 0U;
    p_lane->last_dist_mm = 0U;
    p_lane->hold_pending_ms = UINT32_MAX;
    p_lane->held_closing_cms = 0;
    p_lane->speed_hold_pending_ms = UINT32_MAX;
    p_lane->ever_blocked = 0U;
    p_lane->cleared_at_ms = 0U;
}

static void Reset_AllState(void)
{
    for (uint8_t lane = 0U; lane < (uint8_t)THREAT_LANE_COUNT; lane++)
    {
        Reset_Lane(&s_lanes[lane]);
    }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        s_target_last_side[i] = THREAT_LANE_CENTER;
        s_tracks[i] = (Threat_Track_t){ 0U, 0, 0, 0, 0U, 0U };
    }
    s_stop_latched = false;
    s_fast_frames = 0U;
}

/* Lateral position in the user's frame: negative is left (mirrored if the radar is fitted the other way). */
static int16_t Lateral_Mm(int16_t x_mm)
{
#if (THREAT_MIRROR_X != 0)
    return (x_mm == INT16_MIN) ? INT16_MAX : (int16_t)(-x_mm);
#else
    return x_mm;
#endif
}

/* Forward distance from range and sideways offset: sqrt(d^2 - x^2). Unsigned 32-bit
 * arithmetic, so even d = 65535 cannot overflow; a corrupt x larger than d gives 0. */
static uint16_t Forward_Mm(uint16_t distance_mm, int16_t x_mm)
{
    const uint32_t d = (uint32_t)distance_mm;
    const uint32_t ax = (x_mm < 0) ? ((uint32_t)(-(int32_t)x_mm)) : (uint32_t)x_mm;
    const uint32_t d_sq = d * d;
    const uint32_t x_sq = ax * ax;
    if (x_sq >= d_sq)
    {
        return 0U;
    }
    const uint32_t sq = d_sq - x_sq;
    uint32_t r = d; /* never below the root, so Newton converges down and r stays >= 1 */
    for (uint8_t i = 0U; i < 16U; i++)
    {
        r = (r + (sq / r)) / 2U;
    }
    return (uint16_t)r;
}

/* Smooth one speed sample into a 16-bit state. The sample is clamped first (far above any
 * walking speed), so the 32-bit products below cannot overflow however the radar jumps. */
static int16_t Smooth_Speed(int16_t state, int32_t sample_mmps, int32_t dt_ms)
{
    int32_t s = sample_mmps;
    if (s > THREAT_CROSSING_SPEED_CLAMP_MMPS)
    {
        s = THREAT_CROSSING_SPEED_CLAMP_MMPS;
    }
    else if (s < -THREAT_CROSSING_SPEED_CLAMP_MMPS)
    {
        s = -THREAT_CROSSING_SPEED_CLAMP_MMPS;
    }
    else
    {
        /* in range */
    }
    return (int16_t)((int32_t)state + (((s - (int32_t)state) * dt_ms) / (THREAT_CROSSING_SMOOTH_MS + dt_ms)));
}

/* True if the slot's new position is further from its last one than a person can move
 * in the elapsed time: the radar has handed this slot to a different target (it
 * re-orders its slots when targets appear or drop out). */
static bool Track_Jumped(const Threat_Track_t * const p_trk, int16_t x_mm, uint16_t y_mm, int32_t dt_ms)
{
    const int32_t limit_mm = THREAT_TRACK_JUMP_MARGIN_MM + ((THREAT_TRACK_MAX_MMPS * dt_ms) / 1000);
    const int32_t dx = (int32_t)x_mm - (int32_t)p_trk->prev_x_mm;
    const int32_t dy = (int32_t)y_mm - (int32_t)p_trk->prev_y_mm;
    return (dx > limit_mm) || (dx < -limit_mm) || (dy > limit_mm) || (dy < -limit_mm);
}

/* Update the slot's sideways-speed estimate; true if the target should not be counted
 * because it is only crossing the user's path beyond the STOP zone. *p_new_target is
 * set when the slot now holds a different target, so its old history was dropped. */
static bool Track_IsCrossing(uint8_t slot,
                             int16_t x_mm,
                             uint16_t distance_mm,
                             uint32_t now_ms,
                             bool * const p_new_target)
{
    Threat_Track_t * const p_trk = &s_tracks[slot];
    const uint16_t y_mm = Forward_Mm(distance_mm, x_mm);
    bool known = false;

    *p_new_target = false;
    if ((p_trk->has_prev != 0U) &&
        ((uint32_t)(now_ms - p_trk->prev_ms) <= THREAT_CROSSING_MAX_GAP_MS) &&
        (now_ms != p_trk->prev_ms) &&
        Track_Jumped(p_trk, x_mm, y_mm, (int32_t)(now_ms - p_trk->prev_ms)))
    {
        *p_new_target = true; /* different target: start a fresh track */
        p_trk->has_prev = 0U;
    }

    if ((p_trk->has_prev != 0U) &&
        ((uint32_t)(now_ms - p_trk->prev_ms) <= THREAT_CROSSING_MAX_GAP_MS) &&
        (now_ms != p_trk->prev_ms))
    {
        const int32_t dt_ms = (int32_t)(now_ms - p_trk->prev_ms);
        const int32_t inst_x = (((int32_t)x_mm - (int32_t)p_trk->prev_x_mm) * 1000) / dt_ms;
        const int32_t inst_y = (((int32_t)y_mm - (int32_t)p_trk->prev_y_mm) * 1000) / dt_ms;
        p_trk->vx_mmps = Smooth_Speed(p_trk->vx_mmps, inst_x, dt_ms);
        p_trk->vy_mmps = Smooth_Speed(p_trk->vy_mmps, inst_y, dt_ms);
        known = true;
    }
    else
    {
        p_trk->vx_mmps = 0;
        p_trk->vy_mmps = 0;
    }
    p_trk->prev_x_mm = x_mm;
    p_trk->prev_y_mm = y_mm;
    p_trk->prev_ms = now_ms;
    p_trk->has_prev = 1U;

#if (THREAT_CROSSING_FILTER != 0)
    if (distance_mm <= THREAT_STOP_CLEAR_DIST_MM)
    {
        return false;
    }
    if (!known)
    {
        return false; /* no sideways speed yet: count it (safe default) */
    }
    const int32_t abs_vx = (p_trk->vx_mmps < 0) ? -(int32_t)p_trk->vx_mmps : (int32_t)p_trk->vx_mmps;
    const int32_t abs_vy = (p_trk->vy_mmps < 0) ? -(int32_t)p_trk->vy_mmps : (int32_t)p_trk->vy_mmps;
    return (abs_vx >= THREAT_CROSSING_MIN_MMPS) && (abs_vx > (THREAT_CROSSING_RATIO * abs_vy));
#else
    (void)distance_mm;
    (void)known;
    return false;
#endif
}

static Threat_Lane_t Classify_Side(int16_t x_mm, Threat_Lane_t last_side)
{
    const int16_t inner_mm = (int16_t)(THREAT_SIDE_DEADZONE_MM - THREAT_SIDE_HYSTERESIS_MM);
    Threat_Lane_t side = last_side;

    if (last_side == THREAT_LANE_LEFT)
    {
        if (x_mm > (int16_t)(-inner_mm))
        {
            side = (x_mm > THREAT_SIDE_DEADZONE_MM) ? THREAT_LANE_RIGHT : THREAT_LANE_CENTER;
        }
    }
    else if (last_side == THREAT_LANE_RIGHT)
    {
        if (x_mm < inner_mm)
        {
            side = (x_mm < (int16_t)(-THREAT_SIDE_DEADZONE_MM)) ? THREAT_LANE_LEFT : THREAT_LANE_CENTER;
        }
    }
    else
    {
        if (x_mm < (int16_t)(-THREAT_SIDE_DEADZONE_MM))
        {
            side = THREAT_LANE_LEFT;
        }
        else if (x_mm > THREAT_SIDE_DEADZONE_MM)
        {
            side = THREAT_LANE_RIGHT;
        }
        else
        {
            side = THREAT_LANE_CENTER;
        }
    }

    return side;
}

static void Update_LaneOccupancy(Threat_LaneState_t * const p_lane,
                                 bool present,
                                 uint16_t nearest_mm,
                                 uint32_t now_ms)
{
    const bool was_blocked = p_lane->blocked;

    if (present && (nearest_mm <= THREAT_ALERT_ENTER_DIST_MM))
    {
        p_lane->blocked = 1U;
        p_lane->last_dist_mm = nearest_mm;
        p_lane->hold_pending_ms = UINT32_MAX;
    }
    else if (present && (nearest_mm <= THREAT_ALERT_CLEAR_DIST_MM))
    {
        if (p_lane->blocked)
        {
            p_lane->last_dist_mm = nearest_mm;
            p_lane->hold_pending_ms = UINT32_MAX;
        }
    }
    else if (p_lane->blocked)
    {
        if (p_lane->hold_pending_ms == UINT32_MAX)
        {
            p_lane->hold_pending_ms = now_ms;
        }

        if ((uint32_t)(now_ms - p_lane->hold_pending_ms) >= THREAT_LANE_HOLD_MS)
        {
            p_lane->blocked = 0U;
            p_lane->hold_pending_ms = UINT32_MAX;
        }
    }
    else
    {
        /* Lane already clear */
    }

    if (p_lane->blocked)
    {
        p_lane->ever_blocked = 1U;
    }
    else if (was_blocked)
    {
        p_lane->cleared_at_ms = now_ms;
    }
    else
    {
        /* still clear: keep the time it cleared */
    }
}

/* A lane is "free" only if it is clear and has stayed clear long enough to trust. */
static bool Lane_IsFree(const Threat_LaneState_t * const p_lane, uint32_t now_ms)
{
    if (p_lane->blocked)
    {
        return false;
    }
    return (!p_lane->ever_blocked) ||
           ((uint32_t)(now_ms - p_lane->cleared_at_ms) >= THREAT_FREE_LANE_CLEAR_MS);
}

static int16_t Update_LaneSpeed(Threat_LaneState_t * const p_lane,
                                bool present,
                                int16_t live_speed_cms,
                                uint32_t now_ms)
{
    if (!present)
    {
        p_lane->held_closing_cms = 0;
        p_lane->speed_hold_pending_ms = UINT32_MAX;
        return 0;
    }

    if (live_speed_cms > 0)
    {
        p_lane->held_closing_cms = live_speed_cms;
        p_lane->speed_hold_pending_ms = UINT32_MAX;
        return live_speed_cms;
    }

    if (p_lane->speed_hold_pending_ms == UINT32_MAX)
    {
        p_lane->speed_hold_pending_ms = now_ms;
    }

    if ((p_lane->held_closing_cms > 0) &&
        ((uint32_t)(now_ms - p_lane->speed_hold_pending_ms) <= THREAT_SPEED_HOLD_MS))
    {
        return p_lane->held_closing_cms;
    }

    p_lane->held_closing_cms = 0;
    p_lane->speed_hold_pending_ms = UINT32_MAX;
    return 0;
}

/*
 * Convert the radar's raw speed to a closing speed (positive = approaching).
 * The decoded magnitude is at most 32767, so the negation fits int16_t.
 */
static int16_t Closing_Cms(int16_t raw_speed_cms)
{
#if (THREAT_NEGATIVE_SPEED_IS_APPROACHING != 0)
    return (int16_t)(-raw_speed_cms);
#else
    return raw_speed_cms;
#endif
}

static uint16_t Compute_Ttc20ms(uint16_t dist_mm, int16_t closing_speed_cms)
{
    if (closing_speed_cms > 0)
    {
        /* Direct 20 ms ticks: floor(distance_mm * 5 / speed_cms).
         * The uint32_t intermediate bounds even UINT16_MAX * 5 safely.
         */
        uint32_t ticks = ((uint32_t)dist_mm * 5U) / (uint32_t)closing_speed_cms;
        return (ticks <= BLE_TTC_MAX_TICKS) ? (uint16_t)(ticks & 0x0FFFU) : BLE_TTC_NO_THREAT;
    }

    return BLE_TTC_NO_THREAT;
}

/* Side part of the cue: which neighbouring lane is confirmed free
 * (0 = centre clear, 3 = no confirmed-free lane). */
static uint8_t Compute_DirectionCue(bool center_blocked, bool left_free, bool right_free)
{
    if (!center_blocked)
    {
        return 0U;
    }
    if (left_free)
    {
        return (uint8_t)((uint8_t)THREAT_LANE_LEFT << THREAT_CUE_SIDE_SHIFT);
    }
    if (right_free)
    {
        return (uint8_t)((uint8_t)THREAT_LANE_RIGHT << THREAT_CUE_SIDE_SHIFT);
    }
    return (uint8_t)((uint8_t)THREAT_CUE_SIDE_ALL_BLOCKED << THREAT_CUE_SIDE_SHIFT);
}

/* STOP: the user is about to walk into an obstacle in their own (centre) lane.
 * With all three lanes blocked the centre is blocked too, so it needs no extra case. */
static bool Compute_Stop(bool center_blocked, uint16_t center_dist_mm, bool fast_approach)
{
    if (!center_blocked)
    {
        s_stop_latched = false;
    }
    else if ((center_dist_mm <= THREAT_STOP_ENTER_DIST_MM) || fast_approach)
    {
        s_stop_latched = true;
    }
    else if ((center_dist_mm > THREAT_STOP_CLEAR_DIST_MM) && !fast_approach)
    {
        s_stop_latched = false;
    }
    else
    {
        /* between enter and clear: keep the previous state */
    }
    return s_stop_latched;
}

uint8_t ThreatAssessment_GetBlockedMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t lane = 0U; lane < (uint8_t)THREAT_LANE_COUNT; lane++)
    {
        if (s_lanes[lane].blocked)
        {
            mask |= (uint8_t)(1U << lane);
        }
    }
    return mask;
}

void ThreatAssessment_Init(void)
{
    Reset_AllState();
    s_synthetic_ms = 0U;
}

void ThreatAssessment_ProcessTargetsAt(const Radar_Target_t * const p_targets,
                                       uint8_t target_count,
                                       Grip_State_t cane_grip_telemetry,
                                       uint32_t now_ms,
                                       Threat_Result_t * const p_out_result)
{
    if (p_out_result == NULL)
    {
        return;
    }

    /*
     * FR6: Suppress head-level obstacle audio alerts when the cane is explicitly
     * confirmed as released (GRIP_STATE_RELEASED). If the cane is held or wireless
     * grip telemetry is unconfirmed/lost (GRIP_STATE_UNKNOWN), obstacle alerts
     * remain active as a safety fail-safe. As in putra-branch, the lanes are
     * still scanned while released so their occupancy stays current; only the
     * reported result is forced clear (see the end of this function).
     */
    bool lane_present[THREAT_LANE_COUNT] = { false, false, false };
    uint16_t lane_nearest_mm[THREAT_LANE_COUNT] = { UINT16_MAX, UINT16_MAX, UINT16_MAX };
    int16_t lane_live_speed[THREAT_LANE_COUNT] = { 0, 0, 0 };

    uint16_t min_valid_dist = UINT16_MAX;
    int16_t min_valid_speed = 0;
    bool found_valid = false;

    const uint8_t safe_count = (p_targets != NULL) ? target_count : 0U;

    for (uint8_t i = 0U; i < safe_count; i++)
    {
        const Radar_Target_t * const p_t = &p_targets[i];

        if (!p_t->is_valid)
        {
            if (i < 3U)
            {
                s_target_last_side[i] = THREAT_LANE_CENTER;
                s_tracks[i].has_prev = 0U;
            }
            continue;
        }

        bool new_target = false;
        const bool crossing = (i < 3U) &&
            Track_IsCrossing(i, Lateral_Mm(p_t->x_mm), p_t->distance_mm, now_ms, &new_target);
        if (new_target)
        {
            s_target_last_side[i] = THREAT_LANE_CENTER; /* classify afresh, no hysteresis carry-over */
        }

        /* Corridor gate: ignore targets outside +/- THREAT_CORRIDOR_HALF_MM */
        const int16_t x_mm = Lateral_Mm(p_t->x_mm);
        if ((x_mm < (int16_t)(-THREAT_CORRIDOR_HALF_MM)) ||
            (x_mm > THREAT_CORRIDOR_HALF_MM))
        {
            if (i < 3U)
            {
                s_target_last_side[i] = THREAT_LANE_CENTER;
            }
            continue;
        }

        if (crossing)
        {
            continue; /* only crossing the path beyond the STOP zone */
        }

        const int16_t closing_cms = Closing_Cms(p_t->speed_cms);

        Threat_Lane_t last_side = (i < 3U) ? s_target_last_side[i] : THREAT_LANE_CENTER;
        Threat_Lane_t side = Classify_Side(x_mm, last_side);
        if (i < 3U)
        {
            s_target_last_side[i] = side;
        }

        if (!lane_present[side] || (p_t->distance_mm < lane_nearest_mm[side]))
        {
            lane_present[side] = true;
            lane_nearest_mm[side] = p_t->distance_mm;
            lane_live_speed[side] = closing_cms;
        }

        if (!found_valid || (p_t->distance_mm < min_valid_dist))
        {
            found_valid = true;
            min_valid_dist = p_t->distance_mm;
            min_valid_speed = closing_cms;
        }
    }

    for (uint8_t i = safe_count; i < 3U; i++)
    {
        s_target_last_side[i] = THREAT_LANE_CENTER;
    }

    int16_t lane_eff_speed[THREAT_LANE_COUNT] = { 0, 0, 0 };
    bool any_blocked = false;

    for (uint8_t lane = 0U; lane < (uint8_t)THREAT_LANE_COUNT; lane++)
    {
        Update_LaneOccupancy(&s_lanes[lane], lane_present[lane], lane_nearest_mm[lane], now_ms);
        lane_eff_speed[lane] = Update_LaneSpeed(&s_lanes[lane], lane_present[lane], lane_live_speed[lane], now_ms);
        if (s_lanes[lane].blocked)
        {
            any_blocked = true;
        }
    }

    /* Fast approach in the user's own lane (jogger, cyclist): STOP on time-to-collision
     * as well as distance. The closing-speed floor is well above walking pace, so a
     * walking target still gets STOP from distance alone. */
    {
        const Threat_LaneState_t * const p_c = &s_lanes[THREAT_LANE_CENTER];
        const int16_t c_speed = lane_eff_speed[THREAT_LANE_CENTER];
        const bool fast_now = (p_c->blocked != 0U) && (c_speed >= THREAT_FAST_CLOSING_MIN_CMS) &&
            (Compute_Ttc20ms(p_c->last_dist_mm, c_speed) <= THREAT_FAST_STOP_TTC_20MS);
        if (!fast_now)
        {
            s_fast_frames = 0U;
        }
        else if (s_fast_frames < UINT8_MAX)
        {
            s_fast_frames++;
        }
        else
        {
            /* saturated */
        }
    }
    const bool fast_approach = (s_fast_frames >= THREAT_FAST_STOP_FRAMES);

    if (any_blocked)
    {
        uint16_t min_blocked_dist = UINT16_MAX;
        uint8_t nearest_lane = 0U;
        uint16_t best_ttc = BLE_TTC_NO_THREAT;
        int16_t best_ttc_speed = 0;

        for (uint8_t lane = 0U; lane < (uint8_t)THREAT_LANE_COUNT; lane++)
        {
            if (!s_lanes[lane].blocked)
            {
                continue;
            }

            if (s_lanes[lane].last_dist_mm < min_blocked_dist)
            {
                min_blocked_dist = s_lanes[lane].last_dist_mm;
                nearest_lane = lane;
            }

            if (lane_eff_speed[lane] > 0)
            {
                uint16_t lane_ttc = Compute_Ttc20ms(s_lanes[lane].last_dist_mm, lane_eff_speed[lane]);
                if (lane_ttc < best_ttc)
                {
                    best_ttc = lane_ttc;
                    best_ttc_speed = lane_eff_speed[lane];
                }
            }
        }

        p_out_result->is_threat_active = 1U;
        p_out_result->nearest_distance_mm = min_blocked_dist;
        uint8_t cue = Compute_DirectionCue(
            s_lanes[THREAT_LANE_CENTER].blocked,
            Lane_IsFree(&s_lanes[THREAT_LANE_LEFT], now_ms),
            Lane_IsFree(&s_lanes[THREAT_LANE_RIGHT], now_ms));
        if (Compute_Stop(s_lanes[THREAT_LANE_CENTER].blocked,
                         s_lanes[THREAT_LANE_CENTER].last_dist_mm, fast_approach))
        {
            cue |= (uint8_t)THREAT_CUE_STOP;
        }
        p_out_result->open_direction_cue = (uint16_t)(cue & 0x07U);

        if (best_ttc < BLE_TTC_NO_THREAT)
        {
            p_out_result->ttc_20ms = (uint16_t)(best_ttc & 0x0FFFU);
            p_out_result->closing_speed_cms = best_ttc_speed;
        }
        else
        {
            p_out_result->ttc_20ms = BLE_TTC_NO_THREAT;
            if (lane_eff_speed[nearest_lane] > 0)
            {
                p_out_result->closing_speed_cms = lane_eff_speed[nearest_lane];
            }
            else
            {
                p_out_result->closing_speed_cms = lane_present[nearest_lane] ? lane_live_speed[nearest_lane] : 0;
            }
        }
    }
    else if (found_valid)
    {
        s_stop_latched = false;
        p_out_result->is_threat_active = 0U;
        p_out_result->nearest_distance_mm = min_valid_dist;
        p_out_result->closing_speed_cms = min_valid_speed;
        p_out_result->ttc_20ms = (uint16_t)(Compute_Ttc20ms(min_valid_dist, min_valid_speed) & 0x0FFFU);
        p_out_result->open_direction_cue = 0U;
    }
    else
    {
        s_stop_latched = false;
        Clear_Result(p_out_result);
    }

    if (cane_grip_telemetry == GRIP_STATE_RELEASED)
    {
        /* FR6: a confirmed release silences the alert regardless of the lanes. */
        Clear_Result(p_out_result);
    }
}

void ThreatAssessment_ProcessTargets(const Radar_Target_t * const p_targets,
                                     uint8_t target_count,
                                     Grip_State_t cane_grip_telemetry,
                                     Threat_Result_t * const p_out_result)
{
    for (uint8_t lane = 0U; lane < (uint8_t)THREAT_LANE_COUNT; lane++)
    {
        if (s_lanes[lane].blocked && (s_lanes[lane].hold_pending_ms == UINT32_MAX))
        {
            s_lanes[lane].hold_pending_ms = s_synthetic_ms;
        }
        if ((s_lanes[lane].held_closing_cms > 0) && (s_lanes[lane].speed_hold_pending_ms == UINT32_MAX))
        {
            s_lanes[lane].speed_hold_pending_ms = s_synthetic_ms;
        }
    }

    s_synthetic_ms += THREAT_LANE_HOLD_MS + 1U;
    ThreatAssessment_ProcessTargetsAt(p_targets, target_count, cane_grip_telemetry,
                                      s_synthetic_ms, p_out_result);
}
