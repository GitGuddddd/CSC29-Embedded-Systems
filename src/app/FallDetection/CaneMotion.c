#include "CaneMotion.h"
#include <math.h>
#include <stddef.h>

/* Initial bench thresholds for the 100 Hz cane IMU. */
#define CANE_SAMPLE_GAP_MS          (25U)
#define CANE_GYRO_ONLY_MS           (750U)
#define CANE_GYRO_LIMIT_DPS         (990.0f)  /* IMU range is +/-1000 deg/s. */
#define CANE_QUIET_GYRO_SQ          (625.0f)  /* (25 deg/s)^2 */
#define CANE_UP_SEED_SAMPLES        (3U)
#define CANE_UP_SEED_COS            (0.98480775f) /* cos(10 degrees) */
#define CANE_LIFT_G                (0.45f)
#define CANE_LIFT_SIDE_G            (0.25f)
#define CANE_LIFT_WINDOW_MS         (250U)
#define CANE_LOW_G_SQ               (0.25f)   /* (0.5 g)^2 */
#define CANE_IMPACT_G_SQ            (6.25f)   /* (2.5 g)^2 */
#define CANE_LOW_G_MIN_MS           (30U)
#define CANE_LOW_G_MIN_SAMPLES      (3U)
#define CANE_RELEASED_DROP_MS       (60U)
#define CANE_LIFT_DROP_MS           (160U)
#define CANE_IMPACT_WINDOW_MS       (800U)
#define CANE_SLIP_G                (0.55f)
#define CANE_SLIP_GYRO_SQ           (4900.0f) /* (70 deg/s)^2 */
#define CANE_SLIP_MIN_MS            (30U)
#define CANE_TILT_COS               (0.76604444f) /* cos(40 degrees) */
#define CANE_TILT_STILL_MS          (250U)
#define CANE_CONFIRM_WINDOW_MS      (1500U)

static float Dot(const float a[3], const float b[3])
{
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static void Normalize(float out[3], const float v[3])
{
    float scale = 1.0f / sqrtf(Dot(v, v));
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        out[i] = v[i] * scale;
    }
}

static void Seed_Up(CaneMotion_t * const p_motion, const float accel[3], bool quiet, uint32_t now_ms)
{
    if (!quiet)
    {
        p_motion->reference_samples = 0U;
        return;
    }

    float measured[3];
    Normalize(measured, accel);
    if ((p_motion->reference_samples == 0U) || (Dot(p_motion->up, measured) < CANE_UP_SEED_COS))
    {
        p_motion->reference_samples = 1U;
    }
    else
    {
        ++p_motion->reference_samples;
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        p_motion->up[i] = measured[i];
    }
    if (p_motion->reference_samples == CANE_UP_SEED_SAMPLES)
    {
        p_motion->up_valid = true;
        p_motion->last_reference_ms = now_ms;
        p_motion->reference_samples = 0U;
    }
}

/* A stationary accelerometer measures about +1 g in the world-up direction.
 * Rotate that reference in sensor coordinates using the gyroscope. Strong
 * acceleration, impact and free fall cannot be used to correct it. */
static void Update_Up(CaneMotion_t * const p_motion, const Imu_Sample_t * const p_sample,
                      const float accel[3], bool quiet, uint32_t elapsed_ms)
{
    uint32_t now_ms = p_sample->timestamp_ms;
    if ((fabsf(p_sample->gx) >= CANE_GYRO_LIMIT_DPS) ||
        (fabsf(p_sample->gy) >= CANE_GYRO_LIMIT_DPS) ||
        (fabsf(p_sample->gz) >= CANE_GYRO_LIMIT_DPS))
    {
        p_motion->up_valid = false;
        p_motion->reference_samples = 0U;
        return;
    }

    if (p_motion->up_valid && ((now_ms - p_motion->last_reference_ms) > CANE_GYRO_ONLY_MS))
    {
        p_motion->up_valid = false;
        p_motion->direction_history_valid = false;
    }
    if (!p_motion->up_valid)
    {
        Seed_Up(p_motion, accel, quiet, now_ms);
        return;
    }

    float * const up = p_motion->up;
    float radians = (float)elapsed_ms * 0.00001745329252f;
    /* Normalized midpoint integration of d(up)/dt = up x gyro. */
    const float rx = p_sample->gx * radians;
    const float ry = p_sample->gy * radians;
    const float rz = p_sample->gz * radians;
    const float mid[3] = {
        up[0] + (0.5f * ((up[1] * rz) - (up[2] * ry))),
        up[1] + (0.5f * ((up[2] * rx) - (up[0] * rz))),
        up[2] + (0.5f * ((up[0] * ry) - (up[1] * rx)))
    };
    const float predicted[3] = {
        up[0] + (mid[1] * rz) - (mid[2] * ry),
        up[1] + (mid[2] * rx) - (mid[0] * rz),
        up[2] + (mid[0] * ry) - (mid[1] * rx)
    };
    Normalize(up, predicted);

    if (quiet)
    {
        float measured[3];
        Normalize(measured, accel);
        if (Dot(up, measured) >= 0.97f)
        {
            float alpha = (float)elapsed_ms / (500.0f + (float)elapsed_ms);
            for (unsigned int i = 0U; i < 3U; ++i)
            {
                up[i] += alpha * (measured[i] - up[i]);
            }
            Normalize(up, up);
            p_motion->last_reference_ms = now_ms;
        }
    }
}

static float Median_Of_Three(float a, float b, float c)
{
    if (a > b)
    {
        float swap = a;
        a = b;
        b = swap;
    }
    if (c < a)
    {
        return a;
    }
    return (c < b) ? c : b;
}

static void Measure_Direction(CaneMotion_t * const p_motion, const float accel[3], float a_sq,
                              bool new_sample, CaneMotion_Result_t * const p_result)
{
    if (!p_motion->up_valid)
    {
        p_motion->direction_history_valid = false;
        return;
    }

    float along_up = Dot(accel, p_motion->up);
    CaneMotion_Direction_t raw = {
        .vertical_g = along_up - 1.0f,
        .horizontal_g = sqrtf(fmaxf(0.0f, a_sq - (along_up * along_up)))
    };
    CaneMotion_Direction_t * const history = p_motion->direction_history;
    if (!p_motion->direction_history_valid)
    {
        /* The first trusted reading follows a coherent quiet seed. */
        for (unsigned int i = 0U; i < 3U; ++i)
        {
            history[i] = raw;
        }
        p_motion->next_direction_sample = 0U;
        p_motion->direction_history_valid = true;
    }
    else if (new_sample)
    {
        history[p_motion->next_direction_sample] = raw;
        p_motion->next_direction_sample = (uint8_t)((p_motion->next_direction_sample + 1U) % 3U);
    }

    p_result->vertical_raw_g = raw.vertical_g;
    p_result->horizontal_raw_g = raw.horizontal_g;
    p_result->vertical_g = Median_Of_Three(history[0].vertical_g, history[1].vertical_g, history[2].vertical_g);
    p_result->horizontal_g = Median_Of_Three(history[0].horizontal_g, history[1].horizontal_g,
                                            history[2].horizontal_g);
}

static void Set_State(CaneMotion_t * const p_motion, CaneMotion_State_t state, uint32_t now_ms)
{
    p_motion->state = state;
    p_motion->state_since_ms = now_ms;
    p_motion->tilted_still = false;
}

static void Start_Motion(CaneMotion_t * const p_motion, CaneMotion_State_t state, uint32_t now_ms)
{
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        p_motion->onset_up[i] = p_motion->up[i];
    }
    p_motion->onset_up_valid = p_motion->up_valid;
    Set_State(p_motion, state, now_ms);
}

static bool Tilt_Settled(CaneMotion_t * const p_motion, const float accel[3],
                         float magnitude_g, bool quiet, uint32_t now_ms)
{
    if (!p_motion->onset_up_valid || !quiet ||
        (Dot(p_motion->onset_up, accel) >= (CANE_TILT_COS * magnitude_g)))
    {
        p_motion->tilted_still = false;
        return false;
    }
    if (!p_motion->tilted_still)
    {
        p_motion->tilted_since_ms = now_ms;
        p_motion->tilted_still = true;
    }
    return (now_ms - p_motion->tilted_since_ms) >= CANE_TILT_STILL_MS;
}

static CaneMotion_Event_t Accept_Impact(CaneMotion_t * const p_motion, bool released, uint32_t now_ms)
{
    /* An abrupt catch after a lift can look like a landing. Confirm changed
     * posture before alarming on this shorter, ambiguous sequence. */
    if (released && !p_motion->low_g_after_lift)
    {
        return CANE_MOTION_EVENT_DROP;
    }
    Set_State(p_motion, CANE_MOTION_CHECK_TILT, now_ms);
    return CANE_MOTION_EVENT_NONE;
}

CaneMotion_Result_t CaneMotion_Step(CaneMotion_t * const p_motion,
                                  const Imu_Sample_t * const p_sample, Grip_State_t grip,
                                  CaneMotion_AlarmPolicy_t policy)
{
    CaneMotion_Result_t result = {0};
    if (p_sample == NULL)
    {
        *p_motion = (CaneMotion_t){0};
        return result;
    }

    uint32_t now_ms = p_sample->timestamp_ms;
    uint32_t elapsed_ms = now_ms - p_motion->last_sample_ms;
    bool repeated = p_motion->have_sample && (elapsed_ms == 0U);
    if (elapsed_ms > CANE_SAMPLE_GAP_MS)
    {
        *p_motion = (CaneMotion_t){0};
        elapsed_ms = 0U;
    }
    p_motion->have_sample = true;
    p_motion->last_sample_ms = now_ms;

    const float accel[3] = {p_sample->ax, p_sample->ay, p_sample->az};
    const float gyro[3] = {p_sample->gx, p_sample->gy, p_sample->gz};
    float a_sq = Dot(accel, accel);
    float g_sq = Dot(gyro, gyro);
    if (!isfinite(a_sq) || !isfinite(g_sq))
    {
        *p_motion = (CaneMotion_t){0};
        return result;
    }
    bool quiet = (a_sq >= 0.81f) && (a_sq <= 1.21f) && (g_sq <= CANE_QUIET_GYRO_SQ);
    bool full_policy = policy == CANE_MOTION_ALARM_FULL;
    bool low_g = full_policy && (a_sq < CANE_LOW_G_SQ);
    bool impact = full_policy && (a_sq > CANE_IMPACT_G_SQ);
    bool released = grip == GRIP_STATE_RELEASED;
    result.accel_g = sqrtf(a_sq);
    result.rotation_dps = sqrtf(g_sq);
    if (!repeated)
    {
        Update_Up(p_motion, p_sample, accel, quiet, elapsed_ms);
    }
    result.up_valid = p_motion->up_valid;

    Measure_Direction(p_motion, accel, a_sq, !repeated, &result);
    if (repeated)
    {
        result.state = p_motion->state;
        result.pending = p_motion->state != CANE_MOTION_IDLE;
        return result;
    }
    if (!low_g)
    {
        p_motion->low_g_reported = false;
    }
    if (p_motion->low_g_reported)
    {
        /* A single continuous released drop produces one event. */
        return result;
    }

    if (p_motion->lift_active && ((now_ms - p_motion->last_lift_ms) >= CANE_LIFT_WINDOW_MS))
    {
        p_motion->lift_active = false;
    }
    bool lifting = full_policy && (p_motion->state == CANE_MOTION_IDLE) && result.up_valid &&
                   (result.vertical_g >= CANE_LIFT_G) && (result.horizontal_g <= CANE_LIFT_SIDE_G);
    /* The median can retain the old sign for one frame at the end of a pulse. */
    if (lifting && p_motion->lift_candidate && (result.vertical_raw_g >= CANE_LIFT_G) &&
        (result.horizontal_raw_g <= CANE_LIFT_SIDE_G))
    {
        if (!p_motion->lift_active)
        {
            result.event = CANE_MOTION_EVENT_LIFT;
        }
        p_motion->lift_active = true;
        p_motion->last_lift_ms = now_ms;
    }
    p_motion->lift_candidate = lifting;

    /* Keep raw low-g evidence even if a held cane changes from LOW_G to
     * SLIP_START. A debounced release can arrive after that transition. */
    if (low_g)
    {
        if (p_motion->low_g_samples == 0U)
        {
            p_motion->low_g_since_ms = now_ms;
            p_motion->low_g_after_lift = p_motion->lift_active;
        }
        if (p_motion->low_g_samples < CANE_LOW_G_MIN_SAMPLES)
        {
            ++p_motion->low_g_samples;
        }
    }
    uint32_t low_g_duration_ms = now_ms - p_motion->low_g_since_ms;
    bool qualified_low_g = (p_motion->low_g_samples >= CANE_LOW_G_MIN_SAMPLES) &&
                           (low_g_duration_ms >= CANE_LOW_G_MIN_MS);
    if (!low_g)
    {
        p_motion->low_g_samples = 0U;
    }

    bool downward_slip = full_policy && result.up_valid && !p_motion->lift_active &&
                         (result.vertical_g <= -CANE_SLIP_G) && (g_sq >= CANE_SLIP_GYRO_SQ);
    bool sideways_slip = result.up_valid && (result.horizontal_g >= CANE_SLIP_G);
    bool slipping = downward_slip || sideways_slip;
    bool tracking_collapse = (p_motion->state == CANE_MOTION_SLIP_TRACK) ||
                             (p_motion->state == CANE_MOTION_CHECK_TILT);
    /* Rotation and unloading together can be a held tip slip. A released
     * drop takes precedence even if a slip was already being tracked. */
    if ((low_g || (released && qualified_low_g)) && (p_motion->state != CANE_MOTION_LOW_G) &&
        (released || (!downward_slip && !tracking_collapse)))
    {
        if (p_motion->state != CANE_MOTION_WAIT_IMPACT)
        {
            Start_Motion(p_motion, CANE_MOTION_LOW_G, now_ms);
        }
        else
        {
            Set_State(p_motion, CANE_MOTION_LOW_G, now_ms);
        }
    }

    uint32_t duration_ms = now_ms - p_motion->state_since_ms;
    switch (p_motion->state)
    {
        case CANE_MOTION_IDLE:
            if (slipping)
            {
                Start_Motion(p_motion, CANE_MOTION_SLIP_START, now_ms);
                p_motion->slip_is_downward = !sideways_slip;
            }
            break;

        case CANE_MOTION_LOW_G:
        {
            uint32_t released_ms = p_motion->low_g_after_lift ? CANE_LIFT_DROP_MS : CANE_RELEASED_DROP_MS;
            if (released && qualified_low_g && (low_g_duration_ms >= released_ms) && (low_g || impact))
            {
                result.event = CANE_MOTION_EVENT_DROP;
            }
            else if (!released && low_g && downward_slip)
            {
                Set_State(p_motion, CANE_MOTION_SLIP_START, now_ms);
                p_motion->slip_is_downward = true;
            }
            else if (!low_g && !qualified_low_g)
            {
                Set_State(p_motion, CANE_MOTION_IDLE, now_ms);
            }
            else if (impact)
            {
                result.event = Accept_Impact(p_motion, released, now_ms);
            }
            else if (!low_g)
            {
                Set_State(p_motion, CANE_MOTION_WAIT_IMPACT, now_ms);
            }
            break;
        }

        case CANE_MOTION_WAIT_IMPACT:
            if (duration_ms > CANE_IMPACT_WINDOW_MS)
            {
                Set_State(p_motion, CANE_MOTION_IDLE, now_ms);
            }
            else if (impact)
            {
                result.event = Accept_Impact(p_motion, released, now_ms);
            }
            break;

        case CANE_MOTION_SLIP_START:
            if (!(p_motion->slip_is_downward ? downward_slip : sideways_slip))
            {
                /* Don't combine unrelated downward and sideways excursions. */
                Set_State(p_motion, slipping ? CANE_MOTION_SLIP_START : CANE_MOTION_IDLE, now_ms);
                p_motion->slip_is_downward = !sideways_slip;
            }
            else if (duration_ms >= CANE_SLIP_MIN_MS)
            {
                result.event = CANE_MOTION_EVENT_SLIP;
                Set_State(p_motion, CANE_MOTION_SLIP_TRACK, now_ms);
            }
            break;

        case CANE_MOTION_SLIP_TRACK:
        case CANE_MOTION_CHECK_TILT:
            if (duration_ms > CANE_CONFIRM_WINDOW_MS)
            {
                Set_State(p_motion, CANE_MOTION_IDLE, now_ms);
            }
            else if (Tilt_Settled(p_motion, accel, result.accel_g, quiet, now_ms))
            {
                result.event = CANE_MOTION_EVENT_FALL;
            }
            break;

        default:
            Set_State(p_motion, CANE_MOTION_IDLE, now_ms);
            break;
    }

    if ((result.event == CANE_MOTION_EVENT_DROP) || (result.event == CANE_MOTION_EVENT_FALL))
    {
        p_motion->low_g_reported = (result.event == CANE_MOTION_EVENT_DROP) && low_g;
        p_motion->low_g_samples = 0U;
        Set_State(p_motion, CANE_MOTION_IDLE, now_ms);
    }
    result.state = p_motion->state;
    result.pending = p_motion->state != CANE_MOTION_IDLE;
    return result;
}
