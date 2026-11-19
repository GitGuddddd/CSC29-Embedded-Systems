#include "GroundProjection.h"
#include "GroundProjection_Cfg.h"
#include <math.h>
#include <string.h>

/*
 * \file GroundProjection.c
 * \brief Implementation of ground-height classification and calibration
 * Compliance: Barr-C:2018
 */

_Static_assert(GP_CAL_TILT_MIN_SAMPLES > 0U && GP_CAL_TILT_MIN_SAMPLES <= GP_CAL_SAMPLES,
               "Tilt calibration must fit within the range calibration window");

static void Reset_Votes(Gp_Detector_t * const p_d)
{
    p_d->candidate = GP_STATE_UNKNOWN;
    p_d->candidate_count = 0U;
}

static void Reset_History(Gp_Detector_t * const p_d)
{
    p_d->history_count = 0U;
    p_d->history_next = 0U;
    memset(p_d->history, 0, sizeof(p_d->history));
}

static void Reset_CalibrationSamples(Gp_Detector_t * const p_d)
{
    p_d->cal_count = 0U;
    p_d->cal_sum = 0U;
    p_d->cal_min = UINT16_MAX;
    p_d->cal_max = 0U;
    p_d->cal_tilt_count = 0U;
    memset(p_d->cal_tilt_sum, 0, sizeof(p_d->cal_tilt_sum));
}

void GroundProjection_Init(Gp_Detector_t * const p_d, uint32_t now_ms)
{
    if (p_d == NULL)
    {
        return;
    }

    memset(p_d, 0, sizeof(*p_d));
    p_d->state = GP_STATE_DISARMED;
    p_d->candidate = GP_STATE_UNKNOWN;
    p_d->last_sample_ms = now_ms;

    Reset_History(p_d);
    Reset_CalibrationSamples(p_d);
}

void GroundProjection_SetArmed(Gp_Detector_t * const p_d, bool armed, uint32_t now_ms)
{
    if ((p_d == NULL) || (p_d->is_armed == armed))
    {
        return;
    }

    p_d->is_armed = armed;
    p_d->is_calibrating = false;
    p_d->last_sample_ms = now_ms;

    if (!armed)
    {
        /* Nobody can feel a calibration outcome while the cane is released. A pending
         * event (e.g. CANCELLED on let-go) must not replay as a CAL_FAILED buzz on the
         * next grip, which only re-arms without restarting calibration once calibrated.
         */
        p_d->cal_event = GP_CAL_EVENT_NONE;
    }

    Reset_Votes(p_d);
    Reset_History(p_d);
    Reset_CalibrationSamples(p_d);

    p_d->state = (armed) ? GP_STATE_UNKNOWN : GP_STATE_DISARMED;
}

void GroundProjection_StartCalibration(Gp_Detector_t * const p_d, uint32_t now_ms)
{
    if (p_d == NULL)
    {
        return;
    }

    /* Staged: the previous baseline stays valid until the new one completes */
    Reset_Votes(p_d);
    Reset_History(p_d);
    Reset_CalibrationSamples(p_d);
    p_d->change_pending = false;

    p_d->cal_start_ms = now_ms;
    p_d->cal_event = GP_CAL_EVENT_NONE;
    p_d->cal_tilt_saved = false;
    p_d->last_sample_ms = now_ms;
    p_d->is_calibrating = p_d->is_armed;
    p_d->state = p_d->is_calibrating ? GP_STATE_CALIBRATING : GP_STATE_DISARMED;
}

/* Drop the partial window; classification resumes on the old baseline if there is one */
static void Abort_Calibration(Gp_Detector_t * const p_d, uint32_t now_ms, Gp_CalEvent_t reason)
{
    Reset_Votes(p_d);
    Reset_History(p_d);
    Reset_CalibrationSamples(p_d);

    p_d->is_calibrating = false;
    p_d->cal_event = reason;
    p_d->last_sample_ms = now_ms;
    p_d->state = GP_STATE_UNKNOWN;
}

void GroundProjection_CancelCalibration(Gp_Detector_t * const p_d, uint32_t now_ms)
{
    if ((p_d == NULL) || (!p_d->is_calibrating))
    {
        return;
    }

    Abort_Calibration(p_d, now_ms, GP_CAL_EVENT_CANCELLED);
}

Gp_CalEvent_t GroundProjection_TakeCalEvent(Gp_Detector_t * const p_d)
{
    if (p_d == NULL)
    {
        return GP_CAL_EVENT_NONE;
    }

    Gp_CalEvent_t event = p_d->cal_event;
    p_d->cal_event = GP_CAL_EVENT_NONE;
    return event;
}

void GroundProjection_Tick(Gp_Detector_t * const p_d, uint32_t now_ms)
{
    if ((p_d == NULL) || (!p_d->is_armed))
    {
        return;
    }

    /* Not enough steady readings in time (cane moving, uneven ground): give up */
    if (p_d->is_calibrating &&
        ((uint32_t)(now_ms - p_d->cal_start_ms) >= GP_CAL_TIMEOUT_MS))
    {
        Abort_Calibration(p_d, now_ms, GP_CAL_EVENT_TIMEOUT);
    }

    if ((uint32_t)(now_ms - p_d->last_sample_ms) >= GP_STALE_TIMEOUT_MS)
    {
        if (p_d->is_calibrating)
        {
            p_d->cal_event = GP_CAL_EVENT_SENSOR_FAULT;
            p_d->is_calibrating = false;
        }
        p_d->state = GP_STATE_SENSOR_FAULT;
        Reset_Votes(p_d);
        Reset_History(p_d);
        Reset_CalibrationSamples(p_d);
    }
}

static void Vote(Gp_Detector_t * const p_d, Gp_State_t next)
{
    if (p_d->candidate != next)
    {
        p_d->candidate = next;
        p_d->candidate_count = 1U;
    }
    else if (p_d->candidate_count < GP_CONFIRM_SAMPLES)
    {
        ++p_d->candidate_count;
    }
    else
    {
        /* Count maintained at maximum */
    }

    if (p_d->candidate_count >= GP_CONFIRM_SAMPLES)
    {
        p_d->state = next;
    }
}

static uint16_t Median5(const uint16_t * const v)
{
    uint16_t a[5];
    memcpy(a, v, sizeof(a));

    for (uint32_t i = 1U; i < 5U; ++i)
    {
        uint16_t x = a[i];
        uint32_t j = i;

        while ((j > 0U) && (a[j - 1U] > x))
        {
            a[j] = a[j - 1U];
            --j;
        }
        a[j] = x;
    }

    return a[2];
}

static void Push_History(Gp_Detector_t * const p_d, uint16_t mm)
{
    p_d->history[p_d->history_next] = mm;
    p_d->history_next = (uint8_t)((p_d->history_next + 1U) % 5U);

    if (p_d->history_count < 5U)
    {
        ++p_d->history_count;
    }
}

static void Start_Fresh_CalibrationWindow(Gp_Detector_t * const p_d, uint16_t mm)
{
    Reset_CalibrationSamples(p_d);
    p_d->cal_count = 1U;
    p_d->cal_sum = mm;
    p_d->cal_min = mm;
    p_d->cal_max = mm;
}

static void Save_TiltReference(Gp_Detector_t * const p_d)
{
    if (p_d->cal_tilt_count < GP_CAL_TILT_MIN_SAMPLES)
    {
        return;
    }

    float count = (float)p_d->cal_tilt_count;
    float x = p_d->cal_tilt_sum[0] / count;
    float y = p_d->cal_tilt_sum[1] / count;
    float z = p_d->cal_tilt_sum[2] / count;
    float magnitude_sq = (x * x) + (y * y) + (z * z);
    if (isfinite(magnitude_sq) && (magnitude_sq >= GP_CAL_TILT_MEAN_MIN_SQ))
    {
        p_d->tilt_reference[0] = x;
        p_d->tilt_reference[1] = y;
        p_d->tilt_reference[2] = z;
        p_d->is_tilt_calibrated = true;
        p_d->cal_tilt_saved = true;
    }
}

void GroundProjection_Feed(Gp_Detector_t * const p_d,
                           Gp_Quality_t quality,
                           uint16_t range_mm,
                           uint32_t now_ms)
{
    GroundProjection_FeedWithImu(p_d, quality, range_mm, now_ms, NULL);
}

void GroundProjection_FeedWithImu(Gp_Detector_t * const p_d,
                                 Gp_Quality_t quality,
                                 uint16_t range_mm,
                                 uint32_t now_ms,
                                 const Imu_Sample_t * const p_imu)
{
    if ((quality == GP_RANGE_VALID) &&
        ((range_mm < GP_MIN_RANGE_MM) || (range_mm > GP_MAX_RANGE_MM)))
    {
        quality = GP_RANGE_INVALID;
    }
    GroundProjection_FeedProjected(p_d, quality, range_mm, now_ms, p_imu);
}

void GroundProjection_FeedProjected(Gp_Detector_t * const p_d,
                                    Gp_Quality_t quality,
                                    uint16_t vertical_mm,
                                    uint32_t now_ms,
                                    const Imu_Sample_t * const p_imu)
{
    if ((p_d == NULL) || (!p_d->is_armed))
    {
        return;
    }

    GroundProjection_Tick(p_d, now_ms);

    if (quality == GP_RANGE_INVALID)
    {
        Reset_Votes(p_d);
        Reset_History(p_d);

        if ((!p_d->is_calibrating) &&
            (p_d->state != GP_STATE_POSE_UNAVAILABLE) &&
            (p_d->state != GP_STATE_SENSOR_FAULT))
        {
            p_d->state = GP_STATE_UNKNOWN;
        }
        return;
    }

    p_d->last_sample_ms = now_ms;

    if (quality == GP_RANGE_POSE_UNAVAILABLE)
    {
        Reset_History(p_d);
        Reset_Votes(p_d);
        Reset_CalibrationSamples(p_d);
        p_d->change_pending = false;
        p_d->state = GP_STATE_POSE_UNAVAILABLE;
        return;
    }

    if (p_d->state == GP_STATE_POSE_UNAVAILABLE)
    {
        p_d->state = p_d->is_calibrating ? GP_STATE_CALIBRATING : GP_STATE_UNKNOWN;
    }

    if (quality == GP_RANGE_NO_RETURN)
    {
        Reset_History(p_d);
        Reset_CalibrationSamples(p_d);

        if (!p_d->is_calibrating)
        {
            Vote(p_d, GP_STATE_NO_GROUND);
            if (p_d->state != GP_STATE_NO_GROUND)
            {
                p_d->state = GP_STATE_UNKNOWN;
            }
        }
        return;
    }

    if (p_d->is_calibrating)
    {
        if (p_d->cal_count == 0U)
        {
            Start_Fresh_CalibrationWindow(p_d, vertical_mm);
        }
        else
        {
            uint16_t lo = (vertical_mm < p_d->cal_min) ? vertical_mm : p_d->cal_min;
            uint16_t hi = (vertical_mm > p_d->cal_max) ? vertical_mm : p_d->cal_max;

            if ((uint16_t)(hi - lo) > GP_CAL_SPREAD_MM)
            {
                Start_Fresh_CalibrationWindow(p_d, vertical_mm);
            }
            else
            {
                p_d->cal_min = lo;
                p_d->cal_max = hi;
                p_d->cal_sum += vertical_mm;

                if (p_d->cal_count < GP_CAL_SAMPLES)
                {
                    ++p_d->cal_count;
                }
            }
        }

        if (p_imu != NULL)
        {
            p_d->cal_tilt_sum[0] += p_imu->ax;
            p_d->cal_tilt_sum[1] += p_imu->ay;
            p_d->cal_tilt_sum[2] += p_imu->az;
            ++p_d->cal_tilt_count;
        }

        if (p_d->cal_count >= GP_CAL_SAMPLES)
        {
            p_d->baseline_mm = (uint16_t)(p_d->cal_sum / GP_CAL_SAMPLES);
            p_d->filtered_mm = p_d->baseline_mm;
            p_d->is_calibrating = false;
            p_d->is_calibrated = true;
            Save_TiltReference(p_d);
            p_d->cal_event = GP_CAL_EVENT_DONE;

            Reset_History(p_d);
            Reset_Votes(p_d);
            p_d->state = GP_STATE_LEVEL;
        }
        return;
    }

    if (!p_d->is_calibrated)
    {
        p_d->state = GP_STATE_UNKNOWN;
        return;
    }

    /* Mark the first raw reading past a step threshold (latency start point) */
    if ((p_d->state != GP_STATE_STEP_UP) && (p_d->state != GP_STATE_STEP_DOWN))
    {
        int32_t raw_delta = (int32_t)vertical_mm - (int32_t)p_d->baseline_mm;
        bool raw_changed = (raw_delta > (int32_t)GP_DOWN_ENTER_MM) ||
                           (raw_delta <= -(int32_t)GP_UP_ENTER_MM);

        if (raw_changed && (!p_d->change_pending))
        {
            p_d->change_pending = true;
            p_d->change_start_ms = now_ms;
        }
        else if (!raw_changed)
        {
            p_d->change_pending = false;
        }
        else
        {
            /* Change already being timed */
        }
    }

    Push_History(p_d, vertical_mm);

    if (p_d->history_count < 5U)
    {
        return;
    }

    p_d->filtered_mm = Median5(p_d->history);
    int32_t delta = (int32_t)p_d->filtered_mm - (int32_t)p_d->baseline_mm;
    Gp_State_t next;

    if (p_d->state == GP_STATE_STEP_UP)
    {
        next = (delta < -(int32_t)GP_UP_CLEAR_MM) ? GP_STATE_STEP_UP : GP_STATE_LEVEL;
    }
    else if (p_d->state == GP_STATE_STEP_DOWN)
    {
        next = (delta > (int32_t)GP_DOWN_CLEAR_MM) ? GP_STATE_STEP_DOWN : GP_STATE_LEVEL;
    }
    else
    {
        if (delta <= -(int32_t)GP_UP_ENTER_MM)
        {
            next = GP_STATE_STEP_UP;
        }
        else if (delta > (int32_t)GP_DOWN_ENTER_MM)
        {
            next = GP_STATE_STEP_DOWN;
            /* If catastrophic drop > 250mm, fast-path immediately to STEP_DOWN */
            if (delta > (int32_t)GP_DOWN_FASTPATH_MM)
            {
                p_d->candidate = GP_STATE_STEP_DOWN;
                p_d->candidate_count = GP_CONFIRM_SAMPLES;
                p_d->state = GP_STATE_STEP_DOWN;
                return;
            }
        }
        else
        {
            next = GP_STATE_LEVEL;
        }
    }

    Vote(p_d, next);

    /* Integer EMA baseline tracking while stably LEVEL */
    if ((p_d->state == GP_STATE_LEVEL) && (next == GP_STATE_LEVEL))
    {
        int32_t error = (int32_t)p_d->filtered_mm - (int32_t)p_d->baseline_mm;
        int32_t correction = (error * GP_EMA_NUM) >> GP_EMA_DEN_SHIFT;
        int32_t updated = (int32_t)p_d->baseline_mm + correction;

        if (updated < 0)
        {
            updated = 0;
        }
        else if (updated > 65535)
        {
            updated = 65535;
        }
        else
        {
            /* In-range */
        }

        p_d->baseline_mm = (uint16_t)updated;
    }
}

Alert_Type_t GroundProjection_GetHapticRequest(const Gp_Detector_t * const p_d)
{
    if ((p_d == NULL) || (!p_d->is_armed))
    {
        return ALERT_NONE;
    }

    switch (p_d->state)
    {
        case GP_STATE_STEP_DOWN:
            return ALERT_CLIFF_WARNING;
        case GP_STATE_STEP_UP:
            return ALERT_CLIFF_STEP_UP;
        case GP_STATE_NO_GROUND:
        case GP_STATE_SENSOR_FAULT:
            return ALERT_SENSOR_FAULT;
        case GP_STATE_UNKNOWN:
            /*
             * No baseline after a reboot means step warnings are off: prompt the
             * user to calibrate (Tribunal 1, 12:50PM). A calibrated UNKNOWN is a
             * short gap while the filter refills, so it stays silent; a gap that
             * lasts 500 ms becomes SENSOR_FAULT and gets the caution cue.
             */
            return (p_d->is_calibrated) ? ALERT_NONE : ALERT_CALIBRATE_PROMPT;
        default:
            return ALERT_NONE;
    }
}

const char * GroundProjection_GetCalEventName(Gp_CalEvent_t event)
{
    switch (event)
    {
        case GP_CAL_EVENT_NONE:         return "NONE";
        case GP_CAL_EVENT_DONE:         return "DONE";
        case GP_CAL_EVENT_TIMEOUT:      return "TIMEOUT (no steady level reading within 10 s)";
        case GP_CAL_EVENT_SENSOR_FAULT: return "SENSOR_FAULT (no ToF reading for 500 ms)";
        case GP_CAL_EVENT_CANCELLED:    return "CANCELLED";
        default:                        return "INVALID_EVENT";
    }
}

const char * GroundProjection_GetStateName(Gp_State_t state)
{
    switch (state)
    {
        case GP_STATE_DISARMED:     return "DISARMED";
        case GP_STATE_CALIBRATING:  return "CALIBRATING";
        case GP_STATE_UNKNOWN:      return "UNKNOWN";
        case GP_STATE_LEVEL:        return "LEVEL";
        case GP_STATE_STEP_UP:      return "STEP_UP";
        case GP_STATE_STEP_DOWN:    return "STEP_DOWN";
        case GP_STATE_NO_GROUND:    return "NO_GROUND";
        case GP_STATE_SENSOR_FAULT: return "SENSOR_FAULT";
        case GP_STATE_POSE_UNAVAILABLE: return "GROUND_UNAVAILABLE";
        default:                    return "INVALID_STATE";
    }
}
