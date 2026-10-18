#include "GroundProjection.h"
#include "GroundProjection_Cfg.h"
#include <string.h>

/*
 * \file GroundProjection.c
 * \brief Implementation of Ground-height classifier using integer math
 * Compliance: Barr-C:2018
 */

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
    p_d->last_sample_ms = now_ms;

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

    p_d->is_calibrated = false;
    p_d->baseline_mm = 0U;
    p_d->filtered_mm = 0U;

    Reset_Votes(p_d);
    Reset_History(p_d);
    Reset_CalibrationSamples(p_d);

    p_d->last_sample_ms = now_ms;
    p_d->state = (p_d->is_armed) ? GP_STATE_CALIBRATING : GP_STATE_DISARMED;
}

void GroundProjection_Tick(Gp_Detector_t * const p_d, uint32_t now_ms)
{
    if ((p_d == NULL) || (!p_d->is_armed))
    {
        return;
    }

    if ((uint32_t)(now_ms - p_d->last_sample_ms) >= GP_STALE_TIMEOUT_MS)
    {
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
    p_d->cal_count = 1U;
    p_d->cal_sum = mm;
    p_d->cal_min = mm;
    p_d->cal_max = mm;
}

void GroundProjection_Feed(Gp_Detector_t * const p_d,
                           Gp_Quality_t quality,
                           uint16_t range_mm,
                           uint32_t now_ms)
{
    if ((p_d == NULL) || (!p_d->is_armed))
    {
        return;
    }

    GroundProjection_Tick(p_d, now_ms);

    if ((quality == GP_RANGE_VALID) &&
        ((range_mm < GP_MIN_RANGE_MM) || (range_mm > GP_MAX_RANGE_MM)))
    {
        quality = GP_RANGE_INVALID;
    }

    if (quality == GP_RANGE_INVALID)
    {
        Reset_Votes(p_d);
        Reset_History(p_d);

        if ((p_d->state != GP_STATE_CALIBRATING) &&
            (p_d->state != GP_STATE_SENSOR_FAULT))
        {
            p_d->state = GP_STATE_UNKNOWN;
        }
        return;
    }

    p_d->last_sample_ms = now_ms;

    if (quality == GP_RANGE_NO_RETURN)
    {
        Reset_History(p_d);
        Reset_CalibrationSamples(p_d);

        if (p_d->state != GP_STATE_CALIBRATING)
        {
            Vote(p_d, GP_STATE_NO_GROUND);
            if (p_d->state != GP_STATE_NO_GROUND)
            {
                p_d->state = GP_STATE_UNKNOWN;
            }
        }
        return;
    }

    if (p_d->state == GP_STATE_CALIBRATING)
    {
        if (p_d->cal_count == 0U)
        {
            Start_Fresh_CalibrationWindow(p_d, range_mm);
        }
        else
        {
            uint16_t lo = (range_mm < p_d->cal_min) ? range_mm : p_d->cal_min;
            uint16_t hi = (range_mm > p_d->cal_max) ? range_mm : p_d->cal_max;

            if ((uint16_t)(hi - lo) > GP_CAL_SPREAD_MM)
            {
                Start_Fresh_CalibrationWindow(p_d, range_mm);
            }
            else
            {
                p_d->cal_min = lo;
                p_d->cal_max = hi;
                p_d->cal_sum += range_mm;

                if (p_d->cal_count < GP_CAL_SAMPLES)
                {
                    ++p_d->cal_count;
                }
            }
        }

        if (p_d->cal_count >= GP_CAL_SAMPLES)
        {
            p_d->baseline_mm = (uint16_t)(p_d->cal_sum / GP_CAL_SAMPLES);
            p_d->filtered_mm = p_d->baseline_mm;
            p_d->is_calibrated = true;

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

    Push_History(p_d, range_mm);

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
        default:                    return "INVALID_STATE";
    }
}
