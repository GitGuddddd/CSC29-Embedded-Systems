#include "AlertManager.h"
#include "AlertManager_Cfg.h"
#include <string.h>

/*
 * \file AlertManager.c
 * \brief Audio/Haptic Alert Arbitration and Distance-to-Pitch mapping
 * Compliance: Barr-C:2018
 */

static bool s_fall_alarm_latched = false;
static bool s_fall_alarm_muted = false;
static uint32_t s_fall_start_time_ms = 0U;
static Alert_Type_t s_previous_request = ALERT_NONE;
static uint32_t s_request_start_ms = 0U;

void AlertManager_Init(void)
{
    s_fall_alarm_latched = false;
    s_fall_alarm_muted = false;
    s_fall_start_time_ms = 0U;
    s_previous_request = ALERT_NONE;
    s_request_start_ms = 0U;
}

void AlertManager_AcknowledgeReset(void)
{
    s_fall_alarm_latched = false;
    s_fall_alarm_muted = false;
    s_fall_start_time_ms = 0U;
}

void AlertManager_SetFallState(bool latched, bool paused, uint32_t now_ms)
{
    if (latched && (!s_fall_alarm_latched || (s_fall_alarm_muted && !paused)))
    {
        s_fall_start_time_ms = now_ms;
    }
    s_fall_alarm_latched = latched;
    s_fall_alarm_muted = latched && paused;
}

static bool Fall_PulseOn(uint32_t elapsed_ms)
{
    static const uint16_t phases_ms[] = {
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_MEDIUM_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_LONG_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_MEDIUM_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_BURST_GAP_MS
    };
    uint32_t t = elapsed_ms % ALERT_FALL_PERIOD_MS;
    for (uint32_t i = 0U; i < (sizeof(phases_ms) / sizeof(phases_ms[0])); ++i)
    {
        if (t < phases_ms[i])
        {
            return ((i % 2U) == 0U);
        }
        t -= phases_ms[i];
    }
    return false;
}

uint16_t AlertManager_MapDistanceToPitch(uint16_t distance_mm)
{
    if (distance_mm >= ALERT_OBSTACLE_MAX_DIST_MM)
    {
        return ALERT_BUZZER_MIN_FREQ_HZ;
    }
    if (distance_mm <= ALERT_OBSTACLE_MIN_DIST_MM)
    {
        return ALERT_BUZZER_MAX_FREQ_HZ;
    }

    /* Linear interpolation between 2000mm (1500Hz) and 300mm (3500Hz) */
    uint32_t dist_span = ALERT_OBSTACLE_MAX_DIST_MM - ALERT_OBSTACLE_MIN_DIST_MM; /* 1700 */
    uint32_t freq_span = ALERT_BUZZER_MAX_FREQ_HZ - ALERT_BUZZER_MIN_FREQ_HZ;     /* 2000 */
    uint32_t offset = ALERT_OBSTACLE_MAX_DIST_MM - distance_mm;

    uint32_t pitch = ALERT_BUZZER_MIN_FREQ_HZ + ((offset * freq_span) / dist_span);
    return (uint16_t)pitch;
}

void AlertManager_Arbitrate(Alert_Type_t requested_alert,
                            uint16_t param,
                            uint32_t now_ms,
                            Alert_Output_t * const p_out)
{
    if (p_out == NULL)
    {
        return;
    }

    memset(p_out, 0, sizeof(*p_out));

    if (requested_alert != s_previous_request)
    {
        s_previous_request = requested_alert;
        s_request_start_ms = now_ms;
    }

    if (requested_alert == ALERT_FALL_ALARM)
    {
        if (!s_fall_alarm_latched)
        {
            s_fall_alarm_latched = true;
            s_fall_alarm_muted = false;
            s_fall_start_time_ms = now_ms;
        }
    }

    /* Output whether fall is latched regardless of acoustic mute */
    p_out->fall_latched = s_fall_alarm_latched;

    /* Priority 1: Fall Emergency */
    if (s_fall_alarm_latched)
    {
        if (s_fall_alarm_muted)
        {
            p_out->active_alert = ALERT_FALL_MUTED;
            p_out->buzzer_active = false; /* Bounded pause is owned by the mode manager. */
            p_out->haptic_active = false; /* FR4: Inhibit haptics */
        }
        else
        {
            p_out->active_alert = ALERT_FALL_ALARM;
            p_out->buzzer_active = Fall_PulseOn((uint32_t)(now_ms - s_fall_start_time_ms));
            p_out->generated_freq_hz = ALERT_FALL_FREQ_A_HZ;
            p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
            p_out->haptic_active = false; /* FR4: Inhibit haptics */
        }
        return;
    }

    /* Priority 2: Head-fall delivery fallback on the glasses. This deliberately
     * uses a sparse, low-duty cue rather than the cane's emergency siren. Bench
     * sound-pressure validation is still required for the final enclosure.
     */
    if (requested_alert == ALERT_FALL_DELIVERY_FAULT)
    {
        p_out->active_alert = ALERT_FALL_DELIVERY_FAULT;
        p_out->buzzer_active = ((uint32_t)(now_ms - s_request_start_ms) %
                                ALERT_DELIVERY_PERIOD_MS) < ALERT_DELIVERY_PULSE_MS;
        p_out->generated_freq_hz = ALERT_DELIVERY_FREQ_HZ;
        p_out->generated_duty_percent = ALERT_DELIVERY_DUTY_PERCENT;
        return;
    }

    /* Priority 3: Cliff Warning (Cane Haptic only) */
    if (requested_alert == ALERT_CLIFF_WARNING)
    {
        p_out->active_alert = ALERT_CLIFF_WARNING;
        p_out->haptic_active = true;
        p_out->buzzer_active = false;
        return;
    }

    /* Priority 4: Obstacle Proximity (Glasses Buzzer) */
    if (requested_alert == ALERT_OBSTACLE_PROXIMITY)
    {
        p_out->active_alert = ALERT_OBSTACLE_PROXIMITY;
        p_out->buzzer_active = true;
        p_out->generated_freq_hz = AlertManager_MapDistanceToPitch(param);
        p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
        return;
    }

    /* Priority 5: Sensor Fault Tone */
    if (requested_alert == ALERT_SENSOR_FAULT)
    {
        p_out->active_alert = ALERT_SENSOR_FAULT;
        uint32_t t = (uint32_t)(now_ms - s_request_start_ms) % ALERT_FAULT_PERIOD_MS;
        p_out->buzzer_active = (t < ALERT_FAULT_PULSE_MS) ||
                              ((t >= (ALERT_FAULT_PULSE_MS + ALERT_FAULT_GAP_MS)) &&
                               (t < ((2U * ALERT_FAULT_PULSE_MS) + ALERT_FAULT_GAP_MS)));
        p_out->generated_freq_hz = ALERT_FAULT_FREQ_HZ;
        p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
        return;
    }

    /* Priority 6: Link Loss Haptic Warning (FR16: degraded mode) */
    if (requested_alert == ALERT_LINK_LOSS)
    {
        p_out->active_alert = ALERT_LINK_LOSS;
        p_out->haptic_active = true;
        p_out->buzzer_active = false;
        return;
    }
}
