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

void AlertManager_Init(void)
{
    s_fall_alarm_latched = false;
    s_fall_alarm_muted = false;
    s_fall_start_time_ms = 0U;
}

void AlertManager_AcknowledgeReset(void)
{
    s_fall_alarm_latched = false;
    s_fall_alarm_muted = false;
    s_fall_start_time_ms = 0U;
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

    /*
     * FR3 & NFR4: If 30 seconds elapse without explicit reset, 
     * transition from active siren into muted latched state (SYS_FALL_MUTED).
     */
    if (s_fall_alarm_latched && (!s_fall_alarm_muted) &&
        ((now_ms - s_fall_start_time_ms) >= ALERT_MAX_FALL_TIMEOUT_MS))
    {
        s_fall_alarm_muted = true;
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
            p_out->buzzer_active = false; /* Muted after 30 seconds */
            p_out->haptic_active = false; /* FR4: Inhibit haptics */
        }
        else
        {
            p_out->active_alert = ALERT_FALL_ALARM;
            p_out->buzzer_active = true;
            /* Toggle alternating fall alarm frequency every 250ms */
            p_out->generated_freq_hz = (((now_ms / 250U) % 2U) == 0U) ?
                                       ALERT_FALL_FREQ_A_HZ : ALERT_FALL_FREQ_B_HZ;
            p_out->haptic_active = false; /* FR4: Inhibit haptics */
        }
        return;
    }

    /* Priority 2: Cliff Warning (Cane Haptic only) */
    if (requested_alert == ALERT_CLIFF_WARNING)
    {
        p_out->active_alert = ALERT_CLIFF_WARNING;
        p_out->haptic_active = true;
        p_out->buzzer_active = false;
        return;
    }

    /* Priority 3: Obstacle Proximity (Glasses Buzzer) */
    if (requested_alert == ALERT_OBSTACLE_PROXIMITY)
    {
        p_out->active_alert = ALERT_OBSTACLE_PROXIMITY;
        p_out->buzzer_active = true;
        p_out->generated_freq_hz = AlertManager_MapDistanceToPitch(param);
        return;
    }

    /* Priority 4: Sensor Fault Tone */
    if (requested_alert == ALERT_SENSOR_FAULT)
    {
        p_out->active_alert = ALERT_SENSOR_FAULT;
        p_out->buzzer_active = true;
        p_out->generated_freq_hz = ALERT_FAULT_FREQ_HZ;
        return;
    }

    /* Priority 5: Link Loss Haptic Warning (FR16: degraded mode) */
    if (requested_alert == ALERT_LINK_LOSS)
    {
        p_out->active_alert = ALERT_LINK_LOSS;
        p_out->haptic_active = true;
        p_out->buzzer_active = false;
        return;
    }
}
