#include "ThreatAssessment.h"
#include "ThreatAssessment_Cfg.h"
#include <string.h>

/*
 * \file ThreatAssessment.c
 * \brief Pure obstacle threat assessment logic with hysteresis (FR8, FR13)
 * Compliance: Barr-C:2018
 */

static bool s_alert_latched = false;

void ThreatAssessment_Init(void)
{
    s_alert_latched = false;
}

void ThreatAssessment_ProcessTargets(const Radar_Target_t * const p_targets,
                                     uint8_t target_count,
                                     Grip_State_t cane_grip_telemetry,
                                     Threat_Result_t * const p_out_result)
{
    if ((p_out_result == NULL) || (p_targets == NULL) || (target_count == 0U))
    {
        if (p_out_result != NULL)
        {
            memset(p_out_result, 0, sizeof(*p_out_result));
        }
        s_alert_latched = false;
        return;
    }

    /*
     * FR6: Suppress head-level obstacle audio alerts when the cane is explicitly 
     * confirmed as released (GRIP_STATE_RELEASED). If the cane is held or wireless 
     * grip telemetry is unconfirmed/lost (GRIP_STATE_UNKNOWN), obstacle alerts 
     * remain active as a safety fail-safe.
     */
    if (cane_grip_telemetry == GRIP_STATE_RELEASED)
    {
        s_alert_latched = false;
        memset(p_out_result, 0, sizeof(*p_out_result));
        return;
    }

    uint16_t min_dist = UINT16_MAX;
    int16_t min_dist_speed = 0;
    bool found_valid = false;

    for (uint8_t i = 0U; i < target_count; i++)
    {
        const Radar_Target_t * p_t = &p_targets[i];

        if (!p_t->is_valid)
        {
            continue;
        }

        /* Ignore departing targets moving away faster than 10 cm/s */
        if (p_t->speed_cms < THREAT_DEPARTING_SPEED_LIMIT)
        {
            continue;
        }

        if (p_t->distance_mm < min_dist)
        {
            min_dist = p_t->distance_mm;
            min_dist_speed = p_t->speed_cms;
            found_valid = true;
        }
    }

    if (!found_valid)
    {
        s_alert_latched = false;
        memset(p_out_result, 0, sizeof(*p_out_result));
        return;
    }

    /* Apply Hysteresis: Enter < 2000mm, Clear > 2100mm */
    if (!s_alert_latched)
    {
        if (min_dist <= THREAT_ALERT_ENTER_DIST_MM)
        {
            s_alert_latched = true;
        }
    }
    else
    {
        if (min_dist > THREAT_ALERT_CLEAR_DIST_MM)
        {
            s_alert_latched = false;
        }
    }

    p_out_result->is_threat_active = s_alert_latched;
    p_out_result->nearest_distance_mm = min_dist;
    p_out_result->closing_speed_cms = min_dist_speed;

    /* Compute TTC = distance_mm / (speed_cms * 10) in seconds -> convert to ms */
    if (min_dist_speed > 0)
    {
        /* speed in mm/s is speed_cms * 10 */
        uint32_t speed_mms = (uint32_t)min_dist_speed * 10U;
        p_out_result->time_to_collision_ms = (uint16_t)((min_dist * 1000U) / speed_mms);
    }
    else
    {
        p_out_result->time_to_collision_ms = UINT16_MAX;
    }
}
