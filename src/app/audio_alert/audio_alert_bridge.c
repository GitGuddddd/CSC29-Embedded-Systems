/** @file audio_alert_bridge.c
 *
 * @brief BRING-UP ONLY: maps radar messages onto Brian's alert.c calls.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stddef.h>
#include <stdint.h>

#include "alert.h"
#include "audio_alert_bridge.h"
#include "rte_type.h"

// alert_set_obstacle: a value of 0 or less means silent.
#define AUDIO_ALERT_BRIDGE_SILENT    (0)

/*!
 * @brief Hand one radar message to the alert code.
 *
 * OBSTACLE(d) sets the distance, CLEAR silences the obstacle tone,
 * RADAR_FAULT and RADAR_OK set and clear the radar fault bit. The tone itself
 * is made later by alert_update(), which the caller runs regularly.
 *
 * @param[in] p_msg  Message from the Audio Alert queue; NULL is ignored.
 */
void
audio_alert_bridge_apply (const rte_audio_msg_t * p_msg)
{
    if (NULL == p_msg)
    {
        return;
    }

    switch (p_msg->type)
    {
        case RTE_AUDIO_OBSTACLE:
            alert_set_obstacle((int32_t) p_msg->d_mm);
            break;

        case RTE_AUDIO_OBSTACLE_CLEAR:
            alert_set_obstacle(AUDIO_ALERT_BRIDGE_SILENT);
            break;

        case RTE_AUDIO_RADAR_FAULT:
            alert_set_fault(ALERT_FAULT_RADAR, true);
            break;

        case RTE_AUDIO_RADAR_OK:
            alert_set_fault(ALERT_FAULT_RADAR, false);
            break;

        default:
            break;
    }
}

/*** end of file ***/
