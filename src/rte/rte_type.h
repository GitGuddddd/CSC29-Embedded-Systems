/** @file rte_type.h
 *
 * @brief Types shared between drivers, application components and the RTE.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * NOTE: The audio and BLE message layouts are placeholders until the Audio
 * Alert owner and the BLE owner agree them (see docs/deviations.md, OPEN-1).
 */

#ifndef RTE_TYPE_H
#define RTE_TYPE_H

#include <stdbool.h>
#include <stdint.h>

/** The LD2450 reports up to three targets per frame. */
#define RTE_RADAR_NUM_TARGETS    (3u)

/** One radar target slot. An empty slot has b_valid set to false. */
typedef struct
{
    bool     b_valid;
    int16_t  x_mm;
    int16_t  y_mm;
    int16_t  speed_cms;
    uint16_t resolution_mm;
} rte_radar_target_t;

/** Kind of message Glasses_Radar sends to Glasses_Threat. */
typedef enum
{
    RTE_RADAR_MSG_FRAME = 0,
    RTE_RADAR_MSG_FAULT = 1
} rte_radar_msg_type_t;

/** Item of the radar queue (Glasses_Radar to Glasses_Threat). */
typedef struct
{
    rte_radar_msg_type_t type;
    uint32_t             time_ms;
    rte_radar_target_t   targets[RTE_RADAR_NUM_TARGETS];
} rte_radar_msg_t;

/** Kind of message sent to the Audio Alert task. */
typedef enum
{
    RTE_AUDIO_OBSTACLE       = 0,
    RTE_AUDIO_OBSTACLE_CLEAR = 1,
    RTE_AUDIO_RADAR_FAULT    = 2,
    RTE_AUDIO_RADAR_OK       = 3
} rte_audio_msg_type_t;

/** Item of the Audio Alert queue (radar related variants only). */
typedef struct
{
    rte_audio_msg_type_t type;
    uint16_t             d_mm;
} rte_audio_msg_t;

/** Item of the BLE obstacle queue. ttc_ms is RTE_TTC_NONE_MS if unknown. */
typedef struct
{
    bool     b_active;
    uint16_t d_mm;
    uint16_t ttc_ms;
} rte_ble_obstacle_t;

/** Time to collision value meaning "not approaching". */
#define RTE_TTC_NONE_MS    (0xFFFFu)

#endif /* RTE_TYPE_H */

/*** end of file ***/
