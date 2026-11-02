/** @file rte_type.h
 *
 * @brief Types shared between drivers, application components and the RTE.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * NOTE: The audio and BLE message layouts are placeholders until the Audio
 * Alert owner and the BLE owner agree them (see docs/deviations.md, OPEN-1).
 * The same holds for the grip message from the BLE task (OPEN-12).
 */

#ifndef RTE_TYPE_H
#define RTE_TYPE_H

#include <stdbool.h>
#include <stdint.h>

/** The LD2450 reports up to three targets per frame. */
#define RTE_RADAR_NUM_TARGETS    (3u)

/**
 * One radar target slot. An empty slot has b_valid set to false.
 * x_mm and y_mm exist only between the driver and the debug print; they are
 * not in the radar queue message. d_mm is the straight-line distance worked
 * out by the driver (at most 46339).
 */
typedef struct
{
    bool     b_valid;
    int16_t  x_mm;
    int16_t  y_mm;
    uint16_t d_mm;
    int16_t  speed_cms;
} rte_radar_target_t;

/**
 * Grip state of the cane as seen by the glasses (report 6.6, FR6).
 * UNKNOWN is the start-up value and the value the BLE task must send when no
 * cane packet has arrived for 500 ms. Glasses_Threat treats UNKNOWN like
 * HELD, so only a confirmed RELEASED silences the obstacle alert.
 */
typedef enum
{
    RTE_GRIP_UNKNOWN  = 0,
    RTE_GRIP_HELD     = 1,
    RTE_GRIP_RELEASED = 2
} rte_grip_state_t;

/** Kind of message Glasses_Threat receives on the radar queue. */
typedef enum
{
    RTE_RADAR_MSG_FRAME = 0,
    RTE_RADAR_MSG_FAULT = 1,
    RTE_RADAR_MSG_GRIP  = 2
} rte_radar_msg_type_t;

/** Size of one packed radar queue message (102 bits, see rte_pack.h). */
#define RTE_RADAR_MSG_BYTES    (13u)

/**
 * Item of the radar queue (to Glasses_Threat): the packed bytes, copied whole
 * through the kernel queue. Use rte_pack_radar() and
 * rte_pack_unpack_radar(); layout in docs/radar_message_layout.md.
 */
typedef struct
{
    uint8_t bytes[RTE_RADAR_MSG_BYTES];
} rte_radar_msg_t;

/**
 * The same message as ordinary fields, for the code that works with it.
 * Frames and faults come from Glasses_Radar, grip state from the Glasses BLE
 * task. Only the fields of the given type are meaningful. In a message that
 * went through the queue, x_mm and y_mm are 0 and d_mm has a 2 mm step.
 */
typedef struct
{
    rte_radar_msg_type_t type;
    uint32_t             time_ms;
    rte_grip_state_t     grip;
    rte_radar_target_t   targets[RTE_RADAR_NUM_TARGETS];
} rte_radar_data_t;

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
