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
 * Which side of the walking path a target is on (FR9 cue byte). Decided by
 * radar_monitor_apply_corridor() from x_mm, before x_mm itself is dropped.
 * CENTER for an empty slot. Code 3 is reserved (an unpacked corrupt slot
 * reads as CENTER, the safe value: no direction is suggested).
 */
typedef enum
{
    RTE_SIDE_CENTER = 0,
    RTE_SIDE_LEFT   = 1,
    RTE_SIDE_RIGHT  = 2
} rte_cue_side_t;

/**
 * One radar target slot. An empty slot has b_valid set to false.
 * x_mm and y_mm exist only between the driver and the debug print; they are
 * not in the radar queue message. side IS in the message (2 bits): the
 * geometry it needs is already gone by the time Glasses_Threat sees the
 * target, so radar_monitor works it out first and only the small verdict
 * travels. d_mm is the straight-line distance worked out by the driver (at
 * most 46339).
 */
typedef struct
{
    bool           b_valid;
    int16_t        x_mm;
    int16_t        y_mm;
    uint16_t       d_mm;
    int16_t        speed_cms;
    rte_cue_side_t side;
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

/**
 * Size of one packed radar queue message (108 bits used of 112, see
 * rte_pack.h). 13 bytes (102 bits) until the FR9 cue byte work (8 Nov): a
 * side field was added to each of the 3 target slots.
 */
#define RTE_RADAR_MSG_BYTES    (14u)

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

/**
 * Item of the Audio Alert queue (radar related variants only). cue is
 * meaningful for RTE_AUDIO_OBSTACLE only; every other type sends 0.
 */
typedef struct
{
    rte_audio_msg_type_t type;
    uint16_t             d_mm;
    uint8_t              cue;
} rte_audio_msg_t;

/**
 * FR9 cue byte (prof feedback, 7 Nov 2026): tells Audio Alert what to do,
 * not just how close. OPEN-20/OPEN-22 (docs/deviations.md): bit 0 is STOP
 * (all three lanes - left/middle/right - are BLOCKED); bits 1-2 are which
 * way is open to walk (rte_cue_side_t), from per-lane occupancy
 * (threat_assessment_direction_cue()), not the side of a single alerting
 * target. Bits 3-7 are always 0. A bit-flag byte describing the obstacle,
 * not a reject mask (OPEN-4): there is nothing here to reject, every bit
 * says something true about the alert that is already active.
 */
#define RTE_CUE_STOP        (0x01u)
#define RTE_CUE_SIDE_SHIFT  (1u)
#define RTE_CUE_SIDE_MASK   (0x03u)

/**
 * Item of the BLE obstacle queue (OPEN-25, 14 Nov): one distance and one
 * TTC per lane, not a single nearest-lane summary - the nearest lane by
 * distance is not always the one closing in soonest (a farther lane
 * approaching fast can have a shorter TTC than a nearer, slower one), so
 * reporting only one lane's numbers could understate urgency on another.
 * Both arrays are indexed by rte_cue_side_t (CENTER/LEFT/RIGHT). A lane
 * that is not currently BLOCKED reports RTE_LANE_NONE_MM / RTE_TTC_NONE_MS
 * in its slot; a BLOCKED lane with no live speed_cms reading this frame
 * (only held via its hold-grace window) reports a real d_mm but
 * RTE_TTC_NONE_MS. See docs/deviations.md OPEN-25.
 */
typedef struct
{
    bool     b_active;
    uint16_t d_mm[3];
    uint16_t ttc_ms[3];
} rte_ble_obstacle_t;

/** Time to collision value meaning "not approaching". */
#define RTE_TTC_NONE_MS    (0xFFFFu)

/** Per-lane BLE distance value meaning "this lane is not BLOCKED"
 *  (OPEN-25). Safe as a sentinel: threat_assessment_distance_mm() never
 *  returns above 46339. */
#define RTE_LANE_NONE_MM    (0xFFFFu)

#endif /* RTE_TYPE_H */

/*** end of file ***/
