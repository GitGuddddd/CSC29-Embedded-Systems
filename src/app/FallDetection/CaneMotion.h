#ifndef CANEMOTION_H
#define CANEMOTION_H

#include "Rte_Type.h"

typedef enum
{
    CANE_MOTION_IDLE = 0U,
    CANE_MOTION_LOW_G,
    CANE_MOTION_WAIT_IMPACT,
    CANE_MOTION_SLIP_START,
    CANE_MOTION_SLIP_TRACK,
    CANE_MOTION_CHECK_TILT
} CaneMotion_State_t;

typedef enum
{
    CANE_MOTION_EVENT_NONE = 0U,
    CANE_MOTION_EVENT_LIFT,
    CANE_MOTION_EVENT_SLIP,
    CANE_MOTION_EVENT_DROP,
    CANE_MOTION_EVENT_FALL
} CaneMotion_Event_t;

typedef enum
{
    CANE_MOTION_ALARM_FULL = 0U,
    CANE_MOTION_ALARM_SLIP_ONLY
} CaneMotion_AlarmPolicy_t;

typedef struct
{
    float vertical_g;
    float horizontal_g;
} CaneMotion_Direction_t;

typedef struct
{
    CaneMotion_State_t state;
    float up[3];
    float onset_up[3];
    CaneMotion_Direction_t direction_history[3];
    uint32_t last_sample_ms;
    uint32_t last_reference_ms;
    uint32_t last_lift_ms;
    uint32_t low_g_since_ms;
    uint32_t state_since_ms;
    uint32_t tilted_since_ms;
    uint8_t reference_samples;
    uint8_t next_direction_sample;
    uint8_t low_g_samples;
    bool have_sample;
    bool up_valid;
    bool direction_history_valid;
    bool onset_up_valid;
    bool lift_candidate;
    bool lift_active;
    bool low_g_after_lift;
    bool low_g_reported;
    bool slip_is_downward;
    bool tilted_still;
} CaneMotion_t;

typedef struct
{
    CaneMotion_Event_t event;
    CaneMotion_State_t state;
    bool pending;
    bool up_valid;
    float vertical_g;      /* Median-filtered world-up acceleration, gravity removed. */
    float horizontal_g;    /* Median-filtered acceleration magnitude in the horizontal plane. */
    float vertical_raw_g;  /* Current unfiltered world-up projection, gravity removed. */
    float horizontal_raw_g;
    float accel_g;      /* Raw three-axis magnitude, including gravity. */
    float rotation_dps;
} CaneMotion_Result_t;

/* Zero-initialize the detector and use a fixed policy until it is reset.
 * A NULL sample or timestamp gap clears motion evidence. DROP and FALL are alarm events. */
CaneMotion_Result_t CaneMotion_Step(CaneMotion_t * const p_motion,
                                  const Imu_Sample_t * const p_sample, Grip_State_t grip,
                                  CaneMotion_AlarmPolicy_t policy);

#endif /* CANEMOTION_H */
