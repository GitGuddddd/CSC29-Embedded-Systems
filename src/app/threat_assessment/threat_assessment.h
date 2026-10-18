/** @file threat_assessment.h
 *
 * @brief Glasses obstacle decision: nearest target, hysteresis, TTC, pitch.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Reference: Group 29 Week 6 design review, statechart 6.6 (bottom).
 *   FR8   nearest valid non-departing target within 2000 mm is an obstacle;
 *         the alert starts below 2000 mm and stops above 2100 mm (NFR8).
 *   FR9   while active, OBSTACLE(d_mm) goes to Audio Alert on every frame.
 *   FR10  OBSTACLE(CLEAR) when the target is beyond 2100 mm, none was seen
 *         for 500 ms, the range trend showed the target leaving for 2 frames,
 *         or the radar is in RADAR_FAULT.
 *   A target is departing when its sensor speed or its range trend over
 *   3 frames is above +10 cm/s.
 *   FR13  closing speed and TTC = distance / closing speed go to BLE.
 */

#ifndef THREAT_ASSESSMENT_H
#define THREAT_ASSESSMENT_H

#include <stdbool.h>
#include <stdint.h>

#include "rte_type.h"

/** Number of past frames kept for the range trend (keep in step with cfg). */
#define THREAT_ASSESSMENT_HISTORY_LEN    (3u)

/** State of Glasses_Threat: CLEAR (b_active false) or OBSTACLE_ACTIVE. */
typedef struct
{
    bool     b_active;
    bool     b_seen;
    uint32_t last_seen_ms;
    uint32_t last_d_mm;
    uint16_t last_ttc_ms;
    uint32_t hist_d_mm[THREAT_ASSESSMENT_HISTORY_LEN];  /**< [0] = newest. */
    uint32_t hist_ms[THREAT_ASSESSMENT_HISTORY_LEN];
    uint8_t  hist_count;
    uint8_t  depart_frames;  /**< Frames in a row with only departing. */
} threat_assessment_t;

/** What the caller must send after one threat_assessment_update(). */
typedef struct
{
    bool     b_post_obstacle;  /**< Send OBSTACLE(d_mm) (every frame). */
    bool     b_post_clear;     /**< Send OBSTACLE(CLEAR) (once per exit). */
    bool     b_active;         /**< State after this update. */
    uint16_t d_mm;             /**< Distance for OBSTACLE(d_mm). */
    uint16_t ttc_ms;           /**< RTE_TTC_NONE_MS if not approaching. */
} threat_assessment_out_t;

/** Link-time configuration (defined in threat_assessment_lcfg.c). */
typedef struct
{
    /** Assumption A8: negative speed means approaching the sensor. */
    bool b_negative_speed_is_approaching;
} threat_assessment_lcfg_t;

extern const threat_assessment_lcfg_t g_threat_assessment_lcfg;

void threat_assessment_init (threat_assessment_t * p_state);
bool threat_assessment_reset (threat_assessment_t * p_state);
void threat_assessment_update (threat_assessment_t * p_state,
                               const rte_radar_target_t * p_targets,
                               uint32_t now_ms,
                               threat_assessment_out_t * p_out);

void threat_assessment_process_frame (threat_assessment_t * p_state,
                                      const rte_radar_target_t * p_targets,
                                      uint32_t now_ms);
void threat_assessment_process_fault (threat_assessment_t * p_state);

uint16_t threat_assessment_distance_mm (const rte_radar_target_t * p_target);
int32_t threat_assessment_closing_cms (const rte_radar_target_t * p_target);
bool threat_assessment_is_departing (const rte_radar_target_t * p_target);
uint16_t threat_assessment_ttc_ms (uint32_t d_mm, int32_t closing_cms);
uint16_t threat_assessment_pitch_hz (uint32_t d_mm);

#endif /* THREAT_ASSESSMENT_H */

/*** end of file ***/
