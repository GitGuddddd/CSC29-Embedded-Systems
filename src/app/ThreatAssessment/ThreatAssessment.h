#ifndef THREATASSESSMENT_H
#define THREATASSESSMENT_H

#include "Rte_Type.h"

/*
 * \file ThreatAssessment.h
 * \brief Glasses Threat Decision Component (FR7, FR8, FR13)
 * Compliance: Barr-C:2018
 */

typedef enum
{
    THREAT_LANE_CENTER = 0U,
    THREAT_LANE_LEFT   = 1U,
    THREAT_LANE_RIGHT  = 2U,
    THREAT_LANE_COUNT  = 3U
} Threat_Lane_t;

#define THREAT_CUE_STOP        (0x01U)
#define THREAT_CUE_SIDE_SHIFT  (1U)
/* Side field value 3: no confirmed-free side lane (both blocked, or only just cleared). 0/1/2 are the Threat_Lane_t values. */
#define THREAT_CUE_SIDE_ALL_BLOCKED (3U)

typedef struct
{
    uint16_t nearest_distance_mm;
    int16_t closing_speed_cms;
    uint16_t ttc_20ms : 12;       /* 0-4094 ticks; 0xFFF unavailable/out of range */
    uint16_t is_threat_active : 1;
    uint16_t open_direction_cue : 3;
} Threat_Result_t;

/* True while every blocked lane's obstacle has been moving away for
 * THREAT_RECEDE_FRAMES frames (during STOP only beyond
 * THREAT_STOP_RECEDE_MIN_MM). Used to keep the buzzer quiet for departing
 * obstacles; the threat result, lanes and the STOP cue bit are unaffected. */
bool ThreatAssessment_IsReceding(void);

/* Debug: bit n set = lane n (THREAT_LANE_CENTER/LEFT/RIGHT) is currently blocked. */
uint8_t ThreatAssessment_GetBlockedMask(void);

void ThreatAssessment_Init(void);
void ThreatAssessment_ProcessTargets(const Radar_Target_t * const p_targets,
                                     uint8_t target_count,
                                     Grip_State_t cane_grip_telemetry,
                                     Threat_Result_t * const p_out_result);
void ThreatAssessment_ProcessTargetsAt(const Radar_Target_t * const p_targets,
                                       uint8_t target_count,
                                       Grip_State_t cane_grip_telemetry,
                                       uint32_t now_ms,
                                       Threat_Result_t * const p_out_result);

#endif /* THREATASSESSMENT_H */
