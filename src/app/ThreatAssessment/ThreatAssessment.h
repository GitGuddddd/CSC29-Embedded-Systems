#ifndef THREATASSESSMENT_H
#define THREATASSESSMENT_H

#include "Rte_Type.h"

/*
 * \file ThreatAssessment.h
 * \brief Glasses Threat Decision Component (FR7, FR8, FR13)
 * Compliance: Barr-C:2018
 */

typedef struct
{
    bool is_threat_active;
    uint16_t nearest_distance_mm;
    int16_t closing_speed_cms;
    uint16_t time_to_collision_ms;
} Threat_Result_t;

void ThreatAssessment_Init(void);
void ThreatAssessment_ProcessTargets(const Radar_Target_t * const p_targets,
                                     uint8_t target_count,
                                     Grip_State_t cane_grip_telemetry,
                                     Threat_Result_t * const p_out_result);

#endif /* THREATASSESSMENT_H */
