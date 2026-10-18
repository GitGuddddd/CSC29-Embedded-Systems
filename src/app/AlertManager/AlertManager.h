#ifndef ALERTMANAGER_H
#define ALERTMANAGER_H

#include "Rte_Type.h"

/*
 * \file AlertManager.h
 * \brief Audio/Haptic Arbitrator interface
 * Ensures Fall Alarm > Cliff Alert > Obstacle Proximity > Fault Tones
 * Compliance: Barr-C:2018
 */

typedef struct
{
    Alert_Type_t active_alert;
    uint16_t generated_freq_hz;
    bool buzzer_active;
    bool haptic_active;
    bool fall_latched;
} Alert_Output_t;

void AlertManager_Init(void);
uint16_t AlertManager_MapDistanceToPitch(uint16_t distance_mm);
void AlertManager_AcknowledgeReset(void);
void AlertManager_Arbitrate(Alert_Type_t requested_alert,
                            uint16_t param,
                            uint32_t now_ms,
                            Alert_Output_t * const p_out);

#endif /* ALERTMANAGER_H */
