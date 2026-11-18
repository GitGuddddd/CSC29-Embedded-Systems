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
    uint8_t generated_duty_percent;
    bool buzzer_active;
    bool haptic_active;
    bool fall_latched;
} Alert_Output_t;

void AlertManager_Init(void);
uint16_t AlertManager_MapDistanceToPitch(uint16_t distance_mm);
void AlertManager_AcknowledgeReset(void);
/* Synchronize the mode manager's incident and audio state without conflating pause/reset. */
void AlertManager_SetFallState(bool latched, bool paused, uint32_t now_ms);
void AlertManager_Arbitrate(Alert_Type_t requested_alert,
                            uint16_t param,
                            uint32_t now_ms,
                            Alert_Output_t * const p_out);

typedef struct
{
    bool is_glasses_node;
    bool local_fall_latched;
    bool local_alarm_paused;
    bool peer_fall_active;
    bool delivery_fallback;
    bool cane_link_fresh;
    bool cane_buzzer_fault;
    bool obstacle_active;
    uint16_t obstacle_dist_mm;
    bool sensor_fault;
} Alert_SyncInput_t;

void AlertManager_ArbitrateSynced(const Alert_SyncInput_t * const p_in,
                                  uint32_t now_ms,
                                  Alert_Output_t * const p_out);

#endif /* ALERTMANAGER_H */
