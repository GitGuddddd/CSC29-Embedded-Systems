#ifndef RTE_H
#define RTE_H

#include "Rte_Type.h"

/*
 * \file Rte.h
 * \brief Runtime Environment API interface
 * Application components interact with sensors/actuators solely through these functions.
 * Compliance: Barr-C:2018
 */

/* Sensors */
Rte_Status_t Rte_Read_ImuSample(Imu_Sample_t * const p_sample);
Rte_Status_t Rte_Read_GripState(bool * const p_is_held);
Rte_Status_t Rte_Read_WirelessGripTelemetry(Grip_State_t * const p_grip_telemetry);
Rte_Status_t Rte_Read_TofDistance(uint16_t * const p_dist_mm);
Rte_Status_t Rte_Read_RadarTargets(Radar_Target_t * const p_targets, uint8_t max_count);

/* Actuators / Output Alerts */
Rte_Status_t Rte_Write_HapticPattern(uint8_t pattern_id);
Rte_Status_t Rte_Write_BuzzerTone(uint16_t freq_hz, uint16_t duration_ms);
Rte_Status_t Rte_Write_Alert(Alert_Type_t alert_type, uint16_t param);

/* Telemetry & Wireless */
Rte_Status_t Rte_Send_BleThreatPacket(const Ble_AdvancedThreatPacket_t * const p_packet);
Rte_Status_t Rte_Receive_BleThreatPacket(Ble_AdvancedThreatPacket_t * const p_packet);

/* System Timing */
uint32_t Rte_Get_TimeMs(void);

#endif /* RTE_H */
