#include "Rte.h"
#include <string.h>

/*
 * \file Rte.c
 * \brief Runtime Environment router / mockable dispatcher
 * Maps high-level RTE calls to driver interfaces or RTOS IPC queues.
 * Compliance: Barr-C:2018
 */

static Imu_Sample_t s_last_imu_sample;
static bool s_last_grip_state = false;
static Grip_State_t s_last_wireless_grip = GRIP_STATE_UNKNOWN;
static uint16_t s_last_tof_dist = 0U;
static uint32_t s_mock_time_ms = 0U;

Rte_Status_t Rte_Read_ImuSample(Imu_Sample_t * const p_sample)
{
    if (p_sample == NULL)
    {
        return RTE_STATUS_ERROR;
    }

    *p_sample = s_last_imu_sample;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Read_GripState(bool * const p_is_held)
{
    if (p_is_held == NULL)
    {
        return RTE_STATUS_ERROR;
    }

    *p_is_held = s_last_grip_state;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Read_WirelessGripTelemetry(Grip_State_t * const p_grip_telemetry)
{
    if (p_grip_telemetry == NULL)
    {
        return RTE_STATUS_ERROR;
    }

    *p_grip_telemetry = s_last_wireless_grip;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Read_TofDistance(uint16_t * const p_dist_mm)
{
    if (p_dist_mm == NULL)
    {
        return RTE_STATUS_ERROR;
    }

    *p_dist_mm = s_last_tof_dist;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Read_RadarTargets(Radar_Target_t * const p_targets, uint8_t max_count)
{
    if ((p_targets == NULL) || (max_count == 0U))
    {
        return RTE_STATUS_ERROR;
    }

    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Write_HapticPattern(uint8_t pattern_id)
{
    (void)pattern_id;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Write_BuzzerTone(uint16_t freq_hz, uint16_t duration_ms)
{
    (void)freq_hz;
    (void)duration_ms;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Write_Alert(Alert_Type_t alert_type, uint16_t param)
{
    (void)alert_type;
    (void)param;
    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Send_BleThreatPacket(const Ble_AdvancedThreatPacket_t * const p_packet)
{
    if (p_packet == NULL)
    {
        return RTE_STATUS_ERROR;
    }

    return RTE_STATUS_OK;
}

Rte_Status_t Rte_Receive_BleThreatPacket(Ble_AdvancedThreatPacket_t * const p_packet)
{
    if (p_packet == NULL)
    {
        return RTE_STATUS_ERROR;
    }

    return RTE_STATUS_OK;
}

uint32_t Rte_Get_TimeMs(void)
{
    return s_mock_time_ms;
}

/* Test/Driver hooks for injecting RTE data */
void Rte_TestHook_SetImu(const Imu_Sample_t * const p_s)
{
    if (p_s != NULL)
    {
        s_last_imu_sample = *p_s;
    }
}

void Rte_TestHook_SetTimeMs(uint32_t time_ms)
{
    s_mock_time_ms = time_ms;
}

void Rte_TestHook_SetGrip(bool is_held)
{
    s_last_grip_state = is_held;
}

void Rte_TestHook_SetWirelessGrip(Grip_State_t grip_telemetry)
{
    s_last_wireless_grip = grip_telemetry;
}

void Rte_TestHook_SetTof(uint16_t dist_mm)
{
    s_last_tof_dist = dist_mm;
}
