#ifndef CANEMODE_H
#define CANEMODE_H

#include "Rte_Type.h"

/* Pure mode manager. Power, the incident latch and technical faults are independent.
 * Only the perception task owns this context; hardware actions follow its outputs.
 */
typedef enum
{
    CANE_POWER_STARTING = 0U,
    CANE_POWER_MONITORING,
    CANE_POWER_STANDBY
} CaneMode_Power_t;

typedef enum
{
    CANE_ALARM_CLEAR = 0U,
    CANE_ALARM_LATCHED,
    CANE_ALARM_PAUSED
} CaneMode_Alarm_t;

typedef struct
{
    bool running;
    uint32_t since_ms;
} CaneMode_Timer_t;

typedef struct
{
    Grip_State_t grip;
    bool imu_ready;
    bool imu_sleeping;
    bool imu_fault;
    bool sample_valid;
    Imu_Sample_t sample;
    bool local_fall;
    bool peer_fall; /* One event per peer incident, not its repeated latched flag. */
    bool fall_pending;
    bool ground_hazard;
    bool technical_fault;
} CaneMode_Input_t;

typedef struct
{
    Sys_State_t state;
    bool imu_sleep_requested;
    bool fall_detection_enabled;
    bool ground_enabled;
    bool reset_fall_detection;
    bool fall_latched;
    bool alarm_paused;
    bool degraded;
    uint8_t status_flags;
} CaneMode_Output_t;

typedef struct
{
    CaneMode_Power_t power;
    CaneMode_Alarm_t alarm;
    Grip_State_t previous_grip;
    bool acknowledge_armed;
    bool override_armed;
    bool override_holding;
    bool park_requested;
    bool park_override;
    bool parked_by_user;
    bool wake_armed;
    uint32_t pause_start_ms;
    uint32_t last_step_ms;
    CaneMode_Timer_t park;
    CaneMode_Timer_t recovery;
    CaneMode_Timer_t release;
    CaneMode_Timer_t hold;
    CaneMode_Timer_t wake;
} CaneMode_t;

void CaneMode_Init(CaneMode_t * const p_mode, uint32_t now_ms);
/* Restored incidents resume audible and require normal guarded recovery. */
void CaneMode_RestoreIncident(CaneMode_t * const p_mode);
void CaneMode_Step(CaneMode_t * const p_mode, const CaneMode_Input_t * const p_in,
                   uint32_t now_ms, CaneMode_Output_t * const p_out);
const char * CaneMode_GetStateName(Sys_State_t state);

#endif /* CANEMODE_H */
