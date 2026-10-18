#ifndef FALLDETECTION_H
#define FALLDETECTION_H

#include "Rte_Type.h"

/*
 * \file FallDetection.h
 * \brief Application SWC interface for Fall Detection
 * Compliance: Barr-C:2018
 */

typedef enum
{
    FALL_STATE_IDLE = 0U,
    FALL_STATE_FREEFALL,
    FALL_STATE_WAIT_IMPACT,
    FALL_STATE_SETTLING,
    FALL_STATE_CHECK_STILL
} Fall_MachineState_t;

typedef struct
{
    float freefall_thresh_sq;
    float impact_thresh_sq;
    uint32_t freefall_min_ms;
    uint32_t impact_window_ms;
    uint32_t settle_ms;
    uint32_t still_ms;
} Fall_Config_t;

/* Global Link-time configurations */
extern const Fall_Config_t FallConfig_Head;
extern const Fall_Config_t FallConfig_Cane;

void FallDetection_Init(const Fall_Config_t * const p_cfg);
Fall_MachineState_t FallDetection_GetState(void);
const char * FallDetection_GetStateName(void);
bool FallDetection_Step(const Imu_Sample_t * const p_sample, uint32_t now_ms);

#endif /* FALLDETECTION_H */
