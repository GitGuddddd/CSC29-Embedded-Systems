#ifndef GROUNDPROJECTION_H
#define GROUNDPROJECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file GroundProjection.h
 * \brief Ground-height classifier and virtual ground-plane protection
 * Compliance: Barr-C:2018
 */

typedef enum
{
    GP_STATE_DISARMED = 0U,
    GP_STATE_CALIBRATING,
    GP_STATE_UNKNOWN,
    GP_STATE_LEVEL,
    GP_STATE_STEP_UP,
    GP_STATE_STEP_DOWN,
    GP_STATE_NO_GROUND,
    GP_STATE_SENSOR_FAULT
} Gp_State_t;

typedef enum
{
    GP_RANGE_VALID = 0U,
    GP_RANGE_NO_RETURN,
    GP_RANGE_INVALID
} Gp_Quality_t;

/* How the last calibration ended; every outcome except DONE keeps the previous baseline */
typedef enum
{
    GP_CAL_EVENT_NONE = 0U,
    GP_CAL_EVENT_DONE,
    GP_CAL_EVENT_TIMEOUT,
    GP_CAL_EVENT_SENSOR_FAULT,
    GP_CAL_EVENT_CANCELLED
} Gp_CalEvent_t;

typedef struct
{
    Gp_State_t state;
    Gp_State_t candidate;
    bool is_armed;
    bool is_calibrated;

    uint16_t baseline_mm;
    uint16_t filtered_mm;

    uint16_t history[5];
    uint8_t history_count;
    uint8_t history_next;

    uint8_t candidate_count;

    uint8_t cal_count;
    uint16_t cal_min;
    uint16_t cal_max;
    uint32_t cal_sum;
    uint32_t cal_start_ms;
    Gp_CalEvent_t cal_event;

    uint32_t last_sample_ms;

    /* Time of the first raw reading past a step threshold, for latency logging */
    bool change_pending;
    uint32_t change_start_ms;
} Gp_Detector_t;

void GroundProjection_Init(Gp_Detector_t * const p_d, uint32_t now_ms);
void GroundProjection_SetArmed(Gp_Detector_t * const p_d, bool armed, uint32_t now_ms);
void GroundProjection_StartCalibration(Gp_Detector_t * const p_d, uint32_t now_ms);
void GroundProjection_CancelCalibration(Gp_Detector_t * const p_d, uint32_t now_ms);
Gp_CalEvent_t GroundProjection_TakeCalEvent(Gp_Detector_t * const p_d);
void GroundProjection_Feed(Gp_Detector_t * const p_d, Gp_Quality_t quality, uint16_t range_mm, uint32_t now_ms);
void GroundProjection_Tick(Gp_Detector_t * const p_d, uint32_t now_ms);
Alert_Type_t GroundProjection_GetHapticRequest(const Gp_Detector_t * const p_d);
const char * GroundProjection_GetStateName(Gp_State_t state);
const char * GroundProjection_GetCalEventName(Gp_CalEvent_t event);

#endif /* GROUNDPROJECTION_H */
