#ifndef GROUNDPROJECTION_H
#define GROUNDPROJECTION_H

#include <stdint.h>
#include <stdbool.h>

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

    uint32_t last_sample_ms;
} Gp_Detector_t;

void GroundProjection_Init(Gp_Detector_t * const p_d, uint32_t now_ms);
void GroundProjection_SetArmed(Gp_Detector_t * const p_d, bool armed, uint32_t now_ms);
void GroundProjection_StartCalibration(Gp_Detector_t * const p_d, uint32_t now_ms);
void GroundProjection_Feed(Gp_Detector_t * const p_d, Gp_Quality_t quality, uint16_t range_mm, uint32_t now_ms);
void GroundProjection_Tick(Gp_Detector_t * const p_d, uint32_t now_ms);
const char * GroundProjection_GetStateName(Gp_State_t state);

#endif /* GROUNDPROJECTION_H */
