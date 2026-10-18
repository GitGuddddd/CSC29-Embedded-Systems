#include "FallDetection.h"
#include "FallDetection_Cfg.h"
#include <stddef.h>

/*
 * \file FallDetection.c
 * \brief Pure algorithmic fall detection state machine
 * Decoupled from hardware; only receives Imu_Sample_t
 * Compliance: Barr-C:2018
 */

static const char * s_state_names[] = {
    "IDLE",
    "FREEFALL",
    "WAIT_IMPACT",
    "SETTLING",
    "CHECK_STILL"
};

static Fall_MachineState_t s_state = FALL_STATE_IDLE;
static Fall_Config_t s_active_cfg;
static uint32_t s_t0_ms = 0U;

static float s_ref_x = 0.0f;
static float s_ref_y = 0.0f;
static float s_ref_z = 1.0f;

static float s_cur_x = 0.0f;
static float s_cur_y = 0.0f;
static float s_cur_z = 0.0f;

extern float sqrtf(float);

static float Vector_MagSq(float x, float y, float z)
{
    return (x * x) + (y * y) + (z * z);
}

static float Vector_CosineSimilarity(float ax, float ay, float az, float bx, float by, float bz)
{
    float dot = (ax * bx) + (ay * by) + (az * bz);
    float mag_a_sq = Vector_MagSq(ax, ay, az);
    float mag_b_sq = Vector_MagSq(bx, by, bz);

    if ((mag_a_sq < 1e-6f) || (mag_b_sq < 1e-6f))
    {
        return 0.0f;
    }

    return dot / sqrtf(mag_a_sq * mag_b_sq);
}

void FallDetection_Init(const Fall_Config_t * const p_cfg)
{
    s_state = FALL_STATE_IDLE;
    s_t0_ms = 0U;
    s_ref_x = 0.0f;
    s_ref_y = 0.0f;
    s_ref_z = 1.0f;

    if (p_cfg != NULL)
    {
        s_active_cfg = *p_cfg;
    }
    else
    {
        s_active_cfg = FallConfig_Head;
    }
}

Fall_MachineState_t FallDetection_GetState(void)
{
    return s_state;
}

const char * FallDetection_GetStateName(void)
{
    return s_state_names[s_state];
}

bool FallDetection_Step(const Imu_Sample_t * const p_sample, uint32_t now_ms)
{
    if (p_sample == NULL)
    {
        return false;
    }

    float a_sq = Vector_MagSq(p_sample->ax, p_sample->ay, p_sample->az);
    float g_sq = Vector_MagSq(p_sample->gx, p_sample->gy, p_sample->gz);
    bool fall_confirmed = false;

    switch (s_state)
    {
        case FALL_STATE_IDLE:
        {
            s_ref_x += 0.01f * (p_sample->ax - s_ref_x);
            s_ref_y += 0.01f * (p_sample->ay - s_ref_y);
            s_ref_z += 0.01f * (p_sample->az - s_ref_z);

            if (a_sq < s_active_cfg.freefall_thresh_sq)
            {
                s_state = FALL_STATE_FREEFALL;
                s_t0_ms = now_ms;
            }
            else if (a_sq > s_active_cfg.impact_thresh_sq)
            {
                s_state = FALL_STATE_SETTLING;
                s_t0_ms = now_ms;
            }
            break;
        }

        case FALL_STATE_FREEFALL:
        {
            if (a_sq >= s_active_cfg.freefall_thresh_sq)
            {
                if ((now_ms - s_t0_ms) < s_active_cfg.freefall_min_ms)
                {
                    s_state = FALL_STATE_IDLE;
                }
                else if (a_sq > s_active_cfg.impact_thresh_sq)
                {
                    s_state = FALL_STATE_SETTLING;
                    s_t0_ms = now_ms;
                }
                else
                {
                    s_state = FALL_STATE_WAIT_IMPACT;
                    s_t0_ms = now_ms;
                }
            }
            break;
        }

        case FALL_STATE_WAIT_IMPACT:
        {
            if (a_sq > s_active_cfg.impact_thresh_sq)
            {
                s_state = FALL_STATE_SETTLING;
                s_t0_ms = now_ms;
            }
            else if ((now_ms - s_t0_ms) > s_active_cfg.impact_window_ms)
            {
                s_state = FALL_STATE_IDLE;
            }
            break;
        }

        case FALL_STATE_SETTLING:
        {
            if ((now_ms - s_t0_ms) >= s_active_cfg.settle_ms)
            {
                s_state = FALL_STATE_CHECK_STILL;
                s_t0_ms = now_ms;
                s_cur_x = p_sample->ax;
                s_cur_y = p_sample->ay;
                s_cur_z = p_sample->az;
            }
            break;
        }

        case FALL_STATE_CHECK_STILL:
        {
            s_cur_x += 0.1f * (p_sample->ax - s_cur_x);
            s_cur_y += 0.1f * (p_sample->ay - s_cur_y);
            s_cur_z += 0.1f * (p_sample->az - s_cur_z);

            if ((a_sq < FALL_STILL_ACC_MIN_G_SQ) ||
                (a_sq > FALL_STILL_ACC_MAX_G_SQ) ||
                (g_sq > FALL_STILL_GYRO_DPS_SQ))
            {
                s_state = FALL_STATE_IDLE;
            }
            else if ((now_ms - s_t0_ms) >= s_active_cfg.still_ms)
            {
                s_state = FALL_STATE_IDLE;
                float cos_sim = Vector_CosineSimilarity(s_ref_x, s_ref_y, s_ref_z,
                                                        s_cur_x, s_cur_y, s_cur_z);
                if (cos_sim < FALL_TILT_COS_THRESH)
                {
                    fall_confirmed = true;
                }
            }
            break;
        }

        default:
        {
            s_state = FALL_STATE_IDLE;
            break;
        }
    }

    return fall_confirmed;
}
