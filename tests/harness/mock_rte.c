#include "rte/Rte.h"

/*
 * \file mock_rte.c
 * \brief Host testing mock stubs for RTE layer
 */

static Imu_Sample_t s_mock_imu;
static Grip_State_t s_mock_grip = GRIP_STATE_HELD;

void Mock_Rte_SetImu(const Imu_Sample_t * const p_s)
{
    if (p_s != NULL)
    {
        s_mock_imu = *p_s;
    }
}

void Mock_Rte_SetGrip(Grip_State_t grip)
{
    s_mock_grip = grip;
}
