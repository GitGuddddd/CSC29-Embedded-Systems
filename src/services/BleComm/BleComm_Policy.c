#include "BleComm_Policy.h"
#include "BleComm_Cfg.h"
#include "services/Watchdog/Watchdog_Cfg.h"
#include <stddef.h>

#define BLE_UNITS_FROM_MS(ms) (((ms) * 8U) / 5U)

/* Budget two advertising opportunities, including controller delay and task margin.
 * This is a scheduling budget, not a guarantee of reception in an RF environment.
 */
_Static_assert(((2U * (BLECOMM_CFG_ADV_QUIET_MS + BLECOMM_CFG_ADV_RANDOM_MS)) +
                BLECOMM_CFG_DELIVERY_MARGIN_MS) < WATCHDOG_CFG_DEADMAN_TIMEOUT_MS,
               "Advertising cadence leaves no BLE deadman margin");
_Static_assert(BLECOMM_CFG_PAYLOAD_QUIET_MS < BLECOMM_CFG_ADV_QUIET_MS,
               "Refresh the heartbeat faster than quiet advertising");
_Static_assert(BLECOMM_CFG_PAYLOAD_FAST_MS < BLECOMM_CFG_ADV_FAST_MS,
               "Refresh the heartbeat faster than urgent advertising");

static const BleComm_Profile_t s_profiles[] = {
    {
        .radio = { BLE_UNITS_FROM_MS(BLECOMM_CFG_ADV_QUIET_MS),
                   BLE_UNITS_FROM_MS(BLECOMM_CFG_SCAN_INTERVAL_MS),
                   BLE_UNITS_FROM_MS(BLECOMM_CFG_SCAN_NORMAL_MS) },
        .payload_period_ms = BLECOMM_CFG_PAYLOAD_QUIET_MS
    },
    {
        .radio = { BLE_UNITS_FROM_MS(BLECOMM_CFG_ADV_ACTIVE_MS),
                   BLE_UNITS_FROM_MS(BLECOMM_CFG_SCAN_INTERVAL_MS),
                   BLE_UNITS_FROM_MS(BLECOMM_CFG_SCAN_NORMAL_MS) },
        .payload_period_ms = BLECOMM_CFG_PAYLOAD_ACTIVE_MS
    },
    {
        .radio = { BLE_UNITS_FROM_MS(BLECOMM_CFG_ADV_FAST_MS),
                   BLE_UNITS_FROM_MS(BLECOMM_CFG_SCAN_INTERVAL_MS),
                   BLE_UNITS_FROM_MS(BLECOMM_CFG_SCAN_URGENT_MS) },
        .payload_period_ms = BLECOMM_CFG_PAYLOAD_FAST_MS
    }
};

BleComm_ProfileId_t BleComm_SelectProfile(Sys_State_t state, uint8_t flags, bool link_lost)
{
    if (link_lost || ((flags & (BLE_FLAG_FALL_ALERT | BLE_FLAG_ALARM_PAUSED |
                               BLE_FLAG_SENSOR_FAULT | BLE_FLAG_OBSTACLE_ACTIVE)) != 0U))
    {
        return BLECOMM_PROFILE_FAST;
    }

    /* Enumerate every mode, including currently-unused IDLE and HARD_FAULT.
     * -Wswitch-enum makes a newly added mode require an explicit decision here.
     */
    switch (state)
    {
        case SYS_STATE_IDLE:
        case SYS_STATE_STANDBY:
            return BLECOMM_PROFILE_QUIET;
        case SYS_STATE_ACTIVE:
            return BLECOMM_PROFILE_ACTIVE;
        case SYS_STATE_INIT:
        case SYS_STATE_WATCH:
        case SYS_STATE_UNGRIPPED_WATCH:
        case SYS_STATE_FALL_CONFIRMED:
        case SYS_STATE_FALL_MUTED:
        case SYS_STATE_HARD_FAULT:
        case SYS_STATE_FAULT_DEGRADED:
        case SYS_STATE_COUNT:
        default:
            return BLECOMM_PROFILE_FAST;
    }
}

const BleComm_Profile_t * BleComm_GetProfile(BleComm_ProfileId_t profile)
{
    if ((uint32_t)profile > (uint32_t)BLECOMM_PROFILE_FAST)
    {
        profile = BLECOMM_PROFILE_FAST;
    }
    return &s_profiles[(uint32_t)profile];
}

void BleComm_PolicyInit(BleComm_Policy_t * const p_policy)
{
    if (p_policy != NULL)
    {
        p_policy->initialized = false;
        p_policy->effective = BLECOMM_PROFILE_FAST;
        p_policy->last_urgent_ms = 0U;
    }
}

BleComm_ProfileId_t BleComm_PolicyStep(BleComm_Policy_t * const p_policy,
                                      BleComm_ProfileId_t desired, uint32_t now_ms)
{
    if (p_policy == NULL)
    {
        return BLECOMM_PROFILE_FAST;
    }
    if ((uint32_t)desired > (uint32_t)BLECOMM_PROFILE_FAST)
    {
        desired = BLECOMM_PROFILE_FAST;
    }
    if (!p_policy->initialized || (desired >= p_policy->effective))
    {
        p_policy->effective = desired;
        p_policy->last_urgent_ms = now_ms;
        p_policy->initialized = true;
    }
    else if ((uint32_t)(now_ms - p_policy->last_urgent_ms) >= BLECOMM_CFG_DOWNSHIFT_HOLD_MS)
    {
        p_policy->effective = desired;
        p_policy->last_urgent_ms = now_ms;
    }
    else
    {
        /* Defer downshifts, never an escalation; avoid repeated HCI stop/start chatter. */
    }
    return p_policy->effective;
}
