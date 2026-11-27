#ifndef BLECOMM_POLICY_H
#define BLECOMM_POLICY_H

#include "Rte_Type.h"
#include "drivers/BleRadio/BleRadio.h"

/* Ordered by urgency. States may intentionally share a physical radio profile. */
typedef enum
{
    BLECOMM_PROFILE_QUIET = 0U,
    BLECOMM_PROFILE_ACTIVE,
    BLECOMM_PROFILE_FAST
} BleComm_ProfileId_t;

typedef struct
{
    BleRadio_Settings_t radio;
    uint16_t payload_period_ms;
} BleComm_Profile_t;

typedef struct
{
    bool initialized;
    BleComm_ProfileId_t effective;
    uint32_t last_urgent_ms;
} BleComm_Policy_t;

BleComm_ProfileId_t BleComm_SelectProfile(Sys_State_t state, uint8_t flags, bool link_lost);
const BleComm_Profile_t * BleComm_GetProfile(BleComm_ProfileId_t profile);
void BleComm_PolicyInit(BleComm_Policy_t * const p_policy);
BleComm_ProfileId_t BleComm_PolicyStep(BleComm_Policy_t * const p_policy,
                                      BleComm_ProfileId_t desired, uint32_t now_ms);

#endif /* BLECOMM_POLICY_H */
