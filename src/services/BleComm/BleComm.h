#ifndef BLECOMM_H
#define BLECOMM_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file BleComm.h
 * \brief Connectionless BLE broadcaster/observer communication service (FR14, FR15, NFR13)
 * Compliance: Barr-C:2018
 */

typedef enum
{
    BLE_ROLE_OBSERVER_CANE = 0U,
    BLE_ROLE_BROADCASTER_GLASSES
} Ble_Role_t;

typedef struct
{
    Ble_Role_t role;
    uint8_t device_id;
} BleComm_Config_t;

extern const BleComm_Config_t BleComm_Config_Cane;
extern const BleComm_Config_t BleComm_Config_Glasses;

void BleComm_Init(const BleComm_Config_t * const p_cfg);
bool BleComm_SendPacket(const Ble_AdvancedThreatPacket_t * const p_pkt);
bool BleComm_ReceivePacket(const Ble_AdvancedThreatPacket_t * const p_pkt);
bool BleComm_PopReceivedPacket(Ble_AdvancedThreatPacket_t * const p_pkt);
void BleComm_InjectRxPacket(const Ble_AdvancedThreatPacket_t * const p_pkt);

#endif /* BLECOMM_H */
