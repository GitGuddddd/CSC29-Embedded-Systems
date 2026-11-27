#include "BleComm.h"

/*
 * \file BleComm_Lcfg.c
 * \brief Link-time configuration for Cane vs Glasses BLE communication service
 * Compliance: Barr-C:2018
 */

const BleComm_Config_t BleComm_Config_Cane = {
    .role      = BLE_ROLE_OBSERVER_CANE,
    .device_id = 0x01U,
    .pair_id   = BLE_DEFAULT_PAIR_ID
};

const BleComm_Config_t BleComm_Config_Glasses = {
    .role      = BLE_ROLE_BROADCASTER_GLASSES,
    .device_id = 0x02U,
    .pair_id   = BLE_DEFAULT_PAIR_ID
};
