#ifndef BLECOMM_H
#define BLECOMM_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"
#include "BleComm_Policy.h"

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
    uint8_t pair_id;
} BleComm_Config_t;

extern const BleComm_Config_t BleComm_Config_Cane;
extern const BleComm_Config_t BleComm_Config_Glasses;

typedef struct
{
    bool initialized;
    bool active;
    uint8_t fall_seq;
} BleComm_PeerFall_t;

/* Keep this tracker across transient link loss: reconnecting is not a new fall. */
bool BleComm_TrackPeerFall(BleComm_PeerFall_t * const p_tracker,
                          const Ble_AdvancedThreatPacket_t * const p_packet);

void BleComm_Init(const BleComm_Config_t * const p_cfg);
void BleComm_Encode(const Ble_AdvancedThreatPacket_t * const p_in, Ble_RawPacket_t * const p_out);
bool BleComm_Decode(const Ble_RawPacket_t * const p_in, uint8_t expected_pair_id, Ble_AdvancedThreatPacket_t * const p_out);
bool BleComm_CheckNewFall(uint8_t rx_fall_seq, uint8_t last_fall_seq);

bool BleComm_SendPacket(const Ble_AdvancedThreatPacket_t * const p_pkt);
/* Enqueue payload and state together. False means retry; never wait for queue space. */
bool BleComm_Publish(const Ble_AdvancedThreatPacket_t * const p_pkt, Sys_State_t state, bool link_lost);
bool BleComm_ReceivePacketAt(const Ble_AdvancedThreatPacket_t * const p_pkt, uint32_t now_ms);
bool BleComm_PopReceivedPacket(Ble_AdvancedThreatPacket_t * const p_pkt, uint32_t * const p_received_ms);
#ifndef PICO_ON_DEVICE
bool BleComm_ReceivePacket(const Ble_AdvancedThreatPacket_t * const p_pkt);
void BleComm_InjectRxPacket(const Ble_AdvancedThreatPacket_t * const p_pkt);
#endif
/* True only when the BTstack worker has progressed while its controller is ready. */
bool BleComm_Core1_Task(void);

#endif /* BLECOMM_H */
