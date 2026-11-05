#ifndef SAFETYDELIVERY_H
#define SAFETYDELIVERY_H

#include <stdbool.h>
#include <stdint.h>
#include "Rte_Type.h"

/*
 * \file SafetyDelivery.h
 * \brief Sender-side critical-event delivery supervision interface
 * Compliance: Barr-C:2018
 */

/* Sender-side supervision of one locally originated critical event. */
typedef struct
{
    uint16_t local_session_id;
    uint16_t peer_session_id;
    uint16_t receipt_peer_session_id;
    uint32_t event_started_ms;
    uint32_t last_peer_ms;
    uint8_t event_seq : 4;
    uint8_t active : 1;
    uint8_t peer_seen : 1;
    uint8_t receipt_confirmed : 1;
    uint8_t reserved : 1;
} SafetyDelivery_t;

void SafetyDelivery_Init(SafetyDelivery_t * const p_delivery, uint16_t local_session_id);
void SafetyDelivery_Start(SafetyDelivery_t * const p_delivery, uint16_t local_session_id,
                          uint8_t event_seq, uint32_t now_ms);
void SafetyDelivery_Clear(SafetyDelivery_t * const p_delivery);
void SafetyDelivery_ObservePeer(SafetyDelivery_t * const p_delivery,
                                const Ble_AdvancedThreatPacket_t * const p_packet,
                                uint32_t received_ms);
bool SafetyDelivery_IsConfirmed(const SafetyDelivery_t * const p_delivery);
bool SafetyDelivery_NeedsFallback(const SafetyDelivery_t * const p_delivery,
                                  uint32_t now_ms);

#endif /* SAFETYDELIVERY_H */
