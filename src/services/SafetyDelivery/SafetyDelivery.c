#include "SafetyDelivery.h"
#include "SafetyDelivery_Cfg.h"
#include <stddef.h>
#include <string.h>

/*
 * \file SafetyDelivery.c
 * \brief Boot-scoped critical-event receipt and degraded-fallback supervision
 * Compliance: Barr-C:2018
 */

void SafetyDelivery_Init(SafetyDelivery_t * const p_delivery, uint32_t local_session_id)
{
    if (p_delivery != NULL)
    {
        memset(p_delivery, 0, sizeof(*p_delivery));
        p_delivery->local_session_id = local_session_id;
    }
}

void SafetyDelivery_Start(SafetyDelivery_t * const p_delivery, uint32_t event_id,
                          uint32_t now_ms)
{
    if (p_delivery != NULL)
    {
        p_delivery->event_id = event_id;
        p_delivery->event_started_ms = now_ms;
        p_delivery->active = true;
        p_delivery->receipt_confirmed = false;
        p_delivery->receipt_peer_session_id = 0U;
    }
}

void SafetyDelivery_Clear(SafetyDelivery_t * const p_delivery)
{
    if (p_delivery != NULL)
    {
        p_delivery->active = false;
        p_delivery->receipt_confirmed = false;
        p_delivery->receipt_peer_session_id = 0U;
    }
}

void SafetyDelivery_ObservePeer(SafetyDelivery_t * const p_delivery,
                                const Ble_AdvancedThreatPacket_t * const p_packet,
                                uint32_t received_ms)
{
    if ((p_delivery == NULL) || (p_packet == NULL))
    {
        return;
    }

    if (p_delivery->peer_seen &&
        (p_packet->session_id != p_delivery->peer_session_id))
    {
        /* A peer reboot invalidates a receipt issued by its previous application. */
        p_delivery->receipt_confirmed = false;
        p_delivery->receipt_peer_session_id = 0U;
    }
    p_delivery->peer_seen = true;
    p_delivery->last_peer_ms = received_ms;
    p_delivery->peer_session_id = p_packet->session_id;

    if (p_delivery->active &&
        (p_packet->protocol_version == BLE_PROTOCOL_VERSION) &&
        (p_packet->session_id != 0U) && p_packet->peer_ack_valid &&
        (p_packet->peer_ack_session_id == p_delivery->local_session_id) &&
        (p_packet->peer_ack_event_id == p_delivery->event_id))
    {
        p_delivery->receipt_confirmed = true;
        p_delivery->receipt_peer_session_id = p_packet->session_id;
    }
}

bool SafetyDelivery_IsConfirmed(const SafetyDelivery_t * const p_delivery)
{
    return (p_delivery != NULL) && p_delivery->active && p_delivery->receipt_confirmed &&
           (p_delivery->receipt_peer_session_id != 0U) &&
           (p_delivery->receipt_peer_session_id == p_delivery->peer_session_id);
}

bool SafetyDelivery_NeedsFallback(const SafetyDelivery_t * const p_delivery,
                                  uint32_t now_ms)
{
    if ((p_delivery == NULL) || !p_delivery->active)
    {
        return false;
    }
    if (SafetyDelivery_IsConfirmed(p_delivery))
    {
        return !p_delivery->peer_seen ||
               ((uint32_t)(now_ms - p_delivery->last_peer_ms) >=
                SAFETYDELIVERY_CFG_PEER_TIMEOUT_MS);
    }
    return ((uint32_t)(now_ms - p_delivery->event_started_ms) >=
            SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS);
}
