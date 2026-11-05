#include "BleComm.h"
#include "BleComm_Cfg.h"
#include "drivers/BleRadio/BleRadio.h"
#include <stdatomic.h>
#include <string.h>

#ifdef PICO_ON_DEVICE
#include "pico/rand.h"
#endif

/*
 * \file BleComm.c
 * \brief CYW43439 Bluetooth LE Connectionless Broadcaster Implementation (Compact 6-byte Unified Packet)
 * Compliance: Barr-C:2018
 */

static BleComm_Config_t s_cfg;
static uint16_t s_last_rx_seq = 0xFFFFU;
static uint32_t s_last_rx_ms;
static uint8_t s_last_rx_flags;
static uint8_t s_last_rx_fall;
static uint32_t s_last_rx_session;
static uint32_t s_last_rx_event;
static uint32_t s_last_rx_ack_session;
static uint32_t s_last_rx_ack_event;
static bool s_last_rx_ack_valid;
static uint32_t s_session_id;

typedef struct
{
    Ble_AdvancedThreatPacket_t packet;
    BleComm_ProfileId_t profile;
} BleComm_Tx_t;

typedef struct
{
    Ble_AdvancedThreatPacket_t packet;
    uint32_t received_ms;
} BleComm_Rx_t;

/* SPSC: Core 0 -> BTstack worker for TX; BTstack worker -> Core 0 for RX.
 * Acquire/release indices publish complete snapshots. SDK atomics on RP2040 may
 * use short spinlocks: no claim that volatile alone, or every atomic, is lock-free.
 * These queues are used by tasks, not by ISRs.
 */
static BleComm_Rx_t s_rx_queue[BLECOMM_CFG_RX_QUEUE_SIZE];
static _Atomic(uint32_t) s_rx_q_head;
static _Atomic(uint32_t) s_rx_q_tail;
static BleComm_Tx_t s_tx_queue[BLECOMM_CFG_TX_QUEUE_SIZE];
static _Atomic(uint32_t) s_tx_q_head;
static _Atomic(uint32_t) s_tx_q_tail;
static BleComm_Tx_t s_current_tx;
static bool s_have_tx;
static BleComm_Policy_t s_policy;
static _Atomic(uint32_t) s_worker_generation;
static uint32_t s_observed_generation;
static bool s_start_attempted;
static bool s_radio_started;
static bool s_profile_programmed;
static uint8_t s_adv_data[BLECOMM_CFG_ADV_MAX_BYTES];
static uint8_t s_adv_data_len;

_Static_assert(BLECOMM_CFG_RX_QUEUE_SIZE > 1U, "RX queue needs a sentinel slot");
_Static_assert(BLECOMM_CFG_TX_QUEUE_SIZE > 1U, "TX queue needs a sentinel slot");
_Static_assert(sizeof(Ble_RawPacket_t) == 6U, "Compact status packet changed");
_Static_assert(sizeof(Ble_SafetyExtension_t) == 18U, "Safety extension changed");
_Static_assert(sizeof(Ble_RawFrame_t) == 24U, "Extended frame changed");
_Static_assert((7U + sizeof(Ble_RawFrame_t)) <= BLECOMM_CFG_ADV_MAX_BYTES,
               "Extended manufacturer frame exceeds legacy advertising capacity");

void BleComm_Encode(const Ble_AdvancedThreatPacket_t * const p_in, Ble_RawPacket_t * const p_out)
{
    if ((p_in == NULL) || (p_out == NULL))
    {
        return;
    }

    /* Byte 0: Sync (0xA) | Pair ID (4b) */
    uint8_t pair = (p_in->pair_id != 0U) ? p_in->pair_id : s_cfg.pair_id;
    p_out->preamble_pair_id = (uint8_t)((BLE_PACKET_SYNC_NIBBLE << 4) | (pair & 0x0FU));

    /* Byte 1: Node Type (1b) | Heartbeat Seq (3b) | Rolling Fall Seq (4b) */
    uint8_t type_bit = (p_in->node_type == BLE_NODE_TYPE_CANE) ? 0x80U : 0x00U;
    uint8_t hb_bits = (uint8_t)((p_in->heartbeat_seq & 0x07U) << 4);
    uint8_t fall_bits = (uint8_t)(p_in->rolling_fall_seq & 0x0FU);
    p_out->type_seq_fall = type_bit | hb_bits | fall_bits;

    /* Byte 2: State Flags */
    p_out->state_flags = p_in->state_flags;

    /* Distance (12b): 2mm step */
    uint16_t dist_raw;
    if ((p_in->nearest_dist_mm == 0xFFFFU) || (p_in->nearest_dist_mm > 8190U))
    {
        dist_raw = BLE_DIST_NO_TARGET;
    }
    else
    {
        dist_raw = (uint16_t)(p_in->nearest_dist_mm / 2U);
        if (dist_raw > 0x0FFEU)
        {
            dist_raw = 0x0FFEU;
        }
    }

    /* TTC (12b): 20ms step */
    uint16_t ttc_raw;
    if (p_in->ttc_ms == 0xFFFFU)
    {
        ttc_raw = BLE_TTC_NO_THREAT;
    }
    else
    {
        ttc_raw = (uint16_t)(p_in->ttc_ms / 20U);
        if (ttc_raw > 0x0FFEU)
        {
            ttc_raw = 0x0FFEU;
        }
    }

    /* Pack 12-bit Distance and 12-bit TTC across Bytes 3, 4, 5 */
    p_out->dist_low = (uint8_t)(dist_raw & 0xFFU);
    p_out->dist_high_ttc_low = (uint8_t)(((dist_raw >> 8) & 0x0FU) << 4) | (uint8_t)(ttc_raw & 0x0FU);
    p_out->ttc_high = (uint8_t)((ttc_raw >> 4) & 0xFFU);
}

bool BleComm_Decode(const Ble_RawPacket_t * const p_in, uint8_t expected_pair_id, Ble_AdvancedThreatPacket_t * const p_out)
{
    if ((p_in == NULL) || (p_out == NULL))
    {
        return false;
    }

    /* Verify Sync Nibble */
    uint8_t sync = (uint8_t)((p_in->preamble_pair_id >> 4) & 0x0FU);
    if (sync != BLE_PACKET_SYNC_NIBBLE)
    {
        return false;
    }

    /* Verify Pair ID */
    uint8_t rx_pair_id = (uint8_t)(p_in->preamble_pair_id & 0x0FU);
    if ((expected_pair_id != 0U) && (rx_pair_id != expected_pair_id))
    {
        return false;
    }

    memset(p_out, 0, sizeof(*p_out));

    p_out->preamble = 0xAAU;
    p_out->pair_id = rx_pair_id;
    p_out->node_type = ((p_in->type_seq_fall & 0x80U) != 0U) ? BLE_NODE_TYPE_CANE : BLE_NODE_TYPE_GLASSES;
    p_out->device_id = (p_out->node_type == BLE_NODE_TYPE_CANE) ? 0x01U : 0x02U;
    p_out->heartbeat_seq = (uint8_t)((p_in->type_seq_fall >> 4) & 0x07U);
    p_out->rolling_fall_seq = (uint8_t)(p_in->type_seq_fall & 0x0FU);
    p_out->seq_num = p_out->heartbeat_seq;
    p_out->state_flags = p_in->state_flags;

    /* Decode Distance */
    uint16_t dist_raw = (uint16_t)p_in->dist_low | (uint16_t)(((p_in->dist_high_ttc_low >> 4) & 0x0FU) << 8);
    if (dist_raw >= BLE_DIST_NO_TARGET)
    {
        p_out->nearest_dist_mm = 0xFFFFU;
    }
    else
    {
        p_out->nearest_dist_mm = (uint16_t)(dist_raw * 2U);
    }

    /* Decode TTC */
    uint16_t ttc_raw = (uint16_t)(p_in->dist_high_ttc_low & 0x0FU) | (uint16_t)((uint16_t)p_in->ttc_high << 4);
    if (ttc_raw >= BLE_TTC_NO_THREAT)
    {
        p_out->ttc_ms = 0xFFFFU;
    }
    else
    {
        p_out->ttc_ms = (uint16_t)(ttc_raw * 20U);
    }

    p_out->closing_speed_cms = 0;
    return true;
}

void BleComm_EncodeFrame(const Ble_AdvancedThreatPacket_t * const p_in,
                         Ble_RawFrame_t * const p_out)
{
    if ((p_in == NULL) || (p_out == NULL))
    {
        return;
    }

    BleComm_Encode(p_in, &p_out->status);
    p_out->safety.marker_version = BLE_SAFETY_EXTENSION_MARKER;
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        uint32_t shift = i * 8U;
        p_out->safety.session_id[i] = (uint8_t)(p_in->session_id >> shift);
        p_out->safety.event_id[i] = (uint8_t)(p_in->event_id >> shift);
        p_out->safety.peer_ack_session_id[i] =
            (uint8_t)(p_in->peer_ack_session_id >> shift);
        p_out->safety.peer_ack_event_id[i] =
            (uint8_t)(p_in->peer_ack_event_id >> shift);
    }
    p_out->safety.safety_flags = p_in->peer_ack_valid ? BLE_SAFETY_FLAG_PEER_ACK : 0U;
}

bool BleComm_DecodeFrame(const Ble_RawFrame_t * const p_in, uint8_t expected_pair_id,
                         Ble_AdvancedThreatPacket_t * const p_out)
{
    if ((p_in == NULL) || (p_out == NULL) ||
        (p_in->safety.marker_version != BLE_SAFETY_EXTENSION_MARKER) ||
        !BleComm_Decode(&p_in->status, expected_pair_id, p_out))
    {
        return false;
    }

    p_out->protocol_version = BLE_PROTOCOL_VERSION;
    p_out->session_id = 0U;
    p_out->event_id = 0U;
    p_out->peer_ack_session_id = 0U;
    p_out->peer_ack_event_id = 0U;
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        uint32_t shift = i * 8U;
        p_out->session_id |= (uint32_t)p_in->safety.session_id[i] << shift;
        p_out->event_id |= (uint32_t)p_in->safety.event_id[i] << shift;
        p_out->peer_ack_session_id |=
            (uint32_t)p_in->safety.peer_ack_session_id[i] << shift;
        p_out->peer_ack_event_id |=
            (uint32_t)p_in->safety.peer_ack_event_id[i] << shift;
    }
    p_out->peer_ack_valid = ((p_in->safety.safety_flags & BLE_SAFETY_FLAG_PEER_ACK) != 0U);
    return (p_out->session_id != 0U);
}

uint32_t BleComm_GetSessionId(void)
{
    return s_session_id;
}

bool BleComm_CheckNewFall(uint8_t rx_fall_seq, uint8_t last_fall_seq)
{
    uint8_t delta = (uint8_t)((rx_fall_seq - last_fall_seq) & 0x0FU);
    return (delta >= 1U) && (delta <= 7U);
}

bool BleComm_CheckNewEvent(uint32_t rx_event_id, uint32_t last_event_id)
{
    uint32_t delta = rx_event_id - last_event_id;
    return (delta != 0U) && (delta <= (UINT32_MAX / 2U));
}

bool BleComm_TrackPeerFall(BleComm_PeerFall_t * const p_tracker,
                          const Ble_AdvancedThreatPacket_t * const p_packet)
{
    if ((p_tracker == NULL) || (p_packet == NULL))
    {
        return false;
    }
    bool active = ((p_packet->state_flags & BLE_FLAG_FALL_ALERT) != 0U);
    uint8_t seq = (uint8_t)(p_packet->rolling_fall_seq & 0x0FU);
    bool extended = (p_packet->protocol_version == BLE_PROTOCOL_VERSION) &&
                    (p_packet->session_id != 0U);

    if (extended)
    {
        bool new_session = !p_tracker->initialized || !p_tracker->extended ||
                           (p_packet->session_id != p_tracker->session_id);
        bool same_event = !new_session && (p_packet->event_id == p_tracker->event_id);
        bool event_forward = !new_session &&
                             BleComm_CheckNewEvent(p_packet->event_id,
                                                   p_tracker->event_id);
        if (!new_session && !same_event && !event_forward)
        {
            return false; /* Delayed event identity from this boot session. */
        }

        bool event = active && (new_session || event_forward);
        if (new_session || event_forward)
        {
            p_tracker->session_id = p_packet->session_id;
            p_tracker->event_id = p_packet->event_id;
            p_tracker->fall_seq = seq;
        }
        p_tracker->initialized = true;
        p_tracker->extended = true;
        p_tracker->active = active;
        return event;
    }

    bool forward = BleComm_CheckNewFall(seq, p_tracker->fall_seq);
    bool event = active && (!p_tracker->initialized || p_tracker->extended ||
                            !p_tracker->active || forward);
    if (!p_tracker->initialized || p_tracker->extended || !p_tracker->active || forward)
    {
        p_tracker->fall_seq = seq;
        p_tracker->extended = false;
        p_tracker->session_id = 0U;
        p_tracker->event_id = 0U;
    }
    p_tracker->initialized = true;
    p_tracker->active = active;
    return event;
}

static void SetupAdvData(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    /* 1. Flags AD: LE General Discoverable Mode, BR/EDR Not Supported */
    s_adv_data[0] = 0x02U;
    s_adv_data[1] = 0x01U;
    s_adv_data[2] = 0x06U;

    /* The full local name is already in the scan response. Use the primary
     * advertisement for one atomic status + application-receipt frame.
     */
    Ble_RawFrame_t raw_frame;
    BleComm_EncodeFrame(p_pkt, &raw_frame);
    const uint8_t ad_payload_len = (uint8_t)(1U + 2U + sizeof(raw_frame));
    s_adv_data[3] = ad_payload_len;
    s_adv_data[4] = 0xFFU;
    s_adv_data[5] = 0x29U;
    s_adv_data[6] = 0x00U;
    memcpy(&s_adv_data[7], &raw_frame, sizeof(raw_frame));
    s_adv_data_len = (uint8_t)(7U + sizeof(raw_frame));
}

static void ParseAdvReport(const uint8_t * adv_report_data, uint8_t adv_report_len, uint32_t now_ms)
{
    if ((adv_report_data == NULL) || (adv_report_len > BLECOMM_CFG_ADV_MAX_BYTES))
    {
        return;
    }
    uint8_t i = 0U;
    while ((i + 1U) < adv_report_len)
    {
        uint8_t ad_len = adv_report_data[i];
        if (ad_len == 0U)
        {
            break;
        }
        if ((i + 1U + ad_len) > adv_report_len)
        {
            break;
        }

        uint8_t ad_type = adv_report_data[i + 1U];

        /* Check for Manufacturer Specific Data (0xFF) */
        if ((ad_type == 0xFFU) && (ad_len >= 9U)) /* Type + company ID + compact status. */
        {
            uint16_t company_id = (uint16_t)((uint16_t)adv_report_data[i + 2U] |
                                             ((uint16_t)adv_report_data[i + 3U] << 8));
            if (company_id == 0x0029U)
            {
                Ble_AdvancedThreatPacket_t pkt;
                Ble_NodeType_t own_type = (s_cfg.role == BLE_ROLE_BROADCASTER_GLASSES) ?
                                           BLE_NODE_TYPE_GLASSES : BLE_NODE_TYPE_CANE;
                bool decoded = false;
                uint8_t manufacturer_len = (uint8_t)(ad_len - 3U);
                if (manufacturer_len == sizeof(Ble_RawFrame_t))
                {
                    Ble_RawFrame_t frame;
                    memcpy(&frame, &adv_report_data[i + 4U], sizeof(frame));
                    decoded = BleComm_DecodeFrame(&frame, s_cfg.pair_id, &pkt);
                }
                else if (manufacturer_len == sizeof(Ble_RawPacket_t))
                {
                    Ble_RawPacket_t raw;
                    memcpy(&raw, &adv_report_data[i + 4U], sizeof(raw));
                    decoded = BleComm_Decode(&raw, s_cfg.pair_id, &pkt);
                }
                if (decoded && (pkt.node_type != own_type))
                {
                    (void)BleComm_ReceivePacketAt(&pkt, now_ms);
                }
            }
        }

        i = (uint8_t)(i + ad_len + 1U);
    }
}

static void Radio_Work(bool restarted, uint32_t now_ms)
{
    bool updated = false;
    /* Coalesce only a bounded number of snapshots; never replay a long stale FIFO. */
    for (uint32_t i = 0U; i < (BLECOMM_CFG_TX_QUEUE_SIZE - 1U); ++i)
    {
        uint32_t tail = atomic_load_explicit(&s_tx_q_tail, memory_order_relaxed);
        if (tail == atomic_load_explicit(&s_tx_q_head, memory_order_acquire))
        {
            break;
        }
        s_current_tx = s_tx_queue[tail];
        atomic_store_explicit(&s_tx_q_tail, (tail + 1U) % BLECOMM_CFG_TX_QUEUE_SIZE, memory_order_release);
        s_have_tx = true;
        updated = true;
    }
    BleComm_ProfileId_t previous = s_policy.effective;
    BleComm_ProfileId_t desired = s_have_tx ? s_current_tx.profile : BLECOMM_PROFILE_FAST;
    BleComm_ProfileId_t effective = BleComm_PolicyStep(&s_policy, desired, now_ms);
    if (BleRadio_IsReady())
    {
        bool configure = restarted || !s_profile_programmed || (previous != effective);
        if (configure)
        {
            BleRadio_Configure(&BleComm_GetProfile(effective)->radio);
            s_profile_programmed = true;
        }
        if ((updated || configure) && s_have_tx)
        {
            SetupAdvData(&s_current_tx.packet);
            BleRadio_SetAdvertisingData(s_adv_data, s_adv_data_len);
        }
        uint32_t generation = atomic_load_explicit(&s_worker_generation, memory_order_relaxed);
        atomic_store_explicit(&s_worker_generation, generation + 1U, memory_order_release);
    }
}

void BleComm_Init(const BleComm_Config_t * const p_cfg)
{
    s_cfg = (p_cfg != NULL) ? *p_cfg : BleComm_Config_Cane;
    s_last_rx_seq = 0xFFFFU;
    s_last_rx_ms = 0U;
    s_last_rx_flags = 0U;
    s_last_rx_fall = 0U;
    s_last_rx_session = 0U;
    s_last_rx_event = 0U;
    s_last_rx_ack_session = 0U;
    s_last_rx_ack_event = 0U;
    s_last_rx_ack_valid = false;
#ifdef PICO_ON_DEVICE
    s_session_id = get_rand_32();
    if (s_session_id == 0U)
    {
        s_session_id = 1U;
    }
#else
    /* Deterministic, role-distinct host identity; target boots use hardware entropy. */
    s_session_id = 0x29000000U | (uint32_t)s_cfg.device_id;
#endif
    atomic_store_explicit(&s_rx_q_head, 0U, memory_order_relaxed);
    atomic_store_explicit(&s_rx_q_tail, 0U, memory_order_relaxed);
    atomic_store_explicit(&s_tx_q_head, 0U, memory_order_relaxed);
    atomic_store_explicit(&s_tx_q_tail, 0U, memory_order_relaxed);
    atomic_store_explicit(&s_worker_generation, 0U, memory_order_relaxed);
    s_observed_generation = 0U;
    s_start_attempted = false;
    s_radio_started = false;
    s_profile_programmed = false;
    s_have_tx = false;
    BleComm_PolicyInit(&s_policy);
}

bool BleComm_SendPacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    return BleComm_Publish(p_pkt, SYS_STATE_ACTIVE, false);
}

bool BleComm_Publish(const Ble_AdvancedThreatPacket_t * const p_pkt, Sys_State_t state, bool link_lost)
{
    if (p_pkt == NULL)
    {
        return false;
    }

    uint32_t head = atomic_load_explicit(&s_tx_q_head, memory_order_relaxed);
    uint32_t next_head = (head + 1U) % BLECOMM_CFG_TX_QUEUE_SIZE;
    if (next_head == atomic_load_explicit(&s_tx_q_tail, memory_order_acquire))
    {
        return false; /* Queue full: drop packet without blocking */
    }

    s_tx_queue[head].packet = *p_pkt;
    s_tx_queue[head].packet.protocol_version = BLE_PROTOCOL_VERSION;
    s_tx_queue[head].packet.session_id = s_session_id;
    s_tx_queue[head].profile = BleComm_SelectProfile(state, p_pkt->state_flags, link_lost);
    atomic_store_explicit(&s_tx_q_head, next_head, memory_order_release);
    return true;
}

#ifndef PICO_ON_DEVICE
bool BleComm_ReceivePacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    /* Compatibility entry for host injection; the real transport always timestamps RX. */
    return BleComm_ReceivePacketAt(p_pkt, 0U);
}
#endif

bool BleComm_ReceivePacketAt(const Ble_AdvancedThreatPacket_t * const p_pkt, uint32_t now_ms)
{
    if (p_pkt == NULL)
    {
        return false;
    }
    /* Filter stale or delayed sequence numbers (wrap-safe modulo 8 for heartbeat sequence) */
    if (s_last_rx_seq != 0xFFFFU)
    {
        if ((p_pkt->seq_num <= 7U) && (s_last_rx_seq <= 7U))
        {
            uint8_t delta = (uint8_t)((p_pkt->seq_num - s_last_rx_seq) & 0x07U);
            bool state_changed = (p_pkt->state_flags != s_last_rx_flags) ||
                                  (p_pkt->rolling_fall_seq != s_last_rx_fall) ||
                                  (p_pkt->session_id != s_last_rx_session) ||
                                  (p_pkt->event_id != s_last_rx_event) ||
                                  (p_pkt->peer_ack_valid != s_last_rx_ack_valid) ||
                                  (p_pkt->peer_ack_session_id != s_last_rx_ack_session) ||
                                  (p_pkt->peer_ack_event_id != s_last_rx_ack_event);
            if (((delta == 0U) && !state_changed) ||
                ((delta > 4U) && ((uint32_t)(now_ms - s_last_rx_ms) < BLECOMM_CFG_RX_REBASE_MS)))
            {
                return false;
            }
        }
        else
        {
            if (p_pkt->seq_num <= s_last_rx_seq)
            {
                return false;
            }
        }
    }
    uint32_t head = atomic_load_explicit(&s_rx_q_head, memory_order_relaxed);
    uint32_t next_head = (head + 1U) % BLECOMM_CFG_RX_QUEUE_SIZE;
    if (next_head == atomic_load_explicit(&s_rx_q_tail, memory_order_acquire))
    {
        return false;
    }
    s_rx_queue[head].packet = *p_pkt;
    s_rx_queue[head].received_ms = now_ms;
    atomic_store_explicit(&s_rx_q_head, next_head, memory_order_release);
    /* Only accepted snapshots advance replay/freshness bookkeeping. */
    s_last_rx_seq = p_pkt->seq_num;
    s_last_rx_ms = now_ms;
    s_last_rx_flags = p_pkt->state_flags;
    s_last_rx_fall = p_pkt->rolling_fall_seq;
    s_last_rx_session = p_pkt->session_id;
    s_last_rx_event = p_pkt->event_id;
    s_last_rx_ack_valid = p_pkt->peer_ack_valid;
    s_last_rx_ack_session = p_pkt->peer_ack_session_id;
    s_last_rx_ack_event = p_pkt->peer_ack_event_id;
    return true;
}

bool BleComm_PopReceivedPacket(Ble_AdvancedThreatPacket_t * const p_pkt, uint32_t * const p_received_ms)
{
    if (p_pkt == NULL)
    {
        return false;
    }

    uint32_t tail = atomic_load_explicit(&s_rx_q_tail, memory_order_relaxed);
    if (tail == atomic_load_explicit(&s_rx_q_head, memory_order_acquire))
    {
        return false;
    }

    *p_pkt = s_rx_queue[tail].packet;
    if (p_received_ms != NULL)
    {
        *p_received_ms = s_rx_queue[tail].received_ms;
    }
    atomic_store_explicit(&s_rx_q_tail, (tail + 1U) % BLECOMM_CFG_RX_QUEUE_SIZE, memory_order_release);
    return true;
}

#ifndef PICO_ON_DEVICE
void BleComm_InjectRxPacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    (void)BleComm_ReceivePacket(p_pkt);
}
#endif

bool BleComm_Core1_Task(void)
{
    if (!s_start_attempted)
    {
        s_start_attempted = true;
        const char * const p_name = (s_cfg.role == BLE_ROLE_BROADCASTER_GLASSES) ?
                                    BLECOMM_CFG_DEVICE_NAME_GLASSES : BLECOMM_CFG_DEVICE_NAME_CANE;
        s_radio_started = BleRadio_Start(p_name, Radio_Work, ParseAdvReport);
    }
    if (!s_radio_started)
    {
        return false;
    }
    BleRadio_RequestWork();
    uint32_t generation = atomic_load_explicit(&s_worker_generation, memory_order_acquire);
    bool progressed = (generation != s_observed_generation);
    s_observed_generation = generation;
    return progressed && BleRadio_IsReady();
}
