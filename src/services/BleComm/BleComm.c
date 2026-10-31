#include "BleComm.h"
#include "BleComm_Cfg.h"
#include <string.h>
#include <stdio.h>

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"
#include "btstack.h"
#else
#define __dmb() do { __asm__ __volatile__("" : : : "memory"); } while (0)
#endif

/*
 * \file BleComm.c
 * \brief CYW43439 Bluetooth LE Connectionless Broadcaster Implementation (Compact 6-byte Unified Packet)
 * Compliance: Barr-C:2018
 */

static BleComm_Config_t s_cfg;
static uint16_t s_last_rx_seq = 0xFFFFU;
static uint16_t s_tx_seq = 0U;

#define BLECOMM_RX_QUEUE_SIZE 4U
static Ble_AdvancedThreatPacket_t s_rx_queue[BLECOMM_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_q_head = 0U;
static volatile uint8_t s_rx_q_tail = 0U;

#define BLECOMM_TX_QUEUE_SIZE 4U
static Ble_AdvancedThreatPacket_t s_tx_queue[BLECOMM_TX_QUEUE_SIZE];
static volatile uint8_t s_tx_q_head = 0U;
static volatile uint8_t s_tx_q_tail = 0U;

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

bool BleComm_CheckNewFall(uint8_t rx_fall_seq, uint8_t last_fall_seq)
{
    uint8_t delta = (uint8_t)((rx_fall_seq - last_fall_seq) & 0x0FU);
    return (delta >= 1U) && (delta <= 7U);
}

#ifdef PICO_ON_DEVICE
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint8_t adv_data[31];
static uint8_t adv_data_len = 0;

static uint8_t scan_resp_data[31];
static uint8_t scan_resp_data_len = 0;

static void SetupScanRespData(void)
{
    const char *dev_name = (s_cfg.role == BLE_ROLE_BROADCASTER_GLASSES) ?
                            BLECOMM_CFG_DEVICE_NAME_GLASSES : BLECOMM_CFG_DEVICE_NAME_CANE;
    uint8_t name_len = (uint8_t)strlen(dev_name);
    scan_resp_data[0] = name_len + 1;
    scan_resp_data[1] = 0x09; /* Complete Local Name */
    memcpy(&scan_resp_data[2], dev_name, name_len);
    scan_resp_data_len = name_len + 2;
}

static void SetupAdvData(const Ble_AdvancedThreatPacket_t *p_pkt)
{
    const char *dev_name = (s_cfg.role == BLE_ROLE_BROADCASTER_GLASSES) ?
                            "EG-Glasses" : "EG-Cane";
    uint8_t name_len = (uint8_t)strlen(dev_name);

    /* 1. Flags AD: LE General Discoverable Mode, BR/EDR Not Supported */
    adv_data[0] = 0x02;
    adv_data[1] = 0x01;
    adv_data[2] = 0x06;

    /* 2. Short Local Name AD */
    adv_data[3] = name_len + 1;
    adv_data[4] = 0x08; /* Shortened Local Name */
    memcpy(&adv_data[5], dev_name, name_len);

    uint8_t idx = 5 + name_len;

    Ble_AdvancedThreatPacket_t local_pkt;
    if (p_pkt != NULL)
    {
        local_pkt = *p_pkt;
    }
    else
    {
        memset(&local_pkt, 0, sizeof(local_pkt));
        local_pkt.node_type = (s_cfg.role == BLE_ROLE_BROADCASTER_GLASSES) ?
                              BLE_NODE_TYPE_GLASSES : BLE_NODE_TYPE_CANE;
        local_pkt.pair_id = s_cfg.pair_id;
        local_pkt.nearest_dist_mm = 0xFFFFU;
        local_pkt.ttc_ms = 0xFFFFU;
    }
    local_pkt.heartbeat_seq = (uint8_t)(s_tx_seq & 0x07U);

    Ble_RawPacket_t raw_pkt;
    BleComm_Encode(&local_pkt, &raw_pkt);

    /* 3. Manufacturer Data AD: 1B Type + 2B Company ID + 6B Raw Packet = 9 bytes */
    adv_data[idx]      = 9U;   /* AD length = 9 bytes */
    adv_data[idx + 1]  = 0xFF; /* Manufacturer Specific */
    adv_data[idx + 2]  = 0x29; /* Company ID Low */
    adv_data[idx + 3]  = 0x00; /* Company ID High */
    memcpy(&adv_data[idx + 4], &raw_pkt, sizeof(Ble_RawPacket_t));

    adv_data_len = idx + 10U;
}

static void ParseAdvReport(const uint8_t * adv_report_data, uint8_t adv_report_len)
{
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
        if ((ad_type == 0xFFU) && (ad_len >= 9U)) /* 1B type + 2B Company ID + 6B raw packet */
        {
            uint16_t company_id = (uint16_t)adv_report_data[i + 2U] | ((uint16_t)adv_report_data[i + 3U] << 8);
            if (company_id == 0x0029U)
            {
                Ble_RawPacket_t raw;
                memcpy(&raw, &adv_report_data[i + 4U], sizeof(Ble_RawPacket_t));

                Ble_AdvancedThreatPacket_t pkt;
                if (BleComm_Decode(&raw, s_cfg.pair_id, &pkt))
                {
                    (void)BleComm_ReceivePacket(&pkt);
                }
            }
        }

        i += (ad_len + 1U);
    }
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET)
    {
        return;
    }

    uint8_t event = hci_event_packet_get_type(packet);
    if (event == BTSTACK_EVENT_STATE)
    {
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
        {
            const char *display_name = (s_cfg.role == BLE_ROLE_BROADCASTER_GLASSES) ?
                                       BLECOMM_CFG_DEVICE_NAME_GLASSES : BLECOMM_CFG_DEVICE_NAME_CANE;
            printf("[BLE] BTstack controller working. Enabling BLE Advertising & Passive Scanning...\n");

            /* Set 100ms (10 Hz per NFR13) advertising interval: 160 * 0.625ms = 100ms */
            gap_advertisements_set_params(0x00A0, 0x00A0, 0x00, 0x00, NULL, 0x07, 0x00);

            SetupAdvData(NULL);
            gap_advertisements_set_data(adv_data_len, adv_data);

            SetupScanRespData();
            gap_scan_response_set_data(scan_resp_data_len, scan_resp_data);

            gap_advertisements_enable(1);

            /* Enable Continuous Passive Scanning (NFR13: 80ms scan window / 100ms interval) */
            gap_set_scan_parameters(0, 0x00A0, 0x0080);
            gap_start_scan();

            printf("[BLE] Active as '%s' (Advertising & Scanning started)\n", display_name);
        }
    }
    else if (event == GAP_EVENT_ADVERTISING_REPORT)
    {
        uint8_t event_data_len = gap_event_advertising_report_get_data_length(packet);
        const uint8_t * event_data = gap_event_advertising_report_get_data(packet);
        if ((event_data != NULL) && (event_data_len > 0U))
        {
            ParseAdvReport(event_data, event_data_len);
        }
    }
    else
    {
        /* Other HCI events ignored */
    }
}
#endif

void BleComm_Init(const BleComm_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        s_cfg = *p_cfg;
    }
    s_last_rx_seq = 0xFFFFU;
    s_tx_seq = 0U;
    s_rx_q_head = 0U;
    s_rx_q_tail = 0U;
    s_tx_q_head = 0U;
    s_tx_q_tail = 0U;

#ifdef PICO_ON_DEVICE
    if (cyw43_arch_init())
    {
        printf("[BLE ERROR] Failed to initialize CYW43 Bluetooth hardware\n");
        return;
    }

    l2cap_init();
    sm_init();

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    printf("[BLE] CYW43439 Bluetooth Hardware Initialized on Core 1\n");
#endif
}

bool BleComm_SendPacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    if (p_pkt == NULL)
    {
        return false;
    }

    /* Core 0 Producer: push into lock-free SPSC TX queue */
    uint8_t next_head = (uint8_t)((s_tx_q_head + 1U) % BLECOMM_TX_QUEUE_SIZE);
    if (next_head == s_tx_q_tail)
    {
        return false; /* Queue full: drop packet without blocking */
    }

    s_tx_queue[s_tx_q_head] = *p_pkt;
    __dmb();
    s_tx_q_head = next_head;
    s_tx_seq++;
    return true;
}

bool BleComm_ReceivePacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
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
            if ((delta == 0U) || (delta > 4U))
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
    s_last_rx_seq = p_pkt->seq_num;

    /* Push into reception queue with memory barrier */
    uint8_t next_head = (uint8_t)((s_rx_q_head + 1U) % BLECOMM_RX_QUEUE_SIZE);
    if (next_head != s_rx_q_tail)
    {
        s_rx_queue[s_rx_q_head] = *p_pkt;
        __dmb();
        s_rx_q_head = next_head;
    }

    return true;
}

bool BleComm_PopReceivedPacket(Ble_AdvancedThreatPacket_t * const p_pkt)
{
    if (p_pkt == NULL)
    {
        return false;
    }

    if (s_rx_q_tail == s_rx_q_head)
    {
        return false;
    }

    *p_pkt = s_rx_queue[s_rx_q_tail];
    __dmb();
    s_rx_q_tail = (uint8_t)((s_rx_q_tail + 1U) % BLECOMM_RX_QUEUE_SIZE);
    return true;
}

void BleComm_InjectRxPacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    (void)BleComm_ReceivePacket(p_pkt);
}

void BleComm_Core1_Task(void)
{
#ifdef PICO_ON_DEVICE
    cyw43_arch_poll();

    /* Core 1 Consumer: process pending outbound packets from Core 0 */
    if (s_tx_q_tail != s_tx_q_head)
    {
        Ble_AdvancedThreatPacket_t pkt_to_send = s_tx_queue[s_tx_q_tail];
        __dmb();
        s_tx_q_tail = (uint8_t)((s_tx_q_tail + 1U) % BLECOMM_TX_QUEUE_SIZE);

        SetupAdvData(&pkt_to_send);
        gap_advertisements_set_data(adv_data_len, adv_data);
    }
#endif
}
