#include "BleComm.h"
#include "BleComm_Cfg.h"
#include <string.h>
#include <stdio.h>

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#endif

/*
 * \file BleComm.c
 * \brief CYW43439 Bluetooth LE Connectionless Broadcaster Implementation (11-byte AdvancedThreatPacket)
 * Compliance: Barr-C:2018
 */

static BleComm_Config_t s_cfg;
static uint16_t s_last_rx_seq = 0U;
static uint16_t s_tx_seq = 0U;

#define BLECOMM_RX_QUEUE_SIZE 4U
static Ble_AdvancedThreatPacket_t s_rx_queue[BLECOMM_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_q_head = 0U;
static volatile uint8_t s_rx_q_tail = 0U;

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

    uint8_t preamble = (p_pkt != NULL) ? p_pkt->preamble : 0xAA;
    uint8_t dev_id   = (p_pkt != NULL) ? p_pkt->device_id : s_cfg.device_id;
    uint16_t seq     = s_tx_seq;
    uint8_t flags    = (p_pkt != NULL) ? p_pkt->state_flags : 0x00;
    uint16_t dist    = (p_pkt != NULL) ? p_pkt->nearest_dist_mm : 1200U;
    int16_t speed    = (p_pkt != NULL) ? p_pkt->closing_speed_cms : 15;
    uint16_t ttc     = (p_pkt != NULL) ? p_pkt->ttc_ms : 8000U;

    /* 3. Manufacturer Data AD (13 bytes total: 2B Company ID + 11B AdvancedThreatPacket) */
    adv_data[idx]      = 13;   /* AD length = 13 bytes */
    adv_data[idx + 1]  = 0xFF; /* Manufacturer Specific */
    adv_data[idx + 2]  = 0x29; /* Company ID Low */
    adv_data[idx + 3]  = 0x00; /* Company ID High */
    adv_data[idx + 4]  = preamble;
    adv_data[idx + 5]  = dev_id;
    adv_data[idx + 6]  = (uint8_t)(seq & 0xFF);
    adv_data[idx + 7]  = (uint8_t)((seq >> 8) & 0xFF);
    adv_data[idx + 8]  = flags;
    adv_data[idx + 9]  = (uint8_t)(dist & 0xFF);
    adv_data[idx + 10] = (uint8_t)((dist >> 8) & 0xFF);
    adv_data[idx + 11] = (uint8_t)(speed & 0xFF);
    adv_data[idx + 12] = (uint8_t)((speed >> 8) & 0xFF);
    adv_data[idx + 13] = (uint8_t)(ttc & 0xFF);
    adv_data[idx + 14] = (uint8_t)((ttc >> 8) & 0xFF);

    adv_data_len = idx + 15;
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
        if ((ad_type == 0xFFU) && (ad_len >= 15U)) /* 1B type + 2B Company ID + 11B packet = 14B payload + 1B type = 15B total */
        {
            uint16_t company_id = (uint16_t)adv_report_data[i + 2U] | ((uint16_t)adv_report_data[i + 3U] << 8);
            if (company_id == 0x0029U)
            {
                Ble_AdvancedThreatPacket_t pkt;
                memcpy(&pkt, &adv_report_data[i + 4U], sizeof(Ble_AdvancedThreatPacket_t));

                if (pkt.preamble == 0xAAU)
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
    s_last_rx_seq = 0U;
    s_tx_seq = 0U;
    s_rx_q_head = 0U;
    s_rx_q_tail = 0U;

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
    s_tx_seq++;

#ifdef PICO_ON_DEVICE
    SetupAdvData(p_pkt);
    gap_advertisements_set_data(adv_data_len, adv_data);
#endif

    return true;
}

bool BleComm_ReceivePacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    if (p_pkt == NULL)
    {
        return false;
    }
    /* Filter stale or delayed sequence numbers (FR15) */
    if (p_pkt->seq_num <= s_last_rx_seq)
    {
        return false;
    }
    s_last_rx_seq = p_pkt->seq_num;

    /* Push into reception queue */
    uint8_t next_head = (uint8_t)((s_rx_q_head + 1U) % BLECOMM_RX_QUEUE_SIZE);
    if (next_head != s_rx_q_tail)
    {
        s_rx_queue[s_rx_q_head] = *p_pkt;
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
    s_rx_q_tail = (uint8_t)((s_rx_q_tail + 1U) % BLECOMM_RX_QUEUE_SIZE);
    return true;
}

void BleComm_InjectRxPacket(const Ble_AdvancedThreatPacket_t * const p_pkt)
{
    (void)BleComm_ReceivePacket(p_pkt);
}
