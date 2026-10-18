#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

/*
 * \file btstack_config.h
 * \brief BTstack BLE configuration for Raspberry Pi Pico W CYW43439
 * Compliance: Barr-C:2018
 */

#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif

#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

/* BTstack resource limits */
#define MAX_NR_HCI_CONNECTIONS            1
#define MAX_NR_L2CAP_SERVICES             2
#define MAX_NR_L2CAP_CHANNELS             2
#define MAX_NR_GATT_CLIENTS               1
#define MAX_NR_WHITELIST_ENTRIES          1
#define MAX_NR_SM_LOOKUP_ENTRIES          3
#define MAX_NR_LE_DEVICE_DB_ENTRIES       1
#define NVM_NUM_DEVICE_DB_ENTRIES         1

#define MAX_NR_LE_AUDIO_BROADCAST_SOURCES 0
#define MAX_NR_LE_AUDIO_BROADCAST_SINKS   0

#define MAX_ATT_DB_SIZE                   256

/* HCI Controller to Host Flow Control & CYW43 Buffer Alignments */
#define HCI_INCOMING_PRE_BUFFER_SIZE      14
#define HCI_OUTGOING_PRE_BUFFER_SIZE      4
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT     4
#define HCI_ACL_PAYLOAD_SIZE              (255 + 4)

#define NVM_NUM_LINK_KEYS                 1

#endif /* BTSTACK_CONFIG_H */
