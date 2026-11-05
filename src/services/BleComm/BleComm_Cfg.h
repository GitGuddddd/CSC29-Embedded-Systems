#ifndef BLECOMM_CFG_H
#define BLECOMM_CFG_H

/*
 * \file BleComm_Cfg.h
 * \brief Compile-time configuration for connectionless BLE broadcaster/observer (NFR13)
 * Compliance: Barr-C:2018
 */

#define BLECOMM_CFG_ADV_FAST_MS        (100U)
#define BLECOMM_CFG_ADV_ACTIVE_MS      (150U)
#define BLECOMM_CFG_ADV_QUIET_MS       (200U)
#define BLECOMM_CFG_PAYLOAD_FAST_MS    (50U)
#define BLECOMM_CFG_PAYLOAD_ACTIVE_MS  (50U)
#define BLECOMM_CFG_PAYLOAD_QUIET_MS   (100U)
#define BLECOMM_CFG_SCAN_INTERVAL_MS   (100U)
#define BLECOMM_CFG_SCAN_NORMAL_MS     (80U)
#define BLECOMM_CFG_SCAN_URGENT_MS     (100U)
#define BLECOMM_CFG_DOWNSHIFT_HOLD_MS   (1000U)
#define BLECOMM_CFG_ADV_RANDOM_MS      (10U) /* Controller adds 0-10 ms advertising delay. */
#define BLECOMM_CFG_DELIVERY_MARGIN_MS (50U)
#define BLECOMM_CFG_RX_REBASE_MS       (250U) /* Half-window ordering is ambiguous after a gap. */
#define BLECOMM_CFG_TX_QUEUE_SIZE      (4U)
#define BLECOMM_CFG_RX_QUEUE_SIZE      (8U)
#define BLECOMM_CFG_RADIO_TASK_MS      (10U)
#define BLECOMM_CFG_ADV_MAX_BYTES      (31U)
#define BLECOMM_CFG_DEVICE_NAME_CANE  "EchoGuard Smart Cane"
#define BLECOMM_CFG_DEVICE_NAME_GLASSES "EchoGuard Glasses Node"

#endif /* BLECOMM_CFG_H */
