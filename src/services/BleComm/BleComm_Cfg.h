#ifndef BLECOMM_CFG_H
#define BLECOMM_CFG_H

/*
 * \file BleComm_Cfg.h
 * \brief Compile-time configuration for connectionless BLE broadcaster/observer (NFR13)
 * Compliance: Barr-C:2018
 */

#define BLECOMM_CFG_RATE_HZ           (10U)
#define BLECOMM_CFG_SCAN_WINDOW_MS    (80U)
#define BLECOMM_CFG_ADV_WINDOW_MS     (20U)
#define BLECOMM_CFG_BURST_REPEATS     (5U)
#define BLECOMM_CFG_DEVICE_NAME_CANE  "EchoGuard Smart Cane"
#define BLECOMM_CFG_DEVICE_NAME_GLASSES "EchoGuard Glasses Node"

#endif /* BLECOMM_CFG_H */
