#ifndef SAFETYDELIVERY_CFG_H
#define SAFETYDELIVERY_CFG_H

/*
 * \file SafetyDelivery_Cfg.h
 * \brief Critical-event delivery timing configuration
 * Compliance: Barr-C:2018
 */

/* Allows multiple urgent advertising/scan opportunities plus the return receipt. */
#define SAFETYDELIVERY_CFG_RECEIPT_TIMEOUT_MS (750U)
#define SAFETYDELIVERY_CFG_PEER_TIMEOUT_MS    (500U)

#endif /* SAFETYDELIVERY_CFG_H */
