#ifndef WATCHDOG_CFG_H
#define WATCHDOG_CFG_H

/*
 * \file Watchdog_Cfg.h
 * \brief Compile-time configuration for Deadman link-loss & system watchdog (FR16)
 * Compliance: Barr-C:2018
 */

#define WATCHDOG_CFG_DEADMAN_TIMEOUT_MS  (500U)
#define WATCHDOG_CFG_HW_TIMEOUT_MS       (2000U)

#endif /* WATCHDOG_CFG_H */
