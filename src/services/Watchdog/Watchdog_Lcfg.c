#include "Watchdog.h"
#include "Watchdog_Cfg.h"

/*
 * \file Watchdog_Lcfg.c
 * \brief Link-time configuration for Deadman watchdog
 * Compliance: Barr-C:2018
 */

const Watchdog_Config_t Watchdog_Config_Default = {
    .deadman_timeout_ms = WATCHDOG_CFG_DEADMAN_TIMEOUT_MS
};
