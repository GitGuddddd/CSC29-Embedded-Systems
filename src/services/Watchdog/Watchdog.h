#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Watchdog.h
 * \brief 500ms Deadman Watchdog for BLE link loss and RP2040 hardware watchdog
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint32_t deadman_timeout_ms;
} Watchdog_Config_t;

extern const Watchdog_Config_t Watchdog_Config_Default;

void Watchdog_Init(const Watchdog_Config_t * const p_cfg);
void Watchdog_KickBleDeadman(uint32_t now_ms);
bool Watchdog_CheckBleLinkLoss(uint32_t now_ms);

#endif /* WATCHDOG_H */
