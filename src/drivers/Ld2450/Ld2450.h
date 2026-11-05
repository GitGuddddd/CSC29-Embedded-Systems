#ifndef LD2450_H
#define LD2450_H

#include <stdint.h>
#include <stdbool.h>
#include "Rte_Type.h"

/*
 * \file Ld2450.h
 * \brief HLK-LD2450 radar frame parser and driver interface (FR7)
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t uart_port;
    uint8_t tx_pin;
    uint8_t rx_pin;
} Ld2450_Config_t;

extern const Ld2450_Config_t Ld2450_Config_Glasses;

bool Ld2450_Init(const Ld2450_Config_t * const p_cfg);
bool Ld2450_ParseFrame(const uint8_t * const p_frame, uint32_t len, Radar_Target_t * const p_targets, uint8_t * const p_count);
/* Task-only: bounded scan, newest complete frame; p_targets has room for three targets. */
bool Ld2450_ProcessStream(Radar_Target_t * const p_targets, uint8_t * const p_count, uint32_t now_ms);
bool Ld2450_HasRxFault(void);
uint32_t Ld2450_GetOverrunCount(void);

#endif /* LD2450_H */
