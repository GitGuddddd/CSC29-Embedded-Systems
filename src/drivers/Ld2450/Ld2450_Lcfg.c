#include "Ld2450.h"

/*
 * \file Ld2450_Lcfg.c
 * \brief Link-time configuration for Glasses mmWave radar
 * Compliance: Barr-C:2018
 */

const Ld2450_Config_t Ld2450_Config_Glasses = {
    .uart_port = 0U,
    .tx_pin    = 1U,
    .rx_pin    = 0U
};
