#include "Vl53l0x.h"

/*
 * \file Vl53l0x_Lcfg.c
 * \brief Link-time configuration for Cane VL53L0X sensor
 * Compliance: Barr-C:2018
 */

const Vl53l0x_Config_t Vl53l0x_Config_Cane = {
    .i2c_port  = 1U,
    .dev_addr  = 0x29U,
    .xshut_pin = 255U /* Tied to 3V3 */
};
