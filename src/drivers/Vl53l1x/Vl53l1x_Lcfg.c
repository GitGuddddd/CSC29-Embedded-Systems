#include "Vl53l1x.h"

/*
 * \file Vl53l1x_Lcfg.c
 * \brief Link-time configuration for Cane ToF sensor
 * Compliance: Barr-C:2018
 */

const Vl53l1x_Config_t Vl53l1x_Config_Cane = {
    .i2c_port  = 1U,
    .dev_addr  = 0x29U,
    .xshut_pin = 255U /* Unused / tied to 3V3 */
};
