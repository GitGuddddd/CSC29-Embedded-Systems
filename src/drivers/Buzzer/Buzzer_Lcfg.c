#include "Buzzer.h"

/*
 * \file Buzzer_Lcfg.c
 * \brief Link-time configuration for Maker Pi Pico on-board buzzers
 * Compliance: Barr-C:2018
 */

const Buzzer_Config_t Buzzer_Config_Cane = {
    .gpio_pin = 18U
};

const Buzzer_Config_t Buzzer_Config_Glasses = {
    .gpio_pin = 18U
};
