#include "CalButton.h"
#include "Cane_Board.h"

/*
 * \file CalButton_Lcfg.c
 * \brief Link-time pin mapping for the Cane calibration button
 * Compliance: Barr-C:2018
 */

const CalButton_Config_t CalButton_Config_Cane = {
    .gpio_pin = CANE_PIN_CAL_BTN
};
