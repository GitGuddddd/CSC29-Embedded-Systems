#ifndef CALBUTTON_H
#define CALBUTTON_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file CalButton.h
 * \brief Cane calibration button (GP20): debounce and 2 s hold detection (FR18)
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t gpio_pin;
} CalButton_Config_t;

typedef enum
{
    CALBTN_EVENT_NONE = 0U,
    CALBTN_EVENT_GROUND_HOLD    /* Held for 2 s: calibrate the ground baseline */
} CalButton_Event_t;

extern const CalButton_Config_t CalButton_Config_Cane;

void CalButton_Init(const CalButton_Config_t * const p_cfg);
CalButton_Event_t CalButton_Update(bool raw_pin_level, uint32_t now_ms);

#endif /* CALBUTTON_H */
