#include "CalButton.h"
#include "CalButton_Cfg.h"
#include "hal/Hal_Gpio.h"
#include <stddef.h>

/*
 * \file CalButton.c
 * \brief Time-based debounce and one-shot hold detection for the cane calibration button
 * Compliance: Barr-C:2018
 */

static bool s_raw_pressed = false;
static uint32_t s_raw_since_ms = 0U;
static bool s_pressed = false;
static uint32_t s_press_start_ms = 0U;
static bool s_hold_reported = false;

void CalButton_Init(const CalButton_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        /* Active-low: the button pulls the pin to GND when pressed */
        Hal_Gpio_Init(p_cfg->gpio_pin, HAL_GPIO_DIR_IN, true);
    }
    s_raw_pressed = false;
    s_raw_since_ms = 0U;
    s_pressed = false;
    s_press_start_ms = 0U;
    s_hold_reported = false;
}

CalButton_Event_t CalButton_Update(bool raw_pin_level, uint32_t now_ms)
{
    bool raw_pressed = !raw_pin_level;

    if (raw_pressed != s_raw_pressed)
    {
        s_raw_pressed = raw_pressed;
        s_raw_since_ms = now_ms;
    }

    /* Accept a new level only after it has been stable for the debounce time */
    if ((raw_pressed != s_pressed) &&
        ((uint32_t)(now_ms - s_raw_since_ms) >= CALBTN_CFG_DEBOUNCE_MS))
    {
        s_pressed = raw_pressed;
        if (s_pressed)
        {
            s_press_start_ms = s_raw_since_ms;
            s_hold_reported = false;
        }
    }

    /* Report once per press, as soon as the hold reaches 2 s, so the start cue tells the user to let go */
    if (s_pressed && (!s_hold_reported) &&
        ((uint32_t)(now_ms - s_press_start_ms) >= CALBTN_CFG_GROUND_HOLD_MS))
    {
        s_hold_reported = true;
        return CALBTN_EVENT_GROUND_HOLD;
    }

    return CALBTN_EVENT_NONE;
}
