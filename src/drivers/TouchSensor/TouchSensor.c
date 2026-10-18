#include "TouchSensor.h"
#include "TouchSensor_Cfg.h"
#include "hal/Hal_Gpio.h"
#include <stddef.h>

static Grip_State_t s_committed_state = GRIP_STATE_RELEASED;
static bool s_candidate_level = false;
static uint8_t s_consecutive_count = 0U;

void TouchSensor_Init(const TouchSensor_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        Hal_Gpio_Init(p_cfg->gpio_pin, HAL_GPIO_DIR_IN, false);
    }
    s_committed_state = GRIP_STATE_RELEASED;
    s_candidate_level = false;
    s_consecutive_count = 0U;
}

bool TouchSensor_Update(const TouchSensor_Config_t * const p_cfg, bool raw_pin_level, Grip_State_t * const p_state_out)
{
    (void)p_cfg;

    if (raw_pin_level != s_candidate_level)
    {
        s_candidate_level = raw_pin_level;
        s_consecutive_count = 1U;
    }
    else
    {
        if (s_consecutive_count < TOUCH_CFG_DEBOUNCE_COUNT)
        {
            s_consecutive_count++;
        }
    }

    bool transition_occurred = false;

    if (s_consecutive_count >= TOUCH_CFG_DEBOUNCE_COUNT)
    {
        Grip_State_t new_state = (s_candidate_level) ? GRIP_STATE_HELD : GRIP_STATE_RELEASED;
        if (new_state != s_committed_state)
        {
            s_committed_state = new_state;
            transition_occurred = true;
        }
    }

    if (p_state_out != NULL)
    {
        *p_state_out = s_committed_state;
    }

    return transition_occurred;
}
