#include "Haptic.h"
#include "Haptic_Cfg.h"
#include "hal/Hal_Pwm.h"
#include <stddef.h>

static uint8_t s_haptic_pin = 15U;
static Alert_Type_t s_active_pattern = ALERT_NONE;
static volatile bool s_abort_haptic = false;

void Haptic_Init(const Haptic_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        s_haptic_pin = p_cfg->gpio_pin;
    }
    Hal_Pwm_Init(s_haptic_pin, HAPTIC_CFG_PWM_FREQ_HZ, 0U);
    s_active_pattern = ALERT_NONE;
    s_abort_haptic = false;
}

void Haptic_PlayPattern(Alert_Type_t pattern_type)
{
    /* Physical hazard strictly preempts and aborts background link-loss */
    if ((s_active_pattern == ALERT_LINK_LOSS) && 
        ((pattern_type == ALERT_CLIFF_WARNING) || (pattern_type == ALERT_CLIFF_STEP_UP)))
    {
        s_abort_haptic = true;
        Haptic_Abort();
    }

    s_abort_haptic = false;
    s_active_pattern = pattern_type;
    Hal_Pwm_SetFrequency(s_haptic_pin, HAPTIC_CFG_PWM_FREQ_HZ, HAPTIC_CFG_PWM_DUTY_PCT);
}

void Haptic_Abort(void)
{
    s_abort_haptic = true;
    Hal_Pwm_Stop(s_haptic_pin);
    s_active_pattern = ALERT_NONE;
}

bool Haptic_IsAborted(void)
{
    return s_abort_haptic;
}

void Haptic_Step(uint32_t now_ms)
{
    (void)now_ms;
}
