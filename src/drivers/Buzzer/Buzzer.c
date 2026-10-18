#include "Buzzer.h"
#include "Buzzer_Cfg.h"
#include "hal/Hal_Pwm.h"
#include <stddef.h>

static uint8_t s_buzzer_pin = 18U;

void Buzzer_Init(const Buzzer_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        s_buzzer_pin = p_cfg->gpio_pin;
    }
    Hal_Pwm_Init(s_buzzer_pin, 1000U, 0U);
}

void Buzzer_PlayTone(uint16_t freq_hz)
{
    if (freq_hz > 0U)
    {
        Hal_Pwm_SetFrequency(s_buzzer_pin, freq_hz, BUZZER_CFG_DUTY_PCT);
    }
    else
    {
        Buzzer_Silence();
    }
}

void Buzzer_Silence(void)
{
    Hal_Pwm_Stop(s_buzzer_pin);
}
