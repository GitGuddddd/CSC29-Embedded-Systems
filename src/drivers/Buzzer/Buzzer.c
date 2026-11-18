#include "Buzzer.h"
#include "Buzzer_Cfg.h"
#include "hal/Hal_Pwm.h"
#include <stddef.h>

static uint8_t s_buzzer_pin = 18U;
static bool s_fault_injected = false;
static uint16_t s_active_freq_hz = 0U;
static uint8_t s_active_duty_pct = 0U;

void Buzzer_Init(const Buzzer_Config_t * const p_cfg)
{
    s_fault_injected = false;
    s_active_freq_hz = 0U;
    s_active_duty_pct = 0U;
    if (p_cfg != NULL)
    {
        s_buzzer_pin = p_cfg->gpio_pin;
    }
    Hal_Pwm_Init(s_buzzer_pin, 1000U, 0U);
}

void Buzzer_InjectFault(bool fault_active)
{
    s_fault_injected = fault_active;
    if (fault_active)
    {
        Hal_Pwm_Stop(s_buzzer_pin);
        s_active_freq_hz = 0U;
        s_active_duty_pct = 0U;
    }
}

bool Buzzer_IsHealthy(void)
{
    return !s_fault_injected && Hal_Pwm_IsHealthy(s_buzzer_pin);
}

uint16_t Buzzer_GetActiveFrequencyHz(void)
{
    return s_active_freq_hz;
}

uint8_t Buzzer_GetActiveDutyPercent(void)
{
    return s_active_duty_pct;
}

void Buzzer_PlayTone(uint16_t freq_hz)
{
    Buzzer_PlayToneAtDuty(freq_hz, BUZZER_CFG_DUTY_PCT);
}

void Buzzer_PlayToneAtDuty(uint16_t freq_hz, uint8_t duty_percent)
{
    if (!Buzzer_IsHealthy())
    {
        Hal_Pwm_Stop(s_buzzer_pin);
        s_active_freq_hz = 0U;
        s_active_duty_pct = 0U;
        return;
    }

    if ((freq_hz > 0U) && (duty_percent > 0U))
    {
        uint8_t bounded_duty = (duty_percent <= 100U) ? duty_percent : 100U;
        Hal_Pwm_SetFrequency(s_buzzer_pin, freq_hz, bounded_duty);
        s_active_freq_hz = freq_hz;
        s_active_duty_pct = bounded_duty;
    }
    else
    {
        Buzzer_Silence();
    }
}

void Buzzer_Silence(void)
{
    Hal_Pwm_Stop(s_buzzer_pin);
    s_active_freq_hz = 0U;
    s_active_duty_pct = 0U;
}
