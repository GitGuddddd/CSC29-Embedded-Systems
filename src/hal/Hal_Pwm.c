#include "Hal_Pwm.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#endif

void Hal_Pwm_Init(uint8_t pin, uint32_t freq_hz, uint8_t duty_percent)
{
#ifdef PICO_ON_DEVICE
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);

    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t divider = 16;
    uint32_t wrap = (clock_freq / (divider * freq_hz)) - 1;

    pwm_set_clkdiv(slice_num, (float)divider);
    pwm_set_wrap(slice_num, (uint16_t)wrap);
    pwm_set_chan_level(slice_num, channel, (uint16_t)((wrap * duty_percent) / 100));
    pwm_set_enabled(slice_num, true);
#else
    (void)pin;
    (void)freq_hz;
    (void)duty_percent;
#endif
}

void Hal_Pwm_SetFrequency(uint8_t pin, uint32_t freq_hz, uint8_t duty_percent)
{
#ifdef PICO_ON_DEVICE
    if (freq_hz == 0U)
    {
        Hal_Pwm_Stop(pin);
        return;
    }
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t divider = 16;
    uint32_t wrap = (clock_freq / (divider * freq_hz)) - 1;

    pwm_set_wrap(slice_num, (uint16_t)wrap);
    pwm_set_chan_level(slice_num, channel, (uint16_t)((wrap * duty_percent) / 100));
    pwm_set_enabled(slice_num, true);
#else
    (void)pin;
    (void)freq_hz;
    (void)duty_percent;
#endif
}

void Hal_Pwm_Stop(uint8_t pin)
{
#ifdef PICO_ON_DEVICE
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice_num, false);
#else
    (void)pin;
#endif
}
