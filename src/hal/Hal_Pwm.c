#include "Hal_Pwm.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#endif

static bool s_slice_initialized[8] = {0};
static uint8_t s_slice_duty[8] = {0};

static inline uint8_t Hal_Pwm_SliceIndex(uint8_t pin)
{
#ifdef PICO_ON_DEVICE
    return (uint8_t)(pwm_gpio_to_slice_num(pin) & 7U);
#else
    return (uint8_t)(pin & 7U);
#endif
}

void Hal_Pwm_Init(uint8_t pin, uint32_t freq_hz, uint8_t duty_percent)
{
    uint8_t slice = Hal_Pwm_SliceIndex(pin);
    s_slice_initialized[slice] = true;
    s_slice_duty[slice] = duty_percent;

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
    (void)freq_hz;
#endif
}

void Hal_Pwm_SetFrequency(uint8_t pin, uint32_t freq_hz, uint8_t duty_percent)
{
    if (freq_hz == 0U)
    {
        Hal_Pwm_Stop(pin);
        return;
    }
    uint8_t slice = Hal_Pwm_SliceIndex(pin);
    s_slice_duty[slice] = duty_percent;

#ifdef PICO_ON_DEVICE
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t divider = 16;
    uint32_t wrap = (clock_freq / (divider * freq_hz)) - 1;

    pwm_set_wrap(slice_num, (uint16_t)wrap);
    pwm_set_chan_level(slice_num, channel, (uint16_t)((wrap * duty_percent) / 100));
    pwm_set_enabled(slice_num, true);
#endif
}

void Hal_Pwm_Stop(uint8_t pin)
{
    uint8_t slice = Hal_Pwm_SliceIndex(pin);
    s_slice_duty[slice] = 0U;

#ifdef PICO_ON_DEVICE
    /*
     * Do not disable the slice: a halted slice freezes the pin at whatever level
     * it had (pico-sdk pwm_set_enabled() docs), which left the cane motor stuck on.
     * A 0% duty with the slice running drives the pin low from the next period (<= 1 ms).
     */
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice_num, channel, 0U);
#endif
}

bool Hal_Pwm_IsHealthy(uint8_t pin)
{
#ifdef PICO_ON_DEVICE
    uint8_t slice = Hal_Pwm_SliceIndex(pin);
    if (!s_slice_initialized[slice])
    {
        return false;
    }
    if ((pwm_hw->slice[slice].csr & PWM_CH0_CSR_EN_BITS) == 0U)
    {
        return false;
    }
    if ((s_slice_duty[slice] > 0U) && (pwm_hw->slice[slice].top == 0U))
    {
        return false;
    }
    return true;
#else
    (void)pin;
    return true;
#endif
}
