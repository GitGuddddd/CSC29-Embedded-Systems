#include "buzzer.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

static uint s_pin, s_slice;

void buzzer_init(uint pin) {
    s_pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    s_slice = pwm_gpio_to_slice_num(pin);

    // Divide system clock down to a 1 MHz PWM counter.
    // Then: output freq = 1,000,000 / (wrap + 1)
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    pwm_set_clkdiv(s_slice, div);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(s_slice, true);
}

void buzzer_tone(uint32_t freq_hz) {
    if (freq_hz < 20) {                     // treat as "off"
        pwm_set_gpio_level(s_pin, 0);
        return;
    }
    uint32_t wrap = 1000000u / freq_hz - 1; // 20 Hz → 49999, fits in 16 bits
    pwm_set_wrap(s_slice, (uint16_t)wrap);
    pwm_set_gpio_level(s_pin, (uint16_t)((wrap + 1) / 2));  // 50% duty = loudest
}