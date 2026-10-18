#include "Hal_Gpio.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#endif

void Hal_Gpio_Init(uint8_t pin, Hal_GpioDir_t dir, bool pull_up)
{
#ifdef PICO_ON_DEVICE
    gpio_init(pin);
    gpio_set_dir(pin, (dir == HAL_GPIO_DIR_IN) ? GPIO_IN : GPIO_OUT);
    if (pull_up)
    {
        gpio_pull_up(pin);
    }
#else
    (void)pin;
    (void)dir;
    (void)pull_up;
#endif
}

void Hal_Gpio_Write(uint8_t pin, bool level)
{
#ifdef PICO_ON_DEVICE
    gpio_put(pin, level);
#else
    (void)pin;
    (void)level;
#endif
}

bool Hal_Gpio_Read(uint8_t pin)
{
#ifdef PICO_ON_DEVICE
    return gpio_get(pin);
#else
    (void)pin;
    return false;
#endif
}

void Hal_Gpio_Toggle(uint8_t pin)
{
#ifdef PICO_ON_DEVICE
    gpio_put(pin, !gpio_get(pin));
#else
    (void)pin;
#endif
}
