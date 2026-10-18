#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Hal_Gpio.h
 * \brief Hardware Abstraction Layer for RP2040 GPIOs
 * Compliance: Barr-C:2018
 */

typedef enum
{
    HAL_GPIO_DIR_IN = 0U,
    HAL_GPIO_DIR_OUT
} Hal_GpioDir_t;

void Hal_Gpio_Init(uint8_t pin, Hal_GpioDir_t dir, bool pull_up);
void Hal_Gpio_Write(uint8_t pin, bool level);
bool Hal_Gpio_Read(uint8_t pin);
void Hal_Gpio_Toggle(uint8_t pin);

#endif /* HAL_GPIO_H */
