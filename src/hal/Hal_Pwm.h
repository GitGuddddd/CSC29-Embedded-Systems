#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Hal_Pwm.h
 * \brief Hardware Abstraction Layer for RP2040 PWM slices
 * Compliance: Barr-C:2018
 */

void Hal_Pwm_Init(uint8_t pin, uint32_t freq_hz, uint8_t duty_percent);
void Hal_Pwm_SetFrequency(uint8_t pin, uint32_t freq_hz, uint8_t duty_percent);
void Hal_Pwm_Stop(uint8_t pin);

#endif /* HAL_PWM_H */
