#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Buzzer.h
 * \brief Buzzer driver interface for pitch-variable audio alerts
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t gpio_pin;
} Buzzer_Config_t;

extern const Buzzer_Config_t Buzzer_Config_Cane;
extern const Buzzer_Config_t Buzzer_Config_Glasses;

void Buzzer_Init(const Buzzer_Config_t * const p_cfg);
void Buzzer_InjectFault(bool fault_active);
bool Buzzer_IsHealthy(void);
uint16_t Buzzer_GetActiveFrequencyHz(void);
uint8_t Buzzer_GetActiveDutyPercent(void);
void Buzzer_PlayTone(uint16_t freq_hz);
void Buzzer_PlayToneAtDuty(uint16_t freq_hz, uint8_t duty_percent);
void Buzzer_Silence(void);

#endif /* BUZZER_H */
