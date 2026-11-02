#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include "pico/types.h"

void buzzer_init(uint pin);
void buzzer_tone(uint32_t freq_hz);   // 0 = silent

#endif