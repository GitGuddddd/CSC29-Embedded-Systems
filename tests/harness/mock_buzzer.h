/** @file mock_buzzer.h
 * @brief Access to the host-test buzzer. COPYRIGHT NOTICE: (c) 2026 Group 29. */
#ifndef MOCK_BUZZER_H
#define MOCK_BUZZER_H

#include <stdint.h>

void mock_buzzer_reset (void);
uint32_t mock_buzzer_last_tone_hz (void);
uint32_t mock_buzzer_tone_calls (void);

#endif /* MOCK_BUZZER_H */

/*** end of file ***/
