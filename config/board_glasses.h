/** @file board_glasses.h
 *
 * @brief Pin map of the Smart Glasses node (Pico W number 2).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#ifndef BOARD_GLASSES_H
#define BOARD_GLASSES_H

// LD2450 radar on UART0, 3V3 and a shared GND (report, Section 2).
#define BOARD_RADAR_UART_TX_PIN    (0u)    // Pico TX to LD2450 RX.
#define BOARD_RADAR_UART_RX_PIN    (1u)    // LD2450 TX to Pico RX.

// Maker Pi Pico passive buzzer. Owned by the Audio Alert task.
#define BOARD_BUZZER_PIN           (18u)

// TODO: The head IMU pins (GP2 and GP3 in the team plan) are added by the
// owner of the head-fall detection module.

#endif /* BOARD_GLASSES_H */

/*** end of file ***/
