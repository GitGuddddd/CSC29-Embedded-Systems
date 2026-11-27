#ifndef GLASSES_BOARD_H
#define GLASSES_BOARD_H

#include <stdint.h>

/*
 * \file Glasses_Board.h
 * \brief Hardware Pinout and Bus mapping for Smart Glasses (Pico 2)
 * Compliance: Barr-C:2018
 */

#define GLASSES_UART_RADAR_PORT   (0U)
#define GLASSES_PIN_UART0_TX      (0U)
#define GLASSES_PIN_UART0_RX      (1U)

#define GLASSES_I2C_IMU_PORT      (1U)
#define GLASSES_PIN_I2C1_SDA      (2U)
#define GLASSES_PIN_I2C1_SCL      (3U)

#define GLASSES_PIN_BUZZER_PWM    (18U)
#define GLASSES_BUZZER_PWM_SLICE  (1U)
#define GLASSES_BUZZER_PWM_CHAN   (0U)

#define GLASSES_PIN_STATUS_LED    (10U)
#define GLASSES_PIN_ACK_BTN       (20U)

#endif /* GLASSES_BOARD_H */
