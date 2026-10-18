#ifndef CANE_BOARD_H
#define CANE_BOARD_H

#include <stdint.h>

/*
 * \file Cane_Board.h
 * \brief Hardware Pinout and Bus mapping for Smart Walking Cane (Pico 1)
 * Compliance: Barr-C:2018
 */

#define CANE_I2C_IMU_PORT         (0U)
#define CANE_PIN_I2C0_SDA         (4U)
#define CANE_PIN_I2C0_SCL         (5U)

#define CANE_I2C_TOF_PORT         (1U)
#define CANE_PIN_I2C1_SDA         (6U)
#define CANE_PIN_I2C1_SCL         (7U)

#define CANE_PIN_TOUCH_IN         (14U)
#define CANE_PIN_VIBE_MOTOR       (15U)

#endif /* CANE_BOARD_H */
