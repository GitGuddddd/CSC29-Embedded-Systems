#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Hal_I2c.h
 * \brief Hardware Abstraction Layer for RP2040 I2C peripherals (I2C0, I2C1)
 * Compliance: Barr-C:2018
 */

#define HAL_I2C_TIMEOUT_US (5000U)

void Hal_I2c_Init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint32_t baudrate_hz);
void Hal_I2c_BusReset(uint8_t sda_pin, uint8_t scl_pin);
bool Hal_I2c_WriteReg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t val);
bool Hal_I2c_ReadRegs(uint8_t port, uint8_t addr, uint8_t reg, uint8_t * const p_dst, uint32_t len);

bool Hal_I2c_WriteReg16(uint8_t port, uint8_t addr, uint16_t reg, uint8_t val);
bool Hal_I2c_ReadRegs16(uint8_t port, uint8_t addr, uint16_t reg, uint8_t * const p_dst, uint32_t len);

#endif /* HAL_I2C_H */
