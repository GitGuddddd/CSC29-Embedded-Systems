#include "Hal_I2c.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#endif

/*
 * \file Hal_I2c.c
 * \brief Hardware Abstraction Layer for RP2040 I2C peripherals (I2C0, I2C1)
 * Compliance: Barr-C:2018
 */

void Hal_I2c_Init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint32_t baudrate_hz)
{
#ifdef PICO_ON_DEVICE
    i2c_inst_t * p_i2c = (port == 0U) ? i2c0 : i2c1;
    i2c_init(p_i2c, baudrate_hz);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
#else
    (void)port;
    (void)sda_pin;
    (void)scl_pin;
    (void)baudrate_hz;
#endif
}

void Hal_I2c_BusReset(uint8_t sda_pin, uint8_t scl_pin)
{
#ifdef PICO_ON_DEVICE
    gpio_set_function(sda_pin, GPIO_FUNC_SIO);
    gpio_set_function(scl_pin, GPIO_FUNC_SIO);
    gpio_set_dir(sda_pin, GPIO_IN);
    gpio_set_dir(scl_pin, GPIO_OUT);

    for (uint8_t i = 0U; i < 9U; i++)
    {
        gpio_put(scl_pin, 0);
        sleep_us(5);
        gpio_put(scl_pin, 1);
        sleep_us(5);
        if (gpio_get(sda_pin) != 0)
        {
            break;
        }
    }

    gpio_set_dir(sda_pin, GPIO_OUT);
    gpio_put(sda_pin, 0);
    sleep_us(5);
    gpio_put(scl_pin, 1);
    sleep_us(5);
    gpio_put(sda_pin, 1);
    sleep_us(5);
#else
    (void)sda_pin;
    (void)scl_pin;
#endif
}

bool Hal_I2c_WriteReg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
#ifdef PICO_ON_DEVICE
    i2c_inst_t * p_i2c = (port == 0U) ? i2c0 : i2c1;
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write_timeout_us(p_i2c, addr, buf, 2, false, HAL_I2C_TIMEOUT_US);
    return (ret == 2);
#else
    (void)port;
    (void)addr;
    (void)reg;
    (void)val;
    return true;
#endif
}

bool Hal_I2c_ReadRegs(uint8_t port, uint8_t addr, uint8_t reg, uint8_t * const p_dst, uint32_t len)
{
#ifdef PICO_ON_DEVICE
    i2c_inst_t * p_i2c = (port == 0U) ? i2c0 : i2c1;
    int ret = i2c_write_timeout_us(p_i2c, addr, &reg, 1, true, HAL_I2C_TIMEOUT_US);
    if (ret != 1)
    {
        return false;
    }
    ret = i2c_read_timeout_us(p_i2c, addr, p_dst, len, false, HAL_I2C_TIMEOUT_US);
    return (ret == (int)len);
#else
    (void)port;
    (void)addr;
    (void)reg;
    (void)p_dst;
    (void)len;
    return true;
#endif
}

bool Hal_I2c_WriteReg16(uint8_t port, uint8_t addr, uint16_t reg, uint8_t val)
{
#ifdef PICO_ON_DEVICE
    i2c_inst_t * p_i2c = (port == 0U) ? i2c0 : i2c1;
    uint8_t buf[3] = { (uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), val };
    int ret = i2c_write_timeout_us(p_i2c, addr, buf, 3, false, HAL_I2C_TIMEOUT_US);
    return (ret == 3);
#else
    (void)port;
    (void)addr;
    (void)reg;
    (void)val;
    return true;
#endif
}

bool Hal_I2c_ReadRegs16(uint8_t port, uint8_t addr, uint16_t reg, uint8_t * const p_dst, uint32_t len)
{
#ifdef PICO_ON_DEVICE
    i2c_inst_t * p_i2c = (port == 0U) ? i2c0 : i2c1;
    uint8_t reg_buf[2] = { (uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF) };
    int ret = i2c_write_timeout_us(p_i2c, addr, reg_buf, 2, true, HAL_I2C_TIMEOUT_US);
    if (ret != 2)
    {
        return false;
    }
    ret = i2c_read_timeout_us(p_i2c, addr, p_dst, len, false, HAL_I2C_TIMEOUT_US);
    return (ret == (int)len);
#else
    (void)port;
    (void)addr;
    (void)reg;
    (void)p_dst;
    (void)len;
    return true;
#endif
}
