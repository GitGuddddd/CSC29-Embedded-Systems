#include "Hal_I2c.h"
#include <stddef.h>

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
    gpio_set_dir(scl_pin, GPIO_IN);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    gpio_put(sda_pin, 0);
    gpio_put(scl_pin, 0);

    /* Open-drain bus clear: drive LOW or release, never drive a slave's line HIGH. */
    for (uint8_t i = 0U; (i < 9U) && (gpio_get(sda_pin) == 0); i++)
    {
        gpio_set_dir(scl_pin, GPIO_OUT);
        sleep_us(5);
        gpio_set_dir(scl_pin, GPIO_IN);
        uint32_t start_us = time_us_32();
        while (gpio_get(scl_pin) == 0)
        {
            if ((uint32_t)(time_us_32() - start_us) >= HAL_I2C_TIMEOUT_US)
            {
                return;
            }
        }
        sleep_us(5);
    }

    /* STOP: release SDA only while SCL is observed HIGH. Caller restores I2C mux. */
    if (gpio_get(scl_pin) == 0)
    {
        return;
    }
    gpio_set_dir(sda_pin, GPIO_OUT);
    sleep_us(5);
    gpio_set_dir(sda_pin, GPIO_IN);
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

bool Hal_I2c_WriteRegs16(uint8_t port, uint8_t addr, uint16_t reg, const uint8_t * p_src, uint32_t len)
{
    if ((port > 1U) || (addr > 0x7FU) || (p_src == NULL) || (len == 0U) ||
        (len > HAL_I2C_MAX_WRITE_BYTES) || (len > (0x10000U - (uint32_t)reg)))
    {
        return false;
    }
#ifdef PICO_ON_DEVICE
    i2c_inst_t * p_i2c = (port == 0U) ? i2c0 : i2c1;
    uint8_t buf[HAL_I2C_MAX_WRITE_BYTES + 2U];
    buf[0] = (uint8_t)((reg >> 8) & 0xFFU);
    buf[1] = (uint8_t)(reg & 0xFFU);
    for (uint32_t i = 0U; i < len; i++)
    {
        buf[2U + i] = p_src[i];
    }
    int ret = i2c_write_timeout_us(p_i2c, addr, buf, len + 2U, false, HAL_I2C_TIMEOUT_US);
    return (ret == (int)(len + 2U));
#else
    (void)port;
    (void)addr;
    (void)reg;
    (void)p_src;
    (void)len;
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
