#include "Vl53l1x.h"
#include "Vl53l1x_Cfg.h"
#include "hal/Hal_I2c.h"
#include <stddef.h>
#include <stdio.h>

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#endif

/*
 * \file Vl53l1x.c
 * \brief Dedicated ST VL53L1X Time-of-Flight Driver Implementation
 * Compliance: Barr-C:2018
 */

#define VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR     (0x0086U)
#define VL53L1X_REG_SYSTEM_MODE_START          (0x0087U)
#define VL53L1X_REG_RESULT_RANGE_STATUS        (0x0089U)
#define VL53L1X_REG_RESULT_DISTANCE_MM         (0x0096U)
#define VL53L1X_REG_FIRMWARE_SYSTEM_STATUS     (0x00E5U)
#define VL53L1X_REG_MODEL_ID                   (0x010FU)
#define VL53L1X_NO_TARGET                      ((uint16_t)0xFFFF)

static const uint8_t default_config[91] = {
    0x00, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02,
    0x0a, 0x21, 0x04, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
    0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00
};

bool Vl53l1x_Init(const Vl53l1x_Config_t * const p_cfg)
{
    if (p_cfg == NULL)
    {
        return false;
    }

    uint8_t model_id[2] = {0U};
    /* Ping device Model ID (0x010F) using 16-bit register address */
    if (!Hal_I2c_ReadRegs16(p_cfg->i2c_port, p_cfg->dev_addr, VL53L1X_REG_MODEL_ID, model_id, 2U))
    {
        printf("[VL53L1X ERROR] Failed to read Model ID at 0x010F (Port %d, Addr 0x%02X)\n",
               p_cfg->i2c_port, p_cfg->dev_addr);
        return false;
    }

    printf("[VL53L1X] Hardware Model ID: 0x%02X%02X\n", model_id[0], model_id[1]);

    /* Check firmware boot status at 0x00E5 */
    uint8_t boot_status = 0U;
    uint32_t retries = 50U;
    while (retries > 0U)
    {
        if (Hal_I2c_ReadRegs16(p_cfg->i2c_port, p_cfg->dev_addr, VL53L1X_REG_FIRMWARE_SYSTEM_STATUS, &boot_status, 1U))
        {
            if ((boot_status & 0x01U) != 0U)
            {
                break;
            }
        }
#ifdef PICO_ON_DEVICE
        sleep_ms(2);
#endif
        retries--;
    }

    /* Load 91-byte default ST configuration blob starting at register 0x002D */
    for (uint16_t i = 0U; i < (uint16_t)sizeof(default_config); i++)
    {
        (void)Hal_I2c_WriteReg16(p_cfg->i2c_port, p_cfg->dev_addr, (uint16_t)(0x002DU + i), default_config[i]);
    }

    /* Clear interrupt flag */
    (void)Hal_I2c_WriteReg16(p_cfg->i2c_port, p_cfg->dev_addr, VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);

    /* Start continuous ranging mode (write 0x40 to SYSTEM__MODE_START 0x0087) */
    if (!Hal_I2c_WriteReg16(p_cfg->i2c_port, p_cfg->dev_addr, VL53L1X_REG_SYSTEM_MODE_START, 0x40U))
    {
        printf("[VL53L1X ERROR] Failed to write 0x40 to SYSTEM__MODE_START 0x0087\n");
        return false;
    }

    printf("[VL53L1X] Config loaded & continuous laser ranging started!\n");
    return true;
}

bool Vl53l1x_ReadDistanceMm(const Vl53l1x_Config_t * const p_cfg, uint16_t * const p_dist_mm)
{
    if ((p_cfg == NULL) || (p_dist_mm == NULL))
    {
        return false;
    }

    uint8_t buf[2] = {0U};
    /* Read distance result register 0x0096 */
    if (Hal_I2c_ReadRegs16(p_cfg->i2c_port, p_cfg->dev_addr, VL53L1X_REG_RESULT_DISTANCE_MM, buf, 2U))
    {
        uint16_t dist = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);

        /* Clear interrupt flag */
        (void)Hal_I2c_WriteReg16(p_cfg->i2c_port, p_cfg->dev_addr, VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);

        if ((dist > 20U) && (dist < 4000U))
        {
            *p_dist_mm = dist;
            return true;
        }
        else if ((dist > 0U) && (dist <= 20U))
        {
            *p_dist_mm = dist;
            return true;
        }
        else
        {
            *p_dist_mm = VL53L1X_NO_TARGET;
            return true;
        }
    }

    return false;
}
