#include "Vl53l0x.h"
#include "Vl53l0x_Cfg.h"
#include "hal/Hal_I2c.h"
#include <stddef.h>
#include <stdio.h>

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#endif

/*
 * \file Vl53l0x.c
 * \brief VL53L0X / VL53L0XV2 Time-of-Flight Driver Implementation
 * Compliance: Barr-C:2018
 */

#define VL53L0X_REG_SYSRANGE_START                   (0x00U)
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR          (0x0BU)
#define VL53L0X_REG_RESULT_RANGE_STATUS             (0x14U)
#define VL53L0X_REG_RESULT_DISTANCE_MM              (0x1EU)
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         (0xC0U)

bool Vl53l0x_Init(const Vl53l0x_Config_t * const p_cfg)
{
    if (p_cfg == NULL)
    {
        return false;
    }

    uint8_t model_id = 0U;
    /* Read 8-bit Model ID register 0xC0 (expects 0xEE for VL53L0X) */
    if (!Hal_I2c_ReadRegs(p_cfg->i2c_port, p_cfg->dev_addr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id, 1U))
    {
        return false;
    }

    if (model_id != 0xEEU)
    {
        return false;
    }

    printf("[VL53L0X] Hardware Model ID 0xEE confirmed at Addr 0x%02X\n", p_cfg->dev_addr);

    /* VL53L0X Standard I2C Tuning & Init Sequence */
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x88U, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x80U, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0xFFU, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x00U, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x91U, 0x3CU);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x00U, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0xFFU, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x80U, 0x00U);

    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x01U, 0xFFU);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x80U, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x85U, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0xFFU, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x00U, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x84U, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x83U, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0xFFU, 0x00U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x80U, 0x00U);

    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x0AU, 0x04U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x0BU, 0x01U);
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x01U, 0xE8U);

    /* VHV Calibration cycle */
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x00U, 0x01U);
#ifdef PICO_ON_DEVICE
    sleep_ms(20);
#endif
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, 0x00U, 0x00U);

    /* Start continuous measurement mode */
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, VL53L0X_REG_SYSRANGE_START, 0x02U);
    printf("[VL53L0X] Initialized successfully & continuous ranging active!\n");
    return true;
}

bool Vl53l0x_ReadDistanceMm(const Vl53l0x_Config_t * const p_cfg, uint16_t * const p_dist_mm)
{
    if ((p_cfg == NULL) || (p_dist_mm == NULL))
    {
        return false;
    }

    /* Trigger single measurement start */
    (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, VL53L0X_REG_SYSRANGE_START, 0x01U);

    uint8_t range_data[12] = {0U};
    /* Read 12-byte result starting at reg 0x14 */
    if (Hal_I2c_ReadRegs(p_cfg->i2c_port, p_cfg->dev_addr, VL53L0X_REG_RESULT_RANGE_STATUS, range_data, 12U))
    {
        uint16_t dist = (uint16_t)(((uint16_t)range_data[10] << 8) | range_data[11]);

        /* Clear interrupt flag */
        (void)Hal_I2c_WriteReg(p_cfg->i2c_port, p_cfg->dev_addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);

        if ((dist > 20U) && (dist < 3000U))
        {
            *p_dist_mm = dist;
            return true;
        }
        else if (dist == 0U || dist <= 20U)
        {
            /* 0 mm or <= 20 mm explicitly indicates optical lens obstruction/cover */
            *p_dist_mm = 0U;
            return true;
        }
        else
        {
            /* 0xFFFF / Out-of-range indicates no return target */
            *p_dist_mm = 0xFFFFU;
            return true;
        }
    }

    return false;
}
