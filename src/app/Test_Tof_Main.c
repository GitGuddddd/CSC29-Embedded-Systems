#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

/*
 * \file Test_Tof_Main.c
 * \brief Full ST API Register Sequence for VL53L0XV2 Breakout Module
 */

static void i2c_bus_init(i2c_inst_t *i2c, uint sda, uint scl)
{
    i2c_init(i2c, 100 * 1000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

static bool read_regs8(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *dst, size_t len)
{
    int ret = i2c_write_timeout_us(i2c, addr, &reg, 1, true, 10000);
    if (ret != 1) return false;
    ret = i2c_read_timeout_us(i2c, addr, dst, len, false, 10000);
    return (ret == (int)len);
}

static bool write_reg8(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write_timeout_us(i2c, addr, buf, 2, false, 10000);
    return (ret == 2);
}

static void vl53l0x_full_init(i2c_inst_t *i2c, uint8_t addr)
{
    /* 8051 MCU & I2C Standard Tuning Registers */
    write_reg8(i2c, addr, 0x88, 0x00);
    write_reg8(i2c, addr, 0x80, 0x01);
    write_reg8(i2c, addr, 0xFF, 0x01);
    write_reg8(i2c, addr, 0x00, 0x00);
    write_reg8(i2c, addr, 0x91, 0x3C);
    write_reg8(i2c, addr, 0x00, 0x01);
    write_reg8(i2c, addr, 0xFF, 0x00);
    write_reg8(i2c, addr, 0x80, 0x00);

    /* Disable HIGH_SPEED mode & tuning settings */
    write_reg8(i2c, addr, 0x01, 0xFF);
    write_reg8(i2c, addr, 0x80, 0x01);
    write_reg8(i2c, addr, 0x85, 0x01);
    write_reg8(i2c, addr, 0xFF, 0x01);
    write_reg8(i2c, addr, 0x00, 0x00);
    write_reg8(i2c, addr, 0x84, 0x00);
    write_reg8(i2c, addr, 0x83, 0x00);
    write_reg8(i2c, addr, 0xFF, 0x00);
    write_reg8(i2c, addr, 0x80, 0x00);

    /* Interrupt Config */
    write_reg8(i2c, addr, 0x0A, 0x04);
    write_reg8(i2c, addr, 0x0B, 0x01);

    /* Enable all sequence steps: VHV, Phase, DSS, Pre-range, Final-range */
    write_reg8(i2c, addr, 0x01, 0xE8);

    /* Set Signal Rate Limit = 0.25 MCPS (0x00, 0x02) */
    write_reg8(i2c, addr, 0x44, 0x00);
    write_reg8(i2c, addr, 0x45, 0x02);

    /* Set Pre-Range Pulse Period = 14, Final Range Pulse Period = 10 */
    write_reg8(i2c, addr, 0x50, 0x0E);
    write_reg8(i2c, addr, 0x70, 0x0A);

    /* Perform VHV Calibration */
    write_reg8(i2c, addr, 0x00, 0x01);
    sleep_ms(20);
    write_reg8(i2c, addr, 0x00, 0x00);

    /* Start Continuous Back-to-Back Ranging */
    write_reg8(i2c, addr, 0x00, 0x02);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n==================================================\n");
    printf("   FULL ST API TUNED VL53L0XV2 MEASUREMENT TEST   \n");
    printf("==================================================\n");

    i2c_bus_init(i2c1, 6, 7);

    uint8_t model_id = 0;
    if (read_regs8(i2c1, 0x29, 0xC0, &model_id, 1))
    {
        printf("[HARDWARE DETECTED] VL53L0XV2 Model ID: 0x%02X on I2C1 (GP6/GP7)\n", model_id);
    }

    vl53l0x_full_init(i2c1, 0x29);
    printf("Tuning & continuous ranging initialized!\n");

    uint32_t sample = 0;
    while (true)
    {
        sample++;

        /* Trigger measurement strobe */
        write_reg8(i2c1, 0x29, 0x00, 0x01);
        sleep_ms(50);

        uint8_t range_data[12] = {0};
        if (read_regs8(i2c1, 0x29, 0x14, range_data, 12))
        {
            uint16_t dist_mm = (uint16_t)(((uint16_t)range_data[10] << 8) | range_data[11]);
            uint8_t device_status = range_data[0];

            /* Clear interrupt */
            write_reg8(i2c1, 0x29, 0x0B, 0x01);

            printf("[SAMPLE #%lu] >>> DISTANCE: %4u mm (%3.1f cm) <<< [Status: 0x%02X, Bytes: 0x%02X 0x%02X]\n",
                   (unsigned long)sample, dist_mm, (float)dist_mm / 10.0f, device_status, range_data[10], range_data[11]);
        }

        sleep_ms(150);
    }

    return 0;
}
