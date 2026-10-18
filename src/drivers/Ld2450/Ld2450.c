#include "Ld2450.h"
#include "Ld2450_Cfg.h"
#include "hal/Hal_Uart.h"
#include <string.h>

/*
 * \file Ld2450.c
 * \brief LD2450 frame parser (AA FF 03 00 ... 55 CC) per FR7
 * Compliance: Barr-C:2018
 */

extern uint8_t * Ld2450_Irq_GetBuffer(void);
extern uint32_t Ld2450_Irq_GetTail(void);
extern void Ld2450_Irq_SetTail(uint32_t tail);

bool Ld2450_Init(const Ld2450_Config_t * const p_cfg)
{
    if (p_cfg == NULL)
    {
        return false;
    }
    Hal_Uart_Init(p_cfg->uart_port, p_cfg->tx_pin, p_cfg->rx_pin, LD2450_CFG_BAUDRATE);
    return Hal_Uart_DmaInit(p_cfg->uart_port, Ld2450_Irq_GetBuffer(), LD2450_CFG_CIRC_BUF_SIZE);
}

bool Ld2450_ParseFrame(const uint8_t * const p_frame, uint32_t len, Radar_Target_t * const p_targets, uint8_t * const p_count)
{
    if ((p_frame == NULL) || (p_targets == NULL) || (p_count == NULL) || (len < 30U))
    {
        return false;
    }

    /* Check Header: AA FF 03 00 */
    if ((p_frame[0] != 0xAAU) || (p_frame[1] != 0xFFU) || (p_frame[2] != 0x03U) || (p_frame[3] != 0x00U))
    {
        return false;
    }

    /* Check Footer: 55 CC at offset 28, 29 */
    if ((p_frame[28] != 0x55U) || (p_frame[29] != 0xCCU))
    {
        return false;
    }

    uint8_t count = 0U;

    /* Up to 3 targets: 8 bytes per target starting at index 4 */
    for (uint8_t i = 0U; i < LD2450_CFG_MAX_TARGETS; i++)
    {
        uint32_t offset = 4U + (i * 8U);

        uint16_t x_raw = (uint16_t)(((uint16_t)p_frame[offset + 1U] << 8) | p_frame[offset]);
        int16_t x = (x_raw & 0x8000U) ? -(int16_t)(x_raw & 0x7FFFU) : (int16_t)(x_raw & 0x7FFFU);

        uint16_t y_raw = (uint16_t)(((uint16_t)p_frame[offset + 3U] << 8) | p_frame[offset + 2U]);
        int16_t y = (y_raw & 0x8000U) ? -(int16_t)(y_raw & 0x7FFFU) : (int16_t)(y_raw & 0x7FFFU);

        uint16_t speed_raw = (uint16_t)(((uint16_t)p_frame[offset + 5U] << 8) | p_frame[offset + 4U]);
        int16_t speed = (speed_raw & 0x8000U) ? -(int16_t)(speed_raw & 0x7FFFU) : (int16_t)(speed_raw & 0x7FFFU);

        uint16_t dist_res = (uint16_t)(((uint16_t)p_frame[offset + 7U] << 8) | p_frame[offset + 6U]);

        (void)x;
        (void)dist_res;

        if (y > 0)
        {
            p_targets[count].distance_mm = (uint16_t)y;
            p_targets[count].speed_cms = speed;
            p_targets[count].is_valid = true;
            p_targets[count].angle_deg = 0;
            count++;
        }
    }

    *p_count = count;
    return true;
}

void Ld2450_ProcessStream(Radar_Target_t * const p_targets, uint8_t * const p_count)
{
    if ((p_targets == NULL) || (p_count == NULL))
    {
        return;
    }
    *p_count = 0U;
}
