#include "Ld2450.h"
#include "Ld2450_Cfg.h"

/*
 * \file Ld2450_Irq.c
 * \brief ISR and Circular buffer management for LD2450 UART DMA
 * Compliance: Barr-C:2018
 */

static uint8_t s_circ_buf[LD2450_CFG_CIRC_BUF_SIZE];
static uint32_t s_tail_idx = 0U;

uint8_t * Ld2450_Irq_GetBuffer(void)
{
    return s_circ_buf;
}

uint32_t Ld2450_Irq_GetTail(void)
{
    return s_tail_idx;
}

void Ld2450_Irq_SetTail(uint32_t tail)
{
    s_tail_idx = tail % LD2450_CFG_CIRC_BUF_SIZE;
}
