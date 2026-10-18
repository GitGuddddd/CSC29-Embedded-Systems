/** @file hal_uart.c
 *
 * @brief UART access for uT-Kernel 3.0 on the RP2040 (direct registers).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The kernel's own serial driver is switched off (DEVCNF_USE_SER 0 in
 * config/config_device.h) so that this module owns the UART interrupt.
 * The receive FIFO is enabled and the receive-timeout interrupt drains a
 * partly filled FIFO, so a 30-byte radar frame never waits for more bytes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tk_include.h"

#include "hal_uart.h"

// PL011 UART registers (RP2040 datasheet, section 4.2).
#define HAL_UART0_BASE         (0x40034000UL)
#define HAL_UART1_BASE         (0x40038000UL)
#define HAL_UART_DR            (0x000UL)
#define HAL_UART_RSR           (0x004UL)
#define HAL_UART_FR            (0x018UL)
#define HAL_UART_IBRD          (0x024UL)
#define HAL_UART_FBRD          (0x028UL)
#define HAL_UART_LCR_H         (0x02cUL)
#define HAL_UART_CR            (0x030UL)
#define HAL_UART_IFLS          (0x034UL)
#define HAL_UART_IMSC          (0x038UL)
#define HAL_UART_ICR           (0x044UL)

#define HAL_UART_FR_TXFF       (1UL << 5)
#define HAL_UART_FR_RXFE       (1UL << 4)
#define HAL_UART_RSR_OE        (1UL << 3)
#define HAL_UART_LCR_H_FEN     (1UL << 4)
#define HAL_UART_LCR_H_8BIT    (3UL << 5)
#define HAL_UART_CR_ENABLE     ((1UL << 0) | (1UL << 8) | (1UL << 9))
#define HAL_UART_IMSC_RX_RT    ((1UL << 4) | (1UL << 6))
#define HAL_UART_ICR_ALL       (0x7FFUL)

// Interrupt FIFO level: 0 = trigger at 1/8 full (4 bytes of 32).
#define HAL_UART_IFLS_RX_1_8   (0UL)

// Interrupt numbers and priority (RP2040 NVIC; level 2 as the kernel uses).
#define HAL_UART0_INTNO        (20u)
#define HAL_UART1_INTNO        (21u)
#define HAL_UART_INT_LEVEL     (2)

// Pin function select value for UART (RP2040 datasheet, table 2.19.2).
#define HAL_GPIO_FUNC_UART     (2UL)

// Clock that feeds the UART (set by the kernel start-up).
#define HAL_UART_CLK_HZ        (125000000UL)

static hal_uart_rx_handler_t g_rx_handler[2] = { NULL, NULL };

static UW hal_uart_base (hal_uart_id_t id);
static void hal_uart_isr_uart0 (UINT intno);
static void hal_uart_isr_uart1 (UINT intno);
static void hal_uart_isr (hal_uart_id_t id, UINT intno);

/*!
 * @brief Set up a UART: pins, baud rate, 8N1, FIFO on, no flow control.
 *
 * NOTE: The kernel has already taken the UART block out of reset.
 *
 * @param[in] id         UART instance.
 * @param[in] baud_rate  Line speed in bits per second.
 * @param[in] tx_pin     GPIO number used for TX.
 * @param[in] rx_pin     GPIO number used for RX.
 *
 * @return true when the UART was initialised.
 */
bool
hal_uart_init (hal_uart_id_t id, uint32_t baud_rate, uint8_t tx_pin,
               uint8_t rx_pin)
{
    UW base = hal_uart_base(id);
    UW divisor = (8UL * HAL_UART_CLK_HZ) / baud_rate;
    UW integer_part = divisor >> 7;
    UW fraction_part = 0UL;

    if (0UL == integer_part)
    {
        integer_part = 1UL;
    }
    else
    {
        fraction_part = ((divisor & 0x7fUL) + 1UL) / 2UL;
    }

    // Route the two pins to the UART (the kernel leaves them alone).
    out_w(GPIO_CTRL(tx_pin), HAL_GPIO_FUNC_UART);
    out_w(GPIO_CTRL(rx_pin), HAL_GPIO_FUNC_UART);

    out_w(base + HAL_UART_CR, 0UL);
    out_w(base + HAL_UART_IBRD, integer_part);
    out_w(base + HAL_UART_FBRD, fraction_part);
    out_w(base + HAL_UART_LCR_H, HAL_UART_LCR_H_8BIT | HAL_UART_LCR_H_FEN);
    out_w(base + HAL_UART_IFLS, HAL_UART_IFLS_RX_1_8);
    out_w(base + HAL_UART_ICR, HAL_UART_ICR_ALL);
    out_w(base + HAL_UART_CR, HAL_UART_CR_ENABLE);

    return (true);
}

/*!
 * @brief Write bytes, waiting until all of them are in the TX FIFO.
 *
 * @param[in] id        UART instance.
 * @param[in] p_data    Bytes to send.
 * @param[in] length    Number of bytes.
 */
void
hal_uart_write (hal_uart_id_t id, const uint8_t * p_data, size_t length)
{
    UW base = hal_uart_base(id);

    for (size_t idx = 0u; idx < length; idx++)
    {
        while (0UL != (in_w(base + HAL_UART_FR) & HAL_UART_FR_TXFF))
        {
            // Wait for room in the TX FIFO (only used for short commands).
        }

        out_w(base + HAL_UART_DR, (UW) p_data[idx]);
    }
}

/*!
 * @brief Test whether the RX FIFO holds at least one byte.
 *
 * @param[in] id  UART instance.
 *
 * @return true when a byte can be read.
 */
bool
hal_uart_is_readable (hal_uart_id_t id)
{
    return (0UL == (in_w(hal_uart_base(id) + HAL_UART_FR) & HAL_UART_FR_RXFE));
}

/*!
 * @brief Read one byte from the RX FIFO. Call only when readable.
 *
 * @param[in] id  UART instance.
 *
 * @return The byte.
 */
uint8_t
hal_uart_read_byte (hal_uart_id_t id)
{
    return ((uint8_t) (in_w(hal_uart_base(id) + HAL_UART_DR) & 0xFFUL));
}

/*!
 * @brief Report and clear the hardware FIFO overrun flag.
 *
 * @param[in] id  UART instance.
 *
 * @return true when the FIFO had overrun since the last call.
 */
bool
hal_uart_take_overrun (hal_uart_id_t id)
{
    UW base = hal_uart_base(id);
    bool b_overrun = (0UL != (in_w(base + HAL_UART_RSR) & HAL_UART_RSR_OE));

    if (b_overrun)
    {
        // Any write to the status register clears all receive errors.
        out_w(base + HAL_UART_RSR, 0UL);
    }

    return (b_overrun);
}

/*!
 * @brief Install the receive interrupt handler and enable the interrupt.
 *
 * @param[in] id         UART instance.
 * @param[in] p_handler  Function called from the interrupt.
 */
void
hal_uart_set_rx_handler (hal_uart_id_t id, hal_uart_rx_handler_t p_handler)
{
    UINT intno = (HAL_UART_0 == id) ? HAL_UART0_INTNO : HAL_UART1_INTNO;
    T_DINT dint = { 0 };

    g_rx_handler[id] = p_handler;

    dint.intatr = TA_HLNG;
    dint.inthdr = (HAL_UART_0 == id) ? (FP) hal_uart_isr_uart0
                                     : (FP) hal_uart_isr_uart1;

    if (E_OK == tk_def_int(intno, &dint))
    {
        ClearInt(intno);
        EnableInt(intno, HAL_UART_INT_LEVEL);
        out_w(hal_uart_base(id) + HAL_UART_IMSC, HAL_UART_IMSC_RX_RT);
    }
}

/*!
 * @brief Base address of a UART block.
 *
 * @param[in] id  UART instance.
 *
 * @return The register base address.
 */
static UW
hal_uart_base (hal_uart_id_t id)
{
    return ((HAL_UART_0 == id) ? HAL_UART0_BASE : HAL_UART1_BASE);
}

/*!
 * @brief Kernel interrupt entry for UART0.
 *
 * @param[in] intno  Interrupt number.
 */
static void
hal_uart_isr_uart0 (UINT intno)
{
    hal_uart_isr(HAL_UART_0, intno);
}

/*!
 * @brief Kernel interrupt entry for UART1.
 *
 * @param[in] intno  Interrupt number.
 */
static void
hal_uart_isr_uart1 (UINT intno)
{
    hal_uart_isr(HAL_UART_1, intno);
}

/*!
 * @brief Clear the interrupt source and call the registered handler.
 *
 * WARNING: Runs in interrupt context.
 *
 * @param[in] id     UART instance.
 * @param[in] intno  Interrupt number.
 */
static void
hal_uart_isr (hal_uart_id_t id, UINT intno)
{
    // The handler empties the FIFO, which clears the receive interrupt;
    // the timeout interrupt is cleared through ICR.
    out_w(hal_uart_base(id) + HAL_UART_ICR, HAL_UART_ICR_ALL);
    ClearInt(intno);

    if (NULL != g_rx_handler[id])
    {
        g_rx_handler[id]();
    }
}

/*** end of file ***/
