/** @file ld2450_lcfg.c
 *
 * @brief Link-time configuration of the LD2450 driver.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The command frames follow the LD2450 protocol: header FD FC FB FA, length,
 * command word, value, footer 04 03 02 01.
 */

#include <stdint.h>

#include "board_glasses.h"
#include "ld2450.h"

// Enable configuration mode (command 00FF, value 0001).
static const uint8_t g_cmd_enable_config[] =
{
    0xFDu, 0xFCu, 0xFBu, 0xFAu, 0x04u, 0x00u, 0xFFu, 0x00u,
    0x01u, 0x00u, 0x04u, 0x03u, 0x02u, 0x01u
};

// Switch to multi-target tracking (command 0090).
static const uint8_t g_cmd_set_multi_target[] =
{
    0xFDu, 0xFCu, 0xFBu, 0xFAu, 0x02u, 0x00u, 0x90u, 0x00u,
    0x04u, 0x03u, 0x02u, 0x01u
};

// Leave configuration mode (command 00FE).
static const uint8_t g_cmd_end_config[] =
{
    0xFDu, 0xFCu, 0xFBu, 0xFAu, 0x02u, 0x00u, 0xFEu, 0x00u,
    0x04u, 0x03u, 0x02u, 0x01u
};

// NOTE: The sizeof casts below are safe because every command is shorter
// than 255 bytes, so the length always fits in a uint8_t.
const ld2450_lcfg_t g_ld2450_lcfg =
{
    .uart_id          = HAL_UART_0,
    .tx_pin           = BOARD_RADAR_UART_TX_PIN,
    .rx_pin           = BOARD_RADAR_UART_RX_PIN,
    .enable_config    = { g_cmd_enable_config,
                          (uint8_t) sizeof(g_cmd_enable_config) },
    .set_multi_target = { g_cmd_set_multi_target,
                          (uint8_t) sizeof(g_cmd_set_multi_target) },
    .end_config       = { g_cmd_end_config,
                          (uint8_t) sizeof(g_cmd_end_config) }
};

/*** end of file ***/
