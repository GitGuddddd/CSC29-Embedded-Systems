/** @file ld2450.h
 *
 * @brief HLK-LD2450 mmWave radar driver: configuration and frame parsing.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Reference: HLK-LD2450 serial communication protocol. A frame is the header
 * AA FF 03 00, three 8-byte target blocks and the footer 55 CC. Each block
 * holds X, Y, speed and resolution as little-endian 16-bit values. X, Y and
 * speed are sign-magnitude: bit 15 set means positive.
 */

#ifndef LD2450_H
#define LD2450_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal_uart.h"
#include "rte_type.h"

/** Number of bytes that follow the 4-byte frame header. */
#define LD2450_BODY_LEN    (26u)

/** Result of one attempt to receive a frame. */
typedef enum
{
    LD2450_FRAME_OK          = 0,  /**< Valid frame decoded. */
    LD2450_FRAME_TIMEOUT     = 1,  /**< No header before the deadline. */
    LD2450_FRAME_SHORT       = 2,  /**< Header seen, body incomplete. */
    LD2450_FRAME_BAD_FOOTER  = 3   /**< Body complete, footer not 55 CC. */
} ld2450_frame_status_t;

/** A fixed byte sequence sent to the sensor. */
typedef struct
{
    const uint8_t * p_data;
    uint8_t         length;
} ld2450_command_t;

/** Link-time configuration (defined in ld2450_lcfg.c). */
typedef struct
{
    hal_uart_id_t    uart_id;
    uint8_t          tx_pin;
    uint8_t          rx_pin;
    ld2450_command_t enable_config;
    ld2450_command_t set_multi_target;
    ld2450_command_t end_config;
} ld2450_lcfg_t;

extern const ld2450_lcfg_t g_ld2450_lcfg;

bool ld2450_init (void);
bool ld2450_configure (void);
ld2450_frame_status_t ld2450_receive_frame (rte_radar_target_t * p_targets,
                                            uint32_t deadline_ms);

int16_t ld2450_decode_signed (uint8_t low, uint8_t high);
bool ld2450_decode_body (const uint8_t * p_body,
                         rte_radar_target_t * p_targets);

/** Receive-path statistics for diagnosis (all counted since start-up). */
typedef struct
{
    uint32_t overrun_count;   /**< Hardware FIFO overruns seen by the ISR. */
    uint32_t dropped_bytes;   /**< Bytes lost: stream buffer was full. */
    uint32_t byte_count;      /**< Bytes read from the FIFO. */
    uint32_t isr_count;       /**< Times the receive ISR ran. */
} ld2450_rx_stats_t;

uint32_t ld2450_get_overflow_count (void);
void ld2450_get_rx_stats (ld2450_rx_stats_t * p_stats);
uint32_t ld2450_get_bad_footer_count (void);

#endif /* LD2450_H */

/*** end of file ***/
