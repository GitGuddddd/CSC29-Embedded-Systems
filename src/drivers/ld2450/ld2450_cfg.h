/** @file ld2450_cfg.h
 *
 * @brief Compile-time settings of the LD2450 driver.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#ifndef LD2450_CFG_H
#define LD2450_CFG_H

// Line speed fixed by the sensor firmware.
#define LD2450_BAUD_RATE             (256000u)

// Receive stream buffer. At 256000 baud the 32-byte hardware FIFO fills in
// about 1.25 ms, so the interrupt moves bytes out at once (NFR7). 512 bytes
// hold about 17 frames.
#define LD2450_RX_BUFFER_BYTES       (512u)

// Largest burst the interrupt copies out of the FIFO in one go.
#define LD2450_ISR_CHUNK_BYTES       (32u)

// Time allowed for the rest of a frame after its header was found.
#define LD2450_BODY_TIMEOUT_MS       (100u)

// Time allowed for the sensor to acknowledge one config command.
#define LD2450_ACK_TIMEOUT_MS        (200u)

#endif /* LD2450_CFG_H */

/*** end of file ***/
