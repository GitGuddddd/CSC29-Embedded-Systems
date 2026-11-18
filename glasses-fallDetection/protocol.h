/**
 * @file    protocol.h
 * @brief   Wire format of the status message sent from the glasses node to
 *          the cane node. Shared by both nodes.
 *
 * The format is built with masks and shifts only. Bit-fields are not used
 * on the wire, because the compiler chooses their bit order.
 *
 * Layout of the 2-byte payload:
 *
 *   byte 0: bits 7..5 message type, bits 4..0 fault flags
 *   byte 1: bits 3..0 fall count, bits 7..4 reserved (sent as zero)
 *
 * NOTE: This layout is a draft. The message types, the fault flag values
 * and the meaning of the fall count must be agreed with the cane node.
 * TODO: Confirm that the payload fits the 5-byte connectionless broadcast
 * packet of the cane node.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/* Number of bytes in an encoded payload. */
#define PROTOCOL_PAYLOAD_BYTES  2u

/* Byte 0: message type in bits 7..5. */
#define PROTOCOL_TYPE_SHIFT     5u
#define PROTOCOL_TYPE_MASK      0x07u

/* Byte 0: fault flags in bits 4..0. */
#define PROTOCOL_FAULT_SHIFT    0u
#define PROTOCOL_FAULT_MASK     0x1Fu

/* Byte 1: fall count in bits 3..0. */
#define PROTOCOL_COUNT_SHIFT    0u
#define PROTOCOL_COUNT_MASK     0x0Fu

/* Fault flag bits. They match the values of alert_fault_t in alert.h. */
#define PROTOCOL_FAULT_IMU       0x01u
#define PROTOCOL_FAULT_RADAR     0x02u
#define PROTOCOL_FAULT_TOF       0x04u
#define PROTOCOL_FAULT_BLE_LOSS  0x08u

/* Message types. The type field is 3 bits wide, so 0 to 7 are possible. */
#define PROTOCOL_MSG_STATUS      0u
#define PROTOCOL_MSG_FALL_EVENT  1u

/* A decoded message. Each field holds only as many bits as its wire width. */
typedef struct
{
    uint8_t msg_type;      /* PROTOCOL_MSG_ value, 3 bits on the wire. */
    uint8_t fault_flags;   /* PROTOCOL_FAULT_ bits, 5 bits on the wire. */
    uint8_t fall_count;    /* Number of falls, 4 bits on the wire. */
} protocol_msg_t;

/**
 * Packs a message into the wire format.
 * Returns false, and writes nothing, if a pointer is NULL, the buffer is
 * shorter than PROTOCOL_PAYLOAD_BYTES, or a field is too large for its width.
 */
bool protocol_encode (const protocol_msg_t * p_msg,
                      uint8_t              * p_buf,
                      uint32_t               buf_len);

/**
 * Unpacks a message from the wire format.
 * Returns false, and writes nothing, if a pointer is NULL or the buffer is
 * shorter than PROTOCOL_PAYLOAD_BYTES. The reserved bits are ignored.
 */
bool protocol_decode (const uint8_t  * p_buf,
                      uint32_t         buf_len,
                      protocol_msg_t * p_msg);

#endif /* PROTOCOL_H */

/* end of protocol.h */

