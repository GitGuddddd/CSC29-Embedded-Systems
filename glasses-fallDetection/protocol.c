/**
 * @file    protocol.c
 * @brief   Packs and unpacks the status message of protocol.h.
 *
 * All work is done on uint32_t values with masks and shifts, so no bitwise
 * operator is ever applied to signed data.
 */

#include "protocol.h"

#include <stddef.h>

/**
 * @brief  Packs a message into the wire format.
 * @param  p_msg    Message to pack.
 * @param  p_buf    Output buffer.
 * @param  buf_len  Size of the output buffer in bytes.
 * @return true if the message was packed, false if it was rejected.
 */
bool protocol_encode (const protocol_msg_t * p_msg,
                      uint8_t              * p_buf,
                      uint32_t               buf_len)
{
    bool     b_ok = false;
    uint32_t msg_type;
    uint32_t fault_flags;
    uint32_t fall_count;

    if ((NULL != p_msg) && (NULL != p_buf) &&
        (buf_len >= PROTOCOL_PAYLOAD_BYTES))
    {
        msg_type    = p_msg->msg_type;
        fault_flags = p_msg->fault_flags;
        fall_count  = p_msg->fall_count;

        if ((msg_type <= PROTOCOL_TYPE_MASK) &&
            (fault_flags <= PROTOCOL_FAULT_MASK) &&
            (fall_count <= PROTOCOL_COUNT_MASK))
        {
            /* Casts: after the range checks above, each result fits in
             * 8 bits (byte 0 is at most 0xFF, byte 1 is at most 0x0F). */
            p_buf[0] = (uint8_t)((msg_type << PROTOCOL_TYPE_SHIFT) |
                                 (fault_flags << PROTOCOL_FAULT_SHIFT));
            p_buf[1] = (uint8_t)(fall_count << PROTOCOL_COUNT_SHIFT);
            b_ok     = true;
        }
    }

    return b_ok;
}

/**
 * @brief  Unpacks a message from the wire format.
 * @param  p_buf    Received bytes.
 * @param  buf_len  Number of received bytes.
 * @param  p_msg    Output message.
 * @return true if the message was unpacked, false if it was rejected.
 */
bool protocol_decode (const uint8_t  * p_buf,
                      uint32_t         buf_len,
                      protocol_msg_t * p_msg)
{
    bool     b_ok = false;
    uint32_t byte0;
    uint32_t byte1;

    if ((NULL != p_buf) && (NULL != p_msg) &&
        (buf_len >= PROTOCOL_PAYLOAD_BYTES))
    {
        byte0 = p_buf[0];
        byte1 = p_buf[1];

        /* Casts: each value is masked to at most 5 bits, so it fits in
         * a uint8_t. */
        p_msg->msg_type    = (uint8_t)((byte0 >> PROTOCOL_TYPE_SHIFT) &
                                       PROTOCOL_TYPE_MASK);
        p_msg->fault_flags = (uint8_t)((byte0 >> PROTOCOL_FAULT_SHIFT) &
                                       PROTOCOL_FAULT_MASK);
        p_msg->fall_count  = (uint8_t)((byte1 >> PROTOCOL_COUNT_SHIFT) &
                                       PROTOCOL_COUNT_MASK);
        b_ok               = true;
    }

    return b_ok;
}

/* end of protocol.c */

