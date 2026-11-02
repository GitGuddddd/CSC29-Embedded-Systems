/** @file rte_pack.c
 *
 * @brief Bit-level packing of the radar queue message (minimum-bit change).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Both sides of the queue use these functions, so nobody writes shifts by
 * hand. The message is copied whole through the kernel queue, so no shared
 * variable is read-modify-written.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte_pack.h"
#include "rte_type.h"

#define RTE_PACK_BITS_PER_BYTE    (8u)

// Field positions (bit offsets) in the 102-bit message.
#define RTE_PACK_TYPE_AT          (0u)
#define RTE_PACK_TYPE_BITS        (2u)
#define RTE_PACK_GRIP_AT          (2u)
#define RTE_PACK_GRIP_BITS        (2u)
#define RTE_PACK_TIME_AT          (4u)
#define RTE_PACK_TIME_BITS        (32u)
#define RTE_PACK_SLOT0_AT         (36u)
#define RTE_PACK_SLOT_BITS        (22u)

// Offsets inside one 22-bit slot.
#define RTE_PACK_SLOT_VALID_AT    (0u)
#define RTE_PACK_SLOT_DIST_AT     (1u)
#define RTE_PACK_SLOT_SPEED_AT    (13u)

#define RTE_PACK_CODE_RESERVED    (3u)

/*!
 * @brief Write a value into a bit field of a buffer.
 *
 * @param[in,out] p_buf  The buffer.
 * @param[in]     offset  First bit (0 = least significant bit of byte 0).
 * @param[in]     width   Number of bits, 1 to 32.
 * @param[in]     value   Value; bits above width are ignored.
 */
void
rte_pack_put_bits (uint8_t * p_buf, uint32_t offset, uint32_t width,
                   uint32_t value)
{
    for (uint32_t bit = 0u; bit < width; bit++)
    {
        uint32_t pos = offset + bit;
        uint32_t byte_idx = pos / RTE_PACK_BITS_PER_BYTE;
        uint32_t mask = 1u << (pos % RTE_PACK_BITS_PER_BYTE);

        if (0u != ((value >> bit) & 1u))
        {
            p_buf[byte_idx] = (uint8_t) (p_buf[byte_idx] | mask);
        }
        else
        {
            p_buf[byte_idx] = (uint8_t) (p_buf[byte_idx] & ~mask);
        }
    }
}

/*!
 * @brief Read a bit field from a buffer.
 *
 * @param[in] p_buf   The buffer.
 * @param[in] offset  First bit (0 = least significant bit of byte 0).
 * @param[in] width   Number of bits, 1 to 32.
 *
 * @return The value, zero-extended.
 */
uint32_t
rte_pack_get_bits (const uint8_t * p_buf, uint32_t offset, uint32_t width)
{
    uint32_t value = 0u;

    for (uint32_t bit = 0u; bit < width; bit++)
    {
        uint32_t pos = offset + bit;
        uint32_t byte_idx = pos / RTE_PACK_BITS_PER_BYTE;
        uint32_t flag = (p_buf[byte_idx] >> (pos % RTE_PACK_BITS_PER_BYTE)) &
                        1u;

        value |= (flag << bit);
    }

    return (value);
}

/*!
 * @brief Pack one target slot (22 bits). An empty slot is all zero.
 *
 * Distance is rounded down to a 2 mm step and saturates at 4095 steps. Speed
 * is clamped to the symmetric range so that negating it cannot overflow.
 */
static void
rte_pack_slot (uint8_t * p_buf, uint32_t at, const rte_radar_target_t * p_t)
{
    if (p_t->b_valid)
    {
        uint32_t steps = (uint32_t) p_t->d_mm / RTE_PACK_DISTANCE_STEP_MM;
        int32_t speed = p_t->speed_cms;

        if (steps > RTE_PACK_DISTANCE_MAX)
        {
            steps = RTE_PACK_DISTANCE_MAX;
        }

        if (speed > RTE_PACK_SPEED_MAX_CMS)
        {
            speed = RTE_PACK_SPEED_MAX_CMS;
        }
        else if (speed < -RTE_PACK_SPEED_MAX_CMS)
        {
            speed = -RTE_PACK_SPEED_MAX_CMS;
        }
        else
        {
            // In range, nothing to clamp.
        }

        rte_pack_put_bits(p_buf, at + RTE_PACK_SLOT_VALID_AT, 1u, 1u);
        rte_pack_put_bits(p_buf, at + RTE_PACK_SLOT_DIST_AT,
                     RTE_PACK_DISTANCE_BITS, steps);

        // The cast keeps the two's complement bits; rte_pack_put_bits drops the
        // bits above the field width.
        rte_pack_put_bits(p_buf, at + RTE_PACK_SLOT_SPEED_AT,
                     RTE_PACK_SPEED_BITS, (uint32_t) speed);
    }
}

/*!
 * @brief Unpack one target slot (22 bits).
 */
static void
rte_unpack_slot (const uint8_t * p_buf, uint32_t at, rte_radar_target_t * p_t)
{
    uint32_t valid = rte_pack_get_bits(p_buf, at + RTE_PACK_SLOT_VALID_AT, 1u);

    p_t->b_valid = (0u != valid);
    p_t->x_mm = 0;
    p_t->y_mm = 0;
    p_t->d_mm = 0u;
    p_t->speed_cms = 0;

    if (p_t->b_valid)
    {
        uint32_t steps = rte_pack_get_bits(p_buf, at + RTE_PACK_SLOT_DIST_AT,
                                      RTE_PACK_DISTANCE_BITS);
        uint32_t raw = rte_pack_get_bits(p_buf, at + RTE_PACK_SLOT_SPEED_AT,
                                    RTE_PACK_SPEED_BITS);
        int32_t speed = (int32_t) raw;

        // Sign-extend the two's complement speed field.
        if (0u != (raw & (1u << (RTE_PACK_SPEED_BITS - 1u))))
        {
            speed -= (int32_t) (1u << RTE_PACK_SPEED_BITS);
        }

        // At most 4095 * 2 = 8190, which fits uint16_t.
        p_t->d_mm = (uint16_t) (steps * RTE_PACK_DISTANCE_STEP_MM);
        p_t->speed_cms = (int16_t) speed;
    }
}

/*!
 * @brief Pack a radar queue message (13 bytes, 102 bits used).
 *
 * @param[in]  p_data  The message fields.
 * @param[out] p_msg   The packed bytes (spare bits are 0).
 */
void
rte_pack_radar (const rte_radar_data_t * p_data, rte_radar_msg_t * p_msg)
{
    for (size_t idx = 0u; idx < RTE_RADAR_MSG_BYTES; idx++)
    {
        p_msg->bytes[idx] = 0u;
    }

    rte_pack_put_bits(p_msg->bytes, RTE_PACK_TYPE_AT, RTE_PACK_TYPE_BITS,
                 (uint32_t) p_data->type);
    rte_pack_put_bits(p_msg->bytes, RTE_PACK_GRIP_AT, RTE_PACK_GRIP_BITS,
                 (uint32_t) p_data->grip);
    rte_pack_put_bits(p_msg->bytes, RTE_PACK_TIME_AT, RTE_PACK_TIME_BITS,
                 p_data->time_ms);

    if (RTE_RADAR_MSG_FRAME == p_data->type)
    {
        for (uint32_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
        {
            rte_pack_slot(p_msg->bytes,
                          RTE_PACK_SLOT0_AT + (idx * RTE_PACK_SLOT_BITS),
                          &p_data->targets[idx]);
        }
    }
}

/*!
 * @brief Unpack a radar queue message.
 *
 * A reserved type code (3) is returned as FAULT, which is the safe reading
 * (Glasses_Threat goes to CLEAR). A reserved grip code (3) is returned as
 * UNKNOWN, which keeps the alert on. Spare bits are ignored.
 *
 * @param[in]  p_msg   The packed bytes.
 * @param[out] p_data  The message fields.
 */
void
rte_pack_unpack_radar (const rte_radar_msg_t * p_msg, rte_radar_data_t * p_data)
{
    uint32_t type = rte_pack_get_bits(p_msg->bytes, RTE_PACK_TYPE_AT,
                                 RTE_PACK_TYPE_BITS);
    uint32_t grip = rte_pack_get_bits(p_msg->bytes, RTE_PACK_GRIP_AT,
                                 RTE_PACK_GRIP_BITS);

    p_data->type = (RTE_PACK_CODE_RESERVED == type)
                   ? RTE_RADAR_MSG_FAULT
                   : (rte_radar_msg_type_t) type;
    p_data->grip = (RTE_PACK_CODE_RESERVED == grip)
                   ? RTE_GRIP_UNKNOWN
                   : (rte_grip_state_t) grip;
    p_data->time_ms = rte_pack_get_bits(p_msg->bytes, RTE_PACK_TIME_AT,
                                   RTE_PACK_TIME_BITS);

    for (uint32_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        rte_unpack_slot(p_msg->bytes,
                        RTE_PACK_SLOT0_AT + (idx * RTE_PACK_SLOT_BITS),
                        &p_data->targets[idx]);
    }
}

/*** end of file ***/
