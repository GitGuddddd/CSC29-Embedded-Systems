#include "Ld2450.h"
#include "Ld2450_Cfg.h"
#include <string.h>

/*
 * \file Ld2450.c
 * \brief LD2450 frame parser (AA FF 03 00 ... 55 CC) per FR7
 * Compliance: Barr-C:2018
 */

static int16_t Decode_SignedMagnitude(uint16_t raw)
{
    int16_t magnitude = (int16_t)(raw & 0x7FFFU);
    /* HLK-LD2450 sign-magnitude convention: bit 15 = 1 is positive, bit 15 = 0 is negative.
     * The masked magnitude is at most 32767; its negation also fits int16_t.
     */
    return ((raw & 0x8000U) != 0U) ? magnitude : (int16_t)(-magnitude);
}

static uint16_t Isqrt32(uint32_t n)
{
    uint32_t res = 0U;
    uint32_t bit = 0x40000000U;

    while (bit > n)
    {
        bit >>= 2;
    }

    while (bit != 0U)
    {
        if (n >= (res + bit))
        {
            n -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }

    return (uint16_t)res;
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

    /* Every radar slot keeps its own index: an empty slot is returned as
     * is_valid = false rather than letting later targets shift down. ThreatAssessment
     * tracks per index, so compaction would hand one person's history to another.
     */
    for (uint8_t i = 0U; i < LD2450_CFG_MAX_TARGETS; i++)
    {
        p_targets[i] = (Radar_Target_t){ 0U, 0, 0, 0, false };
    }

    /* Up to 3 targets: 8 bytes per target starting at index 4 */
    for (uint8_t i = 0U; i < LD2450_CFG_MAX_TARGETS; i++)
    {
        uint32_t offset = 4U + (i * 8U);

        uint16_t x_raw = (uint16_t)(((uint16_t)p_frame[offset + 1U] << 8) | p_frame[offset]);
        int16_t x = Decode_SignedMagnitude(x_raw);

        uint16_t y_raw = (uint16_t)(((uint16_t)p_frame[offset + 3U] << 8) | p_frame[offset + 2U]);
        int16_t y = Decode_SignedMagnitude(y_raw);

        uint16_t speed_raw = (uint16_t)(((uint16_t)p_frame[offset + 5U] << 8) | p_frame[offset + 4U]);
        int16_t speed = Decode_SignedMagnitude(speed_raw);

        uint16_t dist_res = (uint16_t)(((uint16_t)p_frame[offset + 7U] << 8) | p_frame[offset + 6U]);

        (void)dist_res;

        /* An empty slot is all zeros (y == 0). The limits reject values the radar cannot
         * physically report, so one corrupt value cannot become, for example, a 15 m/s
         * closing speed. */
        const bool plausible = (y > 0) && (y <= LD2450_CFG_MAX_Y_MM) &&
                               (x >= -LD2450_CFG_MAX_ABS_X_MM) && (x <= LD2450_CFG_MAX_ABS_X_MM) &&
                               (speed >= -LD2450_CFG_MAX_ABS_SPEED_CMS) && (speed <= LD2450_CFG_MAX_ABS_SPEED_CMS);

        if (plausible)
        {
            uint32_t x_sq = (uint32_t)((int32_t)x * (int32_t)x);
            uint32_t y_sq = (uint32_t)((int32_t)y * (int32_t)y);
            uint16_t dist_mm = Isqrt32(x_sq + y_sq);
            p_targets[i].distance_mm = dist_mm;
            p_targets[i].speed_cms = speed;
            p_targets[i].is_valid = true;
            p_targets[i].angle_deg = 0;
            p_targets[i].x_mm = x;
        }
    }

    *p_count = (uint8_t)LD2450_CFG_MAX_TARGETS;
    return true;
}
