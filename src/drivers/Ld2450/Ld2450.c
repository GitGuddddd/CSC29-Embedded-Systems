#include "Ld2450.h"
#include "Ld2450_Cfg.h"
#include <string.h>

/*
 * \file Ld2450.c
 * \brief LD2450 frame parser (AA FF 03 00 ... 55 CC) per FR7
 * Compliance: Barr-C:2018
 */

/* Per-slot parser state, packed to the bits each value needs (minimum-bit
 * representation): bit n of implausible / frozen = slot n of the last parsed
 * frame was rejected for that reason; same_n = consecutive frames slot n has
 * repeated an identical report (saturates at LD2450_CFG_FROZEN_FRAMES).
 * Written by ParseFrame and read by the getters, all from the radar task. */
typedef struct
{
    uint32_t implausible : 3;
    uint32_t frozen : 3;
    uint32_t same_0 : 4;
    uint32_t same_1 : 4;
    uint32_t same_2 : 4;
    uint32_t reserved : 14;
} Ld2450_Flags_t;

_Static_assert(sizeof(Ld2450_Flags_t) == 4U,
               "parser flags must pack into 32 bits");
_Static_assert(LD2450_CFG_MAX_TARGETS <= 3U, "one flag bit per slot");
_Static_assert((LD2450_CFG_FROZEN_FRAMES >= 2U) &&
               (LD2450_CFG_FROZEN_FRAMES <= 15U),
               "frozen counter is 4 bits");

static Ld2450_Flags_t s_ld = { 0U, 0U, 0U, 0U, 0U, 0U };

/* 16-bit fingerprint of each slot's last 8-byte report: 6 bytes instead of
 * 24 for three full copies. A different report matching by chance needs a
 * 1-in-65536 collision on every one of ~10 frames in a row. */
static uint16_t s_last_hash[LD2450_CFG_MAX_TARGETS];

static uint16_t Report_Hash(const uint8_t * const p_report)
{
    uint32_t h = 2166136261U; /* FNV-1a */
    for (uint32_t i = 0U; i < 8U; i++)
    {
        h ^= (uint32_t)p_report[i];
        h *= 16777619U;
    }
    return (uint16_t)((h >> 16) ^ (h & 0xFFFFU));
}

static uint8_t Same_Get(uint8_t slot)
{
    uint32_t n = s_ld.same_0;
    if (slot == 1U)
    {
        n = s_ld.same_1;
    }
    else if (slot == 2U)
    {
        n = s_ld.same_2;
    }
    else
    {
        /* slot 0 */
    }
    return (uint8_t)n;
}

static void Same_Set(uint8_t slot, uint8_t n)
{
    if (slot == 1U)
    {
        s_ld.same_1 = n & 0x0FU;
    }
    else if (slot == 2U)
    {
        s_ld.same_2 = n & 0x0FU;
    }
    else
    {
        s_ld.same_0 = n & 0x0FU;
    }
}

/* Count how many frames in a row this slot has sent the same report; true
 * once it has repeated for LD2450_CFG_FROZEN_FRAMES frames. */
static bool Slot_IsFrozen(uint8_t slot, const uint8_t * const p_report,
                          bool empty)
{
    const uint16_t h = Report_Hash(p_report);
    uint8_t n = 1U;
    if (!empty && (h == s_last_hash[slot]))
    {
        n = Same_Get(slot);
        if (n < LD2450_CFG_FROZEN_FRAMES)
        {
            n++;
        }
    }
    s_last_hash[slot] = h;
    Same_Set(slot, empty ? 0U : n);
    return !empty && (n >= LD2450_CFG_FROZEN_FRAMES);
}

void Ld2450_ResetParser(void)
{
    s_ld = (Ld2450_Flags_t){ 0U, 0U, 0U, 0U, 0U, 0U };
    for (uint8_t i = 0U; i < LD2450_CFG_MAX_TARGETS; i++)
    {
        s_last_hash[i] = 0U;
    }
}

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
     * is_valid = false rather than letting later targets shift down.
     * ThreatAssessment tracks per index, so compaction would hand one person's
     * history to another. */
    for (uint8_t i = 0U; i < LD2450_CFG_MAX_TARGETS; i++)
    {
        p_targets[i] = (Radar_Target_t){ 0U, 0, 0, 0, false };
    }

    /* Up to 3 targets: 8 bytes per target starting at index 4 */
    uint8_t implausible = 0U;
    uint8_t frozen = 0U;
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

        /* An empty slot is all zeros (y == 0). The limits reject values the
         * radar cannot physically report, so one corrupt value cannot become,
         * for example, a 15 m/s closing speed. */
        const bool empty = (x_raw == 0U) && (y_raw == 0U) && (speed_raw == 0U);
        const bool stuck = Slot_IsFrozen(i, &p_frame[offset], empty);
        const bool plausible = (y > 0) && (y <= LD2450_CFG_MAX_Y_MM) &&
                               (x >= -LD2450_CFG_MAX_ABS_X_MM) &&
                               (x <= LD2450_CFG_MAX_ABS_X_MM) &&
                               (speed >= -LD2450_CFG_MAX_ABS_SPEED_CMS) &&
                               (speed <= LD2450_CFG_MAX_ABS_SPEED_CMS);

        if (stuck)
        {
            /* Same report for ~1 s: the radar is repeating a stale target.
             * Treat it as not valid (the lane hold covers the lane) rather
             * than as a real obstacle that never moves. */
            frozen |= (uint8_t)(1U << i);
        }
        else if (plausible)
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
        else if (!empty)
        {
            implausible |= (uint8_t)(1U << i); /* not empty, but rejected */
        }
        else
        {
            /* empty slot */
        }
    }
    s_ld.implausible = implausible & 0x07U;
    s_ld.frozen = frozen & 0x07U;

    *p_count = (uint8_t)LD2450_CFG_MAX_TARGETS;
    return true;
}

uint8_t Ld2450_GetImplausibleMask(void)
{
    return (uint8_t)s_ld.implausible;
}

uint8_t Ld2450_GetFrozenMask(void)
{
    return (uint8_t)s_ld.frozen;
}
