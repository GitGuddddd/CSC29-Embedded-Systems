/** @file mock_rte.c
 *
 * @brief Host stub of the RTE: records every write instead of queuing it.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mock_hal.h"
#include "rte.h"
#include "rte_pack.h"
#include "rte_type.h"

static rte_audio_msg_t g_audio_log[MOCK_LOG_CAPACITY];
static size_t g_audio_count = 0u;
static rte_ble_obstacle_t g_ble_log[MOCK_LOG_CAPACITY];
static size_t g_ble_count = 0u;
static rte_radar_msg_t g_radar_log[MOCK_LOG_CAPACITY];
static size_t g_radar_count = 0u;
static size_t g_radar_frames = 0u;
static size_t g_radar_faults = 0u;
static size_t g_radar_grips = 0u;

/*!
 * @brief Forget everything recorded so far.
 */
void
mock_rte_reset (void)
{
    g_audio_count = 0u;
    g_ble_count = 0u;
    g_radar_count = 0u;
    g_radar_frames = 0u;
    g_radar_faults = 0u;
    g_radar_grips = 0u;
}

/*!
 * @brief Number of audio messages recorded.
 *
 * @return The count.
 */
size_t
mock_audio_count (void)
{
    return (g_audio_count);
}

/*!
 * @brief One recorded audio message.
 *
 * @param[in] index  Position, starting at 0.
 *
 * @return The message.
 */
rte_audio_msg_t
mock_audio_get (size_t index)
{
    return (g_audio_log[index]);
}

/*!
 * @brief Number of BLE messages recorded.
 *
 * @return The count.
 */
size_t
mock_ble_count (void)
{
    return (g_ble_count);
}

/*!
 * @brief One recorded BLE message.
 *
 * @param[in] index  Position, starting at 0.
 *
 * @return The message.
 */
rte_ble_obstacle_t
mock_ble_get (size_t index)
{
    return (g_ble_log[index]);
}

/*!
 * @brief Number of radar frames recorded.
 *
 * @return The count.
 */
size_t
mock_radar_frame_count (void)
{
    return (g_radar_frames);
}

/*!
 * @brief Number of radar fault messages recorded.
 *
 * @return The count.
 */
size_t
mock_radar_fault_count (void)
{
    return (g_radar_faults);
}

/*!
 * @brief Number of grip state messages recorded.
 *
 * @return The count.
 */
size_t
mock_radar_grip_count (void)
{
    return (g_radar_grips);
}

/*!
 * @brief One recorded radar message (frame, fault or grip), as it comes out
 *        of the queue: packed, then unpacked again.
 *
 * @param[in] index  Position, starting at 0.
 *
 * @return The message fields.
 */
rte_radar_data_t
mock_radar_get (size_t index)
{
    rte_radar_data_t data;

    rte_pack_unpack_radar(&g_radar_log[index], &data);

    return (data);
}

/*!
 * @brief One recorded radar message as the packed bytes in the queue.
 *
 * @param[in] index  Position, starting at 0.
 *
 * @return The packed message.
 */
rte_radar_msg_t
mock_radar_get_packed (size_t index)
{
    return (g_radar_log[index]);
}

static void
mock_radar_store (const rte_radar_data_t * p_data)
{
    if (g_radar_count < MOCK_LOG_CAPACITY)
    {
        rte_pack_radar(p_data, &g_radar_log[g_radar_count]);
        g_radar_count++;
    }
}

void
rte_write_radar_frame (const rte_radar_target_t * p_targets, uint32_t time_ms)
{
    rte_radar_data_t data;

    data.type = RTE_RADAR_MSG_FRAME;
    data.time_ms = time_ms;
    data.grip = RTE_GRIP_UNKNOWN;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        data.targets[idx] = p_targets[idx];
    }

    mock_radar_store(&data);
    g_radar_frames++;
}

void
rte_write_radar_fault (void)
{
    rte_radar_data_t data = { .type = RTE_RADAR_MSG_FAULT, .time_ms = 0u,
                              .grip = RTE_GRIP_UNKNOWN };

    mock_radar_store(&data);
    g_radar_faults++;
}

void
rte_write_grip_state (rte_grip_state_t grip)
{
    rte_radar_data_t data = { .type = RTE_RADAR_MSG_GRIP, .time_ms = 0u,
                              .grip = grip };

    mock_radar_store(&data);
    g_radar_grips++;
}

void
rte_write_audio_msg (rte_audio_msg_type_t type, uint16_t d_mm)
{
    if (g_audio_count < MOCK_LOG_CAPACITY)
    {
        g_audio_log[g_audio_count].type = type;
        g_audio_log[g_audio_count].d_mm = d_mm;
        g_audio_count++;
    }
}

void
rte_write_ble_obstacle (bool b_active, uint16_t d_mm, uint16_t ttc_ms)
{
    if (g_ble_count < MOCK_LOG_CAPACITY)
    {
        g_ble_log[g_ble_count].b_active = b_active;
        g_ble_log[g_ble_count].d_mm = d_mm;
        g_ble_log[g_ble_count].ttc_ms = ttc_ms;
        g_ble_count++;
    }
}

/*** end of file ***/
