/** @file rte.c
 *
 * @brief Runtime environment: queues, tasks and glue for the glasses radar.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * This is the only module that knows both the application components and
 * the operating-system adapter (osal.h). It hosts the two tasks of the report:
 *   Glasses_Radar  runs the LD2450 driver under radar_monitor.
 *   Glasses_Threat runs threat_assessment on every frame.
 */

#include <stdbool.h>
#include <stdint.h>

#include <inttypes.h>


#include "feature_cfg.h"
#include "gr_print.h"
#include "hal_time.h"
#include "ld2450.h"
#include "osal.h"
#include "radar_monitor.h"
#include "radar_monitor_cfg.h"
#include "rte.h"
#include "rte_cfg.h"
#include "rte_pack.h"
#include "rte_type.h"
#include "threat_assessment.h"

static osal_queue_t g_radar_queue;
static osal_queue_t g_audio_queue;
static osal_queue_t g_ble_queue;

static void rte_radar_receive_step (radar_monitor_t * p_monitor,
                                    uint32_t * p_frame_count);
static void rte_glasses_radar_task (void * p_arg);
static void rte_glasses_threat_task (void * p_arg);

/*!
 * @brief Create the queues and the radar receive path.
 *
 * Call before rte_start() and before the scheduler runs.
 *
 * @return true when everything was created.
 */
bool
rte_init (void)
{
    return (osal_queue_create(&g_radar_queue, sizeof(rte_radar_msg_t),
                              RTE_RADAR_QUEUE_LENGTH) &&
            osal_queue_create(&g_audio_queue, sizeof(rte_audio_msg_t),
                              RTE_AUDIO_QUEUE_LENGTH) &&
            osal_queue_create(&g_ble_queue, sizeof(rte_ble_obstacle_t),
                              RTE_BLE_QUEUE_LENGTH) &&
            (ld2450_init()));
}

/*!
 * @brief Create Glasses_Radar and Glasses_Threat at Medium priority.
 *
 * @return true when both tasks were created.
 */
bool
rte_start (void)
{
    return (osal_task_create(rte_glasses_threat_task,
                             RTE_THREAT_TASK_STACK_WORDS,
                             RTE_PRIORITY_MEDIUM_OFFSET, NULL) &&
            osal_task_create(rte_glasses_radar_task,
                             RTE_RADAR_TASK_STACK_WORDS,
                             RTE_PRIORITY_MEDIUM_OFFSET, NULL));
}

/*!
 * @brief Hand one valid frame to Glasses_Threat.
 *
 * A frame that does not fit is dropped; the next one replaces it.
 *
 * @param[in] p_targets  The three decoded target slots.
 * @param[in] time_ms    Time the frame was received.
 */
void
rte_write_radar_frame (const rte_radar_target_t * p_targets, uint32_t time_ms)
{
    rte_radar_data_t data;
    rte_radar_msg_t msg;

    data.type = RTE_RADAR_MSG_FRAME;
    data.time_ms = time_ms;
    data.grip = RTE_GRIP_UNKNOWN;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        data.targets[idx] = p_targets[idx];
    }

    rte_pack_radar(&data, &msg);

    // The result is ignored on purpose: a dropped frame is replaced by the
    // next one about 140 ms later.
    (void) osal_queue_send(&g_radar_queue, &msg, 0u);
}

/*!
 * @brief Tell Glasses_Threat that the radar failed (it then goes to CLEAR).
 */
void
rte_write_radar_fault (void)
{
    rte_radar_data_t data = { .type = RTE_RADAR_MSG_FAULT, .time_ms = 0u,
                              .grip = RTE_GRIP_UNKNOWN };
    rte_radar_msg_t msg;

    rte_pack_radar(&data, &msg);

    // The result is ignored on purpose: the short wait makes a lost fault
    // message very unlikely, and no other part can act on the failure.
    (void) osal_queue_send(&g_radar_queue, &msg, RTE_CRITICAL_WAIT_MS);
}

/*!
 * @brief Tell Glasses_Threat the cane grip state (FR6). Called by the BLE task.
 *
 * Waits briefly like CLEAR and FAULT: losing a RELEASED or a HELD would leave
 * the alert in the wrong state until the next grip message.
 *
 * @param[in] grip  HELD or RELEASED from a cane packet, UNKNOWN after 500 ms
 *                  without one.
 */
void
rte_write_grip_state (rte_grip_state_t grip)
{
    rte_radar_data_t data = { .type = RTE_RADAR_MSG_GRIP, .time_ms = 0u,
                              .grip = grip };
    rte_radar_msg_t msg;

    rte_pack_radar(&data, &msg);

    // The result is ignored on purpose. The BLE task is expected to repeat
    // the state with every cane packet (about 10 Hz), so a lost message is
    // corrected by the next one.
    (void) osal_queue_send(&g_radar_queue, &msg, RTE_CRITICAL_WAIT_MS);
}

/*!
 * @brief Send a message to the Audio Alert task.
 *
 * OBSTACLE never waits (sent every frame). CLEAR, RADAR_FAULT and RADAR_OK
 * wait briefly because losing one could leave a wrong tone on.
 *
 * @param[in] type   Kind of message.
 * @param[in] d_mm   Distance, used by RTE_AUDIO_OBSTACLE only.
 */
void
rte_write_audio_msg (rte_audio_msg_type_t type, uint16_t d_mm)
{
    rte_audio_msg_t msg = { .type = type, .d_mm = d_mm };
    uint32_t wait_ms = 0u;

    if (RTE_AUDIO_OBSTACLE != type)
    {
        wait_ms = RTE_CRITICAL_WAIT_MS;
    }

    // The result is ignored on purpose, see the function comment.
    (void) osal_queue_send(&g_audio_queue, &msg, wait_ms);
}

/*!
 * @brief Send the obstacle state to the BLE task (best effort, FR10).
 *
 * @param[in] b_active  true while an obstacle is active.
 * @param[in] d_mm      Distance when active.
 * @param[in] ttc_ms    Time to collision, RTE_TTC_NONE_MS if unknown.
 */
void
rte_write_ble_obstacle (bool b_active, uint16_t d_mm, uint16_t ttc_ms)
{
    rte_ble_obstacle_t msg = { .b_active = b_active, .d_mm = d_mm,
                               .ttc_ms = ttc_ms };

    // Local alerts never depend on BLE, so a full queue is not an error.
    (void) osal_queue_send(&g_ble_queue, &msg, 0u);
}

/*!
 * @brief Receive the next message for the Audio Alert task.
 *
 * @param[out] p_msg       Receives the message.
 * @param[in]  timeout_ms  Longest wait, or RTE_WAIT_FOREVER.
 *
 * @return true when a message was received.
 */
bool
rte_read_audio_msg (rte_audio_msg_t * p_msg, uint32_t timeout_ms)
{
    return (osal_queue_receive(&g_audio_queue, p_msg, timeout_ms));
}

/*!
 * @brief Receive the next message for the BLE task.
 *
 * @param[out] p_msg       Receives the message.
 * @param[in]  timeout_ms  Longest wait, or RTE_WAIT_FOREVER.
 *
 * @return true when a message was received.
 */
bool
rte_read_ble_obstacle (rte_ble_obstacle_t * p_msg, uint32_t timeout_ms)
{
    return (osal_queue_receive(&g_ble_queue, p_msg, timeout_ms));
}

#if (1 == FEATURE_DEBUG_LOGS)
// Start-up facts of the radar task, saved when they happen. The START and
// CONFIG lines are printed before the serial monitor has reconnected after a
// reset, so the first periodic lines print these values again (BOOT line).
#define RTE_DEBUG_BOOT_LINES    (3u)

static uint32_t g_dbg_start_ms = 0u;
static uint32_t g_dbg_start_bytes = 0u;
static uint32_t g_dbg_start_dropped = 0u;
static bool g_dbg_config_seen = false;
static bool g_dbg_config_ok = false;
static uint32_t g_dbg_config_ms = 0u;
static uint32_t g_dbg_config_bytes = 0u;
static uint32_t g_dbg_config_dropped = 0u;
static uint32_t g_dbg_boot_lines = 0u;

/*!
 * @brief Print the receive-path counters with a time stamp (debug only).
 *
 * The first RTE_DEBUG_BOOT_LINES periodic lines are followed by a BOOT line
 * that repeats the start-up values (task start, last configuration result).
 *
 * @param[in] b_timeout  true for the line printed on a frame timeout.
 * @param[in] frames     Valid frames seen so far.
 */
static void
rte_debug_print_rx_stats (bool b_timeout, uint32_t frames)
{
    ld2450_rx_stats_t stats;

    ld2450_get_rx_stats(&stats);
    gr_printf("[Radar] %s t=%" PRIu32 "ms frames=%" PRIu32 " bytes=%" PRIu32
           " isr=%" PRIu32 " overrun=%" PRIu32 " dropped=%" PRIu32
           " footer_bad=%" PRIu32 "\n",
           (b_timeout ? "TIMEOUT" : "frames"), hal_time_now_ms(), frames,
           stats.byte_count, stats.isr_count, stats.overrun_count,
           stats.dropped_bytes, ld2450_get_bad_footer_count());

    if ((!b_timeout) && (g_dbg_boot_lines < RTE_DEBUG_BOOT_LINES))
    {
        g_dbg_boot_lines++;
        gr_printf("[Radar] BOOT start t=%" PRIu32 "ms bytes=%" PRIu32
               " dropped=%" PRIu32 " | config %s t=%" PRIu32 "ms bytes=%"
               PRIu32 " dropped=%" PRIu32 "\n", g_dbg_start_ms,
               g_dbg_start_bytes, g_dbg_start_dropped,
               (!g_dbg_config_seen) ? "NONE" :
               (g_dbg_config_ok ? "OK" : "FAILED"), g_dbg_config_ms,
               g_dbg_config_bytes, g_dbg_config_dropped);
    }
}
#endif

#if (1 == FEATURE_DEBUG_LOGS)
/*!
 * @brief Print the three target slots of one frame (debug only).
 *
 * Format: [T] t=<ms> <slot0> <slot1> <slot2>, each slot x,y,speed in mm and
 * cm/s, or "-" when the slot is empty.
 *
 * @param[in] p_targets  The three decoded target slots.
 * @param[in] now_ms     Time the frame was received.
 */
static void
rte_debug_print_targets (const rte_radar_target_t * p_targets, uint32_t now_ms)
{
    gr_printf("[T] t=%" PRIu32, now_ms);

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        if (p_targets[idx].b_valid)
        {
            gr_printf(" %d,%d,%d", p_targets[idx].x_mm, p_targets[idx].y_mm,
                   p_targets[idx].speed_cms);
        }
        else
        {
            gr_printf(" -");
        }
    }

    gr_printf("\n");
}
#endif

/*!
 * @brief One receive step of Glasses_Radar: wait for a frame, report it.
 *
 * A short read or a bad footer needs no report: the next call resyncs on
 * the next header (FR7).
 *
 * @param[in,out] p_monitor      Supervision state.
 * @param[in,out] p_frame_count  Counts valid frames for the debug line.
 */
static void
rte_radar_receive_step (radar_monitor_t * p_monitor, uint32_t * p_frame_count)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    switch (ld2450_receive_frame(targets,
                                 radar_monitor_get_deadline_ms(p_monitor)))
    {
        case LD2450_FRAME_OK:
            radar_monitor_on_frame(p_monitor, targets, hal_time_now_ms());
            (*p_frame_count)++;

#if (1 == FEATURE_DEBUG_LOGS)
            if ((1 == FEATURE_DEBUG_LOGS) && (1 == FEATURE_DEBUG_LOG_TARGETS))
            {
                rte_debug_print_targets(targets, hal_time_now_ms());
            }

            if ((1 == FEATURE_DEBUG_LOGS) &&
                (0u == (*p_frame_count % FEATURE_DEBUG_LOG_FRAME_INTERVAL)))
            {
                rte_debug_print_rx_stats(false, *p_frame_count);
            }
#endif
            break;

        case LD2450_FRAME_TIMEOUT:
#if (1 == FEATURE_DEBUG_LOGS)
            if (1 == FEATURE_DEBUG_LOGS)
            {
                rte_debug_print_rx_stats(true, *p_frame_count);
            }
#endif
            radar_monitor_on_timeout(p_monitor, hal_time_now_ms());
            break;

        case LD2450_FRAME_SHORT:
        case LD2450_FRAME_BAD_FOOTER:
        default:
            // Nothing to report: the next call resyncs on the next header.
            break;
    }
}

/*!
 * @brief One configuration attempt of Glasses_Radar, with a debug line.
 *
 * The line shows when each attempt ran, whether it worked and how many
 * bytes the receive path had already lost (startup backlog check).
 *
 * @param[in,out] p_monitor  Supervision state.
 */
static void
rte_radar_configure_step (radar_monitor_t * p_monitor)
{
    bool b_ok = ld2450_configure();

    radar_monitor_on_config_result(p_monitor, b_ok, hal_time_now_ms());

#if (1 == FEATURE_DEBUG_LOGS)
    if (1 == FEATURE_DEBUG_LOGS)
    {
        ld2450_rx_stats_t stats;

        ld2450_get_rx_stats(&stats);
        g_dbg_config_seen = true;
        g_dbg_config_ok = b_ok;
        g_dbg_config_ms = hal_time_now_ms();
        g_dbg_config_bytes = stats.byte_count;
        g_dbg_config_dropped = stats.dropped_bytes;
        gr_printf("[Radar] CONFIG %s t=%" PRIu32 "ms bytes=%" PRIu32
               " dropped=%" PRIu32 " | start t=%" PRIu32 "ms bytes=%" PRIu32
               " dropped=%" PRIu32 "\n", (b_ok ? "OK" : "FAILED"),
               hal_time_now_ms(), stats.byte_count, stats.dropped_bytes,
               g_dbg_start_ms, g_dbg_start_bytes, g_dbg_start_dropped);
    }
#endif

    if (b_ok)
    {
        // Count from here on: the start-up backlog is not a steady-state loss.
        ld2450_reset_rx_stats();
    }
}

/*!
 * @brief Glasses_Radar task: drives the LD2450 under radar_monitor.
 *
 * @param[in] p_arg  Unused.
 */
static void
rte_glasses_radar_task (void * p_arg)
{
    radar_monitor_t monitor;
    uint32_t frame_count = 0u;

    // The void cast only silences the unused-parameter warning.
    (void) p_arg;

    radar_monitor_init(&monitor);

#if (1 == FEATURE_DEBUG_LOGS)
    if (1 == FEATURE_DEBUG_LOGS)
    {
        ld2450_rx_stats_t stats;

        ld2450_get_rx_stats(&stats);
        g_dbg_start_ms = hal_time_now_ms();
        g_dbg_start_bytes = stats.byte_count;
        g_dbg_start_dropped = stats.dropped_bytes;
        gr_printf("[Radar] START t=%" PRIu32 "ms bytes=%" PRIu32
               " dropped=%" PRIu32 "\n", g_dbg_start_ms, g_dbg_start_bytes,
               g_dbg_start_dropped);
    }
#endif

    for (;;)
    {
        switch (radar_monitor_get_action(&monitor))
        {
            case RADAR_MONITOR_ACTION_CONFIGURE:
                rte_radar_configure_step(&monitor);
                break;

            case RADAR_MONITOR_ACTION_WAIT_RETRY:
                osal_delay_ms(RADAR_MONITOR_CONFIG_RETRY_MS);
                radar_monitor_on_retry_done(&monitor);
                break;

            case RADAR_MONITOR_ACTION_RECEIVE:
                rte_radar_receive_step(&monitor, &frame_count);
                break;

            default:
                break;
        }
    }
}

/*!
 * @brief Glasses_Threat task: blocks on the radar queue (frames, faults, grip).
 *
 * Frames keep arriving while the radar works, empty frames included, so the
 * 500 ms no-target hold in threat_assessment needs no timer here.
 *
 * @param[in] p_arg  Unused.
 */
static void
rte_glasses_threat_task (void * p_arg)
{
    threat_assessment_t state;
    rte_radar_msg_t msg;
    rte_radar_data_t data;

    // The void cast only silences the unused-parameter warning.
    (void) p_arg;

    threat_assessment_init(&state);

    for (;;)
    {
        if (osal_queue_receive(&g_radar_queue, &msg, OSAL_WAIT_FOREVER))
        {
            rte_pack_unpack_radar(&msg, &data);

            switch (data.type)
            {
                case RTE_RADAR_MSG_FRAME:
                    threat_assessment_process_frame(&state, data.targets,
                                                    data.time_ms);
                    break;

                case RTE_RADAR_MSG_GRIP:
                    threat_assessment_process_grip(&state, data.grip);
                    break;

                case RTE_RADAR_MSG_FAULT:
                default:
                    threat_assessment_process_fault(&state);
                    break;
            }
        }
    }
}

/*** end of file ***/
