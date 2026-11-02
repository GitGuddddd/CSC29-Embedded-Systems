/** @file radar_monitor.c
 *
 * @brief Glasses_Radar supervision: INIT, RUN and FAULT (statechart 6.6).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_monitor.h"
#include "radar_monitor_cfg.h"
#include "rte.h"
#include "rte_type.h"

#define RADAR_MONITOR_HALF_RANGE_MS    (0x80000000u)

static void radar_monitor_enter_fault (radar_monitor_t * p_monitor);
static bool radar_monitor_is_settling (const radar_monitor_t * p_monitor,
                                       uint32_t now_ms);
static void radar_monitor_apply_corridor (
    const rte_radar_target_t * p_in, rte_radar_target_t * p_out);

/*!
 * @brief Start in INIT, with no delay pending.
 *
 * @param[out] p_monitor  Supervision state to initialise.
 */
void
radar_monitor_init (radar_monitor_t * p_monitor)
{
    p_monitor->state = RADAR_MONITOR_STATE_INIT;
    p_monitor->b_delay_pending = false;
    p_monitor->b_fault_reported = false;
    p_monitor->b_listening = false;
    p_monitor->failed_tries = 0u;
    p_monitor->deadline_ms = 0u;
    p_monitor->b_settling = false;
    p_monitor->settle_until_ms = 0u;
}

/*!
 * @brief Tell the task what to do next.
 *
 * @param[in] p_monitor  Supervision state.
 *
 * @return The next action.
 */
radar_monitor_action_t
radar_monitor_get_action (const radar_monitor_t * p_monitor)
{
    radar_monitor_action_t action = RADAR_MONITOR_ACTION_CONFIGURE;

    if (p_monitor->b_delay_pending)
    {
        action = RADAR_MONITOR_ACTION_WAIT_RETRY;
    }
    else if ((RADAR_MONITOR_STATE_RUN == p_monitor->state) ||
             p_monitor->b_listening)
    {
        action = RADAR_MONITOR_ACTION_RECEIVE;
    }
    else
    {
        action = RADAR_MONITOR_ACTION_CONFIGURE;
    }

    return (action);
}

/*!
 * @brief Time by which the next valid frame must arrive.
 *
 * @param[in] p_monitor  Supervision state.
 *
 * @return Absolute deadline in milliseconds.
 */
uint32_t
radar_monitor_get_deadline_ms (const radar_monitor_t * p_monitor)
{
    return (p_monitor->deadline_ms);
}

/*!
 * @brief Report the result of a configuration attempt.
 *
 * @param[in,out] p_monitor  Supervision state.
 * @param[in]     b_ok       true when the sensor acknowledged the config.
 * @param[in]     now_ms     Current time.
 */
void
radar_monitor_on_config_result (radar_monitor_t * p_monitor, bool b_ok,
                                uint32_t now_ms)
{
    if (b_ok)
    {
        // The first frame is given a longer deadline (bench finding).
        p_monitor->state = RADAR_MONITOR_STATE_RUN;
        p_monitor->b_listening = false;
        p_monitor->failed_tries = 0u;
        p_monitor->deadline_ms = now_ms + RADAR_MONITOR_POST_CONFIG_GRACE_MS;
        p_monitor->b_settling = true;
        p_monitor->settle_until_ms = now_ms + RADAR_MONITOR_SETTLE_MS;
    }
    else
    {
        p_monitor->failed_tries++;
        p_monitor->b_delay_pending = true;

        if ((RADAR_MONITOR_STATE_INIT == p_monitor->state) &&
            (p_monitor->failed_tries >= RADAR_MONITOR_CONFIG_MAX_TRIES))
        {
            radar_monitor_enter_fault(p_monitor);
        }
    }
}

/*!
 * @brief Report that the retry pause is over.
 *
 * @param[in,out] p_monitor  Supervision state.
 */
void
radar_monitor_on_retry_done (radar_monitor_t * p_monitor)
{
    p_monitor->b_delay_pending = false;
}

/*!
 * @brief Report a valid frame: restart the 500 ms timer, forward the frame.
 *
 * @param[in,out] p_monitor  Supervision state.
 * @param[in]     p_targets  The three decoded target slots.
 * @param[in]     now_ms     Time the frame was received.
 */
void
radar_monitor_on_frame (radar_monitor_t * p_monitor,
                        const rte_radar_target_t * p_targets, uint32_t now_ms)
{
    rte_radar_target_t gated[RTE_RADAR_NUM_TARGETS];

    p_monitor->deadline_ms = now_ms + RADAR_MONITOR_FRAME_TIMEOUT_MS;

    if (p_monitor->b_listening)
    {
        // The sensor came back by itself: no new config is needed.
        p_monitor->state = RADAR_MONITOR_STATE_RUN;
        p_monitor->b_listening = false;
    }

    if (p_monitor->b_fault_reported)
    {
        rte_write_audio_msg(RTE_AUDIO_RADAR_OK, 0u);
        p_monitor->b_fault_reported = false;
    }

    radar_monitor_apply_corridor(p_targets, gated);
    rte_write_radar_frame(gated, now_ms);
}

/*!
 * @brief Tell whether the sensor is still settling after its configuration.
 *
 * @param[in] p_monitor  Supervision state.
 * @param[in] now_ms     Current time.
 *
 * @return true while now is before the end of the settle time.
 */
static bool
radar_monitor_is_settling (const radar_monitor_t * p_monitor, uint32_t now_ms)
{
    // Unsigned subtraction wraps correctly when the 32-bit clock rolls over.
    uint32_t left_ms = p_monitor->settle_until_ms - now_ms;

    return (p_monitor->b_settling && (left_ms > 0u) &&
            (left_ms < RADAR_MONITOR_HALF_RANGE_MS));
}

/*!
 * @brief Corridor gate: empty every slot that is outside the walking path.
 *
 * Decided on this frame alone, so it adds no delay. A slot with
 * |x| <= RADAR_MONITOR_CORRIDOR_HALF_MM is copied unchanged.
 *
 * @param[in]  p_in   The three decoded target slots.
 * @param[out] p_out  The three slots to send.
 */
static void
radar_monitor_apply_corridor (const rte_radar_target_t * p_in,
                              rte_radar_target_t * p_out)
{
    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        int32_t x_mm = p_in[idx].x_mm;

        p_out[idx] = p_in[idx];

        if (p_in[idx].b_valid &&
            ((x_mm > RADAR_MONITOR_CORRIDOR_HALF_MM) ||
             (x_mm < -RADAR_MONITOR_CORRIDOR_HALF_MM)))
        {
            p_out[idx].b_valid = false;
            p_out[idx].x_mm = 0;
            p_out[idx].y_mm = 0;
            p_out[idx].d_mm = 0u;
            p_out[idx].speed_cms = 0;
        }
    }
}

/*!
 * @brief Report that no valid frame arrived before the deadline (FR11).
 *
 * In RUN the first timeout reports the fault and keeps listening for the
 * retry time. If that ends too, the task configures the sensor again.
 * Inside the settle time after a configuration a timeout only clears the
 * tone (see RADAR_MONITOR_SETTLE_MS).
 *
 * @param[in,out] p_monitor  Supervision state.
 * @param[in]     now_ms     Time of the timeout.
 */
void
radar_monitor_on_timeout (radar_monitor_t * p_monitor, uint32_t now_ms)
{
    if ((RADAR_MONITOR_STATE_RUN == p_monitor->state) &&
        radar_monitor_is_settling(p_monitor, now_ms))
    {
        // The sensor is quiet right after its configuration. Stop the tone
        // (Threat goes to CLEAR) but raise no fault; wait for the end of the
        // settle time before treating silence as a fault.
        rte_write_radar_fault();
        p_monitor->deadline_ms = p_monitor->settle_until_ms;
    }
    else if (RADAR_MONITOR_STATE_RUN == p_monitor->state)
    {
        radar_monitor_enter_fault(p_monitor);
        p_monitor->b_listening = true;
        p_monitor->deadline_ms = now_ms + RADAR_MONITOR_CONFIG_RETRY_MS;
    }
    else if (p_monitor->b_listening)
    {
        // Nothing came back: stop listening, the next action is CONFIGURE.
        p_monitor->b_listening = false;
    }
    else
    {
        // INIT or an idle FAULT: nothing to do.
    }
}

/*!
 * @brief Go to FAULT: RADAR_FAULT to Audio Alert (once) and to Threat.
 *
 * @param[in,out] p_monitor  Supervision state.
 */
static void
radar_monitor_enter_fault (radar_monitor_t * p_monitor)
{
    p_monitor->state = RADAR_MONITOR_STATE_FAULT;

    if (!p_monitor->b_fault_reported)
    {
        rte_write_audio_msg(RTE_AUDIO_RADAR_FAULT, 0u);
        p_monitor->b_fault_reported = true;
    }

    // A stale tone must never stay on: Glasses_Threat goes to CLEAR.
    rte_write_radar_fault();
}

/*** end of file ***/
