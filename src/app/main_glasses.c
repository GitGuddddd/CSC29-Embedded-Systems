/** @file main_glasses.c
 *
 * @brief Glasses launcher: creates the radar objects and tasks.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * WARNING: Both tasks here are stand-ins. The audio task is a BRING-UP bridge to
 * Brian's alert.c (he owns the real Audio Alert task and replaces it). The BLE
 * stub only prints what the BLE task would receive; its owner replaces it.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "alert.h"
#include "audio_alert_bridge.h"
#include "board_glasses.h"
#include "gr_print.h"
#include "hal_time.h"
#include "main_glasses.h"
#include "osal.h"
#include "rte.h"
#include "rte_cfg.h"
#include "rte_type.h"
#include "threat_assessment.h"

#define MAIN_GLASSES_STUB_STACK_WORDS    (512u)

// alert_update() must run regularly (Brian's alert.c is a polled module).
#define MAIN_GLASSES_ALERT_TICK_MS       (10u)

static void main_glasses_stub_audio_task (void * p_arg);
static void main_glasses_stub_ble_task (void * p_arg);

/*!
 * @brief Phase 1: create the queues and set up the radar (UART, interrupt).
 *
 * @return E_OK, or E_NOMEM when an object could not be created.
 */
ER
main_glasses_init (void)
{
    gr_printf("Glasses perception (Group 29): init\n");

    return (rte_init() ? E_OK : E_NOMEM);
}

/*!
 * @brief Phase 2: start the radar, threat and stub tasks.
 *
 * @return E_OK, or E_NOMEM when a task could not be started.
 */
ER
main_glasses_start (void)
{
    bool b_ok = osal_task_create(main_glasses_stub_audio_task,
                                 MAIN_GLASSES_STUB_STACK_WORDS,
                                 RTE_PRIORITY_MEDIUM_OFFSET, NULL) &&
                osal_task_create(main_glasses_stub_ble_task,
                                 MAIN_GLASSES_STUB_STACK_WORDS,
                                 RTE_PRIORITY_BLE_OFFSET, NULL) &&
                (rte_start());

    return (b_ok ? E_OK : E_NOMEM);
}

/*!
 * @brief BRING-UP ONLY: Audio Alert task made of Brian's alert.c.
 *
 * Reads the radar messages, passes them to alert.c and runs alert_update()
 * every MAIN_GLASSES_ALERT_TICK_MS. Still prints each message so the serial
 * captures read the same. Brian replaces this task with his own.
 *
 * @param[in] p_arg  Unused.
 */
static void
main_glasses_stub_audio_task (void * p_arg)
{
    rte_audio_msg_t msg;

    // The void cast only silences the unused-parameter warning.
    (void) p_arg;

    alert_init(BOARD_BUZZER_PIN);

    for (;;)
    {
        if (rte_read_audio_msg(&msg, MAIN_GLASSES_ALERT_TICK_MS))
        {
            uint32_t d_mm = msg.d_mm;

            switch (msg.type)
            {
                case RTE_AUDIO_OBSTACLE:
                    // cue: bit 0 STOP, bits 1-2 side (0 center, 1 left,
                    // 2 right). See rte_type.h RTE_CUE_*.
                    gr_printf(">>> Audio Alert <- OBSTACLE(%" PRIu32
                              "mm) cue=0x%02" PRIX32 "\n",
                              d_mm, (uint32_t) msg.cue);
                    break;

                case RTE_AUDIO_OBSTACLE_CLEAR:
                    gr_printf(">>> Audio Alert <- OBSTACLE(CLEAR)\n");
                    break;

                case RTE_AUDIO_RADAR_FAULT:
                    gr_printf(">>> Audio Alert <- RADAR_FAULT\n");
                    break;

                case RTE_AUDIO_RADAR_OK:
                    gr_printf(">>> Audio Alert <- RADAR_OK\n");
                    break;

                default:
                    break;
            }

            audio_alert_bridge_apply(&msg);
        }

        alert_update(hal_time_now_ms());
    }
}

/*!
 * @brief BRING-UP ONLY: print what the BLE task would receive.
 *
 * @param[in] p_arg  Unused.
 */
static void
main_glasses_stub_ble_task (void * p_arg)
{
    rte_ble_obstacle_t msg;

    // The void cast only silences the unused-parameter warning.
    (void) p_arg;

    for (;;)
    {
        if (rte_read_ble_obstacle(&msg, RTE_WAIT_FOREVER))
        {
            if (!msg.b_active)
            {
                gr_printf(">>> BLE task <- obstacle CLEAR\n");
            }
            else
            {
                // OPEN-25 (14 Nov): one line per lane, not one combined
                // distance/TTC. 18 Nov: a lane that is not BLOCKED
                // (RTE_LANE_NONE_MM) now prints "clear" instead of the
                // raw 0xFFFF sentinel as "65535mm"/"65535ms" - that read
                // as "ACTIVE ... NONE" on the same line, which looks
                // self-contradictory on the bench. "ACTIVE"/"CLEAR"
                // above already says whether an alert is happening at
                // all; each lane line below only says what that one
                // lane itself is doing.
                if (RTE_LANE_NONE_MM == msg.d_mm[RTE_SIDE_LEFT])
                {
                    gr_printf(">>> BLE task <- L clear\n");
                }
                else
                {
                    gr_printf(">>> BLE task <- L d=%" PRIu32 "mm TTC=%"
                              PRIu32 "ms\n",
                              (uint32_t) msg.d_mm[RTE_SIDE_LEFT],
                              (uint32_t) msg.ttc_ms[RTE_SIDE_LEFT]);
                }

                if (RTE_LANE_NONE_MM == msg.d_mm[RTE_SIDE_CENTER])
                {
                    gr_printf(">>> BLE task <- M clear\n");
                }
                else
                {
                    gr_printf(">>> BLE task <- M d=%" PRIu32 "mm TTC=%"
                              PRIu32 "ms\n",
                              (uint32_t) msg.d_mm[RTE_SIDE_CENTER],
                              (uint32_t) msg.ttc_ms[RTE_SIDE_CENTER]);
                }

                if (RTE_LANE_NONE_MM == msg.d_mm[RTE_SIDE_RIGHT])
                {
                    gr_printf(">>> BLE task <- R clear\n");
                }
                else
                {
                    gr_printf(">>> BLE task <- R d=%" PRIu32 "mm TTC=%"
                              PRIu32 "ms\n",
                              (uint32_t) msg.d_mm[RTE_SIDE_RIGHT],
                              (uint32_t) msg.ttc_ms[RTE_SIDE_RIGHT]);
                }
            }
        }
    }
}

/*** end of file ***/
