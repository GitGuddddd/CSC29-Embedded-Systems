/** @file main_glasses.c
 *
 * @brief Glasses launcher: creates the radar objects and tasks.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * WARNING: The two stub tasks only print what the Audio Alert and BLE tasks
 * would receive. The owners of those modules replace them.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "gr_print.h"
#include "main_glasses.h"
#include "osal.h"
#include "rte.h"
#include "rte_cfg.h"
#include "rte_type.h"
#include "threat_assessment.h"

#define MAIN_GLASSES_STUB_STACK_WORDS    (512u)

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
 * @brief BRING-UP ONLY: print what the Audio Alert task would receive.
 *
 * @param[in] p_arg  Unused.
 */
static void
main_glasses_stub_audio_task (void * p_arg)
{
    rte_audio_msg_t msg;

    // The void cast only silences the unused-parameter warning.
    (void) p_arg;

    for (;;)
    {
        if (rte_read_audio_msg(&msg, RTE_WAIT_FOREVER))
        {
            uint32_t d_mm = msg.d_mm;
            uint32_t pitch_hz = threat_assessment_pitch_hz(d_mm);

            switch (msg.type)
            {
                case RTE_AUDIO_OBSTACLE:
                    gr_printf(">>> Audio Alert <- OBSTACLE(%" PRIu32
                              "mm) pitch %" PRIu32 " Hz\n",
                              d_mm, pitch_hz);
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
        }
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
            uint32_t d_mm = msg.d_mm;
            uint32_t ttc_ms = msg.ttc_ms;

            if (!msg.b_active)
            {
                gr_printf(">>> BLE task <- obstacle CLEAR\n");
            }
            else
            {
                gr_printf(">>> BLE task <- obstacle ACTIVE d=%" PRIu32
                          "mm TTC=%" PRIu32 "ms\n",
                          d_mm, ttc_ms);
            }
        }
    }
}

/*** end of file ***/
