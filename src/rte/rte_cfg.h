/** @file rte_cfg.h
 *
 * @brief Compile-time sizes and priorities of the glasses RTE.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * WARNING: Radar, Threat and Audio Alert share the Medium priority. The
 * head-fall task must be configured HIGHER and the BLE task LOWER (NFR9).
 */

#ifndef RTE_CFG_H
#define RTE_CFG_H

#include "osal.h"

// Queue depths in items.
#define RTE_RADAR_QUEUE_LENGTH          (4u)
#define RTE_AUDIO_QUEUE_LENGTH          (8u)
#define RTE_BLE_QUEUE_LENGTH            (4u)

// Task stack sizes in words.
#define RTE_RADAR_TASK_STACK_WORDS      (1024u)
#define RTE_THREAT_TASK_STACK_WORDS     (512u)

// Abstract priority levels (a higher number is more urgent, see osal.h).
#define RTE_PRIORITY_BLE_OFFSET         (1u)
#define RTE_PRIORITY_MEDIUM_OFFSET      (2u)
#define RTE_PRIORITY_HEADFALL_OFFSET    (3u)

// How long a critical message (CLEAR, FAULT, OK) may wait for queue space.
// The per-frame OBSTACLE message never waits: the next frame replaces it.
#define RTE_CRITICAL_WAIT_MS            (10u)

// Timeout value meaning "block until a message arrives".
#define RTE_WAIT_FOREVER                (OSAL_WAIT_FOREVER)

#endif /* RTE_CFG_H */

/*** end of file ***/
