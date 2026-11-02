/** @file audio_alert_bridge.h
 *
 * @brief BRING-UP ONLY: maps radar messages onto Brian's alert.c calls.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Stand-in for the Audio Alert task until its owner (Brian) integrates his
 * own task. Pure logic, so the host tests can run it.
 */

#ifndef AUDIO_ALERT_BRIDGE_H
#define AUDIO_ALERT_BRIDGE_H

#include "rte_type.h"

void audio_alert_bridge_apply (const rte_audio_msg_t * p_msg);

#endif /* AUDIO_ALERT_BRIDGE_H */

/*** end of file ***/
