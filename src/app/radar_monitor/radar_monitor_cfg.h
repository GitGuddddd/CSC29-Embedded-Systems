/** @file radar_monitor_cfg.h
 *
 * @brief Compile-time timing of the radar supervision (FR11).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#ifndef RADAR_MONITOR_CFG_H
#define RADAR_MONITOR_CFG_H

// FR11: no valid frame for this long means RADAR_FAULT.
#define RADAR_MONITOR_FRAME_TIMEOUT_MS      (500u)

// The configuration is retried at this interval while in RADAR_FAULT.
#define RADAR_MONITOR_CONFIG_RETRY_MS       (2000u)

// INIT gives up and reports RADAR_FAULT after this many failed tries.
#define RADAR_MONITOR_CONFIG_MAX_TRIES      (3u)

// WARNING: Measured on the bench: the first frame can take longer than
// 500 ms to appear after "end config" is acknowledged. Without this grace
// the radar was re-initialised in a loop. Record it in assumption A5/A6.
#define RADAR_MONITOR_POST_CONFIG_GRACE_MS  (2000u)

#endif /* RADAR_MONITOR_CFG_H */

/*** end of file ***/
