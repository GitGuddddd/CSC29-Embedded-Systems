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
// the radar was re-initialised in a loop. Documented in report 6.6.
#define RADAR_MONITOR_POST_CONFIG_GRACE_MS  (2000u)

// After a successful configuration the sensor may go quiet for about a second
// (bench: frames, then 0.9 s of silence, then frames again). Within this time
// a 500 ms silence only clears the tone (Threat goes to CLEAR); it does not
// raise RADAR_FAULT. Silence at the end of this time is a fault as before.
#define RADAR_MONITOR_SETTLE_MS             (3000u)

// Corridor gate: a target whose side-to-side position |x| is larger than this
// is not in the walking path and is sent to Glasses_Threat as an empty slot.
// It costs no time: the decision is made on the frame itself. Tune with
// walking tests (docs/target_gates.md). The debug print still shows the target.
// 1200 mm (was 1000 mm until 31 Oct): a missed obstacle is worse than a false
// alert, and the corridor turns with the head, so a wider strip hides fewer
// real obstacles when the head is turned. Starting value from one walk.
#define RADAR_MONITOR_CORRIDOR_HALF_MM      (1200)

#endif /* RADAR_MONITOR_CFG_H */

/*** end of file ***/
