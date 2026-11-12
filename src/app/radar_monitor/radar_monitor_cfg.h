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
// 1500 mm (was 1200 mm until 8 Nov, 1000 mm until 31 Oct): a missed obstacle
// is worse than a false alert, and the corridor turns with the head, so a
// wider strip hides fewer real obstacles when the head is turned. Reframed
// 8 Nov from a narrow cane-sweep model to a full walking-lane model: half
// of a roughly 2.5-3 m single-person walking lane plus clearance, rather
// than just the cane's own sweep arc.
#define RADAR_MONITOR_CORRIDOR_HALF_MM      (1500)

// FR9 cue byte (prof feedback, 7 Nov): a target within this far of the
// sensor's centre line counts as straight ahead (RTE_SIDE_CENTER); beyond it,
// LEFT or RIGHT by the sign of x_mm. Bench-confirmed 7 Nov: negative x is
// left (sensor fixed, walked left to right), so x_mm < 0 is LEFT.
// 300 mm (was 150 mm until 8 Nov): widened after bench use showed CENTER
// standing still read as LEFT. Justified as the sum of three margins the
// sensor has to absorb even when the user is genuinely centred: natural
// lateral sway while walking (roughly 50-100 mm), radar lateral-measurement
// jitter at typical use range (tens of mm, grows with distance), and
// mounting/alignment tolerance on the glasses frame (a few degrees of yaw
// at 1-2 m is real mm of offset). Still a DRAFT value: the sway and jitter
// components are estimates, not bench-measured, so confirm by walking the
// corridor before the report locks it in. Kept well inside
// RADAR_MONITOR_CORRIDOR_HALF_MM so a target can be CENTER only while it is
// also in the corridor.
#define RADAR_MONITOR_SIDE_DEADZONE_MM      (300)

#endif /* RADAR_MONITOR_CFG_H */

/*** end of file ***/
