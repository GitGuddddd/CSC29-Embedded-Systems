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
// walking tests (see docs/testing_guide.md). The debug print still shows
// the target.
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
// 425 mm (OPEN-22, 13 Nov; was 300 mm, was 150 mm until 8 Nov): widened again
// for the lane-occupancy redesign. The same three margins as before (natural
// lateral sway while walking, radar lateral jitter, mounting/alignment
// tolerance) still apply, but OPEN-22 also removed the per-side departing-
// target filter, so a lane now stays BLOCKED until every target in it
// physically clears THREAT_ASSESSMENT_STOP_MM rather than being excused
// early by a trend filter - a narrow lane makes that wait feel twitchy on
// every small head turn. The wider half-width absorbs that natural head sway
// so a walking corridor does not flicker between lanes on every footstep.
// Still a DRAFT value, not bench-measured against the new lane logic; kept
// well inside RADAR_MONITOR_CORRIDOR_HALF_MM so a target can be CENTER only
// while it is also in the corridor.
#define RADAR_MONITOR_SIDE_DEADZONE_MM      (425)

// OPEN-24 (13 Nov): a target sitting near a +/-RADAR_MONITOR_SIDE_DEADZONE_MM
// boundary, with x_mm drifting by a few mm frame to frame (sensor jitter,
// small head sway), could otherwise flip side every frame and make the FR9
// direction cue flicker - a lane now has a 200 ms hold on the way out
// (THREAT_ASSESSMENT_LANE_HOLD_MS) but a target's lane *membership* itself
// had no such hold. This is that hold, applied to the boundary itself
// (Schmitt-trigger style, same shape as the distance START/STOP pair):
// crossing INTO a side still only needs DEADZONE_MM, but once classified
// LEFT or RIGHT, leaving that side back toward CENTER (or past it to the
// far side) needs x_mm to come back inside DEADZONE_MM - HYSTERESIS_MM,
// not just DEADZONE_MM, so a target has to clearly leave, not merely touch
// the line. 75 mm (DRAFT, not bench-measured): a few times the lateral
// jitter a stationary target shows frame to frame, but small next to
// DEADZONE_MM (425) itself so it narrows a lane's effective width only a
// little. Per RTE_RADAR_NUM_TARGETS slot, not per physical target - the
// sensor does not promise a target keeps the same slot frame to frame
// (the same known, bounded risk noted for OPEN-18's removal in
// docs/deviations.md), so this hysteresis is occasionally applied to a
// slot that is now a different physical object than the frame before;
// with only 3 slots and a walking-paced user this is judged acceptable.
#define RADAR_MONITOR_SIDE_HYSTERESIS_MM    (75)

#endif /* RADAR_MONITOR_CFG_H */

/*** end of file ***/
