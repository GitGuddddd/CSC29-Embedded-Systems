/** @file threat_assessment_cfg.h
 *
 * @brief Compile-time thresholds of the obstacle decision.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Source: Group 29 Week 6 design review, FR8, FR10, NFR8 and assumptions
 * A6 and A7. All values are drafts to be tuned in walking tests.
 *
 * OPEN-22 (13 Nov): this file was rewritten end to end for the per-lane
 * occupancy redesign. MIO target selection, 2D collision prediction,
 * frame-to-frame track IDs and the multi-frame departing-target trend
 * filters are all gone - see docs/deviations.md OPEN-22 for the full
 * reasoning. Only the hysteresis and hold-timer constants survive, renamed
 * and re-valued for per-lane rather than single-target use.
 */

#ifndef THREAT_ASSESSMENT_CFG_H
#define THREAT_ASSESSMENT_CFG_H

// FR8 (per lane, OPEN-22): a lane becomes BLOCKED once any valid target in
// it is closer than this. Tighter than the old single-target 2000 mm
// (OPEN-16 through OPEN-21) because the sensor is head-mounted: a slight
// upward head tilt can bring a high wall or a ceiling edge into range, and
// a tighter look-ahead keeps that from reading as an obstacle.
#define THREAT_ASSESSMENT_START_MM          (1800u)

// NFR8 (per lane, OPEN-22): a lane clears back to open only once every
// valid target in it is farther than this. Same reasoning as START_MM: a
// tighter band than the old 2100 mm so head tilt does not trigger it.
#define THREAT_ASSESSMENT_STOP_MM           (1950u)

// Per-lane hold timer (OPEN-22, replaces THREAT_ASSESSMENT_
// NO_TARGET_HOLD_MS, which was 500 ms and applied only when every target
// vanished at once). Each of the 3 lanes now carries its own hold timer:
// a lane's BLOCKED state is only cleared once it has read empty (no valid
// target inside THREAT_ASSESSMENT_STOP_MM) for this long. 200 ms (vs. the
// old 500 ms) because the user scans by turning their head, and feedback
// that lags a head turn by half a second reads as wrong far more often
// than a single-frame radar dropout needs filtering - 200 ms is long
// enough to swallow a one-frame dropout at the sensor's ~100 ms frame
// interval without making head-scanning feel sluggish. DRAFT, not bench-
// measured.
#define THREAT_ASSESSMENT_LANE_HOLD_MS      (200u)

// Time to collision in ms = distance in mm * 100 / closing speed in cm/s.
// OPEN-22 removed all per-target velocity tracking (so FR13 was briefly
// unmet, ttc_ms always RTE_TTC_NONE_MS); OPEN-23 (13 Nov) then OPEN-25
// (14 Nov) restored it, per lane, from each BLOCKED lane's own live
// speed_cms this frame (threat_assessment_scan_lanes() /
// threat_assessment_lane_out()), never from frame-to-frame position
// tracking. See docs/deviations.md OPEN-22, OPEN-23, OPEN-25.
#define THREAT_ASSESSMENT_TTC_FACTOR        (100u)

// FR9 pitch and beep pattern are NOT defined here. They are owned by the
// Audio Alert code (Brian's alert.c): pitch 1000 + (2000 - d) * 2 Hz and a
// beep period of 150 + d * 400 / 2000 ms (agreed 2 Nov). Threat only sends the
// distance.

// REMOVED (OPEN-22): THREAT_ASSESSMENT_STOP_CUE_ON_MM/_OFF_MM/_MIN_HOLD_MS,
// a near-distance STOP sub-hysteresis on the single MIO target's distance.
// It was already dormant (unused by the FR9 cue byte) since OPEN-20, and
// had no single-target distance left to operate on once MIO selection was
// removed, so it is deleted rather than carried forward unused.

#endif /* THREAT_ASSESSMENT_CFG_H */

/*** end of file ***/
