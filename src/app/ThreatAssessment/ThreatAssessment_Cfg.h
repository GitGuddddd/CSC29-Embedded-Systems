#ifndef THREATASSESSMENT_CFG_H
#define THREATASSESSMENT_CFG_H

/*
 * \file ThreatAssessment_Cfg.h
 * \brief Compile-time parameters for mmWave radar obstacle decision
 * Compliance: Barr-C:2018 (FR8, FR13)
 */

/* Per-lane START/STOP hysteresis. Derived from the "ideal warning" in Tourki et al.,
 * "Probabilistic Collision Risk Estimation for Pedestrian Navigation" (arXiv 2506.16219,
 * Sec. III-D, from interviews with vision-impaired users and mobility trainers): warn at
 * least 3 s before an object enters a 1 m safety bubble, and keep warning until it is
 * outside 2 m. At a slow assistive walking pace of about 0.7 m/s (gait speeds below
 * 1.0 m/s are common in older adults) that is 1 m + 3 s * 0.7 m/s = ~3 m, which is
 * within the LD2450's 6 m range. Previously the team used 2000/2100 mm. */
#define THREAT_ALERT_ENTER_DIST_MM   (3000U)
#define THREAT_ALERT_CLEAR_DIST_MM   (3100U)

/*
 * STOP cue ("about to collide"): set when the centre lane (the user's own lane)
 * is blocked and its obstacle is within the STOP distance. This also covers all
 * three lanes blocked, since the centre is then blocked too.
 *
 * The STOP distance grows with the closing speed v so a faster approach is warned
 * earlier (ported from the earlier test-branch patch "Scale STOP distance with
 * closing speed"):
 *     D = MARGIN + v * DELAY + v^2 / (2 * DECEL)
 * clamped to [THREAT_STOP_ENTER_DIST_MM, THREAT_STOP_MAX_DIST_MM]. DELAY covers the
 * user's reaction (about 0.5 s) plus radar frame, lane hold and cue dwell (about
 * 0.35 s, estimated). DECEL is the braking seen on two stand walks (about 0.8 m/s^2).
 * MARGIN is an engineering estimate. D is fixed when STOP latches and STOP clears
 * THREAT_STOP_HYST_MM beyond it. With no closing speed D is the 1500 mm floor.
 * Examples: 0.8 m/s -> 1580 mm, 1.0 m/s -> 1975 mm, 1.1 m/s -> 2191 mm,
 * 1.4 m/s or faster -> capped at 2800 mm. Tune on the real walking test.
 */
#define THREAT_STOP_ENTER_DIST_MM    (1500U)  /* floor: STOP distance at low speed */
#define THREAT_STOP_MAX_DIST_MM      (2800U)  /* ceiling; below THREAT_ALERT_ENTER_DIST_MM */
#define THREAT_STOP_HYST_MM          (100U)
#define THREAT_STOP_CLEAR_DIST_MM    (THREAT_STOP_ENTER_DIST_MM + THREAT_STOP_HYST_MM)
#define THREAT_STOP_MARGIN_MM        (500U)
#define THREAT_STOP_DELAY_MS         (850U)   /* reaction 500 + system 350 */
#define THREAT_STOP_DECEL_MMPS2      (800U)
#define THREAT_STOP_SPEED_CAP_CMS    (300U)   /* speeds above this give the ceiling anyway */


/*
 * Departing obstacles: when every blocked lane's obstacle has been moving away at
 * THREAT_RECEDE_MIN_CMS or more for THREAT_RECEDE_FRAMES radar frames in a row,
 * ThreatAssessment_IsReceding() is true and the glasses keep the
 * buzzer quiet. 16 cm/s is two LD2450 speed steps, above its jitter; 3 frames = 300 ms.
 * The lanes, STOP and BLE are unchanged; an obstacle that stops or turns back beeps again.
 */
#define THREAT_RECEDE_MIN_CMS        (16)
#define THREAT_RECEDE_FRAMES         (3U)

/*
 * ... and the gap must also have grown by this much since the run began. The LD2450
 * reports ~8 cm/s speed steps and jitters by +/-16-24 cm/s on a person standing or
 * walking slowly, so three negative speed readings alone happen by chance (bench log:
 * a person 2.0-2.2 m ahead walking slowly towards the radar went silent every other
 * second). Distance jitter is ~30 mm, so 150 mm is a real departure: ~0.15 s at a
 * normal walk, ~0.4 s at a slow one.
 */
#define THREAT_RECEDE_MIN_GAIN_MM    (150U)

/*
 * A STOP is made quiet the same way when its obstacle is moving away (the radar
 * measures relative speed, so a growing gap means the wearer is not closing on it),
 * but never while the obstacle is within this distance. Bench log: a person walking
 * off from 0.35 m kept the STOP tone on for ~1.5 s until they passed 1.6 m.
 */
#define THREAT_STOP_RECEDE_MIN_MM    (500U)

/*
 * A side lane is only offered as "free" (left/right cue) after it has been clear
 * for this long. "Clear" means nothing was detected, which is weaker than
 * "empty" (stationary targets drop out), so a lane that has only just emptied
 * is not trusted yet.
 */
#define THREAT_FREE_LANE_CLEAR_MS    (500U)

/*
 * Radar mounting: set to 1 if the radar is fitted so that its +x axis points to
 * the user's LEFT. The default (0) treats negative x as left. Check with the
 * left/right test in the testing steps whenever the radar is re-fitted.
 */
#define THREAT_MIRROR_X              (0)

/*
 * HLK-LD2450 speed sign convention.
 * 0: a positive speed_cms means approaching (team convention, same as before).
 * 1: a negative speed_cms means approaching (putra-branch bench assumption).
 * The LD2450 manual (v1.03, Table 10) defines only the encoding, not which
 * direction is positive. CONFIRMED on hardware (debug log, stand test): the radar
 * reports positive speed while the target moves away, so the value is 1 and the
 * sign is inverted. If a future radar or firmware differs, re-check with a walk test and
 * change only this value if the result is opposite.
 *
 * No target is excluded for moving away (putra-branch OPEN-22): a departing
 * target still occupies its lane until it has physically cleared
 * THREAT_ALERT_CLEAR_DIST_MM and the lane hold has elapsed.
 */
#define THREAT_NEGATIVE_SPEED_IS_APPROACHING (1)

#define THREAT_RADAR_TIMEOUT_MS      (500U)

#define THREAT_CORRIDOR_HALF_MM      (1500)
#define THREAT_SIDE_DEADZONE_MM      (425)
/* A body is several radar points and the LD2450 drops still targets for 0.3-0.4 s,
 * so with a person near a lane edge the points flicker between lanes. The wider
 * hysteresis (was 75 mm) keeps a point in its lane, and the longer lane hold (was
 * 200 ms) keeps a blocked lane blocked across a brief dropout instead of
 * toggling the cue. Bench log, 2-2.5 m, slow/still: 77 cue changes in 250 s. */
#define THREAT_SIDE_HYSTERESIS_MM    (150)
#define THREAT_LANE_HOLD_MS          (600U)
/* The LD2450 also loses people who stand still for longer than that, and more often
 * with several in view (bench log, three people standing 1.5-3 m ahead: never more than
 * two lanes seen at once, guidance switching left/right every second or so). A lane
 * whose nearest obstacle was standing still when last seen (closing speed within
 * THREAT_STILL_MAX_CMS, one LD2450 speed step either side of 0 plus jitter, and
 * sideways speed within THREAT_STILL_MAX_LATERAL_MMPS) is held this long instead.
 * Someone who leaves a lane moves first, so they still clear after the normal hold;
 * so does a person swept out of view by the wearer turning their head. */
#define THREAT_STILL_LANE_HOLD_MS    (1500U)
#define THREAT_STILL_MAX_CMS         (16)
#define THREAT_STILL_MAX_LATERAL_MMPS (300)
/* Held closing speed (TTC) across a dropout; unrelated to the lane hold above. */
#define THREAT_SPEED_HOLD_MS         (200U)

/*
 * Crossing filter. A person walking across the user's path at normal pace spends
 * under a second in each lane, so a plain minimum-time-in-lane would need to be too
 * long. Instead, a target farther than THREAT_STOP_CLEAR_DIST_MM whose smoothed sideways
 * speed is at least THREAT_CROSSING_MIN_MMPS and more than THREAT_CROSSING_RATIO times
 * its forward speed is not counted. Inside the STOP zone every target counts, and so does
 * a target whose sideways speed is not known yet (the filter needs it to have been tracked
 * for two frames, for example while approaching from outside the corridor). Set THREAT_CROSSING_FILTER to 0 to disable. Needs walking-test tuning.
 */
#define THREAT_CROSSING_FILTER       (1)
#define THREAT_CROSSING_MIN_MMPS     (500)
#define THREAT_CROSSING_RATIO        (2)
#define THREAT_CROSSING_SMOOTH_MS    (200)
#define THREAT_CROSSING_MAX_GAP_MS   (300U)
/* Sideways/forward speed samples are clamped to this (16 m/s, far above walking) so the
 * 16-bit state and 32-bit products cannot overflow on a corrupt or jumping frame. */
#define THREAT_CROSSING_SPEED_CLAMP_MMPS (16000)

/*
 * The LD2450 re-orders its three slots when a target appears or drops out, so a slot
 * can suddenly hold a different person. A slot whose position moves further than
 * THREAT_TRACK_JUMP_MARGIN_MM + THREAT_TRACK_MAX_MMPS * elapsed time is treated as a
 * new target: its crossing-speed history and side hysteresis are reset instead of
 * reading the jump as fast sideways motion (which hid the target as "crossing").
 * 4 m/s is above jogging pace; at 10 Hz the limit is 550 mm per frame.
 */
#define THREAT_TRACK_MAX_MMPS        (4000)
#define THREAT_TRACK_JUMP_MARGIN_MM  (150)

#endif /* THREATASSESSMENT_CFG_H */
