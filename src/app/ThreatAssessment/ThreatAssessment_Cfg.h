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
 * is blocked and its obstacle is within THREAT_STOP_ENTER_DIST_MM. This also
 * covers all three lanes blocked, since the centre is then blocked too. Clears
 * above THREAT_STOP_CLEAR_DIST_MM (hysteresis, so the tone does not flicker).
 * The 1 m safety bubble above is where a collision is deemed to start, so STOP
 * must fire before it: bubble 1000 mm + roughly 1 s of reaction and stopping at
 * 0.5-1.0 m/s = 1500 mm. This margin is an engineering estimate, not from the
 * paper; tune on the real walking test.
 */
#define THREAT_STOP_ENTER_DIST_MM    (1500U)
#define THREAT_STOP_CLEAR_DIST_MM    (1600U)

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
/* Held closing speed (TTC) across a dropout; unrelated to the lane hold above. */
#define THREAT_SPEED_HOLD_MS         (200U)

#endif /* THREATASSESSMENT_CFG_H */
