/** @file threat_assessment_cfg.h
 *
 * @brief Compile-time thresholds of the obstacle decision.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Source: Group 29 Week 6 design review, FR8, FR10, NFR8 and assumptions
 * A7 and A8. All values are drafts to be tuned in walking tests.
 */

#ifndef THREAT_ASSESSMENT_CFG_H
#define THREAT_ASSESSMENT_CFG_H

// FR8: the alert starts when the nearest non-departing target is closer.
#define THREAT_ASSESSMENT_START_MM          (2000u)

// NFR8: the alert stops only when that target is farther than this.
#define THREAT_ASSESSMENT_STOP_MM           (2100u)

// FR10: no non-departing target for this long ends the alert.
#define THREAT_ASSESSMENT_NO_TARGET_HOLD_MS (500u)

// FR13: a target moving away faster than this is departing and is ignored.
#define THREAT_ASSESSMENT_DEPARTING_CMS     (10)

// Range trend: a target whose range grew faster than THREAT_ASSESSMENT_
// DEPARTING_CMS over this many frames is also departing. The sensor speed
// is coarse (steps of about 8 cm/s) and lags, so it can read 0 while the
// target moves away (bench log 2026-10-18).
#define THREAT_ASSESSMENT_TREND_FRAMES      (3u)

// Samples older than this are not used for the trend (frames were lost).
#define THREAT_ASSESSMENT_TREND_MAX_AGE_MS  (400u)

// The range trend says the nearest target is leaving (and nothing else is
// left to alert on) for this many frames in a row: end the alert now
// instead of replaying the old distance for the 500 ms hold.
#define THREAT_ASSESSMENT_DEPART_CLEAR_FRAMES (2u)

// Time to collision in ms = distance in mm * 100 / closing speed in cm/s.
#define THREAT_ASSESSMENT_TTC_FACTOR        (100u)

// FR9: pitch rises linearly from 1500 Hz at 2000 mm to 3500 Hz at 300 mm.
// The beep is 150 ms on and 350 ms off. The fall alarm (2500 and 4000 Hz,
// 250 ms) and the fault tone (1 kHz) must stay distinguishable (A7).
#define THREAT_ASSESSMENT_PITCH_FAR_MM      (2000u)
#define THREAT_ASSESSMENT_PITCH_NEAR_MM     (300u)
#define THREAT_ASSESSMENT_PITCH_FAR_HZ      (1500u)
#define THREAT_ASSESSMENT_PITCH_NEAR_HZ     (3500u)
#define THREAT_ASSESSMENT_BEEP_ON_MS        (150u)
#define THREAT_ASSESSMENT_BEEP_OFF_MS       (350u)

#endif /* THREAT_ASSESSMENT_CFG_H */

/*** end of file ***/
