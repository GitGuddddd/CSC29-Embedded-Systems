/** @file threat_assessment_cfg.h
 *
 * @brief Compile-time thresholds of the obstacle decision.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Source: Group 29 Week 6 design review, FR8, FR10, NFR8 and assumptions
 * A6 and A7. All values are drafts to be tuned in walking tests.
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
// 4 frames (about 360 ms) since 1 Nov, was 3: a longer baseline so a swaying
// target is less likely to look like one that is leaving. Value to be settled
// by the Pico sway test. Keep THREAT_ASSESSMENT_HISTORY_LEN equal to this, or
// the trend never has enough history. 5 frames is about 450 ms, which is over
// the 400 ms cap below (that cap also limits the jump gate).
#define THREAT_ASSESSMENT_TREND_FRAMES      (4u)

// Speed confirmation: the sensor speed alone can read +16 to +79 cm/s on a target
// that is only swaying or barely moving (1 Nov captures), and it can lead the
// position by about half a second. The nearest target counts as departing by
// speed only if its range also grew faster than this over the trend window.
// When there is not enough history to judge, the speed verdict is kept.
#define THREAT_ASSESSMENT_SPEED_CONFIRM_CMS (5)

// Samples older than this are not used for the trend (frames were lost).
#define THREAT_ASSESSMENT_TREND_MAX_AGE_MS  (400u)

// The range trend says the nearest target is leaving (and nothing else is
// left to alert on) for this many frames in a row: end the alert now
// instead of replaying the old distance for the 500 ms hold.
#define THREAT_ASSESSMENT_DEPART_CLEAR_FRAMES (2u)

// Jump gate (away only): while an alert is active, a nearest range that grows
// by more than this in one frame is ignored once. If the next frame agrees it
// is accepted. A jump TOWARD the user is never held back, so a new nearer
// target starts or updates the alert at once (NFR6). Nobody walks 1 m in
// one frame, so this only catches a one-frame false clear.
#define THREAT_ASSESSMENT_JUMP_AWAY_MM      (1000u)
// Time to collision in ms = distance in mm * 100 / closing speed in cm/s.
#define THREAT_ASSESSMENT_TTC_FACTOR        (100u)

// FR9 pitch and beep pattern are NOT defined here. They are owned by the
// Audio Alert code (Brian's alert.c): pitch 1000 + (2000 - d) * 2 Hz and a
// beep period of 150 + d * 400 / 2000 ms (agreed 2 Nov). Threat only sends the
// distance.

#endif /* THREAT_ASSESSMENT_CFG_H */

/*** end of file ***/
