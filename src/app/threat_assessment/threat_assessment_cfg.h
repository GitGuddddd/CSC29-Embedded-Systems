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

// FR9 cue byte (prof feedback, 7 Nov): STOP cue sub-hysteresis, independent
// of THREAT_ASSESSMENT_START_MM/STOP_MM (those decide whether the alert is on
// at all; these decide whether it is saying STOP). Decided 7-8 Nov: raised
// from 500/600 to 900/1100 after bench listening. ON is checked every frame;
// OFF only after the minimum hold has elapsed (it delays the end, never the
// start): a target closing then slowing at the edge must not flap the cue.
#define THREAT_ASSESSMENT_STOP_CUE_ON_MM       (900u)
#define THREAT_ASSESSMENT_STOP_CUE_OFF_MM      (1100u)
#define THREAT_ASSESSMENT_STOP_CUE_MIN_HOLD_MS (500u)

// MIO tie-break (8 Nov, OPEN-16): when the LD2450 reports two valid, non-
// departing targets whose distances fall within this band of each other,
// plain "nearest wins" no longer has a clear answer (see the HLK-LD2450 MIO
// checklist, "tie-breaking rules are defined"). Within the band: the target
// with a real (approaching) TTC beats one with none; if both or neither have
// a real TTC, the side already being reported wins, to avoid flicker between
// two similarly-risky targets; outside the band, strictly-nearest still
// decides exactly as before. DRAFT: the 100 mm width is a guess, not bench-
// measured (two simultaneous equidistant obstacles are rare to stage on the
// bench); only applied to final target selection, not to the jump-gate/trend
// "any valid target" scan, which is unrelated to which target gets reported.
#define THREAT_ASSESSMENT_MIO_TIE_BAND_MM      (100u)

// Collision risk assessment (8 Nov, OPEN-17, HLK-LD2450 MIO checklist
// section 3): MIO selection by raw distance alone cannot tell a target
// walking across the corridor from one standing still at the same range,
// and a farther target closing head-on is more urgent than a nearer one
// going nowhere. These four constants drive a lightweight 2D motion check
// (see threat_assessment_predicts_collision()) that runs ONLY in final MIO
// selection, alongside (not instead of) the existing distance/TTC/hysteresis
// machinery above.
//
// Frame-to-frame association gate: the LD2450 does not promise slot 0/1/2
// are the same physical object between frames (checklist section 5, "radar
// target slots are not assumed to represent stable identities"), so each
// target is matched to the closest previous-frame position within this
// radius; no match within it means "no velocity this frame", not "match
// slot 0 anyway". Sized a little above one frame's worth of fast walking
// motion (~1.5 m/s at the ~100 ms radar interval is ~150 mm) with margin for
// jitter; DRAFT, not bench-measured against an actual crossing target.
#define THREAT_ASSESSMENT_ASSOC_GATE_MM        (400u)

// Predicted clearance at closest approach: a target whose straight-line
// path brings it within this radius of the sensor counts as "on a collision
// course" even if its current distance is still large. Matches the side
// deadzone's reasoning (OPEN-15): a user-width margin, not a point target.
// DRAFT.
#define THREAT_ASSESSMENT_SAFETY_RADIUS_MM     (400u)

// A predicted closest approach further out than this (in time) is not
// trusted: two frames of noisy position give a noisy velocity, and a
// far-future prediction from that is more guess than signal. 3 s is a few
// steps away at walking pace.
#define THREAT_ASSESSMENT_TCA_HORIZON_MS       (3000u)

// How much a predicted collision course reduces a target's effective
// selection distance, so it can outrank a nearer but safe target (checklist
// section 9: "tests confirm a farther approaching target can outrank a
// nearer safe target"). A flat bonus, not a continuous score: keeps the
// comparison simple and auditable rather than a tuned weighted formula.
// DRAFT: large enough to flip a typical "nearer-but-safe vs farther-but-
// closing" case, not bench-verified against a real crossing walk.
#define THREAT_ASSESSMENT_COLLISION_BONUS_MM   (600u)

// Persistent track IDs (8 Nov, OPEN-18, checklist section 6 "targets are
// associated across consecutive frames"): a track that goes unmatched for
// this many frames in a row is dropped, freeing its slot. Kept small - a
// dropped track's old position can otherwise get re-claimed by an unrelated
// new target that happens to appear nearby, silently inheriting history
// that is not really its own. 3 frames (~300 ms at the ~100 ms radar
// interval) survives one or two noisy/invalid frames without surviving so
// long that a genuinely different object picks up stale history. DRAFT.
#define THREAT_ASSESSMENT_TRACK_LOST_GRACE_FRAMES (3u)

#endif /* THREAT_ASSESSMENT_CFG_H */

/*** end of file ***/
