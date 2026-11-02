# Target gates: corridor gate and jump gate

Status: IMPLEMENTED on the PC (27 Oct 2026), not yet tested on the Pico.
Related: NFR6 (alert within 100 ms of the radar frame), docs/deviations.md
OPEN-5, docs/radar_message_layout.md.

Neither gate smooths or averages a reading, and neither waits for a second
frame before an alert STARTS, so neither adds delay to NFR6.

## Corridor gate (Glasses_Radar, `radar_monitor_on_frame`)

A target with |x| above `RADAR_MONITOR_CORRIDOR_HALF_MM` (1200 mm since 31 Oct;
1000 mm before) is sent to Glasses_Threat as an empty slot. A target exactly on
the edge is kept.

The corridor is relative to the sensor, so it turns with the head. It assumes
the head faces the walking direction. With a head turn the walking path can
fall partly outside it: for an obstacle 1.8 m ahead the corridor misses it at
about 42 degrees of head turn (1200 mm) against about 34 degrees (1000 mm);
at 1.5 m ahead about 53 degrees against 42 (geometry only, not measured).

- Decided on one frame, no state, no delay.
- Removes people passing at the side and side walls from the alert.
- The `[T]` debug print still shows every target, so the width can be tuned.
- The frame itself is still forwarded, so the 500 ms fault timer is unaffected.
- Walk log 2026-10-27 (460 frames): |x| above 800 mm in 45 frames (25 inside
  2000 mm range), above 1000 mm in 14 (13), above 1200 mm in 9 (8). The walker
  was about 0.8 m left of the sensor for part of the walk, so 800 mm would
  have silenced those frames. 1000 mm was the first value. It was widened to
  1200 mm on 31 Oct (a missed obstacle is worse than a false alert, and the
  head-turn case above); 1200 mm ignores 8 of the 13 in-range frames that
  1000 mm ignored in that walk. One walk only: tune it with walking tests
  (target at the side, target crossing the path, head turned).

## Jump gate (Glasses_Threat, `threat_assessment_jump_gate`)

While an alert is active, if the nearest range grows by MORE than
`THREAT_ASSESSMENT_JUMP_AWAY_MM` (1000 mm) in one frame (and the previous
sample is at most 400 ms old), that frame is ignored once: the alert, the
distance and the history stay as they were. If the next frame still shows the
far range it is accepted. A change of exactly 1000 mm is not a jump.

- Away only. A jump toward the user is never held back, so a nearer target
  that appears starts or updates the alert on the same frame.
- Does not apply when no alert is active.
- Its only effect: a one-frame false CLEAR is delayed by one frame (about
  90 ms), which is the safe direction. The 500 ms no-target hold and the
  hysteresis still decide the rest.
- It cannot remove a one-frame false START (a low outlier). That would need a
  second frame and so a delayed start; see the decision in the project notes.

## Far-target hold (Glasses_Threat, `threat_assessment_far_target_must_wait`)

Added 1 Nov (report D4). FR10 ends the alert when the nearest non-departing
target is beyond 2100 mm. Before this change, if the near target was flagged
departing for one frame (sensor speed above 10 cm/s, or the range trend) and
another valid target beyond 2100 mm was inside the corridor, that far target
became "the nearest non-departing target" and the alert ended on the same
frame. Now, while an alert is on and the near target is still inside 2100 mm,
such a frame is handled like a frame with no target: the alert ends after 2
trend-departing frames in a row or after the 500 ms hold, as it does when no
other target is present.

- Why: in the 1 Nov soak (docs/evidence/2026-11-01_soak_738s.txt) a hand held
  at about 0.4 m with a target at about 3.5 m gave 67 CLEAR lines; 22 were
  followed by a new alert within 300 ms and 10 after one frame, caused by
  one-frame sensor speed spikes while the range did not change.
- Replaying that soak through the code (frames parsed from the log, corridor
  gate applied, distances recomputed from x and y, so approximate): CLEARs
  68 to 44, new alert within 300 ms 23 to 9, one-frame gaps 11 to 2. The old
  code gave 68, 23 and 11 in the replay against 67, 22 and 10 in the log.
- Cost: a real exit with a far target present clears one frame (about 90 ms)
  later.
- Does not apply when the near target is already beyond 2100 mm, when the other
  target is inside 2100 mm (the alert then follows that target), or when the
  near target disappears from the radar altogether.

## Speed confirmation (Glasses_Threat, `threat_assessment_range_confirms_speed`)

Added 1 Nov (report D5). The sensor speed alone can read +16 to +79 cm/s on a
target that is only swaying or barely moving, and it can lead the position by
about half a second (1 Nov sway and retreat captures). Before this change a
speed above 10 cm/s for 500 ms ended the alert even when the range had not
changed. Now the nearest target counts as departing by speed only if its range
also grew faster than `THREAT_ASSESSMENT_SPEED_CONFIRM_CMS` (5 cm/s, half the
limit) over the trend window. When there is not enough or too old history
(fewer than `THREAT_ASSESSMENT_TREND_FRAMES` earlier frames, or the older
sample is more than 400 ms old) the speed verdict is kept as before. Targets
other than the nearest one keep the speed rule unchanged.

- Replaying every 1 Nov capture (soak, hand tests, retreats, sway): CLEARs 79
  to 68; of the 10 that disappear, 9 were followed by the alert coming straight
  back. Sway run 1: 5 to 1 (the remaining one is the range trend). Fast exits
  (about 12 to 14 cm/s) clear at the same time as before. 16 CLEARs happen
  later, mostly under 1 s and at most about 1.3 s (the end of the hold-hand
  test took 1.6 s). The result barely changes for a confirmation value of 3 to
  8 cm/s. The replay is approximate: distances are recomputed from x and y.
- Cost: the beep can last a little longer after something really leaves; fewer
  drop-outs while an obstacle is still there.
- Not covered: a target that really is still but moves its range by more than
  5 cm/s over 4 frames is still treated as leaving.

## Tests

- tests/unit/test_radar_monitor.c: gate edge (kept at the limit, dropped one
  mm beyond it on either side; the tests use the config value, so they follow
  it), frame and timer unaffected, caller's array unchanged.
- tests/unit/test_threat_assessment.c: hold then accept, spike that returns,
  jump toward the user, limit and long gap, hold timer.
  Far-target hold: one noisy speed frame, a speed-departing hand and the
  500 ms hold, a real exit after 2 trend frames, a near target already beyond
  2100 mm.
  Speed confirmation: constant range with high sensor speed keeps the alert,
  speed with range growth still ends it after the hold, swaying with no far
  target keeps the alert.

## Still to verify on the Pico

- Alert still starts on the first frame below 2000 mm (NFR6 probe).
- Side-pass test: a person walking past at 1.4 m to the side gives no alert
  (the limit is now 1.2 m; also try 1.1 m to see the nuisance case).
- Head-turn test: walk toward an obstacle with the head turned 30 to 45 degrees
  and note when the alert is lost.
- Cross-path test: a person stepping into the path gives an alert.
- Sway test again with this build: lean about 5 cm for 30 s at about 1 m;
  count the CLEAR lines (run 1 of 1 Nov had 5; the replay predicts about 1).
- Far-target hold: a hand held at about 0.4 m with a target at about 3.5 m in
  the corridor; count the CLEAR lines over 60 s (the 1 Nov soak had 67 in 738 s
  with the old code).
