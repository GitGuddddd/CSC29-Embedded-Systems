# Draft report sentences for the gates (D2, D4)

Draft of 1 Nov, updated after the Pico tests of the same day. Wording only:
nothing here changes the code. Items in [square brackets] are still open. Edit freely before the
sentences go into FR8, FR10 and section 6.6 of the report.

## FR8 (valid target and departing target)

- Corridor gate: "A target more than 1200 mm to the side of the sensor is not
  valid for FR8."
- Departing by sensor speed (FR13, changed 1 Nov): "A target is departing by
  speed only if the sensor reports it moving away faster than 10 cm/s and, for
  the nearest target, its range has also grown faster than 5 cm/s over the last
  4 frames."
- Departing by range trend: "A target is also treated as departing when its
  range grows faster than 10 cm/s over the last 4 frames (about 360 ms)."

## FR10 (when the alert ends)

- Early clear: "A target that looks like it is leaving (sensor speed above
  10 cm/s, or the range trend above) does not end the alert on that frame
  alone, whether or not another target exists. The alert ends after 2 such
  frames in a row, or when no non-departing target has been seen for 500 ms."
- Jump gate: "While an alert is active, a one-frame jump of the nearest range of
  more than 1000 mm away is held for one frame; if the next frame agrees, it is
  accepted."

## Section 6.6 (assumptions and limits to state)

- Corridor: "The corridor is relative to the sensor, so it turns with the head
  and assumes the head faces the walking direction. By geometry (not measured),
  an obstacle 1.8 m ahead is hidden at about 42 degrees of head turn. An object
  more than 1.2 m to the side is ignored. The value was tuned from one walk and
  was confirmed on the Pico on 1 Nov with an angled approach: alerts started only once the target was inside 1200 mm to the side (last frame without an alert at x -1203, first with an alert at x -1199 mm). A separate head-turn test was not run; the angled approach covers it."
- Trend rule: "The sensor speed reads about +-8 cm/s at rest, close to the
  10 cm/s limit, so the range trend is used as well. A swaying target can be
  flagged as leaving and drop the alert briefly; a gap of about 260 ms was seen
  on 30 Oct with a 3-frame window. On the Pico with the 4-frame window, 5 CLEARs remained in a 44 s sway, with gaps of up to 180 ms. The comparison is ignored when the earlier sample is more than
  400 ms old."
- Far-target hold: "Before 1 Nov, a far target beyond 2100 mm inside the
  corridor could end the alert on the first frame in which the near target was
  flagged as leaving. In a 738 s test with a hand at about 0.4 m this gave 67
  CLEAR messages, 22 followed by a new alert within 300 ms. The alert now
  waits for 2 such frames or the 500 ms hold. A real exit with a far target
  present clears about 90 ms later. On the Pico (1 Nov, 35 s with a second target at about 3.5 m and a hand at 0.25 to 0.5 m) the four CLEAR messages came exactly where the new code predicts, and a clear caused by one +16 cm/s speed frame did not happen."
- Speed confirmation (D5): "The sensor speed alone read +16 to +79 cm/s on a
  swaying or barely moving target, and led the position by about half a second.
  In a 44 s sway test at about 1.4 m this ended the alert 5 times, with gaps of
  0.18 to 0.62 s. The speed is now only believed when the range also grows
  faster than 5 cm/s. Replaying the 1 Nov captures, CLEARs fell from 79 to 68
  and the sway test from 5 to 1; a real exit can clear up to about 1.3 s later.
  On the Pico (sway run 2) the speed-caused clears were gone; the 5 remaining CLEARs were trend-based, with gaps of up to 180 ms."
- Known limit (replaced by the above for the sway case): "If the sensor reports a speed above 10 cm/s for 500 ms while the target is really still, the alert can still end after the 500 ms hold if the range also grows by more than 5 cm/s."
- Not covered: "If the near target disappears from the radar for a frame while
  a far target exists, the alert still clears on that frame."
- Settle window (D1, needs team sign-off, OPEN-13): "For 3000 ms after a
  successful configuration, a 500 ms silence only clears the tone (Threat goes
  to CLEAR) and does not raise RADAR_FAULT. This was seen after a cold plug-in
  (silence about 0.9 s after about 10 frames) and also after a board reset
  (3 of 3 captures on 1 Nov). Cause not measured."

## Added after the Pico tests of 1 Nov

- When the alert ends (FR10, measured): "The alert ends about 0.4 s after the person starts walking away, as soon as the range trend and speed confirm it, not only beyond 2100 mm (measured at 0.3 to 0.9 m). The 2100 mm line applies to a target that stays in front of the sensor."
- Slow retreat limit (6.6): "A retreat slower than about 10 cm/s can make the alert switch on and off a few times before it clears. At about 8 cm/s the alert switched off three times in four seconds before staying off at 1.8 m. The alert always ends once the person has left."
- Radar fault (FR11, measured): "With the sensor data line unplugged, RADAR_FAULT was raised 510 ms after the last frame, the obstacle alert was cleared at the same moment, and the system recovered on the first retry after the wire was replugged."
- Not tested on hardware: "The jump gate and the two-target cases were not verified on the Pico. A helper test was attempted, but the sensor reported only one person in most frames (only 7 of 1837 frames had two targets inside the corridor). They are covered by unit tests and by replaying captured frames."
- Unexplained: "Four frames in one capture show a valid target but no alert line and no CLEAR event. Probably log lines missing; not confirmed."
