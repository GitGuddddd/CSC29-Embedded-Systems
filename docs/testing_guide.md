# Testing guide: capturing new bench/walk evidence

The old `docs/evidence/` (30 captures, 18 Oct to 1 Nov) was deleted 15 Nov:
it was all captured against the pre-OPEN-22 single-target MIO design, and
several of the behaviours it was evidence for (the jump gate, far-target
hold, speed-confirmation trend filter) no longer exist in the code. This
file is the checklist for the replacement session, so the new captures are
evidence for what the code actually does now (OPEN-22 through OPEN-25).

## Before you start

1. Rebuild and re-run the host tests first - a capture is only worth taking
   against a build that currently passes:
   ```
   cmake --build build/host
   cd build/host && ctest --output-on-failure
   ```
2. Confirm the current thresholds you are testing against (they drift
   between sessions - read the values out of the `_cfg.h` files rather than
   trusting this list in six months):
   - `THREAT_ASSESSMENT_START_MM` / `_STOP_MM` - per-lane BLOCKED/clear
     distance (`threat_assessment_cfg.h`).
   - `THREAT_ASSESSMENT_LANE_HOLD_MS` - per-lane clear hold.
   - `RADAR_MONITOR_CORRIDOR_HALF_MM` - corridor gate half-width.
   - `RADAR_MONITOR_SIDE_DEADZONE_MM` / `_SIDE_HYSTERESIS_MM` - side
     classification and its OPEN-24 hysteresis band.
3. Capture with the debug build (`FEATURE_DEBUG_LOGS` and
   `FEATURE_DEBUG_LOG_MIO` both 1) so the `[DIR]` line and the BLE stub's
   `>>> BLE task <-` lines are visible over the console/log, not just the
   buzzer by ear.
4. Name each capture `YYYY-MM-DD_<what>.txt` (the old evidence followed
   this convention; keep it so captures sort chronologically) and save it
   under `docs/evidence/` (recreate the folder - it holds only in-progress
   evidence between testing sessions, not a permanent archive, so prune it
   again once a result has been written up into `deviations.md` or the
   report and the raw log is no longer needed).

## What to capture, per behaviour

### Per-lane occupancy and hysteresis (FR8/NFR8, OPEN-22)

- Walk straight toward the sensor in the middle lane: alert starts at or
  just inside `START_MM`, stays on while closer than `STOP_MM`, clears
  `LANE_HOLD_MS` after you cross back out past `STOP_MM`.
- Repeat in the left lane, then the right lane, to confirm each lane's
  hysteresis is independent (block the middle lane with a second person or
  object while testing the left lane, and check the left lane's state is
  unaffected).
- Two lanes blocked at once (e.g. a person in the middle lane, an object in
  the left lane): confirm the alert distance (`d_mm` in the audio message)
  is the nearer of the two, per `threat_assessment_pick_lane()`.

### Direction cue (FR9, OPEN-20, unaffected by OPEN-22)

Walk through enough of the 8-row truth table in `threat_assessment.c`
(`threat_assessment_direction_cue()`) to exercise the cue byte for real:
left-only, right-only, middle-only, left+middle, middle+right, left+right,
all three (STOP). Confirm against the `[DIR]` debug line.

### Side-switch hysteresis near the deadzone boundary (OPEN-24)

Walk a target slowly across the `+/-RADAR_MONITOR_SIDE_DEADZONE_MM`
boundary and hold position right on the line for several seconds (natural
sway, no deliberate movement). Confirm the `[DIR]` line does not flicker
direction every frame - the side should only flip once you have clearly
crossed into the hysteresis margin, not on jitter alone.

### Per-lane TTC to BLE (FR13, OPEN-23/OPEN-25)

- Walk toward a single lane at a steady pace and read the BLE stub's
  `>>> BLE task <- obstacle ACTIVE <L/M/R> d=... ttc=...` line: `ttc_ms`
  should fall as you close in, and read `65535` (`RTE_TTC_NONE_MS`) the
  moment you stop approaching (standing still, or stepping back).
- Block two lanes at different distances and closing speeds at once (e.g.
  walk toward the middle lane while something in the left lane is static,
  or vice versa) and confirm each lane's `ttc_ms` is independent - a
  farther lane closing in fast should not be masked by a nearer, slower
  one. This is the scenario OPEN-25 exists for.
- Hold a lane BLOCKED only via its hold-grace window (step back out past
  `STOP_MM` and watch the `LANE_HOLD_MS` window before it clears): confirm
  that lane's `d_mm` stays real but its `ttc_ms` reads `RTE_TTC_NONE_MS`
  (no live `speed_cms` reading while held).

### Grip (FR6) and radar fault (FR11)

Same as previous sessions, unaffected by this round of changes: toggle
grip HELD/RELEASED/UNKNOWN while an alert is active and confirm only
RELEASED silences it; unplug the sensor's data line and confirm
RADAR_FAULT within 500 ms and recovery on replug.

## Writing results up

Once a capture confirms (or contradicts) a behaviour, summarise the result
in `docs/deviations.md` against the relevant OPEN-N entry (or
`docs/requirements_traceability.md` if it changes an FR/NFR's status), the
same way earlier captures were written up before being deleted. Keep the
raw capture only as long as it takes to do that write-up.
