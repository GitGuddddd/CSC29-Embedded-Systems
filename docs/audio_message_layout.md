# Audio Alert message layout (draft)

Status: DRAFT for Brian (Audio Alert owner). The packed-word proposal below
was never implemented; see "What actually shipped (8 Nov 2026)" first.
Reason: change requirement "Minimum-Bit Data Representation" (Week 8, Putra's
tribunal row, risk R2). Related: docs/grip_interface.md, docs/deviations.md OPEN-1.

## What actually shipped (8 Nov 2026)

FR9 (prof feedback, 7 Nov) asked for a bit-flag byte describing the obstacle
(STOP vs. side) sent to Audio Alert, not a reject mask. Rather than the packed
16-bit word proposed below, `rte_audio_msg_t` kept its plain struct and grew
one field:

```c
typedef struct
{
    rte_audio_msg_type_t type;
    uint16_t              d_mm;
    uint8_t               cue;   /* RTE_CUE_* bits; 0 = no cue, OBSTACLE only */
} rte_audio_msg_t;
```

`cue` bit 0 is `RTE_CUE_STOP` (full stop); bits 1-2 are the side
(`RTE_CUE_SIDE_MASK` after `RTE_CUE_SIDE_SHIFT`: 0 CENTER, 1 LEFT, 2 RIGHT, 3
unused). Threat Assessment sets it in `threat_assessment_build_cue()`: STOP
when the nearest obstacle has held inside 900 mm for at least 500 ms (clears
past 1100 mm, `THREAT_ASSESSMENT_STOP_CUE_*`), side from the target's
`RTE_SIDE_*` classification (docs/radar_message_layout.md). `alert.c` decodes
it with its own `ALERT_CUE_*` constants (duplicated, not shared, to keep
`alert.c` decoupled from RTE types) via `alert_set_cue()`. `rte_write_audio_msg`
gained a third `cue` parameter; call sites outside Threat (radar fault/OK)
pass `0u`.

### Sound design, revised 8 Nov (design B, Putra's call)

The first cut (STOP = continuous, CENTER = the old pulse-rate-for-distance
tone, LEFT/RIGHT only near STOP) risked overloading one channel (timing)
with two meanings at once: how fast it pulsed told you the distance, and how
many beeps per burst told you the side, at the same moment a close obstacle
made both matter most. Revised design, confirmed 8 Nov:

- **Distance now rides on loudness, not pulse rate.** `alert.c` maps
  distance to a PWM duty cycle (`ALERT_MIN_DUTY_PCT` 10 to `ALERT_MAX_DUTY_PCT`
  50; 50 is loudest on this hardware, see `buzzer_utk.c`'s own comment) via
  the new `buzzer_tone_vol(freq_hz, duty_pct)` call (`buzzer_tone(freq_hz)` is
  now a thin wrapper at duty 50). DRAFT, not bench-verified: it is untested
  whether duty-cycle changes read as a smooth loudness ramp on the real
  piezo or as a narrower on/off-ish range; see docs/deviations.md OPEN-14.
- **Side is always on**, not confined to near STOP: CENTER is one
  continuous tone (no pulsing at all now that distance no longer uses
  pulse rate); LEFT is a 2-beep burst then a pause; RIGHT is 3 beeps.
  Pulse/gap timing is fixed (`ALERT_BEEP_PULSE_MS`/`_GAP_MS`); the pause
  after the burst was fixed too until 11 Nov - see OPEN-21 below, it is not
  anymore.
- **STOP only forces loudness, it does not replace the side pattern**
  (confirmed design B over the alternative of STOP overriding to one flat
  tone): STOP LEFT is a max-volume 2-beep burst, not a continuous tone,
  so the side information is not lost exactly when it matters most.
- `cue == 0` (CENTER, no STOP) still behaves like "no cue ever sent" in the
  sense that nothing from Threat changes tone/volume unexpectedly, but the
  tone is now continuous-and-loudness-scaled rather than byte-identical to
  the pre-FR9 pulsing formula — that pre-FR9 formula no longer exists.

### Cue meaning changed (11 Nov 2026, OPEN-20): side-of-threat → direction-to-go

The paragraph above ("side from the target's `RTE_SIDE_*` classification")
describes the *original* FR9 meaning and is now superseded for the side
bits (STOP's byte position is unchanged, only what triggers it changed -
see below). `threat_assessment_build_cue()` no longer reports which side
the single MIO target is on; it reports which way is open to walk, from
independent per-side corridor occupancy
(`threat_assessment_scan_sides()`/`threat_assessment_hold_side()`, same
hysteresis distances as FR8/NFR8 - `THREAT_ASSESSMENT_START_MM`/`_STOP_MM`,
2000/2100 mm - applied per side instead of to the single nearest target):

| Left blocked | Middle blocked | Right blocked | Cue (bits 1-2) |
|---|---|---|---|
| - | - | - | CENTER |
| X | - | - | CENTER (no need to turn) |
| - | - | X | CENTER (no need to turn) |
| - | X | - | LEFT (arbitrary fixed default - both sides equally open) |
| X | X | - | RIGHT |
| - | X | X | LEFT |
| X | - | X | CENTER (middle is the only open path) |
| X | X | X | **STOP** (bit 0) |

STOP (`RTE_CUE_STOP`) now means all three sides are occupied, not "the MIO
target has held inside 900 mm for 500 ms" - the old near-distance STOP
sub-hysteresis (`THREAT_ASSESSMENT_STOP_CUE_*`, `b_stop_cue`) still exists
in `threat_assessment_t` and is still updated every frame, but is no longer
read by `threat_assessment_build_cue()`; it is dormant, kept in case a
near-distance STOP sub-cue is wanted again.

**Nothing downstream needed a code change.** `alert.c`'s decode
(`alert_update()`, "Sound design, revised 8 Nov" below) is purely mechanical
on the bit values - it plays a continuous tone for side-bits `CENTER`, a
2-beep burst for `LEFT`, a 3-beep burst for `RIGHT`, and forces max loudness
for `STOP`, regardless of what those bit patterns are upstream decided to
mean. So "2 beeps" now means "go left" instead of "the obstacle is on your
left", purely by re-reading what the bits stand for; the sound itself,
and `tests/unit/test_audio_alert_bridge.c`'s assertions on it, are
unchanged. Brian should be made aware of the semantic flip before relying
on it, since "go left" and "avoid left" are opposite instructions if
anyone assumes the old meaning while integrating.

### Beep pause now rides on distance too (11 Nov 2026, OPEN-21)

Putra's call, discussed over several rounds: the buzzer already got louder
and higher-pitched as an obstacle closed in (OPEN-14), but the LEFT/RIGHT
beep pattern itself repeated at a fixed rate regardless of distance - all
the escalation was in loudness/pitch, none of it in tempo. Added: the pause
after a full beep burst (`ALERT_BEEP_PAUSE_MS`, previously a fixed 450 ms)
now scales linearly with distance exactly like `duty_pct` does, from
`ALERT_BEEP_PAUSE_MAX_MS` (450 ms) at the farthest alerting distance down to
`ALERT_BEEP_PAUSE_MIN_MS` (150 ms) at the closest; STOP forces the pause
straight to 150 ms regardless of distance, the same way it already forces
max loudness. Beep *count* (2 vs 3) is untouched, so direction is still read
the same way - only how fast the same pattern repeats changes. This was not
"obstacle tracking the user's own movement" (this subsystem has no IMU/
heading input at all, see docs/deviations.md OPEN-17's known limitation):
it is a direct, mechanical consequence of `d_mm` - if the user does not move
away from a blocked lane, the nearest obstacle on it keeps closing, so the
beeps keep speeding up (and getting louder) on their own, with no new
"did the user comply" logic needed. `tests/unit/test_audio_alert_bridge.c`
gained two new cases (`test_cue_pause_shrinks_when_the_obstacle_is_closer`,
`test_cue_stop_forces_the_fastest_pause`) and one existing case
(`test_cue_left_gives_two_beeps_then_a_pause`) had its pause-timing
assertion corrected for the new, no-longer-450-ms pause at its 1362 mm
fixture distance - this is a test-assertion fix following the behaviour
change, not a regression. DRAFT, not bench-verified (same caveat as
OPEN-14): whether a shrinking pause reads as intuitively "faster" on the
real piezo, rather than just irregular, is untested; the 150/450 ms bounds
are reasoned guesses, not bench-measured. Brian owns the final sound
design.

A bench-debug `[DIR]` print (`rte_write_dir_debug()`, same
`FEATURE_DEBUG_LOG_MIO` flag as OPEN-19's `[MIO]` line) prints this
decision in plain language every frame - `[DIR] L=<0|1> M=<0|1> R=<0|1> ->
go left/stay middle/go right/STOP/no alert` - so the byte does not have to
be decoded by hand while bench-testing.

This did not adopt the packed-word layout, so the `rte_audio_msg_t` struct is
still 4+2+1 bytes (padded), not 2 bytes — the size reduction below was not
pursued for this message. Tests: tests/unit/test_audio_alert_bridge.c
(`test_cue_stop_gives_a_continuous_tone` and friends).

Direction: Glasses_Threat and Glasses_Radar (through `rte_write_audio_msg`)
send, Audio Alert (`alert.c`) receives, through the audio queue.

## Now

`rte_audio_msg_t` is a struct of a 4-byte enum and a 16-bit distance: 8 bytes
per message, 64 bytes for the 8-entry queue.

## Proposed: one 16-bit word

| Bits  | Field    | Width | Meaning |
|-------|----------|-------|---------|
| 0-2   | type     | 3     | 0 OBSTACLE, 1 OBSTACLE_CLEAR, 2 RADAR_FAULT, 3 RADAR_OK. 4 and 5 proposed for SENSOR_FAULT and SENSOR_OK (Brian confirms). 6 and 7 reserved. |
| 3-14  | distance | 12    | Units of 2 mm, 0 to 4095 = 0 to 8190 mm. Only meaningful for OBSTACLE; send 0 for every other type. |
| 15    | reserved | 1     | Always 0. |

Bit 0 is the least significant bit of the `uint16_t`. Receiver decodes with
`distance_mm = ((word >> 3) & 0x0FFF) * 2`.

Total: 16 bits, so a 2-byte message and a 16-byte queue (8 x 2 B).

## Rules

- **Rounding:** the sender rounds distance DOWN to an even millimetre value
  (floor). A target 1999 mm away is sent as 1998 mm. This errs toward alerting.
  With the 2 Nov pitch formula (2 Hz per mm) the pitch step is 4 Hz per field step, far below what the ear hears.
- **Saturation:** a distance above 8190 mm is sent as 4095. Threat only sends
  OBSTACLE while a target is inside 2100 mm, so this is a safety net for
  corrupt frames, not a normal value.
- **When sent:** unchanged. OBSTACLE every radar frame while active, CLEAR once
  when the alert ends, RADAR_FAULT once on entering fault, RADAR_OK once on
  recovery.
- **Helpers:** pack and unpack live in `rte_type.h` as shared functions (for
  example `rte_audio_pack(type, d_mm)`, `rte_audio_type(word)`,
  `rte_audio_distance_mm(word)`), so nobody writes the shifts by hand.
- **Atomicity:** the word is copied whole through the kernel queue. No shared
  variable is read-modify-written, so no extra locking is needed.
- **Both sides change in the same commit.** There is no version field.

## Test and evidence

- Round-trip test over all 4096 distance values and every type.
- Saturation and rounding tests (1999 to 1998 mm, 9000 mm to 4095).
- Alert latency test, tribunal "Radar + Audio Alert": `OBSTACLE` / `CLEAR` /
  `FAULT` through the packed word, correct behaviour, fault priority, 100 ms
  or less. Evidence: serial log and buzzer recording.
- Message size: 8 B before, 2 B after (confirm `sizeof` on the Pico build).

## Questions for Brian

1. Is a 3-bit type with 4 = SENSOR_FAULT and 5 = SENSOR_OK right for your fault
   tones, or do you want other codes?
2. Is 2 mm distance resolution fine for the pitch map?
3. Does `alert.c` need anything else from Threat, such as TTC for urgency? There
   are 0 spare bits except bit 15.
4. Can `alert.c` take a `uint16_t` word from the queue, or does it need the
   struct?
