# Glasses Radar Subsystem

Changes since 4 Nov, current state on `putra-branch` (Group 29 Smart eWalker)

This covers only what is actually in the branch today. Anything added and later removed again within this window (the single-target MIO tie-break, frame-to-frame track IDs, and 2D collision prediction, all added 8 Nov and removed by the 13 Nov redesign) is left out.

## FR9 audio redesign — 8 Nov

- Distance now drives loudness (PWM duty cycle) instead of beep rate; side is a fixed-timing beep pattern (CENTER continuous tone, LEFT 2 beeps, RIGHT 3 beeps).
- STOP forces max loudness without erasing the beep pattern.
- Corridor/deadzone constants widened (`SIDE_DEADZONE_MM` 300→425mm by 13 Nov, `CORRIDOR_HALF_MM` 1200→1500mm) after bench testing showed standing dead-centre was misclassifying as LEFT.

## FR9 cue meaning flipped: direction, not threat side — 11 Nov

- The cue byte now means "which way is open to walk" instead of "which side the threat is on" — computed per-side (left/middle/right all independently) rather than from one picked target.
- Beep cadence (the pause between bursts) now scales with distance, escalating as an obstacle closes in.

## Dropped single-target MIO selection for per-lane occupancy — 13 Nov

- The biggest change: removed MIO tie-break, frame-to-frame track IDs, and 2D collision prediction entirely — head-turns made these produce false collision predictions against stationary walls.
- Replaced with: each of 3 lanes (left/middle/right) has its own independent BLOCKED/clear state with hysteresis (1800/1950mm) and a 200ms hold timer.
- The alert is active whenever ANY lane is blocked; reported distance is the nearest among currently-blocked lanes.

## Side-switch hysteresis — 13 Nov

- A target drifting a few mm across the deadzone boundary (sensor jitter, head sway) no longer flips its lane every frame — added a sticky-side rule so a lane assignment has to clearly cross before it changes.

## Per-lane TTC restored to BLE — 14 Nov

- FR13 had gone unmet when per-target velocity tracking was removed in the 13 Nov redesign.
- Restored as an array: BLE now gets `d_mm[3]`/`ttc_ms[3]`, one pair per lane (not just one picked lane), so a farther-but-faster-closing lane on another side isn't hidden.

## `FEATURE_DEBUG_LOG_MIO` renamed to `FEATURE_DEBUG_LOG_DIR` — 15 Nov

- Stale naming cleanup — the flag now gates the `[DIR]` debug line (direction cue), not the long-gone `[MIO]` line, and the name matches what it actually does.

## TTC-hold smoothing + readable bring-up print — 18 Nov, latest

- Bench evidence showed the LD2450's own speed reading dropping out for a frame or two even mid-approach, flickering TTC to NONE and back.
- Each lane now holds its last usable closing speed for up to 200ms before falling back to NONE.
- The bring-up BLE print now shows "L/M/R clear" per lane instead of raw 65535mm/65535ms sentinels.

---

## What matters most for combining with the rest of the team

**BLE wire format changed (14 Nov):** `rte_ble_obstacle_t` went from single `d_mm`/`ttc_ms` fields to `d_mm[3]`/`ttc_ms[3]` arrays, indexed by `rte_cue_side_t` (CENTER/LEFT/RIGHT). This is a bigger payload than before — flagged in `docs/deviations.md` (OPEN-25) as needing sign-off from whoever owns the BLE side, which hasn't happened yet.

**Cue byte meaning flipped (11 Nov):** the same bits that used to mean "the threat is on this side" now mean "go this way" — opposite instructions if Brian's audio code (or anyone else) assumes the old meaning. Also flagged in `docs/audio_message_layout.md`.
