# Audio Alert bring-up (2 Nov 2026)

Purpose: hear the radar alerts on the real buzzer before the Audio Alert task
(owner: Brian) is integrated. This is a stand-in, not the final task.

## What is in the repo

| File | What it is | Owner |
|---|---|---|
| `src/app/audio_alert/alert.c`, `alert.h`, `buzzer.h` | Copied from Brian's glasses-fall-alert code. `alert.c` has one bring-up edit: the obstacle-beep switch can be set from `config/feature_cfg.h` (his default stays 0). | Brian |
| `src/app/audio_alert/buzzer_utk.c` | Implements Brian's `buzzer.h` with the course template's own PWM calls (the template build does not link the Pico SDK `hardware_pwm` and `clocks` modules his `buzzer.c` needs). 1 MHz PWM counter on GP18. Replace when the real task is integrated. | Putra (bring-up) |
| `src/app/audio_alert/audio_alert_bridge.c` | Maps each radar message to an `alert_*` call. | Putra (bring-up) |
| `main_glasses.c` | The audio task now runs `alert_init`, the bridge and `alert_update` every 10 ms. It still prints each `>>>` line. | Putra (bring-up) |
| `config/feature_cfg.h` | `ENABLE_OBSTACLE_BEEP (1)`. | Brian decides the final value |
| `tests/unit/test_audio_alert_bridge.c`, `tests/harness/mock_buzzer.*`, `tests/harness/pico/types.h` | Host tests with a fake buzzer: pitch at 1362 mm = 2276 Hz, 500 mm = 4000 Hz, CLEAR silences, beyond 2000 mm silent, radar fault outranks the obstacle tone, RADAR_OK restores it. | Putra |

## Message to call mapping

| Radar message | alert.c call |
|---|---|
| `OBSTACLE(d)` | `alert_set_obstacle(d)` |
| `OBSTACLE(CLEAR)` | `alert_set_obstacle(0)` (silent) |
| `RADAR_FAULT` | `alert_set_fault(ALERT_FAULT_RADAR, true)` |
| `RADAR_OK` | `alert_set_fault(ALERT_FAULT_RADAR, false)` |

## Bench result (2 Nov 2026, Maker Pi Pico W, buzzer on GP18)

First build with the buzzer was silent and the GP18 LED stayed dark. Cause: the
template releases the PWM block from reset only when `USE_PTMR` is 1, and it is
0 here, so `buzzer_utk.c` now releases it itself in `buzzer_init` (a temporary
boot beep was used to find this and has been removed).

After that fix, by ear (not recorded in a log):
- walking in from about 2 m: the pitch rises and the beeps speed up;
- walking away or leaving the view: the beeping stops, no stuck tone;
- sensor TX wire pulled: radar fault blips, normal beeping returns on replug.

## Not done, not verified

- The pitch values were not measured, only heard. The mapping is Brian's
  (`alert.c`: 1000 + (2000 - d) * 2 Hz).
- Whether the obstacle tone is confused with a fall alarm (OPEN-7) is not
  judged yet; his pitch passes 2500 Hz at 1250 mm and 4000 Hz at 500 mm.
- The 125 MHz system clock is assumed from the template (`cpu_clock.c`, PLL SYS
  125 MHz); `BUZZER_UTK_SYS_CLOCK_HZ` in `buzzer_utk.c` must match.
- Brian's IMU / head fall code is not in this build (separate program).
- Fall alarm and IMU fault tones are not wired; only the radar side is.
- Needs Brian's decision: `ENABLE_OBSTACLE_BEEP` is set to 1 in
  `config/feature_cfg.h` (his file defaults to 0).
