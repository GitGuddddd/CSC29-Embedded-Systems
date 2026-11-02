# Start-up transient of the radar receive path (2026-10-28)

Measured on the Pico W with the debug log on. Four boots, same firmware.

## What was seen

| Observation | Value |
|---|---|
| Radar task start | t = 0 ms, 1 byte received, 0 dropped |
| Configuration finished (`CONFIG OK`) | t = 2240 ms on every boot |
| Bytes received before `CONFIG OK` | 587 (about 1.75 s of sensor output) |
| Bytes dropped before `CONFIG OK` | 44, 64, 66, 70, 77 (varies per boot) |
| Bytes dropped after `CONFIG OK` | 0 (constant for 30 s or more) |
| Sensor silent after configuration | frames t = 2390..3190 ms, then none until about 4080 ms |

## Reading

* The task starts at once, but configuration takes 2240 ms. The waits in
  `ld2450_configure()` are limited to 6 x 200 ms = 1.2 s, so the radar task
  was not running for at least 1 s. The cause is not measured. Higher
  priority start-up work (monitor, USB, Wi-Fi) is the likely reason.
* The 512-byte stream buffer overflowed during that time. These are the
  oldest frames of the first 2 s, so no alert decision is affected.
* The sensor is silent for about 0.9 s after configuration. Before the
  change below this raised RADAR_FAULT at t = 3700 ms on every boot.

## Changes

1. `ld2450_reset_rx_stats()` runs after a successful configuration. The
   NFR7 counters now describe steady state. The `CONFIG` debug line shows
   the transient (bytes and dropped) before the reset.
2. `RADAR_MONITOR_SETTLE_MS` (3000 ms): for 3 s after a successful
   configuration a 500 ms silence only sends CLEAR to Glasses_Threat. The
   tone stops after 500 ms and no RADAR_FAULT is sent to Audio Alert. Silence
   at the end of the window is a fault as in FR11 (see OPEN-13).

## Result after the change

Evidence: `docs/evidence/2026-10-28_startup_before_fix.txt` and
`docs/evidence/2026-10-28_startup_after_fix.txt`.

| | Before | After |
|---|---|---|
| RADAR_FAULT at t = 3700 ms | every boot | none |
| `dropped` on the first periodic line | 44 to 77 | 0 |
| bytes at frame 20 | 1188 (587 + 20 x 30 + ...) | 601 (20 x 30 + 1) |

The capture in the table was taken with the first version of the settle
window, which only held the deadline. In that version the last tone stayed
on for about 0.9 s (t = 3190 to 4080 ms). The current version clears it
after 500 ms. It has not been run on the board yet.

## Open

* Why the radar task is held for about 1 s at start-up is not measured.
* Why the sensor goes quiet after configuration is not confirmed.
