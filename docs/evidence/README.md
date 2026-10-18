# Bench evidence (Group 29, glasses radar)

| File | What it shows |
|---|---|
| 2026-10-18_mtk3_soak_2380frames.log | uT-Kernel 3.0 on the Pico W (course template). Radar frames with overrun=0, dropped=0, footer_bad=0; obstacle alerts at 161 to 386 mm; OBSTACLE(CLEAR) events. |
| 2026-10-18_mtk3_fault_test.log | Same build, 2380 frames. Sensor TX unplugged: RADAR_FAULT and OBSTACLE(CLEAR); RADAR_OK on recovery. A second brief FAULT then OK about 0.5 s after recovery is the sensor's post-config pause. |

Both logs are from the 1000 ms hold build. The hold is now 500 ms (FR10); the byte-loss and fault results still apply. Not yet measured: NFR6 latency, 500 ms fault timing, long range, 10-minute soak.
