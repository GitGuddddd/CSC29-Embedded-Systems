# Requirements traceability (glasses perception slice)

Requirement numbers and wording are taken from the comments of the Group 29
radar modules and the Week 6 design review (statechart 6.6). Check them
against the latest report before submission. "Target" = to be measured on the
Pico, not yet done.

| Req | Short text | Code symbol | Test |
|-----|-----------|-------------|------|
| FR6 | Obstacle alert stays on while the cane is HELD or grip is UNKNOWN; only a confirmed RELEASED silences it | threat_assessment_set_grip, threat_assessment_process_grip, rte_write_grip_state, RTE_GRIP_* | test_threat_assessment (grip tests), test_alert_pipeline (cane released); real BLE source: Target |
| FR7 | Parse LD2450 frames, up to 3 targets, discard a bad footer and resync | ld2450_receive_frame, ld2450_decode_body, ld2450_decode_signed | test_ld2450_parser (all), fuzz_radar_frames |
| FR8 | Nearest valid non-departing target within 2000 mm is an obstacle | threat_assessment_update, THREAT_ASSESSMENT_START_MM, threat_assessment_is_departing | test_threat_assessment (stationary, departing, 2000 edge) |
| FR9 | OBSTACLE(d_mm) to Audio Alert on every frame while active; distance only; pitch and beep pattern are owned by Audio Alert (alert.c, agreed 2 Nov) | threat_assessment_process_frame, rte_write_audio_msg | test_threat_assessment (process_frame), test_alert_pipeline |
| FR10 | OBSTACLE(CLEAR) beyond 2100 mm, after 500 ms without a target, or on RADAR_FAULT; BLE is best effort | threat_assessment_on_target, threat_assessment_on_no_target, threat_assessment_process_fault, rte_write_ble_obstacle | test_threat_assessment (hysteresis, hold, fault), test_alert_pipeline |
| FR11 | No valid radar frame for 500 ms (2 s for the first frame after configuration) is RADAR_FAULT to Audio Alert, cleared when frames resume; config retry every 2 s | radar_monitor_*, RADAR_MONITOR_FRAME_TIMEOUT_MS | test_radar_monitor, test_alert_pipeline |
| FR13 | Closing speed and TTC to BLE; departing targets ignored | threat_assessment_closing_cms, threat_assessment_ttc_ms, rte_ble_obstacle_t | test_threat_assessment (ttc, departing) |
| NFR6 | Alert starts within 100 ms of the radar frame in which the target first falls below 2000 mm | No smoothing; corridor gate and away-only jump gate add no start delay (docs/target_gates.md); the OBSTACLE message is posted in the same pass as the frame is received | Target (log frame received to audio queue write on the Pico) |
| NFR7 | No byte loss at 256000 baud, parsing outside the interrupt | ld2450_rx_isr, LD2450_RX_BUFFER_BYTES, ld2450_get_overflow_count | Target (counter must stay 0 after configuration; start-up transient: see docs/startup_transient.md) |
| NFR8 | Hysteresis: start below 2000 mm, stop above 2100 mm | THREAT_ASSESSMENT_START_MM, THREAT_ASSESSMENT_STOP_MM | test_threat_assessment (hysteresis), test_rte_pack (2 mm edges) |
| NFR9 | Radar and Threat at Medium priority, below head-fall, above BLE | rte_start, RTE_PRIORITY_*_OFFSET in rte_cfg.h | Target |
| FR11 / 6.6 | 2 s grace for the first frame after config (bench finding) | RADAR_MONITOR_POST_CONFIG_GRACE_MS | test_radar_monitor (grace deadline) |
| FR11 / 6.6 | 3 s settle window after config (bench finding: sensor silent 0.9 s after config). Inside it a 500 ms silence clears the tone (CLEAR to Threat) but raises no RADAR_FAULT. After the window FR11 applies as written. Goes beyond the Week 6 wording: see OPEN-13 | RADAR_MONITOR_SETTLE_MS | test_radar_monitor (settle window), test_alert_pipeline, docs/startup_transient.md |
| A6 | Obstacle tone distinguishable from fall alarm and fault tone | alert.c (Brian) | Target (listening test, see OPEN-7) |
| A7 | Negative speed means approaching | g_threat_assessment_lcfg | test_threat_assessment (closing), Target (walk-toward test) |
