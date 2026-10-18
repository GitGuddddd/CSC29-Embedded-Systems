# Requirements traceability (glasses perception slice)

Requirement numbers and wording are taken from the comments of the Group 29
radar modules and the Week 6 design review (statechart 6.6). Check them
against the latest report before submission. "Target" = to be measured on the
Pico, not yet done.

| Req | Short text | Code symbol | Test |
|-----|-----------|-------------|------|
| FR7 | Parse LD2450 frames, up to 3 targets, discard a bad footer and resync | ld2450_receive_frame, ld2450_decode_body, ld2450_decode_signed | test_ld2450_parser (all), fuzz_radar_frames |
| FR8 | Nearest valid non-departing target within 2000 mm is an obstacle | threat_assessment_update, THREAT_ASSESSMENT_START_MM, threat_assessment_is_departing | test_threat_assessment (stationary, departing, 2000 edge) |
| FR9 | OBSTACLE(d_mm) to Audio Alert on every frame while active; pitch 1500 to 3500 Hz | threat_assessment_process_frame, threat_assessment_pitch_hz, rte_write_audio_msg | test_threat_assessment (process_frame, pitch), test_alert_pipeline |
| FR10 | OBSTACLE(CLEAR) beyond 2100 mm, after 500 ms without a target, or on RADAR_FAULT; BLE is best effort | threat_assessment_on_target, threat_assessment_on_no_target, threat_assessment_process_fault, rte_write_ble_obstacle | test_threat_assessment (hysteresis, hold, fault), test_alert_pipeline |
| FR11 | No valid frame for 500 ms is RADAR_FAULT; config retry every 2 s; RADAR_OK when frames resume | radar_monitor_*, RADAR_MONITOR_FRAME_TIMEOUT_MS | test_radar_monitor, test_alert_pipeline |
| FR13 | Closing speed and TTC to BLE; departing targets ignored | threat_assessment_closing_cms, threat_assessment_ttc_ms, rte_ble_obstacle_t | test_threat_assessment (ttc, departing) |
| NFR6 | Alert latency budget (300 ms) | No smoothing; one frame (about 140 ms) from byte to message | Target |
| NFR7 | No byte loss at 256000 baud, parsing outside the interrupt | ld2450_rx_isr, LD2450_RX_BUFFER_BYTES, ld2450_get_overflow_count | Target (counter must stay 0) |
| NFR8 | Hysteresis: start below 2000 mm, stop above 2100 mm | THREAT_ASSESSMENT_START_MM, THREAT_ASSESSMENT_STOP_MM | test_threat_assessment (hysteresis) |
| NFR9 | Radar and Threat at Medium priority, below head-fall, above BLE | rte_start, RTE_PRIORITY_*_OFFSET in rte_cfg.h | Target |
| A5/A6 | Wiring, 2 s grace for the first frame after config | RADAR_MONITOR_POST_CONFIG_GRACE_MS | test_radar_monitor (grace deadline) |
| A7 | Obstacle tone distinguishable from fall alarm and fault tone | threat_assessment_pitch_hz, THREAT_ASSESSMENT_BEEP_*_MS | Target (listening test, see OPEN-7) |
| A8 | Negative speed means approaching | g_threat_assessment_lcfg | test_threat_assessment (closing), Target (walk-toward test) |
