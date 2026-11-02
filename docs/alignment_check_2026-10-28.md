# Alignment check: code and docs against Week 6 and Tribunal 1 (28 Oct 2026)

Scope: the glasses radar slice (Glasses_Radar, Glasses_Threat, radar_monitor,
ld2450 driver, radar message). Checked against the Week 6 Design Review
(FR6 to FR14, NFR6 to NFR10, tasks, section 4, 6.6, A7), the Week 8 Tribunal 1
document (Version A, B, C) and the Minimum-Bit change requirement.

How checked: Week 6 and Tribunal text read in full; every threshold and
timing compared with `*_cfg.h` values and the source; host tests (6 programs)
pass; hardware captures of 18 to 28 Oct read. NOT checked: Brian's audio and
IMU code, Clifton's BLE code, the head-fall task priority (not in this tree).
Nothing here is measured on the Pico for latency (NFR6, NFR10).

Legend: OK = code matches the document. DIFF = code differs from or goes
beyond the document (report or code must change). STALE = the document is
out of date or overclaims. OPEN = not done yet.

## 1. Week 6 requirements

| Item | Week 6 text | Code / config | Verdict |
|---|---|---|---|
| FR7 | UART0 256000 baud, header AA FF 03 00, footer 55 CC, 3 targets, signed speed, bad footer discarded | `LD2450_BAUD_RATE` 256000, 26-byte body, footer check, resync; GP0 TX, GP1 RX (`board_glasses.h`) | OK |
| FR8 | Nearest valid non-departing target within 2000 mm; start below 2000, stop above 2100; departing = away faster than 10 cm/s | 2000 / 2100 / 10 in `threat_assessment_cfg.h`. Extra: corridor gate, range-trend departing rule, 2 mm floor (stop edge becomes 2102 mm) | OK on numbers, DIFF on extras (3.1, 3.2, 3.5) |
| FR9 | OBSTACLE(d) every frame while active; 1500 Hz at 2000 mm to 3500 Hz at 300 mm; beep 150/350 | OBSTACLE(d) every frame: OK. Pitch and beep: changed 2 Nov, owned by Brian's `alert.c` (1000 + (2000 - d) * 2 Hz; period 150 + d * 400 / 2000 ms). Threat's own copy removed | DIFF from Week 6 text (report must change), see 3.6 |
| FR10 | CLEAR when beyond 2100 mm, 500 ms without a non-departing target, or RADAR_FAULT | 500 ms hold, hysteresis, fault path all implemented and seen on hardware (last target to CLEAR in about 540 ms) | OK. Extra: 2-frame early clear for departing targets, jump gate (3.2, 3.3) |
| FR11 | No frame for 500 ms (2 s for the first frame after configuration) is a radar fault to Audio Alert; cleared when frames resume | 500 ms, 2 s grace, retry 2 s, 3 tries as written. New: 3 s settle window where silence only clears the tone | DIFF (OPEN-13, 3.4) |
| FR13 | TTC = distance / closing speed; departing ignored; TTC to BLE | `threat_assessment_ttc_ms`, ms, 0xFFFF means none; to BLE queue | OK. BLE width and unit not agreed (OPEN) |
| FR6 | Only a confirmed RELEASED silences the alert; UNKNOWN acts as HELD | Implemented and unit-tested. Real grip source is Clifton's | OK, source OPEN |
| NFR6 | Alert starts within 100 ms of the frame that first shows the target below 2000 mm | No smoothing; start decided on the same frame, posted in the same pass. Gates do not delay a start | OK by design, NOT MEASURED |
| NFR7 | Interrupt into a stream buffer, no blocking UART reads | ISR to message buffer, 32-byte chunks, `TMO_POL`. Buffer is 512 B | OK; startup loss 44 to 77 bytes (3.7) |
| NFR8 | Hysteresis 2000 / 2100 | Implemented, host tests, hardware seen | OK |
| NFR9 | Radar and Threat Medium, below head fall, above BLE | Priority 20 (Medium) and BLE 21 (Low) in `rtos_port_notes.md` | OK for BLE; head-fall priority not in this tree |
| NFR10 | 100 ms from sensor reading received to audio/haptic | Not measured. Start and stop events not yet agreed with Brian | OPEN |

## 2. Week 6 design text (task table, section 4, statechart 6.6)

| Item | Week 6 text | Code | Verdict |
|---|---|---|---|
| Stream buffer | `sb_uart` 128-byte message buffer, "holds about four 30-byte frames, overflow not defined" | 512 B (`LD2450_RX_BUFFER_BYTES`); overflow drops the whole chunk and counts it | STALE text |
| Queues | `mbx_targets`, `mbx_audio_alert`, `mbx_glasses_ble` as mailboxes (`tk_snd_mbx`) | `osal_queue_*` on message buffers (`tk_cre_mbf`, copy semantics), depth 4 / 8 / 4 | STALE text |
| Radar to Threat | one target list in `mbx_targets`; fault and OK as separate posts | one 13-byte packed queue item carries frame, fault and grip (type field) | DIFF (design improved; text outdated) |
| RADAR_OK | Task table: Threat receives RADAR_FAULT / RADAR_OK | RADAR_OK goes to Audio Alert only; Threat does not need it | STALE text |
| 6.6 states | INIT, 3 failed tries to fault, retry every 2 s, OK on recovery, 2 s first timeout | All implemented in `radar_monitor` | OK |
| 6.6 decode | "decodes each target's distance and speed" | Distance computed in the driver (`ld2450_decode_body`), in 2 mm steps in the message | OK |
| 6.6 grip | Radar keeps parsing whatever the grip state | Yes; grip only gates the alert in Threat | OK |
| Pitch owner | Section 3: "I define the distance-to-pitch mapping, and Audio Alert plays it"; 6.5: `obstacle_freq(d)` in Audio Alert | Decided 2 Nov: Audio Alert (`alert.c`) owns pitch and beep; Threat sends distance only | DIFF from Section 3 text (3.6) |
| A7 | Test sign and value while walking | Walk and bench captures done (27 Oct). Negative = approaching; largest speed magnitude 112 cm/s | Update status (4.3) |

## 3. Differences that need a decision or a report sentence

3.1 Corridor gate (not in Week 6 or Tribunal). A target with |x| above
1200 mm is dropped before Threat sees it. FR8 says "nearest valid" target, so
the report must say valid also means inside the walking corridor. Value tuned
from one walk; checked on the Pico on 1 Nov with an angled approach (alerts
only started inside |x| 1200 mm, see docs/evidence). Safety trade-off: an object
1.2 m to the side, 1.5 m ahead, is ignored. Decide and write it down (docs/target_gates.md).

3.2 Departing by range trend and early clear (not in Week 6). FR8 defines
departing by speed only (more than 10 cm/s away). The code also treats a
target as departing when its range grows over 4 frames (changed from 3 on
1 Nov), and clears after 2 departing-only frames (earlier than the 500 ms
hold). Two further rules since 1 Nov (docs/target_gates.md): the far-target hold
(a target beyond 2100 mm cannot end an alert on one frame) and speed
confirmation (the nearest target counts as departing by speed only if its range
also grows faster than 5 cm/s). Speed readings at rest
wander by about +/-8 cm/s, close to the 10 cm/s threshold. Needs one sentence
in 6.6 and in FR8/FR10.

3.3 Jump gate (not in Week 6). A 1000 mm one-frame jump away is held for one
frame while the alert is active. Effect: a false CLEAR is delayed by one frame
(about 90 ms). Unit-tested only; not yet seen on hardware.

3.4 Settle window. Week 6 gives only the first frame 2 s. The sensor goes
silent about 0.9 s after frames start, so every boot raised a false
RADAR_FAULT. Chosen: for 3 s after configuration a 500 ms silence only clears
the tone. After the window FR11 is exact. Run on the board on 1 Nov: three cold plug-ins and three resets,
no RADAR_FAULT, plus a 738 s soak. See OPEN-13 (team sign-off).

3.5 2 mm distance floor. Alert starts at 1998/1999 mm as before; stop edge is
2102 mm instead of 2101. Within the Tribunal decision (12 bits, 2 mm).

3.6 Pitch and beep constants. Agreed with Brian on 2 Nov: his `alert.c` owns
them, so Threat's copy (cfg values, `threat_assessment_pitch_hz` and its test)
was removed and the bring-up print shows the distance only. Brian's values:
pitch 1000 + (2000 - d) * 2 Hz (1000 Hz at 2000 mm, 4600 Hz at 200 mm and
closer); beep period 150 + d * 400 / 2000 ms with 60 % on (550 ms at 2 m,
190 ms at 0.2 m). Week 6 FR9 says 1500 to 3500 Hz and 150/350 ms, so the report
text must change. Pitch numbers printed in the 1 Nov captures come from the
removed placeholder (1500 to 3500 Hz), not from `alert.c`. Bring-up (2 Nov): a
stand-in task now feeds the radar messages to his `alert.c` (docs/audio_alert_bringup.md);
host-tested, built, flashed and heard on the Maker Pi Pico W on 2 Nov
(rising pitch, silence when clear, radar fault blips; by ear, no log). Brian owns the
real Audio Alert task and the final `ENABLE_OBSTACLE_BEEP` value.

3.7 Startup byte loss. The radar task is not scheduled for at least 1 s during
start-up (configuration takes 2240 ms against a 1.2 s wait budget), so 44 to
77 bytes overflow the buffer. Counters now restart after configuration.
Cause not measured. See docs/startup_transient.md.

## 4. Tribunal 1 text that is stale, overclaimed or unsupported

4.1 Week 6 Architecture Defence row (Putra). "2,380 radar frames with zero
UART overruns, dropped bytes, or bad footers" is true for that log (boot with
47 extra bytes, dropped 0) but later boots lost 44 to 77 bytes before
configuration (several captures, 28 Oct). It was also an older build (before the
packed message and gates). Suggested wording: "zero overruns, dropped bytes
and bad footers in steady state over 2,380 frames; start-up transient of 44 to
77 dropped bytes in the first 2 s, counters restarted after configuration
(docs/startup_transient.md). Re-soak of the current build pending."
Same row: "updated 500 ms clearing behavior remain unverified on hardware" can now
say the 500 ms hold was observed (CLEAR about 540 ms after the last target);
100 ms alert latency remains unmeasured.

4.2 Distance in 12 bits (Version B and gap 10, E10). Implemented: 12 bits,
2 mm, range 0 to 8190 mm, valid bit per slot, saturates at 4095, so NO_TARGET
and out-of-range are defined. Evens seen on the Pico. Still unsupported in the
text: "within the radar's +/-50 mm accuracy" (the manual does not state it, gap
10 says so); "2 bits covers 8192 mm" (typo for 2 mm). Live checks of
hysteresis, TTC and pitch with 2 mm steps: host tests done, hardware partial.

4.3 A7 row. Status "Needs validation" can move to "Partly validated":
walk and bench captures (docs/evidence/2026-10-27_*) confirm negative speed =
approaching, largest speed magnitude 112 cm/s. Remaining: different speeds and
distances, head turns, stationary objects (Week 6 8.3 radar assumptions).

4.4 Resource and Assumption Review. "8 bytes as threatassessment and
rteradarmsg_t as 40 bytes (host build)" does not match docs/radar_message_layout.md
("44 B before, 13 B after; queue 176 B before, 52 B after"). Reconcile the
before value and measure `sizeof` and RAM on the Pico build (gap 8 asked for
this). Retained state (`threat_assessment_t`, `radar_monitor_t`) is not packed
yet; this session added about 8 bytes to `radar_monitor_t` and one flag to the
Threat state.

4.5 Risk R2 (incompatible packed formats). Radar side done. Audio layout
(proposed 2 B) and BLE layout (TTC width and unit, distance, active flag) are
not agreed. Keep Open. Add a timing risk: radar task not scheduled for at least
1 s at start-up (3.7), owner Putra.

4.6 Interface table. `radar_monitor.c` is described as "Receive and parse LD2450
radar frames". Parsing is in `ld2450.c`; `radar_monitor.c` supervises (INIT,
RUN, FAULT, settle) and applies the corridor gate (D-011). Task names in the
table are Putra's files after the AUTOSAR restructure; fine, but fix the
purpose text.

4.7 Test plan. Radar + Audio (packed messages, fault priority, 100 ms or less)
and Radar + BLE (decode, message size) are still Open. No timing start and stop
events are defined yet (gap 9).

## 5. Fix list, ranked

1. Correct the "zero dropped bytes" evidence wording (4.1). Cheapest, and an
   unsupported claim is the worst kind of gap.
2. Decide the FR11 settle window with the team and update FR11 and 6.6 (3.4).
3. Add 6.6 / FR8 / FR10 sentences for the corridor gate, trend rule and jump
   gate (3.1 to 3.3), or remove a gate you cannot defend.
4. Update section 3, 4 and 6.6 text: 512 B buffer, message buffers, one packed
   queue, RADAR_OK to Audio only (section 2).
5. Pitch/beep owner agreed (Brian, 3.6); update FR9 text. Still to do: send the
   audio layout and connect the radar messages to `alert.c`.
6. Measure NFR6 / NFR10 and agree start and stop events (4.7).
7. `sizeof` and RAM on the Pico, before and after (4.4); pack retained state.
8. Update A7 and the 12-bit entry (4.2, 4.3); add the timing risk (4.5).
9. Re-run the settle-window build on the board and save the capture.

## 6. Pico tests of 1 Nov (final build of the day)

| Test | Result | Capture |
|---|---|---|
| Cold plug-in x3, reset x3 | No RADAR_FAULT; settle window works; silence 0.9 s after about 10 frames | startup_* |
| Soak 738 s | 8260 frames, all counters clean (PuTTY log has a 14 s hole, radar counters continuous) | soak_738s |
| Far-target hold (D4) | 4 CLEARs where the new code predicts | far_target_hand_35s |
| Speed confirmation (D5), sway | speed-caused clears gone; 5 CLEARs left, all trend-based, gaps up to 180 ms | sway tests |
| Corridor gate (angled approach) | Alerts start inside \|x\| 1200 mm | angled_approach_gate |
| Approach, walk away (B1, B3) | Start at 1962 to 1984 mm; ends about 0.4 s after leaving | approach_walkaway_b1_b3 |
| Slow approach (B2) | Pass on approach; slow walk away (about 8 cm/s) flaps 3 times before clearing | slow_approach_b2 |
| Fault test | RADAR_FAULT 510 ms after the last frame; recovery on the first retry | fault_test_tx_unplug |
| Head turn | Covered by the angled approach; no separate test | angled_approach_gate |
| Jump gate, two-target | NOT RUN on hardware (second person; sensor reported one target). Unit tests and replay only | two_target_attempt |
