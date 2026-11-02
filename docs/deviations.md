# Deviations from Barr-C:2018 and open items

Findings of tools/barr_check.py (run it before every commit) must be zero.
Everything the tool cannot check was reviewed by hand once; reviewers repeat
it on changes. Deviations below are deliberate and need leader sign-off.

## Deviations

| ID | Rule | Deviation | Reason |
|----|------|-----------|--------|
| D-001 | 4.1.a | File and folder names are lowercase (ld2450.c, ld2450_cfg.h, threat_assessment/) instead of the PascalCase names in Clifton's architecture sketch (Ld2450.c, Ld2450_Cfg.h, ThreatAssessment/). The layers, the file roles (.c, .h, _cfg.h, _lcfg.c, _irq.c) and the folder tree are the same. | Barr-C 4.1.a requires lowercase names; the sketch and the standard disagree on case. Ask the team lead if the case should change. |
| D-002 | 4.1.b | Files that belong to one module share its root name: x_cfg.h and x_lcfg.c belong to module x. The checker groups them. Module names are otherwise unique in the first 8 characters (glasses board and main were renamed board_glasses.h and main_glasses.c for this reason). | AUTOSAR-style split of configuration from code. |
| D-003 | 4.2.a | ld2450_lcfg.c and threat_assessment_lcfg.c include the module header (ld2450.h, threat_assessment.h) instead of a header of their own. ld2450_irq.c has its own private header. | The configuration struct type belongs to the module interface. |
| D-004 | 4.2.c.i | g_ld2450_lcfg and g_threat_assessment_lcfg are declared extern const in the module header. | Link-time configuration pattern; the data is read-only. |
| D-005 | 6.1.d | Unit-test case names in tests/ may exceed 31 characters. src/ and config/ comply. | Names describe the behaviour under test. |
| D-006 | 1.1.a | Host code is built with -std=c99 -pedantic. The firmware target is built as gnu11 because the Pico SDK needs C11. The glasses sources contain no C11 features. | SDK requirement. |
| D-007 | 4.1, 1.2 | soup/ is not checked (Unity, uT-Kernel template). The template files keep their names. | Names are dictated by the third-party code. |
| D-008 | 5.1, 7.1 | Types and macros of uT-Kernel (UINT, UW, ER, FP, EXPORT, out_w, in_w, GPIO_CTRL) and Unity (TEST_ASSERT_*) are used as they are, only in hal_uart.c, hal_time.c, osal.c and main_glasses.c. The interrupt handlers are cast to FP because the kernel declares them that way. | SOUP names. |
| D-009 | 4.1, 4.4.b | The template calls subsystem5_init / subsystem5_start / subsystem5_test. tools/export_to_template.py generates that shim; the code in src/ uses main_glasses_init / main_glasses_start. tk_include.h defines PROHIBIT_DEF_SIZE_T because the kernel header defines its own size_t. | Names and a macro are fixed by the template and the kernel header. |
| D-010 | 7.x | The export step changes one line of the template copy in build/utkernel: DEVCNF_USE_SER 0 in config/config_device.h, so the radar owns the UART0 interrupt. soup/utkernel itself is never edited. The team repo needs the same line (log it in TEAM.md). | The kernel serial driver reads one byte per interrupt with a 50-byte buffer; at 256000 baud that is not safe. |
| D-011 | 4.1.b | The glasses node has the extra application component radar_monitor (INIT / RUN / FAULT supervision) and the services/osal adapter, which are not in the sketch. | The Week 6 design needs the supervision; the adapter keeps kernel calls in one file. |

## Open items (not deviations)

| ID | Item | Owner |
|----|------|-------|
| OPEN-1 | rte_audio_msg_t and rte_ble_obstacle_t are placeholders until the Audio Alert and BLE owners agree them. | Putra, Brian, Clifton |
| OPEN-2 | The firmware (rte.c, ld2450_irq.c, hal_*.c, osal.c, main_glasses.c) builds and links in the course template but has not run on the Pico. First bench run is still to do. | Putra |
| OPEN-3 | Barr-C 6.5.d (default handler for unexpected interrupts): relies on the Pico SDK vector table; not verified. | Putra |
| OPEN-4 | The leader's sketch lists "target priority filters" in the ThreatAssessment link-time config. Only the speed-sign switch exists. Add the filters if the report needs them. | Putra, leader |
| OPEN-5 | No frame smoothing. A single bad range reading can start or stop the alert for one frame; the 500 ms hold only covers missing targets. Weak reflectors give erratic range. Decide after bench tests whether a 2-frame average is needed (NFR6 limits it). Partly addressed 27 Oct: a corridor gate and an away-only jump gate (docs/target_gates.md) were added; neither delays the start of an alert. A one-frame false START is still possible. | Putra |
| OPEN-6 | FR and NFR numbers in docs/requirements_traceability.md come from code comments; confirm against the final report. | Putra |
| OPEN-7 | Pitch and beep numbers are now Brian's (alert.c, agreed 2 Nov): 1000 Hz at 2000 mm up to 4600 Hz at 200 mm, beep period 550 ms down to 190 ms. This differs from Week 6 FR9 (1500 to 3500 Hz, 150/350 ms), so FR9 wording changes in the report. His pitch passes 2500 Hz at 1250 mm and 4000 Hz at 500 mm, close to the two fall-alarm tones; confirm by ear on the real buzzer (A6). | Brian, Putra |
| OPEN-8 | Doxygen comments are written in Barr-C style but doxygen has not been run. | Putra |
| OPEN-9 | Rule 1.6 (every cast has a comment) and rule 8.2 (nesting, final else) were checked by hand, not by tool. | Reviewer |
| OPEN-10 | The firmware builds and links in the course template but has not run on the Pico. Needs the bench check (no TIMEOUT, overrun=0, dropped=0, footer_bad=0, alerts, fault test). | Putra |
| OPEN-11 | DEVCNF_USE_SER is set to 0 in the template config/config_device.h so that the radar owns the UART0 interrupt. Shared file: tell the team and record in TEAM.md. | Putra, Clifton |
| OPEN-12 | Grip state (FR6) is implemented and tested on the PC with injected messages only. The real source is not in this tree. The Glasses BLE owner must call rte_write_grip_state(): HELD or RELEASED from each cane packet, and UNKNOWN when no cane packet has arrived for 500 ms (and none since power-up is already UNKNOWN here). Threat cannot detect a stale RELEASED itself. The cane packet layout (grip field) is still undefined in the report. Agree the message with the BLE owner together with OPEN-1. Draft interface note: docs/grip_interface.md. | Putra, BLE owner |
| OPEN-13 | Week 6 FR11: "If no valid radar frame arrives for 500 ms (2 s for the first frame after configuration), the glasses shall send a radar fault to Audio Alert, and clear it when valid frames resume." Section 6.6 repeats it: after configuration only the first timeout is 2 s. Bench 28 Oct: frames arrive at once after configuration, then the sensor is silent about 0.9 s (t = 3190 to 4080 ms), so the 500 ms rule fired a false RADAR_FAULT on every boot. Chosen 28 Oct: for 3 s after a successful configuration (RADAR_MONITOR_SETTLE_MS) a 500 ms silence only sends CLEAR to Glasses_Threat, so the tone stops after 500 ms; no RADAR_FAULT goes to Audio Alert in that window. Silence at the end of the window is a fault as in FR11. Report 6.6 needs one sentence for this, and the team should agree it. | Putra, leader |
