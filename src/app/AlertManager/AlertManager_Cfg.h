#ifndef ALERTMANAGER_CFG_H
#define ALERTMANAGER_CFG_H

/*
 * \file AlertManager_Cfg.h
 * \brief Alert arbitration and tone/vibe parameters (FR3, FR9, FR10)
 * Compliance: Barr-C:2018
 */

/* Prototype 3-2-3-2 pulse rhythm. These timings do not establish IEC certification. */
#define ALERT_FALL_PULSE_MS            (150U)
#define ALERT_FALL_SHORT_GAP_MS        (100U)
#define ALERT_FALL_MEDIUM_GAP_MS       (300U)
#define ALERT_FALL_LONG_GAP_MS         (500U)
#define ALERT_FALL_BURST_GAP_MS        (2500U)
#define ALERT_FALL_PERIOD_MS          ((10U * ALERT_FALL_PULSE_MS) + \
                                      (6U * ALERT_FALL_SHORT_GAP_MS) + \
                                      (2U * ALERT_FALL_MEDIUM_GAP_MS) + \
                                      ALERT_FALL_LONG_GAP_MS + ALERT_FALL_BURST_GAP_MS)
#define ALERT_FAULT_PULSE_MS           (150U)
#define ALERT_FAULT_GAP_MS             (200U)
#define ALERT_FAULT_PERIOD_MS          (5000U)
#define ALERT_OBSTACLE_PULSE_MS        (150U)

/*
 * FR9 direction cue (ported from putra-branch alert.c, design B). The cue byte
 * means "which way is open": CENTER = continuous tone, LEFT = 2-beep burst,
 * RIGHT = 3-beep burst, repeated after a pause that shrinks as the obstacle
 * gets closer. STOP forces the shortest pause on top of the side pattern.
 * Bit layout mirrors THREAT_CUE_* in ThreatAssessment.h (kept separate so
 * AlertManager does not depend on ThreatAssessment; a test pins them equal).
 * Loudness stays at ALERT_STANDARD_DUTY_PERCENT (ear-safe cap), unlike
 * putra-branch which scaled duty 10-50 % with distance.
 */
#define ALERT_CUE_STOP                 (0x01U)
#define ALERT_CUE_SIDE_SHIFT           (1U)
#define ALERT_CUE_SIDE_MASK            (0x03U)
#define ALERT_CUE_SIDE_CENTER          (0U)
#define ALERT_CUE_SIDE_LEFT            (1U)
#define ALERT_CUE_SIDE_RIGHT           (2U)
#define ALERT_CUE_SIDE_ALL_BLOCKED     (3U)
#define ALERT_OBSTACLE_BEEP_GAP_MS     (150U)
/* No confirmed-free lane (not yet "about to collide"): a burst of 4 short beeps. */
#define ALERT_OBSTACLE_BLOCKED_BEEPS     (4U)
#define ALERT_OBSTACLE_BLOCKED_PULSE_MS  (110U)
#define ALERT_OBSTACLE_BLOCKED_GAP_MS    (90U)

/* The "no free lane" pattern slows down if it keeps playing (crowds): after this long
 * the pause between bursts is stretched, so it does not become constant noise. */
#define ALERT_OBSTACLE_BLOCKED_REPEAT_MS     (5000U)
#define ALERT_OBSTACLE_BLOCKED_LONG_PAUSE_MS (1500U)

#define ALERT_OBSTACLE_CENTER_BEEPS    (1U)
#define ALERT_OBSTACLE_LEFT_BEEPS      (2U)
#define ALERT_OBSTACLE_RIGHT_BEEPS     (3U)
#define ALERT_OBSTACLE_PAUSE_MIN_MS    (150U)
#define ALERT_OBSTACLE_PAUSE_MAX_MS    (450U)

/*
 * Steadiness while walking (obstacle alert only; detection and BLE are not
 * affected):
 *  - a changed cue must persist for ALERT_OBSTACLE_CUE_DWELL_MS before the
 *    pattern switches (a rising STOP and the start of an alert apply at once);
 *  - the distance that drives pitch and pause is low-passed with this time
 *    constant (first-order, about 250 ms lag);
 *  - the pause between bursts is latched at the start of each burst cycle.
 */
#define ALERT_OBSTACLE_CUE_DWELL_MS    (450U)
#define ALERT_OBSTACLE_PITCH_TAU_MS    (250U)
#define ALERT_DELIVERY_PULSE_MS         (100U)
#define ALERT_DELIVERY_PERIOD_MS       (2000U)
#define ALERT_CALIBRATION_PULSE_MS       (150U)

/* Pitch mapping for obstacle proximity: 1500 Hz at 3000 mm down to 3500 Hz at 300 mm */
#define ALERT_BUZZER_MIN_FREQ_HZ       (1500U)
#define ALERT_BUZZER_MAX_FREQ_HZ       (3500U)
#define ALERT_OBSTACLE_MAX_DIST_MM     (3000U)
#define ALERT_OBSTACLE_MIN_DIST_MM     (300U)

#define ALERT_FALL_FREQ_A_HZ           (2500U)
#define ALERT_FALL_FREQ_B_HZ           (4000U)
#define ALERT_FAULT_FREQ_HZ            (1000U)
#define ALERT_DELIVERY_FREQ_HZ         (1500U)
#define ALERT_CAL_STARTED_FREQ_HZ      (1800U)
#define ALERT_CAL_DONE_FREQ_HZ         (2600U)
#define ALERT_CAL_FAILED_FREQ_HZ       (1200U)
#define ALERT_STANDARD_DUTY_PERCENT      (12U)
#define ALERT_DELIVERY_DUTY_PERCENT      (10U)

#endif /* ALERTMANAGER_CFG_H */
