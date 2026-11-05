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

/* Pitch mapping for obstacle proximity: 1500 Hz at 2000 mm down to 3500 Hz at 300 mm */
#define ALERT_BUZZER_MIN_FREQ_HZ       (1500U)
#define ALERT_BUZZER_MAX_FREQ_HZ       (3500U)
#define ALERT_OBSTACLE_MAX_DIST_MM     (2000U)
#define ALERT_OBSTACLE_MIN_DIST_MM     (300U)

#define ALERT_FALL_FREQ_A_HZ           (2500U)
#define ALERT_FALL_FREQ_B_HZ           (4000U)
#define ALERT_FAULT_FREQ_HZ            (1000U)

#endif /* ALERTMANAGER_CFG_H */
