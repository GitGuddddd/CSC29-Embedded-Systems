#ifndef ALERTMANAGER_CFG_H
#define ALERTMANAGER_CFG_H

/*
 * \file AlertManager_Cfg.h
 * \brief Alert arbitration and tone/vibe parameters (FR3, FR9, FR10)
 * Compliance: Barr-C:2018
 */

#define ALERT_MAX_FALL_TIMEOUT_MS      (30000U) /* 30 seconds auto-clear */

/* Pitch mapping for obstacle proximity: 1500 Hz at 2000 mm down to 3500 Hz at 300 mm */
#define ALERT_BUZZER_MIN_FREQ_HZ       (1500U)
#define ALERT_BUZZER_MAX_FREQ_HZ       (3500U)
#define ALERT_OBSTACLE_MAX_DIST_MM     (2000U)
#define ALERT_OBSTACLE_MIN_DIST_MM     (300U)

#define ALERT_FALL_FREQ_A_HZ           (2500U)
#define ALERT_FALL_FREQ_B_HZ           (4000U)
#define ALERT_FAULT_FREQ_HZ            (1000U)

#endif /* ALERTMANAGER_CFG_H */
