#ifndef THREATASSESSMENT_CFG_H
#define THREATASSESSMENT_CFG_H

/*
 * \file ThreatAssessment_Cfg.h
 * \brief Compile-time parameters for mmWave radar obstacle decision
 * Compliance: Barr-C:2018 (FR8, FR13)
 */

#define THREAT_ALERT_ENTER_DIST_MM   (2000U)
#define THREAT_ALERT_CLEAR_DIST_MM   (2100U)

/* Target departing faster than 10 cm/s (-10 cm/s) is ignored */
#define THREAT_DEPARTING_SPEED_LIMIT (-10)

#define THREAT_RADAR_TIMEOUT_MS      (500U)

#endif /* THREATASSESSMENT_CFG_H */
