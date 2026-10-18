#ifndef FALLDETECTION_CFG_H
#define FALLDETECTION_CFG_H

/*
 * \file FallDetection_Cfg.h
 * \brief Compile-time thresholds for FallDetection SWC
 * Uses squared thresholds to prevent software float emulation on Cortex-M0+ (Barr-C:2018)
 */

/* 0.5g -> 0.25g^2 */
#define FALL_FF_THRESH_G_SQ       (0.25f)

/* 2.5g -> 6.25g^2 */
#define FALL_IMPACT_THRESH_G_SQ   (6.25f)

#define FALL_FF_MIN_MS            (50U)
#define FALL_IMPACT_WINDOW_MS     (1000U)
#define FALL_SETTLE_MS            (800U)
#define FALL_STILL_MS             (500U)

/* Stillness acceleration tolerance: (1.0 +/- 0.15)g -> [0.85^2, 1.15^2] */
#define FALL_STILL_ACC_MIN_G_SQ   (0.7225f)
#define FALL_STILL_ACC_MAX_G_SQ   (1.3225f)

/* Gyro stillness threshold: (30 dps)^2 */
#define FALL_STILL_GYRO_DPS_SQ    (900.0f)

/* Cos(45 degrees) */
#define FALL_TILT_COS_THRESH      (0.707f)

#endif /* FALLDETECTION_CFG_H */
