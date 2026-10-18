#ifndef GROUNDPROJECTION_CFG_H
#define GROUNDPROJECTION_CFG_H

/*
 * \file GroundProjection_Cfg.h
 * \brief Compile-time config for GroundProjection (Cliff Detection)
 * Compliance: Barr-C:2018
 */

#define GP_CAL_SAMPLES            (25U)
#define GP_CAL_SPREAD_MM          (40U)
#define GP_MIN_RANGE_MM           (100U)
#define GP_MAX_RANGE_MM           (3000U)

/* Step heights relative to baseline (mm) */
#define GP_UP_ENTER_MM            (80)
#define GP_DOWN_ENTER_MM          (150)
#define GP_UP_CLEAR_MM            (50)
#define GP_DOWN_CLEAR_MM          (60)

/* Severe cliff fast-path: bypass 3-sample voting if drop > 250mm */
#define GP_DOWN_FASTPATH_MM       (250)

#define GP_CONFIRM_SAMPLES        (3U)
#define GP_STALE_TIMEOUT_MS       (500U)

/* Integer EMA: (delta * 5) >> 8 =~ 0.0195 (~2%) */
#define GP_EMA_NUM                (5)
#define GP_EMA_DEN_SHIFT          (8)

#endif /* GROUNDPROJECTION_CFG_H */
