#include "FallDetection.h"
#include "FallDetection_Cfg.h"

/*
 * \file FallDetection_Lcfg.c
 * \brief Link-time calibrated configurations for Head vs Cane nodes
 * Compliance: Barr-C:2018
 */

const Fall_Config_t FallConfig_Head = {
    .freefall_thresh_sq = FALL_FF_THRESH_G_SQ,
    .impact_thresh_sq   = FALL_IMPACT_THRESH_G_SQ,
    .freefall_min_ms    = FALL_FF_MIN_MS,
    .impact_window_ms   = FALL_IMPACT_WINDOW_MS,
    .settle_ms          = FALL_SETTLE_MS,
    .still_ms           = FALL_STILL_MS
};

const Fall_Config_t FallConfig_Cane = {
    .freefall_thresh_sq = 0.1225f, /* 0.35g threshold squared */
    .impact_thresh_sq   = FALL_IMPACT_THRESH_G_SQ,
    .freefall_min_ms    = 30U,
    .impact_window_ms   = 800U,
    .settle_ms          = 600U,
    .still_ms           = 400U
};
