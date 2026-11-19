#include "FallDetection.h"
#include "FallDetection_Cfg.h"

/*
 * \file FallDetection_Lcfg.c
 * \brief Link-time calibrated fall detection for the glasses
 * Compliance: Barr-C:2018
 */

const Fall_Config_t FallConfig_Head = {
    .freefall_thresh_sq  = FALL_FF_THRESH_G_SQ,
    .impact_thresh_sq    = FALL_IMPACT_THRESH_G_SQ,
    .freefall_min_ms     = FALL_FF_MIN_MS,
    .impact_window_ms    = FALL_IMPACT_WINDOW_MS,
    .settle_ms           = FALL_SETTLE_MS,
    .still_ms            = FALL_STILL_MS,
    .pending_ms          = FALL_PENDING_MS
};
