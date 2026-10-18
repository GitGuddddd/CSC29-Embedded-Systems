#ifndef TOUCHSENSOR_CFG_H
#define TOUCHSENSOR_CFG_H

/*
 * \file TouchSensor_Cfg.h
 * \brief Compile-time configuration for capacitive touch grip sensor (FR1, FR2)
 * Compliance: Barr-C:2018
 */

#define TOUCH_CFG_SAMPLE_INTERVAL_MS  (5U)
#define TOUCH_CFG_DEBOUNCE_MS         (20U)
#define TOUCH_CFG_DEBOUNCE_COUNT      (TOUCH_CFG_DEBOUNCE_MS / TOUCH_CFG_SAMPLE_INTERVAL_MS)

#endif /* TOUCHSENSOR_CFG_H */
