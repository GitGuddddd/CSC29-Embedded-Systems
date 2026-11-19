#ifndef FEATURE_CFG_H
#define FEATURE_CFG_H

/*
 * \file Feature_Cfg.h
 * \brief Compile-time feature flags and debug switches
 * Compliance: Barr-C:2018
 */

#define FEATURE_ENABLE_WATCHDOG_REBOOT   (1U)
#define FEATURE_ENABLE_RTT_LOGGING       (0U)
#define FEATURE_ENABLE_FUZZING_STUBS     (0U)

/* Bench trial. Only a lateral slip followed by settled tilt raises a cane IMU alarm. */
#define FEATURE_CANE_SLIP_ONLY_TRIAL     (1U)

#endif /* FEATURE_CFG_H */
