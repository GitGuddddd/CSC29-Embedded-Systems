/** @file feature_cfg.h
 *
 * @brief Global build switches shared by the Cane and the Glasses node.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * NOTE: Debug output itself is guarded with "#ifndef NDEBUG" (Barr-C 2.1.c).
 * FEATURE_SIMULATION_MODE is reserved for bench runs without the sensor.
 */

#ifndef FEATURE_CFG_H
#define FEATURE_CFG_H

// 1 = the radar task prints a status line every
// FEATURE_DEBUG_LOG_FRAME_INTERVAL frames. Only has an effect when NDEBUG is
// not defined.
#define FEATURE_DEBUG_LOGS                  (1)
#define FEATURE_DEBUG_LOG_FRAME_INTERVAL    (20u)

// 1 = print x, y and speed of all three target slots for every valid frame
// (bench diagnosis of range jumps and the departing filter). Only has an
// effect when NDEBUG is not defined. Turn it off for normal runs: it prints
// about 11 lines per second.
#define FEATURE_DEBUG_LOG_TARGETS           (0)

// 1 = use scripted data instead of the real sensor (not implemented yet).
#define FEATURE_SIMULATION_MODE             (0)

#endif /* FEATURE_CFG_H */

/*** end of file ***/
