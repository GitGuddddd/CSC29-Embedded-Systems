#ifndef LD2450_CFG_H
#define LD2450_CFG_H

/*
 * \file Ld2450_Cfg.h
 * \brief Compile-time configuration for HLK-LD2450 mmWave radar driver
 * Compliance: Barr-C:2018
 */

#define LD2450_CFG_BAUDRATE           (256000U)
#define LD2450_CFG_CIRC_BUF_SIZE      (1024U)
#define LD2450_CFG_MAX_TARGETS        (3U)
#define LD2450_CFG_FRAME_TIMEOUT_MS   (500U)
#define LD2450_CFG_FRAME_BYTES        (30U)
#define LD2450_CFG_HEADER_BYTES       (4U)
#define LD2450_CFG_PARSE_BUDGET_BYTES (512U)
#define LD2450_CFG_MAX_POLL_GAP_MS    (100U)

#endif /* LD2450_CFG_H */
