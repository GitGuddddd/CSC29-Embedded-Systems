#ifndef HAPTIC_CFG_H
#define HAPTIC_CFG_H

/*
 * \file Haptic_Cfg.h
 * \brief Compile-time configuration for Cane haptic vibration motor (FR16, FR19)
 * Compliance: Barr-C:2018
 */

#define HAPTIC_CFG_PWM_FREQ_HZ        (1000U)
#define HAPTIC_CFG_PWM_DUTY_PCT       (70U)

/* Pattern: 3 rapid 150ms pulses for STEP_DOWN physical cliff */
#define HAPTIC_CFG_CLIFF_PULSE_MS     (150U)
#define HAPTIC_CFG_CLIFF_GAP_MS       (100U)

/* Pattern: 2 long 500ms pulses for LINK_LOSS network warning */
#define HAPTIC_CFG_LINKLOSS_PULSE_MS  (500U)
#define HAPTIC_CFG_LINKLOSS_GAP_MS    (200U)

#endif /* HAPTIC_CFG_H */
