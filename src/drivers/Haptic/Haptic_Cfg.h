#ifndef HAPTIC_CFG_H
#define HAPTIC_CFG_H

/*
 * \file Haptic_Cfg.h
 * \brief Compile-time configuration for Cane haptic vibration motor (FR16, FR19)
 * Compliance: Barr-C:2018
 *
 * Every cue differs in pulse count and/or pulse length so the user can tell
 * them apart by feel (Tribunal 1 Version C gap: four cues were all "two pulses").
 */

#define HAPTIC_CFG_PWM_FREQ_HZ        (1000U)
#define HAPTIC_CFG_PWM_DUTY_PCT       (70U)

/* STEP_DOWN (physical cliff): 3 short pulses, repeat every 1.4 s */
#define HAPTIC_CFG_CLIFF_PULSES       (3U)
#define HAPTIC_CFG_CLIFF_PULSE_MS     (150U)
#define HAPTIC_CFG_CLIFF_GAP_MS       (100U)
#define HAPTIC_CFG_CLIFF_PERIOD_MS    (1400U)

/* STEP_UP (raised ground): 2 short pulses, repeat every 1.4 s */
#define HAPTIC_CFG_STEPUP_PULSES      (2U)
#define HAPTIC_CFG_STEPUP_PULSE_MS    (150U)
#define HAPTIC_CFG_STEPUP_GAP_MS      (100U)
#define HAPTIC_CFG_STEPUP_PERIOD_MS   (1400U)

/* Caution (ToF fault / no ground): 1 long pulse, repeat every 1.4 s */
#define HAPTIC_CFG_CAUTION_PULSES     (1U)
#define HAPTIC_CFG_CAUTION_PULSE_MS   (600U)
#define HAPTIC_CFG_CAUTION_GAP_MS     (0U)
#define HAPTIC_CFG_CAUTION_PERIOD_MS  (1400U)

/* LINK_LOSS (network warning): 2 long pulses, repeat every 3 s */
#define HAPTIC_CFG_LINKLOSS_PULSES    (2U)
#define HAPTIC_CFG_LINKLOSS_PULSE_MS  (500U)
#define HAPTIC_CFG_LINKLOSS_GAP_MS    (200U)
#define HAPTIC_CFG_LINKLOSS_PERIOD_MS (3000U)

/* Calibration prompt (no valid ground baseline): 4 quick ticks, repeat every 3 s */
#define HAPTIC_CFG_CALPROMPT_PULSES    (4U)
#define HAPTIC_CFG_CALPROMPT_PULSE_MS  (80U)
#define HAPTIC_CFG_CALPROMPT_GAP_MS    (80U)
#define HAPTIC_CFG_CALPROMPT_PERIOD_MS (3000U)

/*
 * Calibration feedback (Week 6 section 6.3 field procedure). These play once
 * through Haptic_PlayOnce(); the period equals the cue length.
 */
/* Started: 1 short pulse */
#define HAPTIC_CFG_CALSTART_PULSES     (1U)
#define HAPTIC_CFG_CALSTART_PULSE_MS   (150U)
#define HAPTIC_CFG_CALSTART_GAP_MS     (0U)
#define HAPTIC_CFG_CALSTART_PERIOD_MS  (150U)

/* Baseline saved: 1 pulse of 1 s */
#define HAPTIC_CFG_CALDONE_PULSES      (1U)
#define HAPTIC_CFG_CALDONE_PULSE_MS    (1000U)
#define HAPTIC_CFG_CALDONE_GAP_MS      (0U)
#define HAPTIC_CFG_CALDONE_PERIOD_MS   (1000U)

/* Failed, previous baseline kept: 2 pulses of 1 s */
#define HAPTIC_CFG_CALFAIL_PULSES      (2U)
#define HAPTIC_CFG_CALFAIL_PULSE_MS    (1000U)
#define HAPTIC_CFG_CALFAIL_GAP_MS      (300U)
#define HAPTIC_CFG_CALFAIL_PERIOD_MS   (2300U)

#endif /* HAPTIC_CFG_H */
