#ifndef CALBUTTON_CFG_H
#define CALBUTTON_CFG_H

/*
 * \file CalButton_Cfg.h
 * \brief Compile-time configuration for the cane calibration button (FR18, Week 6 section 6.3)
 * Compliance: Barr-C:2018
 */

/* A level must stay unchanged this long before it is accepted */
#define CALBTN_CFG_DEBOUNCE_MS      (30U)

/* Hold time that starts ground calibration */
#define CALBTN_CFG_GROUND_HOLD_MS   (2000U)

#endif /* CALBUTTON_CFG_H */
