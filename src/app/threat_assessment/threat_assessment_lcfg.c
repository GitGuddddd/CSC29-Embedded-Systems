/** @file threat_assessment_lcfg.c
 *
 * @brief Link-time configuration of the obstacle decision.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>

#include "threat_assessment.h"

// NOTE: The manual does not say which sign means "approaching". Its example
// of -16 cm/s for a person walking toward the radar and the bench logs agree
// with negative = approaching. If the walk-toward test (A8) shows the
// opposite, change this one value to false.
const threat_assessment_lcfg_t g_threat_assessment_lcfg =
{
    .b_negative_speed_is_approaching = true
};

/*** end of file ***/
