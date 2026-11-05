#ifndef CANEMODE_CFG_H
#define CANEMODE_CFG_H

/* Continuous qualification windows, not delays in the perception task. */
#define CANEMODE_PARK_MS                 (3000U)
#define CANEMODE_RECOVERY_MS             (1000U)
#define CANEMODE_AUDIO_PAUSE_MS          (15000U)
#define CANEMODE_RELEASE_REALARM_MS      (3000U)
#define CANEMODE_PARK_HOLD_MS            (3000U)
#define CANEMODE_WAKE_HOLD_MS            (100U)
#define CANEMODE_MAX_SAMPLE_AGE_MS       (50U)
#define CANEMODE_MAX_STEP_GAP_MS         (50U)

#define CANEMODE_STILL_ACC_MIN_SQ        (0.81f)
#define CANEMODE_STILL_ACC_MAX_SQ        (1.21f)
#define CANEMODE_STILL_GYRO_MAX_SQ       (25.0f)
#define CANEMODE_PARK_COS_SQ             (0.821394f) /* cos(25 degrees)^2 */
#define CANEMODE_RECOVERY_COS_SQ         (0.75f)     /* cos(30 degrees)^2 */

/* Gravity in sensor coordinates with the cane upright, handle uppermost.
 * Configure for the physical mounting; never learn upright from a fallen boot pose.
 * The default matches the existing +Z-up test fixture, pending mounting validation.
 */
#define CANEMODE_UPRIGHT_X               (0.0f)
#define CANEMODE_UPRIGHT_Y               (0.0f)
#define CANEMODE_UPRIGHT_Z               (1.0f)

#endif /* CANEMODE_CFG_H */
