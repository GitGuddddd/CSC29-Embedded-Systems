#ifndef GROUNDBEAM_CFG_H
#define GROUNDBEAM_CFG_H

/* Prototype limits measured from vertical. Tune on the assembled cane. */
#define GROUND_BEAM_CUTOFF_DEG      (35U)
#define GROUND_BEAM_RESUME_DEG      (30U)

/* Accelerometer corrections must agree with gravity predicted by the gyro.
 * Seeding an estimate also requires a moderate rotation rate.
 */
#define GROUND_BEAM_ACCEL_MIN_SQ       (0.81f)   /* (0.9 g)^2 */
#define GROUND_BEAM_ACCEL_MAX_SQ       (1.21f)   /* (1.1 g)^2 */
#define GROUND_BEAM_SEED_GYRO_MAX_SQ    (3600.0f) /* (60 deg/s)^2 */
#define GROUND_BEAM_CORRECTION_COSINE   (0.93969262f) /* cos(20 degrees) */
#define GROUND_BEAM_CORRECTION_TAU_MS   (500.0f)
#define GROUND_BEAM_GYRO_ONLY_LIMIT_MS  (750U)
#define GROUND_BEAM_SAMPLE_GAP_MS       (30U)
#define GROUND_BEAM_FREE_FALL_SQ        (0.25f)   /* (0.5 g)^2 */
#define GROUND_BEAM_GYRO_SATURATION_DPS (990.0f)  /* Configured range is +/-1000 deg/s. */

#endif /* GROUNDBEAM_CFG_H */
