#ifndef GROUNDBEAM_H
#define GROUNDBEAM_H

#include <stdbool.h>
#include <stdint.h>
#include "Rte_Type.h"

typedef enum
{
    GROUND_BEAM_UNALIGNED = 0U,
    GROUND_BEAM_IMU_UNAVAILABLE,
    GROUND_BEAM_OUT_OF_ANGLE,
    GROUND_BEAM_AVAILABLE
} GroundBeam_Status_t;

typedef struct
{
    float aligned_accel[3];
    float gravity[3];
    float cutoff_cosine;
    float resume_cosine;
    uint32_t last_sample_ms;
    uint32_t last_accel_ms;
    bool aligned;
    bool gravity_valid;
    bool pose_available;
} GroundBeam_t;

void GroundBeam_Init(GroundBeam_t * const p_beam);

/* Use the mean stationary accelerometer sample taken with the beam vertically down.
 * A failed alignment preserves the current reference.
 */
bool GroundBeam_Align(GroundBeam_t * const p_beam, const Imu_Sample_t * const p_mean);

/* Call at IMU rate with the next timestamped sample or NULL. A missing frame
 * produces no estimate for that frame. The optional cosine output is signed
 * and is NAN when there is no angle estimate.
 */
GroundBeam_Status_t GroundBeam_Update(GroundBeam_t * const p_beam,
                                     const Imu_Sample_t * const p_imu,
                                     float * const p_cosine);

/* Project a valid slant range using the cosine from GROUND_BEAM_AVAILABLE. */
uint16_t GroundBeam_Project(uint16_t slant_mm, float cosine);

#endif /* GROUNDBEAM_H */
