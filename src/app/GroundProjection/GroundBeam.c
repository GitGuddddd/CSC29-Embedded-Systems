#include "GroundBeam.h"
#include "GroundBeam_Cfg.h"

#include <math.h>
#include <stddef.h>

#define RADIANS_PER_DEGREE (0.01745329252f)

_Static_assert(GROUND_BEAM_RESUME_DEG < GROUND_BEAM_CUTOFF_DEG &&
               GROUND_BEAM_CUTOFF_DEG < 90U,
               "Beam thresholds must form a hysteresis band below 90 degrees");

static float Acceleration_NormSq(const Imu_Sample_t * const p_imu)
{
    return (p_imu->ax * p_imu->ax) + (p_imu->ay * p_imu->ay) + (p_imu->az * p_imu->az);
}

static float Trusted_AccelerationNorm(float a_sq)
{
    if ((a_sq >= GROUND_BEAM_ACCEL_MIN_SQ) && (a_sq <= GROUND_BEAM_ACCEL_MAX_SQ))
    {
        return sqrtf(a_sq);
    }
    return 0.0f;
}

static void Predict_Gravity(float gravity[3], const Imu_Sample_t * const p_imu,
                            float g_sq, uint32_t dt_ms)
{
    if (g_sq == 0.0f)
    {
        return;
    }

    float rate = sqrtf(g_sq);
    float angle = rate * (float)dt_ms * (RADIANS_PER_DEGREE * 0.001f);
    float x = p_imu->gx / rate;
    float y = p_imu->gy / rate;
    float z = p_imu->gz / rate;
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);
    float parallel = ((x * gravity[0]) + (y * gravity[1]) + (z * gravity[2])) *
                     (1.0f - cos_angle);

    /* The stationary acceleration vector rotates opposite the IMU so use g x omega. */
    float gx = (cos_angle * gravity[0]) + (sin_angle * ((gravity[1] * z) - (gravity[2] * y))) +
               (parallel * x);
    float gy = (cos_angle * gravity[1]) + (sin_angle * ((gravity[2] * x) - (gravity[0] * z))) +
               (parallel * y);
    float gz = (cos_angle * gravity[2]) + (sin_angle * ((gravity[0] * y) - (gravity[1] * x))) +
               (parallel * z);
    gravity[0] = gx;
    gravity[1] = gy;
    gravity[2] = gz;
}

static bool Update_Gravity(GroundBeam_t * const p_beam, const Imu_Sample_t * const p_imu)
{
    const uint32_t now_ms = p_imu->timestamp_ms;
    const uint32_t dt_ms = now_ms - p_beam->last_sample_ms;
    if (p_beam->gravity_valid && (dt_ms == 0U))
    {
        return false;
    }

    float a_sq = Acceleration_NormSq(p_imu);
    float g_sq = (p_imu->gx * p_imu->gx) + (p_imu->gy * p_imu->gy) +
                 (p_imu->gz * p_imu->gz);
    if (!isfinite(a_sq) || (a_sq < GROUND_BEAM_FREE_FALL_SQ) || !isfinite(g_sq) ||
        (fabsf(p_imu->gx) >= GROUND_BEAM_GYRO_SATURATION_DPS) ||
        (fabsf(p_imu->gy) >= GROUND_BEAM_GYRO_SATURATION_DPS) ||
        (fabsf(p_imu->gz) >= GROUND_BEAM_GYRO_SATURATION_DPS))
    {
        p_beam->gravity_valid = false;
        return false;
    }

    float norm = Trusted_AccelerationNorm(a_sq);
    float * const p_gravity = p_beam->gravity;
    if (!p_beam->gravity_valid || (dt_ms > GROUND_BEAM_SAMPLE_GAP_MS) ||
        ((uint32_t)(now_ms - p_beam->last_accel_ms) > GROUND_BEAM_GYRO_ONLY_LIMIT_MS))
    {
        p_beam->gravity_valid = (norm > 0.0f) && (g_sq <= GROUND_BEAM_SEED_GYRO_MAX_SQ);
        if (!p_beam->gravity_valid)
        {
            return false;
        }
        p_gravity[0] = p_imu->ax / norm;
        p_gravity[1] = p_imu->ay / norm;
        p_gravity[2] = p_imu->az / norm;
        p_beam->last_accel_ms = now_ms;
    }
    else
    {
        Predict_Gravity(p_gravity, p_imu, g_sq, dt_ms);
        if (norm > 0.0f)
        {
            float ax = p_imu->ax / norm;
            float ay = p_imu->ay / norm;
            float az = p_imu->az / norm;
            float agreement = (ax * p_gravity[0]) + (ay * p_gravity[1]) +
                              (az * p_gravity[2]);
            if (agreement >= GROUND_BEAM_CORRECTION_COSINE)
            {
                float alpha = (float)dt_ms / (GROUND_BEAM_CORRECTION_TAU_MS + (float)dt_ms);
                float x = p_gravity[0] + (alpha * (ax - p_gravity[0]));
                float y = p_gravity[1] + (alpha * (ay - p_gravity[1]));
                float z = p_gravity[2] + (alpha * (az - p_gravity[2]));
                float scale = 1.0f / sqrtf((x * x) + (y * y) + (z * z));
                p_gravity[0] = x * scale;
                p_gravity[1] = y * scale;
                p_gravity[2] = z * scale;
                p_beam->last_accel_ms = now_ms;
            }
        }
    }
    p_beam->last_sample_ms = now_ms;
    return true;
}

void GroundBeam_Init(GroundBeam_t * const p_beam)
{
    if (p_beam != NULL)
    {
        *p_beam = (GroundBeam_t){
            .cutoff_cosine = cosf((float)GROUND_BEAM_CUTOFF_DEG * RADIANS_PER_DEGREE),
            .resume_cosine = cosf((float)GROUND_BEAM_RESUME_DEG * RADIANS_PER_DEGREE)
        };
    }
}

bool GroundBeam_Align(GroundBeam_t * const p_beam, const Imu_Sample_t * const p_mean)
{
    if ((p_beam == NULL) || (p_mean == NULL))
    {
        return false;
    }

    float norm = Trusted_AccelerationNorm(Acceleration_NormSq(p_mean));
    if (norm == 0.0f)
    {
        return false;
    }

    /* The accelerometer's sign convention is the same in both samples. */
    p_beam->aligned_accel[0] = p_mean->ax / norm;
    p_beam->aligned_accel[1] = p_mean->ay / norm;
    p_beam->aligned_accel[2] = p_mean->az / norm;
    p_beam->aligned = true;
    p_beam->gravity_valid = false;
    p_beam->pose_available = false;
    return true;
}

GroundBeam_Status_t GroundBeam_Update(GroundBeam_t * const p_beam,
                                     const Imu_Sample_t * const p_imu,
                                     float * const p_cosine)
{
    if (p_cosine != NULL)
    {
        *p_cosine = NAN;
    }
    if ((p_beam == NULL) || !p_beam->aligned)
    {
        return GROUND_BEAM_UNALIGNED;
    }
    if ((p_imu == NULL) || !Update_Gravity(p_beam, p_imu))
    {
        p_beam->pose_available = false;
        return GROUND_BEAM_IMU_UNAVAILABLE;
    }

    const float * const p_aligned = p_beam->aligned_accel;
    const float * const p_gravity = p_beam->gravity;
    float dot = (p_aligned[0] * p_gravity[0]) + (p_aligned[1] * p_gravity[1]) +
                (p_aligned[2] * p_gravity[2]);
    float cosine = fmaxf(-1.0f, fminf(1.0f, dot));
    if (p_cosine != NULL)
    {
        *p_cosine = cosine;
    }

    float min_cosine = p_beam->pose_available ? p_beam->cutoff_cosine : p_beam->resume_cosine;
    p_beam->pose_available = (cosine >= min_cosine);
    return p_beam->pose_available ? GROUND_BEAM_AVAILABLE : GROUND_BEAM_OUT_OF_ANGLE;
}

uint16_t GroundBeam_Project(uint16_t slant_mm, float cosine)
{
    if (!(cosine > 0.0f))
    {
        return 0U;
    }
    return (uint16_t)(((float)slant_mm * fminf(1.0f, cosine)) + 0.5f);
}
