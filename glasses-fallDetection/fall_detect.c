/**
 * @file    fall_detect.c
 * @brief   Head fall detector: free fall, impact, stillness and tilt check.
 *
 * A fall is confirmed in four steps:
 *   1. Free fall: the acceleration magnitude stays below FF_THRESH_G for at
 *      least FF_MIN_MS.
 *   2. Impact: the magnitude goes above IMPACT_THRESH_G within
 *      IMPACT_WINDOW_MS (or straight away when free fall ends in a hit).
 *   3. Settling: the bouncing after the impact is ignored for SETTLE_MS.
 *   4. Stillness and tilt: once the head has been still for STILL_MS, the
 *      angle between the normal gravity direction and the current one must
 *      be larger than TILT_DEG.
 *
 * The detector state is stored in the private record of node_status.c,
 * which keeps it in 3 bits next to the status flags.
 *
 * NOTE: A fall must start with free fall. A hard hit alone, such as setting
 * the head down or bumping it, is not enough. That case caused false alarms.
 */

/*
 * STATE MACHINE
 *
 * The detector is a state machine. The current state is read at the start
 * of fall_detect_update() and at most one state change happens per sample.
 * The time in a state is measured from g_start_ms, which is set whenever a
 * state is entered (CHECK_STILL also restarts it, see below).
 *
 * States:
 *   IDLE         Normal wear, no fall in progress. The reference gravity
 *                direction g_ref is learned from still samples only.
 *   FREEFALL     The magnitude has dropped below FF_THRESH_G: the head may
 *                be falling.
 *   WAIT_IMPACT  Free fall has ended without a hit yet. Waiting for one.
 *   SETTLING     The impact has happened. The bouncing is ignored.
 *   CHECK_STILL  Waiting for the head to come to rest, then judging the
 *                final pose against g_ref.
 *
 * Transitions (acc is the acceleration magnitude in g):
 *   From          To           Condition
 *   IDLE          FREEFALL     acc < FF_THRESH_G.
 *   FREEFALL      IDLE         acc >= FF_THRESH_G after less than FF_MIN_MS
 *                              (too short, not a fall).
 *   FREEFALL      SETTLING     acc >= FF_THRESH_G after at least FF_MIN_MS,
 *                              and acc > IMPACT_THRESH_G on that sample.
 *   FREEFALL      WAIT_IMPACT  acc >= FF_THRESH_G after at least FF_MIN_MS,
 *                              and acc <= IMPACT_THRESH_G on that sample.
 *   WAIT_IMPACT   SETTLING     acc > IMPACT_THRESH_G.
 *   WAIT_IMPACT   IDLE         IMPACT_WINDOW_MS passed without an impact.
 *   SETTLING      CHECK_STILL  SETTLE_MS passed. g_cur is started from the
 *                              current sample.
 *   CHECK_STILL   IDLE         The head was still (acc within
 *                              STILL_ACC_TOL_G of 1 g and rotation below
 *                              STILL_GYR_DPS) for STILL_MS. A fall is
 *                              reported only if the angle between g_ref
 *                              and g_cur is more than TILT_DEG.
 *   CHECK_STILL   IDLE         CHECK_TIMEOUT_MS passed since CHECK_STILL
 *                              began and the head is not still (no fall).
 *   any other     IDLE         An invalid state number (safety default).
 *
 * Notes:
 *   - fall_detect_update() returns true for exactly one call, on the
 *     CHECK_STILL to IDLE change that confirms a fall.
 *   - FREEFALL has no timeout of its own. It leaves only when the
 *     magnitude rises to FF_THRESH_G or more.
 *   - A slow lean or bend keeps the magnitude near 1 g. A fast one can dip
 *     below FF_THRESH_G and start FREEFALL (seen in the trial logs). The
 *     tilt check then compares with g_ref. g_ref follows only still
 *     samples, fast for the first REF_FAST_SAMPLES after a reset and then
 *     slowly, so a short bend does not become the normal pose.
 *   - fall_detect_reset() forces IDLE. The sensor health logic calls it
 *     when the sensor fails and when it recovers.
 */

#include "fall_detect.h"

#include <math.h>

#include "node_status.h"

/* Tunable thresholds. They are starting points, tuned from the trial logs. */
#define FF_THRESH_G       0.5f    /* Magnitude below this is free fall. */
#define FF_MIN_MS         50u     /* Free fall must last this long. */
#define IMPACT_THRESH_G   2.5f    /* Magnitude above this is an impact. */
#define IMPACT_WINDOW_MS  1000u   /* The impact must follow within this. */
#define SETTLE_MS         800u    /* Bouncing after the impact is ignored. */
#define STILL_MS          500u    /* The head must be still this long. */
#define CHECK_TIMEOUT_MS  3000u   /* Give up if not at rest by then. */
#define STILL_ACC_TOL_G   0.15f   /* Magnitude within 1 g +- this is still. */
#define STILL_GYR_DPS     30.0f   /* Rotation below this is still. */
#define TILT_DEG          45.0f   /* Head orientation change that counts. */

/* Constants of the calculation. */
#define AXIS_COUNT        3u
#define GRAVITY_G         1.0f
#define REF_GAIN_FAST     0.01f   /* About 1 s of averaging at 100 Hz. */
#define REF_GAIN_SLOW     0.0005f /* About 20 s of averaging at 100 Hz. */
#define REF_FAST_SAMPLES  3000u   /* Still samples learned fast: 30 s. */
#define CUR_GAIN          0.1f    /* About 0.1 second of averaging. */
#define MIN_NORM          1.0e-6f
#define RAD_TO_DEG        57.2958f

typedef enum
{
    FALL_STATE_IDLE,
    FALL_STATE_FREEFALL,
    FALL_STATE_WAIT_IMPACT,
    FALL_STATE_SETTLING,
    FALL_STATE_CHECK_STILL,
    FALL_STATE_COUNT
} fall_state_t;

static const char * const gp_state_names[] =
{
    "IDLE",
    "FREEFALL",
    "WAIT_IMPACT",
    "SETTLING",
    "CHECK_STILL"
};

static uint32_t g_start_ms = 0u;   /* Start of the current state. */
static uint32_t g_check_ms = 0u;   /* Start of CHECK_STILL. */
static float    g_ref[AXIS_COUNT] = { 0.0f, 0.0f, 1.0f };
static float    g_cur[AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
static uint16_t g_ref_fast_left = REF_FAST_SAMPLES;   /* Fast samples left. */

static fall_state_t get_state (void);
static void         set_state (fall_state_t state);
static float vector_length (float vec_x, float vec_y, float vec_z);
static float angle_between_deg (const float * p_first, const float * p_second);
static void  average_into (float * p_avg, const imu_sample_t * p_sample,
                           float gain);
static bool  check_still (const imu_sample_t * p_sample, float acc_g,
                          float gyro_dps, uint32_t now_ms);
static bool  is_still (float acc_g, float gyro_dps);
static void  learn_reference (const imu_sample_t * p_sample, float acc_g,
                              float gyro_dps);

/**
 * @brief  Returns the name of the current state, for logging.
 * @return Pointer to a constant string.
 */
const char * fall_detect_state_name (void)
{
    uint8_t      index  = node_status_get_fall_state();
    const char * p_name = "UNKNOWN";

    if (index < FALL_STATE_COUNT)
    {
        p_name = gp_state_names[index];
    }

    return p_name;
}

/**
 * @brief Returns the detector to its idle state.
 */
void fall_detect_reset (void)
{
    set_state(FALL_STATE_IDLE);
    g_ref_fast_left = REF_FAST_SAMPLES;
}

/**
 * @brief  Feeds one sample into the detector.
 * @param  p_sample  Latest sensor sample.
 * @param  now_ms    Milliseconds since boot.
 * @return true exactly once when a fall is confirmed.
 */
bool fall_detect_update (const imu_sample_t * p_sample, uint32_t now_ms)
{
    float acc_g    = vector_length(p_sample->accel_x, p_sample->accel_y,
                                   p_sample->accel_z);
    float gyro_dps = vector_length(p_sample->gyro_x, p_sample->gyro_y,
                                   p_sample->gyro_z);
    bool  b_fall   = false;

    switch (get_state())
    {
        case FALL_STATE_IDLE:
            /* The reference learns the normal head pose from still samples
             * only: fast after a reset, then slowly. */
            learn_reference(p_sample, acc_g, gyro_dps);
            if (acc_g < FF_THRESH_G)
            {
                set_state(FALL_STATE_FREEFALL);
                g_start_ms = now_ms;
            }
        break;

        case FALL_STATE_FREEFALL:
            if (acc_g >= FF_THRESH_G)
            {
                if ((now_ms - g_start_ms) < FF_MIN_MS)
                {
                    set_state(FALL_STATE_IDLE);
                }
                else if (acc_g > IMPACT_THRESH_G)
                {
                    set_state(FALL_STATE_SETTLING);
                    g_start_ms = now_ms;
                }
                else
                {
                    set_state(FALL_STATE_WAIT_IMPACT);
                    g_start_ms = now_ms;
                }
            }
        break;

        case FALL_STATE_WAIT_IMPACT:
            if (acc_g > IMPACT_THRESH_G)
            {
                set_state(FALL_STATE_SETTLING);
                g_start_ms = now_ms;
            }
            else if ((now_ms - g_start_ms) > IMPACT_WINDOW_MS)
            {
                set_state(FALL_STATE_IDLE);
            }
            else
            {
                /* Still waiting for the impact. */
            }
        break;

        case FALL_STATE_SETTLING:
            if ((now_ms - g_start_ms) >= SETTLE_MS)
            {
                set_state(FALL_STATE_CHECK_STILL);
                g_start_ms = now_ms;
                g_check_ms = now_ms;
                g_cur[0]   = p_sample->accel_x;
                g_cur[1]   = p_sample->accel_y;
                g_cur[2]   = p_sample->accel_z;
            }
        break;

        case FALL_STATE_CHECK_STILL:
            b_fall = check_still(p_sample, acc_g, gyro_dps, now_ms);
        break;

        default:
            set_state(FALL_STATE_IDLE);
        break;
    }

    return b_fall;
}

/**
 * @brief  Length of a three-axis vector.
 * @param  vec_x  X component.
 * @param  vec_y  Y component.
 * @param  vec_z  Z component.
 * @return The length, never negative.
 */
static float vector_length (float vec_x, float vec_y, float vec_z)
{
    return sqrtf((vec_x * vec_x) + (vec_y * vec_y) + (vec_z * vec_z));
}

/**
 * @brief  Angle between two three-axis vectors.
 * @param  p_first   First vector.
 * @param  p_second  Second vector.
 * @return The angle in degrees, or 0 if either vector is almost zero.
 */
static float angle_between_deg (const float * p_first, const float * p_second)
{
    float angle = 0.0f;
    float dot   = (p_first[0] * p_second[0]) + (p_first[1] * p_second[1]) +
                  (p_first[2] * p_second[2]);
    float norm  = vector_length(p_first[0], p_first[1], p_first[2]) *
                  vector_length(p_second[0], p_second[1], p_second[2]);
    float cosine;

    if (norm >= MIN_NORM)
    {
        cosine = dot / norm;
        if (cosine > 1.0f)
        {
            cosine = 1.0f;
        }
        else if (cosine < -1.0f)
        {
            cosine = -1.0f;
        }
        else
        {
            /* The cosine is already in range. */
        }
        angle = acosf(cosine) * RAD_TO_DEG;
    }

    return angle;
}

/**
 * @brief Moves a running average of the acceleration towards a sample.
 * @param p_avg     The three average values, updated in place.
 * @param p_sample  Latest sensor sample.
 * @param gain      Share of the new sample, between 0 and 1.
 */
static void average_into (float * p_avg, const imu_sample_t * p_sample,
                          float gain)
{
    p_avg[0] += gain * (p_sample->accel_x - p_avg[0]);
    p_avg[1] += gain * (p_sample->accel_y - p_avg[1]);
    p_avg[2] += gain * (p_sample->accel_z - p_avg[2]);
}

/**
 * @brief  Reads the current detector state from the shared record.
 * @return The current state.
 */
static fall_state_t get_state (void)
{
    /* Cast: the stored number is always one of the states, 0 to 4. */
    return (fall_state_t)node_status_get_fall_state();
}

/**
 * @brief Writes the detector state into the shared record.
 * @param state  The new state.
 */
static void set_state (fall_state_t state)
{
    /* Cast: the states are 0 to 4, which fit in 8 bits. */
    node_status_set_fall_state((uint8_t)state);
}

/**
 * @brief  Runs the CHECK_STILL state: waits until the head is still, then
 *         compares its orientation with the normal one.
 * @param  p_sample  Latest sensor sample.
 * @param  acc_g     Acceleration magnitude of the sample in g.
 * @param  gyro_dps  Rotation rate magnitude of the sample in dps.
 * @param  now_ms    Milliseconds since boot.
 * @return true if a fall is confirmed.
 */
static bool check_still (const imu_sample_t * p_sample, float acc_g,
                         float gyro_dps, uint32_t now_ms)
{
    bool b_fall = false;

    average_into(g_cur, p_sample, CUR_GAIN);

    if (!is_still(acc_g, gyro_dps))
    {
        /* Still rocking or bouncing: restart the stillness timer and keep
         * waiting until the timeout. */
        g_start_ms = now_ms;
        if ((now_ms - g_check_ms) >= CHECK_TIMEOUT_MS)
        {
            set_state(FALL_STATE_IDLE);
        }
    }
    else if ((now_ms - g_start_ms) >= STILL_MS)
    {
        set_state(FALL_STATE_IDLE);
        if (angle_between_deg(g_ref, g_cur) > TILT_DEG)
        {
            b_fall = true;
        }
    }
    else
    {
        /* Still, but not for long enough yet. */
    }

    return b_fall;
}

/**
 * @brief  Tells whether the head is still.
 * @param  acc_g     Acceleration magnitude in g.
 * @param  gyro_dps  Rotation rate magnitude in dps.
 * @return true if the magnitude is within STILL_ACC_TOL_G of 1 g and the
 *         rotation is at most STILL_GYR_DPS.
 */
static bool is_still (float acc_g, float gyro_dps)
{
    return (fabsf(acc_g - GRAVITY_G) <= STILL_ACC_TOL_G) &&
           (gyro_dps <= STILL_GYR_DPS);
}

/**
 * @brief Updates g_ref, the normal gravity direction, from one sample.
 *
 * Only still samples are learned. The first REF_FAST_SAMPLES of them after a
 * reset use REF_GAIN_FAST, so the reference finds the worn pose quickly.
 * After that REF_GAIN_SLOW is used, so a bend held for a few seconds does
 * not become the normal pose.
 *
 * @param p_sample  Latest sensor sample.
 * @param acc_g     Acceleration magnitude of the sample in g.
 * @param gyro_dps  Rotation rate magnitude of the sample in dps.
 */
static void learn_reference (const imu_sample_t * p_sample, float acc_g,
                             float gyro_dps)
{
    float gain = REF_GAIN_SLOW;

    if (is_still(acc_g, gyro_dps))
    {
        if (g_ref_fast_left > 0u)
        {
            gain = REF_GAIN_FAST;
            g_ref_fast_left--;
        }

        average_into(g_ref, p_sample, gain);
    }
}

/* end of fall_detect.c */

