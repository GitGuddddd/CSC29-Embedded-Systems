#include "fall_detect.h"
#include <math.h>

// ---- Tunable thresholds (starting points, tune from your logs) ----
#define FF_THRESH_G       0.5f    // |a| below this = free fall
#define FF_MIN_MS         50      // free fall must last this long
#define IMPACT_THRESH_G   2.5f    // |a| above this = impact
#define IMPACT_WINDOW_MS  1000    // impact must follow free fall within this
#define SETTLE_MS         800     // ignore bouncing after impact
#define STILL_MS          500     // must be still this long to confirm
#define CHECK_TIMEOUT_MS  3000    // give up if it has not come to rest by now
#define STILL_ACC_TOL_G   0.15f   // |a| within 1 +- this = still
#define STILL_GYR_DPS     30.0f   // |gyro| below this = still
#define TILT_DEG          45.0f   // head orientation change vs normal

typedef enum { IDLE, FREEFALL, WAIT_IMPACT, SETTLING, CHECK_STILL } state_t;
static const char *names[] = { "IDLE", "FREEFALL", "WAIT_IMPACT", "SETTLING", "CHECK_STILL" };

static state_t st = IDLE;
static uint32_t t0;
static uint32_t t_check;            // when CHECK_STILL started
static float ref[3] = { 0, 0, 1 };  // normal head gravity direction (slow average)
static float cur[3];                // gravity direction after the fall

static float mag3(float x, float y, float z) { return sqrtf(x * x + y * y + z * z); }

static float angle_deg(const float a[3], const float b[3]) {
    float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    float m = mag3(a[0], a[1], a[2]) * mag3(b[0], b[1], b[2]);
    if (m < 1e-6f) return 0;
    float c = d / m;
    if (c > 1) c = 1;
    if (c < -1) c = -1;
    return acosf(c) * 57.2958f;
}

const char *fall_detect_state_name(void) { return names[st]; }

void fall_detect_reset(void) { st = IDLE; }

bool fall_detect_update(const imu_sample_t *s, uint32_t now_ms) {
    float a = mag3(s->ax, s->ay, s->az);
    float g = mag3(s->gx, s->gy, s->gz);

    switch (st) {
    case IDLE:
        ref[0] += 0.01f * (s->ax - ref[0]);
        ref[1] += 0.01f * (s->ay - ref[1]);
        ref[2] += 0.01f * (s->az - ref[2]);
        // A fall must start with free fall. A hard hit alone (setting the head
        // down, bumping it, shaking) is not enough: it caused false alarms.
        if (a < FF_THRESH_G) { st = FREEFALL; t0 = now_ms; }
        break;

    case FREEFALL:
        if (a >= FF_THRESH_G) {
            if (now_ms - t0 < FF_MIN_MS)  st = IDLE;
            else if (a > IMPACT_THRESH_G) { st = SETTLING;    t0 = now_ms; }
            else                          { st = WAIT_IMPACT; t0 = now_ms; }
        }
        break;

    case WAIT_IMPACT:
        if (a > IMPACT_THRESH_G)                   { st = SETTLING; t0 = now_ms; }
        else if (now_ms - t0 > IMPACT_WINDOW_MS)   st = IDLE;
        break;

    case SETTLING:
        if (now_ms - t0 >= SETTLE_MS) {
            st = CHECK_STILL;
            t0 = now_ms;
            t_check = now_ms;
            cur[0] = s->ax; cur[1] = s->ay; cur[2] = s->az;
        }
        break;

    case CHECK_STILL:
        cur[0] += 0.1f * (s->ax - cur[0]);
        cur[1] += 0.1f * (s->ay - cur[1]);
        cur[2] += 0.1f * (s->az - cur[2]);
        if (fabsf(a - 1.0f) > STILL_ACC_TOL_G || g > STILL_GYR_DPS) {
            // Still rocking or bouncing: restart the stillness timer and keep waiting.
            t0 = now_ms;
            if (now_ms - t_check >= CHECK_TIMEOUT_MS) st = IDLE;
        } else if (now_ms - t0 >= STILL_MS) {
            st = IDLE;
            if (angle_deg(ref, cur) > TILT_DEG) return true;
        }
        break;
    }
    return false;
}
