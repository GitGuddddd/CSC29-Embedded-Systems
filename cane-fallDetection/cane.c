/*
 * cane.c - existing ToF repurposed for downward ToF/IMU projection
 * INF2004 Smart Walking Aid - Cane subsystem
 *
 * Combines:
 *    - DFR0030 capacitive touch sensor (grip / "is it being held" detection)
 *    - GY-9250 IMU (MPU6500/9250/9255)       (fall detection + gravity orientation)
 *    - Existing VL53L1X aimed downward      (IMU-projected ground distance)
 * to drive:
 *    - DFR0440 vibration motor               (haptic feedback)
 *    - Piezo buzzer                          (audible fall alarm)
 *
 * Hardware wiring (Dual I2C Bus Configuration):
 *    I2C0        GP4 = SDA, GP5 = SCL   -> IMU only (0x68)
 *    I2C1        GP6 = SDA, GP7 = SCL   -> Existing ToF, aimed downward (0x29)
 *    GP14        DFR0030 touch sensor    -> digital input, HIGH = touched/held
 *    GP15        DFR0440 vibration motor -> PWM output
 *    GP18        Buzzer                  -> PWM output (change BUZZER_PIN if wired elsewhere)
 *
 * Power: IMU, ToF, and touch sensor run off Pico's 3V3 rail.
 * Vibration motor gets 5V power tap, driven via GP15 PWM signal.
 *
 * Cooperative loop with bounded I2C calls; not an RTOS timing guarantee.
 * IMU is sampled once per iteration for both fall detection and projection.
 * While a USB serial monitor is attached, a live status line is printed.
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "vl53l1x/vl53l1x.h"
#include "step_ground/step_ground.h"
#include "step_ground/ground_pose.h"

// ---------------- Pin & bus configuration ----------------
#define IMU_I2C_PORT            i2c0
#define IMU_SDA_PIN             4
#define IMU_SCL_PIN             5

#define TOF_I2C_PORT            i2c1
#define TOF_SDA_PIN             6
#define TOF_SCL_PIN             7

#define TOUCH_PIN               14
#define VIBRATION_PIN           15
#define BUZZER_PIN              18      // Maker Pi Pico onboard buzzer = GP18. Do NOT use GP14 (shares a PWM slice with GP15).

#define I2C_TIMEOUT_US          5000

#define MPU9250_ADDR             0x68
#define MPU9250_REG_ACCEL_CONFIG 0x1C
#define MPU9250_REG_GYRO_CONFIG  0x1B
#define MPU9250_REG_CONFIG       0x1A
#define MPU9250_REG_SMPLRT_DIV   0x19
#define MPU9250_REG_ACCEL_XOUT_H 0x3B
#define MPU9250_REG_PWR_MGMT_1   0x6B
#define MPU9250_REG_WHO_AM_I     0x75
#define MPU9250_ACCEL_FS_8G      0x10    // ACCEL_FS_SEL = 2 -> +/-8g
#define MPU9250_LSB_PER_G_8G     4096.0f

// ---------------- Timing ----------------
#define LOOP_TICK_MS            5       // IMU impact spikes are short; sample fast
#define SENSOR_RETRY_MS         2000    // retry interval for a sensor that failed to start

// ---------------- Fall detection thresholds ----------------
#define FREEFALL_THRESHOLD_G    0.35f   // measured |a| below this = free-fall
#define IMPACT_THRESHOLD_G      2.5f    // measured |a| above this = impact (needs > +/-2g range)
#define FALL_WINDOW_MS          800     // impact must follow free-fall within this window
#define RELEASE_GRACE_MS        1000    // how long after release we still watch for free-fall

// ---------------- Ground detection ----------------
#define TOF_POLL_MS               100   // ~10 Hz polling of the ToF sensor

// ---------------- Serial status output ----------------
#define STATUS_PRINT_MS           200   // live status line rate while a serial monitor is attached

// ---------------- Vibration ----------------
#define VIB_PWM_FREQ_HZ           1000
#define VIB_PWM_WRAP              999

// ---------------- Buzzer (fall alarm) ----------------
#define BUZZER_PWM_WRAP           999
// Loudest setting: 50% duty is the maximum-volume square wave for a passive buzzer
// (100% would be a constant DC level = silent). For an ACTIVE buzzer use 100 instead.
#define BUZZER_DUTY_PERCENT       50
#define BUZZER_ALARM_MAX_MS       30000 // stop sounding after this long even if the cane isn't picked up

typedef enum {
    VIB_NONE,
    VIB_STEP_UP,        // two short pulses
    VIB_STEP_DOWN,      // three short pulses
    VIB_CAUTION         // one long pulse: unknown/fault
} vibration_pattern_t;

typedef struct {
    uint8_t  duty_percent;
    uint16_t duration_ms;
} vib_step_t;

static const vib_step_t pattern_up[] = {{70,150},{0,100},{70,150},{0,1000}};
static const vib_step_t pattern_down[] = {{70,150},{0,100},{70,150},{0,100},{70,150},{0,750}};
static const vib_step_t pattern_caution[] = {{70,600},{0,800}};
_Static_assert(((VIBRATION_PIN >> 1) & 7) != ((BUZZER_PIN >> 1) & 7), "PWM slice conflict");
_Static_assert(TOF_SDA_PIN != IMU_SDA_PIN && TOF_SCL_PIN != IMU_SCL_PIN, "ToF and IMU use separate bus pins");

// Fall alarm: fast two-tone siren with only tiny gaps, looped until stopped.
// freq_hz = 0 means silent. Frequencies are ignored by an active buzzer.
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
} buzzer_step_t;

static const buzzer_step_t pattern_fall_alarm[] = {
    {3000, 120}, {0, 30}, {2000, 120}, {0, 30},
};

// ---------------- State ----------------
static bool imu_ok = false;
static bool ground_tof_ok = false;
static ground_pose_t pose;
static sg_detector_t ground;
static gp_vec3 accel_g, gyro_dps;
static bool imu_sample_ok = false, projected_ok = false;
static uint32_t projection_sample_ms;
static uint16_t ground_raw_mm = VL53L1X_NO_TARGET, vertical_mm;
static float accel_magnitude;
static vibration_pattern_t active_vibration = VIB_NONE;
static uint8_t imu_who_am_i = 0;

static bool touch_held = false;
static bool touch_held_prev = false;

static bool watching_for_fall = false;
static bool freefall_seen = false;
static absolute_time_t release_time;
static absolute_time_t freefall_time;
static bool fall_detected = false;   // latched until the cane is gripped again

static const vib_step_t *vib_pattern = NULL;
static size_t vib_pattern_len = 0;
static size_t vib_step = 0;
static absolute_time_t vib_step_end;


static bool buzzer_alarm_active = false;
static size_t buzzer_step = 0;
static absolute_time_t buzzer_step_end;
static absolute_time_t buzzer_alarm_start_time;

static inline int64_t ms_since(absolute_time_t t) {
    return absolute_time_diff_us(t, get_absolute_time()) / 1000;
}

// ================= Sensor access =================

static bool touch_sensor_read(void) {
    return gpio_get(TOUCH_PIN); // DFR0030 outputs HIGH while touched
}

static bool mpu9250_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_timeout_us(IMU_I2C_PORT, MPU9250_ADDR, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool mpu9250_read_regs(uint8_t reg, uint8_t *dst, size_t len) {
    return i2c_write_timeout_us(IMU_I2C_PORT, MPU9250_ADDR, &reg, 1, true, I2C_TIMEOUT_US) == 1 &&
           i2c_read_timeout_us(IMU_I2C_PORT, MPU9250_ADDR, dst, len, false, I2C_TIMEOUT_US) == (int)len;
}

static bool mpu9250_init(void) {
    // 0x71 = MPU9250, 0x73 = MPU9255, 0x70 = MPU6500 (common on GY-9250 boards)
    if (!mpu9250_read_regs(MPU9250_REG_WHO_AM_I, &imu_who_am_i, 1)) return false;
    if (imu_who_am_i != 0x70 && imu_who_am_i != 0x71 && imu_who_am_i != 0x73) return false;

    if (!mpu9250_write_reg(MPU9250_REG_PWR_MGMT_1, 0x00)) return false; // wake from sleep
    sleep_ms(10);
    // Default +/-2g range saturates each axis at 2g, which is below the 2.5g
    // impact threshold. Use +/-8g so impacts can actually be measured.
    return mpu9250_write_reg(MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_FS_8G) &&
           mpu9250_write_reg(MPU9250_REG_GYRO_CONFIG, 0x08) && // +/-500 dps, 65.5 LSB/dps
           mpu9250_write_reg(MPU9250_REG_CONFIG, 0x03) &&      // gyro DLPF 41 Hz
           mpu9250_write_reg(MPU9250_REG_SMPLRT_DIV, 4);      // 1 kHz / 5 = 200 Hz
}

// One 14-byte burst shares a timestamp for accelerometer and gyro.
static int16_t signed_be(const uint8_t *b) { return (int16_t)(((uint16_t)b[0] << 8) | b[1]); }
static bool mpu9250_read_sample(void) {
    uint8_t raw[14];
    if (!mpu9250_read_regs(MPU9250_REG_ACCEL_XOUT_H, raw, sizeof raw)) return false;
    accel_g = (gp_vec3){signed_be(raw)/4096.0f, signed_be(raw+2)/4096.0f, signed_be(raw+4)/4096.0f};
    // raw[6..7] are temperature, gyro begins at register 0x43.
    gyro_dps = (gp_vec3){signed_be(raw+8)/65.5f, signed_be(raw+10)/65.5f, signed_be(raw+12)/65.5f};
    accel_magnitude = sqrtf(accel_g.x*accel_g.x + accel_g.y*accel_g.y + accel_g.z*accel_g.z);
    return true;
}

// ================= Vibration motor control =================

static void vibration_init(void) {
    gpio_set_function(VIBRATION_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(VIBRATION_PIN);
    // Without a divider the PWM runs at clk_sys / 1000 (~125-150 kHz),
    // far too fast for a motor driver. Divide down to VIB_PWM_FREQ_HZ.
    float div = (float)clock_get_hz(clk_sys) / (VIB_PWM_FREQ_HZ * (VIB_PWM_WRAP + 1));
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, VIB_PWM_WRAP);
    pwm_set_gpio_level(VIBRATION_PIN, 0);
    pwm_set_enabled(slice, true);
}

static void vibration_set(uint8_t duty_percent) {
    pwm_set_gpio_level(VIBRATION_PIN, (VIB_PWM_WRAP * duty_percent) / 100);
}

// One motor owner; state changes cancel the old pattern immediately.
static void vibration_select(vibration_pattern_t pattern) {
    if (active_vibration == pattern) return;
    active_vibration = pattern;
    vib_pattern = NULL;
    vibration_set(0);
    switch (pattern) {
        case VIB_STEP_UP: vib_pattern=pattern_up; vib_pattern_len=4; break;
        case VIB_STEP_DOWN: vib_pattern=pattern_down; vib_pattern_len=6; break;
        case VIB_CAUTION: vib_pattern=pattern_caution; vib_pattern_len=2; break;
        default: return;
    }
    vib_step=0;
    vibration_set(vib_pattern[0].duty_percent);
    vib_step_end=make_timeout_time_ms(vib_pattern[0].duration_ms);
}

// Advance the current pattern; call every loop tick.
static void vibration_update(void) {
    if (vib_pattern == NULL || !time_reached(vib_step_end)) return;

    vib_step++;
    if (vib_step >= vib_pattern_len) vib_step = 0; // repeat only while selected
    vibration_set(vib_pattern[vib_step].duty_percent);
    vib_step_end = make_timeout_time_ms(vib_pattern[vib_step].duration_ms);
}

// ================= Buzzer control =================

static void buzzer_init(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_wrap(slice, BUZZER_PWM_WRAP);
    pwm_set_gpio_level(BUZZER_PIN, 0);
    pwm_set_enabled(slice, true);
}

// Play a tone at freq_hz, or silence when freq_hz == 0.
static void buzzer_set_tone(uint16_t freq_hz) {
    if (freq_hz == 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        return;
    }
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / ((float)freq_hz * (BUZZER_PWM_WRAP + 1)));
    pwm_set_gpio_level(BUZZER_PIN, (BUZZER_PWM_WRAP * BUZZER_DUTY_PERCENT) / 100);
}

static void buzzer_alarm_start(void) {
    buzzer_alarm_active = true;
    buzzer_step = 0;
    buzzer_alarm_start_time = get_absolute_time();
    buzzer_set_tone(pattern_fall_alarm[0].freq_hz);
    buzzer_step_end = make_timeout_time_ms(pattern_fall_alarm[0].duration_ms);
}

static void buzzer_alarm_stop(void) {
    buzzer_alarm_active = false;
    buzzer_set_tone(0);
}

// Advance the alarm pattern (loops until stopped or timed out); call every loop tick.
static void buzzer_update(void) {
    if (!buzzer_alarm_active) return;

    if (ms_since(buzzer_alarm_start_time) >= BUZZER_ALARM_MAX_MS) {
        buzzer_alarm_stop();
        return;
    }
    if (!time_reached(buzzer_step_end)) return;

    buzzer_step = (buzzer_step + 1) % (sizeof(pattern_fall_alarm) / sizeof(pattern_fall_alarm[0]));
    buzzer_set_tone(pattern_fall_alarm[buzzer_step].freq_hz);
    buzzer_step_end = make_timeout_time_ms(pattern_fall_alarm[buzzer_step].duration_ms);
}

// ================= Fall detection =================

static void fall_watch_start(void) {
    watching_for_fall = true;
    freefall_seen = false;
    release_time = get_absolute_time();
}

// Step the fall detector with one IMU sample. Returns true on a fall.
static bool fall_watch_update(void) {
    // Check elapsed windows even if an I2C sample was missed.
    if ((!freefall_seen && ms_since(release_time) > RELEASE_GRACE_MS) ||
        (freefall_seen && ms_since(freefall_time) > FALL_WINDOW_MS)) {
        watching_for_fall = false;
        return false;
    }
    if (!imu_sample_ok) return false;
    float mag = accel_magnitude;

    if (!freefall_seen) {
        if (mag < FREEFALL_THRESHOLD_G) {
            freefall_seen = true;
            freefall_time = get_absolute_time();
        } else if (ms_since(release_time) > RELEASE_GRACE_MS) {
            watching_for_fall = false; // set down deliberately, not a fall
        }
    } else {
        if (mag > IMPACT_THRESHOLD_G) {
            watching_for_fall = false;
            return true;
        }
        if (ms_since(freefall_time) > FALL_WINDOW_MS) {
            watching_for_fall = false; // free-fall with no impact (e.g. caught)
        }
    }
    return false;
}

// ================= Serial status output =================

static uint32_t clock_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static void print_startup_report(void) {
    printf("\n=== Cane with existing ToF and IMU ground projection ===\n");
    printf("IMU=%d id=0x%02X ground_tof=%d. I2C0 GP4/5: IMU; I2C1 GP6/7: ground ToF.\n",
           imu_ok, imu_who_am_i, ground_tof_ok);
    printf("A sensor that failed to start is retried every %d ms.\n", SENSOR_RETRY_MS);
    printf("m: beam PLUMB DOWN and still for mount/gyro calibration; c: calibrate level-ground height.\n");
    printf("Calibration is RAM-only. Keep sensor and IMU rigidly fixed; repeat m then c after reboot.\n");
}

static void print_status(void) {
    uint32_t now=clock_ms();
    printf("held=%d imu=%d a=%.2fg down_raw=%u "
           "mount=%d(%u) pose_ok=%d cos=%.3f vertical=%u projection_ok=%d base=%u ground=%s\n",
           touch_held, imu_sample_ok, accel_magnitude,
           (unsigned)ground_raw_mm, pose.mounted, pose.mount_samples,
           gp_ready(&pose,now), pose.cosine, (unsigned)vertical_mm,
           projected_ok && gp_ready(&pose,now) && (uint32_t)(now-projection_sample_ms)<SG_STALE_MS,
           (unsigned)ground.baseline_mm, sg_state_name(ground.state));
}

static void service_ground(uint32_t now) {
    uint16_t range;

    /*
     * vl53l1x_read_mm() returns:
     *   false -> no new measurement yet OR I2C/read failure
     *   true  -> measurement available
     *
     * If true, range is either:
     *   normal distance in mm
     *   VL53L1X_NO_TARGET when no valid target was detected
     */
    if (!vl53l1x_read_mm(&range)) {
        return;
    }

    projected_ok = false;
    ground_raw_mm = range;

    now = clock_ms();

    if (!gp_ready(&pose, now)) {
        sg_feed(&ground, SG_RANGE_INVALID, 0, now);
    } else if (range != VL53L1X_NO_TARGET &&
               gp_project(&pose, range, now, &vertical_mm)) {

        projected_ok = true;
        projection_sample_ms = now;

        sg_feed(&ground,
                SG_RANGE_VALID,
                vertical_mm,
                now);
    } else if (range == VL53L1X_NO_TARGET) {
        sg_feed(&ground,
                SG_RANGE_NO_RETURN,
                0,
                now);
    } else {
        sg_feed(&ground,
                SG_RANGE_INVALID,
                0,
                now);
    }
}

static vibration_pattern_t desired_haptic(void) {
    if (!touch_held || pose.mounting || ground.state==SG_CALIBRATING) return VIB_NONE;
    // Highest to lowest: confirmed drop, raised ground, caution.
    if (ground.state==SG_STEP_DOWN) return VIB_STEP_DOWN;
    if (ground.state==SG_STEP_UP) return VIB_STEP_UP;
    if (ground.state==SG_NO_GROUND || ground.state==SG_SENSOR_FAULT ||
        (pose.mounted && ground.calibrated && ground.state==SG_UNKNOWN)) return VIB_CAUTION;
    return VIB_NONE;
}

// ================= Sensor start / retry =================

static bool ground_tof_start(void) {
    return vl53l1x_init(TOF_I2C_PORT) &&
           vl53l1x_start_ranging();
}

// Re-attempt startup of any sensor that failed at boot. Called every loop but
// only acts once per SENSOR_RETRY_MS, and never while a fall is being watched
// so a slow init can't make the loop miss the free-fall/impact window.
static void sensors_retry(uint32_t now) {
    static uint32_t last_attempt_ms = 0;

    if (imu_ok && ground_tof_ok) return;
    if (watching_for_fall) return;
    if ((uint32_t)(now - last_attempt_ms) < SENSOR_RETRY_MS) return;
    last_attempt_ms = now;

    if (!imu_ok) {
        imu_ok = mpu9250_init();
        if (imu_ok) printf("IMU started on retry (id=0x%02X). Re-run 'm' to calibrate mount.\n", imu_who_am_i);
    }
    if (!ground_tof_ok) {
        ground_tof_ok = ground_tof_start();
        if (ground_tof_ok) printf("Ground ToF started on retry.\n");
    }
}

// ================= Main =================

int main(void) {
    stdio_init_all();

    // Original I2C0 pins for IMU only (0x68).
    i2c_init(IMU_I2C_PORT, 400 * 1000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    // Original I2C1 pins for the existing ToF (0x29), now aimed at the ground.
    i2c_init(TOF_I2C_PORT, 400 * 1000);
    gpio_set_function(TOF_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA_PIN);
    gpio_pull_up(TOF_SCL_PIN);

    // Init Digital GPIO
    gpio_init(TOUCH_PIN);
    gpio_set_dir(TOUCH_PIN, GPIO_IN);
    gpio_pull_down(TOUCH_PIN);

    // Init Peripherals
    vibration_init();
    buzzer_init();

    imu_ok = mpu9250_init();
    ground_tof_ok = ground_tof_start();   // if this fails, sensors_retry() keeps trying in the main loop
    gp_init(&pose);
    sg_init(&ground,clock_ms());

    absolute_time_t last_status_print = get_absolute_time();
    uint32_t last_ground_poll = clock_ms();
    bool usb_was_connected = false;

    while (true) {
        // ---- Serial monitor: report on (re)connect, then a live status line ----
        bool usb_connected = stdio_usb_connected();
        if (usb_connected && !usb_was_connected) print_startup_report();
        usb_was_connected = usb_connected;

        // ---- Touch / grip monitoring ----
        touch_held = touch_sensor_read();

        if (touch_held_prev && !touch_held) {
            // Just let go -> arm the fall watch
            if (imu_ok) fall_watch_start();
        } else if (!touch_held_prev && touch_held) {
            // Just gripped -> picked back up, so any fall watch/alert is over
            watching_for_fall = false;
            fall_detected = false;
            buzzer_alarm_stop();
        }
        touch_held_prev = touch_held;

        // ---- Retry any sensor that failed to start ----
        sensors_retry(clock_ms());

        // Read IMU once, even while held. Fall detection uses raw acceleration;
        // projection uses the complementary gravity estimate and gyro bias.
        imu_sample_ok = imu_ok && mpu9250_read_sample();
        uint32_t now=clock_ms();
        if (imu_sample_ok) gp_update(&pose,accel_g,gyro_dps,now);
        else gp_invalidate(&pose);

        int command=getchar_timeout_us(0);
        if (command=='m' || command=='M') {
            if (imu_sample_ok) {
                gp_start_mount(&pose); sg_init(&ground,now); projected_ok=false;
                printf("Mount calibration: point the DOWNWARD ToF optical beam PLUMB DOWN and stay still.\n");
            } else printf("Mount calibration refused: IMU unavailable.\n");
        }
        sg_set_armed(&ground,touch_held && pose.mounted,now);
        if (command=='c' || command=='C') {
            if (ground_tof_ok && ground.armed && gp_ready(&pose,now)) {
                sg_start_calibration(&ground,now);
                printf("Ground baseline: hold at normal height over known level ground.\n");
            } else printf("Baseline refused: need down ToF, held grip, completed mount and stable pose.\n");
        }
        if (ground.armed && !gp_ready(&pose,now)) {
            projected_ok=false;
            sg_feed(&ground,SG_RANGE_INVALID,0,now);
        }

        // ---- Fall detection (only after release) ----
        if (watching_for_fall && fall_watch_update()) {
            fall_detected = true;
            printf("FALL DETECTED!\n");
            buzzer_alarm_start();  // sounds until the cane is gripped again (or BUZZER_ALARM_MAX_MS)
            // Note: Vibration motor is intentionally omitted here as
            // the user has let go of the cane. Send Bluetooth alert packet here.
        }

        // The single existing ToF now supplies only the ground classifier.
        now=clock_ms();
        if (ground_tof_ok && (uint32_t)(now-last_ground_poll)>=TOF_POLL_MS) {
            last_ground_poll=now; service_ground(now);
        }
        sg_tick(&ground,clock_ms());
        vibration_select(desired_haptic());
        vibration_update();
        buzzer_update();
        if (usb_connected && ms_since(last_status_print) >= STATUS_PRINT_MS) {
            last_status_print=get_absolute_time(); print_status();
        }
        sleep_ms(LOOP_TICK_MS);
    }
}
