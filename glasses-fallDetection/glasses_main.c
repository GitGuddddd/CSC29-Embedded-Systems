/**
 * @file    glasses_main.c
 * @brief   Main module of the glasses node: head fall detection, IMU fault
 *          handling (FR17), status LED, acknowledge button, trial logging
 *          and sampling jitter measurement.
 *
 * The 100 Hz loop reads the IMU, runs the fall detector and updates the LED
 * and the buzzer. The debug features (trial log, RAM log, jitter log) are
 * switched in glasses_main.h.
 *
 * The status flags and counters are kept by node_status.c, not here.
 */

#include "glasses_main.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#if RAM_LOG
#include <string.h>
#endif

#include "hardware/i2c.h"
#if RAM_LOG
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdio_usb.h"
#endif
#include "pico/stdlib.h"

#include "alert.h"
#include "fall_detect.h"
#include "mpu9250.h"
#include "node_status.h"

#define IMU_I2C               i2c1
#define TIME_HALF_RANGE_MS    0x80000000u
#define LED_FAULT_PERIOD_MS   1000u
#define LED_FAULT_ON_MS       500u
#define LED_FALL_PERIOD_MS    200u
#define LED_FALL_ON_MS        100u
#define BLINK_READY_COUNT     2u
#define BLINK_READY_MS        100u
#define LOG_ENABLED           ((0 != DEBUG_PRINT) || (0 != RAM_LOG))

#if TRIAL_LOG
#define TRIAL_DEBOUNCE_MS     300u
#define ACCEL_LOG_PERIOD_MS   500u
#define MILLI_G_PER_G         1000.0f
#endif

#if JITTER_LOG
#define JITTER_SKIP_PERIODS   2u
#endif

#if RAM_LOG
#define BUTTON_POLL_MS        10u
#define USB_POLL_MS           100u
#define USB_SETTLE_MS         1000u
#define SAVE_OK_BLINKS        3u
#define SAVE_OK_MS            300u
#define SAVE_FAIL_BLINKS      10u
#define SAVE_FAIL_MS          50u

/* Record types of the log. */
#define REC_TRIAL             0u   /* val[0] is the trial number. */
#define REC_ACCEL             1u   /* val[0..2] are x, y, z in milli-g. */
#define REC_TEXT              2u   /* text_idx indexes the name table. */
#define REC_FALL              3u   /* val[0] is the fall count. */
#define LOG_VALUES            3u
#define LOG_NAME_NONE         0xFFu

/* Flash copy of the log. It survives power loss, so no cable is needed in
 * a trial. Layout at the end of flash: [header sector][records]. The header
 * is written last, so a save that was interrupted is never mistaken for a
 * good log. */
#define FLASH_LOG_MAGIC       0x474C4F47u
#define FLASH_LOG_NAME_LEN    24u
#define FLASH_LOG_HDR_BYTES   512u   /* Header rounded up to whole pages. */

/* Cast: the record size is 12 bytes, so it fits in 32 bits. */
#define LOG_REC_BYTES         ((uint32_t)sizeof(log_rec_t))
#define FLASH_LOG_SIZE        (((FLASH_SECTOR_SIZE +                        \
                                 (RAM_LOG_RECORDS * LOG_REC_BYTES) +        \
                                 FLASH_SECTOR_SIZE - 1u) /                  \
                                FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE)

/* Cast: the flash size is a positive constant. */
#define FLASH_LOG_OFFSET      (((uint32_t)PICO_FLASH_SIZE_BYTES) -          \
                               FLASH_LOG_SIZE)
#endif

#if JITTER_LOG
typedef struct
{
    uint32_t count;         /* Samples in this window. */
    uint32_t origin_us;     /* Start of the interval before the first one. */
    uint32_t dt_min_us;
    uint32_t dt_max_us;
    uint32_t work_max_us;   /* Longest time spent working in one period. */
    uint32_t overruns;      /* Periods where the work ended too late. */
    uint32_t read_fails;
} jitter_stats_t;
#endif

#if RAM_LOG
typedef struct
{
    uint32_t t_ms;
    uint8_t  type;
    uint8_t  text_idx;
    int16_t  val[LOG_VALUES];
} log_rec_t;

typedef struct
{
    uint32_t magic;
    uint32_t count;
    uint32_t dropped;
    uint32_t name_count;
    char     names[RAM_LOG_NAMES][FLASH_LOG_NAME_LEN];
} flash_hdr_t;

/* The union gives the header the alignment of its fields and lets it be
 * programmed to flash as plain bytes. */
typedef union
{
    flash_hdr_t hdr;
    uint8_t     bytes[FLASH_LOG_HDR_BYTES];
} flash_hdr_buf_t;
#endif

static uint32_t g_imu_reinit_due_ms = 0u;
static uint32_t g_fall_ms           = 0u;

#if LOG_ENABLED
static const char * gp_last_state = "";
#endif

#if JITTER_LOG
static jitter_stats_t g_jit;
static uint32_t       g_jit_prev_start_us = 0u;
static uint32_t       g_jit_skip          = 0u;
#endif

#if RAM_LOG
static log_rec_t       g_log_buf[RAM_LOG_RECORDS];
static uint32_t        g_log_len        = 0u;
static uint32_t        g_log_dropped    = 0u;
static uint32_t        g_log_name_count = 0u;
static const char *    gp_log_names[RAM_LOG_NAMES];
static flash_hdr_buf_t g_hdr_buf;
static uint32_t        g_dump_down_ms   = 0u;
static bool            gb_is_dump_down  = false;
#endif

#if TRIAL_LOG
static uint32_t g_trial_no     = 0u;
static uint32_t g_trial_btn_ms = 0u;
static uint32_t g_accel_log_ms = 0u;
#endif

static void     jitter_reset (void);
static void     jitter_note_read_fail (void);
static void     jitter_record (uint32_t start_us, uint32_t work_us,
                               bool b_overrun);
static void     log_text (uint32_t now_ms, const char * p_text);
static void     log_fall (uint32_t now_ms, uint32_t count);
static void     pins_init (void);
static void     button_init (uint32_t pin);
static void     i2c_setup (void);
static void     led_blink (uint32_t count, uint32_t on_ms, uint32_t off_ms);
static uint32_t clock_ms (void);
static bool     deadline_passed (uint32_t now_ms, uint32_t due_ms);
static bool     imu_start (void);
static void     imu_enter_fault (uint32_t now_ms);
static void     imu_enter_ok (void);
static bool     imu_service (uint32_t now_ms);
static void     imu_handle_sample (uint32_t now_ms,
                                   const imu_sample_t * p_sample);
static void     imu_handle_read_fail (uint32_t now_ms);
static void     on_fall_detected (uint32_t now_ms);
static void     status_led_update (uint32_t now_ms);
static bool     log_button_service (uint32_t now_ms);
#if DEBUG_PRINT
static const char * cal_result_text (mpu9250_cal_t result);
#endif
#if JITTER_LOG
static void     jitter_print (uint32_t start_us);
#endif
#if TRIAL_LOG
static int16_t  to_milli_g (float accel_g);
static void     log_trial (uint32_t now_ms, uint32_t trial);
static void     log_accel (uint32_t now_ms, const imu_sample_t * p_sample);
static void     trial_log (uint32_t now_ms, const imu_sample_t * p_sample);
#endif
#if RAM_LOG
static void     log_push (uint32_t now_ms, uint8_t type, uint8_t text_idx,
                          int16_t val_a, int16_t val_b, int16_t val_c);
static uint8_t  log_name_index (const char * p_text);
static void     print_records (const log_rec_t * p_recs, uint32_t count,
                               const char * const * pp_names);
static uint32_t round_up (uint32_t value, uint32_t unit);
static bool     flash_log_save (void);
static void     log_dump (void);
#endif

/**
 * @brief  Entry point: starts the IMU, then runs the 100 Hz loop forever.
 * @return Never returns.
 */
int main (void)
{
    absolute_time_t next;
    uint32_t        start_us;
    uint32_t        now_ms;
    bool            b_overrun;

    /* There is no wait for USB: output is dropped if nothing is connected. */
    stdio_init_all();
    pins_init();
    i2c_setup();
    alert_init(BUZZER_PIN);
    node_status_init();

    if (imu_start())
    {
        imu_enter_ok();
        led_blink(BLINK_READY_COUNT, BLINK_READY_MS, BLINK_READY_MS);
    }
    else
    {
        imu_enter_fault(clock_ms());
    }

    jitter_reset();
    next = get_absolute_time();

    for (;;)
    {
        next     = delayed_by_ms(next, SAMPLE_PERIOD_MS);
        start_us = time_us_32();
        now_ms   = clock_ms();

        if (imu_service(now_ms))
        {
            /* The sensor start-up blocks, so restart the schedule. */
            next = get_absolute_time();
            jitter_reset();
        }

        status_led_update(now_ms);
        alert_update(now_ms);

        if (log_button_service(now_ms))
        {
            /* A dump or save takes seconds, so restart the schedule. */
            next = get_absolute_time();
            jitter_reset();
        }
        else
        {
            b_overrun = (absolute_time_diff_us(get_absolute_time(), next) < 0);
            jitter_record(start_us, time_us_32() - start_us, b_overrun);
            sleep_until(next);
        }
    }

    return 0;
}

/**
 * @brief Clears the jitter statistics and starts a new window.
 */
static void jitter_reset (void)
{
#if JITTER_LOG
    g_jit.count       = 0u;
    g_jit.dt_min_us   = UINT32_MAX;
    g_jit.dt_max_us   = 0u;
    g_jit.work_max_us = 0u;
    g_jit.overruns    = 0u;
    g_jit.read_fails  = 0u;

    /* The print or the sensor start-up itself distorts the next periods. */
    g_jit_skip        = JITTER_SKIP_PERIODS;
#else
    /* Nothing to do: jitter logging is switched off. */
#endif
}

/**
 * @brief Counts one failed sensor read in the current jitter window.
 */
static void jitter_note_read_fail (void)
{
#if JITTER_LOG
    g_jit.read_fails++;
#else
    /* Nothing to do: jitter logging is switched off. */
#endif
}

/**
 * @brief Records the timing of one loop pass.
 * @param start_us   Time at the start of the pass.
 * @param work_us    Time spent working in the pass.
 * @param b_overrun  true if the work ended after the deadline.
 */
static void jitter_record (uint32_t start_us, uint32_t work_us,
                           bool b_overrun)
{
#if JITTER_LOG
    uint32_t dt_us = start_us - g_jit_prev_start_us;

    if (g_jit_skip > 0u)
    {
        g_jit_skip--;
        g_jit_prev_start_us = start_us;
    }
    else
    {
        if (0u == g_jit.count)
        {
            g_jit.origin_us = g_jit_prev_start_us;
        }
        g_jit_prev_start_us = start_us;

        if (dt_us < g_jit.dt_min_us)
        {
            g_jit.dt_min_us = dt_us;
        }
        if (dt_us > g_jit.dt_max_us)
        {
            g_jit.dt_max_us = dt_us;
        }
        if (work_us > g_jit.work_max_us)
        {
            g_jit.work_max_us = work_us;
        }
        if (b_overrun)
        {
            g_jit.overruns++;
        }
        g_jit.count++;

        if (g_jit.count >= JITTER_WINDOW)
        {
            jitter_print(start_us);
            jitter_reset();
        }
    }
#else
    /* Casts: the arguments are not used while jitter logging is off. */
    (void)start_us;
    (void)work_us;
    (void)b_overrun;
#endif
}

#if JITTER_LOG
/**
 * @brief Prints the summary line of the current jitter window.
 * @param start_us  Time at the start of the last pass of the window.
 */
static void jitter_print (uint32_t start_us)
{
    uint32_t mean_us = (start_us - g_jit.origin_us) / g_jit.count;
    uint32_t falls   = node_status_fall_count();

    printf("JITTER n=%" PRIu32 " dt_us min=%" PRIu32 " mean=%" PRIu32
           " max=%" PRIu32 " work_max_us=%" PRIu32 " overruns=%" PRIu32
           " read_fails=%" PRIu32 " falls=%" PRIu32 "\n",
           g_jit.count, g_jit.dt_min_us, mean_us, g_jit.dt_max_us,
           g_jit.work_max_us, g_jit.overruns, g_jit.read_fails, falls);
}
#endif

/**
 * @brief Logs a state or event name.
 * @param now_ms  Milliseconds since boot.
 * @param p_text  Name to log. It must be a string literal, because only the
 *                pointer is stored.
 */
static void log_text (uint32_t now_ms, const char * p_text)
{
#if RAM_LOG
    log_push(now_ms, REC_TEXT, log_name_index(p_text), 0, 0, 0);
#elif DEBUG_PRINT
    printf("[%" PRIu32 " ms] %s\n", now_ms, p_text);
#else
    /* Casts: the arguments are not used while logging is off. */
    (void)now_ms;
    (void)p_text;
#endif
}

/**
 * @brief Logs a confirmed fall.
 * @param now_ms  Milliseconds since boot.
 * @param count   Fall count after this fall, at most 15.
 */
static void log_fall (uint32_t now_ms, uint32_t count)
{
#if RAM_LOG
    /* Cast: the fall count is at most 15, so it fits in 16 bits. */
    log_push(now_ms, REC_FALL, 0u, (int16_t)count, 0, 0);
#elif DEBUG_PRINT
    printf("[%" PRIu32 " ms] *** FALL DETECTED *** (#%" PRIu32 ")\n",
           now_ms, count);
#else
    /* Casts: the arguments are not used while logging is off. */
    (void)now_ms;
    (void)count;
#endif
}

/**
 * @brief Sets the pins of the LED, the buttons and the IMU bus.
 */
static void pins_init (void)
{
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    button_init(ACK_BUTTON_PIN);
#if TRIAL_LOG
    button_init(TRIAL_BUTTON_PIN);
#endif
#if RAM_LOG
    button_init(DUMP_BUTTON_PIN);
#endif
}

/**
 * @brief Sets a pin as an input with a pull-up, for an active-low button.
 * @param pin  GPIO number.
 */
static void button_init (uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

/**
 * @brief Starts the I2C bus of the IMU.
 */
static void i2c_setup (void)
{
    i2c_init(IMU_I2C, IMU_I2C_BAUD_HZ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
}

/**
 * @brief Blinks the status LED. It blocks until the blinks are done.
 * @param count   Number of blinks.
 * @param on_ms   Time the LED is on for each blink.
 * @param off_ms  Time the LED is off after each blink.
 */
static void led_blink (uint32_t count, uint32_t on_ms, uint32_t off_ms)
{
    uint32_t idx;

    for (idx = 0u; idx < count; idx++)
    {
        gpio_put(STATUS_LED_PIN, true);
        sleep_ms(on_ms);
        gpio_put(STATUS_LED_PIN, false);
        sleep_ms(off_ms);
    }
}

/**
 * @brief  Reads the millisecond clock.
 * @return Milliseconds since boot. It wraps after about 49 days.
 */
static uint32_t clock_ms (void)
{
    return to_ms_since_boot(get_absolute_time());
}

/**
 * @brief  Tells whether a time has been reached, also across the wrap.
 * @param  now_ms  Current time in milliseconds.
 * @param  due_ms  Time to compare with.
 * @return true if now_ms is at or after due_ms, for times less than half
 *         the clock range apart.
 */
static bool deadline_passed (uint32_t now_ms, uint32_t due_ms)
{
    return ((now_ms - due_ms) < TIME_HALF_RANGE_MS);
}

#if DEBUG_PRINT
/**
 * @brief  Gives a name to a gyro calibration result, for the log.
 * @param  result  Calibration result.
 * @return Pointer to a constant string.
 */
static const char * cal_result_text (mpu9250_cal_t result)
{
    const char * p_text;

    switch (result)
    {
        case MPU9250_CAL_OK:
            p_text = "ok";
        break;

        case MPU9250_CAL_MOVING:
            p_text = "moving";
        break;

        case MPU9250_CAL_BUS_ERROR:
        default:
            p_text = "bus error";
        break;
    }

    return p_text;
}
#endif

/**
 * @brief  Verifies WHO_AM_I, configures the sensor and, on the first
 *         success only, zeroes the gyro.
 *
 * It blocks for about 150 ms, plus about 1 s per calibration try the first
 * time.
 *
 * NOTE: The glasses must be still for about 1 s at first power-on. Later
 * re-inits keep the old bias, because the wearer may be moving.
 *
 * @return true if the sensor is ready.
 */
static bool imu_start (void)
{
    bool          b_ok      = mpu9250_init(IMU_I2C, IMU_I2C_ADDR);
    uint32_t      attempt   = 0u;
    float         range_dps = 0.0f;
    mpu9250_cal_t result;

    while (b_ok && (!node_status_is_cal_done()) &&
           (attempt < GYRO_CAL_TRIES))
    {
        result = mpu9250_calibrate_gyro(GYRO_CAL_SAMPLES, &range_dps);
        attempt++;

#if DEBUG_PRINT
        printf("[%" PRIu32 " ms] GYRO_CAL try %" PRIu32 ": %s "
               "(range %.1f dps)\n", clock_ms(), attempt,
               cal_result_text(result), range_dps);
#endif

        if (MPU9250_CAL_OK == result)
        {
            node_status_set_cal_done();
        }
        else if (MPU9250_CAL_BUS_ERROR == result)
        {
            /* Flaky bus: stay in SENSOR_FAULT and retry later. */
            b_ok = false;
        }
        else
        {
            /* The sensor was moving: try again. */
        }
    }

    if (b_ok && (!node_status_is_cal_done()))
    {
        /* Never still: carry on with zero bias. The stillness check allows
         * 30 dps, so a typical bias of 1 to 2 dps is harmless. Power-cycle
         * the board while it is still to calibrate again. */
        node_status_set_cal_done();
#if DEBUG_PRINT
        printf("[%" PRIu32 " ms] GYRO_CAL skipped, zero bias\n", clock_ms());
#endif
    }

    return b_ok;
}

/**
 * @brief Enters the SENSOR_FAULT state.
 * @param now_ms  Milliseconds since boot.
 */
static void imu_enter_fault (uint32_t now_ms)
{
    node_status_set_sensor_fault(true);
    g_imu_reinit_due_ms = now_ms + IMU_REINIT_MS;
    fall_detect_reset();
    alert_set_fault(ALERT_FAULT_IMU, true);
    log_text(now_ms, "SENSOR_FAULT");

    /* TODO: Post SENSOR_FAULT to the BLE task. */
}

/**
 * @brief Leaves the SENSOR_FAULT state.
 */
static void imu_enter_ok (void)
{
    node_status_set_sensor_fault(false);
    node_status_clear_read_fails();
    fall_detect_reset();
    alert_set_fault(ALERT_FAULT_IMU, false);
    log_text(clock_ms(), "SENSOR_OK");

    /* TODO: Post SENSOR_OK to the BLE task. */
}

/**
 * @brief  Runs one pass of the IMU state machine: read a sample, or try to
 *         restart the sensor while it is in the fault state.
 * @param  now_ms  Milliseconds since boot.
 * @return true if the pass tried to restart the sensor, which blocks, so
 *         the caller must restart the loop schedule.
 */
static bool imu_service (uint32_t now_ms)
{
    imu_sample_t sample;
    bool         b_restarted = false;

    if (!node_status_is_sensor_fault())
    {
        if (mpu9250_read(&sample))
        {
            imu_handle_sample(now_ms, &sample);
        }
        else
        {
            imu_handle_read_fail(now_ms);
        }
    }
    else if (deadline_passed(now_ms, g_imu_reinit_due_ms))
    {
        b_restarted = true;
        if (imu_start())
        {
            imu_enter_ok();
        }
        else
        {
            g_imu_reinit_due_ms = clock_ms() + IMU_REINIT_MS;
        }
    }
    else
    {
        /* SENSOR_FAULT: waiting for the next restart attempt. */
    }

    return b_restarted;
}

/**
 * @brief Handles one good sample: trial log, fall detector, state log.
 * @param now_ms     Milliseconds since boot.
 * @param p_sample   The sample.
 */
static void imu_handle_sample (uint32_t now_ms,
                               const imu_sample_t * p_sample)
{
#if LOG_ENABLED
    const char * p_state;
#endif

    node_status_clear_read_fails();
#if TRIAL_LOG
    trial_log(now_ms, p_sample);
#endif

    if (fall_detect_update(p_sample, now_ms))
    {
        on_fall_detected(now_ms);
    }

#if LOG_ENABLED
    p_state = fall_detect_state_name();
    if (p_state != gp_last_state)
    {
        gp_last_state = p_state;
        log_text(now_ms, p_state);
    }
#endif
}

/**
 * @brief Handles one failed read: after too many in a row, enter the fault
 *        state.
 * @param now_ms  Milliseconds since boot.
 */
static void imu_handle_read_fail (uint32_t now_ms)
{
    uint8_t fails = node_status_note_read_fail();

    jitter_note_read_fail();
    if (fails >= IMU_FAIL_LIMIT)
    {
        imu_enter_fault(now_ms);
    }
}

/**
 * @brief Handles a confirmed fall. The fall event leaves from here: the cane
 *        sounds the alarm, not the glasses.
 * @param now_ms  Milliseconds since boot.
 */
static void on_fall_detected (uint32_t now_ms)
{
    node_status_fall_detected();
    g_fall_ms = now_ms;
    log_fall(now_ms, node_status_fall_count());

    /* TODO: Send FALL_EVENT to the BLE task. Build the payload with
     * protocol_encode() and agree the format with the cane node. */
}

/**
 * @brief Drives the status LED and reads the acknowledge button.
 *
 * SENSOR_FAULT blinks at 1 Hz and takes precedence. After a fall the LED
 * blinks fast for FALL_LATCH_MS and then stays on until acknowledged.
 *
 * @param now_ms  Milliseconds since boot.
 */
static void status_led_update (uint32_t now_ms)
{
    if (node_status_is_sensor_fault())
    {
        gpio_put(STATUS_LED_PIN,
                 ((now_ms % LED_FAULT_PERIOD_MS) < LED_FAULT_ON_MS));
    }
    else if (node_status_is_fall_latched())
    {
        if ((now_ms - g_fall_ms) >= FALL_LATCH_MS)
        {
            node_status_clear_fall_latched();
        }
        else
        {
            gpio_put(STATUS_LED_PIN,
                     (((now_ms - g_fall_ms) % LED_FALL_PERIOD_MS) <
                      LED_FALL_ON_MS));
        }
    }
    else
    {
        gpio_put(STATUS_LED_PIN, node_status_is_fall_unacked());
    }

    if (node_status_is_fall_unacked() && (0 == gpio_get(ACK_BUTTON_PIN)))
    {
        node_status_fall_ack();
    }
}

#if TRIAL_LOG
/**
 * @brief  Converts an acceleration to milli-g.
 * @param  accel_g  Acceleration in g, within +-8 g.
 * @return The acceleration in milli-g.
 */
static int16_t to_milli_g (float accel_g)
{
    /* Cast: the range is +-8 g, so the result is within +-8000. */
    return (int16_t)(accel_g * MILLI_G_PER_G);
}

/**
 * @brief Logs the start of a trial.
 * @param now_ms  Milliseconds since boot.
 * @param trial   Trial number.
 */
static void log_trial (uint32_t now_ms, uint32_t trial)
{
#if RAM_LOG
    /* Cast: the trial number stays far below 32767 in a session. */
    log_push(now_ms, REC_TRIAL, 0u, (int16_t)trial, 0, 0);
#else
    printf("[%" PRIu32 " ms] ===== TRIAL %" PRIu32 " =====\n", now_ms,
           trial);
#endif
}

/**
 * @brief Logs one acceleration sample in milli-g.
 * @param now_ms    Milliseconds since boot.
 * @param p_sample  The sample.
 */
static void log_accel (uint32_t now_ms, const imu_sample_t * p_sample)
{
#if RAM_LOG
    log_push(now_ms, REC_ACCEL, 0u, to_milli_g(p_sample->accel_x),
             to_milli_g(p_sample->accel_y), to_milli_g(p_sample->accel_z));
#else
    printf("[%" PRIu32 " ms] A,%d,%d,%d\n", now_ms,
           to_milli_g(p_sample->accel_x), to_milli_g(p_sample->accel_y),
           to_milli_g(p_sample->accel_z));
#endif
}

/**
 * @brief Marks the start of a trial and logs the acceleration at 2 Hz.
 *
 * Press the trial button (GP21) while holding still, just before each trial.
 * The marker splits the log into trials. The A,ax,ay,az lines give the head
 * orientation over time.
 *
 * @param now_ms    Milliseconds since boot.
 * @param p_sample  Latest sample.
 */
static void trial_log (uint32_t now_ms, const imu_sample_t * p_sample)
{
    if ((0 == gpio_get(TRIAL_BUTTON_PIN)) &&
        ((now_ms - g_trial_btn_ms) >= TRIAL_DEBOUNCE_MS))
    {
        g_trial_btn_ms = now_ms;
        g_trial_no++;
        log_trial(now_ms, g_trial_no);
    }

    if ((now_ms - g_accel_log_ms) >= ACCEL_LOG_PERIOD_MS)
    {
        g_accel_log_ms = now_ms;
        log_accel(now_ms, p_sample);
    }
}
#endif

#if RAM_LOG
/**
 * @brief Adds one record to the RAM log. When the log is full, the newest
 *        records are lost and counted.
 * @param now_ms    Milliseconds since boot.
 * @param type      Record type, one of the REC_ values.
 * @param text_idx  Index into the name table, for text records.
 * @param val_a     First value.
 * @param val_b     Second value.
 * @param val_c     Third value.
 */
static void log_push (uint32_t now_ms, uint8_t type, uint8_t text_idx,
                      int16_t val_a, int16_t val_b, int16_t val_c)
{
    log_rec_t * p_rec;

    if (g_log_len < RAM_LOG_RECORDS)
    {
        p_rec           = &g_log_buf[g_log_len];
        p_rec->t_ms     = now_ms;
        p_rec->type     = type;
        p_rec->text_idx = text_idx;
        p_rec->val[0]   = val_a;
        p_rec->val[1]   = val_b;
        p_rec->val[2]   = val_c;
        g_log_len++;
    }
    else
    {
        g_log_dropped++;
    }
}

/**
 * @brief  Finds or adds a text in the name table. The texts are string
 *         literals, so the pointer is remembered instead of copying.
 * @param  p_text  String literal.
 * @return Index of the text, or LOG_NAME_NONE if the table is full.
 */
static uint8_t log_name_index (const char * p_text)
{
    uint8_t  index   = LOG_NAME_NONE;
    bool     b_found = false;
    uint32_t idx;

    for (idx = 0u; (!b_found) && (idx < g_log_name_count); idx++)
    {
        if (gp_log_names[idx] == p_text)
        {
            /* Cast: idx is below RAM_LOG_NAMES (16), so it fits. */
            index   = (uint8_t)idx;
            b_found = true;
        }
    }

    if ((!b_found) && (g_log_name_count < RAM_LOG_NAMES))
    {
        gp_log_names[g_log_name_count] = p_text;

        /* Cast: the count is below RAM_LOG_NAMES (16), so it fits. */
        index = (uint8_t)g_log_name_count;
        g_log_name_count++;
    }

    return index;
}

/**
 * @brief Prints records in the same format the live log used.
 * @param p_recs    Records to print.
 * @param count     Number of records.
 * @param pp_names  Name table for the text records.
 */
static void print_records (const log_rec_t * p_recs, uint32_t count,
                           const char * const * pp_names)
{
    const log_rec_t * p_rec;
    const char *      p_text;
    uint32_t          idx;

    for (idx = 0u; idx < count; idx++)
    {
        p_rec = &p_recs[idx];

        switch (p_rec->type)
        {
            case REC_TRIAL:
                printf("[%" PRIu32 " ms] ===== TRIAL %d =====\n",
                       p_rec->t_ms, p_rec->val[0]);
            break;

            case REC_ACCEL:
                printf("[%" PRIu32 " ms] A,%d,%d,%d\n", p_rec->t_ms,
                       p_rec->val[0], p_rec->val[1], p_rec->val[2]);
            break;

            case REC_FALL:
                printf("[%" PRIu32 " ms] *** FALL DETECTED *** (#%d)\n",
                       p_rec->t_ms, p_rec->val[0]);
            break;

            case REC_TEXT:
            default:
                p_text = "?";
                if (p_rec->text_idx < RAM_LOG_NAMES)
                {
                    p_text = pp_names[p_rec->text_idx];
                }
                printf("[%" PRIu32 " ms] %s\n", p_rec->t_ms, p_text);
            break;
        }
    }
}

/**
 * @brief  Rounds a value up to a multiple of a unit.
 * @param  value  Value to round.
 * @param  unit   Unit, not zero.
 * @return The smallest multiple of unit that is not below value.
 */
static uint32_t round_up (uint32_t value, uint32_t unit)
{
    return ((value + unit - 1u) / unit) * unit;
}

/**
 * @brief  Copies the RAM log to flash.
 *
 * WARNING: Interrupts are off for about a second. Call it only with the
 * cable unplugged and never in the middle of a trial.
 *
 * @return false if there is nothing to save or the read-back does not
 *         match.
 */
static bool flash_log_save (void)
{
    bool            b_ok = false;
    uint32_t        rec_bytes;
    uint32_t        prog_bytes;
    uint32_t        erase_bytes;
    uint32_t        irq_state;
    uint32_t        idx;
    const uint8_t * p_saved;

    /* An empty log is never saved, so a good flash log is not wiped. */
    if (g_log_len > 0u)
    {
        rec_bytes   = g_log_len * LOG_REC_BYTES;
        prog_bytes  = round_up(rec_bytes, FLASH_PAGE_SIZE);
        erase_bytes = FLASH_SECTOR_SIZE +
                      round_up(prog_bytes, FLASH_SECTOR_SIZE);

        memset(g_hdr_buf.bytes, 0, FLASH_LOG_HDR_BYTES);
        g_hdr_buf.hdr.magic      = FLASH_LOG_MAGIC;
        g_hdr_buf.hdr.count      = g_log_len;
        g_hdr_buf.hdr.dropped    = g_log_dropped;
        g_hdr_buf.hdr.name_count = g_log_name_count;
        for (idx = 0u; idx < g_log_name_count; idx++)
        {
            strncpy(g_hdr_buf.hdr.names[idx], gp_log_names[idx],
                    FLASH_LOG_NAME_LEN - 1u);
        }

        irq_state = save_and_disable_interrupts();
        flash_range_erase(FLASH_LOG_OFFSET, erase_bytes);

        /* Cast: the records are programmed as raw bytes. */
        flash_range_program(FLASH_LOG_OFFSET + FLASH_SECTOR_SIZE,
                            (const uint8_t *)g_log_buf, prog_bytes);
        flash_range_program(FLASH_LOG_OFFSET, g_hdr_buf.bytes,
                            FLASH_LOG_HDR_BYTES);
        restore_interrupts(irq_state);

        /* Cast: the flash is memory mapped, so the saved bytes can be read
         * back directly. */
        p_saved = (const uint8_t *)(XIP_BASE + FLASH_LOG_OFFSET +
                                    FLASH_SECTOR_SIZE);
        b_ok    = (0 == memcmp(p_saved, g_log_buf, rec_bytes));
    }

    return b_ok;
}

/**
 * @brief Prints the log saved in flash (if any), then the RAM log. Both are
 *        kept, so the dump can be repeated if it was missed.
 */
static void log_dump (void)
{
    const flash_hdr_t * p_fh;
    const log_rec_t *   p_recs;
    const char *        p_names[RAM_LOG_NAMES];
    uint32_t            idx;

    while (!stdio_usb_connected())
    {
        sleep_ms(USB_POLL_MS);
    }
    sleep_ms(USB_SETTLE_MS);   /* Let the terminal finish opening. */

    /* Casts: the flash is memory mapped. The header is at a sector boundary
     * and the records start one sector later, so both are aligned. */
    p_fh   = (const flash_hdr_t *)(XIP_BASE + FLASH_LOG_OFFSET);
    p_recs = (const log_rec_t *)(XIP_BASE + FLASH_LOG_OFFSET +
                                 FLASH_SECTOR_SIZE);

    if ((FLASH_LOG_MAGIC == p_fh->magic) &&
        (p_fh->count <= RAM_LOG_RECORDS))
    {
        for (idx = 0u; idx < RAM_LOG_NAMES; idx++)
        {
            p_names[idx] = p_fh->names[idx];
        }
        printf("FLASHLOG_BEGIN records=%" PRIu32 " dropped=%" PRIu32 "\n",
               p_fh->count, p_fh->dropped);
        print_records(p_recs, p_fh->count, p_names);
        printf("FLASHLOG_END\n");
    }
    else
    {
        printf("FLASHLOG_NONE\n");
    }

    printf("RAMLOG_BEGIN records=%" PRIu32 " dropped=%" PRIu32 "\n",
           g_log_len, g_log_dropped);
    print_records(g_log_buf, g_log_len, gp_log_names);
    printf("RAMLOG_END\n");
}
#endif

/**
 * @brief  Handles the dump button (GP22). Holding it for DUMP_HOLD_MS
 *         dumps the log over USB, or saves it to flash when no cable is
 *         connected. 3 slow blinks mean saved, 10 fast blinks mean failed.
 * @param  now_ms  Milliseconds since boot.
 * @return true if a dump or save was done, which takes seconds, so the
 *         caller must restart the loop schedule.
 */
static bool log_button_service (uint32_t now_ms)
{
    bool b_done = false;

#if RAM_LOG
    if (0 == gpio_get(DUMP_BUTTON_PIN))
    {
        if (!gb_is_dump_down)
        {
            gb_is_dump_down = true;
            g_dump_down_ms  = now_ms;
        }
    }
    else
    {
        gb_is_dump_down = false;
    }

    if (gb_is_dump_down && ((now_ms - g_dump_down_ms) >= DUMP_HOLD_MS))
    {
        gb_is_dump_down = false;
        if (stdio_usb_connected())
        {
            log_dump();
        }
        else if (flash_log_save())
        {
            led_blink(SAVE_OK_BLINKS, SAVE_OK_MS, SAVE_OK_MS);
        }
        else
        {
            led_blink(SAVE_FAIL_BLINKS, SAVE_FAIL_MS, SAVE_FAIL_MS);
        }

        while (0 == gpio_get(DUMP_BUTTON_PIN))
        {
            sleep_ms(BUTTON_POLL_MS);
        }
        fall_detect_reset();
        b_done = true;
    }
#else
    /* Cast: the argument is not used while the RAM log is off. */
    (void)now_ms;
#endif

    return b_done;
}

/* end of glasses_main.c */

