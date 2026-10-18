/*
 * glasses_node.c - Glasses node: Head Fall Detection, IMU fault handling (FR17),
 * status LED / acknowledge button, and sampling-jitter logging.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include "pico/stdio_usb.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"
#include "mpu9250.h"
#include "alert.h"
#include "fall_detect.h"

#define IMU_I2C           i2c1
#define IMU_SDA           2
#define IMU_SCL           3
#define IMU_ADDR          0x68
#define BUZZER_PIN        18      /* onboard buzzer, board mute switch must be ON */
#define STATUS_LED_PIN    10      /* onboard LED */
#define ACK_BUTTON_PIN    20      /* onboard user button, active low */

#define SAMPLE_PERIOD_MS  10u     /* 100 Hz */
#define FALL_LATCH_MS     3000u   /* fast-blink window after a fall */
#define IMU_FAIL_LIMIT    3u      /* consecutive failed reads before SENSOR_FAULT */
#define IMU_REINIT_MS     2000u   /* retry interval in SENSOR_FAULT (match statechart 6.4) */
#define GYRO_CAL_SAMPLES  200
#define GYRO_CAL_TRIES    3u      /* if still "moving" after this many, run with zero bias */

#define TRIAL_LOG         1       /* trial marker button + 2 Hz accel log; set 0 for jitter runs */
#define TRIAL_BUTTON_PIN  21      /* onboard button next to GP20, active low */

#define RAM_LOG           1       /* 1 = keep trial log in RAM instead of printing it */
#define DUMP_BUTTON_PIN   22      /* onboard button, active low: save log to flash / dump it */
#define DUMP_HOLD_MS      1500u   /* hold GP22 this long to save/dump (stops accidental presses) */
#define RAM_LOG_RECORDS   8192u   /* 12 bytes each = 96 KB; about 68 min at 2 Hz */
#define RAM_LOG_NAMES     16u

#define DEBUG_PRINT       1       /* set 0 for the battery build and for jitter runs */
#define JITTER_LOG        1       /* one summary line per JITTER_WINDOW samples */
#define JITTER_WINDOW     1000u   /* 10 s at 100 Hz */

typedef enum
{
    IMU_STATE_OK,
    IMU_STATE_FAULT
} imu_state_t;

static imu_state_t imu_state = IMU_STATE_FAULT;
static uint32_t    imu_fail_count = 0u;
static uint32_t    imu_reinit_due_ms = 0u;
static bool        gyro_cal_done = false;

static bool        fall_latched = false;   /* true during the fast-blink window */
static bool        fall_unacked = false;   /* true until the button is pressed */
static uint32_t    fall_count = 0u;
static uint32_t    fall_ms = 0u;

/* ---------------------------------------------------------------- jitter */

#if JITTER_LOG
typedef struct
{
    uint32_t count;        /* samples in this window */
    uint32_t origin_us;    /* start time of the interval before the first sample */
    uint32_t dt_min_us;
    uint32_t dt_max_us;
    uint32_t work_max_us;  /* longest time spent doing work in one period */
    uint32_t overruns;     /* periods where work finished after the deadline */
    uint32_t read_fails;
} jitter_stats_t;

static jitter_stats_t jit;
static uint32_t jit_prev_start_us = 0u;
static uint32_t jit_skip = 0u;   /* iterations ignored after a report or re-init */
#endif

static void jitter_reset (void)
{
#if JITTER_LOG
    jit.count       = 0u;
    jit.dt_min_us   = UINT32_MAX;
    jit.dt_max_us   = 0u;
    jit.work_max_us = 0u;
    jit.overruns    = 0u;
    jit.read_fails  = 0u;
    jit_skip        = 2u;   /* the printf / re-init itself distorts the next periods */
#endif
}

static void jitter_note_read_fail (void)
{
#if JITTER_LOG
    jit.read_fails++;
#endif
}

static void jitter_record (uint32_t start_us, uint32_t work_us, bool overrun)
{
#if JITTER_LOG
    uint32_t dt = start_us - jit_prev_start_us;

    if (jit_skip > 0u)
    {
        jit_skip--;
        jit_prev_start_us = start_us;
        return;
    }

    if (0u == jit.count)
    {
        jit.origin_us = jit_prev_start_us;
    }
    jit_prev_start_us = start_us;

    if (dt < jit.dt_min_us)
    {
        jit.dt_min_us = dt;
    }
    if (dt > jit.dt_max_us)
    {
        jit.dt_max_us = dt;
    }
    if (work_us > jit.work_max_us)
    {
        jit.work_max_us = work_us;
    }
    if (overrun)
    {
        jit.overruns++;
    }
    jit.count++;

    if (jit.count >= JITTER_WINDOW)
    {
        printf("JITTER n=%u dt_us min=%u mean=%u max=%u work_max_us=%u overruns=%u read_fails=%u falls=%u\n",
               (unsigned int)jit.count,
               (unsigned int)jit.dt_min_us,
               (unsigned int)((start_us - jit.origin_us) / jit.count),
               (unsigned int)jit.dt_max_us,
               (unsigned int)jit.work_max_us,
               (unsigned int)jit.overruns,
               (unsigned int)jit.read_fails,
               (unsigned int)fall_count);
        jitter_reset();
    }
#else
    (void)start_us;
    (void)work_us;
    (void)overrun;
#endif
}

/* -------------------------------------------------------------- log sink */

#define LOG_ENABLED   ((0 != DEBUG_PRINT) || (0 != RAM_LOG))

#if RAM_LOG
typedef enum
{
    REC_TRIAL = 0,   /* v[0] = trial number */
    REC_ACCEL,       /* v[0..2] = ax, ay, az in milli-g */
    REC_TEXT,        /* text_idx = index into log_names */
    REC_FALL         /* v[0] = fall count */
} rec_type_t;

typedef struct
{
    uint32_t t_ms;
    uint8_t  type;
    uint8_t  text_idx;
    int16_t  v[3];
} log_rec_t;

static log_rec_t log_buf[RAM_LOG_RECORDS];
static uint32_t log_len = 0u;
static uint32_t log_dropped = 0u;
static const char *log_names[RAM_LOG_NAMES];
static uint32_t log_name_count = 0u;

static void log_push (uint32_t now_ms, rec_type_t type, uint8_t text_idx,
                      int16_t a, int16_t b, int16_t c)
{
    if (log_len < RAM_LOG_RECORDS)
    {
        log_rec_t *r = &log_buf[log_len];

        r->t_ms = now_ms;
        r->type = (uint8_t)type;
        r->text_idx = text_idx;
        r->v[0] = a;
        r->v[1] = b;
        r->v[2] = c;
        log_len++;
    }
    else
    {
        log_dropped++;   /* buffer full: newest records are lost */
    }
}

/* Texts are string literals, so remember the pointer instead of copying. */
static uint8_t log_name_index (const char *text)
{
    uint32_t i;

    for (i = 0u; i < log_name_count; i++)
    {
        if (log_names[i] == text)
        {
            return (uint8_t)i;
        }
    }
    if (log_name_count < RAM_LOG_NAMES)
    {
        log_names[log_name_count] = text;
        log_name_count++;
        return (uint8_t)(log_name_count - 1u);
    }
    return 0xFFu;
}

static int16_t to_milli_g (float g)
{
    return (int16_t)(g * 1000.0f);   /* range is +-8 g, so it always fits */
}
#endif

static void log_text (uint32_t now_ms, const char *text)
{
#if RAM_LOG
    log_push(now_ms, REC_TEXT, log_name_index(text), 0, 0, 0);
#elif DEBUG_PRINT
    printf("[%u ms] %s\n", (unsigned int)now_ms, text);
#else
    (void)now_ms;
    (void)text;
#endif
}

static void log_fall (uint32_t now_ms, uint32_t count)
{
#if RAM_LOG
    log_push(now_ms, REC_FALL, 0u, (int16_t)count, 0, 0);
#elif DEBUG_PRINT
    printf("[%u ms] *** FALL DETECTED *** (#%u)\n", (unsigned int)now_ms, (unsigned int)count);
#else
    (void)now_ms;
    (void)count;
#endif
}

#if TRIAL_LOG
static void log_trial (uint32_t now_ms, uint32_t trial)
{
#if RAM_LOG
    log_push(now_ms, REC_TRIAL, 0u, (int16_t)trial, 0, 0);
#else
    printf("[%u ms] ===== TRIAL %u =====\n", (unsigned int)now_ms, (unsigned int)trial);
#endif
}

static void log_accel (uint32_t now_ms, const imu_sample_t *s)
{
#if RAM_LOG
    log_push(now_ms, REC_ACCEL, 0u, to_milli_g(s->ax), to_milli_g(s->ay), to_milli_g(s->az));
#else
    printf("[%u ms] A,%.2f,%.2f,%.2f\n", (unsigned int)now_ms,
           (double)s->ax, (double)s->ay, (double)s->az);
#endif
}
#endif

#if RAM_LOG
/* ---- flash copy of the log: survives power loss, so no cable is needed in the trial.
 * Layout at the end of flash: [header sector][records ...]. The header is written last,
 * so a save that was interrupted is never mistaken for a good log. */
#define FLASH_LOG_MAGIC     0x474C4F47u
#define FLASH_LOG_NAME_LEN  24u
#define FLASH_LOG_HDR_BYTES 512u    /* header rounded up to whole flash pages */
#define FLASH_LOG_SIZE      (((FLASH_SECTOR_SIZE + (RAM_LOG_RECORDS * sizeof(log_rec_t)) + \
                              FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE)
#define FLASH_LOG_OFFSET    (PICO_FLASH_SIZE_BYTES - FLASH_LOG_SIZE)

typedef struct
{
    uint32_t magic;
    uint32_t count;
    uint32_t dropped;
    uint32_t name_count;
    char     names[RAM_LOG_NAMES][FLASH_LOG_NAME_LEN];
} flash_hdr_t;

static uint8_t flash_hdr_buf[FLASH_LOG_HDR_BYTES] __attribute__((aligned(4)));

/* Prints records in the same format the live log used. */
static void print_records (const log_rec_t *recs, uint32_t n, const char *const *names)
{
    uint32_t i;

    for (i = 0u; i < n; i++)
    {
        const log_rec_t *r = &recs[i];

        switch ((rec_type_t)r->type)
        {
        case REC_TRIAL:
            printf("[%u ms] ===== TRIAL %d =====\n", (unsigned int)r->t_ms, (int)r->v[0]);
            break;
        case REC_ACCEL:
            printf("[%u ms] A,%.2f,%.2f,%.2f\n", (unsigned int)r->t_ms,
                   (double)r->v[0] / 1000.0, (double)r->v[1] / 1000.0, (double)r->v[2] / 1000.0);
            break;
        case REC_FALL:
            printf("[%u ms] *** FALL DETECTED *** (#%d)\n", (unsigned int)r->t_ms, (int)r->v[0]);
            break;
        case REC_TEXT:
        default:
            printf("[%u ms] %s\n", (unsigned int)r->t_ms,
                   (r->text_idx < RAM_LOG_NAMES) ? names[r->text_idx] : "?");
            break;
        }
    }
}

/* Copies the RAM log to flash. Returns false if there is nothing to save or the
 * read-back does not match. Interrupts are off for about a second: only call this
 * with the cable unplugged, never in the middle of a trial. */
static bool flash_log_save (void)
{
    flash_hdr_t *hdr = (flash_hdr_t *)flash_hdr_buf;
    uint32_t rec_bytes = log_len * (uint32_t)sizeof(log_rec_t);
    uint32_t prog_bytes = ((rec_bytes + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    uint32_t erase_bytes = FLASH_SECTOR_SIZE +
                           (((prog_bytes + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE);
    uint32_t irq;
    uint32_t i;
    const uint8_t *saved = (const uint8_t *)(XIP_BASE + FLASH_LOG_OFFSET + FLASH_SECTOR_SIZE);

    if (0u == log_len)
    {
        return false;   /* never wipe a good flash log with an empty one */
    }

    memset(flash_hdr_buf, 0, sizeof flash_hdr_buf);
    hdr->magic = FLASH_LOG_MAGIC;
    hdr->count = log_len;
    hdr->dropped = log_dropped;
    hdr->name_count = log_name_count;
    for (i = 0u; i < log_name_count; i++)
    {
        strncpy(hdr->names[i], log_names[i], FLASH_LOG_NAME_LEN - 1u);
    }

    irq = save_and_disable_interrupts();
    flash_range_erase(FLASH_LOG_OFFSET, erase_bytes);
    flash_range_program(FLASH_LOG_OFFSET + FLASH_SECTOR_SIZE, (const uint8_t *)log_buf, prog_bytes);
    flash_range_program(FLASH_LOG_OFFSET, flash_hdr_buf, FLASH_LOG_HDR_BYTES);
    restore_interrupts(irq);

    return (0 == memcmp(saved, log_buf, rec_bytes));
}

/* Prints the log saved in flash (if any), then the current RAM log. Both are
 * kept, so pressing the button again repeats the dump if it was missed. */
static void ram_log_dump (void)
{
    const flash_hdr_t *fh = (const flash_hdr_t *)(XIP_BASE + FLASH_LOG_OFFSET);
    const char *fnames[RAM_LOG_NAMES];
    uint32_t i;

    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }
    sleep_ms(1000);   /* let the terminal finish opening */

    if ((FLASH_LOG_MAGIC == fh->magic) && (fh->count <= RAM_LOG_RECORDS))
    {
        for (i = 0u; i < RAM_LOG_NAMES; i++)
        {
            fnames[i] = fh->names[i];
        }
        printf("FLASHLOG_BEGIN records=%u dropped=%u\n", (unsigned int)fh->count, (unsigned int)fh->dropped);
        print_records((const log_rec_t *)(XIP_BASE + FLASH_LOG_OFFSET + FLASH_SECTOR_SIZE),
                      fh->count, fnames);
        printf("FLASHLOG_END\n");
    }
    else
    {
        printf("FLASHLOG_NONE\n");
    }

    printf("RAMLOG_BEGIN records=%u dropped=%u\n", (unsigned int)log_len, (unsigned int)log_dropped);
    print_records(log_buf, log_len, log_names);
    printf("RAMLOG_END\n");

    while (0 == gpio_get(DUMP_BUTTON_PIN))
    {
        sleep_ms(10);   /* wait for the button to be released */
    }
}
#endif

/* ------------------------------------------------------------- hardware */

static void i2c_setup (void)
{
    i2c_init(IMU_I2C, 400 * 1000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
}

static void led_blink (int n, int on_ms, int off_ms)
{
    for (int i = 0; i < n; i++)
    {
        gpio_put(STATUS_LED_PIN, 1);
        sleep_ms((uint32_t)on_ms);
        gpio_put(STATUS_LED_PIN, 0);
        sleep_ms((uint32_t)off_ms);
    }
}

static uint32_t clock_ms (void)
{
    return to_ms_since_boot(get_absolute_time());
}

/* ------------------------------------------------------------ IMU state */

/* Verify WHO_AM_I, configure the sensor and (first success only) zero the gyro.
 * Blocks for roughly 150 ms, plus about 1 s per calibration try the first time. */
static bool imu_start (void)
{
    if (!mpu9250_init(IMU_I2C, IMU_ADDR))
    {
        return false;
    }

    /* NOTE: the glasses must be still for about 1 s at first power-on.
     * Later re-inits keep the old bias, because the wearer may be moving. */
    for (uint32_t attempt = 0u; (!gyro_cal_done) && (attempt < GYRO_CAL_TRIES); attempt++)
    {
        float range = 0.0f;
        mpu9250_cal_t result = mpu9250_calibrate_gyro(GYRO_CAL_SAMPLES, &range);

#if DEBUG_PRINT
        printf("[%u ms] GYRO_CAL try %u: %s (range %.1f dps)\n",
               (unsigned int)clock_ms(), (unsigned int)(attempt + 1u),
               (MPU9250_CAL_OK == result) ? "ok" :
               ((MPU9250_CAL_MOVING == result) ? "moving" : "bus error"),
               (double)range);
#endif
        if (MPU9250_CAL_OK == result)
        {
            gyro_cal_done = true;
        }
        else if (MPU9250_CAL_BUS_ERROR == result)
        {
            return false;   /* flaky bus: stay in SENSOR_FAULT and retry later */
        }
        else
        {
            /* Moving: try again. */
        }
    }

    if (!gyro_cal_done)
    {
        /* Never still: carry on with zero bias. The stillness check allows 30 dps, so a
         * typical 1-2 dps bias is harmless. Power-cycle while still to recalibrate. */
        gyro_cal_done = true;
#if DEBUG_PRINT
        printf("[%u ms] GYRO_CAL skipped, zero bias\n", (unsigned int)clock_ms());
#endif
    }
    return true;
}

static void imu_enter_fault (uint32_t now_ms)
{
    imu_state = IMU_STATE_FAULT;
    imu_reinit_due_ms = now_ms + IMU_REINIT_MS;
    fall_detect_reset();
    alert_set_fault(ALERT_FAULT_IMU, true);
    log_text(now_ms, "SENSOR_FAULT");
    /* TODO: post SENSOR_FAULT to the BLE task. */
}

static void imu_enter_ok (void)
{
    imu_state = IMU_STATE_OK;
    imu_fail_count = 0u;
    fall_detect_reset();
    alert_set_fault(ALERT_FAULT_IMU, false);
    log_text(clock_ms(), "SENSOR_OK");
    /* TODO: post SENSOR_OK to the BLE task. */
}

/* ---------------------------------------------------------------- falls */

/* The fall event leaves from here. The cane sounds the alarm, not the glasses. */
static void on_fall_detected (uint32_t now_ms)
{
    fall_latched = true;
    fall_unacked = true;
    fall_count++;
    fall_ms = now_ms;
    log_fall(now_ms, fall_count);
    /* TODO: send FALL_EVENT to the BLE task (agree the format with Clifton). */
}

static void status_led_update (uint32_t now_ms)
{
    if (IMU_STATE_FAULT == imu_state)
    {
        /* SENSOR_FAULT: 1 Hz blink, takes precedence over the fall indication. */
        gpio_put(STATUS_LED_PIN, (now_ms % 1000u) < 500u);
    }
    else if (fall_latched)
    {
        /* Fast blink for FALL_LATCH_MS ... */
        if ((now_ms - fall_ms) >= FALL_LATCH_MS)
        {
            fall_latched = false;
        }
        else
        {
            gpio_put(STATUS_LED_PIN, ((now_ms - fall_ms) % 200u) < 100u);
        }
    }
    else
    {
        /* ... then solid ON until the button is pressed. */
        gpio_put(STATUS_LED_PIN, fall_unacked);
    }

    if (fall_unacked && (0 == gpio_get(ACK_BUTTON_PIN)))
    {
        fall_unacked = false;
    }
}

/* ---------------------------------------------------------- trial logging */

#if TRIAL_LOG
static uint32_t trial_no = 0u;
static uint32_t trial_btn_ms = 0u;
static uint32_t dump_down_ms = 0u;     /* when GP22 was first seen pressed */
static bool     dump_down    = false;
static uint32_t accel_log_ms = 0u;

/* Press the trial button (GP21) while holding still, just before each trial.
 * The marker splits the log into trials. A,ax,ay,az lines (2 Hz) give the
 * head orientation over time, which is the data the slump path will need. */
static void trial_log (uint32_t now_ms, const imu_sample_t *s)
{
    if ((0 == gpio_get(TRIAL_BUTTON_PIN)) && ((now_ms - trial_btn_ms) >= 300u))
    {
        trial_btn_ms = now_ms;
        trial_no++;
        log_trial(now_ms, trial_no);
    }

    if ((now_ms - accel_log_ms) >= 500u)
    {
        accel_log_ms = now_ms;
        log_accel(now_ms, s);
    }
}
#endif

/* ----------------------------------------------------------------- main */

int main (void)
{
    imu_sample_t s;
    absolute_time_t next;
#if LOG_ENABLED
    const char *last_state = "";
#endif

    stdio_init_all();   /* no USB wait: output is dropped if nothing is connected */

    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_init(ACK_BUTTON_PIN);
    gpio_set_dir(ACK_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(ACK_BUTTON_PIN);
#if TRIAL_LOG
    gpio_init(TRIAL_BUTTON_PIN);
    gpio_set_dir(TRIAL_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(TRIAL_BUTTON_PIN);
#endif
#if RAM_LOG
    gpio_init(DUMP_BUTTON_PIN);
    gpio_set_dir(DUMP_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(DUMP_BUTTON_PIN);
#endif

    i2c_setup();
    alert_init(BUZZER_PIN);

    if (imu_start())
    {
        imu_enter_ok();
        led_blink(2, 100, 100);   /* ready: two quick blinks */
    }
    else
    {
        imu_enter_fault(clock_ms());
    }

    jitter_reset();
    next = get_absolute_time();

    while (true)
    {
        uint32_t start_us;
        uint32_t now;
        bool overrun;

        next = delayed_by_ms(next, SAMPLE_PERIOD_MS);
        start_us = time_us_32();
        now = clock_ms();

        if (IMU_STATE_OK == imu_state)
        {
            if (mpu9250_read(&s))
            {
                imu_fail_count = 0u;
#if TRIAL_LOG
                trial_log(now, &s);
#endif
                if (fall_detect_update(&s, now))
                {
                    on_fall_detected(now);
                }
#if LOG_ENABLED
                if (fall_detect_state_name() != last_state)
                {
                    last_state = fall_detect_state_name();
                    log_text(now, last_state);
                }
#endif
            }
            else
            {
                imu_fail_count++;
                jitter_note_read_fail();
                if (imu_fail_count >= IMU_FAIL_LIMIT)
                {
                    imu_enter_fault(now);
                }
            }
        }
        else if ((int32_t)(now - imu_reinit_due_ms) >= 0)
        {
            if (imu_start())
            {
                imu_enter_ok();
            }
            else
            {
                imu_reinit_due_ms = clock_ms() + IMU_REINIT_MS;
            }
            next = get_absolute_time();   /* init blocks, so restart the schedule */
            jitter_reset();
        }
        else
        {
            /* SENSOR_FAULT: waiting for the next re-init attempt. */
        }

        status_led_update(now);
        alert_update(now);

#if RAM_LOG
        if (0 == gpio_get(DUMP_BUTTON_PIN))
        {
            if (!dump_down)
            {
                dump_down    = true;
                dump_down_ms = now;
            }
        }
        else
        {
            dump_down = false;
        }

        if (dump_down && ((now - dump_down_ms) >= DUMP_HOLD_MS))
        {
            dump_down = false;
            if (stdio_usb_connected())
            {
                ram_log_dump();
            }
            else
            {
                /* No cable: save the log to flash. 3 slow blinks = saved, 10 fast = failed. */
                if (flash_log_save())
                {
                    led_blink(3, 300, 300);
                }
                else
                {
                    led_blink(10, 50, 50);
                }
            }
            while (0 == gpio_get(DUMP_BUTTON_PIN))
            {
                sleep_ms(10);
            }
            fall_detect_reset();
            next = get_absolute_time();   /* the dump takes seconds, restart the schedule */
            jitter_reset();
            continue;
        }
#endif

        overrun = (absolute_time_diff_us(get_absolute_time(), next) < 0);
        jitter_record(start_us, time_us_32() - start_us, overrun);
        sleep_until(next);
    }
}
