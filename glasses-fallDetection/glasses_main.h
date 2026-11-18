/**
 * @file    glasses_main.h
 * @brief   Configuration of the glasses node: pins, timing and the debug
 *          feature switches.
 */

#ifndef GLASSES_MAIN_H
#define GLASSES_MAIN_H

/* Pins. The buttons are active low. */
#define IMU_SDA_PIN       2u
#define IMU_SCL_PIN       3u
#define BUZZER_PIN        18u     /* The mute switch must be ON. */
#define STATUS_LED_PIN    10u     /* Onboard LED. */
#define ACK_BUTTON_PIN    20u     /* Acknowledges a fall. */
#define TRIAL_BUTTON_PIN  21u     /* Marks the start of a trial. */
#define DUMP_BUTTON_PIN   22u     /* Saves or dumps the log. */

/* IMU bus. */
#define IMU_I2C_ADDR      0x68u
#define IMU_I2C_BAUD_HZ   400000u

/* Timing. */
#define SAMPLE_PERIOD_MS  10u     /* 100 Hz. */
#define FALL_LATCH_MS     3000u   /* Fast-blink window after a fall. */
#define IMU_FAIL_LIMIT    3u      /* Failed reads in a row before a fault. */
#define IMU_REINIT_MS     2000u   /* Retry interval while in SENSOR_FAULT. */
#define GYRO_CAL_SAMPLES  200u
#define GYRO_CAL_TRIES    3u      /* After this many, run with zero bias. */
#define DUMP_HOLD_MS      1500u   /* Hold the dump button this long. */

/* Debug features. Set all of them to 0 for the battery build. */
#define TRIAL_LOG         1       /* Trial marker button and 2 Hz accel log. */
#define RAM_LOG           1       /* 1 keeps the log in RAM, not on the port. */
#define RAM_LOG_RECORDS   8192u   /* 12 bytes each: 96 KB, about 68 min. */
#define RAM_LOG_NAMES     16u
#define DEBUG_PRINT       1       /* Print events to the USB port. */
#define JITTER_LOG        1       /* One summary line per JITTER_WINDOW. */
#define JITTER_WINDOW     1000u   /* 10 s at 100 Hz. */

#endif /* GLASSES_MAIN_H */

/* end of glasses_main.h */

