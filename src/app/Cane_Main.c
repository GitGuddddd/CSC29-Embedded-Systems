#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "Cane_Board.h"
#include "Feature_Cfg.h"
#include "rte/Rte.h"
#include "hal/Hal_I2c.h"
#include "hal/Hal_Gpio.h"
#include "hal/Hal_Pwm.h"
#include "drivers/Mpu9250/Mpu9250.h"
#include "drivers/Vl53l1x/Vl53l1x.h"
#include "drivers/Vl53l0x/Vl53l0x.h"
#include "drivers/TouchSensor/TouchSensor.h"
#include "drivers/CalButton/CalButton.h"
#include "drivers/Haptic/Haptic.h"
#include "drivers/Buzzer/Buzzer.h"
#include "services/BleComm/BleComm.h"
#include "services/Watchdog/Watchdog.h"
#include "services/Watchdog/Watchdog_Cfg.h"
#include "app/FallDetection/FallDetection.h"
#include "app/GroundProjection/GroundProjection.h"
#include "app/AlertManager/AlertManager.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#endif

/*
 * \file Cane_Main.c
 * \brief Smart Walking Cane Cyclic Scheduler & Dual-Core Launcher
 * Compliance: Barr-C:2018
 */

static Gp_Detector_t s_ground_detector;
static uint32_t s_sys_tick_ms = 0U;
static bool s_is_l0x = false;
static bool s_is_l1x = false;

/* Re-initialise the ToF after this long without any reading (see Task 3) */
#define CANE_TOF_RECOVERY_MS      (3000U)

static uint32_t GetCurrentTimeMs(void)
{
#ifdef PICO_ON_DEVICE
    return to_ms_since_boot(get_absolute_time());
#else
    return s_sys_tick_ms;
#endif
}

/*
 * Re-runs the ToF set-up for the model found at boot (both are tried if none was).
 * It blocks on I2C for roughly 15 ms, so callers keep it out of fall-detection windows.
 */
static bool Tof_Reinit(void)
{
    if (s_is_l1x)
    {
        return Vl53l1x_Init(&Vl53l1x_Config_Cane);
    }
    if (s_is_l0x)
    {
        return Vl53l0x_Init(&Vl53l0x_Config_Cane);
    }

    s_is_l0x = Vl53l0x_Init(&Vl53l0x_Config_Cane);
    if (!s_is_l0x)
    {
        s_is_l1x = Vl53l1x_Init(&Vl53l1x_Config_Cane);
    }
    return (s_is_l0x || s_is_l1x);
}

/*
 * Starts ground calibration from the GP20 button or serial 'c'. It needs the
 * grip: the user must be holding the cane in walking position to feel the cues.
 */
static void Ground_StartCalibration(uint32_t now_ms, bool grip_held, const char * p_source)
{
    if (!grip_held)
    {
        printf("[GROUND] Grip the cane before calibrating (%s).\n", p_source);
        return;
    }

    GroundProjection_StartCalibration(&s_ground_detector, now_ms);
    Haptic_PlayOnce(ALERT_CAL_STARTED, now_ms);
    printf("[GROUND] Calibrating (%s): hold the cane still on level ground...\n", p_source);
}

/*
 * Core 1 Entry: Dedicated to CYW43439 SPI radio I/O & BLE Broadcaster/Observer
 */
static void Core1_Entry(void)
{
    while (true)
    {
#ifdef PICO_ON_DEVICE
        BleComm_Core1_Task();
        sleep_ms(10);
#else
        break;
#endif
    }
}

int main(void)
{
#ifdef PICO_ON_DEVICE
    stdio_init_all();
#endif

    /* Wait 1 second for USB serial initialization */
    sleep_ms(1000);

    /* Initialize Hardware Abstraction Layer */
    Hal_I2c_Init(CANE_I2C_IMU_PORT, CANE_PIN_I2C0_SDA, CANE_PIN_I2C0_SCL, 400000U);
    Hal_I2c_Init(CANE_I2C_TOF_PORT, CANE_PIN_I2C1_SDA, CANE_PIN_I2C1_SCL, 400000U);

    TouchSensor_Init(&TouchSensor_Config_Cane);
    CalButton_Init(&CalButton_Config_Cane);
    Haptic_Init(&Haptic_Config_Cane);
    Buzzer_Init(&Buzzer_Config_Cane);
    Watchdog_Init(&Watchdog_Config_Default);
    BleComm_Init(&BleComm_Config_Cane);

    bool imu_ok = Mpu9250_Init(&Mpu9250_Config_Cane);
    printf("[IMU INIT] Cane MPU9250 Status -> %s\n", imu_ok ? "OK" : "FAILED (will retry)");

    /* Model Selection: Try VL53L0X first (0xC0 = 0xEE), then VL53L1X */
    s_is_l0x = Vl53l0x_Init(&Vl53l0x_Config_Cane);
    if (!s_is_l0x)
    {
        s_is_l1x = Vl53l1x_Init(&Vl53l1x_Config_Cane);
    }
    printf("[TOF INIT] Model Detected -> %s\n", s_is_l0x ? "VL53L0X" : (s_is_l1x ? "VL53L1X" : "NONE"));

    FallDetection_Init(&FallConfig_Cane);
    AlertManager_Init();
    /*
     * Use the real clock: the 1 s USB wait above means the first ToF sample
     * arrives > 500 ms after time 0, which the stale-sensor check treated as a
     * fault and silently killed the boot calibration.
     */
    uint32_t boot_ms = GetCurrentTimeMs();
    GroundProjection_Init(&s_ground_detector, boot_ms);
    GroundProjection_SetArmed(&s_ground_detector, true, boot_ms);
    GroundProjection_StartCalibration(&s_ground_detector, boot_ms);
    printf("[GROUND] Auto-calibrating ground height on startup... Hold cane steady.\n");
    printf("[GROUND] To recalibrate later: grip the cane and hold the GP20 button for 2 s (or send 'c').\n");

#ifdef PICO_ON_DEVICE
    /* Launch Core 1 for BLE wireless communication polling */
    multicore_launch_core1(Core1_Entry);
#endif

    printf("\n==================================================\n");
    printf("        EchoGuard Smart Cane Node Initialized\n");
    printf("==================================================\n");

    uint32_t last_touch_ms = 0U;
    uint32_t last_imu_ms = 0U;
    uint32_t last_imu_retry_ms = 0U;
    uint32_t last_tof_ms = 0U;
    uint32_t last_watchdog_ms = 0U;
    uint32_t last_telemetry_ms = 0U;

    uint32_t last_cmd_ms = 0U;
    uint32_t last_tof_alive_ms = GetCurrentTimeMs();
    uint32_t last_tof_reinit_ms = last_tof_alive_ms;
    bool tof_read_failed = false;

    Grip_State_t prev_grip = GRIP_STATE_RELEASED;
    uint16_t last_dist_mm = 0U;
    Gp_State_t last_gstate = GP_STATE_LEVEL;
    Alert_Type_t prev_active_cue = ALERT_NONE;
    uint8_t last_glasses_fall_seq = 0xFFU;
    bool prev_link_lost = false;

    Cane_CompactStatus_t cane_status = {
        .grip_state     = (uint16_t)GRIP_STATE_RELEASED,
        .fall_latched   = 0U,
        .tof_obstructed = 0U,
        .imu_healthy    = imu_ok ? 1U : 0U,
        .link_lost      = 0U,
        .tx_seq         = 0U,
        .fall_seq       = 0U,
        .reserved       = 0U
    };

    while (true)
    {
        uint32_t now_ms = GetCurrentTimeMs();

        /* Auto-retry IMU initialization if it failed on startup or disconnected */
        if (!imu_ok && ((now_ms - last_imu_retry_ms) >= 1000U))
        {
            last_imu_retry_ms = now_ms;
            imu_ok = Mpu9250_Init(&Mpu9250_Config_Cane);
            if (imu_ok)
            {
                printf("[IMU RECOVERY] Cane MPU9250 successfully re-initialized!\n");
            }
        }

        /* Task 1: Touch Detection (every 5 ms, 200 Hz per FR1) */
        if ((now_ms - last_touch_ms) >= 5U)
        {
            last_touch_ms = now_ms;
            bool raw_touch = Hal_Gpio_Read(CANE_PIN_TOUCH_IN);
            Grip_State_t current_grip;
            (void)TouchSensor_Update(&TouchSensor_Config_Cane, raw_touch, &current_grip);

            if (current_grip != prev_grip)
            {
                printf("[TOUCH] State changed -> %s (Raw GPIO: %d)\n",
                       (current_grip == GRIP_STATE_HELD) ? "HELD" : "RELEASED", raw_touch);

                if (current_grip == GRIP_STATE_HELD)
                {
                    /* Regrip detected: immediately silence alarm and reset detection pipeline */
                    if (cane_status.fall_latched != 0U)
                    {
                        printf("[RECOVERY] Regrip confirmed! Silencing Siren & Resetting Fall Alarm\n");
                        cane_status.fall_latched = 0U;
                        AlertManager_AcknowledgeReset();
                        Buzzer_Silence();
                    }
                    FallDetection_Reset();
                }
                else
                {
                    printf("[TOUCH] Cane released - Fall Watch Armed\n");
                    /* Reported as a failed calibration by the ground task */
                    GroundProjection_CancelCalibration(&s_ground_detector, now_ms);
                }
                prev_grip = current_grip;
                cane_status.grip_state = (uint16_t)current_grip;
                Haptic_SetGripHeld(current_grip == GRIP_STATE_HELD);
            }
        }

        /* Calibration inputs: GP20 button held 2 s, or serial 'c' for bench tests (every 50 ms) */
        if ((now_ms - last_cmd_ms) >= 50U)
        {
            last_cmd_ms = now_ms;
            bool grip_held = (prev_grip == GRIP_STATE_HELD);

            if (CalButton_Update(Hal_Gpio_Read(CANE_PIN_CAL_BTN), now_ms) == CALBTN_EVENT_GROUND_HOLD)
            {
                Ground_StartCalibration(now_ms, grip_held, "button");
            }
#ifdef PICO_ON_DEVICE
            int ch = getchar_timeout_us(0);
            if ((ch == (int)'c') || (ch == (int)'C'))
            {
                Ground_StartCalibration(now_ms, grip_held, "serial");
            }
#endif
        }

        /* Task 2: IMU Fall Detection (every 5 ms, 200 Hz per Section 3) */
        if ((now_ms - last_imu_ms) >= 5U)
        {
            last_imu_ms = now_ms;
            Imu_Sample_t imu_sample;
            if (imu_ok && Mpu9250_ReadSample(&Mpu9250_Config_Cane, &imu_sample))
            {
                cane_status.imu_healthy = 1U;
                bool fall_detected = FallDetection_Step(&imu_sample, now_ms);
                if (fall_detected)
                {
                    printf("[ALERT] *** FALL DETECTED! *** Triggering Buzzer Siren\n");
                    cane_status.fall_latched = 1U;
                    cane_status.fall_seq = (uint16_t)((cane_status.fall_seq + 1U) & 0x0FU);
                    Alert_Output_t alert_out;
                    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, now_ms, &alert_out);
                    if (alert_out.buzzer_active)
                    {
                        Buzzer_PlayTone(alert_out.generated_freq_hz);
                    }
                }
            }
            else
            {
                imu_ok = false;
                cane_status.imu_healthy = 0U;
            }
        }

        /* Task 3: Cliff Detection & Ground Projection (every 100 ms, 10 Hz) */
        if ((now_ms - last_tof_ms) >= 100U)
        {
            last_tof_ms = now_ms;
            uint16_t dist_mm = 0U;
            bool read_ok = false;

            if (s_is_l0x)
            {
                read_ok = Vl53l0x_ReadDistanceMm(&Vl53l0x_Config_Cane, &dist_mm);
            }
            else if (s_is_l1x)
            {
                read_ok = Vl53l1x_ReadDistanceMm(&Vl53l1x_Config_Cane, &dist_mm);
            }

            if (read_ok && dist_mm > 20U && dist_mm != 0xFFFFU)
            {
                /* Valid Ground Range */
                if (cane_status.tof_obstructed != 0U)
                {
                    printf("[TOF RECOVERY] Optical Sensor cleared (Distance: %u mm)\n", dist_mm);
                    cane_status.tof_obstructed = 0U;
                }
                last_dist_mm = dist_mm;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_VALID, dist_mm, now_ms);
            }
            else if (read_ok && dist_mm <= 20U)
            {
                /* 0 mm or <= 20 mm: SENSOR OPTICALLY OBSTRUCTED / BLINDED */
                if (cane_status.tof_obstructed == 0U)
                {
                    cane_status.tof_obstructed = 1U;
                    printf("[TOF FAULT] Optical Sensor Obstructed (<= 20 mm)! Triggering Caution Warning\n");
                }
                last_dist_mm = 0U;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_INVALID, 0U, now_ms);
            }
            else if (read_ok && dist_mm == 0xFFFFU)
            {
                /* True Open Air / No Return Target */
                cane_status.tof_obstructed = 0U;
                last_dist_mm = 0xFFFFU;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_NO_RETURN, 0U, now_ms);
            }
            else
            {
                /* I2C/read error is not a no-return: it must not vote toward NO_GROUND or hide a stale sensor */
                cane_status.tof_obstructed = 0U;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_INVALID, 0U, now_ms);
            }

            /* Any answer except "no target" (a real or covered range) shows the sensor is alive */
            tof_read_failed = !read_ok;
            if (read_ok && (dist_mm != 0xFFFFU))
            {
                last_tof_alive_ms = now_ms;
            }

            /*
             * A VL53L1X that glitches or browns out restarts idle, then reports
             * "no target" (or stops answering) until it is set up again, which
             * used to need a replug. Re-initialise it after 3 s without a reading,
             * at most every 3 s, and never while a fall is being checked.
             */
            if (((now_ms - last_tof_alive_ms) >= CANE_TOF_RECOVERY_MS) &&
                ((now_ms - last_tof_reinit_ms) >= CANE_TOF_RECOVERY_MS) &&
                (FallDetection_GetState() == FALL_STATE_IDLE))
            {
                last_tof_reinit_ms = now_ms;
                uint8_t range_status = 0U;
                if (s_is_l1x && Vl53l1x_ReadRangeStatus(&Vl53l1x_Config_Cane, &range_status))
                {
                    printf("[TOF REINIT] No reading for %lu ms (last read: %s, range status 0x%02X). Re-initialising...\n",
                           (unsigned long)(now_ms - last_tof_alive_ms), read_ok ? "no target" : "I2C error",
                           range_status);
                }
                else
                {
                    printf("[TOF REINIT] No reading for %lu ms (last read: %s, range status unreadable). Re-initialising...\n",
                           (unsigned long)(now_ms - last_tof_alive_ms), read_ok ? "no target" : "I2C error");
                }

                if (Tof_Reinit())
                {
                    printf("[TOF REINIT] Sensor restarted.\n");
                }
                else
                {
                    printf("[TOF REINIT] Sensor not responding; retrying in %u ms.\n", (unsigned int)CANE_TOF_RECOVERY_MS);
                }
            }

            Gp_State_t gstate = s_ground_detector.state;
            if (gstate != last_gstate)
            {
                printf("[CLIFF] State -> %s (Distance: %u mm)\n", GroundProjection_GetStateName(gstate), last_dist_mm);
                last_gstate = gstate;
            }

            /* Calibration feedback: 1 s pulse = saved, two 1 s pulses = failed (Week 6 section 6.3) */
            Gp_CalEvent_t cal_event = GroundProjection_TakeCalEvent(&s_ground_detector);
            if (cal_event == GP_CAL_EVENT_DONE)
            {
                Haptic_PlayOnce(ALERT_CAL_DONE, now_ms);
                printf("[GROUND] Calibration complete. Baseline: %u mm\n", s_ground_detector.baseline_mm);
            }
            else if (cal_event != GP_CAL_EVENT_NONE)
            {
                Haptic_PlayOnce(ALERT_CAL_FAILED, now_ms);
                printf("[GROUND] Calibration failed: %s. Previous baseline kept: %s\n",
                       GroundProjection_GetCalEventName(cal_event),
                       s_ground_detector.is_calibrated ? "yes" : "none");
            }
            else
            {
                /* No calibration finished this cycle */
            }

            Alert_Type_t ground_req = (cane_status.tof_obstructed != 0U) ?
                                      ALERT_SENSOR_FAULT : GroundProjection_GetHapticRequest(&s_ground_detector);
            Haptic_Request(HAPTIC_SRC_GROUND, ground_req);
        }

        /* Task 4: BLE Packet Reception & Deadman Watchdog Check (every 50 ms) */
        if ((now_ms - last_watchdog_ms) >= 50U)
        {
            last_watchdog_ms = now_ms;

            Ble_AdvancedThreatPacket_t rx_pkt;
            while (BleComm_PopReceivedPacket(&rx_pkt))
            {
                /* Verify packet is from Glasses node (0x02) */
                if (rx_pkt.device_id == 0x02U)
                {
                    Watchdog_KickBleDeadman(now_ms);

                    /* Wrap-safe rolling fall sequence check (Gap 2) */
                    if (last_glasses_fall_seq == 0xFFU)
                    {
                        last_glasses_fall_seq = rx_pkt.rolling_fall_seq;
                        printf("[BLE SYNC] Linked to Glasses (FallSeq Baseline: %u)\n", rx_pkt.rolling_fall_seq);
                    }
                    else
                    {
                        bool is_new_fall = BleComm_CheckNewFall(rx_pkt.rolling_fall_seq, last_glasses_fall_seq);
                        if (is_new_fall)
                        {
                            printf("[BLE ALERT] *** REMOTE HEAD FALL CONFIRMED (Seq: %u -> %u)! *** Triggering Siren\n",
                                   last_glasses_fall_seq, rx_pkt.rolling_fall_seq);
                            last_glasses_fall_seq = rx_pkt.rolling_fall_seq;
                        }

                        if ((is_new_fall || ((rx_pkt.state_flags & BLE_FLAG_FALL_ALERT) != 0U)) && (cane_status.fall_latched == 0U))
                        {
                            printf("[BLE ALERT] Latched Fall Alert Active! Triggering Buzzer Siren\n");
                            cane_status.fall_latched = 1U;
                            Alert_Output_t alert_out;
                            AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, now_ms, &alert_out);
                            if (alert_out.buzzer_active)
                            {
                                Buzzer_PlayTone(alert_out.generated_freq_hz);
                            }
                        }
                    }
                }
            }

            bool is_link_lost = Watchdog_CheckBleLinkLoss(now_ms);
            if (is_link_lost != prev_link_lost)
            {
                prev_link_lost = is_link_lost;
                cane_status.link_lost = is_link_lost ? 1U : 0U;
                if (is_link_lost)
                {
                    printf("[WATCHDOG] *** BLE LINK LOSS DETECTED (>%u ms)! Entering Degraded Haptic Mode ***\n",
                           (unsigned int)WATCHDOG_CFG_DEADMAN_TIMEOUT_MS);
                    last_glasses_fall_seq = 0xFFU;
                }
                else
                {
                    printf("[WATCHDOG] BLE Link Restored! Resuming Normal Operation\n");
                }
            }

            Haptic_Request(HAPTIC_SRC_LINK, is_link_lost ? ALERT_LINK_LOSS : ALERT_NONE);
        }

        /* Haptic arbitration and pattern timing (every loop, non-blocking) */
        Haptic_Step(now_ms);
        Alert_Type_t active_cue = Haptic_GetActivePattern();
        if ((active_cue != prev_active_cue) && Haptic_IsMotorOn() && s_ground_detector.change_pending &&
            ((active_cue == ALERT_CLIFF_WARNING) || (active_cue == ALERT_CLIFF_STEP_UP)))
        {
            /* Gap 9 evidence: first changed ToF reading -> motor PWM on */
            printf("[GROUND LATENCY] %s: first changed reading -> motor on = %lu ms\n",
                   (active_cue == ALERT_CLIFF_WARNING) ? "STEP_DOWN" : "STEP_UP",
                   (unsigned long)(now_ms - s_ground_detector.change_start_ms));
        }
        prev_active_cue = active_cue;

        /* Task 5: Periodic Telemetry Heartbeat & BLE Broadcast (every 100 ms / 10 Hz) */
        if ((now_ms - last_telemetry_ms) >= 100U)
        {
            last_telemetry_ms = now_ms;

            uint8_t flags = 0;
            if (cane_status.fall_latched != 0U)                    flags |= BLE_FLAG_FALL_ALERT;
            if ((cane_status.tof_obstructed != 0U) || (!s_is_l0x && !s_is_l1x)) flags |= BLE_FLAG_SENSOR_FAULT;
            if (s_ground_detector.state == GP_STATE_STEP_DOWN)     flags |= BLE_FLAG_OBSTACLE_ACTIVE;
            if (cane_status.grip_state == (uint16_t)GRIP_STATE_HELD) flags |= BLE_FLAG_CANE_HELD;

            Ble_AdvancedThreatPacket_t cane_pkt = {
                .preamble = 0xAAU,
                .device_id = 0x01U,
                .pair_id = BLE_DEFAULT_PAIR_ID,
                .node_type = BLE_NODE_TYPE_CANE,
                .heartbeat_seq = (uint8_t)cane_status.tx_seq,
                .rolling_fall_seq = (uint8_t)cane_status.fall_seq,
                .seq_num = (uint16_t)cane_status.tx_seq,
                .state_flags = flags,
                .nearest_dist_mm = (last_dist_mm == 0xFFFFU) ? 0xFFFFU : last_dist_mm,
                .closing_speed_cms = 0,
                .ttc_ms = 0xFFFFU
            };
            cane_status.tx_seq = (uint16_t)((cane_status.tx_seq + 1U) & 0x07U);
            BleComm_SendPacket(&cane_pkt);
        }

        /* Diagnostic Heartbeat (every 1000 ms) */
        static uint32_t s_last_diag_print_ms = 0U;
        if ((now_ms - s_last_diag_print_ms) >= 1000U)
        {
            s_last_diag_print_ms = now_ms;
            /* err=1: the last ToF read failed on I2C, so the distance shown is stale */
            printf("[HEARTBEAT] Grip: %s (GP14=%d) | FallState: %s (latched=%d) | ToF: %u mm (obs=%d, err=%d) | LinkLost: %d\n",
                   (cane_status.grip_state == (uint16_t)GRIP_STATE_HELD) ? "HELD" : "RELEASED",
                   Hal_Gpio_Read(CANE_PIN_TOUCH_IN),
                   FallDetection_GetStateName(),
                   (int)cane_status.fall_latched,
                   last_dist_mm,
                   (int)cane_status.tof_obstructed,
                   (int)tof_read_failed,
                   (int)cane_status.link_lost);
        }

#ifdef PICO_ON_DEVICE
        sleep_us(500);
#else
        s_sys_tick_ms += 1U;
        break;
#endif
    }

    return 0;
}
