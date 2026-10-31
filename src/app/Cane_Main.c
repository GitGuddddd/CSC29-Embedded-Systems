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
#include "drivers/Haptic/Haptic.h"
#include "drivers/Buzzer/Buzzer.h"
#include "services/BleComm/BleComm.h"
#include "services/Watchdog/Watchdog.h"
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

static uint32_t GetCurrentTimeMs(void)
{
#ifdef PICO_ON_DEVICE
    return to_ms_since_boot(get_absolute_time());
#else
    return s_sys_tick_ms;
#endif
}

/*
 * Core 1 Entry: Dedicated to CYW43439 SPI radio I/O & BLE Broadcaster/Observer
 */
static void Core1_Entry(void)
{
    while (true)
    {
#ifdef PICO_ON_DEVICE
        cyw43_arch_poll();
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
    GroundProjection_Init(&s_ground_detector, 0U);
    GroundProjection_SetArmed(&s_ground_detector, true, 0U);
    GroundProjection_StartCalibration(&s_ground_detector, 0U);

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

    Grip_State_t prev_grip = GRIP_STATE_RELEASED;
    uint16_t last_dist_mm = 0U;
    Gp_State_t last_gstate = GP_STATE_LEVEL;
    bool fall_alarm_latched = false;
    bool tof_obstructed = false;
    uint8_t last_glasses_fall_seq = 0xFFU;
    uint8_t cane_tx_seq = 0U;
    uint8_t cane_fall_count = 0U;

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
                    if (fall_alarm_latched)
                    {
                        printf("[RECOVERY] Regrip confirmed! Silencing Siren & Resetting Fall Alarm\n");
                        fall_alarm_latched = false;
                        AlertManager_AcknowledgeReset();
                        Buzzer_Silence();
                    }
                    FallDetection_Reset();
                }
                else
                {
                    printf("[TOUCH] Cane released - Fall Watch Armed\n");
                }
                prev_grip = current_grip;
            }
        }

        /* Task 2: IMU Fall Detection (every 5 ms, 200 Hz per Section 3) */
        if ((now_ms - last_imu_ms) >= 5U)
        {
            last_imu_ms = now_ms;
            Imu_Sample_t imu_sample;
            if (imu_ok && Mpu9250_ReadSample(&Mpu9250_Config_Cane, &imu_sample))
            {
                bool fall_detected = FallDetection_Step(&imu_sample, now_ms);
                if (fall_detected)
                {
                    printf("[ALERT] *** FALL DETECTED! *** Triggering Buzzer Siren\n");
                    fall_alarm_latched = true;
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
                if (tof_obstructed)
                {
                    printf("[TOF RECOVERY] Optical Sensor cleared (Distance: %u mm)\n", dist_mm);
                    tof_obstructed = false;
                }
                last_dist_mm = dist_mm;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_VALID, dist_mm, now_ms);
                Gp_State_t gstate = s_ground_detector.state;

                if (gstate != last_gstate)
                {
                    printf("[CLIFF] State -> %d (Distance: %u mm)\n", (int)gstate, dist_mm);
                    last_gstate = gstate;
                }

                if (gstate == GP_STATE_STEP_DOWN)
                {
                    Haptic_PlayPattern(ALERT_CLIFF_WARNING);
                }
                else if (gstate == GP_STATE_STEP_UP)
                {
                    Haptic_PlayPattern(ALERT_CLIFF_STEP_UP);
                }
                else
                {
                    Haptic_Abort();
                }
            }
            else if (read_ok && dist_mm <= 20U)
            {
                /* 0 mm or <= 20 mm: SENSOR OPTICALLY OBSTRUCTED / BLINDED */
                if (!tof_obstructed)
                {
                    tof_obstructed = true;
                    printf("[TOF FAULT] Optical Sensor Obstructed (<= 20 mm)! Triggering Caution Warning\n");
                    Haptic_PlayPattern(ALERT_SENSOR_FAULT);
                }
                last_dist_mm = 0U;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_INVALID, 0U, now_ms);
            }
            else if (read_ok && dist_mm == 0xFFFFU)
            {
                /* True Open Air / No Return Target */
                tof_obstructed = false;
                last_dist_mm = 0xFFFFU;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_NO_RETURN, 0U, now_ms);
            }
            else
            {
                tof_obstructed = false;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_NO_RETURN, 0U, now_ms);
            }
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
                    else if (BleComm_CheckNewFall(rx_pkt.rolling_fall_seq, last_glasses_fall_seq) ||
                             ((rx_pkt.state_flags & BLE_FLAG_FALL_ALERT) != 0U))
                    {
                        printf("[BLE ALERT] *** REMOTE HEAD FALL CONFIRMED (Seq: %u -> %u)! *** Triggering Siren\n",
                               last_glasses_fall_seq, rx_pkt.rolling_fall_seq);
                        last_glasses_fall_seq = rx_pkt.rolling_fall_seq;
                        if (!fall_alarm_latched)
                        {
                            fall_alarm_latched = true;
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

            if (Watchdog_CheckBleLinkLoss(now_ms))
            {
                last_glasses_fall_seq = 0xFFU; /* Reset baseline on link loss */
                Haptic_PlayPattern(ALERT_LINK_LOSS);
            }
        }

        /* Task 5: Periodic Telemetry Heartbeat & BLE Broadcast (every 100 ms / 10 Hz) */
        if ((now_ms - last_telemetry_ms) >= 100U)
        {
            last_telemetry_ms = now_ms;

            uint8_t flags = 0;
            if (fall_alarm_latched)                                flags |= BLE_FLAG_FALL_ALERT;
            if (tof_obstructed || (!s_is_l0x && !s_is_l1x))       flags |= BLE_FLAG_SENSOR_FAULT;
            if (s_ground_detector.state == GP_STATE_STEP_DOWN)     flags |= BLE_FLAG_OBSTACLE_ACTIVE;
            if (prev_grip == GRIP_STATE_HELD)                      flags |= BLE_FLAG_CANE_HELD;

            Ble_AdvancedThreatPacket_t cane_pkt = {
                .preamble = 0xAAU,
                .device_id = 0x01U,
                .pair_id = BLE_DEFAULT_PAIR_ID,
                .node_type = BLE_NODE_TYPE_CANE,
                .heartbeat_seq = (uint8_t)(cane_tx_seq & 0x07U),
                .rolling_fall_seq = (uint8_t)(cane_fall_count & 0x0FU),
                .seq_num = cane_tx_seq,
                .state_flags = flags,
                .nearest_dist_mm = (last_dist_mm == 0xFFFFU) ? 0xFFFFU : last_dist_mm,
                .closing_speed_cms = 0,
                .ttc_ms = 0xFFFFU
            };
            cane_tx_seq++;
            BleComm_SendPacket(&cane_pkt);
        }

        /* Diagnostic Heartbeat (every 1000 ms) */
        static uint32_t s_last_diag_print_ms = 0U;
        if ((now_ms - s_last_diag_print_ms) >= 1000U)
        {
            s_last_diag_print_ms = now_ms;
            printf("[HEARTBEAT] Grip: %s (GP14=%d) | FallState: %s (latched=%d) | ToF: %u mm (obs=%d)\n",
                   (prev_grip == GRIP_STATE_HELD) ? "HELD" : "RELEASED",
                   Hal_Gpio_Read(CANE_PIN_TOUCH_IN),
                   FallDetection_GetStateName(),
                   (int)fall_alarm_latched,
                   last_dist_mm,
                   (int)tof_obstructed);
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
