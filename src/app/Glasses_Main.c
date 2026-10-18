#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "Glasses_Board.h"
#include "Feature_Cfg.h"
#include "rte/Rte.h"
#include "hal/Hal_I2c.h"
#include "hal/Hal_Uart.h"
#include "hal/Hal_Gpio.h"
#include "hal/Hal_Pwm.h"
#include "drivers/Mpu9250/Mpu9250.h"
#include "drivers/Ld2450/Ld2450.h"
#include "drivers/Buzzer/Buzzer.h"
#include "services/BleComm/BleComm.h"
#include "app/FallDetection/FallDetection.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "app/AlertManager/AlertManager.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#endif

/*
 * \file Glasses_Main.c
 * \brief Smart Glasses Cyclic Scheduler & Dual-Core Launcher
 * Compliance: Barr-C:2018
 */

static uint32_t s_sys_tick_ms = 0U;

static uint32_t GetCurrentTimeMs(void)
{
#ifdef PICO_ON_DEVICE
    return to_ms_since_boot(get_absolute_time());
#else
    return s_sys_tick_ms;
#endif
}

/* 
 * Core 1 Entry: Dedicated to 10 Hz connectionless periodic advertising (FR14)
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

    sleep_ms(1000);

    /* Initialize Hardware Abstraction Layer */
    Hal_I2c_Init(GLASSES_I2C_IMU_PORT, GLASSES_PIN_I2C1_SDA, GLASSES_PIN_I2C1_SCL, 400000U);
    Hal_Uart_Init(GLASSES_UART_RADAR_PORT, GLASSES_PIN_UART0_TX, GLASSES_PIN_UART0_RX, 256000U);
    Hal_Gpio_Init(GLASSES_PIN_BUZZER_PWM, HAL_GPIO_DIR_OUT, false);
    Hal_Gpio_Init(GLASSES_PIN_STATUS_LED, HAL_GPIO_DIR_OUT, false);
    Hal_Gpio_Init(GLASSES_PIN_ACK_BTN, HAL_GPIO_DIR_IN, true);

    Buzzer_Init(&Buzzer_Config_Glasses);
    Ld2450_Init(&Ld2450_Config_Glasses);
    BleComm_Init(&BleComm_Config_Glasses);

    bool imu_ok = Mpu9250_Init(&Mpu9250_Config_Glasses);
    printf("[IMU INIT] Glasses MPU6500 Status -> %s\n", imu_ok ? "OK" : "FAILED (will retry)");

    FallDetection_Init(&FallConfig_Head);
    ThreatAssessment_Init();
    AlertManager_Init();

#ifdef PICO_ON_DEVICE
    multicore_launch_core1(Core1_Entry);
#endif

    printf("\n==================================================\n");
    printf("        EchoGuard Glasses Node Initialized         \n");
    printf("==================================================\n");

    uint32_t last_head_imu_ms = 0U;
    uint32_t last_imu_retry_ms = 0U;
    uint32_t last_radar_ms = 0U;
    uint32_t last_btn_ms = 0U;
    uint32_t last_telemetry_ms = 0U;

    bool head_fall_latched = false;
    Threat_Result_t current_threat = {0};
    uint32_t last_valid_radar_frame_ms = 0U;

    Grip_State_t cane_grip_telemetry = GRIP_STATE_UNKNOWN;
    uint32_t last_cane_pkt_ms = 0U;

    while (true)
    {
        uint32_t now_ms = GetCurrentTimeMs();

        /* Process incoming BLE packets from Cane Node */
        Ble_AdvancedThreatPacket_t rx_pkt;
        while (BleComm_PopReceivedPacket(&rx_pkt))
        {
            if (rx_pkt.device_id == 0x01U) /* 0x01 = Cane Node */
            {
                last_cane_pkt_ms = now_ms;
                cane_grip_telemetry = ((rx_pkt.state_flags & 0x10U) != 0U) ? GRIP_STATE_HELD : GRIP_STATE_RELEASED;
            }
        }

        /* If no packet from Cane for > 500ms, revert to GRIP_STATE_UNKNOWN (fail-safe active) */
        if ((last_cane_pkt_ms == 0U) || ((now_ms - last_cane_pkt_ms) > 500U))
        {
            cane_grip_telemetry = GRIP_STATE_UNKNOWN;
        }

        /* Auto-retry IMU initialization if it failed on startup or disconnected */
        if (!imu_ok && ((now_ms - last_imu_retry_ms) >= 1000U))
        {
            last_imu_retry_ms = now_ms;
            imu_ok = Mpu9250_Init(&Mpu9250_Config_Glasses);
            if (imu_ok)
            {
                printf("[IMU RECOVERY] Glasses MPU6500 successfully re-initialized!\n");
            }
        }

        /* Task 1: Head Fall Detection (every 10 ms, 100 Hz per NFR9) */
        if ((now_ms - last_head_imu_ms) >= 10U)
        {
            last_head_imu_ms = now_ms;
            Imu_Sample_t imu_sample;
            if (imu_ok && Mpu9250_ReadSample(&Mpu9250_Config_Glasses, &imu_sample))
            {
                bool fall_detected = FallDetection_Step(&imu_sample, now_ms);
                if (fall_detected)
                {
                    printf("[GLASSES ALERT] *** HEAD FALL DETECTED! ***\n");
                    head_fall_latched = true;
                    Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
                }
            }
            else
            {
                imu_ok = false;
                /* 1 Hz blink for SENSOR_FAULT */
                if ((now_ms % 1000U) < 500U)
                {
                    Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
                }
                else
                {
                    Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
                }
            }
        }

        /* Task 2: Radar Parser & Threat Assessment (every 20 ms / 50 Hz) */
        if ((now_ms - last_radar_ms) >= 20U)
        {
            last_radar_ms = now_ms;
            uint8_t * p_circ_buf = Ld2450_Irq_GetBuffer();
            uint32_t tail = Ld2450_Irq_GetTail();
            uint32_t head = Hal_Uart_GetDmaWriteIndex(256U);

            if (head != tail)
            {
                Radar_Target_t targets[3];
                uint8_t target_count = 0U;

                if (Ld2450_ParseFrame(&p_circ_buf[tail], 30U, targets, &target_count))
                {
                    last_valid_radar_frame_ms = now_ms;
                    ThreatAssessment_ProcessTargets(targets, target_count, cane_grip_telemetry, &current_threat);

                    if (current_threat.is_threat_active)
                    {
                        Alert_Output_t alert_out;
                        AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, current_threat.nearest_distance_mm, now_ms, &alert_out);
                        if (alert_out.buzzer_active)
                        {
                            Buzzer_PlayTone(alert_out.generated_freq_hz);
                        }
                    }
                    else
                    {
                        Buzzer_Silence();
                    }
                    Ld2450_Irq_SetTail((tail + 30U) % 256U);
                }
            }
        }

        /* Task 3: Acknowledge Button Check (every 50 ms) */
        if ((now_ms - last_btn_ms) >= 50U)
        {
            last_btn_ms = now_ms;
            if (!Hal_Gpio_Read(GLASSES_PIN_ACK_BTN))
            {
                head_fall_latched = false;
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
                AlertManager_AcknowledgeReset();
            }
        }

        /* Task 4: Heartbeat Telemetry & BLE Advertisement (every 100 ms / 10 Hz) */
        if ((now_ms - last_telemetry_ms) >= 100U)
        {
            last_telemetry_ms = now_ms;
            bool radar_timeout = (now_ms - last_valid_radar_frame_ms) > 500U;

            uint8_t flags = 0;
            if (head_fall_latched) flags |= 0x01; /* FLAG_FALL_ALERT */
            if (radar_timeout)     flags |= 0x02; /* FLAG_RADAR_FAULT */
            if (current_threat.is_threat_active && !radar_timeout) flags |= 0x04; /* FLAG_OBSTACLE_ACTIVE */

            uint16_t ttc = (radar_timeout || (current_threat.time_to_collision_ms == UINT16_MAX)) ? 0U : current_threat.time_to_collision_ms;

            Ble_AdvancedThreatPacket_t heartbeat_pkt = {
                .preamble = 0xAA,
                .device_id = 0x02,
                .seq_num = (uint16_t)(now_ms & 0xFFFF),
                .state_flags = flags,
                .nearest_dist_mm = radar_timeout ? 0U : current_threat.nearest_distance_mm,
                .closing_speed_cms = radar_timeout ? 0 : current_threat.closing_speed_cms,
                .ttc_ms = ttc
            };
            BleComm_SendPacket(&heartbeat_pkt);
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
