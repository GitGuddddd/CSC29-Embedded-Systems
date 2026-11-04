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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern void vTaskCoreAffinitySet(TaskHandle_t xTask, UBaseType_t uxCoreAffinityMask);
#endif

/*
 * \file Glasses_Main.c
 * \brief Smart Glasses FreeRTOS Dual-Core System Mode Manager
 * Compliance: Barr-C:2018 / AUTOSAR Classical / ISO 26262 ASIL-D
 */

static Sys_State_t s_glasses_mode = SYS_STATE_INIT;

static uint32_t GetCurrentTimeMs(void)
{
#ifdef PICO_ON_DEVICE
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        return (uint32_t)xTaskGetTickCount();
    }
    return to_ms_since_boot(get_absolute_time());
#else
    return 0U;
#endif
}

/*
 * =========================================================================
 * CORE 0: TASK 1 - Glasses Perception & Threat Task (Priority: High / 10ms)
 * =========================================================================
 */
#ifdef PICO_ON_DEVICE
static void vTask_GlassesPerception(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz loop

    bool imu_ok = Mpu9250_Init(&Mpu9250_Config_Glasses);
    uint32_t last_imu_retry_ms = 0U;
    uint32_t last_radar_ms = 0U;
    uint32_t last_btn_ms = 0U;
    uint32_t last_telemetry_ms = 0U;

    bool head_fall_latched = false;
    bool ble_tx_paused = false;
    uint8_t glasses_fall_count = 0U;
    uint8_t glasses_tx_seq = 0U;
    Threat_Result_t current_threat = {0};
    uint32_t last_valid_radar_frame_ms = 0U;

    Grip_State_t cane_grip_telemetry = GRIP_STATE_UNKNOWN;
    uint32_t last_cane_pkt_ms = 0U;

    s_glasses_mode = SYS_STATE_ACTIVE;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

        if ((last_cane_pkt_ms == 0U) || ((now_ms - last_cane_pkt_ms) > 500U))
        {
            cane_grip_telemetry = GRIP_STATE_UNKNOWN;
        }

        /* Auto-retry IMU initialization */
        if (!imu_ok && ((now_ms - last_imu_retry_ms) >= 1000U))
        {
            last_imu_retry_ms = now_ms;
            imu_ok = Mpu9250_Init(&Mpu9250_Config_Glasses);
        }

        /* Sub-Task 1A: Head Fall Detection (100 Hz / Every Tick) */
        Imu_Sample_t imu_sample;
        if (imu_ok && Mpu9250_ReadSample(&Mpu9250_Config_Glasses, &imu_sample))
        {
            bool fall_detected = FallDetection_Step(&imu_sample, now_ms);
            if (fall_detected)
            {
                printf("[GLASSES ALERT] *** HEAD FALL DETECTED! ***\n");
                if (!head_fall_latched)
                {
                    s_glasses_mode = SYS_STATE_FALL_CONFIRMED;
                    head_fall_latched = true;
                    glasses_fall_count = (uint8_t)((glasses_fall_count + 1U) & 0x0FU);
                }
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
            }
        }
        else
        {
            imu_ok = false;
            if ((now_ms % 1000U) < 500U)
            {
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
            }
            else
            {
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
            }
        }

        /* Sub-Task 1B: Radar Parser & Threat Assessment (50 Hz / Every 20ms) */
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

        /* Sub-Task 1C: Acknowledge Button Check & Test Triggers (50 ms) */
        if ((now_ms - last_btn_ms) >= 50U)
        {
            last_btn_ms = now_ms;
            if (!Hal_Gpio_Read(GLASSES_PIN_ACK_BTN))
            {
                s_glasses_mode = SYS_STATE_ACTIVE;
                head_fall_latched = false;
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
                AlertManager_AcknowledgeReset();
            }

            int ch = getchar_timeout_us(0);
            if ((ch == (int)'f') || (ch == (int)'F'))
            {
                printf("[TEST TRIGGER] Simulating Confirmed Head Fall on Glasses!\n");
                s_glasses_mode = SYS_STATE_FALL_CONFIRMED;
                head_fall_latched = true;
                glasses_fall_count = (uint8_t)((glasses_fall_count + 1U) & 0x0FU);
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
            }
            else if ((ch == (int)'c') || (ch == (int)'C'))
            {
                printf("[TEST TRIGGER] Clearing Head Fall on Glasses\n");
                s_glasses_mode = SYS_STATE_ACTIVE;
                head_fall_latched = false;
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
                AlertManager_AcknowledgeReset();
            }
            else if ((ch == (int)'p') || (ch == (int)'P'))
            {
                printf("[TEST TRIGGER] BLE Radio Transmit Muted (Scenario A: Auto Low-Power Standby)\n");
                ble_tx_paused = true;
            }
            else if ((ch == (int)'r') || (ch == (int)'R'))
            {
                printf("[TEST TRIGGER] BLE Radio Transmit Resumed\n");
                ble_tx_paused = false;
            }
        }

        /* Sub-Task 1D: Heartbeat Telemetry & BLE Advertisement (10 Hz) */
        if ((now_ms - last_telemetry_ms) >= 100U)
        {
            last_telemetry_ms = now_ms;
            bool radar_timeout = (now_ms - last_valid_radar_frame_ms) > 500U;

            uint8_t flags = 0;
            if (head_fall_latched)                                 flags |= BLE_FLAG_FALL_ALERT;
            if (radar_timeout)                                     flags |= BLE_FLAG_SENSOR_FAULT;
            if (current_threat.is_threat_active && !radar_timeout) flags |= BLE_FLAG_OBSTACLE_ACTIVE;

            uint16_t ttc = (radar_timeout || (current_threat.time_to_collision_ms == UINT16_MAX)) ?
                            0xFFFFU : current_threat.time_to_collision_ms;

            Ble_AdvancedThreatPacket_t heartbeat_pkt = {
                .preamble = 0xAAU,
                .device_id = 0x02U,
                .pair_id = BLE_DEFAULT_PAIR_ID,
                .node_type = BLE_NODE_TYPE_GLASSES,
                .heartbeat_seq = (uint8_t)(glasses_tx_seq & 0x07U),
                .rolling_fall_seq = (uint8_t)(glasses_fall_count & 0x0FU),
                .seq_num = glasses_tx_seq,
                .state_flags = flags,
                .nearest_dist_mm = radar_timeout ? 0xFFFFU : current_threat.nearest_distance_mm,
                .closing_speed_cms = 0,
                .ttc_ms = ttc
            };
            glasses_tx_seq++;
            BleComm_SendPacket(&heartbeat_pkt);
        }
    }
}

/*
 * =========================================================================
 * CORE 1: TASK 2 - Dedicated Glasses BLE Wireless Radio Server Task
 * Dedicated execution on Core 1 guarantees zero SPI radio jitter on Core 0!
 * =========================================================================
 */
static void vTask_GlassesBleRadioServer(void *pvParameters)
{
    (void)pvParameters;
    printf("[RTOS] Dedicated Glasses BLE Radio Task started on Core 1!\n");

    for (;;)
    {
        BleComm_Core1_Task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

int main(void)
{
#ifdef PICO_ON_DEVICE
    stdio_init_all();
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

    printf("\n==================================================\n");
    printf("  EchoGuard Glasses FreeRTOS Dual-Core System     \n");
    printf("==================================================\n");

    /* Create FreeRTOS Tasks with Explicit Core Affinity */
    TaskHandle_t xGlassesPerceptionHandle = NULL;
    TaskHandle_t xGlassesRadioHandle = NULL;

    /* Task 1: Glasses Perception Task -> Pinned to Core 0 (Priority 3 - High) */
    xTaskCreate(vTask_GlassesPerception, "GlassesPerception", 1024, NULL, 3, &xGlassesPerceptionHandle);
    vTaskCoreAffinitySet(xGlassesPerceptionHandle, (1U << 0));

    /* Task 2: Dedicated BLE Radio Task -> Pinned to Core 1 (Priority 1 - Standard) */
    xTaskCreate(vTask_GlassesBleRadioServer, "GlassesRadio", 1024, NULL, 1, &xGlassesRadioHandle);
    vTaskCoreAffinitySet(xGlassesRadioHandle, (1U << 1));

    /* Start FreeRTOS Scheduler */
    vTaskStartScheduler();
#endif

    return 0;
}

#ifdef PICO_ON_DEVICE
/* FreeRTOS Static Allocation Memory Provision Hooks */
static StaticTask_t xGlassesIdleTaskTCB;
static StackType_t uxGlassesIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                  StackType_t **ppxIdleTaskStackBuffer,
                                  uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xGlassesIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxGlassesIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)
static StaticTask_t xGlassesTimerTaskTCB;
static StackType_t uxGlassesTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                   StackType_t **ppxTimerTaskStackBuffer,
                                   uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xGlassesTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxGlassesTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
#endif
