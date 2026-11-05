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
#include "services/BleComm/BleComm_Cfg.h"
#include "services/ImuSupervisor/ImuSupervisor.h"
#include "services/Watchdog/Watchdog.h"
#include "services/Watchdog/Watchdog_Cfg.h"
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
 * \brief Glasses FreeRTOS SMP perception, supervised IMU and cane-status consumer
 * Style: Barr-C:2018; head alarms remain offloaded to the cane.
 */

static Sys_State_t s_glasses_mode = SYS_STATE_INIT;

static uint32_t GetCurrentTimeMs(void)
{
#ifdef PICO_ON_DEVICE
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        return (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
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

    ImuSupervisor_t imu;
    ImuSupervisor_Init(&imu, &Mpu9250_Config_Glasses,
                        GLASSES_PIN_I2C1_SDA, GLASSES_PIN_I2C1_SCL, GetCurrentTimeMs());
    bool sleep_requested = false;
    uint32_t last_movement_ms = GetCurrentTimeMs();
    uint32_t last_imu_sample_ms = last_movement_ms;
    uint32_t last_radar_ms = 0U;
    uint32_t last_btn_ms = 0U;
    uint32_t last_telemetry_ms = 0U;
    uint8_t last_sent_flags = 0xFFU;
    uint16_t buzzer_freq_hz = 0U;
    bool ack_was_pressed = !Hal_Gpio_Read(GLASSES_PIN_ACK_BTN);

    bool head_fall_latched = false;
    uint8_t glasses_fall_count = 0U;
    uint8_t glasses_tx_seq = 0U;
    Threat_Result_t current_threat = {0};
    uint32_t last_valid_radar_frame_ms = 0U;

    Grip_State_t cane_grip_telemetry = GRIP_STATE_UNKNOWN;
    uint32_t last_cane_pkt_ms = 0U;
    bool cane_seen = false;
    bool cane_standby = false;
    uint8_t last_cane_flags = 0xFFU;

    s_glasses_mode = SYS_STATE_INIT;
    Sys_State_t prev_printed_mode = SYS_STATE_INIT;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t now_ms = GetCurrentTimeMs();

        /* Log System Mode State Transitions */
        if (s_glasses_mode != prev_printed_mode)
        {
            const char *mode_str = "ACTIVE";
            if (s_glasses_mode == SYS_STATE_WATCH)          mode_str = "WATCH (HAZARD)";
            if (s_glasses_mode == SYS_STATE_FALL_CONFIRMED) mode_str = "FALL_CONFIRMED";
            if (s_glasses_mode == SYS_STATE_INIT)          mode_str = "IMU STARTING";
            if (s_glasses_mode == SYS_STATE_FAULT_DEGRADED) mode_str = "FAULT DEGRADED";
            if (s_glasses_mode == SYS_STATE_STANDBY)        mode_str = "STANDBY (DOCKED)";
            printf("[SYS MODE] Glasses State Transition -> %s\n", mode_str);
            prev_printed_mode = s_glasses_mode;
        }

        /* Process incoming BLE packets from Cane Node */
        Ble_AdvancedThreatPacket_t rx_pkt;
        while (BleComm_PopReceivedPacket(&rx_pkt))
        {
            if (rx_pkt.device_id == 0x01U) /* 0x01 = Cane Node */
            {
                last_cane_pkt_ms = now_ms;
                cane_seen = true;
                cane_standby = ((rx_pkt.state_flags & BLE_FLAG_STANDBY) != 0U);
                /* An override can park the cane with the touch pad still held. */
                cane_grip_telemetry = !cane_standby && ((rx_pkt.state_flags & BLE_FLAG_CANE_HELD) != 0U) ?
                                      GRIP_STATE_HELD : GRIP_STATE_RELEASED;
                if (rx_pkt.state_flags != last_cane_flags)
                {
                    printf("[CANE STATUS] fall=%u paused=%u standby=%u user_park=%u\n",
                           (unsigned)((rx_pkt.state_flags & BLE_FLAG_FALL_ALERT) != 0U),
                           (unsigned)((rx_pkt.state_flags & BLE_FLAG_ALARM_PAUSED) != 0U),
                           (unsigned)cane_standby,
                           (unsigned)((rx_pkt.state_flags & BLE_FLAG_PARKED_BY_USER) != 0U));
                    last_cane_flags = rx_pkt.state_flags;
                }
            }
        }

        if (!cane_seen || ((now_ms - last_cane_pkt_ms) >= WATCHDOG_CFG_DEADMAN_TIMEOUT_MS))
        {
            cane_grip_telemetry = GRIP_STATE_UNKNOWN;
            cane_standby = false;
        }

        /* The BLE receiver stays awake: a fresh cane wake or uncertain link restores
         * head monitoring. The local acknowledgement button can also wake the glasses.
         */
        if (sleep_requested && (!cane_standby || !Hal_Gpio_Read(GLASSES_PIN_ACK_BTN) || head_fall_latched))
        {
            sleep_requested = false;
            last_movement_ms = now_ms;
            FallDetection_Reset();
        }
        ImuSupervisor_Step(&imu, sleep_requested, now_ms);
        if (imu.fault)
        {
            sleep_requested = false;
        }

        /* Sub-Task 1A: Head Fall Detection (100 Hz / Every Tick) */
        bool new_head_fall = false;
        if (imu.sample_valid)
        {
            if ((uint32_t)(now_ms - last_imu_sample_ms) > 50U)
            {
                last_movement_ms = now_ms;
            }
            last_imu_sample_ms = now_ms;
            const Imu_Sample_t * const p_s = &imu.sample;
            float a_sq = (p_s->ax * p_s->ax) + (p_s->ay * p_s->ay) + (p_s->az * p_s->az);
            float g_sq = (p_s->gx * p_s->gx) + (p_s->gy * p_s->gy) + (p_s->gz * p_s->gz);
            if ((a_sq < 0.81f) || (a_sq > 1.21f) || (g_sq > 25.0f))
            {
                last_movement_ms = now_ms;
            }
            if (!head_fall_latched && FallDetection_Step(&imu.sample, now_ms))
            {
                printf("[GLASSES ALERT] Head fall latched; offloading audio to cane.\n");
                head_fall_latched = true;
                new_head_fall = true;
                glasses_fall_count = (uint8_t)((glasses_fall_count + 1U) & 0x0FU);
                sleep_requested = false;
            }
        }
        else if (imu.state != IMU_SUPERVISOR_ASLEEP)
        {
            last_movement_ms = now_ms;
            FallDetection_Reset();
        }

        /* Explicit cane parking plus local stillness, never an unresolved head fall. */
        if (!sleep_requested && cane_standby && imu.sample_valid && !imu.fault &&
            !head_fall_latched && (FallDetection_GetState() == FALL_STATE_IDLE) &&
            ((uint32_t)(now_ms - last_movement_ms) >= 60000U))
        {
            sleep_requested = true;
            FallDetection_Reset();
        }

        /* Sub-Task 1B: Radar Parser & Threat Assessment (50 Hz / Every 20ms) */
        if ((now_ms - last_radar_ms) >= 20U)
        {
            last_radar_ms = now_ms;
            uint8_t * p_circ_buf = Ld2450_Irq_GetBuffer();
            uint32_t tail = Ld2450_Irq_GetTail();
            uint32_t head = Hal_Uart_GetDmaWriteIndex(256U);

            uint32_t available = (head + 256U - tail) % 256U;
            if (available >= 30U)
            {
                Radar_Target_t targets[3];
                uint8_t target_count = 0U;

                uint8_t frame[30];
                for (uint32_t i = 0U; i < sizeof(frame); ++i)
                {
                    frame[i] = p_circ_buf[(tail + i) % 256U];
                }
                if (Ld2450_ParseFrame(frame, sizeof(frame), targets, &target_count))
                {
                    last_valid_radar_frame_ms = now_ms;
                    ThreatAssessment_ProcessTargets(targets, target_count, cane_grip_telemetry, &current_threat);

                    Ld2450_Irq_SetTail((tail + 30U) % 256U);
                }
                else
                {
                    /* Resynchronize after dropped bytes or a partially overwritten frame. */
                    Ld2450_Irq_SetTail((tail + 1U) % 256U);
                }
            }
        }

        /* Sub-Task 1C: Acknowledge Button Check & Test Triggers (50 ms) */
        if ((now_ms - last_btn_ms) >= 50U)
        {
            last_btn_ms = now_ms;
            bool ack_pressed = !Hal_Gpio_Read(GLASSES_PIN_ACK_BTN);
            if (ack_pressed && !ack_was_pressed && !new_head_fall)
            {
                head_fall_latched = false;
                sleep_requested = false;
                last_movement_ms = now_ms;
                FallDetection_Reset();
                AlertManager_AcknowledgeReset();
            }
            ack_was_pressed = ack_pressed;

            int ch = getchar_timeout_us(0);
            if ((ch == (int)'f') || (ch == (int)'F'))
            {
                printf("[TEST TRIGGER] Simulating Confirmed Head Fall on Glasses!\n");
                sleep_requested = false;
                head_fall_latched = true;
                glasses_fall_count = (uint8_t)((glasses_fall_count + 1U) & 0x0FU);
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
            }
            else if ((ch == (int)'c') || (ch == (int)'C'))
            {
                printf("[TEST TRIGGER] Clearing Head Fall on Glasses\n");
                sleep_requested = false;
                head_fall_latched = false;
                last_movement_ms = now_ms;
                FallDetection_Reset();
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
                AlertManager_AcknowledgeReset();
            }
        }

        bool radar_timeout = (now_ms - last_valid_radar_frame_ms) >= 500U;
        bool standby = (imu.state == IMU_SUPERVISOR_ASLEEP) && sleep_requested;
        bool obstacle = !standby && (cane_grip_telemetry != GRIP_STATE_RELEASED) &&
                        current_threat.is_threat_active && !radar_timeout;
        /* Derive the display mode once, so sensor recovery cannot overwrite a fall. */
        if (head_fall_latched)
        {
            s_glasses_mode = SYS_STATE_FALL_CONFIRMED;
        }
        else if (standby)
        {
            s_glasses_mode = SYS_STATE_STANDBY;
        }
        else if (imu.fault)
        {
            s_glasses_mode = SYS_STATE_FAULT_DEGRADED;
        }
        else if (imu.state == IMU_SUPERVISOR_STARTING)
        {
            s_glasses_mode = SYS_STATE_INIT;
        }
        else
        {
            s_glasses_mode = obstacle ? SYS_STATE_WATCH : SYS_STATE_ACTIVE;
        }
        Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, head_fall_latched ||
                       (imu.fault && ((now_ms % 1000U) < 500U)));

        Alert_Type_t request = ALERT_NONE;
        if (!head_fall_latched && !standby)
        {
            if (obstacle)
            {
                request = ALERT_OBSTACLE_PROXIMITY;
            }
            else if (imu.fault || radar_timeout)
            {
                request = ALERT_SENSOR_FAULT;
            }
        }
        Alert_Output_t alert_out;
        AlertManager_Arbitrate(request, current_threat.nearest_distance_mm, now_ms, &alert_out);
        uint16_t next_freq_hz = alert_out.buzzer_active ? alert_out.generated_freq_hz : 0U;
        if (next_freq_hz != buzzer_freq_hz)
        {
            Buzzer_PlayTone(next_freq_hz);
            buzzer_freq_hz = next_freq_hz;
        }

        uint32_t active_telemetry_period_ms = BLECOMM_CFG_TELEMETRY_IDLE_MS;
        if (obstacle || head_fall_latched || imu.fault)
        {
            active_telemetry_period_ms = BLECOMM_CFG_TELEMETRY_ALERT_MS;
        }

        uint8_t flags = 0U;
        if (head_fall_latched)
        {
            flags |= BLE_FLAG_FALL_ALERT;
        }
        if (radar_timeout || imu.fault)
        {
            flags |= BLE_FLAG_SENSOR_FAULT;
        }
        if (obstacle)
        {
            flags |= BLE_FLAG_OBSTACLE_ACTIVE;
        }
        if (standby)
        {
            flags |= BLE_FLAG_STANDBY;
        }
        if ((flags != last_sent_flags) || ((now_ms - last_telemetry_ms) >= active_telemetry_period_ms))
        {
            uint16_t ttc = (radar_timeout || standby || (current_threat.time_to_collision_ms == UINT16_MAX)) ?
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
                .nearest_dist_mm = (radar_timeout || standby) ? 0xFFFFU : current_threat.nearest_distance_mm,
                .closing_speed_cms = 0,
                .ttc_ms = ttc
            };
            if (BleComm_SendPacket(&heartbeat_pkt))
            {
                glasses_tx_seq++;
                last_telemetry_ms = now_ms;
                last_sent_flags = flags;
            }
        }

        /* Token-Passing: Notify Core 1 that Core 0 safety loop is alive */
        Watchdog_SetCore0Alive();
    }
}

/*
 * =========================================================================
 * CORE 1: TASK 2 - Dedicated Glasses BLE Wireless Radio Server Task
 * Radio work runs on Core 1; shared-bus and scheduling interference still apply.
 * =========================================================================
 */
static void vTask_GlassesBleRadioServer(void *pvParameters)
{
    (void)pvParameters;
    printf("[RTOS] Dedicated Glasses BLE Radio Task started on Core 1!\n");

    for (;;)
    {
        BleComm_Core1_Task();

        /* Token-Passing: Feed HW Watchdog only if Core 0 is actively setting the token */
        Watchdog_FeedHardware();

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

    FallDetection_Init(&FallConfig_Head);
    ThreatAssessment_Init();
    AlertManager_Init();
    Watchdog_Init(&Watchdog_Config_Default);

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
