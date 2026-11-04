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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern void vTaskCoreAffinitySet(TaskHandle_t xTask, UBaseType_t uxCoreAffinityMask);
#endif

/*
 * \file Cane_Main.c
 * \brief Smart Walking Cane FreeRTOS Dual-Core (AMP) System Mode Manager (BswM / ComM)
 * Compliance: Barr-C:2018 / AUTOSAR Classical Mode Management / ISO 26262 ASIL-D
 */

static Gp_Detector_t s_ground_detector;
static bool s_is_l0x = false;
static bool s_is_l1x = false;
static Sys_State_t s_sys_mode = SYS_STATE_INIT;

#define CANE_TOF_RECOVERY_MS      (3000U)

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
 * =========================================================================
 * CORE 0: TASK 1 - Safety & Perception Task (Priority: High / 10ms Period)
 * Handles System Mode Management (Sys_State_t), Fall Detection, ToF, Touch
 * =========================================================================
 */
#ifdef PICO_ON_DEVICE
static void vTask_SafetyPerception(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz cyclic loop

    bool imu_ok = Mpu9250_Init(&Mpu9250_Config_Cane);
    uint32_t last_imu_retry_ms = 0U;
    uint32_t last_tof_ms = 0U;
    uint32_t last_tof_alive_ms = GetCurrentTimeMs();
    uint32_t last_tof_reinit_ms = last_tof_alive_ms;
    uint32_t last_telemetry_ms = 0U;
    uint32_t last_cmd_ms = 0U;

    Grip_State_t prev_grip = GRIP_STATE_RELEASED;
    uint16_t last_dist_mm = 0U;
    Gp_State_t last_gstate = GP_STATE_LEVEL;
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

    s_sys_mode = SYS_STATE_ACTIVE;
    Sys_State_t prev_printed_mode = SYS_STATE_INIT;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t now_ms = GetCurrentTimeMs();

        /* Log System State Transitions */
        if (s_sys_mode != prev_printed_mode)
        {
            const char *mode_str = "ACTIVE";
            if (s_sys_mode == SYS_STATE_WATCH)          mode_str = "WATCH (HAZARD)";
            if (s_sys_mode == SYS_STATE_FALL_CONFIRMED) mode_str = "FALL_CONFIRMED";
            if (s_sys_mode == SYS_STATE_FALL_MUTED)     mode_str = "FALL_MUTED";
            if (s_sys_mode == SYS_STATE_HARD_FAULT)     mode_str = "HARD_FAULT";
            printf("[SYS MODE] Cane State Transition -> %s\n", mode_str);
            prev_printed_mode = s_sys_mode;
        }

        /* IMU Recovery Supervisor */
        if (!imu_ok && ((now_ms - last_imu_retry_ms) >= 1000U))
        {
            last_imu_retry_ms = now_ms;
            imu_ok = Mpu9250_Init(&Mpu9250_Config_Cane);
            if (imu_ok)
            {
                printf("[IMU RECOVERY] Cane MPU9250 successfully re-initialized!\n");
                if (s_sys_mode == SYS_STATE_HARD_FAULT)
                {
                    s_sys_mode = SYS_STATE_ACTIVE;
                }
            }
            else
            {
                s_sys_mode = SYS_STATE_HARD_FAULT;
            }
        }

        /* Sub-Task 1A: Touch Sensor Sampling (200 Hz / Every Tick) */
        bool raw_touch = Hal_Gpio_Read(CANE_PIN_TOUCH_IN);
        Grip_State_t current_grip;
        (void)TouchSensor_Update(&TouchSensor_Config_Cane, raw_touch, &current_grip);

        if (current_grip != prev_grip)
        {
            printf("[TOUCH] State changed -> %s (Raw GPIO: %d)\n",
                   (current_grip == GRIP_STATE_HELD) ? "HELD" : "RELEASED", raw_touch);

            if (current_grip == GRIP_STATE_HELD)
            {
                if (s_sys_mode == SYS_STATE_FALL_CONFIRMED)
                {
                    printf("[RECOVERY] Regrip confirmed! Silencing Siren & Resetting Fall Alarm\n");
                    s_sys_mode = SYS_STATE_FALL_MUTED;
                    cane_status.fall_latched = 0U;
                    AlertManager_AcknowledgeReset();
                    Buzzer_Silence();
                }
                FallDetection_Reset();
            }
            else
            {
                printf("[TOUCH] Cane released - Fall Watch Armed\n");
                GroundProjection_CancelCalibration(&s_ground_detector, now_ms);
            }
            prev_grip = current_grip;
            cane_status.grip_state = (uint16_t)current_grip;
            Haptic_SetGripHeld(current_grip == GRIP_STATE_HELD);
        }

        /* Sub-Task 1B: Calibration Button Check (50 ms) */
        if ((now_ms - last_cmd_ms) >= 50U)
        {
            last_cmd_ms = now_ms;
            bool grip_held = (prev_grip == GRIP_STATE_HELD);

            if (CalButton_Update(Hal_Gpio_Read(CANE_PIN_CAL_BTN), now_ms) == CALBTN_EVENT_GROUND_HOLD)
            {
                Ground_StartCalibration(now_ms, grip_held, "button");
            }
            int ch = getchar_timeout_us(0);
            if ((ch == (int)'c') || (ch == (int)'C'))
            {
                Ground_StartCalibration(now_ms, grip_held, "serial");
            }
        }

        /* Sub-Task 1C: Local IMU Fall Detection (100 Hz / Every Tick) */
        Imu_Sample_t imu_sample;
        if (imu_ok && Mpu9250_ReadSample(&Mpu9250_Config_Cane, &imu_sample))
        {
            cane_status.imu_healthy = 1U;
            bool fall_detected = FallDetection_Step(&imu_sample, now_ms);
            if (fall_detected)
            {
                printf("[ALERT] *** FALL DETECTED! *** Triggering Buzzer Siren\n");
                s_sys_mode = SYS_STATE_FALL_CONFIRMED;
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

        /* Sub-Task 1D: Ground ToF Cliff Detection (10 Hz / Every 100ms) */
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
                cane_status.tof_obstructed = 0U;
                last_dist_mm = 0xFFFFU;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_NO_RETURN, 0U, now_ms);
            }
            else
            {
                cane_status.tof_obstructed = 0U;
                GroundProjection_Feed(&s_ground_detector, GP_RANGE_INVALID, 0U, now_ms);
            }

            if (read_ok && (dist_mm != 0xFFFFU))
            {
                last_tof_alive_ms = now_ms;
            }

            /* ToF Recovery Supervisor */
            if (((now_ms - last_tof_alive_ms) >= CANE_TOF_RECOVERY_MS) &&
                ((now_ms - last_tof_reinit_ms) >= CANE_TOF_RECOVERY_MS) &&
                (FallDetection_GetState() == FALL_STATE_IDLE))
            {
                last_tof_reinit_ms = now_ms;
                (void)Tof_Reinit();
            }

            Gp_State_t gstate = s_ground_detector.state;
            if (gstate != last_gstate)
            {
                printf("[CLIFF] State -> %s (Distance: %u mm)\n", GroundProjection_GetStateName(gstate), last_dist_mm);
                last_gstate = gstate;
            }

            /* Update System Mode for Active Hazards */
            if ((gstate == GP_STATE_STEP_DOWN) || (gstate == GP_STATE_STEP_UP))
            {
                if (s_sys_mode == SYS_STATE_ACTIVE)
                {
                    s_sys_mode = SYS_STATE_WATCH;
                }
            }
            else if (s_sys_mode == SYS_STATE_WATCH)
            {
                s_sys_mode = SYS_STATE_ACTIVE;
            }

            Gp_CalEvent_t cal_event = GroundProjection_TakeCalEvent(&s_ground_detector);
            if (cal_event == GP_CAL_EVENT_DONE)
            {
                Haptic_PlayOnce(ALERT_CAL_DONE, now_ms);
                printf("[GROUND] Calibration complete. Baseline: %u mm\n", s_ground_detector.baseline_mm);
            }
            else if (cal_event != GP_CAL_EVENT_NONE)
            {
                Haptic_PlayOnce(ALERT_CAL_FAILED, now_ms);
            }

            Alert_Type_t ground_req = (cane_status.tof_obstructed != 0U) ?
                                      ALERT_SENSOR_FAULT : GroundProjection_GetHapticRequest(&s_ground_detector);
            Haptic_Request(HAPTIC_SRC_GROUND, ground_req);
        }

        /* Sub-Task 1E: BLE Wireless Reception & Watchdog Check */
        Ble_AdvancedThreatPacket_t rx_pkt;
        while (BleComm_PopReceivedPacket(&rx_pkt))
        {
            if (rx_pkt.device_id == 0x02U)
            {
                Watchdog_KickBleDeadman(now_ms);

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
                        printf("[BLE ALERT] *** REMOTE HEAD FALL CONFIRMED (Seq: %u -> %u)! ***\n",
                               last_glasses_fall_seq, rx_pkt.rolling_fall_seq);
                        last_glasses_fall_seq = rx_pkt.rolling_fall_seq;
                    }

                    if ((is_new_fall || ((rx_pkt.state_flags & BLE_FLAG_FALL_ALERT) != 0U)) && (cane_status.fall_latched == 0U))
                    {
                        s_sys_mode = SYS_STATE_FALL_CONFIRMED;
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
                printf("[WATCHDOG] BLE LINK LOSS DETECTED (>%u ms)\n", (unsigned int)WATCHDOG_CFG_DEADMAN_TIMEOUT_MS);
                last_glasses_fall_seq = 0xFFU;
            }
        }
        Haptic_Request(HAPTIC_SRC_LINK, is_link_lost ? ALERT_LINK_LOSS : ALERT_NONE);

        /* Sub-Task 1F: Telemetry Broadcast (10 Hz) */
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
    }
}

/*
 * =========================================================================
 * CORE 0: TASK 2 - Haptic Actuator Task (Priority: Medium / 10ms Period)
 * =========================================================================
 */
static void vTask_AlertActuator(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t now_ms = GetCurrentTimeMs();
        Haptic_Step(now_ms);
    }
}

/*
 * =========================================================================
 * CORE 1: TASK 3 - Dedicated BLE Wireless Radio Server Task
 * Dedicated execution on Core 1 guarantees zero SPI radio jitter on Core 0!
 * =========================================================================
 */
static void vTask_BleRadioServer(void *pvParameters)
{
    (void)pvParameters;
    printf("[RTOS] Dedicated BLE Radio Task started on Core 1!\n");

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
    Hal_I2c_Init(CANE_I2C_IMU_PORT, CANE_PIN_I2C0_SDA, CANE_PIN_I2C0_SCL, 400000U);
    Hal_I2c_Init(CANE_I2C_TOF_PORT, CANE_PIN_I2C1_SDA, CANE_PIN_I2C1_SCL, 400000U);

    TouchSensor_Init(&TouchSensor_Config_Cane);
    CalButton_Init(&CalButton_Config_Cane);
    Haptic_Init(&Haptic_Config_Cane);
    Buzzer_Init(&Buzzer_Config_Cane);
    Watchdog_Init(&Watchdog_Config_Default);
    BleComm_Init(&BleComm_Config_Cane);

    s_is_l0x = Vl53l0x_Init(&Vl53l0x_Config_Cane);
    if (!s_is_l0x)
    {
        s_is_l1x = Vl53l1x_Init(&Vl53l1x_Config_Cane);
    }

    FallDetection_Init(&FallConfig_Cane);
    AlertManager_Init();

    uint32_t boot_ms = GetCurrentTimeMs();
    GroundProjection_Init(&s_ground_detector, boot_ms);
    GroundProjection_SetArmed(&s_ground_detector, true, boot_ms);
    GroundProjection_StartCalibration(&s_ground_detector, boot_ms);

    printf("\n==================================================\n");
    printf("  EchoGuard Smart Cane FreeRTOS Dual-Core System  \n");
    printf("==================================================\n");

    /* Create FreeRTOS Tasks with Explicit Core Affinity */
    TaskHandle_t xSafetyHandle = NULL;
    TaskHandle_t xActuatorHandle = NULL;
    TaskHandle_t xRadioHandle = NULL;

    /* Task 1: Safety & Perception Task -> Pinned to Core 0 (Priority 3 - High) */
    xTaskCreate(vTask_SafetyPerception, "SafetyTask", 1024, NULL, 3, &xSafetyHandle);
    vTaskCoreAffinitySet(xSafetyHandle, (1U << 0));

    /* Task 2: Alert Actuator Task -> Pinned to Core 0 (Priority 2 - Medium) */
    xTaskCreate(vTask_AlertActuator, "ActuatorTask", 512, NULL, 2, &xActuatorHandle);
    vTaskCoreAffinitySet(xActuatorHandle, (1U << 0));

    /* Task 3: Dedicated BLE Radio Task -> Pinned to Core 1 (Priority 1 - Standard) */
    xTaskCreate(vTask_BleRadioServer, "RadioTask", 1024, NULL, 1, &xRadioHandle);
    vTaskCoreAffinitySet(xRadioHandle, (1U << 1));

    /* Start FreeRTOS Scheduler */
    vTaskStartScheduler();
#endif

    return 0;
}

#ifdef PICO_ON_DEVICE
/* FreeRTOS Static Allocation Memory Provision Hooks */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                  StackType_t **ppxIdleTaskStackBuffer,
                                  uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                   StackType_t **ppxTimerTaskStackBuffer,
                                   uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
#endif
