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
#include "services/BleComm/BleComm_Cfg.h"
#include "services/ImuSupervisor/ImuSupervisor.h"
#include "services/IncidentRetention/IncidentRetention.h"
#include "services/Diagnostics/Diagnostics.h"
#include "services/Watchdog/Watchdog.h"
#include "services/Watchdog/Watchdog_Cfg.h"
#include "app/FallDetection/FallDetection.h"
#include "app/GroundProjection/GroundProjection.h"
#include "app/AlertManager/AlertManager.h"
#include "app/CaneMode/CaneMode.h"

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
 * \brief Cane FreeRTOS SMP integration: single-owner mode manager and supervised I/O
 * Style: Barr-C:2018; application policy is separate from HAL and driver actions.
 */

static Gp_Detector_t s_ground_detector;
static bool s_is_l0x = false;
static bool s_is_l1x = false;
static Sys_State_t s_sys_mode = SYS_STATE_INIT;

#define CANE_TOF_RECOVERY_MS      (3000U)

static uint32_t GetCurrentTimeMs(void)
{
#ifdef PICO_ON_DEVICE
    /* Same monotonic epoch as radio receive timestamps, before and after RTOS start. */
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

    uint32_t start_ms = GetCurrentTimeMs();
    IncidentRetention_Record_t retained_incident = {0};
    bool incident_restored = IncidentRetention_Restore(&retained_incident);
    if (incident_restored)
    {
        BleComm_ExcludeSession(retained_incident.session_id);
    }
    ImuSupervisor_t imu;
    ImuSupervisor_Init(&imu, &Mpu9250_Config_Cane, CANE_PIN_I2C0_SDA, CANE_PIN_I2C0_SCL, start_ms);
    CaneMode_t mode;
    CaneMode_Init(&mode, start_ms);
    if (incident_restored)
    {
        CaneMode_RestoreIncident(&mode);
        printf("[INCIDENT RETENTION] Restored unresolved cane alarm after watchdog reset.\n");
    }
    CaneMode_Output_t mode_out = { .state = SYS_STATE_INIT };
    uint32_t last_tof_ms = 0U;
    uint32_t last_tof_alive_ms = GetCurrentTimeMs();
    uint32_t last_tof_reinit_ms = last_tof_alive_ms;
    uint32_t last_telemetry_ms = 0U;
    uint32_t last_cmd_ms = 0U;

    Grip_State_t prev_grip = GRIP_STATE_RELEASED;
    uint16_t last_dist_mm = 0xFFFFU;
    Gp_State_t last_gstate = GP_STATE_LEVEL;
    BleComm_PeerFall_t peer_fall = {0};
    bool prev_link_lost = false;
    uint8_t last_sent_flags = 0xFFU;
    uint8_t last_sent_event_seq = UINT8_MAX;
    uint16_t last_sent_session_id = 0U;
    Sys_State_t last_sent_state = SYS_STATE_COUNT;
    BleComm_ProfileId_t last_sent_profile = BLECOMM_PROFILE_FAST;
    uint16_t buzzer_freq_hz = 0U;
    uint8_t buzzer_duty_percent = 0U;
    bool peer_ack_valid = false;
    uint16_t peer_ack_session_id = 0U;
    uint8_t peer_ack_event_seq = 0U;
    uint32_t last_glasses_ms = 0U;
    bool last_sent_peer_ack_valid = false;
    uint16_t last_sent_peer_ack_session_id = 0U;
    uint8_t last_sent_peer_ack_event_seq = 0U;

    Cane_CompactStatus_t cane_status = {
        .grip_state     = (uint16_t)GRIP_STATE_RELEASED,
        .fall_latched   = 0U,
        .tof_obstructed = 0U,
        .imu_healthy    = 0U,
        .link_lost      = 0U,
        .tx_seq         = 0U,
        .fall_seq       = incident_restored ? retained_incident.event_seq : 0U,
        .reserved       = 0U
    };
    if (incident_restored)
    {
        IncidentRetention_Store(true, BleComm_GetSessionId(), (uint8_t)cane_status.fall_seq);
    }

    s_sys_mode = SYS_STATE_INIT;
    Sys_State_t prev_printed_mode = SYS_STATE_INIT;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t now_ms = GetCurrentTimeMs();

        /* Touch remains powered and sampled at 100 Hz, including IMU standby. */
        bool raw_touch = Hal_Gpio_Read(CANE_PIN_TOUCH_IN);
        Grip_State_t current_grip;
        (void)TouchSensor_Update(&TouchSensor_Config_Cane, raw_touch, now_ms, &current_grip);

        if (current_grip != prev_grip)
        {
            printf("[TOUCH] State changed -> %s (Raw GPIO: %d)\n",
                   (current_grip == GRIP_STATE_HELD) ? "HELD" : "RELEASED", raw_touch);

            if (current_grip == GRIP_STATE_RELEASED)
            {
                GroundProjection_CancelCalibration(&s_ground_detector, now_ms);
            }
            prev_grip = current_grip;
            cane_status.grip_state = (uint16_t)current_grip;
        }

        bool remote_fall = false;
        bool peer_event_identity_valid = false;
        uint16_t peer_event_session_id = 0U;
        uint8_t peer_event_seq = 0U;
        Ble_AdvancedThreatPacket_t rx_pkt;
        uint32_t received_ms;
        for (uint32_t i = 0U; i < BLECOMM_CFG_RX_QUEUE_SIZE; ++i)
        {
            if (!BleComm_PopReceivedPacket(&rx_pkt, &received_ms))
            {
                break;
            }
            if ((rx_pkt.device_id == 0x02U) &&
                ((uint32_t)(now_ms - received_ms) < WATCHDOG_CFG_DEADMAN_TIMEOUT_MS))
            {
                Watchdog_KickBleDeadman(received_ms);
                last_glasses_ms = received_ms;
                if (BleComm_TrackPeerFall(&peer_fall, &rx_pkt))
                {
                    remote_fall = true;
                    if ((rx_pkt.protocol_version == BLE_PROTOCOL_VERSION) &&
                        (rx_pkt.session_id != 0U))
                    {
                        peer_event_identity_valid = true;
                        peer_event_session_id = rx_pkt.session_id;
                        peer_event_seq = rx_pkt.rolling_fall_seq;
                    }
                }
            }
        }

        /* A regrip/new incident may cancel normal parking before the I2C sleep command. */
        bool sleep_requested = mode_out.imu_sleep_requested;
        if ((mode.power != CANE_POWER_STANDBY) &&
            (remote_fall || ((current_grip == GRIP_STATE_HELD) && !mode.park_override)))
        {
            sleep_requested = false;
        }
        ImuSupervisor_Step(&imu, sleep_requested, now_ms);
        bool local_fall = false;
        if (imu.sample_valid && mode_out.fall_detection_enabled)
        {
            local_fall = FallDetection_Step(&imu.sample, now_ms);
        }
        else if (!imu.sample_valid)
        {
            FallDetection_Reset();
        }
        if (local_fall)
        {
            cane_status.fall_seq = (uint16_t)(BleComm_NextFallSequence(
                (uint8_t)cane_status.fall_seq) & BLE_FALL_SEQ_MASK);
            IncidentRetention_Store(true, BleComm_GetSessionId(), (uint8_t)cane_status.fall_seq);
        }

        bool ground_enabled = mode_out.ground_enabled &&
                              (current_grip == GRIP_STATE_HELD) && !local_fall && !remote_fall;
        GroundProjection_SetArmed(&s_ground_detector, ground_enabled, now_ms);

        /* Sub-Task 1D: Ground ToF Cliff Detection (10 Hz / Every 100ms) */
        if (((now_ms - last_tof_ms) >= 100U) && ground_enabled)
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

        /* Recovery I/O can take longer than one period; stale IMU data must not
         * qualify a parking or upright-recovery transition after that delay.
         */
        now_ms = GetCurrentTimeMs();
        GroundProjection_Tick(&s_ground_detector, now_ms);
        bool ground_fault = (cane_status.tof_obstructed != 0U) || (!s_is_l0x && !s_is_l1x) ||
                            (s_ground_detector.state == GP_STATE_SENSOR_FAULT);
        CaneMode_Input_t mode_in = {
            .grip = current_grip,
            .imu_ready = (imu.state == IMU_SUPERVISOR_RUNNING),
            .imu_sleeping = (imu.state == IMU_SUPERVISOR_ASLEEP),
            .imu_fault = imu.fault,
            .sample_valid = imu.sample_valid,
            .sample = imu.sample,
            .local_fall = local_fall,
            .peer_fall = remote_fall,
            .fall_pending = (FallDetection_GetState() != FALL_STATE_IDLE),
            .ground_hazard = (s_ground_detector.state == GP_STATE_STEP_DOWN) ||
                             (s_ground_detector.state == GP_STATE_STEP_UP),
            .technical_fault = ground_fault
        };
        CaneMode_Step(&mode, &mode_in, now_ms, &mode_out);
        s_sys_mode = mode_out.state;
        cane_status.fall_latched = mode_out.fall_latched ? 1U : 0U;
        IncidentRetention_Store(mode_out.fall_latched, BleComm_GetSessionId(),
                                (uint8_t)cane_status.fall_seq);
        cane_status.imu_healthy = imu.fault ? 0U : 1U;
        if (mode_out.reset_fall_detection)
        {
            FallDetection_Reset();
        }
        bool ground_was_armed = s_ground_detector.is_armed;
        GroundProjection_SetArmed(&s_ground_detector, mode_out.ground_enabled, now_ms);
        if (mode_out.ground_enabled && !ground_was_armed && !s_ground_detector.is_calibrated)
        {
            Ground_StartCalibration(now_ms, true, "first grip");
        }
        Haptic_SetGripHeld(mode_out.ground_enabled);
        if (!mode_out.ground_enabled)
        {
            Haptic_Request(HAPTIC_SRC_GROUND, ALERT_NONE);
        }

        /* The mode manager is the sole alarm owner; the arbitrator renders each pulse. */
        AlertManager_SetFallState(mode_out.fall_latched, mode_out.alarm_paused, now_ms);
        Alert_Output_t alert_out;
        bool technical_audio = mode_out.degraded && (mode.power != CANE_POWER_STANDBY);
        AlertManager_Arbitrate(technical_audio ? ALERT_SENSOR_FAULT : ALERT_NONE,
                               0U, now_ms, &alert_out);
        uint16_t next_freq_hz = alert_out.buzzer_active ? alert_out.generated_freq_hz : 0U;
        uint8_t next_duty_percent = alert_out.buzzer_active ?
                                    alert_out.generated_duty_percent : 0U;
        if ((next_freq_hz != buzzer_freq_hz) || (next_duty_percent != buzzer_duty_percent))
        {
            Buzzer_PlayToneAtDuty(next_freq_hz, next_duty_percent);
            buzzer_freq_hz = next_freq_hz;
            buzzer_duty_percent = next_duty_percent;
        }
        if (remote_fall && peer_event_identity_valid && mode_out.fall_latched)
        {
            /* Receipt is emitted only after the mode manager latched the incident
             * and the alarm request was rendered by the application path above.
             */
            peer_ack_valid = true;
            peer_ack_session_id = peer_event_session_id;
            peer_ack_event_seq = peer_event_seq;
        }
        if (!peer_fall.extended || !peer_fall.active ||
            (peer_fall.session_id != peer_ack_session_id) ||
            (peer_fall.fall_seq != peer_ack_event_seq) ||
            ((uint32_t)(now_ms - last_glasses_ms) >= WATCHDOG_CFG_DEADMAN_TIMEOUT_MS))
        {
            /* Receipt lease follows the accepted, still-current head event. */
            peer_ack_valid = false;
            peer_ack_session_id = 0U;
            peer_ack_event_seq = 0U;
        }
        if (s_sys_mode != prev_printed_mode)
        {
            printf("[SYS MODE] Cane -> %s\n", CaneMode_GetStateName(s_sys_mode));
            prev_printed_mode = s_sys_mode;
        }

        if ((now_ms - last_cmd_ms) >= 50U)
        {
            last_cmd_ms = now_ms;
            bool calibrate_allowed = mode_out.ground_enabled;
            if (CalButton_Update(Hal_Gpio_Read(CANE_PIN_CAL_BTN), now_ms) == CALBTN_EVENT_GROUND_HOLD)
            {
                Ground_StartCalibration(now_ms, calibrate_allowed, "button");
            }
            int ch = getchar_timeout_us(0);
            if ((ch == (int)'c') || (ch == (int)'C'))
            {
                Ground_StartCalibration(now_ms, calibrate_allowed, "serial");
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
            }
        }
        Haptic_Request(HAPTIC_SRC_LINK, is_link_lost ? ALERT_LINK_LOSS : ALERT_NONE);

        uint8_t flags = mode_out.status_flags;
        if (mode_out.ground_enabled && mode_in.ground_hazard)
        {
            flags |= BLE_FLAG_OBSTACLE_ACTIVE;
        }
        BleComm_ProfileId_t profile = BleComm_SelectProfile(s_sys_mode, flags, is_link_lost);
        uint32_t telemetry_period_ms = BleComm_GetProfile(profile)->payload_period_ms;
        if ((flags != last_sent_flags) || (s_sys_mode != last_sent_state) ||
            (cane_status.fall_seq != last_sent_event_seq) ||
            (BleComm_GetSessionId() != last_sent_session_id) ||
            (profile != last_sent_profile) ||
            (peer_ack_valid != last_sent_peer_ack_valid) ||
            (peer_ack_session_id != last_sent_peer_ack_session_id) ||
            (peer_ack_event_seq != last_sent_peer_ack_event_seq) ||
            ((now_ms - last_telemetry_ms) >= telemetry_period_ms))
        {
            Ble_AdvancedThreatPacket_t cane_pkt = {
                .preamble = 0xAAU,
                .device_id = 0x01U,
                .pair_id = BLE_DEFAULT_PAIR_ID,
                .node_type = BLE_NODE_TYPE_CANE,
                .heartbeat_seq = (uint8_t)cane_status.tx_seq,
                .rolling_fall_seq = (uint8_t)cane_status.fall_seq,
                .seq_num = (uint16_t)cane_status.tx_seq,
                .state_flags = flags,
                .nearest_dist_mm = mode_out.ground_enabled ? last_dist_mm : 0xFFFFU,
                .closing_speed_cms = 0,
                .ttc_ms = 0xFFFFU,
                .peer_ack_valid = peer_ack_valid,
                .peer_ack_session_id = peer_ack_session_id,
                .peer_ack_event_seq = peer_ack_event_seq & BLE_FALL_SEQ_MASK
            };
            if (BleComm_Publish(&cane_pkt, s_sys_mode, is_link_lost))
            {
                cane_status.tx_seq = (uint16_t)((cane_status.tx_seq + 1U) & 0x07U);
                last_telemetry_ms = now_ms;
                last_sent_flags = flags;
                last_sent_event_seq = (uint8_t)cane_status.fall_seq;
                last_sent_session_id = BleComm_GetSessionId();
                last_sent_state = s_sys_mode;
                last_sent_profile = profile;
                last_sent_peer_ack_valid = peer_ack_valid;
                last_sent_peer_ack_session_id = peer_ack_session_id;
                last_sent_peer_ack_event_seq = peer_ack_event_seq;
            }
        }

        /* Token-Passing: Notify Core 1 that Core 0 safety loop is alive */
        Watchdog_SetCore0Alive();
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
 * Radio work runs on Core 1; shared-bus and scheduling interference still apply.
 * =========================================================================
 */
static void vTask_BleRadioServer(void *pvParameters)
{
    (void)pvParameters;
    printf("[RTOS] Dedicated BLE Radio Task started on Core 1!\n");

    for (;;)
    {
        if (BleComm_Core1_Task())
        {
            /* A running polling task alone cannot hide a stalled BTstack worker. */
            (void)Watchdog_FeedHardware();
        }
        vTaskDelay(pdMS_TO_TICKS(BLECOMM_CFG_RADIO_TASK_MS));
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
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    Watchdog_Init(&Watchdog_Config_Default);

    printf("\n==================================================\n");
    printf("  EchoGuard Smart Cane FreeRTOS Dual-Core System  \n");
    printf("==================================================\n");

    /* Create FreeRTOS Tasks with Explicit Core Affinity */
    TaskHandle_t xSafetyHandle = NULL;
    TaskHandle_t xActuatorHandle = NULL;
    TaskHandle_t xRadioHandle = NULL;
    TaskHandle_t xDiagnosticsHandle = NULL;

    /* Task 1: Safety & Perception Task -> Pinned to Core 0 (Priority 3 - High) */
    BaseType_t created = xTaskCreate(vTask_SafetyPerception, "SafetyTask", 1024U, NULL, 3U, &xSafetyHandle);
    configASSERT(created == pdPASS);
    vTaskCoreAffinitySet(xSafetyHandle, (1U << 0));

    /* Task 2: Alert Actuator Task -> Pinned to Core 0 (Priority 2 - Medium) */
    created = xTaskCreate(vTask_AlertActuator, "ActuatorTask", 512U, NULL, 2U, &xActuatorHandle);
    configASSERT(created == pdPASS);
    vTaskCoreAffinitySet(xActuatorHandle, (1U << 0));

    /* Task 3: Dedicated BLE Radio Task -> Pinned to Core 1 (Priority 1 - Standard) */
    created = xTaskCreate(vTask_BleRadioServer, "RadioTask", 1024U, NULL, 1U, &xRadioHandle);
    configASSERT(created == pdPASS);
    vTaskCoreAffinitySet(xRadioHandle, (1U << 1));

    created = xTaskCreate(Diagnostics_Task, "Diagnostics", 768U, NULL, 1U, &xDiagnosticsHandle);
    configASSERT(created == pdPASS);
    vTaskCoreAffinitySet(xDiagnosticsHandle, (1U << 0));

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
