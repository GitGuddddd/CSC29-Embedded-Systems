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
#include "drivers/BleRadio/BleRadio.h"
#include "services/BleComm/BleComm.h"
#include "services/BleComm/BleComm_Cfg.h"
#include "services/ImuSupervisor/ImuSupervisor.h"
#include "services/IncidentRetention/IncidentRetention.h"
#include "services/Diagnostics/Diagnostics.h"
#include "services/Watchdog/Watchdog.h"
#include "services/Watchdog/Watchdog_Cfg.h"
#include "services/SafetyDelivery/SafetyDelivery.h"
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
    /* Use the radio's monotonic boot-time epoch for receive freshness. */
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
typedef struct
{
    uint8_t sequence : 4;
    uint8_t latched : 1;
    uint8_t reserved : 3;
} Glasses_FallState_t; /* Private Core 0 record; history is accumulated by the phone. */

/* Ends the head-fall incident on the glasses: used by GP20, the 'c' test key
 * and an acknowledgement made on the cane.
 */
static void Clear_HeadFall(Glasses_FallState_t * const p_fall,
                           SafetyDelivery_t * const p_delivery)
{
    p_fall->latched = 0U;
    FallDetection_Reset();
    AlertManager_AcknowledgeReset();
    SafetyDelivery_Clear(p_delivery);
    IncidentRetention_Store(false, BleComm_GetSessionId(), p_fall->sequence);
}

/* Plain-language view of the radar decision, toggled by the 'v' key: the three
 * lanes, what the glasses decide and the buzzer pattern (ported from the
 * earlier test-branch patch "Add a plain-language radar view"). cue: bit0 STOP,
 * bits1-2 side (0 centre, 1 left, 2 right, 3 no free side lane). quiet: 0
 * audible, 1 standby, 2 cane not held, 3 radar timeout, 4 obstacle moving
 * away. */
static const char * const s_quiet_text[5] = { "", "standby", "cane not held",
                                              "radar timeout",
                                              "obstacle moving away" };

/* The decision and the buzzer pattern, in words, for the lanes and cue. */
static void RadarView_Words(uint8_t blocked_mask,
                            uint8_t cue,
                            const char ** const pp_decision,
                            const char ** const pp_buzzer)
{
    const bool stop = ((cue & THREAT_CUE_STOP) != 0U);
    const uint8_t side = (uint8_t)((cue >> THREAT_CUE_SIDE_SHIFT) & 0x03U);
    const char *decision = "ALL CLEAR";
    const char *buzzer = "SILENT";

    if (blocked_mask != 0U)
    {
        if (stop)
        {
            buzzer = "CONTINUOUS TONE";
            if (side == (uint8_t)THREAT_CUE_SIDE_ALL_BLOCKED)
            {
                decision = "STOP (no way round)";
            }
            else if (side == (uint8_t)THREAT_LANE_LEFT)
            {
                decision = "STOP (left is open)";
            }
            else if (side == (uint8_t)THREAT_LANE_RIGHT)
            {
                decision = "STOP (right is open)";
            }
            else
            {
                decision = "STOP";
            }
        }
        else if (side == (uint8_t)THREAT_LANE_LEFT)
        {
            decision = "MOVE LEFT";
            buzzer = "2 BEEPS";
        }
        else if (side == (uint8_t)THREAT_LANE_RIGHT)
        {
            decision = "MOVE RIGHT";
            buzzer = "3 BEEPS";
        }
        else if (side == (uint8_t)THREAT_CUE_SIDE_ALL_BLOCKED)
        {
            decision = "NO FREE LANE (slow down)";
            buzzer = "4 SHORT BEEPS";
        }
        else
        {
            decision = "KEEP GOING (obstacle at the side)";
            buzzer = "1 BEEP";
        }
    }
    *pp_decision = decision;
    *pp_buzzer = buzzer;
}

static void RadarView_Print(uint8_t blocked_mask,
                            uint8_t cue,
                            uint16_t dist_mm,
                            int16_t closing_cms,
                            uint8_t quiet)
{
    const char *decision = "ALL CLEAR";
    const char *buzzer = "SILENT";
    RadarView_Words(blocked_mask, cue, &decision, &buzzer);

    printf("\n----------------------------------------\n");
    printf(" LEFT   : %s\n",
           ((blocked_mask & (1U << (uint8_t)THREAT_LANE_LEFT)) != 0U)
               ? "OBSTACLE DETECTED"
               : "clear");
    printf(" MIDDLE : %s\n",
           ((blocked_mask & (1U << (uint8_t)THREAT_LANE_CENTER)) != 0U)
               ? "OBSTACLE DETECTED"
               : "clear");
    printf(" RIGHT  : %s\n",
           ((blocked_mask & (1U << (uint8_t)THREAT_LANE_RIGHT)) != 0U)
               ? "OBSTACLE DETECTED"
               : "clear");
    if (blocked_mask != 0U)
    {
        printf(" NEAREST: %u mm, closing %d cm/s\n", (unsigned)dist_mm,
               (int)closing_cms);
    }
    printf(" DECISION: %s\n", decision);
    if ((quiet != 0U) && (quiet <= 4U) && (blocked_mask != 0U))
    {
        printf(" BUZZER  : SILENT (%s), would be %s\n", s_quiet_text[quiet],
               buzzer);
    }
    else
    {
        printf(" BUZZER  : %s\n", buzzer);
    }
}

/* Snapshot for the readable radar debug ('d' key). */
typedef struct
{
    Radar_Target_t targets[3]; /* last radar frame, as the radar sent it */
    Threat_Result_t threat;
    uint32_t now_ms;
    uint8_t blocked_mask;      /* bit n = lane n blocked */
    uint8_t quiet;             /* 0 audible, else s_quiet_text index */
    uint8_t filters;           /* THREAT_FLT_* since the last print */
    uint8_t implausible;       /* bit n = slot n rejected by the driver */
    Grip_State_t grip;
    bool radar_timeout;
} RadarDebug_t;

/* "1.23 m" from millimetres. */
static void Print_Metres(uint32_t mm)
{
    printf("%u.%02u m", (unsigned)(mm / 1000U), (unsigned)((mm % 1000U) / 10U));
}

/* One radar slot in words: where the target is, how it moves, and what the
 * decision made of it. */
static void RadarDebug_PrintSlot(const RadarDebug_t * const p_dbg,
                                 uint8_t slot)
{
    static const char * const status_text[6] = {
        "empty", "-> centre lane", "-> left lane", "-> right lane",
        "ignored: outside the 1.5 m corridor",
        "ignored: only crossing the path"
    };
    const Radar_Target_t * const p_t = &p_dbg->targets[slot];
    printf("  slot %u  : ", (unsigned)slot);
    if (!p_t->is_valid)
    {
        printf("%s\n", (((p_dbg->implausible >> slot) & 1U) != 0U)
                            ? "rejected: impossible value from the radar"
                            : "empty");
        return;
    }

    const int16_t lat = ThreatAssessment_UserLateralMm(p_t->x_mm);
    const uint32_t abs_lat = (lat < 0) ? (uint32_t)(-(int32_t)lat)
                                       : (uint32_t)lat;
    const int16_t closing = ThreatAssessment_ClosingCms(p_t->speed_cms);
    uint8_t status = ThreatAssessment_GetSlotStatus(slot);
    if (status > 5U)
    {
        status = 0U;
    }

    Print_Metres(ThreatAssessment_ForwardMm(p_t->distance_mm, lat));
    printf(" ahead, ");
    if (abs_lat < 50U)
    {
        printf("in line with you");
    }
    else
    {
        Print_Metres(abs_lat);
        printf(" %s", (lat < 0) ? "left" : "right");
    }
    if ((closing > -8) && (closing < 8))
    {
        printf(", standing still");
    }
    else if (closing > 0)
    {
        printf(", coming closer at %d cm/s", (int)closing);
    }
    else
    {
        printf(", moving away at %d cm/s", (int)(-closing));
    }
    printf("  %s\n", status_text[status]);
}

/* Readable radar debug: decision, lanes, nearest obstacle, each radar slot,
 * and which filters acted. */
static void RadarDebug_Print(const RadarDebug_t * const p_dbg)
{
    static const char * const flt_text[6] = {
        "outside corridor", "crossing", "impossible value",
        "radar re-ordered its slots", "moving away (quiet)",
        "still-person hold"
    };
    static const char * const grip_text[3] = { "released", "held",
                                               "unknown" };
    const uint8_t mask = p_dbg->blocked_mask;
    const char *decision = "ALL CLEAR";
    const char *buzzer = "SILENT";
    RadarView_Words(mask, (uint8_t)p_dbg->threat.open_direction_cue,
                    &decision, &buzzer);

    printf("\n[RADAR %u.%u s] %s | buzzer: ",
           (unsigned)(p_dbg->now_ms / 1000U),
           (unsigned)((p_dbg->now_ms % 1000U) / 100U), decision);
    if ((p_dbg->quiet != 0U) && (p_dbg->quiet <= 4U) && (mask != 0U))
    {
        printf("SILENT (%s)\n", s_quiet_text[p_dbg->quiet]);
    }
    else
    {
        printf("%s\n", buzzer);
    }

    printf("  lanes   : left %s | centre %s | right %s\n",
           (((mask >> (uint8_t)THREAT_LANE_LEFT) & 1U) != 0U) ? "BLOCKED"
                                                               : "clear",
           (((mask >> (uint8_t)THREAT_LANE_CENTER) & 1U) != 0U) ? "BLOCKED"
                                                                 : "clear",
           (((mask >> (uint8_t)THREAT_LANE_RIGHT) & 1U) != 0U) ? "BLOCKED"
                                                                : "clear");
    if (mask != 0U)
    {
        printf("  nearest : ");
        Print_Metres(p_dbg->threat.nearest_distance_mm);
        printf(", closing %d cm/s, ", (int)p_dbg->threat.closing_speed_cms);
        if (p_dbg->threat.ttc_20ms >= BLE_TTC_NO_THREAT)
        {
            printf("no collision course\n");
        }
        else
        {
            const uint32_t ttc_ms = (uint32_t)p_dbg->threat.ttc_20ms * 20U;
            printf("collision in %u.%u s\n", (unsigned)(ttc_ms / 1000U),
                   (unsigned)((ttc_ms % 1000U) / 100U));
        }
    }
    for (uint8_t slot = 0U; slot < 3U; slot++)
    {
        RadarDebug_PrintSlot(p_dbg, slot);
    }

    printf("  filters : ");
    if (p_dbg->filters == 0U)
    {
        printf("none");
    }
    else
    {
        bool first = true;
        for (uint8_t b = 0U; b < 6U; b++)
        {
            if (((p_dbg->filters >> b) & 1U) != 0U)
            {
                printf("%s%s", first ? "" : ", ", flt_text[b]);
                first = false;
            }
        }
    }
    printf(" | cane: %s | radar: %s\n",
           ((uint8_t)p_dbg->grip <= 2U) ? grip_text[p_dbg->grip] : "?",
           p_dbg->radar_timeout ? "TIMED OUT" : "ok");
}

static void vTask_GlassesPerception(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz loop

    uint32_t start_ms = GetCurrentTimeMs();
    IncidentRetention_Record_t retained_incident = {0};
    bool incident_restored = IncidentRetention_Restore(&retained_incident);
    if (incident_restored)
    {
        BleComm_ExcludeSession(retained_incident.session_id);
    }
    ImuSupervisor_t imu;
    ImuSupervisor_Init(&imu, &Mpu9250_Config_Glasses,
                        GLASSES_PIN_I2C1_SDA, GLASSES_PIN_I2C1_SCL, start_ms);
    /* Gyro offset calibration: uses the normal 10 ms samples, never blocks. */
    Mpu9250_GyroCal_t gyro_cal;
    Mpu9250_GyroCalInit(&gyro_cal);
    bool gyro_cal_wait_logged = false;
    bool sleep_requested = false;
    uint32_t last_movement_ms = start_ms;
    uint32_t last_imu_sample_ms = last_movement_ms;
    uint32_t last_radar_ms = 0U;
    uint32_t last_radar_retry_ms = 0U;
    uint32_t last_btn_ms = 0U;
    uint32_t last_telemetry_ms = 0U;
    uint8_t last_sent_flags = 0xFFU;
    uint8_t last_sent_event_seq = UINT8_MAX;
    uint16_t last_sent_session_id = 0U;
    Sys_State_t last_sent_state = SYS_STATE_COUNT;
    BleComm_ProfileId_t last_sent_profile = BLECOMM_PROFILE_FAST;
    uint16_t buzzer_freq_hz = 0U;
    uint8_t buzzer_duty_percent = 0U;
    bool ack_was_pressed = !Hal_Gpio_Read(GLASSES_PIN_ACK_BTN);

    Glasses_FallState_t head_fall = {
        .sequence = incident_restored ? retained_incident.event_seq : 0U,
        .latched = incident_restored ? 1U : 0U
    };
    uint8_t glasses_tx_seq = 0U;
    SafetyDelivery_t fall_delivery;
    SafetyDelivery_Init(&fall_delivery, BleComm_GetSessionId());
    if (incident_restored)
    {
        SafetyDelivery_Start(&fall_delivery, BleComm_GetSessionId(), head_fall.sequence, start_ms);
        IncidentRetention_Store(true, BleComm_GetSessionId(), head_fall.sequence);
        printf("[INCIDENT RETENTION] Restored unresolved head fall after watchdog reset.\n");
    }
    bool previous_delivery_fallback = false;
    Threat_Result_t current_threat = {0};
    /* 'r': raw one-line radar debug, 10 Hz (for logs) */
    bool radar_raw = false;
    uint32_t radar_raw_last_ms = 0U;
    /* 'd': readable radar debug, 2 Hz */
    bool radar_debug = false;
    uint32_t radar_debug_last_ms = 0U;
    RadarDebug_t radar_dbg = { 0 };
    /* toggled by 'v': plain-language lanes, decision and buzzer */
    bool radar_view = false;
    uint32_t radar_view_last_ms = 0U;
    uint32_t radar_view_sig = UINT32_MAX;
    int16_t radar_dbg_x[3] = { 0, 0, 0 };   /* last frame, for the d debug line */
    uint8_t radar_dbg_valid = 0U;
    /* THREAT_FLT_* bits of every frame since the last raw / readable print */
    uint8_t radar_raw_flt = 0U;
    uint8_t radar_dbg_flt = 0U;
    uint32_t last_valid_radar_frame_ms = 0U;

    Grip_State_t cane_grip_telemetry = GRIP_STATE_UNKNOWN;
    uint32_t last_cane_pkt_ms = 0U;
    bool cane_seen = false;
    bool cane_standby = false;
    bool cane_buzzer_fault = false;
    bool cane_fall_active = false;
    bool cane_sensor_fault = false;
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
        uint32_t received_ms;
        bool cane_acknowledged = false;
        for (uint32_t i = 0U; i < BLECOMM_CFG_RX_QUEUE_SIZE; ++i)
        {
            if (!BleComm_PopReceivedPacket(&rx_pkt, &received_ms))
            {
                break;
            }
            if ((rx_pkt.device_id == 0x01U) &&
                ((uint32_t)(now_ms - received_ms) < WATCHDOG_CFG_DEADMAN_TIMEOUT_MS))
            {
                SafetyDelivery_ObservePeer(&fall_delivery, &rx_pkt, received_ms);
                /* The cane took this exact fall (receipt) and has since
                 * cleared its alarm (touch pad or cane recovery).
                 */
                if ((head_fall.latched != 0U) &&
                    SafetyDelivery_IsConfirmed(&fall_delivery) &&
                    ((rx_pkt.state_flags &
                      (BLE_FLAG_FALL_ALERT | BLE_FLAG_ALARM_PAUSED)) == 0U))
                {
                    cane_acknowledged = true;
                }
                last_cane_pkt_ms = received_ms;
                cane_seen = true;
                cane_standby = ((rx_pkt.state_flags & BLE_FLAG_STANDBY) != 0U);
                cane_buzzer_fault = ((rx_pkt.state_flags & BLE_FLAG_BUZZER_FAULT) != 0U);
                cane_fall_active = ((rx_pkt.state_flags & BLE_FLAG_FALL_ALERT) != 0U) &&
                                   ((rx_pkt.state_flags & BLE_FLAG_ALARM_PAUSED) == 0U);
                cane_sensor_fault = ((rx_pkt.state_flags & BLE_FLAG_SENSOR_FAULT) != 0U);
                /* An override can park the cane with the touch pad still held. */
                cane_grip_telemetry = !cane_standby && ((rx_pkt.state_flags & BLE_FLAG_CANE_HELD) != 0U) ?
                                      GRIP_STATE_HELD : GRIP_STATE_RELEASED;
                if (rx_pkt.state_flags != last_cane_flags)
                {
                    printf("[CANE STATUS] fall=%u paused=%u standby=%u user_park=%u buzzer_fault=%u\n",
                           (unsigned)((rx_pkt.state_flags & BLE_FLAG_FALL_ALERT) != 0U),
                           (unsigned)((rx_pkt.state_flags & BLE_FLAG_ALARM_PAUSED) != 0U),
                           (unsigned)cane_standby,
                           (unsigned)((rx_pkt.state_flags & BLE_FLAG_PARKED_BY_USER) != 0U),
                           (unsigned)cane_buzzer_fault);
                    last_cane_flags = rx_pkt.state_flags;
                }
            }
        }

        if (cane_acknowledged)
        {
            printf("[GLASSES] Head fall acknowledged on the cane; cleared.\n");
            Clear_HeadFall(&head_fall, &fall_delivery);
            sleep_requested = false;
            last_movement_ms = now_ms;
        }

        if (!cane_seen || ((now_ms - last_cane_pkt_ms) >= WATCHDOG_CFG_DEADMAN_TIMEOUT_MS))
        {
            cane_grip_telemetry = GRIP_STATE_UNKNOWN;
            cane_standby = false;
            cane_buzzer_fault = false;
            cane_fall_active = false;
            cane_sensor_fault = false;
        }

        /* The BLE receiver stays awake: a fresh cane wake or uncertain link restores
         * head monitoring. The local acknowledgement button can also wake the glasses.
         */
        if (sleep_requested && (!cane_standby || !Hal_Gpio_Read(GLASSES_PIN_ACK_BTN) || head_fall.latched))
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
            Mpu9250_GyroCalStatus_t gyro_cal_status =
                Mpu9250_GyroCalStep(&gyro_cal, &imu.sample);
            if (gyro_cal_status == MPU9250_GYRO_CAL_APPLIED)
            {
                float off_x = 0.0f;
                float off_y = 0.0f;
                float off_z = 0.0f;
                Mpu9250_GetGyroOffset(&off_x, &off_y, &off_z);
                printf("[IMU] Gyro offset set: x=%.2f y=%.2f z=%.2f dps\n",
                       (double)off_x, (double)off_y, (double)off_z);
            }
            else if ((gyro_cal_status == MPU9250_GYRO_CAL_REJECTED) &&
                     !gyro_cal_wait_logged)
            {
                printf("[IMU] Gyro calibration waiting: keep the glasses "
                       "still for 1 s\n");
                gyro_cal_wait_logged = true;
            }
            else
            {
                /* Collecting or already calibrated: nothing to report. */
            }

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
            if (!head_fall.latched && FallDetection_Step(&imu.sample, now_ms))
            {
                printf("[GLASSES ALERT] Head fall latched; awaiting cane application receipt.\n");
                head_fall.latched = 1U;
                new_head_fall = true;
                head_fall.sequence = BleComm_NextFallSequence(head_fall.sequence) & BLE_FALL_SEQ_MASK;
                SafetyDelivery_Start(&fall_delivery, BleComm_GetSessionId(), head_fall.sequence, now_ms);
                IncidentRetention_Store(true, BleComm_GetSessionId(), head_fall.sequence);
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
            !head_fall.latched && (FallDetection_GetState() == FALL_STATE_IDLE) &&
            ((uint32_t)(now_ms - last_movement_ms) >= 60000U))
        {
            sleep_requested = true;
            FallDetection_Reset();
        }

        /* Sub-Task 1B: Radar Parser & Threat Assessment (50 Hz / Every 20ms) */
        if ((now_ms - last_radar_ms) >= 20U)
        {
            last_radar_ms = now_ms;
            Radar_Target_t targets[3];
            uint8_t target_count = 0U;
            if (Ld2450_ProcessStream(targets, &target_count, now_ms))
            {
                last_valid_radar_frame_ms = now_ms;
                radar_dbg_valid = 0U;
                for (uint8_t k = 0U; (k < 3U) && (k < target_count); k++)
                {
                    radar_dbg_x[k] = targets[k].x_mm;
                    if (targets[k].is_valid)
                    {
                        radar_dbg_valid |= (uint8_t)(1U << k);
                    }
                }
                ThreatAssessment_ProcessTargetsAt(targets, target_count, cane_grip_telemetry, now_ms, &current_threat);
                uint8_t flt = ThreatAssessment_GetFilterFlags();
                if (Ld2450_GetImplausibleMask() != 0U)
                {
                    flt |= THREAT_FLT_IMPLAUSIBLE;
                }
                radar_raw_flt |= flt;
                radar_dbg_flt |= flt;
                for (uint8_t k = 0U; k < 3U; k++)
                {
                    radar_dbg.targets[k] = targets[k];
                }
                radar_dbg.implausible = Ld2450_GetImplausibleMask();
            }
            if (Ld2450_HasRxFault() && ((uint32_t)(now_ms - last_radar_retry_ms) >= 1000U))
            {
                last_radar_retry_ms = now_ms;
                (void)Ld2450_Init(&Ld2450_Config_Glasses);
                /* Do not reuse the old threat after restarting the DMA stream. */
                current_threat.is_threat_active = false;
                current_threat.open_direction_cue = 0U;
            }
        }

        /* Sub-Task 1C: Acknowledge Button Check & Test Triggers (50 ms) */
        if ((now_ms - last_btn_ms) >= 50U)
        {
            last_btn_ms = now_ms;
            bool ack_pressed = !Hal_Gpio_Read(GLASSES_PIN_ACK_BTN);
            if (ack_pressed && !ack_was_pressed && !new_head_fall)
            {
                Clear_HeadFall(&head_fall, &fall_delivery);
                sleep_requested = false;
                last_movement_ms = now_ms;
            }
            ack_was_pressed = ack_pressed;

            int ch = getchar_timeout_us(0);
            if ((ch == (int)'f') || (ch == (int)'F'))
            {
                printf("[TEST TRIGGER] Simulating Confirmed Head Fall on Glasses!\n");
                sleep_requested = false;
                head_fall.latched = 1U;
                head_fall.sequence = BleComm_NextFallSequence(head_fall.sequence) & BLE_FALL_SEQ_MASK;
                SafetyDelivery_Start(&fall_delivery, BleComm_GetSessionId(), head_fall.sequence, now_ms);
                IncidentRetention_Store(true, BleComm_GetSessionId(), head_fall.sequence);
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, true);
            }
            else if ((ch == (int)'c') || (ch == (int)'C'))
            {
                printf("[TEST TRIGGER] Clearing Head Fall on Glasses\n");
                Clear_HeadFall(&head_fall, &fall_delivery);
                sleep_requested = false;
                last_movement_ms = now_ms;
                Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, false);
            }
            else if ((ch == (int)'d') || (ch == (int)'D'))
            {
                radar_debug = !radar_debug;
                radar_dbg_flt = 0U;
                printf("[RADAR DEBUG] %s\n",
                       radar_debug ? "ON (readable, twice a second)" : "OFF");
            }
            else if ((ch == (int)'r') || (ch == (int)'R'))
            {
                radar_raw = !radar_raw;
                printf("[RADAR RAW] %s\n",
                       radar_raw ? "ON (one line, 10 Hz)" : "OFF");
            }
            else if ((ch == (int)'v') || (ch == (int)'V'))
            {
                radar_view = !radar_view;
                /* print the current state straight away */
                radar_view_sig = UINT32_MAX;
                printf("[RADAR VIEW] %s\n",
                       radar_view ? "ON (prints when the decision changes)"
                                  : "OFF");
            }
            else if ((ch == (int)'s') || (ch == (int)'S'))
            {
                printf("[STATUS] up=%u mode=%u ready=%u adv=%u scan=%u imu_f=%u radar_to=%u cane_seen=%u tx_seq=%u\n",
                       (unsigned)now_ms, (unsigned)s_glasses_mode,
                       (unsigned)BleRadio_IsReady(), (unsigned)BleRadio_IsAdvertising(),
                       (unsigned)BleRadio_IsScanning(), (unsigned)imu.fault,
                       (unsigned)((now_ms - last_valid_radar_frame_ms) >= 500U || Ld2450_HasRxFault()),
                       (unsigned)cane_seen, (unsigned)glasses_tx_seq);
            }
        }

        bool radar_timeout = ((now_ms - last_valid_radar_frame_ms) >= 500U) || Ld2450_HasRxFault();
        if (radar_raw && ((now_ms - radar_raw_last_ms) >= 100U))
        {
            radar_raw_last_ms = now_ms;
            /* ttc is in 20 ms ticks; 4095 = none. cue: bit0 STOP, bits1-2 side (0 centre, 1 left, 2 right). */
            /* ln: blocked lanes (bit0 centre, bit1 left, bit2 right); v: valid target slots; x: raw radar x per slot (mm). */
            printf("[RADAR] t=%u dist=%u spd=%d ttc=%u thr=%u cue=%u to=%u "
                   "grip=%u ln=%u v=%u x=%d,%d,%d",
                   (unsigned)now_ms, (unsigned)current_threat.nearest_distance_mm,
                   (int)current_threat.closing_speed_cms, (unsigned)current_threat.ttc_20ms,
                   (unsigned)current_threat.is_threat_active, (unsigned)current_threat.open_direction_cue,
                   (unsigned)radar_timeout, (unsigned)cane_grip_telemetry,
                   (unsigned)ThreatAssessment_GetBlockedMask(), (unsigned)radar_dbg_valid,
                   (int)radar_dbg_x[0], (int)radar_dbg_x[1], (int)radar_dbg_x[2]);
            /* flt: THREAT_FLT_* bits since the last line (bit0 outside
             * corridor, bit1 crossing, bit2 implausible value, bit3 slot
             * re-ordered, bit4 moving away, bit5 still-person hold). */
            printf(" flt=0x%02X\n", (unsigned)radar_raw_flt);
            radar_raw_flt = 0U;
        }
        bool standby = (imu.state == IMU_SUPERVISOR_ASLEEP) && sleep_requested;
        bool obstacle = !standby && (cane_grip_telemetry != GRIP_STATE_RELEASED) &&
                        current_threat.is_threat_active && !radar_timeout;
        /* Buzzer only: stay quiet while every blocked lane's obstacle is moving
         * away. */
        bool obstacle_audible = obstacle && !ThreatAssessment_IsReceding();
        /* Why the buzzer is quiet, checked in the same order as the gates
         * above (for the v and d views). */
        uint8_t quiet = 0U;
        if (current_threat.is_threat_active && !obstacle_audible)
        {
            if (standby)
            {
                quiet = 1U;
            }
            else if (radar_timeout)
            {
                quiet = 3U;
            }
            else if (!obstacle)
            {
                /* the only remaining gate on obstacle: cane released */
                quiet = 2U;
            }
            else
            {
                quiet = 4U; /* obstacle moving away */
            }
        }
        if (radar_debug && ((uint32_t)(now_ms - radar_debug_last_ms) >= 500U))
        {
            radar_debug_last_ms = now_ms;
            radar_dbg.threat = current_threat;
            radar_dbg.now_ms = now_ms;
            radar_dbg.blocked_mask = ThreatAssessment_GetBlockedMask();
            radar_dbg.quiet = quiet;
            radar_dbg.filters = radar_dbg_flt;
            radar_dbg.grip = cane_grip_telemetry;
            radar_dbg.radar_timeout = radar_timeout;
            RadarDebug_Print(&radar_dbg);
            radar_dbg_flt = 0U;
        }
        if (radar_view)
        {
            const uint8_t view_mask = ThreatAssessment_GetBlockedMask();
            const uint32_t sig =
                (uint32_t)view_mask |
                ((uint32_t)current_threat.open_direction_cue << 4U) |
                ((uint32_t)quiet << 8U);
            if ((sig != radar_view_sig) &&
                ((uint32_t)(now_ms - radar_view_last_ms) >= 200U))
            {
                radar_view_sig = sig;
                radar_view_last_ms = now_ms;
                RadarView_Print(view_mask,
                                (uint8_t)current_threat.open_direction_cue,
                                current_threat.nearest_distance_mm,
                                current_threat.closing_speed_cms, quiet);
            }
        }
        /* Derive the display mode once, so sensor recovery cannot overwrite a fall. */
        if (head_fall.latched)
        {
            s_glasses_mode = SYS_STATE_FALL_CONFIRMED;
        }
        else if (standby)
        {
            s_glasses_mode = SYS_STATE_STANDBY;
        }
        else if (imu.fault || radar_timeout)
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
        Hal_Gpio_Write(GLASSES_PIN_STATUS_LED, head_fall.latched ||
                       (imu.fault && ((now_ms % 1000U) < 500U)));

        bool delivery_fallback = head_fall.latched &&
                                 SafetyDelivery_NeedsFallback(&fall_delivery, now_ms);
        bool cane_fresh = cane_seen &&
                          ((uint32_t)(now_ms - last_cane_pkt_ms) < WATCHDOG_CFG_DEADMAN_TIMEOUT_MS);
        Alert_SyncInput_t sync_in = {
            .is_glasses_node = true,
            .local_fall_latched = (head_fall.latched != 0U),
            .local_alarm_paused = false,
            .peer_fall_active = cane_fall_active,
            .delivery_fallback = delivery_fallback,
            .cane_link_fresh = cane_fresh,
            .cane_buzzer_fault = cane_buzzer_fault,
            .obstacle_active = obstacle_audible,
            .obstacle_dist_mm = current_threat.nearest_distance_mm,
            .obstacle_cue = (uint8_t)current_threat.open_direction_cue,
            .sensor_fault = !standby && (imu.fault || radar_timeout),
            .peer_sensor_fault = !standby && cane_sensor_fault
        };
        Alert_Output_t alert_out;
        AlertManager_ArbitrateSynced(&sync_in, now_ms, &alert_out);
        uint16_t next_freq_hz = alert_out.buzzer_active ? alert_out.generated_freq_hz : 0U;
        uint8_t next_duty_percent = alert_out.buzzer_active ?
                                    alert_out.generated_duty_percent : 0U;
        if ((next_freq_hz != buzzer_freq_hz) || (next_duty_percent != buzzer_duty_percent))
        {
            Buzzer_PlayToneAtDuty(next_freq_hz, next_duty_percent);
            buzzer_freq_hz = next_freq_hz;
            buzzer_duty_percent = next_duty_percent;
        }
        if (delivery_fallback != previous_delivery_fallback)
        {
            printf("[HEAD FALL DELIVERY] %s (receipt=%u cane_fresh=%u)\n",
                   delivery_fallback ? "LOCAL LOW-DUTY FALLBACK" : "cane receipt path active",
                   (unsigned)SafetyDelivery_IsConfirmed(&fall_delivery),
                   (unsigned)(cane_seen &&
                              ((uint32_t)(now_ms - last_cane_pkt_ms) <
                               WATCHDOG_CFG_DEADMAN_TIMEOUT_MS)));
            previous_delivery_fallback = delivery_fallback;
        }

        uint8_t flags = 0U;
        if (head_fall.latched)
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
        bool link_lost = (cane_grip_telemetry == GRIP_STATE_UNKNOWN);
        BleComm_ProfileId_t profile = BleComm_SelectProfile(s_glasses_mode, flags, link_lost);
        uint32_t telemetry_period_ms = BleComm_GetProfile(profile)->payload_period_ms;
        if ((flags != last_sent_flags) || (s_glasses_mode != last_sent_state) ||
            (head_fall.sequence != last_sent_event_seq) ||
            (BleComm_GetSessionId() != last_sent_session_id) ||
            (profile != last_sent_profile) || ((now_ms - last_telemetry_ms) >= telemetry_period_ms))
        {
            uint16_t ttc = (radar_timeout || standby) ?
                           BLE_TTC_NO_THREAT : current_threat.ttc_20ms;

            Ble_AdvancedThreatPacket_t heartbeat_pkt = {
                .preamble = 0xAAU,
                .device_id = 0x02U,
                .pair_id = BLE_DEFAULT_PAIR_ID,
                .node_type = BLE_NODE_TYPE_GLASSES,
                .heartbeat_seq = (uint8_t)(glasses_tx_seq & 0x07U),
                .rolling_fall_seq = head_fall.sequence,
                .seq_num = glasses_tx_seq,
                .state_flags = flags,
                .nearest_dist_mm = (radar_timeout || standby) ? 0xFFFFU : current_threat.nearest_distance_mm,
                .closing_speed_cms = 0,
                .ttc_20ms = ttc & 0x0FFFU
            };
            if (BleComm_Publish(&heartbeat_pkt, s_glasses_mode, link_lost))
            {
                glasses_tx_seq++;
                last_telemetry_ms = now_ms;
                last_sent_flags = flags;
                last_sent_event_seq = head_fall.sequence;
                last_sent_session_id = BleComm_GetSessionId();
                last_sent_state = s_glasses_mode;
                last_sent_profile = profile;
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
        if (BleComm_Core1_Task())
        {
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
    Hal_I2c_Init(GLASSES_I2C_IMU_PORT, GLASSES_PIN_I2C1_SDA, GLASSES_PIN_I2C1_SCL, 400000U);
    Hal_Uart_Init(GLASSES_UART_RADAR_PORT, GLASSES_PIN_UART0_TX, GLASSES_PIN_UART0_RX, 256000U);
    Hal_Gpio_Init(GLASSES_PIN_BUZZER_PWM, HAL_GPIO_DIR_OUT, false);
    Hal_Gpio_Init(GLASSES_PIN_STATUS_LED, HAL_GPIO_DIR_OUT, false);
    Hal_Gpio_Init(GLASSES_PIN_ACK_BTN, HAL_GPIO_DIR_IN, true);

    Buzzer_Init(&Buzzer_Config_Glasses);
    (void)Ld2450_Init(&Ld2450_Config_Glasses); /* Task-level fault/retry handles failure. */
    BleComm_Init(&BleComm_Config_Glasses);

    FallDetection_Init(&FallConfig_Head);
    ThreatAssessment_Init();
    AlertManager_Init();
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_GLASSES);
    Watchdog_Init(&Watchdog_Config_Default);

    printf("\n==================================================\n");
    printf("  EchoGuard Glasses FreeRTOS Dual-Core System     \n");
    printf("==================================================\n");

    /* Create FreeRTOS Tasks with Explicit Core Affinity */
    TaskHandle_t xGlassesPerceptionHandle = NULL;
    TaskHandle_t xGlassesRadioHandle = NULL;
    TaskHandle_t xDiagnosticsHandle = NULL;

    /* Task 1: Glasses Perception Task -> Pinned to Core 0 (Priority 3 - High) */
    BaseType_t created = xTaskCreate(vTask_GlassesPerception, "GlassesPerception", 1024U, NULL, 3U,
                                    &xGlassesPerceptionHandle);
    configASSERT(created == pdPASS);
    vTaskCoreAffinitySet(xGlassesPerceptionHandle, (1U << 0));

    /* Task 2: Dedicated BLE Radio Task -> Pinned to Core 1 (Priority 1 - Standard) */
    created = xTaskCreate(vTask_GlassesBleRadioServer, "GlassesRadio", 1024U, NULL, 1U, &xGlassesRadioHandle);
    configASSERT(created == pdPASS);
    vTaskCoreAffinitySet(xGlassesRadioHandle, (1U << 1));

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
