#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
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
#include "app/FallDetection/CaneMotion.h"
#include "app/GroundProjection/GroundBeam.h"
#include "app/GroundProjection/GroundProjection.h"
#include "app/GroundProjection/GroundProjection_Cfg.h"
#include "app/AlertManager/AlertManager.h"
#include "app/CaneMode/CaneMode.h"
#include "app/CaneMode/CaneMode_Cfg.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
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
#define CANE_BEAM_ALIGN_SAMPLES   (50U)
#define CANE_BEAM_ALIGN_TIMEOUT_MS (5000U)
#define CANE_GROUND_LOG_SETTLE_MS (300U)

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

typedef enum
{
    GROUND_CAL_FEEDBACK_NONE = 0U,
    GROUND_CAL_FEEDBACK_TONE,
    GROUND_CAL_FEEDBACK_TONE_AND_HAPTIC
} Ground_CalFeedback_t;

static Ground_CalFeedback_t s_cal_feedback = GROUND_CAL_FEEDBACK_NONE;

/* While gripped, the motor is reserved for uneven ground (step down / step up).
 * Link loss, sensor faults and calibration prompts are not felt in the handle.
 */
static Alert_Type_t Ground_HapticCue(const Gp_Detector_t * const p_d)
{
    if (!p_d->is_armed || !p_d->is_calibrated)
    {
        return ALERT_NONE;
    }
    switch (p_d->state)
    {
        case GP_STATE_STEP_DOWN:
        /* No return within range over calibrated ground = drop deeper than the ToF can see. */
        case GP_STATE_NO_GROUND:
            return ALERT_CLIFF_WARNING;
        case GP_STATE_STEP_UP: /* raised ground or an obstacle under the beam */
            return ALERT_CLIFF_STEP_UP;
        default:
            return ALERT_NONE;
    }
}

static void Ground_StartCalibration(uint32_t now_ms, bool grip_held, const char * p_source,
                                    Ground_CalFeedback_t feedback)
{
    if (!grip_held)
    {
        printf("[GROUND] Grip the cane before calibrating (%s).\n", p_source);
        return;
    }

    GroundProjection_StartCalibration(&s_ground_detector, now_ms);
    s_cal_feedback = feedback;
    if (feedback != GROUND_CAL_FEEDBACK_NONE)
    {
        AlertManager_PlayCalibrationCue(ALERT_CAL_STARTED, now_ms);
    }
    if (feedback == GROUND_CAL_FEEDBACK_TONE_AND_HAPTIC)
    {
        Haptic_PlayOnce(ALERT_CAL_STARTED, now_ms);
    }
    printf("[GROUND] Calibrating distance and tilt (%s): hold the cane still on level ground...\n", p_source);
}

#ifdef PICO_ON_DEVICE
static Gp_CalEvent_t Ground_ReportCalibration(uint32_t now_ms)
{
    Gp_CalEvent_t event = GroundProjection_TakeCalEvent(&s_ground_detector);
    if (event == GP_CAL_EVENT_NONE)
    {
        return event;
    }

    bool tilt_saved = (event == GP_CAL_EVENT_DONE) && s_ground_detector.cal_tilt_saved;
    if (event == GP_CAL_EVENT_DONE)
    {
        const char * tilt_status = "unavailable";
        if (tilt_saved)
        {
            tilt_status = "set to 0.0 deg";
        }
        else if (s_ground_detector.is_tilt_calibrated)
        {
            tilt_status = "previous reference kept";
        }
        printf("[GROUND] Calibration complete. Baseline: %u mm, IMU zero: %s\n",
               (unsigned int)s_ground_detector.baseline_mm, tilt_status);
    }
    else
    {
        printf("[GROUND] Calibration ended: %s\n", GroundProjection_GetCalEventName(event));
    }

    if (s_cal_feedback != GROUND_CAL_FEEDBACK_NONE)
    {
        Alert_Type_t cue = tilt_saved ? ALERT_CAL_DONE : ALERT_CAL_FAILED;
        AlertManager_PlayCalibrationCue(cue, now_ms);
        if ((s_cal_feedback == GROUND_CAL_FEEDBACK_TONE_AND_HAPTIC) &&
            (event != GP_CAL_EVENT_CANCELLED))
        {
            Haptic_PlayOnce(cue, now_ms);
        }
    }
    s_cal_feedback = GROUND_CAL_FEEDBACK_NONE;
    return event;
}

static void Ground_SetArmed(bool armed, uint32_t now_ms)
{
    if (!armed && s_ground_detector.is_armed)
    {
        GroundProjection_CancelCalibration(&s_ground_detector, now_ms);
        (void)Ground_ReportCalibration(now_ms);
    }
    GroundProjection_SetArmed(&s_ground_detector, armed, now_ms);
}

static const Imu_Sample_t * Ground_GetStillImuSample(const ImuSupervisor_t * const p_imu)
{
    if (!p_imu->sample_valid)
    {
        return NULL;
    }

    const Imu_Sample_t * const p_s = &p_imu->sample;
    float a_sq = (p_s->ax * p_s->ax) + (p_s->ay * p_s->ay) + (p_s->az * p_s->az);
    float g_sq = (p_s->gx * p_s->gx) + (p_s->gy * p_s->gy) + (p_s->gz * p_s->gz);
    if ((a_sq >= CANEMODE_STILL_ACC_MIN_SQ) && (a_sq <= CANEMODE_STILL_ACC_MAX_SQ) &&
        (g_sq <= CANEMODE_STILL_GYRO_MAX_SQ))
    {
        return p_s;
    }
    return NULL;
}

typedef struct
{
    bool active;
    uint32_t start_ms;
    uint32_t count;
    Imu_Sample_t sum;
} BeamCapture_t;

static void Ground_StartBeamCapture(BeamCapture_t * const p_capture, uint32_t now_ms)
{
    GroundProjection_CancelCalibration(&s_ground_detector, now_ms);
    (void)Ground_ReportCalibration(now_ms);
    *p_capture = (BeamCapture_t){ .active = true, .start_ms = now_ms };
    printf("[GROUND] Aligning ToF beam: keep it pointing straight down and hold still.\n");
}

/* Return true when a new beam alignment replaces the previous one. */
static bool Ground_UpdateBeamCapture(BeamCapture_t * const p_capture, GroundBeam_t * const p_beam,
                                     const ImuSupervisor_t * const p_imu, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - p_capture->start_ms) >= CANE_BEAM_ALIGN_TIMEOUT_MS)
    {
        p_capture->active = false;
        printf("[GROUND] Beam alignment timed out. Hold the beam straight down and send v again.\n");
        return false;
    }

    const Imu_Sample_t * const p_sample = Ground_GetStillImuSample(p_imu);
    if (p_sample == NULL)
    {
        p_capture->sum = (Imu_Sample_t){0};
        p_capture->count = 0U;
        return false;
    }
    p_capture->sum.ax += p_sample->ax;
    p_capture->sum.ay += p_sample->ay;
    p_capture->sum.az += p_sample->az;
    ++p_capture->count;
    if (p_capture->count < CANE_BEAM_ALIGN_SAMPLES)
    {
        return false;
    }

    p_capture->active = false;
    const float count = (float)p_capture->count;
    Imu_Sample_t mean = {
        .ax = p_capture->sum.ax / count,
        .ay = p_capture->sum.ay / count,
        .az = p_capture->sum.az / count
    };
    if (!GroundBeam_Align(p_beam, &mean))
    {
        printf("[GROUND] Beam alignment rejected. Hold the beam straight down and send v again.\n");
        return false;
    }

    printf("[GROUND] Beam aligned (X=%.5f Y=%.5f Z=%.5f).\n",
           (double)mean.ax, (double)mean.ay, (double)mean.az);
    printf("[GROUND] Hold at your walking angle on level ground and send c to calibrate distance.\n");
    return true;
}

static void Log_GroundSample(Gp_State_t state, bool state_changed, uint16_t raw_mm,
                             uint16_t vertical_mm, GroundBeam_Status_t beam_status,
                             float beam_angle_deg)
{
    char raw[16] = "N/A";
    char vertical[16] = "N/A";
    char beam[20] = "pose unavailable";
    if (raw_mm != UINT16_MAX)
    {
        (void)snprintf(raw, sizeof(raw), "%u mm", (unsigned int)raw_mm);
    }
    if (vertical_mm != UINT16_MAX)
    {
        (void)snprintf(vertical, sizeof(vertical), "%u mm", (unsigned int)vertical_mm);
    }
    if (isfinite(beam_angle_deg))
    {
        (void)snprintf(beam, sizeof(beam), "%.1f deg", (double)beam_angle_deg);
    }
    const char * const beam_text = (beam_status == GROUND_BEAM_UNALIGNED) ? "unaligned" : beam;
    printf("[CLIFF] %s%s (Vertical Distance: %s, Beam from vertical: %s, Raw Distance: %s)\n",
           state_changed ? "State -> " : "Sample ", GroundProjection_GetStateName(state),
           vertical, beam_text, raw);
}

/* The safety task queues snapshots without waiting for USB or formatting. */
#define CANE_MOTION_LOG_PERIOD_MS (50U)
#define CANE_MOTION_LOG_DEPTH (16U)

typedef struct
{
    uint32_t now_ms;
    Grip_State_t grip;
    bool enabled;
    bool sample_valid;
    float ax;
    float ay;
    float az;
    CaneMotion_Result_t motion;
} Motion_Log_t;

static StaticQueue_t s_motion_log_queue;
static uint8_t s_motion_log_buffer[CANE_MOTION_LOG_DEPTH * sizeof(Motion_Log_t)];
static QueueHandle_t s_motion_log;

static void Drain_Motion_Log(void)
{
    static const char * const event_names[] = {"-", "LIFT", "SLIP", "DROP", "FALL"};
    static const char * const state_names[] = {
        "IDLE", "LOW_G", "WAIT_IMPACT", "SLIP_START", "SLIP_TRACK", "CHECK_TILT"
    };
    Motion_Log_t entry;
    if ((xQueueReceive(s_motion_log, &entry, 0U) != pdPASS) || !stdio_usb_connected())
    {
        return;
    }
    const CaneMotion_Result_t * const p_motion = &entry.motion;
    bool measured = entry.enabled && entry.sample_valid;
    const char * const grip = (entry.grip == GRIP_STATE_HELD) ? "held" :
                              ((entry.grip == GRIP_STATE_RELEASED) ? "released" : "unknown");
    printf("[MOTION] t=%lu ms event=%s state=%s grip=%s enabled=%u imu=%u up_valid=%u "
           "accel_g=(%.2f,%.2f,%.2f) norm_g=%.2f vert_g=%+.2f vert_raw_g=%+.2f "
           "horiz_g=%.2f horiz_raw_g=%.2f gyro_dps=%.1f\n",
           (unsigned long)entry.now_ms, event_names[p_motion->event], state_names[p_motion->state],
           grip, (unsigned int)entry.enabled, (unsigned int)entry.sample_valid,
           (unsigned int)p_motion->up_valid, (double)entry.ax, (double)entry.ay, (double)entry.az,
           measured ? (double)p_motion->accel_g : NAN,
           p_motion->up_valid ? (double)p_motion->vertical_g : NAN,
           p_motion->up_valid ? (double)p_motion->vertical_raw_g : NAN,
           p_motion->up_valid ? (double)p_motion->horizontal_g : NAN,
           p_motion->up_valid ? (double)p_motion->horizontal_raw_g : NAN,
           measured ? (double)p_motion->rotation_dps : NAN);
}

static Diagnostics_Config_t s_cane_diagnostics = { .drain_log = Drain_Motion_Log };
static const CaneMotion_AlarmPolicy_t s_motion_policy = FEATURE_CANE_SLIP_ONLY_TRIAL ?
    CANE_MOTION_ALARM_SLIP_ONLY : CANE_MOTION_ALARM_FULL;

/*
 * =========================================================================
 * CORE 0: TASK 1 - Safety & Perception Task (Priority: High / 10ms Period)
 * Handles System Mode Management (Sys_State_t), Fall Detection, ToF, Touch
 * =========================================================================
 */
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
    GroundBeam_t ground_beam;
    GroundBeam_Init(&ground_beam);
    const Imu_Sample_t mounted_beam = {
        .ax = CANE_TOF_DOWN_ACCEL_X,
        .ay = CANE_TOF_DOWN_ACCEL_Y,
        .az = CANE_TOF_DOWN_ACCEL_Z
    };
    if (!GroundBeam_Align(&ground_beam, &mounted_beam))
    {
        printf("[GROUND] Using raw range. Aim the ToF beam straight down and send v to enable projection.\n");
    }
    BeamCapture_t beam_capture = {0};
    bool log_ground_samples = false;
    CaneMode_t mode;
    CaneMode_Init(&mode, start_ms);
    /* Standby would sleep the IMU; a released cane must always detect free fall. */
    CaneMode_SetStandbyEnabled(&mode, false);
    /* Picking the cane back up clears the alarm (no 15 s pause) and resumes detection. */
    CaneMode_SetGripClearsAlarm(&mode, true);
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
    CaneMotion_t motion = {0};
    CaneMotion_State_t prev_motion_state = CANE_MOTION_IDLE;
    bool prev_up_valid = false;
    bool log_motion_samples = false;
    uint32_t last_motion_log_ms = start_ms;

    Grip_State_t prev_grip = GRIP_STATE_RELEASED;
    uint16_t last_dist_mm = 0xFFFFU;
    uint16_t last_ground_geometry = BLE_TTC_NO_THREAT;
    uint32_t last_ground_sample_ms = start_ms;
    Gp_State_t previous_ground_state = GP_STATE_DISARMED;
    Gp_State_t logged_ground_state = GP_STATE_DISARMED;
    uint32_t ground_state_since_ms = start_ms;
    BleComm_PeerFall_t peer_fall = {0};
    bool prev_link_lost = false;
    uint8_t last_sent_flags = 0xFFU;
    uint8_t last_sent_event_seq = UINT8_MAX;
    uint16_t last_sent_session_id = 0U;
    Sys_State_t last_sent_state = SYS_STATE_COUNT;
    BleComm_ProfileId_t last_sent_profile = BLECOMM_PROFILE_FAST;
    uint16_t buzzer_freq_hz = 0U;
    uint8_t buzzer_duty_percent = 0U;
    bool peer_obstacle_active = false;
    uint16_t peer_obstacle_dist_mm = 0xFFFFU;
    bool peer_sensor_fault = false;
    /* Who raised the latched fall: the cane buzzer only sounds cane-detected falls.
     * Defaults to true so a retained or unknown-source incident still sounds locally.
     */
    bool cane_fall_is_local = true;
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
                peer_obstacle_active = ((rx_pkt.state_flags & BLE_FLAG_OBSTACLE_ACTIVE) != 0U);
                peer_obstacle_dist_mm = rx_pkt.nearest_dist_mm;
                peer_sensor_fault = ((rx_pkt.state_flags & BLE_FLAG_SENSOR_FAULT) != 0U);
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
        if (beam_capture.active && Ground_UpdateBeamCapture(&beam_capture, &ground_beam, &imu, now_ms))
        {
            /* A raw or previously projected distance baseline cannot be reused. */
            GroundProjection_Init(&s_ground_detector, now_ms);
            Haptic_Request(HAPTIC_SRC_GROUND, ALERT_NONE);
            last_ground_geometry = BLE_TTC_NO_THREAT;
        }
        const Imu_Sample_t * const p_motion_sample =
            (imu.sample_valid && mode_out.fall_detection_enabled) ? &imu.sample : NULL;
        CaneMotion_Result_t motion_result = CaneMotion_Step(&motion, p_motion_sample,
                                                           current_grip, s_motion_policy);
        bool local_fall = (motion_result.event == CANE_MOTION_EVENT_DROP) ||
                          (motion_result.event == CANE_MOTION_EVENT_FALL);
        if ((motion_result.event != CANE_MOTION_EVENT_NONE) ||
            (motion_result.state != prev_motion_state) || (motion_result.up_valid != prev_up_valid) ||
            (log_motion_samples && ((now_ms - last_motion_log_ms) >= CANE_MOTION_LOG_PERIOD_MS)))
        {
            const Motion_Log_t entry = {
                .now_ms = now_ms,
                .grip = current_grip,
                .enabled = mode_out.fall_detection_enabled,
                .sample_valid = imu.sample_valid,
                .ax = imu.sample_valid ? imu.sample.ax : NAN,
                .ay = imu.sample_valid ? imu.sample.ay : NAN,
                .az = imu.sample_valid ? imu.sample.az : NAN,
                .motion = motion_result
            };
            (void)xQueueSend(s_motion_log, &entry, 0U);
            last_motion_log_ms = now_ms;
        }
        prev_motion_state = motion_result.state;
        prev_up_valid = motion_result.up_valid;
        if (local_fall)
        {
            cane_fall_is_local = true;
        }
        else if (remote_fall && !mode_out.fall_latched)
        {
            /* A new incident raised only by the glasses: the glasses buzzer owns it. */
            cane_fall_is_local = false;
        }
        if (local_fall)
        {
            cane_status.fall_seq = (uint16_t)(BleComm_NextFallSequence(
                (uint8_t)cane_status.fall_seq) & BLE_FALL_SEQ_MASK);
            IncidentRetention_Store(true, BleComm_GetSessionId(), (uint8_t)cane_status.fall_seq);
        }

        bool ground_enabled = mode_out.ground_enabled &&
                              (current_grip == GRIP_STATE_HELD) && !local_fall && !remote_fall;
        Ground_SetArmed(ground_enabled, now_ms);
        if (!ground_enabled)
        {
            previous_ground_state = GP_STATE_DISARMED;
            logged_ground_state = GP_STATE_DISARMED;
            last_dist_mm = UINT16_MAX;
            last_ground_geometry = BLE_TTC_NO_THREAT;
        }

        /* Track rotation at the IMU rate, including cycles with no ToF read. */
        float beam_cosine;
        GroundBeam_Status_t beam_status = GroundBeam_Update(&ground_beam,
            imu.sample_valid ? &imu.sample : NULL, &beam_cosine);

        /* Sub-Task 1D: Ground ToF Cliff Detection (10 Hz / Every 100ms) */
        if (((now_ms - last_tof_ms) >= 100U) && ground_enabled)
        {
            last_tof_ms = now_ms;
            uint16_t dist_mm = 0U;
            uint16_t ground_range_mm = 0U;
            Gp_Quality_t quality = GP_RANGE_INVALID;
            bool read_ok = false;

            if (s_is_l0x)
            {
                read_ok = Vl53l0x_ReadDistanceMm(&Vl53l0x_Config_Cane, &dist_mm);
            }
            else if (s_is_l1x)
            {
                read_ok = Vl53l1x_ReadDistanceMm(&Vl53l1x_Config_Cane, &dist_mm);
            }

            if (read_ok && (dist_mm >= GP_MIN_RANGE_MM) && (dist_mm != 0xFFFFU))
            {
                if (cane_status.tof_obstructed != 0U)
                {
                    printf("[TOF] Close object cleared (Distance: %u mm)\n", dist_mm);
                    cane_status.tof_obstructed = 0U;
                }
                ground_range_mm = dist_mm;
                quality = GP_RANGE_VALID;
            }
            else if (read_ok && (dist_mm < GP_MIN_RANGE_MM))
            {
                /* Anything nearer than the classifier minimum (incl. <= 20 mm "obstructed")
                 * is an object right under the beam: classify it as the closest valid
                 * range so it votes STEP_UP (vibration), never INVALID -> SENSOR_FAULT.
                 */
                if (cane_status.tof_obstructed == 0U)
                {
                    cane_status.tof_obstructed = 1U;
                    printf("[TOF] Object very close (%u mm) -> obstacle\n", dist_mm);
                }
                ground_range_mm = GP_MIN_RANGE_MM;
                quality = GP_RANGE_VALID;
            }
            else if (read_ok && dist_mm == 0xFFFFU)
            {
                cane_status.tof_obstructed = 0U;
                quality = GP_RANGE_NO_RETURN;
            }
            else
            {
                cane_status.tof_obstructed = 0U;
            }

            if ((quality == GP_RANGE_VALID) && (ground_range_mm > GP_MAX_RANGE_MM))
            {
                quality = GP_RANGE_INVALID;
            }
            uint32_t range_ms = GetCurrentTimeMs();
            const Imu_Sample_t * p_range_imu = NULL;
            if (imu.sample_valid &&
                ((uint32_t)(range_ms - imu.sample.timestamp_ms) <= CANEMODE_MAX_SAMPLE_AGE_MS))
            {
                p_range_imu = &imu.sample;
            }
            if ((p_range_imu == NULL) && ground_beam.aligned)
            {
                beam_status = GROUND_BEAM_IMU_UNAVAILABLE;
                beam_cosine = NAN;
            }
            uint16_t vertical_mm = UINT16_MAX;
            if ((quality != GP_RANGE_INVALID) && (beam_capture.active ||
                ((beam_status != GROUND_BEAM_UNALIGNED) && (beam_status != GROUND_BEAM_AVAILABLE))))
            {
                quality = GP_RANGE_POSE_UNAVAILABLE;
            }
            else if ((quality == GP_RANGE_VALID) && (beam_status == GROUND_BEAM_AVAILABLE))
            {
                vertical_mm = GroundBeam_Project(ground_range_mm, beam_cosine);
                ground_range_mm = vertical_mm;
            }

            const float radians_to_degrees = 180.0f / 3.14159265f;
            float beam_angle_deg = isfinite(beam_cosine) ? acosf(beam_cosine) * radians_to_degrees : NAN;
            /* Keep range and beam geometry from the same poll. The phone uses
             * the projection-valid bit to mirror the firmware's pose gate. */
            last_ground_sample_ms = range_ms;
            last_dist_mm = read_ok ? dist_mm : UINT16_MAX;
            last_ground_geometry = BLE_TTC_NO_THREAT;
            if (!beam_capture.active && isfinite(beam_angle_deg))
            {
                uint16_t steps = (uint16_t)lroundf(beam_angle_deg * 5.0f); /* 0.2 degrees per step */
                last_ground_geometry = BleComm_PackCaneGeometry(steps, vertical_mm != UINT16_MAX);
            }

            const Imu_Sample_t * p_cal_imu = NULL;
            if ((quality == GP_RANGE_VALID) && s_ground_detector.is_calibrating && (p_range_imu != NULL))
            {
                p_cal_imu = Ground_GetStillImuSample(&imu);
            }
            GroundProjection_FeedProjected(&s_ground_detector, quality, ground_range_mm, range_ms, p_cal_imu);

            if ((read_ok && (dist_mm != 0xFFFFU)) || (quality == GP_RANGE_POSE_UNAVAILABLE))
            {
                last_tof_alive_ms = now_ms;
            }

            /* ToF Recovery Supervisor */
            if (((now_ms - last_tof_alive_ms) >= CANE_TOF_RECOVERY_MS) &&
                ((now_ms - last_tof_reinit_ms) >= CANE_TOF_RECOVERY_MS) &&
                !motion_result.pending)
            {
                last_tof_reinit_ms = now_ms;
                (void)Tof_Reinit();
            }

            Gp_State_t gstate = s_ground_detector.state;
            if (gstate != previous_ground_state)
            {
                previous_ground_state = gstate;
                ground_state_since_ms = range_ms;
            }
            /* Brief gaps can alternate these two states during a swing. Log real alerts
             * immediately and keep the full sample stream available for debugging.
             */
            bool waiting_for_ground = (gstate == GP_STATE_UNKNOWN) || (gstate == GP_STATE_POSE_UNAVAILABLE);
            bool report_state_change = (gstate != logged_ground_state) &&
                (log_ground_samples || !waiting_for_ground ||
                 ((uint32_t)(range_ms - ground_state_since_ms) >= CANE_GROUND_LOG_SETTLE_MS));
            if (log_ground_samples || report_state_change)
            {
                Log_GroundSample(gstate, report_state_change, read_ok ? dist_mm : UINT16_MAX,
                                 vertical_mm, beam_status, beam_angle_deg);
                logged_ground_state = gstate;
            }

            /* Close objects and no-ground vibrate; stale/missing ToF stays visible via
             * serial, the BLE sensor-fault flag and FAULT_DEGRADED mode (no vibration).
             */
            Haptic_Request(HAPTIC_SRC_GROUND, Ground_HapticCue(&s_ground_detector));
        }

        /* Recovery I/O can take longer than one period; stale IMU data must not
         * qualify a parking or upright-recovery transition after that delay.
         */
        now_ms = GetCurrentTimeMs();
        GroundProjection_Tick(&s_ground_detector, now_ms);
        Gp_CalEvent_t cal_event = Ground_ReportCalibration(now_ms);
        if ((cal_event == GP_CAL_EVENT_DONE) && !ground_beam.aligned)
        {
            printf("[GROUND] Raw baseline saved. For vertical distance, aim the ToF straight down and send v.\n");
        }
        /* A close object is an obstacle (vibration), not a technical fault. */
        bool ground_fault = (!s_is_l0x && !s_is_l1x) ||
                            (s_ground_detector.state == GP_STATE_SENSOR_FAULT);
        /* Retry without feedback while gripped if there is still no distance baseline. */
        if ((cal_event != GP_CAL_EVENT_NONE) && (cal_event != GP_CAL_EVENT_DONE) &&
            !s_ground_detector.is_calibrated && s_ground_detector.is_armed && !beam_capture.active)
        {
            Ground_StartCalibration(now_ms, true, "retry", GROUND_CAL_FEEDBACK_NONE);
        }
        CaneMode_Input_t mode_in = {
            .grip = current_grip,
            .imu_ready = (imu.state == IMU_SUPERVISOR_RUNNING),
            .imu_sleeping = (imu.state == IMU_SUPERVISOR_ASLEEP),
            .imu_fault = imu.fault,
            .sample_valid = imu.sample_valid,
            .sample = imu.sample,
            .local_fall = local_fall,
            .peer_fall = remote_fall,
            .fall_pending = motion_result.pending,
            .ground_hazard = (s_ground_detector.state == GP_STATE_STEP_DOWN) ||
                             (s_ground_detector.state == GP_STATE_STEP_UP),
            .technical_fault = ground_fault
        };
        CaneMode_Step(&mode, &mode_in, now_ms, &mode_out);
        s_sys_mode = mode_out.state;
        cane_status.fall_latched = mode_out.fall_latched ? 1U : 0U;
        IncidentRetention_Store(mode_out.fall_latched, BleComm_GetSessionId(),
                                (uint8_t)cane_status.fall_seq);
        cane_status.imu_healthy = ((imu.state == IMU_SUPERVISOR_RUNNING) ||
                                  (imu.state == IMU_SUPERVISOR_ASLEEP)) ? 1U : 0U;
        if (mode_out.reset_fall_detection)
        {
            motion = (CaneMotion_t){0};
        }
        bool ground_was_armed = s_ground_detector.is_armed;
        Ground_SetArmed(mode_out.ground_enabled, now_ms);
        if (mode_out.ground_enabled && !ground_was_armed &&
            !s_ground_detector.is_calibrated && !beam_capture.active)
        {
            Ground_StartCalibration(now_ms, true, "first grip", GROUND_CAL_FEEDBACK_TONE);
        }
        Haptic_SetGripHeld(mode_out.ground_enabled);
        if (!mode_out.ground_enabled)
        {
            Haptic_Request(HAPTIC_SRC_GROUND, ALERT_NONE);
        }

        bool is_link_lost = Watchdog_CheckBleLinkLoss(now_ms);
        if (is_link_lost)
        {
            peer_obstacle_active = false;
            peer_obstacle_dist_mm = 0xFFFFU;
            peer_sensor_fault = false;
        }
        if (is_link_lost != prev_link_lost)
        {
            prev_link_lost = is_link_lost;
            cane_status.link_lost = is_link_lost ? 1U : 0U;
            if (is_link_lost)
            {
                printf("[WATCHDOG] BLE LINK LOSS DETECTED (>%u ms)\n", (unsigned int)WATCHDOG_CFG_DEADMAN_TIMEOUT_MS);
            }
        }
        /* Link loss is reported over serial and BLE status, not through the grip motor. */
        Haptic_Request(HAPTIC_SRC_LINK, ALERT_NONE);

        if (!mode_out.fall_latched)
        {
            cane_fall_is_local = true;
        }
        /* The arbitrator renders the alarm or secondary cues using synced dual-node rules. */
        Alert_SyncInput_t sync_in = {
            .is_glasses_node = false,
            .local_fall_latched = mode_out.fall_latched && cane_fall_is_local,
            .local_alarm_paused = mode_out.alarm_paused,
            .peer_fall_active = mode_out.fall_latched && !cane_fall_is_local,
            .delivery_fallback = false,
            .cane_link_fresh = !is_link_lost,
            .cane_buzzer_fault = !Buzzer_IsHealthy(),
            .obstacle_active = false,
            .obstacle_dist_mm = 0xFFFFU,
            .sensor_fault = (mode.power != CANE_POWER_STANDBY) &&
                            ((mode_out.status_flags & BLE_FLAG_SENSOR_FAULT) != 0U),
            .peer_sensor_fault = (mode.power != CANE_POWER_STANDBY) && peer_sensor_fault
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
        if (remote_fall && peer_event_identity_valid && mode_out.fall_latched && Buzzer_IsHealthy())
        {
            /* Receipt is emitted only after the mode manager latched the incident,
             * the buzzer is healthy, and the alarm request was rendered by the application path above.
             */
            peer_ack_valid = true;
            peer_ack_session_id = peer_event_session_id;
            peer_ack_event_seq = peer_event_seq;
        }
        if (!Buzzer_IsHealthy() || !peer_fall.extended || !peer_fall.active ||
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
            bool button_calibration = (CalButton_Update(Hal_Gpio_Read(CANE_PIN_CAL_BTN), now_ms) ==
                                       CALBTN_EVENT_GROUND_HOLD);
            int ch = getchar_timeout_us(0);
            if ((ch == (int)'v') || (ch == (int)'V'))
            {
                Ground_StartBeamCapture(&beam_capture, now_ms);
            }
            else if (button_calibration || (ch == (int)'c') || (ch == (int)'C'))
            {
                if (beam_capture.active)
                {
                    printf("[GROUND] Wait for beam alignment before calibrating distance.\n");
                }
                else
                {
                    Ground_StartCalibration(now_ms, mode_out.ground_enabled,
                                            button_calibration ? "button" : "serial",
                                            GROUND_CAL_FEEDBACK_TONE_AND_HAPTIC);
                }
            }
            else if ((ch == (int)'d') || (ch == (int)'D'))
            {
                log_ground_samples = !log_ground_samples;
                printf("[GROUND] Per-sample logging %s.\n", log_ground_samples ? "on" : "off");
            }
            else if ((ch == (int)'f') || (ch == (int)'F'))
            {
                log_motion_samples = !log_motion_samples;
                printf("[MOTION] 20 Hz sample logging %s. Cane alarm policy: %s.\n",
                       log_motion_samples ? "on" : "off",
                       (s_motion_policy == CANE_MOTION_ALARM_SLIP_ONLY) ? "lateral slip only" : "full");
            }
            else if ((ch == (int)'b') || (ch == (int)'B'))
            {
                bool new_fault = Buzzer_IsHealthy();
                Buzzer_InjectFault(new_fault);
                printf("[TEST TRIGGER] Cane Buzzer Fault -> %s\n", new_fault ? "FAULT" : "OK");
            }
        }

        uint8_t flags = mode_out.status_flags;
        if (mode_out.ground_enabled && mode_in.ground_hazard)
        {
            flags |= BLE_FLAG_OBSTACLE_ACTIVE;
        }
        if (!Buzzer_IsHealthy())
        {
            flags |= BLE_FLAG_BUZZER_FAULT;
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
            bool ground_sample_fresh = mode_out.ground_enabled &&
                                       ((now_ms - last_ground_sample_ms) < GP_STALE_TIMEOUT_MS);
            Ble_AdvancedThreatPacket_t cane_pkt = {
                .preamble = 0xAAU,
                .device_id = 0x01U,
                .pair_id = BLE_DEFAULT_PAIR_ID,
                .node_type = BLE_NODE_TYPE_CANE,
                .heartbeat_seq = (uint8_t)cane_status.tx_seq,
                .rolling_fall_seq = (uint8_t)cane_status.fall_seq,
                .seq_num = (uint16_t)cane_status.tx_seq,
                .state_flags = flags,
                .nearest_dist_mm = ground_sample_fresh ? last_dist_mm : UINT16_MAX,
                .closing_speed_cms = 0,
                .ttc_20ms = (ground_sample_fresh && !beam_capture.active) ?
                           last_ground_geometry : BLE_TTC_NO_THREAT,
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

    AlertManager_Init();

    uint32_t boot_ms = GetCurrentTimeMs();
    GroundProjection_Init(&s_ground_detector, boot_ms);
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    Watchdog_Init(&Watchdog_Config_Default);

    printf("\n==================================================\n");
    printf("  EchoGuard Smart Cane FreeRTOS Dual-Core System  \n");
    printf("==================================================\n");
    printf("[MOTION] Send f over USB serial to toggle direction/acceleration samples.\n");

    s_motion_log = xQueueCreateStatic(CANE_MOTION_LOG_DEPTH, sizeof(Motion_Log_t),
                                      s_motion_log_buffer, &s_motion_log_queue);
    configASSERT(s_motion_log != NULL);

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

    created = xTaskCreate(Diagnostics_Task, "Diagnostics", 768U, &s_cane_diagnostics, 1U,
                          &xDiagnosticsHandle);
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
