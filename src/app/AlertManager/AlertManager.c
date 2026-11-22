#include "AlertManager.h"
#include "AlertManager_Cfg.h"
#include <string.h>

/*
 * \file AlertManager.c
 * \brief Audio/Haptic Alert Arbitration and Distance-to-Pitch mapping
 * Compliance: Barr-C:2018
 */

static bool s_fall_alarm_latched = false;
static bool s_fall_alarm_muted = false;
static uint32_t s_fall_start_time_ms = 0U;
static Alert_Type_t s_previous_request = ALERT_NONE;
static uint32_t s_request_start_ms = 0U;
static Alert_Type_t s_calibration_cue = ALERT_NONE;
static uint32_t s_calibration_start_ms = 0U;
/* Obstacle renderer flags and the three cues, packed to the bits they need
 * (data-representation change). A cue is 3 bits: bit 0 STOP, bits 1-2 side.
 * Only the main-loop alert arbitration touches them (AlertManager_Init runs
 * before the scheduler starts; AlertManager_SetObstacleCue is called from
 * AlertManager_ArbitrateSynced in the same task), so the non-atomic
 * read-modify-write of this shared word is safe. */
typedef struct
{
    /* obstacle render state below must restart when 0 */
    uint16_t valid : 1;
    uint16_t track_valid : 1;  /* side pattern start time is valid */
    uint16_t pattern_side : 2; /* cue side 0..3 of the running pattern */
    uint16_t beep_on : 1;      /* buzzer state of the last obstacle render */
    uint16_t cue_latest : 3;   /* cue last set by AlertManager_SetObstacleCue */
    uint16_t cue_pending : 3;  /* cue waiting out ALERT_OBSTACLE_CUE_DWELL_MS */
    uint16_t cue_play : 3;     /* cue being played */
    uint16_t reserved : 2;
} Obstacle_Flags_t;

#define OBS_CUE_BITS (0x07U) /* STOP bit + 2-bit side */

_Static_assert(sizeof(Obstacle_Flags_t) == 2U,
               "Obstacle flags and cues must pack into 16 bits");
_Static_assert(((ALERT_CUE_STOP |
                 (ALERT_CUE_SIDE_MASK << ALERT_CUE_SIDE_SHIFT)) &
                ~OBS_CUE_BITS) == 0U,
               "a cue must fit 3 bits");

static Obstacle_Flags_t s_obs_flags = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

void AlertManager_Init(void)
{
    s_obs_flags.cue_latest = 0U;
    s_obs_flags.valid = 0U;
    s_fall_alarm_latched = false;
    s_fall_alarm_muted = false;
    s_fall_start_time_ms = 0U;
    s_previous_request = ALERT_NONE;
    s_request_start_ms = 0U;
    s_calibration_cue = ALERT_NONE;
    s_calibration_start_ms = 0U;
}

void AlertManager_SetObstacleCue(uint8_t cue)
{
    s_obs_flags.cue_latest = cue & OBS_CUE_BITS;
}

void AlertManager_AcknowledgeReset(void)
{
    s_fall_alarm_latched = false;
    s_fall_alarm_muted = false;
    s_fall_start_time_ms = 0U;
}

void AlertManager_SetFallState(bool latched, bool paused, uint32_t now_ms)
{
    if (latched && (!s_fall_alarm_latched || (s_fall_alarm_muted && !paused)))
    {
        s_fall_start_time_ms = now_ms;
    }
    s_fall_alarm_latched = latched;
    s_fall_alarm_muted = latched && paused;
}

void AlertManager_PlayCalibrationCue(Alert_Type_t cue, uint32_t now_ms)
{
    if ((cue == ALERT_CAL_STARTED) || (cue == ALERT_CAL_DONE) || (cue == ALERT_CAL_FAILED))
    {
        s_calibration_cue = cue;
        s_calibration_start_ms = now_ms;
    }
}

static void Render_CalibrationCue(uint32_t now_ms, Alert_Output_t * const p_out)
{
    if ((uint32_t)(now_ms - s_calibration_start_ms) >= ALERT_CALIBRATION_PULSE_MS)
    {
        s_calibration_cue = ALERT_NONE;
    }

    switch (s_calibration_cue)
    {
        case ALERT_CAL_STARTED: p_out->generated_freq_hz = ALERT_CAL_STARTED_FREQ_HZ; break;
        case ALERT_CAL_DONE:    p_out->generated_freq_hz = ALERT_CAL_DONE_FREQ_HZ; break;
        case ALERT_CAL_FAILED:  p_out->generated_freq_hz = ALERT_CAL_FAILED_FREQ_HZ; break;
        default: return;
    }
    p_out->active_alert = s_calibration_cue;
    p_out->buzzer_active = true;
    p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
}

static bool Fall_PulseOn(uint32_t elapsed_ms)
{
    static const uint16_t phases_ms[] = {
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_MEDIUM_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_LONG_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_MEDIUM_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_SHORT_GAP_MS,
        ALERT_FALL_PULSE_MS, ALERT_FALL_BURST_GAP_MS
    };
    uint32_t t = elapsed_ms % ALERT_FALL_PERIOD_MS;
    for (uint32_t i = 0U; i < (sizeof(phases_ms) / sizeof(phases_ms[0])); ++i)
    {
        if (t < phases_ms[i])
        {
            return ((i % 2U) == 0U);
        }
        t -= phases_ms[i];
    }
    return false;
}

/*
 * FR9 obstacle beep pattern (design B, putra-branch) with steadiness handling.
 *  - CENTER: a burst of 1 beep.
 *  - LEFT / RIGHT: a burst of 2 / 3 beeps (150 ms on, 150 ms off), then a pause
 *    that shrinks linearly from 450 ms at 3000 mm to 150 ms at 300 mm.
 *  - NO FREE LANE: 4 short beeps; the pause stretches if it keeps playing (crowds).
 *  - STOP (about to collide): continuous tone at the top pitch for as long as STOP lasts.
 * While walking, the cue and the distance change constantly, so:
 *  - the rendered cue only follows the requested cue after it has been stable
 *    for ALERT_OBSTACLE_CUE_DWELL_MS (a rising STOP applies at once);
 *  - the distance used for pitch and pause is low-passed (tau 250 ms);
 *  - the pause is latched when a burst cycle starts, so the rhythm cannot
 *    shift while the distance changes.
 */
#define OBS_DIST_Q (4U) /* smoothed distance kept in 1/16 mm to avoid rounding stalls */

static uint32_t s_obs_pending_since_ms = 0U;
static uint32_t s_obs_last_ms = 0U;
static uint32_t s_obs_dist_q = 0U;
static uint32_t s_obs_cycle_start_ms = 0U;
static uint32_t s_obs_cycle_pause_ms = 0U;
static uint32_t s_obs_pattern_since_ms = 0U;
static uint32_t s_obs_last_off_ms = 0U; /* when the last obstacle beep ended */

static uint32_t Obstacle_SmoothedDistMm(void)
{
    return s_obs_dist_q >> OBS_DIST_Q;
}

static bool Obstacle_StopActive(void)
{
    return ((s_obs_flags.cue_play & ALERT_CUE_STOP) != 0U);
}

static uint32_t Obstacle_PauseMs(uint32_t distance_mm)
{
    uint32_t d = distance_mm;
    if (d < ALERT_OBSTACLE_MIN_DIST_MM)
    {
        d = ALERT_OBSTACLE_MIN_DIST_MM;
    }
    if (d > ALERT_OBSTACLE_MAX_DIST_MM)
    {
        d = ALERT_OBSTACLE_MAX_DIST_MM;
    }

    return ALERT_OBSTACLE_PAUSE_MAX_MS -
           (((ALERT_OBSTACLE_MAX_DIST_MM - d) *
             (ALERT_OBSTACLE_PAUSE_MAX_MS - ALERT_OBSTACLE_PAUSE_MIN_MS)) /
            (ALERT_OBSTACLE_MAX_DIST_MM - ALERT_OBSTACLE_MIN_DIST_MM));
}

static uint32_t Obstacle_Side(void)
{
    return ((uint32_t)s_obs_flags.cue_play >> ALERT_CUE_SIDE_SHIFT) &
           ALERT_CUE_SIDE_MASK;
}

/* Remember when the current side pattern and the STOP tone began. */
static void Obstacle_TrackPattern(uint32_t now_ms)
{
    const uint32_t side = Obstacle_Side();
    if ((s_obs_flags.track_valid == 0U) || (side != (uint32_t)s_obs_flags.pattern_side))
    {
        s_obs_flags.track_valid = 1U;
        s_obs_flags.pattern_side = (uint8_t)side;
        s_obs_pattern_since_ms = now_ms;
    }
}

/* Pause for the next burst: distance based, stretched if "no free lane" keeps repeating. */
static uint32_t Obstacle_CyclePauseMs(uint32_t now_ms)
{
    uint32_t pause_ms = Obstacle_PauseMs(Obstacle_SmoothedDistMm());

    /* Multi-beep patterns are counted, so the pause must stay clearly longer
     * than the gap inside the burst. Only bites below about 1.5 m, where the
     * centre lane is already in STOP, so it affects side-obstacle combinations
     * only. */
    if ((Obstacle_Side() != ALERT_CUE_SIDE_CENTER) &&
        (pause_ms < ALERT_OBSTACLE_MULTI_PAUSE_MIN_MS))
    {
        pause_ms = ALERT_OBSTACLE_MULTI_PAUSE_MIN_MS;
    }

    if ((Obstacle_Side() == ALERT_CUE_SIDE_ALL_BLOCKED) &&
        ((uint32_t)(now_ms - s_obs_pattern_since_ms) >= ALERT_OBSTACLE_BLOCKED_REPEAT_MS) &&
        (pause_ms < ALERT_OBSTACLE_BLOCKED_LONG_PAUSE_MS))
    {
        pause_ms = ALERT_OBSTACLE_BLOCKED_LONG_PAUSE_MS;
    }
    return pause_ms;
}

static void Obstacle_StartCycle(uint32_t now_ms)
{
    s_obs_cycle_start_ms = now_ms;
    s_obs_cycle_pause_ms = Obstacle_CyclePauseMs(now_ms);
}

/* Start the next pattern only after a clear silence, so the end of the old
 * burst cannot run into the new one and be counted as one longer burst. */
static void Obstacle_StartCycleSeparated(uint32_t now_ms)
{
    uint32_t start_ms = now_ms;
    const uint32_t since_off = (uint32_t)(now_ms - s_obs_last_off_ms);
    if (since_off < ALERT_OBSTACLE_SWITCH_GAP_MS)
    {
        start_ms = now_ms + (ALERT_OBSTACLE_SWITCH_GAP_MS - since_off);
    }
    s_obs_cycle_start_ms = start_ms;
    s_obs_cycle_pause_ms = Obstacle_CyclePauseMs(now_ms);
}

static bool Obstacle_BurstUnfinished(uint32_t now_ms);

static void Obstacle_Update(uint32_t now_ms, uint16_t distance_mm)
{
    if (s_obs_flags.valid == 0U)
    {
        /* Start of an obstacle alert: apply the cue and distance immediately. */
        s_obs_flags.valid = 1U;
        s_obs_flags.cue_play = s_obs_flags.cue_latest;
        s_obs_flags.cue_pending = s_obs_flags.cue_latest;
        s_obs_pending_since_ms = now_ms;
        s_obs_last_ms = now_ms;
        s_obs_dist_q = ((uint32_t)distance_mm) << OBS_DIST_Q;
        s_obs_flags.track_valid = 0U;
        s_obs_flags.beep_on = 0U;
        s_obs_last_off_ms = now_ms - ALERT_OBSTACLE_SWITCH_GAP_MS;
        Obstacle_TrackPattern(now_ms);
        Obstacle_StartCycle(now_ms);
        return;
    }

    /* First-order low-pass on distance: dist += (new - dist) * dt / (tau + dt). */
    uint32_t dt = (uint32_t)(now_ms - s_obs_last_ms);
    s_obs_last_ms = now_ms;
    if (dt > 1000U)
    {
        dt = 1000U;
    }
    const int32_t target_q = (int32_t)(((uint32_t)distance_mm) << OBS_DIST_Q);
    const int32_t err_q = target_q - (int32_t)s_obs_dist_q;
    s_obs_dist_q = (uint32_t)((int32_t)s_obs_dist_q +
                              ((err_q * (int32_t)dt) / (int32_t)(ALERT_OBSTACLE_PITCH_TAU_MS + dt)));

    /* Cue dwell: follow the requested cue only once it has been stable. */
    if (s_obs_flags.cue_latest != s_obs_flags.cue_pending)
    {
        s_obs_flags.cue_pending = s_obs_flags.cue_latest;
        s_obs_pending_since_ms = now_ms;
    }
    if (s_obs_flags.cue_pending != s_obs_flags.cue_play)
    {
        const bool stop_rising =
            ((s_obs_flags.cue_pending & ALERT_CUE_STOP) != 0U) &&
            ((s_obs_flags.cue_play & ALERT_CUE_STOP) == 0U);
        const bool beep_to_beep =
            ((s_obs_flags.cue_pending & ALERT_CUE_STOP) == 0U) &&
            ((s_obs_flags.cue_play & ALERT_CUE_STOP) == 0U);
        if (stop_rising)
        {
            s_obs_flags.cue_play = s_obs_flags.cue_pending;
            Obstacle_TrackPattern(now_ms);
            Obstacle_StartCycle(now_ms);
        }
        else if ((uint32_t)(now_ms - s_obs_pending_since_ms) >=
                 ALERT_OBSTACLE_CUE_DWELL_MS)
        {
            if (!beep_to_beep)
            {
                /* Leaving STOP for a beep pattern: the continuous tone ends
                 * here, then the same clear silence as any other pattern
                 * change, so the end of STOP is heard as an end and the first
                 * beep is not run onto the tone. */
                const bool stop_falling =
                    ((s_obs_flags.cue_play & ALERT_CUE_STOP) != 0U) &&
                    ((s_obs_flags.cue_pending & ALERT_CUE_STOP) == 0U);
                s_obs_flags.cue_play = s_obs_flags.cue_pending;
                Obstacle_TrackPattern(now_ms);
                if (stop_falling)
                {
                    s_obs_last_off_ms = now_ms;
                    Obstacle_StartCycleSeparated(now_ms);
                }
                else
                {
                    Obstacle_StartCycle(now_ms);
                }
            }
            else if (!Obstacle_BurstUnfinished(now_ms))
            {
                /* Beep pattern to beep pattern: let the current burst finish,
                 * then leave a clear gap before the new pattern's first
                 * beep. */
                s_obs_flags.cue_play = s_obs_flags.cue_pending;
                Obstacle_TrackPattern(now_ms);
                Obstacle_StartCycleSeparated(now_ms);
            }
            else
            {
                /* wait for the burst that is sounding to end */
            }
        }
    }
}

/* Beep count, pulse and gap of the current side pattern. */
static void Obstacle_PatternShape(uint32_t * const p_beeps,
                                  uint32_t * const p_pulse_ms,
                                  uint32_t * const p_gap_ms)
{
    const uint32_t side = Obstacle_Side();
    *p_beeps = ALERT_OBSTACLE_CENTER_BEEPS;
    *p_pulse_ms = ALERT_OBSTACLE_PULSE_MS;
    *p_gap_ms = ALERT_OBSTACLE_BEEP_GAP_MS;
    if (side == ALERT_CUE_SIDE_ALL_BLOCKED)
    {
        *p_beeps = ALERT_OBSTACLE_BLOCKED_BEEPS;
        *p_pulse_ms = ALERT_OBSTACLE_BLOCKED_PULSE_MS;
        *p_gap_ms = ALERT_OBSTACLE_BLOCKED_GAP_MS;
    }
    else if (side == ALERT_CUE_SIDE_LEFT)
    {
        *p_beeps = ALERT_OBSTACLE_LEFT_BEEPS;
    }
    else if (side == ALERT_CUE_SIDE_RIGHT)
    {
        *p_beeps = ALERT_OBSTACLE_RIGHT_BEEPS;
    }
    else
    {
        /* centre clear: one beep */
    }
}

/* True while a burst is part-way through: from its first pulse until its last
 * pulse ends. A pattern change waits for this, so the user never hears a cut
 * burst (a 3-beep "move right" cut after two beeps would be heard as "move
 * left"). */
static bool Obstacle_BurstUnfinished(uint32_t now_ms)
{
    if ((int32_t)(now_ms - s_obs_cycle_start_ms) < 0)
    {
        /* still in the separation gap: nothing of this burst sounded yet */
        return false;
    }
    uint32_t beeps = 0U;
    uint32_t pulse_ms = 0U;
    uint32_t gap_ms = 0U;
    Obstacle_PatternShape(&beeps, &pulse_ms, &gap_ms);
    const uint32_t burst_end_ms = ((pulse_ms + gap_ms) * beeps) - gap_ms;
    const uint32_t cycle_ms =
        ((pulse_ms + gap_ms) * beeps) + s_obs_cycle_pause_ms;
    const uint32_t elapsed =
        (uint32_t)(now_ms - s_obs_cycle_start_ms) % cycle_ms;
    return (s_obs_flags.beep_on != 0U) ||
           ((elapsed > 0U) && (elapsed < burst_end_ms));
}

static bool Obstacle_BeepOn(uint32_t now_ms)
{
    if (Obstacle_StopActive())
    {
        /* STOP: continuous tone for as long as STOP lasts. */
        return true;
    }

    uint32_t beeps = 0U;
    uint32_t pulse_ms = 0U;
    uint32_t gap_ms = 0U;
    Obstacle_PatternShape(&beeps, &pulse_ms, &gap_ms);
    if ((int32_t)(now_ms - s_obs_cycle_start_ms) < 0)
    {
        return false; /* separation gap before a new pattern */
    }

    const uint32_t slot_ms = pulse_ms + gap_ms;
    const uint32_t burst_ms = slot_ms * beeps;
    const uint32_t cycle_ms = burst_ms + s_obs_cycle_pause_ms;

    uint32_t elapsed = (uint32_t)(now_ms - s_obs_cycle_start_ms);
    if (elapsed >= cycle_ms)
    {
        /* New cycle: advance by whole cycles, then latch the pause for it. */
        s_obs_cycle_start_ms += cycle_ms * (elapsed / cycle_ms);
        s_obs_cycle_pause_ms = Obstacle_CyclePauseMs(now_ms);
        elapsed = (uint32_t)(now_ms - s_obs_cycle_start_ms);
    }

    return (elapsed < burst_ms) && ((elapsed % slot_ms) < pulse_ms);
}

uint16_t AlertManager_MapDistanceToPitch(uint16_t distance_mm)
{
    if (distance_mm >= ALERT_OBSTACLE_MAX_DIST_MM)
    {
        return ALERT_BUZZER_MIN_FREQ_HZ;
    }
    if (distance_mm <= ALERT_OBSTACLE_MIN_DIST_MM)
    {
        return ALERT_BUZZER_MAX_FREQ_HZ;
    }

    /* Linear interpolation between 2000mm (1500Hz) and 300mm (3500Hz) */
    uint32_t dist_span = ALERT_OBSTACLE_MAX_DIST_MM - ALERT_OBSTACLE_MIN_DIST_MM; /* 1700 */
    uint32_t freq_span = ALERT_BUZZER_MAX_FREQ_HZ - ALERT_BUZZER_MIN_FREQ_HZ;     /* 2000 */
    uint32_t offset = ALERT_OBSTACLE_MAX_DIST_MM - distance_mm;

    uint32_t pitch = ALERT_BUZZER_MIN_FREQ_HZ + ((offset * freq_span) / dist_span);
    return (uint16_t)pitch;
}

void AlertManager_Arbitrate(Alert_Type_t requested_alert,
                            uint16_t param,
                            uint32_t now_ms,
                            Alert_Output_t * const p_out)
{
    if (p_out == NULL)
    {
        return;
    }

    memset(p_out, 0, sizeof(*p_out));

    /* An obstacle that clears mid-beep lets that beep finish (at most one
     * pulse), so the last beep is never cut to a blip. Only when nothing else
     * is requested, never STOP. */
    if ((requested_alert == ALERT_NONE) &&
        (s_previous_request == ALERT_OBSTACLE_PROXIMITY) &&
        (s_obs_flags.beep_on != 0U) && !Obstacle_StopActive() &&
        !s_fall_alarm_latched)
    {
        p_out->active_alert = ALERT_OBSTACLE_PROXIMITY;
        p_out->buzzer_active = Obstacle_BeepOn(now_ms);
        if (p_out->buzzer_active)
        {
            p_out->generated_freq_hz = AlertManager_MapDistanceToPitch(
                (uint16_t)Obstacle_SmoothedDistMm());
            p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
            return;
        }
        s_obs_flags.beep_on = 0U;
        memset(p_out, 0, sizeof(*p_out));
    }

    if (requested_alert != s_previous_request)
    {
        s_previous_request = requested_alert;
        s_request_start_ms = now_ms;
        s_obs_flags.valid = 0U;
    }

    if (requested_alert == ALERT_FALL_ALARM)
    {
        if (!s_fall_alarm_latched)
        {
            s_fall_alarm_latched = true;
            s_fall_alarm_muted = false;
            s_fall_start_time_ms = now_ms;
        }
    }

    /* Output whether fall is latched regardless of acoustic mute */
    p_out->fall_latched = s_fall_alarm_latched;

    /* Priority 1: Fall Emergency */
    if (s_fall_alarm_latched)
    {
        if (s_fall_alarm_muted)
        {
            p_out->active_alert = ALERT_FALL_MUTED;
            p_out->buzzer_active = false; /* Bounded pause is owned by the mode manager. */
            p_out->haptic_active = false; /* FR4: Inhibit haptics */
        }
        else
        {
            p_out->active_alert = ALERT_FALL_ALARM;
            p_out->buzzer_active = Fall_PulseOn((uint32_t)(now_ms - s_fall_start_time_ms));
            p_out->generated_freq_hz = ALERT_FALL_FREQ_A_HZ;
            p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
            p_out->haptic_active = false; /* FR4: Inhibit haptics */
        }
        return;
    }

    /* Priority 2: Head-fall delivery fallback on the glasses. This deliberately
     * uses a sparse, low-duty cue rather than the cane's emergency siren. Bench
     * sound-pressure validation is still required for the final enclosure.
     */
    if (requested_alert == ALERT_FALL_DELIVERY_FAULT)
    {
        p_out->active_alert = ALERT_FALL_DELIVERY_FAULT;
        p_out->buzzer_active = ((uint32_t)(now_ms - s_request_start_ms) %
                                ALERT_DELIVERY_PERIOD_MS) < ALERT_DELIVERY_PULSE_MS;
        p_out->generated_freq_hz = ALERT_DELIVERY_FREQ_HZ;
        p_out->generated_duty_percent = ALERT_DELIVERY_DUTY_PERCENT;
        return;
    }

    /* Priority 3: Cliff Warning (Cane Haptic only) */
    if (requested_alert == ALERT_CLIFF_WARNING)
    {
        p_out->active_alert = ALERT_CLIFF_WARNING;
        p_out->haptic_active = true;
        p_out->buzzer_active = false;
        return;
    }

    /* Priority 4: Obstacle Proximity (FR9 direction-cue beep pattern) */
    if (requested_alert == ALERT_OBSTACLE_PROXIMITY)
    {
        p_out->active_alert = ALERT_OBSTACLE_PROXIMITY;
        Obstacle_Update(now_ms, param);
        p_out->buzzer_active = Obstacle_BeepOn(now_ms);
        if ((s_obs_flags.beep_on != 0U) && !p_out->buzzer_active)
        {
            s_obs_last_off_ms = now_ms;
        }
        s_obs_flags.beep_on = p_out->buzzer_active ? 1U : 0U;
        p_out->generated_freq_hz = Obstacle_StopActive()
                                       ? (uint16_t)ALERT_BUZZER_MAX_FREQ_HZ
                                       : AlertManager_MapDistanceToPitch((uint16_t)Obstacle_SmoothedDistMm());
        p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
        return;
    }

    /* Priority 5: Sensor Fault Tone */
    if (requested_alert == ALERT_SENSOR_FAULT)
    {
        p_out->active_alert = ALERT_SENSOR_FAULT;
        uint32_t t = (uint32_t)(now_ms - s_request_start_ms) % ALERT_FAULT_PERIOD_MS;
        p_out->buzzer_active = (t < ALERT_FAULT_PULSE_MS) ||
                              ((t >= (ALERT_FAULT_PULSE_MS + ALERT_FAULT_GAP_MS)) &&
                               (t < ((2U * ALERT_FAULT_PULSE_MS) + ALERT_FAULT_GAP_MS)));
        p_out->generated_freq_hz = ALERT_FAULT_FREQ_HZ;
        p_out->generated_duty_percent = ALERT_STANDARD_DUTY_PERCENT;
        return;
    }

    /* Priority 6: Link Loss Haptic Warning (FR16: degraded mode) */
    if (requested_alert == ALERT_LINK_LOSS)
    {
        p_out->active_alert = ALERT_LINK_LOSS;
        p_out->haptic_active = true;
        p_out->buzzer_active = false;
        return;
    }
}

void AlertManager_ArbitrateSynced(const Alert_SyncInput_t * const p_in,
                                  uint32_t now_ms,
                                  Alert_Output_t * const p_out)
{
    if ((p_in == NULL) || (p_out == NULL))
    {
        return;
    }

    if (!p_in->is_glasses_node)
    {
        /* Cane node: plays every latched fall (its own and the head fall
         * received from the glasses) and its own sensor faults.
         */
        AlertManager_SetFallState(p_in->local_fall_latched ||
                                  p_in->peer_fall_active,
                                  p_in->local_alarm_paused, now_ms);
        Alert_Type_t req = ALERT_NONE;
        if (p_in->sensor_fault)
        {
            req = ALERT_SENSOR_FAULT;
        }
        /* the cane never plays obstacle beeps: nothing to finish */
        s_obs_flags.beep_on = 0U;
        AlertManager_Arbitrate(req, 0U, now_ms, p_out);
        if ((p_out->active_alert == ALERT_NONE) && !p_in->peer_fall_active && !p_in->cane_buzzer_fault)
        {
            Render_CalibrationCue(now_ms, p_out);
        }
        else
        {
            s_calibration_cue = ALERT_NONE;
        }
        if (p_in->cane_buzzer_fault)
        {
            p_out->buzzer_active = false;
            p_out->generated_freq_hz = 0U;
            p_out->generated_duty_percent = 0U;
        }
    }
    else
    {
        /* Glasses node */
        AlertManager_SetObstacleCue(p_in->obstacle_cue);
        if (p_in->cane_link_fresh && p_in->cane_buzzer_fault)
        {
            /* Cane buzzer failed -> Failover: Glasses plays its own alarms plus Cane's */
            AlertManager_SetFallState(p_in->local_fall_latched || p_in->peer_fall_active,
                                      p_in->local_alarm_paused, now_ms);
            Alert_Type_t req = ALERT_NONE;
            if (p_in->obstacle_active)
            {
                req = ALERT_OBSTACLE_PROXIMITY;
            }
            else if (p_in->sensor_fault || p_in->peer_sensor_fault)
            {
                req = ALERT_SENSOR_FAULT;
            }
            AlertManager_Arbitrate(req, p_in->obstacle_dist_mm, now_ms, p_out);
        }
        else
        {
            /* Normal: the cane plays the head-fall siren. The glasses play only
             * the sparse delivery cue if the cane has not confirmed the fall
             * (no receipt in time, or the cane link went stale).
             */
            AlertManager_SetFallState(false, false, now_ms);
            Alert_Type_t req = ALERT_NONE;
            if (p_in->local_fall_latched)
            {
                if (p_in->delivery_fallback)
                {
                    req = ALERT_FALL_DELIVERY_FAULT;
                }
                else
                {
                    /* Cane confirmed the fall: glasses stay quiet. */
                }
            }
            else if (p_in->obstacle_active)
            {
                req = ALERT_OBSTACLE_PROXIMITY;
            }
            else if (p_in->sensor_fault && !p_in->peer_sensor_fault)
            {
                req = ALERT_SENSOR_FAULT;
            }
            else
            {
                /* Nothing to play. */
            }
            AlertManager_Arbitrate(req, p_in->obstacle_dist_mm, now_ms, p_out);
        }
    }
}
