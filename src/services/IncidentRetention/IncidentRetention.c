#include "IncidentRetention.h"
#include <stddef.h>

#ifdef PICO_ON_DEVICE
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#endif

/*
 * \file IncidentRetention.c
 * \brief Integrity-tagged incident record in RP2040 watchdog scratch words 0-3
 * Compliance: Barr-C:2018
 */

#define INCIDENT_RETENTION_WORD_COUNT       (4U)
#define INCIDENT_RETENTION_COMMIT_BASE      (0x45470200U)
#define INCIDENT_RETENTION_ROLE_SHIFT       (4U)
#define INCIDENT_RETENTION_CHECK_SALT       (0xA53C9E71U)
#define INCIDENT_RETENTION_PAYLOAD_MASK     (0x000FFFFFU)

static IncidentRetention_Role_t s_role;
static IncidentRetention_Record_t s_restored_record;
static bool s_initialized;
static bool s_restore_valid;

#ifndef PICO_ON_DEVICE
static uint32_t s_test_scratch[INCIDENT_RETENTION_WORD_COUNT];
static bool s_test_watchdog_reset;
#endif

static uint32_t Rotate_Left(uint32_t value, uint32_t shift)
{
    return (value << shift) | (value >> (32U - shift));
}

static uint32_t Commit_Word(IncidentRetention_Role_t role)
{
    return INCIDENT_RETENTION_COMMIT_BASE |
           ((uint32_t)role << INCIDENT_RETENTION_ROLE_SHIFT);
}

static uint32_t Check_Word(uint32_t commit, uint32_t payload,
                           uint32_t payload_complement)
{
    return commit ^ Rotate_Left(payload, 7U) ^
           Rotate_Left(payload_complement, 17U) ^ INCIDENT_RETENTION_CHECK_SALT;
}

static uint32_t Read_Word(uint32_t index)
{
#ifdef PICO_ON_DEVICE
    return watchdog_hw->scratch[index];
#else
    return s_test_scratch[index];
#endif
}

static void Write_Word(uint32_t index, uint32_t value)
{
#ifdef PICO_ON_DEVICE
    watchdog_hw->scratch[index] = value;
#else
    s_test_scratch[index] = value;
#endif
}

static bool Enabled_Watchdog_Caused_Reset(void)
{
#ifdef PICO_ON_DEVICE
    return watchdog_enable_caused_reboot();
#else
    return s_test_watchdog_reset;
#endif
}

static void Clear_Record(void)
{
    /* Invalidate first so a reset during clearing cannot revive partial data. */
    Write_Word(0U, 0U);
    Write_Word(1U, 0U);
    Write_Word(2U, 0U);
    Write_Word(3U, 0U);
    s_restore_valid = false;
}

void IncidentRetention_Init(IncidentRetention_Role_t role)
{
    bool valid_role = (role == INCIDENT_RETENTION_ROLE_CANE) ||
                      (role == INCIDENT_RETENTION_ROLE_GLASSES);
    s_role = role;
    s_initialized = valid_role;
    s_restore_valid = false;
    s_restored_record = (IncidentRetention_Record_t){0};

    uint32_t commit = Read_Word(0U);
    uint32_t payload = Read_Word(1U);
    uint32_t payload_complement = Read_Word(2U);
    uint32_t check = Read_Word(3U);
    bool active_commit = valid_role && Enabled_Watchdog_Caused_Reset() &&
                         (commit == Commit_Word(role));
    if (active_commit)
    {
        bool payload_valid = ((payload & ~INCIDENT_RETENTION_PAYLOAD_MASK) == 0U) &&
                             ((payload >> 4U) != 0U) &&
                             (payload_complement == ~payload) &&
                             (check == Check_Word(commit, payload, payload_complement));
        /* The active marker is fail-safe. A torn/corrupt counter starts a new
         * event zero in the new boot session rather than clearing the alarm.
         */
        s_restored_record.session_id = payload_valid ? (uint16_t)(payload >> 4U) : 0U;
        s_restored_record.event_seq = payload_valid ? (uint8_t)(payload & 0x0FU) : 0U;
        s_restore_valid = true;
    }
    else
    {
        /* Power/run-pin/programming resets must not revive an old incident. */
        Clear_Record();
    }
}

bool IncidentRetention_Restore(IncidentRetention_Record_t * const p_record)
{
    if (!s_initialized || !s_restore_valid || (p_record == NULL))
    {
        return false;
    }
    *p_record = s_restored_record;
    return true;
}

void IncidentRetention_Store(bool active, uint16_t session_id, uint8_t event_seq)
{
    if (!s_initialized)
    {
        return;
    }
    if (!active)
    {
        if (s_restore_valid)
        {
            Clear_Record();
        }
        return;
    }
    event_seq &= 0x0FU;
    if (s_restore_valid && (s_restored_record.session_id == session_id) &&
        (s_restored_record.event_seq == event_seq))
    {
        return;
    }

    uint32_t commit = Commit_Word(s_role);
    uint32_t payload = ((uint32_t)session_id << 4U) | event_seq;
    uint32_t payload_complement = ~payload;
    uint32_t check = Check_Word(commit, payload, payload_complement);
    /* Mark active first. A reset during the remaining writes restores a
     * conservative event zero rather than losing the unresolved alarm.
     */
    Write_Word(0U, commit);
    Write_Word(1U, payload);
    Write_Word(2U, payload_complement);
    Write_Word(3U, check);
    s_restored_record.session_id = session_id;
    s_restored_record.event_seq = event_seq & 0x0FU;
    s_restore_valid = true;
}

#ifndef PICO_ON_DEVICE
void IncidentRetention_TestPowerCycle(void)
{
    for (uint32_t i = 0U; i < INCIDENT_RETENTION_WORD_COUNT; ++i)
    {
        s_test_scratch[i] = 0U;
    }
    s_test_watchdog_reset = false;
    s_role = (IncidentRetention_Role_t)0;
    s_initialized = false;
    s_restore_valid = false;
    s_restored_record = (IncidentRetention_Record_t){0};
}

void IncidentRetention_TestSetWatchdogReset(bool caused_by_enabled_watchdog)
{
    s_test_watchdog_reset = caused_by_enabled_watchdog;
}

void IncidentRetention_TestRestartRuntime(void)
{
    s_initialized = false;
    s_restore_valid = false;
}

void IncidentRetention_TestCorruptWord(uint32_t index, uint32_t value)
{
    if (index < INCIDENT_RETENTION_WORD_COUNT)
    {
        s_test_scratch[index] = value;
    }
}

uint32_t IncidentRetention_TestReadWord(uint32_t index)
{
    return (index < INCIDENT_RETENTION_WORD_COUNT) ? s_test_scratch[index] : 0U;
}
#endif
