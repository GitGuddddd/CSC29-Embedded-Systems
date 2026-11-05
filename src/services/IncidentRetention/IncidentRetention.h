#ifndef INCIDENTRETENTION_H
#define INCIDENTRETENTION_H

#include <stdbool.h>
#include <stdint.h>

/*
 * \file IncidentRetention.h
 * \brief Watchdog-reset retention for an unresolved application incident latch
 * Compliance: Barr-C:2018
 */

typedef enum
{
    INCIDENT_RETENTION_ROLE_CANE = 1U,
    INCIDENT_RETENTION_ROLE_GLASSES = 2U
} IncidentRetention_Role_t;

typedef struct
{
    uint32_t event_id;
} IncidentRetention_Record_t;

/* Initialize before Watchdog_Init(), which programs the SDK watchdog marker.
 * Runtime Store/Restore calls belong to the Core 0 safety owner.
 */
void IncidentRetention_Init(IncidentRetention_Role_t role);
bool IncidentRetention_Restore(IncidentRetention_Record_t * const p_record);
void IncidentRetention_Store(bool active, uint32_t event_id);

#ifndef PICO_ON_DEVICE
/* Host-only hooks exercise reset cause, retained scratch and torn-record handling. */
void IncidentRetention_TestPowerCycle(void);
void IncidentRetention_TestSetWatchdogReset(bool caused_by_enabled_watchdog);
void IncidentRetention_TestRestartRuntime(void);
void IncidentRetention_TestCorruptWord(uint32_t index, uint32_t value);
uint32_t IncidentRetention_TestReadWord(uint32_t index);
#endif

#endif /* INCIDENTRETENTION_H */
