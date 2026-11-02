# Grip state interface: Glasses_Threat and the Glasses BLE task

Status: draft for the BLE owner. See also docs/deviations.md, OPEN-1 and OPEN-12.

## What you call

    void rte_write_grip_state (rte_grip_state_t grip);    /* rte.h */

## Values (rte_type.h)

- `RTE_GRIP_HELD`: the cane packet says the cane is gripped.
- `RTE_GRIP_RELEASED`: the cane packet says the cane is let go.
- `RTE_GRIP_UNKNOWN`: no valid cane packet for 500 ms.

## What we need from the BLE task

1. On every received cane packet, call it with HELD or RELEASED. Please send
   it on every packet (about 10 Hz), not only on changes. The queue can drop a
   message, and the next packet then corrects it.
2. If no cane packet has arrived for 500 ms, call it once with UNKNOWN. This is
   the BLE task's job: Glasses_Threat only sees messages, so a stale RELEASED
   would silence the obstacle alert until the next message arrives.
3. Nothing is needed at power-up. Glasses_Threat starts as UNKNOWN.

## What Glasses_Threat does with it

- HELD and UNKNOWN leave the obstacle alert active (report FR6).
- RELEASED ends an active alert with one OBSTACLE(CLEAR) and keeps it silent.
- When grip returns to HELD or UNKNOWN, the next radar frame restarts the
  alert if a target is inside 2000 mm.
- A radar fault does not reset the stored grip state.

## Behaviour to know about

- The call waits up to 10 ms (RTE_CRITICAL_WAIT_MS) for queue space, then
  drops the message. It never blocks longer.
- Call it from the BLE task, not from an interrupt.
- The queue is shared with radar frames and holds 4 items (RTE_RADAR_QUEUE_LENGTH).

## Not settled yet

- Which bit or byte of the cane's 11-byte packet carries grip is undefined in
  the report. The BLE task decodes it; Glasses_Threat only receives the enum.
- Tested only with injected messages on the PC (tests/unit/test_threat_assessment.c,
  tests/integration/test_alert_pipeline.c), not with a real BLE source or on the Pico.

## Questions for the BLE owner

- Can the BLE task send on every packet?
- Does the 500 ms UNKNOWN timeout match the cane's own deadman timing?
