# Audio Alert message layout (draft)

Status: DRAFT for Brian (Audio Alert owner). Nothing in the code uses this yet.
Reason: change requirement "Minimum-Bit Data Representation" (Week 8, Putra's
tribunal row, risk R2). Related: docs/grip_interface.md, docs/deviations.md OPEN-1.

Direction: Glasses_Threat and Glasses_Radar (through `rte_write_audio_msg`)
send, Audio Alert (`alert.c`) receives, through the audio queue.

## Now

`rte_audio_msg_t` is a struct of a 4-byte enum and a 16-bit distance: 8 bytes
per message, 64 bytes for the 8-entry queue.

## Proposed: one 16-bit word

| Bits  | Field    | Width | Meaning |
|-------|----------|-------|---------|
| 0-2   | type     | 3     | 0 OBSTACLE, 1 OBSTACLE_CLEAR, 2 RADAR_FAULT, 3 RADAR_OK. 4 and 5 proposed for SENSOR_FAULT and SENSOR_OK (Brian confirms). 6 and 7 reserved. |
| 3-14  | distance | 12    | Units of 2 mm, 0 to 4095 = 0 to 8190 mm. Only meaningful for OBSTACLE; send 0 for every other type. |
| 15    | reserved | 1     | Always 0. |

Bit 0 is the least significant bit of the `uint16_t`. Receiver decodes with
`distance_mm = ((word >> 3) & 0x0FFF) * 2`.

Total: 16 bits, so a 2-byte message and a 16-byte queue (8 x 2 B).

## Rules

- **Rounding:** the sender rounds distance DOWN to an even millimetre value
  (floor). A target 1999 mm away is sent as 1998 mm. This errs toward alerting.
  With the 2 Nov pitch formula (2 Hz per mm) the pitch step is 4 Hz per field step, far below what the ear hears.
- **Saturation:** a distance above 8190 mm is sent as 4095. Threat only sends
  OBSTACLE while a target is inside 2100 mm, so this is a safety net for
  corrupt frames, not a normal value.
- **When sent:** unchanged. OBSTACLE every radar frame while active, CLEAR once
  when the alert ends, RADAR_FAULT once on entering fault, RADAR_OK once on
  recovery.
- **Helpers:** pack and unpack live in `rte_type.h` as shared functions (for
  example `rte_audio_pack(type, d_mm)`, `rte_audio_type(word)`,
  `rte_audio_distance_mm(word)`), so nobody writes the shifts by hand.
- **Atomicity:** the word is copied whole through the kernel queue. No shared
  variable is read-modify-written, so no extra locking is needed.
- **Both sides change in the same commit.** There is no version field.

## Test and evidence

- Round-trip test over all 4096 distance values and every type.
- Saturation and rounding tests (1999 to 1998 mm, 9000 mm to 4095).
- Alert latency test, tribunal "Radar + Audio Alert": `OBSTACLE` / `CLEAR` /
  `FAULT` through the packed word, correct behaviour, fault priority, 100 ms
  or less. Evidence: serial log and buzzer recording.
- Message size: 8 B before, 2 B after (confirm `sizeof` on the Pico build).

## Questions for Brian

1. Is a 3-bit type with 4 = SENSOR_FAULT and 5 = SENSOR_OK right for your fault
   tones, or do you want other codes?
2. Is 2 mm distance resolution fine for the pitch map?
3. Does `alert.c` need anything else from Threat, such as TTC for urgency? There
   are 0 spare bits except bit 15.
4. Can `alert.c` take a `uint16_t` word from the queue, or does it need the
   struct?
