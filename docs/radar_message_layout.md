# Radar queue message layout (draft)

Status: IMPLEMENTED on the PC (27 Oct 2026), internal to Putra's subsystem.
Code: src/rte/rte_pack.[ch] (pack and unpack), distance in ld2450.c,
tests/unit/test_rte_pack.c. Not yet measured or run on the Pico.
Reason: change requirement "Minimum-Bit Data Representation" (Week 8).
Related: docs/audio_message_layout.md, docs/grip_interface.md.

Direction: Glasses_Radar (frames, faults) and the Glasses BLE task (grip,
through `rte_write_grip_state`) send, Glasses_Threat receives, through the
radar queue. Other buddies never read this queue, so only Putra needs to agree
it. The BLE owner keeps calling `rte_write_grip_state(enum)`; the packing is
hidden behind that call.

## Before

`rte_radar_msg_t` was 44 bytes (4 B type, 4 B time, 3 x 10 B target slots, 4 B
grip). The queue holds 4, so 176 bytes.

## Proposed: 102 bits in a 13-byte array

Bit n is bit (n % 8) of byte (n / 8), counting from 0 (least significant bit
first). Pack and unpack with two shared helpers, `rte_pack_put_bits(buf, offset,
width, value)` and `rte_pack_get_bits(buf, offset, width)`, so nobody writes shifts
by hand.

| Bits   | Field  | Width | Meaning |
|--------|--------|-------|---------|
| 0-1    | type   | 2     | 0 FRAME, 1 FAULT, 2 GRIP, 3 reserved |
| 2-3    | grip   | 2     | 0 UNKNOWN, 1 HELD, 2 RELEASED, 3 reserved. 0 for FRAME and FAULT. |
| 4-35   | time   | 32    | Frame receive time in ms from boot (the `hal_time` clock). FRAME only, else 0. |
| 36-57  | slot 0 | 22    | See slot layout |
| 58-79  | slot 1 | 22    | See slot layout |
| 80-101 | slot 2 | 22    | See slot layout |
| 102-103| spare  | 2     | Always 0 |

The timestamp stays 32 bits as a justified exception (same reasoning as Brian's
fall-detection timestamps): it is the system clock, it wraps after about
49.7 days with unsigned subtraction still correct, and Threat's stored times
use the same width, so no mixed-width arithmetic. Shrinking it would save only
2 bytes per message.

Slot layout (22 bits, offsets inside the slot):

| Bits  | Field    | Width | Meaning |
|-------|----------|-------|---------|
| 0     | valid    | 1     | 1 = a real target, 0 = empty slot (all other slot bits 0) |
| 1-12  | distance | 12    | Units of 2 mm, 0 to 4095 = 0 to 8190 mm. Rounded down, saturates at 4095. |
| 13-21 | speed    | 9     | Signed cm/s, two's complement, clamped to -255..+255, sign as the sensor sends it. |

Total 102 bits, 13 bytes, so the 4-entry queue is 52 bytes (from 176).

## Changes to what the message carries

- `resolution_mm` is dropped. No logic uses it.
- `x` and `y` are dropped. Threat only ever uses the distance, so Glasses_Radar
  sends the straight-line distance, worked out before the message is built.
  Decided: the ld2450 driver computes it, in `ld2450_decode_body()` right after
  `x` and `y` are decoded (protocol geometry; `radar_monitor` and the RTE stay
  free of maths). The decoded target keeps `x` and `y` for the radar task's
  debug print and gains a distance field. Threat still decides what is an
  obstacle.
- Speed is clamped symmetrically so that negating it (the approaching-sign
  switch in `threat_assessment_closing_cms`) can never overflow 9 bits.
- Frame, fault and grip messages share one fixed-size queue item, so a fault or
  grip message wastes the unused 80 bits. Accepted.

## Behaviour changes to test

- Distance is floored to an even value. The start edge is unchanged (1999 mm
  starts, 2000 mm does not). The stop edge moves by 1 mm: 2101 mm now reads
  as 2100 and holds, so the alert clears at 2102 mm. Hysteresis edge tests
  must use 2 mm steps (test_packed_distance_edges_through_threat).
- A corrupt frame with a valid footer can give a distance up to 46339 mm. It
  saturates at 4095 (8190 mm), which means far and never alerts.
- Speed above 255 cm/s is clamped. TTC at that speed is already tiny.

## Scope

This covers the queue message only. Each task still works on ordinary
integers on its own stack while it runs. The retained state in
`threat_assessment_t` and `radar_monitor_t` is a separate decision, to be
packed after this message works.

## Test and evidence

- Round-trip tests for every field at its minimum, maximum and out-of-range value.
- Threat unit tests rewritten to feed distance directly, with the 2 mm edges.
- Pipeline test (bytes, then radar monitor, then Threat, then audio) unchanged
  in outcome.
- Message size: 44 B before, 13 B after. Queue 176 B before, 52 B after
  (confirm `sizeof` on the Pico build).

## Open questions

None. Speed width settled by capture (assumption A7), 27 Oct 2026.

Evidence: docs/evidence/2026-10-27_speed_walk.txt (walk, 415 to 2192 mm) and
docs/evidence/2026-10-27_speed_bench.txt (bench, 183 to 515 mm). Largest
|speed| seen was 112 cm/s, with no reading above 255, so 9 bits with a +/-255
clamp holds. Negative speed matched approaching in 96 of 100 clear-motion
samples (the 4 misses were at turnarounds), which agrees with
threat_assessment_lcfg.c. Keep the width as one named constant.

Settled: timestamp stays 32 bits (justified exception, see above). Distance is
computed in the ld2450 driver. Speed is 9 bits.

## Implementation notes

- Functions: `rte_pack_put_bits`, `rte_pack_get_bits`, `rte_pack_radar`,
  `rte_pack_unpack_radar`. `rte_radar_msg_t` is now the 13 packed bytes and
  `rte_radar_data_t` is the same message as ordinary fields.
- A reserved type code (3) unpacks as FAULT and a reserved grip code (3) as
  UNKNOWN, the safe readings. Spare bits are written as 0 and ignored.
- `resolution_mm` is removed. `x_mm` and `y_mm` stay in the decoded target
  for the debug print only; they are not in the message.
- The speed width is `RTE_PACK_SPEED_BITS` in rte_pack.h.
