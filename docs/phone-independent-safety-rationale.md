# Phone-independent safety: architecture rationale and robustness assessment

Baseline assessment of firmware and Android revision `67c4148`, updated for the
compact v2 receipts, minimum-bit/phone-history split, glasses fallback and watchdog retention.

## Recommendation

Retain local detection and primary actuation on the wearables, with direct cane–glasses
communication. Android is the supervisory interface and an additional alert channel.
It may aggregate data without becoming a mandatory intermediary for a local warning.

The architecture is defensible for a prototype. End-to-end emergency delivery and
fault-tolerant operation are not yet demonstrated sufficiently for an unconditional
robustness or safety claim. Source tests and BARR-C discipline do not establish RF
delivery probability, actuator effectiveness or hardware worst-case latency.

## State the requirement first

> Loss of the phone, its Bluetooth service, or the app process shall not disable the
> wearable system's primary local hazard warnings.

This requirement makes phone independence an architectural decision rather than a
claim that a smartphone is incapable of processing sensor data. If a project instead
defines a managed phone as mandatory equipment, a phone-centered architecture can be
reasonable, but its availability and failure behavior must be justified accordingly.

## Actual roles in the current implementation

- **Glasses:** local IMU fall detection and radar threat assessment; local obstacle
  audio; broadcast head-fall/status events. Lost cane telemetry becomes UNKNOWN grip,
  which retains obstacle alerts rather than suppressing them. A missing or expired
  cane application receipt starts a sparse low-duty local head-fall cue.
- **Cane:** local ground/local-fall processing and haptics; receives head-fall events
  and drives the larger fall alarm. It echoes the exact head-fall identity only after
  its application latches the incident and issues the alarm request.
- **Android:** observes both nodes, presents pair-scoped status and supplies additional
  phone haptics. It currently sends no device wake/reset/relay command.

Neither wearable waits for an Android decision before its existing local processing.
The current radio topology uses advertiser/observer roles, not GATT client/server
connections. In a future GATT design, GAP central/peripheral and GATT client/server
roles remain distinct from who owns the safety decision.

## Why Android should not be a mandatory alarm relay

1. **Fewer required operational dependencies.** A phone-mediated head-fall alarm
   requires both wearables, two successful BLE transfers, a powered/enabled phone,
   permissions, a running app and timely OS dispatch. Direct communication avoids
   making those phone conditions prerequisites for the cane alarm.
2. **Control of timing and recovery.** We control MCU task priorities, queue bounds,
   watchdog policy and deployed firmware. Android's scan and foreground-service APIs
   support background BLE, but do not provide this application a hard upper bound on
   callback-to-actuator latency. Foreground service improves continuity; it is not a
   hard-real-time or unkillable execution guarantee.
3. **Fault containment.** UI, permissions, phone battery and app lifecycle failures can
   remove the dashboard/additional haptics while local detection continues. Putting
   Android in series turns those same failures into missing primary warnings.
4. **Validation scope.** A mandatory phone expands the alarm path across handset
   hardware, Bluetooth implementations, Android versions and app lifecycle states.
   That variability must be tested as part of the primary safety path.
5. **The MCU workload already fits.** The nodes already perform their own sensing and
   classification and exchange compact results. Extra phone processing capacity is
   useful for visualization/analysis, but is not required for those implemented loops.

An Android relay could work without Internet access. Internet availability is therefore
not the central objection. A favorably positioned phone might also improve RF coverage;
direct communication is not automatically a better radio path in every body orientation.
The decisive argument is avoiding an unnecessary mandatory phone dependency.

Android's own documentation supports the lifecycle boundary:

- Callback-based background BLE discovery requires the app process to remain alive;
  PendingIntent scanning and CompanionDeviceService are supported recovery/lifecycle
  mechanisms. Connected BLE links close if the process is killed. [1]
- Android 13+ lets users stop an app even while it runs a foreground service; the
  entire app is removed from memory and no stop callback is delivered. [2]
- LOW_LATENCY scanning requests the highest scan duty cycle and is recommended for
  foreground use; it is not a deadline guarantee. [3]

## Current strengths and remaining delivery gaps

The shared BLE policy already covers all states, with 100/150/200 ms base advertising
intervals, immediate escalation, a 1 s downshift hold, bounded queues, task-owned
BTstack operations, timestamped RX, duplicate handling and worker-aware watchdogs.
The 500 ms deadman detects missing fresh peer telemetry. These are useful mechanisms.

The six-byte status is now followed by a six-byte v2 receipt extension. It carries
16-bit transport epochs and a packed four-bit receipt sequence; the local count already
occupies the status prefix. A fresh heartbeat alone is not delivery proof. The glasses
fall back locally if the matching receipt is absent for 750 ms or cane freshness later
expires. The event remains in coalesced status while its incident latch is active, and
an unsuccessful queue publication is retried by the next application cycle.

Concrete findings:

| Finding | Evidence | Consequence |
| --- | --- | --- |
| Application receipt is not actuator feedback | Exact receipt in `Rte_Type.h`, `BleComm.c` and both main loops | Confirms cane application acceptance/alarm request, not physical sound output or sound pressure |
| Glasses fallback is implemented but not acoustically validated | `SafetyDelivery.c`, `Glasses_Main.c`, low-duty `AlertManager` cue | Software enters a defined degraded mode; enclosure SPL and user suitability still require bench/usability evidence |
| Watchdog retention is volatile and reset-cause scoped | `IncidentRetention.c` uses RP2040 watchdog scratch 0–3 and `watchdog_enable_caused_reboot()` | Timeout resets restore unresolved incidents; power loss, run-pin reset, firmware-loading reboot and invalid role markers intentionally do not |
| Compact transport identity is finite and unauthenticated | Four-bit event sequence, nonzero 16-bit epoch renewed before sequence reuse; pair ID remains default 1 | Distinguishes ordinary wraps and reboots within the stated model, not arbitrary historical replay or every random boot collision |
| Firmware transport discards advertiser identity/RSSI | `BleRadio_RxCallback_t` carries data/length/time only | Firmware has no address binding or link-quality history comparable to Android's role/address scope |
| Reset recovery affects both cores | Shared MCU hardware watchdog | Core affinity does not isolate power, memory or whole-chip reset; recovery introduces a monitoring interruption |

The existing two-opportunity budget, `2 × (200 + 10) + 50 = 470 ms`, is a scheduling
budget, not a guaranteed delivery bound. Losses can be bursty and correlated. Do not
infer reliability by assuming independent packet loss without measurements.

## Highest-value enhancements

### Critical-event receipt and retention — implemented in software

The protocol now uses `(origin node, 16-bit transport epoch, four-bit event sequence)`. The active latch
retains the identity independently of heartbeat wrap and coalescible range telemetry.
The cane returns a receipt only after its application has latched the event and issued
the alarm request. The glasses wait a bounded 750 ms before degraded fallback and
re-enter fallback if cane freshness expires. An actuator command is not proof that sound
was physically produced; that requires hardware feedback or a verified output path.

Keep **transport receipt**, **user audio pause** and **incident clearance** distinct.
Receipts reference the relevant origin epoch/sequence and never clear a fall. User
pause/recovery remains local; Android has no remote alarm-clear authority.

### Tribunal 1 minimum-bit constraint and smartphone offload

Tribunal 1's injected constraint requires each application field to use only the bits
its valid range needs. The Brian/Clifton negotiation reduces the fall count from eight
bits to three/four bits and moves cumulative statistics to the phone (meeting 29:05–30:40).
The later E2 clarification keeps a four-bit rolling sequence in every packet, rather
than resetting it merely because a connectionless transmission was attempted.

The v2 implementation follows that split. Local counts are four bits, already encoded
in the original status prefix; the redundant 32-bit local event counter is removed.
Android accumulates and persists the observed increments, including a known rollover.
It marks unknown baselines, reboots and missed epochs as incomplete history rather
than inventing an exact lifetime total. The phone is a history sink, not a required
alarm relay. All reception/receipt/fallback decisions still work without it.

Receipt metadata has a different job from statistics. Two 16-bit epoch tags and a packed
four-bit receipt/validity/rollover byte are enough for this prototype's chosen transport
namespace; the origin event nibble is not duplicated. The epoch renews before 15 → 0,
and active watchdog retention lets a reboot exclude the known prior epoch. This is a
documented prototype transport choice, not proof that 16 bits are sufficient for every
fault/replay model. Random boot collisions and eventual namespace reuse remain possible.
A stricter risk requirement needs a revised/provisioned/authenticated exchange and
evidence, not an arbitrarily wider lifetime fall counter. Timer and integrity-register
widths remain separately justified; neither is cumulative application history.

The complete manufacturer frame is now 12 bytes and primary advertising data 19 bytes.
Available capacity is not justification for filling the 31-byte ceiling. Shorter packets
reduce encoded data/queue storage and raw airtime; they do not prove a corresponding
percentage improvement in end-to-end latency, RF recovery or battery life.

Cross-compiler layout evidence against `4828dd4` (RP2040 C ABI, `arm-none-eabi-gcc`
with Cortex-M0+ / Thumb flags; not host `sizeof`):

| Item | Previous bytes | Compact v2 bytes |
| --- | ---: | ---: |
| Manufacturer frame | 24 | 12 |
| Decoded `Ble_AdvancedThreatPacket_t` | 40 | 22 |
| `BleComm_PeerFall_t` | 16 | 6 |
| `SafetyDelivery_t` | 28 | 20 |
| `IncidentRetention_Record_t` | 4 | 4 |

Whole-image static BSS decreases by 252 bytes on each node in the current Release
build (cane 71680 → 71428, glasses 72648 → 72396). Flash text increases by 528/480
bytes respectively for epoch renewal and stricter parsing/receipt handling. These are
software layout/build results, not measured task jitter, physical delivery or power data.

The subsequent BLE-only dependency cleanup keeps the same FreeRTOS worker and removes
unused lwIP initialization/pools plus the SDK default static radio stack. Against
`3c2d3a8`, BSS decreases by another 14588 bytes per node (cane 71428 → 56840,
glasses 72396 → 57808), and ELF text by 9148 bytes. Both application radio workers
remain at 4096 bytes. Source/ELF ownership checks pass, but radio hardware, power,
worst-case scheduling and watchdog reset behavior remain unvalidated.

The subsequent TTC correction removes both 16-bit millisecond wrap paths without widening
the application field: threat results and decoded packets retain the existing 12-bit,
20 ms wire unit. A 200-second estimate is explicitly unavailable (`0xFFF`), not 3392 ms;
the valid maximum 81880 ms remains 4094 ticks, not 16344 ms after narrowing. The threat
result packs its active flag with the TTC field, reducing its target layout from eight
to six bytes. Codec quantization is performed once and the wire layout is unchanged.
All 4096 TTC values, boundary inputs and the shared Android maximum vector are tested.
These are software correctness/layout results, not physical hazard-detection validation.

Header-first radar parsing subsequently reduces a rejected noise candidate from a
30-byte volatile copy to four bytes while preserving the producer/overwrite checks.
Valid headers still undergo full body/footer validation. A reproducible native benchmark
shows lower synthetic noise CPU cost with comparable valid-input results; the glasses
image uses 96 more ELF text bytes and no additional BSS. No sensing rate, DMA size,
scan duty or safety deadline was reduced, and physical timing remains unvalidated.

### Glasses fallback and watchdog-reset survival implemented; power loss remains

The glasses now use a 100 ms pulse every 2 s at 10% PWM duty when the cane is unavailable
or fails to confirm delivery. This intentionally is not the cane siren. PWM duty is not
an acoustic exposure measurement; frequency, enclosure SPL and user suitability still
require validation before calling the response ear-safe.

Both nodes retain an unresolved latch, four-bit sequence and prior 16-bit epoch in four RP2040 watchdog
scratch words. The role/version commit marker is written before an active payload and
cleared before the remaining words on incident clearance; complement/check words detect
a torn payload. A valid active marker fails safe: corrupt payload data restores the
alarm as event zero under
the newly generated boot session rather than clearing it. Writes occur only when the
incident becomes active, changes identity or clears, and only from the Core 0 task.

Restoration is accepted only when the SDK reports that the enabled hardware watchdog
caused the reboot. A retained cane audio pause resumes audible. A restored glasses event
starts a new 750 ms receipt deadline, and the old cane receipt is not retained as proof
for the new application instance. Normal guarded/user clearance is still required.

The scratch record is volatile. Surviving power loss or brownout additionally needs an
appropriate nonvolatile journal and endurance/power-fail analysis; heartbeat-rate flash
logging would be inappropriate. Hardware fault injection must still verify the actual
RP2040 reset-cause and scratch-retention behavior.

### Versioning implemented; provisioning/authentication remain

The 12-byte v2 manufacturer frame is versioned and handles reboot epochs explicitly with
a nonzero 16-bit tag. Provision a stronger device/group identity and authenticate
safety-relevant control/event messages. Radio CRC handles transmission corruption; it
does not authenticate the sender. Address filtering is useful for accidental cross-talk,
but is not cryptographic authentication either.

The original six-byte status remains the frame prefix. Removing the repeated short name
from the primary advertisement avoids unnecessary airtime; the full name stays in the
scan response. The six-byte receipt extension plus flags/AD headers brings primary data
to 19 bytes. Upgrade both wearables together; older exact-size parsers cannot consume v2.

## Transport and energy choices to benchmark

Phone-independent operation does not require staying with advertising-only transport.
A direct cane–glasses connection is a valid alternative to measure: connected BLE offers
data-channel hopping, link-layer acknowledgements/retries and scheduled connection
events. These can improve reliability and receiver energy use, but connection loss and
re-establishment still require explicit handling. GATT indications/transport ACKs also
do not by themselves prove the cane application latched or actuated the alarm.

A practical candidate is one authenticated direct wearable connection for critical
events, while advertisements continue providing phone-observable status. Current
`btstack_config.h` limits each node to one HCI connection; connection roles/resources
must be checked before promising concurrent additional phone connections.

Measure before choosing. Current 80–100% requested scanning duty is likely a larger
power opportunity than shaving a few status bytes. Blindly lowering scan duty risks
missing alarm/wake traffic and violating the freshness deadline. Also profile unused
Wi-Fi/lwIP build components, diagnostic overhead and antenna placement near the hand,
battery and metal cane parts.

Additional useful work: staged/non-blocking sensor reinitialization, a glasses sleep
policy that does not equate stillness with being unworn, and Android selection restoration
plus CompanionDeviceService/appropriate background discovery. These improve resilience
without giving the phone authority to disable local alerts.

## Evidence needed for a stronger claim

Measure confirmed-event-to-alarm-command latency separately from physical-fall-to-detection
latency. Record median/tail/observed maximum, deadline misses, longest loss burst, retry
counts, queue drops and current consumption, with sample sizes and test conditions.
Use shared logic-analyzer GPIO markers or origin-measured acknowledgement RTT; subtracting
unsynchronized clocks from two nodes is not a valid one-way latency measurement.

Exercise phone-off/app-stop/Bluetooth-off, cane-off, either-node reboot, body shadowing,
different antenna orientations, congested Wi-Fi, burst packet loss, two pairs nearby,
sensor faults during radio traffic and low-battery/brownout behavior. Verify the defined
local fallback when event delivery fails. The code tests already run are not substitutes
for these physical fault-injection and RF measurements.

## Short defense statement

> We keep primary detection, alarm state and actuation on the wearables so that an
> unavailable phone cannot disable their local safety functions. Android provides
> supervision and additional alerts, but its battery, permissions and app lifecycle
> are not prerequisites for the primary warning path. Direct communication reduces
> mandatory dependencies; delivery receipts, local fallback behavior and measured
> fault-injection results are how we substantiate robustness.

### References

1. [Android: Communicate in the background](https://developer.android.com/develop/connectivity/bluetooth/ble/background)
2. [Android: Handle users stopping an app with a foreground service](https://developer.android.com/develop/background-work/services/fgs/handle-user-stopping)
3. [Android: BLE scan settings](https://developer.android.com/reference/android/bluetooth/le/ScanSettings#SCAN_MODE_LOW_LATENCY)
4. Project implementation: `src/app/Cane_Main.c`, `src/app/Glasses_Main.c`,
   `src/app/ThreatAssessment/ThreatAssessment.c`, `src/app/CaneMode/CaneMode.c`,
   `src/services/BleComm/BleComm.c`, `src/services/IncidentRetention/IncidentRetention.c`,
   `src/drivers/BleRadio/BleRadio.c`, `src/rte/Rte_Type.h`.
