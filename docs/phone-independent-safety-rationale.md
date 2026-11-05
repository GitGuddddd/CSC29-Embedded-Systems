# Phone-independent safety: architecture rationale and robustness assessment

Baseline assessment of firmware and Android revision `67c4148`, updated for the
versioned critical-event receipt and glasses fallback implementation.

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

The six-byte status is now followed by a versioned 18-byte safety extension. It carries
a random 32-bit boot session, 32-bit local event counter and exact peer application
receipt. A fresh heartbeat alone still is not treated as delivery proof. The glasses
fall back locally if the matching receipt is absent for 750 ms or cane freshness later
expires. The event remains in coalesced status while its incident latch is active, and
an unsuccessful queue publication is retried by the next application cycle.

Concrete findings:

| Finding | Evidence | Consequence |
| --- | --- | --- |
| Application receipt is not actuator feedback | Exact receipt in `Rte_Type.h`, `BleComm.c` and both main loops | Confirms cane application acceptance/alarm request, not physical sound output or sound pressure |
| Glasses fallback is implemented but not acoustically validated | `SafetyDelivery.c`, `Glasses_Main.c`, low-duty `AlertManager` cue | Software enters a defined degraded mode; enclosure SPL and user suitability still require bench/usability evidence |
| Latches initialize clear | `CaneMode_Init()` and `head_fall_latched = false` | A locally originated incident can disappear after a reset; a still-latched remote peer might reassert its event, but local retention is not guaranteed |
| Event identity is stronger but unauthenticated | 32-bit random boot session and event counter; four-bit pair ID remains default 1 | Removes practical heartbeat/fall-counter ambiguity, but does not authenticate a peer or prevent deliberate spoofing/replay |
| Firmware transport discards advertiser identity/RSSI | `BleRadio_RxCallback_t` carries data/length/time only | Firmware has no address binding or link-quality history comparable to Android's role/address scope |
| Reset recovery affects both cores | Shared MCU hardware watchdog | Core affinity does not isolate power, memory or whole-chip reset; recovery introduces a monitoring interruption |

The existing two-opportunity budget, `2 × (200 + 10) + 50 = 470 ms`, is a scheduling
budget, not a guaranteed delivery bound. Losses can be bursty and correlated. Do not
infer reliability by assuming independent packet loss without measurements.

## Highest-value enhancements

### Critical-event receipt and retention — implemented in software

The protocol now uses `(origin node, boot/session ID, event counter)`. The active latch
retains the identity independently of heartbeat wrap and coalescible range telemetry.
The cane returns a receipt only after its application has latched the event and issued
the alarm request. The glasses wait a bounded 750 ms before degraded fallback and
re-enter fallback if cane freshness expires. An actuator command is not proof that sound
was physically produced; that requires hardware feedback or a verified output path.

Keep **transport receipt**, **user audio pause** and **incident clearance** distinct.
Each acknowledgement/reset references the relevant event, so a delayed message cannot
clear a newer fall. Relay messages retain their original event identity to prevent loops.

### Glasses fallback implemented; reset survival remains

The glasses now use a 100 ms pulse every 2 s at 10% PWM duty when the cane is unavailable
or fails to confirm delivery. This intentionally is not the cane siren. PWM duty is not
an acoustic exposure measurement; frequency, enclosure SPL and user suitability still
require validation before calling the response ear-safe.

Preserve critical latch/event state across watchdog resets. A retained record can cover
watchdog recovery; surviving power loss additionally needs an appropriate nonvolatile
journal. Write only meaningful event transitions, with integrity/atomic-update handling,
rather than logging every heartbeat to flash. Flash operations remain outside ISRs.

### Versioning implemented; provisioning/authentication remain

The 24-byte manufacturer frame is versioned and handles reboot epochs explicitly with
a random 32-bit session. Provision a stronger device/group identity and authenticate
safety-relevant control/event messages. Radio CRC handles transmission corruption; it
does not authenticate the sender. Address filtering is useful for accidental cross-talk,
but is not cryptographic authentication either.

The original six-byte status remains the frame prefix. Removing the repeated short name
from the primary advertisement made room for the 18-byte extension; the full name stays
in the scan response. Flags and AD headers bring the legacy advertisement to 31 bytes.

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
   `src/services/BleComm/BleComm.c`, `src/drivers/BleRadio/BleRadio.c`, `src/rte/Rte_Type.h`.
