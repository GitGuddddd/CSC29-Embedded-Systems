# Android telemetry synchronization

The Android companion consumes the firmware's six-byte status plus six-byte v2 receipt
extension in one 12-byte manufacturer frame. It retains telemetry-only support for the
former 24-byte v1 frame, six-byte status, five-byte legacy and 10/11-byte advanced formats. It
observes broadcasts; selecting a node does not create a GATT connection or a Bluetooth
bond, and the phone does not send an alarm-clear or wake command.

## Status contract

| Flag | Android behavior |
| --- | --- |
| `0x01` FALL_ALERT | Keep the incident latched and visible |
| `0x02` SENSOR_FAULT | Show generic sensing/data-path degradation; the packet cannot identify an IMU failure versus optical/radar obstruction |
| `0x04` OBSTACLE_ACTIVE | Respect the firmware's calibrated ground/threat decision |
| `0x08` ALARM_PAUSED | Show audio pause while retaining the incident; stop that node's phone fall signal |
| `0x10` CANE_HELD | Show compact cane grip status; this can remain set during deliberate parking |
| `0x20` STANDBY | Show sensor sleep and suppress distance-derived alarms |
| `0x40` PARKED_BY_USER | Explain deliberate parking and release-then-hold wake when the grip is still held |

Fall status has priority over standby: a remote incident can be reported while the
cane IMU is asleep. A standby fault flag remains visible as secondary information.
The app does not infer the full `Sys_State_t`, radio interval or a remaining pause
countdown: those values are not transmitted in the packet.

For a current v2 glasses fall, Android compares the glasses `(16-bit epoch, four-bit sequence)` with
the cane's application-receipt identity. It reports confirmed only while both node
heartbeats are fresh and the identities match exactly. A receipt means the cane
application latched that event and issued its alarm request; it is not proof of sound
pressure, a user acknowledgement or incident clearance. Legacy six-byte compact
packets and former v1 frames remain readable but cannot confirm the v2 receipt contract.

Modern cane data uses the firmware's hazard flag rather than assuming every ground
distance over 1500 mm is a cliff. A parked/released cane's unavailable range is not
a drop-off. Modern glasses data likewise respects the firmware's threat filtering.
Legacy distance-only packets retain their legacy threshold interpretation.

Distance/TTC sentinels display as unavailable/no target. Compact packets do not
contain closing speed, so the UI does not display the decoder's synthetic zero as
a measured 0 cm/s. Rolling fall sequence numbers are labeled as sequences, not
lifetime incident totals.

## Phone-owned cumulative history

Tribunal 1 explicitly keeps the wearable fall count small and offloads accumulation to
the smartphone. `PhoneFallHistory` stores accepted local counter increments per selected
pair/address/role. It keeps a `Long` total on the phone; the firmware keeps only its
four-bit rolling count. The first packet establishes a baseline rather than claiming
all past falls. Heartbeats, pauses and a cane relay with an unchanged local count do not
add events. An independent cane fall increments its own history separately.

A known next epoch with the rollover bit set accounts for 15 → 0 without resetting the
phone total. A reboot, multiple missed epochs or backward/uninterpretable data keeps
the recorded total but marks history incomplete; the app cannot reconstruct arbitrary
offline loss from a four-bit counter. Future observed increments can still accumulate.
This is a phone-recorded count, never an unconditional lifetime medical statistic.

`PreferencesFallHistoryStore` persists one validated record on counter/epoch changes,
not every heartbeat. Clearing monitoring selection does not erase history; reselection
and process recreation can resume the saved baseline. Selecting the other node of the
same pair does not create a second history for the same address/role. Replacing a device
or changing its address starts a separate record; pair/address scope is not provisioned
user identity. Preferences are written
asynchronously, so sudden process/device failure may lose the latest unsaved transition.
Statistics never acknowledge, clear, relay or inhibit a wearable alarm. Phone-off
operation preserves the wearable delivery/fallback path, even though history may have gaps.

## Pair scope and freshness

- Discovery remains separate from monitoring until the user selects a node.
- Compact monitoring admits the selected address and the opposite node role with
  the matching pair ID. Each role binds one address for the selection's lifetime.
  Other pairs and competing same-role addresses cannot overwrite live cards.
- Legacy data has no pair ID and is scoped to the selected address.
- Each node owns a 16-bit transport epoch, four-bit rolling fall count, heartbeat sequence,
  capture timestamp and link state. Epochs renew before counter reuse; boot and normal
  rollover are distinguished, and the immediately retired epoch cannot reassert. A healthy
  cane update cannot clear a glasses alarm or conceal a stale glasses heartbeat.
- Selection starts in WAITING, not ACTIVE. A fresh accepted packet establishes the
  link; stale reports remain explicitly labeled as historical.
- Android `ScanResult.timestampNanos` and `SystemClock.elapsedRealtime()` share the
  monotonic clock domain. Old/future scan results are rejected, batch results are
  processed in timestamp order, and callbacks from stopped scan generations are ignored.
- A 500 ms freshness deadline is checked every 100 ms. Identical packet repeats do
  not refresh it. The compact three-bit sequence rebases after a 250 ms gap, matching
  the firmware, while changed incident/status fields survive same-sequence wrap.
- Stopping scanning marks all data non-live and cancels phone alerts. Clearing or
  switching selection clears the previous monitoring session.

Android is not a real-time radio controller. OS scheduling, permissions and scan
delivery can produce a freshness warning even when a Pico is still running. A lost
link is presented as unavailable fresh telemetry, not proof of a particular hardware
failure. Pair IDs and address binding provide scope, not cryptographic authentication.

## Phone alerts and foreground service

One immutable monitoring snapshot feeds both the dashboard and foreground service.
The service starts one collector per instance, even if the activity starts it again
on resume. Its ongoing notification updates silently; `SafetyHapticPolicy` is the
sole owner of phone vibration decisions.

Heartbeat updates do not restart a waveform. New incidents and pause-to-alarm
transitions can alert promptly; sustained alerts use bounded reminder intervals
(fall 5 s, critical proximity 2 s, warning 3 s, fault/link loss 10 s). Repeated relay
reports do not restart an already-playing fall waveform. Pause cancels that node's
fall signal without clearing its latch. Another node's independent active fall
continues to take priority.

The phone follows reported pause/recovery, rather than inventing a local 15-second
alarm reset. If telemetry goes stale during an incident, the last fall report stays
visible and link-loss handling takes over. Device selection is session state; the
dashboard requires selection again after process recreation.

## Verification and build

Use JDK 17 with the Gradle 8.4 wrapper and an installed Android SDK. Machine-specific
JDK paths are not stored in `gradle.properties`.

```sh
./gradlew :app:testDebugUnitTest :app:lintDebug :app:assembleDebug
./gradlew :app:assembleDebugAndroidTest
# With an emulator/device available:
./gradlew :app:connectedDebugAndroidTest
```

The JVM tests cover firmware flag vectors, sentinels, legacy compatibility, malformed
AD structures, calibrated hazard interpretation, pair/address isolation, independent
heartbeat deadlines, counter wrap/rebase, scan restart, stale latched incidents,
phone haptic deduplication/pause/re-alarm, exact compact cane receipts, the shared firmware
golden vector, cumulative history above 15, persisted baselines and incomplete-history handling.

Compose instrumentation cases cover paused incidents, held-grip parking, independent
head falls, generic faults, compact sequence versus cumulative history, and large-text scrolling.
They use decoded firmware packet
fixtures and do not simulate real Bluetooth reception. The test APK compiles; running
these cases still requires a usable emulator/device. The local emulator attempt was
blocked by available disk space.

Real-device verification remains for foreground/background scanning, Bluetooth and
permission changes, actual haptic cancellation, and two-Pico packet loss/interference.
