# Cane mode manager

`CaneMode` is a pure, single-owner software component. `Cane_Main.c` supplies debounced
grip, timestamped IMU observations, fall events and fault inputs. It applies outputs
through `ImuSupervisor`, `AlertManager`, `Haptic` and `BleComm`.

Power (`STARTING`, `MONITORING`, `STANDBY`), the incident latch (`CLEAR`, `LATCHED`,
`PAUSED`) and technical fault status are independent. `Sys_State_t` is a derived view:
sensor recovery and ordinary ground hazards cannot overwrite an unresolved fall.
Existing enum values are preserved; `UNGRIPPED_WATCH` and `FAULT_DEGRADED` are appended.

## Behaviour

| Current state | Qualified event | Result |
| --- | --- | --- |
| INIT / waking | IMU settles for 100 ms, then three successful acquisitions | ACTIVE if held; otherwise UNGRIPPED_WATCH |
| ACTIVE / WATCH | Debounced release | UNGRIPPED_WATCH; continue detecting falls silently |
| UNGRIPPED_WATCH | Released, within 25 degrees of upright and still for 3 s; no fall candidate | Request hardware sleep; STANDBY only after readback confirmation |
| UNGRIPPED_WATCH | Regrip | ACTIVE / WATCH; keep any fall evidence already in progress |
| Monitoring | Local confirmed fall or a new peer fall incident | FALL_CONFIRMED; latch the incident |
| FALL_CONFIRMED | Fresh release-to-held transition after the event | FALL_MUTED (15 s bounded audio pause); retain fall telemetry |
| Either fall state | Held, within 30 degrees of upright and still continuously for 1 s | Clear the incident and rearm monitoring |
| FALL_MUTED | Released for 3 s while not upright, or the 15 s pause expires | FALL_CONFIRMED; resume the same incident, without incrementing its sequence |
| FALL_MUTED | After acknowledgement, release while still, then a second held/still interval of 3 s | Request deliberate parking; clear the latch only after hardware sleep succeeds |
| STANDBY | Debounced held grip for 100 ms | Wake the IMU, wait for readiness, then resume monitoring |
| Any awake state | Three consecutive IMU read failures, or a failed init/power command | Technical fault overlay; preserve any fall latch; retry the IMU bus after 1 s |

The grip that completes a parking override is **consumed**: keeping the same hand on
the sensor does not immediately wake the cane. Release and hold again to wake.
Continuous grip during a fall cannot acknowledge or invoke the parking override.
The first acknowledgement hold cannot also be the deliberate parking hold.

There is no automatic “tap and leave still” clearance, no unconditional five-second
disarm and no automatic indefinite mute after 30 seconds. Stillness while released
is not proof that an unresolved incident has ended. A ToF fault remains reported but
does not prevent recovery qualified by a healthy IMU.

Missing/stale IMU observations reset pose qualification. An observation gap over
50 ms resets continuous timers and cancels uncommitted parking. Time comparisons
use unsigned elapsed differences across the 32-bit timer wrap. New fall events take
precedence over acknowledgement/parking in the same cycle. A failed or late sleep
acknowledgement cannot clear the incident. An IMU fault cancels the override gesture;
sensor recovery cannot restart it while the same grip remains held.

## Pose and timing configuration

`CaneMode_Cfg.h` contains the thresholds. Stillness currently means acceleration
magnitude within 0.9–1.1 g and gyro magnitude at most 5 degrees/s. These are initial
prototype values for physical validation, not a clinical classifier.

`CANEMODE_UPRIGHT_X/Y/Z` is the **mounted sensor's gravity direction when the cane is
upright, handle uppermost**. The default is +Z, matching the existing test fixture.
Check the actual mounting before relying on automatic parking/recovery. The signed
dot product rejects an upside-down cane; startup on the floor never recalibrates
the floor pose as “upright”. Ground-distance calibration does not calibrate this axis.

Touch debounce is elapsed-time based (20 ms) and works with the task's actual 10 ms
sampling period. The touch pad must remain powered in standby. Holding the grip for
30 seconds, or longer, is normal operation and is not diagnosed as a stuck sensor.
A single digital touch input cannot reliably distinguish a held hand from every
stuck/wet-pad condition; the separate gesture and bounded pause provide the guards
available to this implementation.

## Power and hardware boundaries

`Mpu9250_SetSleep()` uses MPU-9250 Register Map section 4.34: register `0x6B`, sleep
bit `0x40`. It preserves clock/temperature settings, exits cycle/gyro-standby modes
and checks register readback. Wake does not reset the device or discard gyro bias.
`ImuSupervisor` waits 100 ms and requires three valid acquisitions before readiness.
Asleep, it issues no sensor reads or repeated sleep writes. Failed I/O does not become
healthy simply because a reinitialization call succeeded.

The driver uses only accel/gyro. It never starts the MPU-9250's separate AK8963
magnetometer, whose cold-power default is power-down; MPU-6500 boards have no AK8963.
Chip sleep current is not the breakout-board or Pico W system current. Neither
board-level current savings nor sensor/pin wake latency has been measured here.

BLE remains available during IMU sleep. A remote head fall can still sound the cane
alarm while its IMU is asleep; touch wakes the IMU for local recovery checks.
This is sensor standby, not RP2040 dormant mode or radio power-off. Both cyclic tasks
continue their watchdog handoff during standby.

Glasses consume explicit cane standby status rather than inferring it from a lost
link. Their IMU may sleep after 60 s of observed local stillness with the cane parked
and no head-fall latch or pending fall. Fresh cane wake status, uncertain/lost link,
or the local acknowledgement button restores head monitoring through the same IMU
supervisor. Their radio and radar stream handling stay active. Cane acknowledgement
does not send a blanket command clearing the glasses' independent head-fall latch.

## Audio and telemetry

The cane renders the 3-2-3-2 pulse rhythm cyclically through `AlertManager`; pattern
gaps and audio pause do not clear the latch. Sensor faults use a separate two-pulse
pattern. This specifies prototype timing only, not IEC acoustic conformance or an
ASIL rating. The glasses continue to offload fall audio to the cane.

The 6-byte wire format stays the same. State-byte bits are:

| Bit | Meaning |
| --- | --- |
| `0x01` | Fall incident latched, including during audio pause |
| `0x02` | Sensor / technical fault |
| `0x04` | Obstacle / ground hazard |
| `0x08` | Audio paused |
| `0x10` | Cane touch held |
| `0x20` | IMU standby confirmed |
| `0x40` | Deliberately parked through the post-fall override |

Standby takes precedence over HELD when glasses decide whether ground use is active;
both bits can legitimately be set immediately after a parking override. A new peer
fall is detected even in the first received packet. Its tracker survives transient
link loss, so repeated advertisements cannot cancel a pause or reset its timer.
The incident counter advances for a new local event, not for re-sounding an old one.
Consumers can use the added flags without treating a paused fall as “no incident”.

## Verification

Host checks include transition boundaries, upside-down and moving poses, stale/NaN
samples, timer wrap, stuck-high/long-normal grips, interrupted overrides, failed and
late sleep acknowledgement, degraded recovery, peer-event deduplication, pulse
boundaries, power-register readback and a full drop→pause→park→touch-wake pipeline.
Both node firmware targets are cross-built separately from those host tests.

Physical validation still needs the boards: verify mounted-axis thresholds, touch
debounce and release-before-hold gestures, MPU sleep current/readback, wake settling,
buzzer timing/output level, and two-node state propagation under packet loss.

Register reference: [MPU-9250 Register Map and Descriptions](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf).
