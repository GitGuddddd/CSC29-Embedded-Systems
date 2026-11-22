# EchoGuard: Smart Mobility Aid & eWalker Assistance System

**Group 29 | INF2004 Real-Time Embedded Systems**

A dual-node real-time embedded assistance system designed for visually impaired users. Comprising a **Smart Walking Cane Node** (`Cane_Node`) and a **Smart Glasses Node** (`Glasses_Node`) built on Raspberry Pi Pico W (RP2040) microcontrollers, the system integrates ground drop/cliff detection, mmWave radar head-level obstacle assessment, multi-stage fall detection, ear-safe asymmetric alerting, and an Android companion mobile application.

---

## Architecture Overview

Uses **FreeRTOS SMP with task affinity** across the RP2040's dual ARM Cortex-M0+ cores:
- **Core 0**: Dedicated to hard real-time sensor polling (100 Hz IMU loops, ToF ground projection, mmWave radar parsing, and actuator PWM execution).
- **Core 1**: Runs the radio service for BLE advertising and scanning. The cores still share memory, buses and the scheduler.

---

## Key Features & Algorithms

### 1. Virtual Ground Plane Projection

- **Distance correction** uses raw ToF ranges until the beam is aligned. After alignment, the firmware multiplies each slanted range by the cosine of the beam's angle from vertical. Ground calibration and step detection both use that vertical distance.
- **Angle cutoff** suspends ground alerts when the beam exceeds the provisional 35° limit or the IMU cannot provide a trustworthy angle. Detection resumes at 30° or less with a valid angle. The firmware tracks the beam at the IMU sample rate using the gyroscope and corrects the estimate with reliable accelerometer readings.
- **Step detection** uses a five-sample median and vote confirmation for drop-offs (`STEP_DOWN`, more than 150 mm below the baseline) and rises (`STEP_UP`, at least 80 mm above it).

#### Bench setup

1. Aim the mounted ToF beam straight down and hold the cane still. Send `v` over USB serial to capture roughly half a second of stationary IMU samples within a five-second window.
2. After alignment prints the measured X, Y and Z accelerometer values and clears the old ground baseline, hold the cane at its normal walking angle over level ground. Send `c`, hold the GP20 calibration button for two seconds, or release and regrip to calibrate the vertical distance baseline. A tone signals the start and result of calibration.
3. Send `d` to toggle per-sample logs of the raw range, vertical distance and beam angle. With per-sample logs off, `UNKNOWN` and `GROUND_UNAVAILABLE` appear after 300 ms of stability. Before choosing a cutoff, bench-test a flat floor, known steps up and down, and typical cane swing. Check for missed steps during out-of-angle periods as well as false alerts.

Serial alignment lasts until reboot. To use a measured alignment at boot, put the printed values in `CANE_TOF_DOWN_ACCEL_X`, `CANE_TOF_DOWN_ACCEL_Y` and `CANE_TOF_DOWN_ACCEL_Z`, then rebuild the cane firmware. [Cane board configuration](config/Cane_Board.h)

### 2. Fall and slip detection

- The current cane build uses the lateral-slip-only bench trial. A filtered lateral acceleration of at least 0.55 g for 30 ms logs a possible `SLIP`. A slip that leaves the cane at least 40° from its starting attitude and still for 250 ms raises the cane's fall alarm. Vertical motion and straight drops are excluded from this trial. Smooth sliding at constant speed can be invisible to an IMU.
- The cane establishes world up from three consistent quiet accelerometer samples and tracks it with the gyro during motion. A three-sample median filters signed vertical and horizontal acceleration. Vertical readings remain available for diagnostics during the trial.
- Set `FEATURE_CANE_SLIP_ONLY_TRIAL` to `0U` in `config/Feature_Cfg.h` and rebuild to restore the full cane policy. That policy adds downward motion with rotation as slip evidence and uses sustained low g and landing impact to detect drops. The drop path requires extra evidence after a recent lift.
- The glasses use their own `FallDetection` state machine for free fall, impact and post-impact verification.

The cane perception task owns the motion state machine and steps it with each fresh IMU sample. The lateral-slip-only build uses these transitions.

| Current state | Event and guard | Action | Next state |
|---|---|---|---|
| `IDLE` | Valid world-up estimate and lateral acceleration of at least 0.55 g | Capture the starting attitude and start the slip timer | `SLIP_START` |
| `SLIP_START` | Lateral acceleration stays at or above the threshold for 30 ms | Emit `SLIP` and start the confirmation window | `SLIP_TRACK` |
| `SLIP_START` | Lateral evidence ends before qualification | Clear the candidate | `IDLE` |
| `SLIP_TRACK` | The cane tilts about 40° or more from the starting attitude and stays still for 250 ms within the confirmation window | Emit `FALL` to the cane mode manager, which latches the alarm | `IDLE` |
| `SLIP_TRACK` | The 1.5 s confirmation window expires | Clear the candidate | `IDLE` |
| Any state | Missing or invalid IMU sample, or sample gap greater than 25 ms | Reset the motion evidence and world-up estimate | `IDLE` |

Send `f` over the cane's USB serial port to toggle the 20 Hz `[MOTION]` stream. Events and state changes also log with the stream off. `vert_g` and `horiz_g` show filtered acceleration. `vert_raw_g` and `horiz_raw_g` show the corresponding raw projections. Positive vertical acceleration points up, and `up_valid=0` means the direction estimate is unavailable. Compare normal lifts, slips in several directions, and controlled drops on a padded bench before relying on the prototype thresholds.

### 3. Asymmetric Local Emergency Protocol
- **Acoustic Protection**: The Smart Glasses never play the cane's emergency siren near the wearer's ears. If the cane does not return an application receipt within 750 ms, or disappears after receipt, the glasses use a sparse 100 ms / 2 s cue at 10% PWM duty. Final enclosure SPL still requires bench validation; PWM duty is not a certification claim.
- **Remote Offloading**: The glasses broadcast an epoch-scoped four-bit head-fall sequence. The cane returns a receipt only after its application latches the matching event and issues the alarm request, then plays a non-blocking 3-2-3-2 pattern and inhibits ground haptics. A transport receipt, user audio pause and incident clearance remain separate operations.
- **Watchdog Reset Retention**: Each wearable records only unresolved-incident transitions in RP2040 watchdog scratch words. A timeout reset restores the latch under a new boot session; a retained cane pause resumes audible, and a restored glasses event must obtain a fresh cane receipt. Power loss, reset-pin resets and firmware-loading reboots deliberately do not restore this volatile record.
- **Acknowledge / Recover / Park**: A fresh grip pauses the cane alarm for at most 15 s without clearing its BLE fall flag. Holding the cane upright and still for 1 s clears the incident. Releasing it while unrecovered for 3 s re-sounds the alarm. To park it horizontally, acknowledge, release, then hold again for 3 s while still. The initial acknowledgement hold cannot perform this override.
- **Real IMU Sleep**: Standby writes and verifies the MPU `PWR_MGMT_1.SLEEP` bit. The touch sensor stays powered; release then hold to wake after an override. Wake-up waits for IMU settling and three valid samples. BLE and the RTOS/watchdog remain running. The glasses retain their own head-fall latch until their local acknowledgement button is pressed.
- **Technical Faults**: Three consecutive failed IMU reads cause degraded operation and bounded bus-recovery retries. An IMU fault cannot qualify upright recovery or overwrite a fall latch. Technical audio uses a distinct two-pulse pattern.
- **Staged IMU Bring-Up/Recovery**: Startup and fault recovery perform one initialization register action per perception cycle. The required 50 ms reset wait and 100 ms settling interval are elapsed-time gates anchored after I/O completion, not task sleeps; three valid samples still qualify readiness. Grip, BLE and alarm work continues during these waits. Verified sleep/wake remains separate and does not reset gyro calibration. Individual I2C transactions are still bounded polling, not a measured scheduling guarantee.

See [cane mode manager behaviour and verification](src/app/CaneMode/README.md) for transitions, mounting configuration and telemetry flags. Alarm timing is a prototype configuration, not an IEC/ASIL certification claim.

### 4. Context-Aware BLE Advertising
- **BLE-Only Runtime**: The node targets use the FreeRTOS CYW43 architecture without lwIP/TCP-IP. `BleRadio` supplies its own Core 1 async context and 4 KB worker stack; the unused SDK default stack is disabled, not the actual radio worker. Sampling, scan duty, receipt deadlines and watchdog supervision are unchanged. Memory/build savings do not establish hardware, power or RF validation.
- **All-State Policy**: Manufacturer data (company ID `0x0029`) carries the original 6-byte status plus a 6-byte v2 receipt extension: a 16-bit transport epoch, 16-bit receipted epoch, and packed receipt sequence/validity/rollover flags. The event sequence already occupies four bits in the status prefix; no lifetime counter is stored or duplicated. The complete primary advertisement is 19 bytes, not the 31-byte ceiling; the full name remains in the scan response. Epochs renew on reboot and before the fall sequence wraps, but are finite prototype tags, not authentication. The controller advertising interval is 150 ms in ACTIVE, 200 ms in IDLE/STANDBY, and 100 ms during initialization, release-watch, hazards, either fall state and faults. Fault flags or link loss override any state to the fast profile. Payloads refresh every 50 ms (fast/active) or 100 ms (quiet); state/receipt changes publish immediately. Downshifts wait 1 s to avoid controller reconfiguration chatter.
- **Phone-Owned History**: Android persists per-node cumulative fall increments from accepted compact telemetry, including a known 15-to-0 rollover. Heartbeats, audio pauses and a cane relay do not increment its local fall count. The phone labels incomplete history after a reboot, multiple missed epochs or an unknown initial baseline; it does not invent an exact lifetime total or become a mandatory safety relay.
- **Radio Ownership**: The Core 1 BTstack worker owns GAP/HCI operations and its host-wake GPIO IRQ. Task callbacks perform parsing/configuration; the hardware IRQ only signals deferred work. Passive scanning stays available during IMU standby.
- **Link Recovery**: Link loss requests the fast advertising/full-scan profile. Actual recovery latency depends on RF conditions and is a bench-validation item.
- **Freshness Filtering**: Rejects identical repeated snapshots and stale application-queue entries; short heartbeat sequences rebase after a reception gap. The audit documents the ordering limits.

### 5. Link-Loss Deadman Watchdog
- **Safety Degradation**: If the Cane scanner misses periodic broadcast packets from the Glasses for $> 500\,\text{ms}$, a software watchdog triggers a localized 2 long-pulse haptic warning (`500ms ON / 200ms OFF`), informing the user that head-level obstacle tracking is offline while keeping local ground sensing active.

### 6. mmWave Radar Head-Level Threat Assessment
- **Header-First Parsing**: The task peeks four header bytes before copying a full 30-byte radar candidate. Noise rejection retains the same DMA overwrite checks, ring size, parser budget and frame validation. Native benchmark improvements are not a hardware latency or power claim.
- **HLK-LD2450 Processing**: UART at 256,000 baud feeds an aligned 1024-byte DMA ring. A private completion ISR acknowledges and re-arms each 30-byte block; the 20 ms task parses committed data with bounded work, partial-frame resynchronization and overrun detection. Each target stays in its own radar slot (an empty slot is returned as not valid rather than shifting later targets down), and a slot holding a physically impossible value (|x| > 6.5 m, y outside 0–8.5 m or |speed| > 10 m/s) is rejected and flagged (`LD2450_CFG_MAX_*` in `Ld2450_Cfg.h`).
- **Bounded TTC**: Time-to-collision uses the same 12-bit, 20 ms tick representation internally and on BLE. Values beyond 81.88 s, non-positive closing speeds and unavailable targets use `0xFFF`, rather than wrapping a 16-bit millisecond value into a false urgent estimate. An unavailable TTC does not suppress the distance-based obstacle alert.
- **Target Tracking**: Evaluates up to 3 targets per frame into left/centre/right lanes (1500 mm corridor, 425 mm side deadzone with 150 mm hysteresis). Each lane is BLOCKED at $\le 3000\,\text{mm}$ and held until clear of $3100\,\text{mm}$ (derived from Tourki et al., arXiv 2506.16219: warn about 3 s before a 1 m safety bubble at assistive walking speeds). After its target is lost, a lane stays blocked for 600 ms, or 1.5 s if the target was standing still when last seen (the LD2450 drops still people; someone who leaves a lane has to move first, so they still clear after 600 ms) (`THREAT_LANE_HOLD_MS`, `THREAT_STILL_*`). The radar sometimes hands a slot to a different person; a jump larger than a person could move between frames resets that slot's history so it is not mistaken for a crossing (`THREAT_TRACK_*`). Closing speed uses positive raw speed = approaching (selectable via `THREAT_NEGATIVE_SPEED_IS_APPROACHING`; confirm direction with a walk-toward test); TTC holds the last usable closing speed for 200 ms across brief speed-reading dropouts. BLE carries the soonest TTC across blocked lanes (6-byte packet unchanged).
- **Crossing Filter**: A person only crossing the user's path is not counted while beyond the STOP zone (about 1600 mm): a target whose smoothed sideways speed is at least 500 mm/s and more than twice its forward speed is skipped (`THREAT_CROSSING_*` in `ThreatAssessment_Cfg.h`); inside the STOP zone every target counts. If the radar is fitted the other way round, set `THREAT_MIRROR_X` to swap left and right.
- **Speed-Scaled STOP**: STOP ("about to collide") is the centre lane blocked within a distance that grows with closing speed: $D = 500 + v \cdot 0.85\,\text{s} + v^2 / (2 \cdot 0.8\,\text{m/s}^2)$, floored at 1500 mm and capped at 2800 mm (0.8 m/s → 1580 mm, 1.0 m/s → 1975 mm, 1.4 m/s or faster → 2800 mm). $D$ is fixed when STOP latches and STOP clears 100 mm beyond it. It covers all three lanes blocked and never triggers for side-lane-only obstacles. The reaction (0.5 s), system delay (0.35 s) and braking (0.8 m/s²) figures are estimates to be tuned on walking tests (`THREAT_STOP_*`).
- **Departing Obstacles**: The buzzer stays quiet while every blocked lane's obstacle is moving away: at least 16 cm/s away for 3 radar frames and the gap grown by at least 150 mm since it started moving away (speed jitter on a still person does not count). STOP is never silenced within 500 mm. The lanes stay blocked and the beeps return as soon as one obstacle stops or turns back (`THREAT_RECEDE_*`, `ThreatAssessment_IsReceding()`).
- **Pitch-Modulated Beeping**: Maps threat proximity ($3000\,\text{mm}$ to $300\,\text{mm}$) to beep pitch ($1500\,\text{Hz}$ to $3500\,\text{Hz}$), using a low-passed distance (about 250 ms) so the pitch does not jump with radar noise.
- **Direction Cue Pattern**: the cue byte means "which way is open", measured from the glasses (lanes move with the head). An obstacle only to a side (centre clear) is a 1-beep burst, "move left" is a 2-beep burst, "move right" is a 3-beep burst (150 ms on / 150 ms off), repeated after a pause that shrinks from 450 ms at 3000 mm to 150 ms at 300 mm (at least 280 ms for the 2- and 3-beep bursts so they stay countable). A side lane is only offered as free after it has been clear for 500 ms; if neither side is confirmed free, 4 short beeps (110 ms on / 90 ms off) play ("no free lane"), and their pause stretches to 1.5 s if the pattern keeps playing for more than 5 s (crowds). STOP plays as a continuous 3500 Hz tone for as long as it lasts, followed by at least 400 ms of silence before any beeps. Loudness stays at the fixed ear-safe 12% duty; distance does not change duty.
- **Pattern Changes**: A changed cue must persist for 250 ms before it is used (a rising STOP applies at once). A burst that has started always finishes, so a count is never cut short (a 3-beep "move right" can never be heard as 2 beeps), then 400 ms of silence separates it from the new pattern. When the obstacle clears, the last beep plays to its full length.
- **Minimum-Bit State and Filter Flags**: All radar decision state is packed into bit-fields sized to each value's range, with compile-time checks (for example STOP state, moving-away counters, per-slot lanes and filter flags share one 32-bit word). Each filtering step sets one bit of a shared filter bitmask (`THREAT_FLT_*`): outside corridor `0x01`, crossing `0x02`, impossible value `0x04`, slot re-ordered `0x08`, moving away `0x10`, still-person hold `0x20`.
- **Serial Debug (glasses USB serial)**: `v` prints the lanes, decision and buzzer in plain words whenever the decision changes (and why the buzzer is quiet: standby, cane not held, radar timeout or obstacle moving away). `d` prints a readable block twice a second: decision and buzzer, lanes, nearest obstacle with closing speed and time to collision, each radar slot (metres ahead, metres left/right, moving or still, and which lane it went to or why it was ignored), the filters that acted, cane grip and radar state. `r` prints the compact one-line format at 10 Hz for logging (ending in `flt=0xNN`).

See [BLE and interrupt design audit](docs/ble-interrupt-audit.md) for the complete state table,
polling/DMA/IRQ rationale, ISR and IVT ownership, BARR-C checks and measurement boundaries.

---

## Hardware Pin Allocation

### Smart Walking Cane Node (`Cane_Node` - Pico 1)

| Peripheral / Module | Pin / Interface | Signal Name | Description / Owner SWC |
|---------------------|-----------------|-------------|--------------------------|
| **MPU9250 IMU** | I2C0 (GP4 SDA / GP5 SCL) | `I2C0_SDA` / `SCL` | 9-DOF IMU for fall detection & pose (`FallDetection`) |
| **VL53L1X ToF** | I2C1 (GP6 SDA / GP7 SCL) | `I2C1_SDA` / `SCL` | Downward distance measurement (`GroundProjection`) |
| **Capacitive Touch** | GPIO GP14 | `TOUCH_IN` | Digital grip detection (`TouchSensor`) |
| **Vibration Motor** | PWM GP15 (Slice 7 Ch B) | `HAPTIC_PWM` | Haptic warning feedback (`Haptic`) |
| **Calibration Button** | GPIO GP20 (active-low, pull-up) | `CAL_BTN` | Hold 2 s to calibrate the ground baseline (`CalButton`) |
| **Piezo Siren** | PWM GP18 (Slice 1 Ch A) | `SIREN_PWM` | Emergency acoustic alarm (`AlertManager`) |

### Smart Glasses Node (`Glasses_Node` - Pico 2)

| Peripheral / Module | Pin / Interface | Signal Name | Description / Owner SWC |
|---------------------|-----------------|-------------|--------------------------|
| **MPU6500 IMU** | I2C1 (GP2 SDA / GP3 SCL) | `I2C1_SDA` / `SCL` | Head IMU for head-drop detection (`FallDetection`) |
| **HLK-LD2450 Radar** | UART0 (GP0 TX / GP1 RX) | `UART0_TX` / `RX` | 24GHz mmWave radar @ 256k baud + DMA (`Ld2450`) |
| **Passive Buzzer** | PWM GP18 (Slice 1 Ch A) | `BUZZER_PWM` | Proximity & fault audio beeps (`AlertManager`) |

---

## Directory Structure

```text
.
├── CMakeLists.txt                  # Root CMake configuration (Target Firmware & Host Tests)
├── Makefile                        # Convenience make tasks
├── docs/                           # Architecture & traceability documentation
│   ├── requirements_traceability.md# RTM mapping FRs/NFRs to SWCs and tests
│   ├── deviations.md               # Architectural deviation records
│   └── soup_risk_assessment.md     # Software of Unknown Provenance risk analysis
├── src/                            # Embedded C Firmware Source Code
│   ├── app/                        # Software Components (SWCs) & Main Entrypoints
│   │   ├── Cane_Main.c             # Smart Cane main loop & task scheduler
│   │   ├── Glasses_Main.c          # Smart Glasses main loop & task scheduler
│   │   ├── AlertManager/           # Centralized alarm arbitration & pitch mapping
│   │   ├── FallDetection/          # Free-fall & impact detection state machine
│   │   ├── GroundProjection/       # IMU-compensated ground plane projection
│   │   ├── radar_monitor/          # mmWave radar monitor & supervision state machine
│   │   └── threat_assessment/      # mmWave radar threat classification, corridor gate & TTC
│   ├── drivers/                    # Hardware Peripheral Drivers
│   │   ├── Buzzer/                 # Piezo buzzer PWM frequency driver
│   │   ├── CalButton/              # Cane calibration button (GP20) debounce & 2 s hold
│   │   ├── Haptic/                 # Vibration motor pulse pattern driver & non-blocking arbiter
│   │   ├── Ld2450/                 # Radar frame parser and task-level DMA stream consumer
│   │   ├── Mpu9250/                # MPU9250 / MPU6500 IMU I2C driver
│   │   ├── TouchSensor/            # Capacitive touch sensor debouncer
│   │   ├── Vl53l0x/                # VL53L0X ToF laser distance driver
│   │   └── Vl53l1x/                # VL53L1X ToF long-range distance driver
│   ├── hal/                        # Hardware Abstraction Layer (I2C, UART, GPIO, PWM, Timer)
│   ├── rte/                        # Runtime Environment (Rte.c & Rte_Type.h)
│   └── services/                   # System Services
│       ├── BleComm/                # BLE packet formatting & sequence tracking
│       └── Watchdog/               # 500ms link-loss deadman watchdog
├── tests/                          # Unit, Integration, Fuzz & Harness Tests
│   ├── harness/                    # Test mocks & frame builders
│   ├── integration/                # Pipeline & failure scenario integration tests
│   ├── unit/                       # Unit tests (GroundProjection, Haptic, Threat, etc.)
│   └── fuzz/                       # Fuzzing harnesses
└── app/                            # Android Companion Application
    ├── build.gradle.kts            # Android Gradle build configuration
    └── src/main/java/com/cliftonzg/echoguard/ # Kotlin, Jetpack Compose, Hilt, BLE
```

---

## Build & Deployment Instructions

### Prerequisites

1. **Embedded Firmware Build Requirements**:
   - CMake (v3.13 or higher)
   - ARM Embedded Toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-g++`, `newlib`)
   - Raspberry Pi Pico SDK (v2.0.0 or higher) set via `PICO_SDK_PATH`
2. **Android Application Build Requirements**:
   - Android Studio Ladybug or newer
   - JDK 17
   - Android SDK (API 34 / 35)

---

### 1. Building Embedded Firmware (RP2040 Cross-Compilation)

To compile the `Cane_Node.uf2` and `Glasses_Node.uf2` binaries:

```bash
# Clone repository and set up build directory
mkdir build_firmware && cd build_firmware

# Configure CMake targeting Pico W board
cmake -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico_w ..

# Compile binaries
make -j$(nproc) Cane_Node Glasses_Node
```

#### Outputs Generated:
- `build_firmware/Cane_Node.uf2` - Firmware for the Smart Walking Cane.
- `build_firmware/Glasses_Node.uf2` - Firmware for the Smart Glasses.

#### Flashing to Raspberry Pi Pico W:
1. Disconnect the Pico W from USB power.
2. Hold down the onboard **BOOTSEL** button and connect the USB cable to your host PC.
3. Release **BOOTSEL**; the Pico W will mount as a USB Mass Storage device (`RPI-RP2`).
4. Copy `Cane_Node.uf2` or `Glasses_Node.uf2` into the mounted drive.
5. The device will automatically reboot and start executing the firmware.

### 2. Building & Deploying the Android Companion Mobile App

The Android companion (`app/`) observes EchoGuard broadcasts. Select either compact node
to monitor its pair, with separate cane/glasses heartbeat and incident status. The UI
distinguishes paused falls, deliberate parking, sensor sleep, generic faults and stale
telemetry. See [Android telemetry behavior](docs/android-telemetry.md).

```bash
# Use JDK 17; validate and build via the Gradle wrapper
./gradlew :app:testDebugUnitTest :app:lintDebug :app:assembleDebug

# Deploy debug build to connected Android device
./gradlew :app:installDebug

# UI regression tests on a connected device/emulator
./gradlew :app:connectedDebugAndroidTest
```

---
