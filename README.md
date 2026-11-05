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
- **Sensor Fusion**: Dynamically projects ToF distance vectors (`VL53L1X` / `VL53L0X`) onto the virtual ground plane using `MPU9250` pitch and roll angle estimates via complementary filtering.
- **Pendulum Compensation**: Rejects false drop-off alerts caused by the user's natural pendulum cane swinging motion during walking.
- **Voting Filter**: Applies a 5-sample median filter and 3-vote threshold logic to detect genuine drop-offs (`STEP_DOWN > +150mm`) or curb rises (`STEP_UP <= -80mm`).
- **Ground Calibration**: Starts on the first active grip when no baseline exists. To recalibrate, grip the cane at walking height over level ground and hold the GP20 button for 2 s (or send `c` over serial). Vibration feedback: 1 short pulse = started, one 1 s pulse = saved, two 1 s pulses = failed (no steady reading within 10 s, sensor fault or cane released; the previous baseline is kept). Without a baseline the gripped cane plays 4 quick ticks every 3 s.

### 2. Dual-Stage Fall Detection Engine
- **Physics Pipeline**: Monitors 3-axis accelerometer/gyroscope magnitude for free-fall ($|a| < 0.35g$ for cane, $< 0.5g$ for head) followed by impact ($|a| > 2.5g$ within 800–1000 ms) and post-impact stillness/tilt verification.
- **Release Watch**: Releasing the touch sensor enters silent `UNGRIPPED_WATCH`. Upright, still and released for 3 continuous seconds permits IMU standby; elapsed release time alone never disarms a pending fall.

### 3. Asymmetric Ear-Safe Emergency Protocol
- **Acoustic Protection**: To avoid disorienting the wearer near their ears, the Smart Glasses buzzer remains silent when a head-drop or fall is detected.
- **Remote Offloading**: The glasses broadcast head-fall incidents to the cane. The cane retains the incident latch, plays a non-blocking 3-2-3-2 buzzer pattern and inhibits ground haptics during the incident. Repeated packets for the same incident do not undo acknowledgement.
- **Acknowledge / Recover / Park**: A fresh grip pauses the cane alarm for at most 15 s without clearing its BLE fall flag. Holding the cane upright and still for 1 s clears the incident. Releasing it while unrecovered for 3 s re-sounds the alarm. To park it horizontally, acknowledge, release, then hold again for 3 s while still. The initial acknowledgement hold cannot perform this override.
- **Real IMU Sleep**: Standby writes and verifies the MPU `PWR_MGMT_1.SLEEP` bit. The touch sensor stays powered; release then hold to wake after an override. Wake-up waits for IMU settling and three valid samples. BLE and the RTOS/watchdog remain running. The glasses retain their own head-fall latch until their local acknowledgement button is pressed.
- **Technical Faults**: Three consecutive failed IMU reads cause degraded operation and bounded bus-recovery retries. An IMU fault cannot qualify upright recovery or overwrite a fall latch. Technical audio uses a distinct two-pulse pattern.

See [cane mode manager behaviour and verification](src/app/CaneMode/README.md) for transitions, mounting configuration and telemetry flags. Alarm timing is a prototype configuration, not an IEC/ASIL certification claim.

### 4. Context-Aware BLE Advertising
- **All-State Policy**: Manufacturer data (company ID `0x0029`) carries a 6-byte status packet. The controller advertising interval is 150 ms in ACTIVE, 200 ms in IDLE/STANDBY, and 100 ms during initialization, release-watch, hazards, either fall state and faults. Fault flags or link loss override any state to the fast profile. Payloads refresh every 50 ms (fast/active) or 100 ms (quiet); state changes publish immediately. Downshifts wait 1 s to avoid controller reconfiguration chatter.
- **Radio Ownership**: The Core 1 BTstack worker owns GAP/HCI operations and its host-wake GPIO IRQ. Task callbacks perform parsing/configuration; the hardware IRQ only signals deferred work. Passive scanning stays available during IMU standby.
- **Link Recovery**: Link loss requests the fast advertising/full-scan profile. Actual recovery latency depends on RF conditions and is a bench-validation item.
- **Freshness Filtering**: Rejects identical repeated snapshots and stale application-queue entries; short heartbeat sequences rebase after a reception gap. The audit documents the ordering limits.

### 5. Link-Loss Deadman Watchdog
- **Safety Degradation**: If the Cane scanner misses periodic broadcast packets from the Glasses for $> 500\,\text{ms}$, a software watchdog triggers a localized 2 long-pulse haptic warning (`500ms ON / 200ms OFF`), informing the user that head-level obstacle tracking is offline while keeping local ground sensing active.

### 6. mmWave Radar Head-Level Threat Assessment
- **HLK-LD2450 Processing**: UART at 256,000 baud feeds an aligned 1024-byte DMA ring. A private completion ISR acknowledges and re-arms each 30-byte block; the 20 ms task parses committed data with bounded work, partial-frame resynchronization and overrun detection.
- **Target Tracking**: Tracks up to 3 targets, calculates closing speed and Time-To-Collision (TTC), and ignores departing targets ($> 10\,\text{cm/s}$ moving away).
- **Pitch-Modulated Beeping**: Maps threat proximity ($2000\,\text{mm}$ to $300\,\text{mm}$) with $100\,\text{mm}$ hysteresis to a frequency-modulated beep pattern ($1500\,\text{Hz}$ to $3500\,\text{Hz}$).

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

The Android companion application (`app/`) connects to the EchoGuard system to display real-time telemetry, threat alerts, and fall notifications.

```bash
# Build debug APK via Gradle wrapper
./gradlew :app:assembleDebug

# Deploy debug build to connected Android device
./gradlew :app:installDebug
```

---
