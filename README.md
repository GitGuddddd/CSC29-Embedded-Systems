# EchoGuard: Smart Mobility Aid & eWalker Assistance System

**Group 29 | INF2004 Real-Time Embedded Systems**

A dual-node real-time embedded assistance system designed for visually impaired users. Comprising a **Smart Walking Cane Node** (`Cane_Node`) and a **Smart Glasses Node** (`Glasses_Node`) built on Raspberry Pi Pico W (RP2040) microcontrollers, the system integrates ground drop/cliff detection, mmWave radar head-level obstacle assessment, multi-stage fall detection, ear-safe asymmetric alerting, and an Android companion mobile application.

---

## Architecture Overview

Uses an **Asymmetric Multiprocessing (AMP)** model across the RP2040's dual ARM Cortex-M0+ cores:
- **Core 0**: Dedicated to hard real-time sensor polling (100 Hz IMU loops, ToF ground projection, mmWave radar parsing, and actuator PWM execution).
- **Core 1**: Dedicated to the Infineon CYW43439 SPI radio stack, managing connectionless BLE periodic advertising and scanning without introducing timing jitter into Core 0 loops.

---

## Key Features & Algorithms

### 1. Virtual Ground Plane Projection
- **Sensor Fusion**: Dynamically projects ToF distance vectors (`VL53L1X` / `VL53L0X`) onto the virtual ground plane using `MPU9250` pitch and roll angle estimates via complementary filtering.
- **Pendulum Compensation**: Rejects false drop-off alerts caused by the user's natural pendulum cane swinging motion during walking.
- **Voting Filter**: Applies a 5-sample median filter and 3-vote threshold logic to detect genuine drop-offs (`STEP_DOWN > +150mm`) or curb rises (`STEP_UP <= -80mm`).

### 2. Dual-Stage Fall Detection Engine
- **Physics Pipeline**: Monitors 3-axis accelerometer/gyroscope magnitude for free-fall ($|a| < 0.35g$ for cane, $< 0.5g$ for head) followed by impact ($|a| > 2.5g$ within 800–1000 ms) and post-impact stillness/tilt verification.
- **Release Watch Window**: When the user lets go of the cane (capacitive touch released), a 1000 ms watch window opens to detect falls while preventing false alarms during deliberate cane placement.

### 3. Asymmetric Ear-Safe Emergency Protocol
- **Acoustic Protection**: To avoid disorienting the wearer near their ears, the Smart Glasses buzzer remains silent when a head-drop or fall is detected.
- **Remote Offloading**: The glasses offload emergency alerts via connectionless BLE broadcasts to the Cane, which latches a high-decibel piezo siren and haptic motor vibration.
- **Reset Logic**: Holding the cane grip continuously for > 3 seconds or pressing the onboard acknowledge button sends a `CANCEL` command over BLE, silencing alarms on both nodes.

### 4. Connectionless BLE Periodic Advertising (ADV_NONCONN_IND)
- **Torso Attenuation Survival**: Replaces standard paired GATT connections with stateless $10\,\text{Hz}$ non-connectable advertising carrying an 11-byte `AdvancedThreatPacket` (`0xFF29` manufacturer ID).
- **Fast Recovery**: Provides $\sim 72\,\text{ms}$ average recovery from human body shadowing ($\approx 22\times$ faster than GATT reconnection timeouts).
- **Monotonic Sequence Filtering**: Rejects delayed, duplicate, or stale packets escaping RF baseband buffers during interference bursts.

### 5. Link-Loss Deadman Watchdog
- **Safety Degradation**: If the Cane scanner misses periodic broadcast packets from the Glasses for $> 500\,\text{ms}$, a software watchdog triggers a localized 2 long-pulse haptic warning (`500ms ON / 200ms OFF`), informing the user that head-level obstacle tracking is offline while keeping local ground sensing active.

### 6. mmWave Radar Head-Level Threat Assessment
- **HLK-LD2450 Processing**: Ingests 24GHz mmWave radar telemetry over UART at 256,000 baud via RP2040 hardware DMA directly into a circular buffer.
- **Target Tracking**: Tracks up to 3 targets, calculates closing speed and Time-To-Collision (TTC), and ignores departing targets ($> 10\,\text{cm/s}$ moving away).
- **Pitch-Modulated Beeping**: Maps threat proximity ($2000\,\text{mm}$ to $300\,\text{mm}$) with $100\,\text{mm}$ hysteresis to a frequency-modulated beep pattern ($1500\,\text{Hz}$ to $3500\,\text{Hz}$).

---

## Hardware Pin Allocation

### Smart Walking Cane Node (`Cane_Node` - Pico 1)

| Peripheral / Module | Pin / Interface | Signal Name | Description / Owner SWC |
|---------------------|-----------------|-------------|--------------------------|
| **MPU9250 IMU** | I2C0 (GP4 SDA / GP5 SCL) | `I2C0_SDA` / `SCL` | 9-DOF IMU for fall detection & pose (`FallDetection`) |
| **VL53L1X ToF** | I2C1 (GP6 SDA / GP7 SCL) | `I2C1_SDA` / `SCL` | Downward distance measurement (`GroundProjection`) |
| **Capacitive Touch** | GPIO GP14 | `TOUCH_IN` | Digital grip detection (`TouchSensor`) |
| **Vibration Motor** | PWM GP15 (Slice 7 Ch A) | `HAPTIC_PWM` | Haptic warning feedback (`Haptic`) |
| **Piezo Siren** | PWM GP18 (Slice 1 Ch A) | `SIREN_PWM` | Emergency acoustic alarm (`AlertManager`) |

### Smart Glasses Node (`Glasses_Node` - Pico 2)

| Peripheral / Module | Pin / Interface | Signal Name | Description / Owner SWC |
|---------------------|-----------------|-------------|--------------------------|
| **MPU6500 IMU** | I2C1 (GP2 SDA / GP3 SCL) | `I2C1_SDA` / `SCL` | Head IMU for head-drop detection (`FallDetection`) |
| **HLK-LD2450 Radar** | UART0 (GP0 RX / GP1 TX) | `UART0_RX` / `TX` | 24GHz mmWave radar @ 256k baud + DMA (`Ld2450`) |
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
│   │   ├── Haptic/                 # Vibration motor pulse pattern driver & non-blocking arbiter
│   │   ├── Ld2450/                 # HLK-LD2450 mmWave radar driver & UART stream buffer
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
