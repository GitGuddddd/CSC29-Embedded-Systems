cmake_minimum_required(VERSION 3.13)

# -------------------------------------------------------------
# Smart Mobility Aid (Group 29) - Root CMakeLists.txt
# Supports both:
#   1. Target Cross-Compilation (RP2040 Pico SDK -> Cane & Glasses .uf2 binaries)
#   2. Host Automated Verification (CTest / Unity / Fuzzers)
# -------------------------------------------------------------

# Option to build host tests or target firmware
option(BUILD_TESTS "Build host unit, integration, and fuzz test suite" OFF)

# Check if PICO_SDK_PATH is set (environment or CMake cache)
if (DEFINED ENV{PICO_SDK_PATH} AND NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if (NOT DEFINED PICO_SDK_PATH AND EXISTS "/Users/ironite/pico-sdk")
    set(PICO_SDK_PATH "/Users/ironite/pico-sdk")
endif()

# Check if we are building for RP2040 target
if (NOT BUILD_TESTS AND PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(STATUS "[Build Target] Raspberry Pi Pico SDK detected: ${PICO_SDK_PATH}")
    include(${CMAKE_CURRENT_SOURCE_DIR}/soup/pico_sdk/pico_sdk_import.cmake)
    project(SmartMobilityAid_AUTOSAR C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    pico_sdk_init()
    set(BUILD_FOR_PICO_TARGET TRUE)
else()
    message(STATUS "[Build Host] Building in Host Test/Simulation mode (Native Clang/GCC)")
    project(SmartMobilityAid_AUTOSAR C CXX)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(BUILD_FOR_PICO_TARGET FALSE)
endif()

# Common include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rte
    ${CMAKE_CURRENT_SOURCE_DIR}/soup
    ${CMAKE_CURRENT_SOURCE_DIR}/soup/unity/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/harness
)

# =============================================================
# 1. RP2040 Target Firmware Binaries (.elf / .uf2 / .bin)
# =============================================================
if (BUILD_FOR_PICO_TARGET)
    add_compile_definitions(PICO_ON_DEVICE=1)

    # Common Firmware Sources for Cane Node
    set(CANE_SOURCES
        src/app/Cane_Main.c
        src/app/FallDetection/FallDetection.c
        src/app/FallDetection/FallDetection_Lcfg.c
        src/app/GroundProjection/GroundProjection.c
        src/app/AlertManager/AlertManager.c
        src/drivers/Mpu9250/Mpu9250.c
        src/drivers/Mpu9250/Mpu9250_Lcfg.c
        src/drivers/Vl53l1x/Vl53l1x.c
        src/drivers/Vl53l1x/Vl53l1x_Lcfg.c
        src/drivers/Vl53l0x/Vl53l0x.c
        src/drivers/Vl53l0x/Vl53l0x_Lcfg.c
        src/drivers/TouchSensor/TouchSensor.c
        src/drivers/TouchSensor/TouchSensor_Lcfg.c
        src/drivers/Haptic/Haptic.c
        src/drivers/Haptic/Haptic_Lcfg.c
        src/drivers/Buzzer/Buzzer.c
        src/drivers/Buzzer/Buzzer_Lcfg.c
        src/services/BleComm/BleComm.c
        src/services/BleComm/BleComm_Lcfg.c
        src/services/Watchdog/Watchdog.c
        src/services/Watchdog/Watchdog_Lcfg.c
        src/hal/Hal_I2c.c
        src/hal/Hal_Gpio.c
        src/hal/Hal_Pwm.c
        src/rte/Rte.c
    )

    add_executable(Cane_Node ${CANE_SOURCES})
    target_link_libraries(Cane_Node PRIVATE
        pico_stdlib
        pico_multicore
        pico_cyw43_arch_none
        pico_btstack_cyw43
        pico_btstack_ble
        hardware_i2c
        hardware_pwm
        hardware_gpio
        hardware_clocks
        m
    )
    pico_enable_stdio_usb(Cane_Node 1)
    pico_enable_stdio_uart(Cane_Node 0)
    pico_add_extra_outputs(Cane_Node)
    pico_set_program_name(Cane_Node "EchoGuard Smart Cane")
    pico_set_program_version(Cane_Node "1.0.0")

    # Dedicated Standalone ToF Hardware Diagnostic Binary
    add_executable(Test_Tof src/app/Test_Tof_Main.c)
    target_link_libraries(Test_Tof PRIVATE
        pico_stdlib
        hardware_i2c
        hardware_gpio
    )
    pico_enable_stdio_usb(Test_Tof 1)
    pico_enable_stdio_uart(Test_Tof 0)
    pico_add_extra_outputs(Test_Tof)

    # Common Firmware Sources for Glasses Node
    set(GLASSES_SOURCES
        src/app/Glasses_Main.c
        src/app/FallDetection/FallDetection.c
        src/app/FallDetection/FallDetection_Lcfg.c
        src/app/ThreatAssessment/ThreatAssessment.c
        src/app/AlertManager/AlertManager.c
        src/drivers/Mpu9250/Mpu9250.c
        src/drivers/Mpu9250/Mpu9250_Lcfg.c
        src/drivers/Ld2450/Ld2450.c
        src/drivers/Ld2450/Ld2450_Lcfg.c
        src/drivers/Ld2450/Ld2450_Irq.c
        src/drivers/Buzzer/Buzzer.c
        src/drivers/Buzzer/Buzzer_Lcfg.c
        src/services/BleComm/BleComm.c
        src/services/BleComm/BleComm_Lcfg.c
        src/services/Watchdog/Watchdog.c
        src/services/Watchdog/Watchdog_Lcfg.c
        src/hal/Hal_I2c.c
        src/hal/Hal_Uart.c
        src/hal/Hal_Gpio.c
        src/hal/Hal_Pwm.c
        src/rte/Rte.c
    )

    add_executable(Glasses_Node ${GLASSES_SOURCES})
    target_link_libraries(Glasses_Node PRIVATE
        pico_stdlib
        pico_multicore
        pico_cyw43_arch_none
        pico_btstack_cyw43
        pico_btstack_ble
        hardware_i2c
        hardware_uart
        hardware_dma
        hardware_pwm
        hardware_gpio
        hardware_clocks
        m
    )
    pico_enable_stdio_usb(Glasses_Node 1)
    pico_enable_stdio_uart(Glasses_Node 0)
    pico_add_extra_outputs(Glasses_Node)
    pico_set_program_name(Glasses_Node "EchoGuard Glasses Node")
    pico_set_program_version(Glasses_Node "1.0.0")
endif()

# =============================================================
# 2. Host Unit, Integration, and Fuzz Tests (CTest)
# =============================================================
enable_testing()

add_library(unity STATIC
    soup/unity/src/unity.c
)
target_include_directories(unity PUBLIC soup/unity/src tests/harness)

# ----------------- Unit Tests -----------------
# 1. FallDetection Unit Test
add_executable(test_FallDetection
    tests/unit/test_FallDetection.c
    src/app/FallDetection/FallDetection.c
    src/app/FallDetection/FallDetection_Lcfg.c
)
target_link_libraries(test_FallDetection PRIVATE unity m)
add_test(NAME Unit_FallDetection COMMAND test_FallDetection)

# 2. GroundProjection Unit Test
add_executable(test_GroundProjection
    tests/unit/test_GroundProjection.c
    src/app/GroundProjection/GroundProjection.c
)
target_link_libraries(test_GroundProjection PRIVATE unity)
add_test(NAME Unit_GroundProjection COMMAND test_GroundProjection)

# 3. ThreatAssessment Unit Test
add_executable(test_ThreatAssessment
    tests/unit/test_ThreatAssessment.c
    src/app/ThreatAssessment/ThreatAssessment.c
)
target_link_libraries(test_ThreatAssessment PRIVATE unity)
add_test(NAME Unit_ThreatAssessment COMMAND test_ThreatAssessment)

# 4. AlertManager Unit Test
add_executable(test_AlertManager
    tests/unit/test_AlertManager.c
    src/app/AlertManager/AlertManager.c
)
target_link_libraries(test_AlertManager PRIVATE unity)
add_test(NAME Unit_AlertManager COMMAND test_AlertManager)

# 5. Ld2450 Parser Unit Test
add_executable(test_Ld2450_Parser
    tests/unit/test_Ld2450_Parser.c
    src/drivers/Ld2450/Ld2450.c
    src/drivers/Ld2450/Ld2450_Lcfg.c
    src/drivers/Ld2450/Ld2450_Irq.c
    src/hal/Hal_Uart.c
)
target_link_libraries(test_Ld2450_Parser PRIVATE unity)
add_test(NAME Unit_Ld2450_Parser COMMAND test_Ld2450_Parser)

# ----------------- Integration Tests -----------------
# 1. Threat to Alert Pipeline
add_executable(test_Threat_to_Alert_Pipeline
    tests/integration/test_Threat_to_Alert_Pipeline.c
    src/app/ThreatAssessment/ThreatAssessment.c
    src/app/AlertManager/AlertManager.c
    src/rte/Rte.c
)
target_link_libraries(test_Threat_to_Alert_Pipeline PRIVATE unity)
add_test(NAME Integration_ThreatToAlert COMMAND test_Threat_to_Alert_Pipeline)

# 2. Fall to BLE Dispatch
add_executable(test_Fall_to_Ble_Dispatch
    tests/integration/test_Fall_to_Ble_Dispatch.c
    src/app/FallDetection/FallDetection.c
    src/app/FallDetection/FallDetection_Lcfg.c
    src/services/BleComm/BleComm.c
    src/services/BleComm/BleComm_Lcfg.c
)
target_link_libraries(test_Fall_to_Ble_Dispatch PRIVATE unity m)
add_test(NAME Integration_FallToBle COMMAND test_Fall_to_Ble_Dispatch)

# 3. System Failure Scenarios & Auto-Recovery
add_executable(test_System_FailureScenarios
    tests/integration/test_System_FailureScenarios.c
    src/app/FallDetection/FallDetection.c
    src/app/FallDetection/FallDetection_Lcfg.c
    src/app/GroundProjection/GroundProjection.c
    src/app/ThreatAssessment/ThreatAssessment.c
    src/app/AlertManager/AlertManager.c
    src/services/BleComm/BleComm.c
    src/services/BleComm/BleComm_Lcfg.c
    src/services/Watchdog/Watchdog.c
    src/services/Watchdog/Watchdog_Lcfg.c
    src/drivers/Ld2450/Ld2450.c
    src/drivers/Ld2450/Ld2450_Lcfg.c
    src/drivers/Ld2450/Ld2450_Irq.c
    src/hal/Hal_Uart.c
)
target_link_libraries(test_System_FailureScenarios PRIVATE unity m)
add_test(NAME Integration_SystemFailureScenarios COMMAND test_System_FailureScenarios)

# ----------------- Fuzz Tests -----------------
# 1. FallDetection Fuzzer
add_executable(fuzz_FallDetection
    tests/fuzz/fuzz_FallDetection.c
    src/app/FallDetection/FallDetection.c
    src/app/FallDetection/FallDetection_Lcfg.c
)
target_link_libraries(fuzz_FallDetection PRIVATE m)
add_test(NAME Fuzz_FallDetection COMMAND fuzz_FallDetection)

# 2. Radar Frames Fuzzer
add_executable(fuzz_RadarFrames
    tests/fuzz/fuzz_RadarFrames.c
    src/drivers/Ld2450/Ld2450.c
    src/drivers/Ld2450/Ld2450_Lcfg.c
    src/drivers/Ld2450/Ld2450_Irq.c
    src/hal/Hal_Uart.c
)
add_test(NAME Fuzz_RadarFrames COMMAND fuzz_RadarFrames)
