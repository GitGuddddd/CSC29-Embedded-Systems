# Smart Mobility Aid, Glasses node (perception slice).
#
#   Host tests:  cmake -S . -B build/host && cmake --build build/host
#                ctest --test-dir build/host --output-on-failure
#   Firmware:    cmake --build build/host --target glasses_firmware
#                (micro T-Kernel 3.0 from soup/utkernel, built with make;
#                 needs arm-none-eabi-gcc and PICO_SDK_PATH, see
#                 docs/rtos_port_notes.md). Result: build/utkernel/build_make
#                 /mtk3pico_smp1_usb_cdc_wifi.uf2
#   Cane firmware: owned by the cane team (not in this slice).

cmake_minimum_required(VERSION 3.13)
project(smart_mobility_aid_glasses C)

set(HOST_SANITIZE OFF CACHE BOOL "Build the host tests with ASan and UBSan")
set(ROOT ${CMAKE_CURRENT_LIST_DIR})

set(INCLUDE_DIRS
    ${ROOT}/config
    ${ROOT}/src/hal
    ${ROOT}/src/drivers/ld2450
    ${ROOT}/src/rte
    ${ROOT}/src/services/osal
    ${ROOT}/src/app/threat_assessment
    ${ROOT}/src/app/radar_monitor)

set(LD2450_SOURCES
    ${ROOT}/src/drivers/ld2450/ld2450.c
    ${ROOT}/src/drivers/ld2450/ld2450_lcfg.c)

set(APP_SOURCES
    ${ROOT}/src/app/threat_assessment/threat_assessment.c
    ${ROOT}/src/app/threat_assessment/threat_assessment_lcfg.c
    ${ROOT}/src/app/radar_monitor/radar_monitor.c)

enable_testing()

# Firmware: stage soup/utkernel, export the radar into it, build with make.
add_custom_target(glasses_firmware
    COMMAND python3 ${ROOT}/tools/export_to_template.py
    COMMAND make -C ${ROOT}/build/utkernel/build_make -j8
    COMMENT "Building the glasses firmware (uT-Kernel 3.0, course template)"
    VERBATIM)

set(STRICT_FLAGS
    -std=c99 -pedantic -Wall -Wextra -Werror -Wconversion
    -Wsign-conversion -Wshadow -Wundef -Wstrict-prototypes
    -Wmissing-prototypes -Wpointer-arith -Wcast-qual)

if(HOST_SANITIZE)
    add_compile_options(-fsanitize=address,undefined
                        -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# SOUP: compiled with its own flags, never with ours.
add_library(unity STATIC ${ROOT}/soup/unity/unity.c)
target_include_directories(unity PUBLIC ${ROOT}/soup/unity)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_CONFIG_H)
target_include_directories(unity PUBLIC ${ROOT}/tests/harness)

set(HARNESS_SOURCES
    ${ROOT}/tests/harness/mock_hal.c
    ${ROOT}/tests/harness/mock_rte.c
    ${ROOT}/tests/harness/frame_builder.c)

function(add_host_test name source)
    add_executable(${name} ${source} ${HARNESS_SOURCES} ${ARGN})
    target_include_directories(${name} PRIVATE
        ${INCLUDE_DIRS} ${ROOT}/tests/harness)
    target_compile_options(${name} PRIVATE ${STRICT_FLAGS})
    target_link_libraries(${name} unity)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_ld2450_parser
    ${ROOT}/tests/unit/test_ld2450_parser.c ${LD2450_SOURCES})
add_host_test(test_threat_assessment
    ${ROOT}/tests/unit/test_threat_assessment.c ${APP_SOURCES})
add_host_test(test_radar_monitor
    ${ROOT}/tests/unit/test_radar_monitor.c ${APP_SOURCES})
add_host_test(test_alert_pipeline
    ${ROOT}/tests/integration/test_alert_pipeline.c
    ${LD2450_SOURCES} ${APP_SOURCES})
add_host_test(fuzz_radar_frames
    ${ROOT}/tests/fuzz/fuzz_radar_frames.c
    ${LD2450_SOURCES} ${APP_SOURCES})
