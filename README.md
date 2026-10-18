# Smart Mobility Aid: glasses perception slice (Putra)

The part of the glasses node that reads the HLK-LD2450 radar and decides when
an obstacle is near. It follows the team's layered layout and Barr-C:2018.
Cane code, head-fall, Audio Alert and BLE are other people's modules and are
not in this tree; `rte_audio_msg_t` and `rte_ble_obstacle_t` are the hand-off
points to them.

    config/        board_glasses.h, feature_cfg.h
    soup/          third-party code: utkernel (course template), unity
    src/services/osal/   osal (uT-Kernel adapter), tk_include.h
    src/hal/       hal_uart, hal_time (only place that touches kernel and chip registers)
    src/drivers/ld2450/   ld2450 (protocol), _irq (interrupt + stream buffer),
                          _cfg.h, _lcfg.c
    src/rte/       rte (queues, tasks, glue), rte_type.h, rte_cfg.h
    src/app/threat_assessment/   obstacle decision, TTC, pitch, _cfg.h, _lcfg.c
    src/app/radar_monitor/       INIT / RUN / FAULT supervision, _cfg.h
    src/app/main_glasses.c       launcher + print-only stubs
    tests/         unit, integration, fuzz, harness (mocks)
    docs/          traceability, deviations, SOUP risk, abbreviations
    tools/barr_check.py          automatic Barr-C scan

## Data flow

    UART0 RX interrupt -> stream buffer -> Glasses_Radar (radar_monitor)
      -> radar queue -> Glasses_Threat (threat_assessment)
      -> Audio Alert queue (OBSTACLE, CLEAR, RADAR_FAULT, RADAR_OK) + BLE queue

Application components call only `rte_write_*`. The RTE is the only module
that knows both the components and the kernel (through osal).

## Build and test on a PC

    cmake -S . -B build/host -G Ninja
    cmake --build build/host
    ctest --test-dir build/host --output-on-failure
    cmake -S . -B build/san -G Ninja -DHOST_SANITIZE=ON    # ASan + UBSan
    python3 tools/barr_check.py                            # must print 0 findings

## Build the firmware (micro T-Kernel 3.0, course template)

See docs/rtos_port_notes.md: `cmake --build build/host --target glasses_firmware`
(or `python3 tools/export_to_template.py` then `make -j8` in build/utkernel/build_make).
It builds and links; it has not run on the Pico yet (docs/deviations.md
OPEN-10).

## Behaviour (Group 29 report)

- Obstacle: nearest valid target that is not moving away faster than 10 cm/s;
  starts below 2000 mm, stops above 2100 mm, or 1 s without such a target.
- A stationary object inside the range does alert (it is not departing).
- No valid frame for 500 ms: RADAR_FAULT to Audio Alert, Threat goes to CLEAR,
  config retried every 2 s, RADAR_OK when frames resume. The first frame after
  config gets 2 s (bench finding).
