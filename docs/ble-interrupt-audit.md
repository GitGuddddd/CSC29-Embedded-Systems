# BLE policy and interrupt design audit

This documents the application-owned runtime paths for the Pico W cane and glasses.
The configuration is FreeRTOS SMP with task affinity. Application policy and parsing
run in tasks; hardware interrupts perform only the work that cannot be deferred.

## State-to-radio policy

Intervals below are **configured base advertising intervals**, not measured packet
arrival times. BLE adds 0–10 ms advertising delay and RF collisions can lose packets.
Scanning is passive; advertising is scannable/non-connectable (`ADV_SCAN_IND`) so a
phone may retrieve the full device name from the scan response without creating a
GATT connection. The primary advertisement contains flags and one atomic 24-byte
manufacturer frame; together with AD headers this consumes the 31-byte legacy
advertising budget, so the redundant short-name AD was removed.

| Frame bytes | Content |
| --- | --- |
| 0–5 | Backward-compatible compact status |
| 6 | Extension marker/version `0xE1` |
| 7–10 | Origin's random 32-bit boot session, little-endian |
| 11–14 | Origin's 32-bit local event counter, little-endian |
| 15–18 | Session ID of the peer event being receipted, little-endian |
| 19–22 | Event ID being receipted, little-endian |
| 23 | Safety flags; bit 0 marks the peer receipt fields valid |

The receiver accepts either exactly six legacy bytes or the complete 24-byte versioned
frame; a truncated extension is not silently interpreted as legacy. The cane emits a
receipt only after its mode manager latches the matching glasses event and the alert
path issues its alarm request. This is application acceptance, not user acknowledgement,
incident clearance, authentication or proof of physical actuator output.

| `Sys_State_t` | Base advertising | Payload refresh | Scan window / interval | Reason |
| --- | ---: | ---: | ---: | --- |
| INIT | 100 ms | 50 ms | 100 / 100 ms | Discover/synchronize promptly during startup or wake |
| IDLE | 200 ms | 100 ms | 80 / 100 ms | Quiet operation with peer reception still available |
| ACTIVE | 150 ms | 50 ms | 80 / 100 ms | Walking telemetry with reduced advertising frequency |
| WATCH | 100 ms | 50 ms | 100 / 100 ms | Current obstacle / ground hazard |
| UNGRIPPED_WATCH | 100 ms | 50 ms | 100 / 100 ms | Resolve parking versus a developing fall promptly |
| FALL_CONFIRMED | 100 ms | 50 ms | 100 / 100 ms | Repeat the latched incident promptly |
| FALL_MUTED | 100 ms | 50 ms | 100 / 100 ms | Audio pause still represents an unresolved incident |
| HARD_FAULT | 100 ms | 50 ms | 100 / 100 ms | Report a fault while the application can still run |
| FAULT_DEGRADED | 100 ms | 50 ms | 100 / 100 ms | Report degraded sensing and recovery |
| STANDBY | 200 ms | 100 ms | 80 / 100 ms | Reduce advertising while retaining remote alarm / wake reception |
| Invalid / unknown value | 100 ms | 50 ms | 100 / 100 ms | Conservative fallback |

Fall, paused-alarm, obstacle or sensor-fault flags, and local peer-link loss, override
any state to FAST. Thus standby with an unresolved peer alarm is not quiet. Escalation
is immediate at the next worker dispatch. Downshifts wait 1 s after the last more-urgent
profile request, avoiding repeated stop/configure/start cycles around state boundaries.

`BleComm_Policy.c` enumerates every real state. Its `-Wswitch-enum` error check and the
all-state unit test require a decision whenever the enum grows. `SYS_STATE_COUNT` is
a sentinel, not an operational mode, and also selects the conservative fallback.

### Freshness and the 500 ms link deadline

The longest base interval is deliberately 200 ms, not 500 ms:

`2 × (200 ms + 10 ms advertising delay) + 50 ms task/transport budget = 470 ms < 500 ms`.

This budgets two advertising opportunities; it is not a guarantee of RF reception.
Payload refresh is faster than advertising so the controller can repeat a current
application heartbeat. State, fault or selected-profile changes publish immediately,
and the application retries if the bounded TX queue is full.

The worker never invents heartbeat increments. Repeated, identical advertisements
from a frozen perception task do not refresh the peer watchdog. A three-bit sequence
cannot provide indefinite ordering: after a 250 ms reception gap, nonduplicate
sequences may establish a new baseline instead of rejecting a live peer indefinitely.
Changed incident/status fields survive a same-sequence wrap. This is duplicate/order
handling, not authenticated replay protection.

Within one boot session, the full event counter must advance in the forward half of
the modulo-32-bit space. Duplicate identities and delayed older identities do not
re-latch a peer incident; `UINT32_MAX` to zero is accepted as forward wrap. A changed
random boot session establishes a new event epoch. The random value reduces accidental
reboot aliasing but is neither provisioned identity nor cryptographic authentication.

For a glasses-originated head fall, 750 ms is the software confirmation deadline. It
allows several FAST-profile advertising/scan opportunities in both directions, but is
not an RF delivery guarantee. Missing confirmation starts the sparse local glasses cue;
confirmation suppresses that cue only while fresh cane packets continue arriving.

RX records carry the transport-callback timestamp, using the same monotonic boot-time
epoch as the application. Draining an old queue does not turn old data into a new
heartbeat. Both main loops discard records already 500 ms old. These timestamps do
not reconstruct the over-the-air arrival time of data buffered inside the controller.

## Execution ownership

`BleComm_Init()` prepares software state before the scheduler. The Core 1 radio task
then starts a dedicated SDK async context, pinned to Core 1 with a static worker
stack. SDK host-wake IRQ setup executes on that same core. Application GAP/HCI setup,
advertising/scan parameter changes and payload updates execute only in the worker.
Runtime assertions check task identity, core and absence of exception context.

The Core 0 producer queues a complete payload-plus-profile snapshot. The worker
coalesces a bounded number of pending snapshots and preserves the newest application
sequence. Profile changes call the actual `gap_advertisements_set_params()` and scan
configuration APIs; ordinary payload refreshes do not restart scanning/advertising.
HCI startup/restart reapplies the cached profile and data, and no fabricated healthy
advertisement is enabled before the application supplies a packet.

TX and RX queues use C11 acquire/release indices. On RP2040 the SDK may implement
atomics with short hardware-spinlock critical sections; they are not described as
universally lock-free. The application DMA ISR uses neither atomics nor spinlocks.

## Polling, interrupts and DMA decisions

| Path | Mechanism / owner | Why this mechanism | What runs in the ISR |
| --- | --- | --- | --- |
| Cane capacitive grip, GP14 | 10 ms task polling; elapsed-time 20 ms debounce | Human input, known cadence, no edge storm from bounce; touch remains powered during IMU standby | Nothing; no touch GPIO IRQ is enabled |
| GP20 calibration / glasses acknowledge | 50 ms task sampling and hold/edge logic | Human response times; calibration requires a 2 s hold; no per-edge interrupt needed | Nothing |
| MPU accel/gyro, I2C | 100 Hz task acquisition while awake; hardware sleep in standby | Short periodic transfers and no configured data-ready interrupt pin; software needs a time-aligned sample, not a per-byte ISR | Nothing; no application I2C IRQ |
| Downward ToF, I2C | 100 ms task acquisition | Small, low-rate status/distance transfers; no configured GPIO data-ready pin | Nothing; sensor `SYSTEM_INTERRUPT_CLEAR` acknowledges the sensor's own latch, not the CPU NVIC |
| LD2450 UART0, GP0 TX / GP1 RX | RX DREQ-paced DMA; Core 0 completion IRQ; 20 ms bounded task parsing | Avoid up to 25,600 byte interrupts/s; completion gives a clear memory-ownership boundary; parsing can be deferred | Acknowledge only the allocated channel, commit completed byte count, re-arm; record bus error instead of retrying in IRQ |
| CYW43 host-wake | SDK GPIO level IRQ on Core 1, deferred async worker | Incoming radio work is asynchronous; masking the level source prevents an interrupt storm until serviced | Mask host-wake and notify the SDK worker; no packet parsing or SPI protocol processing |
| CYW43 SPI / PIO transport | SDK-managed DMA and task-level completion handling | Short transport bursts under a single worker owner; no application byte-level SPI IRQ | No additional application ISR |
| Motor / buzzer PWM | Hardware PWM carrier; task-level pattern stepping | Hardware generates thousands of carrier edges/s without CPU interrupts | No PWM-wrap ISR |
| USB CDC diagnostics | SDK USB controller IRQ plus low-priority `Diagnostics_Task` | Controller service is interrupt-driven; USB protocol processing belongs in task context | SDK endpoint servicing/event enqueue; the former `tud_task()` software-IRQ worker is disabled |
| FreeRTOS scheduling | SysTick on Core 0; PendSV/SVCall and cross-core SIO IRQs owned by the port | Tick deadlines, context switches, and SMP wake/yield must integrate with the kernel | Kernel scheduling / FIFO notification only; application sensing and radio parsing remain tasks |
| SDK alarm pool | Default TIMER_IRQ_3 on Core 0 | Supports SDK time/wakeup services such as bounded sensor startup sleeps | SDK alarm dispatch / wake signaling; no application sensor or radio algorithm is registered there |
| BLE deadman / hardware watchdog | Task elapsed-time check / RP2040 hardware reset | No periodic watchdog ISR is necessary; a live interrupt cannot hide a stalled task | None |

Polling here means task-level bounded acquisition, not polling with interrupts
disabled. I2C has 5 ms transaction timeouts; a failing transaction and sensor
reinitialization can exceed the nominal 10 ms application period. Gap-aware mode
logic discards interrupted qualification windows, and sensor faults enter degraded
operation. A fault-recovery loop is not claimed to meet a normal-mode 100 Hz WCET.

The RP2040 remains scheduled during sensor standby, so a touch IRQ is not needed to
wake the MCU. MCU dormant mode would require a different wake/timebase design.

### Radar DMA details and bounds

The backing ring is 1024 bytes and naturally aligned for the DMA hardware's address-bit
wrapping. RX completes in 30-byte blocks, matching the protocol frame size. The ISR
publishes only fully completed blocks, not a speculative mid-transfer count. The
hardware write address is preserved on re-arm, including when a block spans ring end.

At 256,000 baud with 8N1 framing, the maximum byte rate is 25,600/s and the maximum
completion rate is about 854 IRQs/s (30 times fewer than per-byte interrupts).
Normally this is one IRQ per received radar frame. The 16-byte UART FIFO gives up to
625 microseconds of full-line-rate buffering for the re-arm gap; actual worst-case
IRQ latency must be measured under simultaneous USB/radio load.

The task reserves one block of ring capacity for in-flight DMA writes and rechecks
the producer after copying a candidate. Monotonic modulo-32-bit byte counters detect
overrun independently of wrapped ring addresses. An overrun or a polling gap above
100 ms discards the backlog and reports a fault instead of refreshing a threat from
old data. A fresh complete frame, not merely DMA reconfiguration, clears RX fault.

Parsing advances at most 512 source bytes per call. Incomplete frames wait for more
committed bytes; corrupt framing resynchronizes one byte at a time. After a byte slip,
the final bytes of a frame may require the next DMA block before they become visible.
The parser never runs in the completion ISR. `Ld2450_Irq.c` was renamed to
`Ld2450_Stream.c` to reflect that distinction; the real ISR belongs to `Hal_Uart.c`.

## IRQ, ISR and IVT roles

- **IRQ:** the hardware request source and NVIC enable/priority. UART DMA uses the
  SDK shared `DMA_IRQ_1` dispatch on Core 0 and checks/acknowledges only its channel.
  Its NVIC priority is `0x80`. Cyw43 uses `IO_IRQ_BANK0` on Core 1. The SDK's raw GPIO
  handler ordering value is not the same thing as NVIC interrupt priority.
- **ISR:** `hal_uart_rx_dma_isr` is private, `void(void)`, named with `_isr`, and uses
  GCC's ARM interrupt attribute. It has no loops, out-of-line calls, allocation,
  logging, floating-point processing, I2C accesses, mutexes, atomics or RTOS calls.
  Only hardware-immediate work is done before returning to the SDK dispatcher.
- **IVT:** SDK startup supplies the flash vector table and copies the dispatch table
  into SRAM. SDK registration installs driver handlers; the FreeRTOS port installs
  SysTick/PendSV/SVCall and SIO handlers. Application code does not hand-patch VTOR or
  replace the kernel's exception vectors.
- **Unexpected interrupts:** the SDK default vector stubs remain installed for
  unused sources. They are terminal fault traps, not ordinary application handlers;
  the hardware watchdog provides reset recovery after it has been armed.

Cortex-M0+ has PRIMASK rather than BASEPRI. This port handles its own kernel masking
and exception priorities; unused Cortex-M3-style priority-ceiling definitions were
removed from `FreeRTOSConfig.h`. No application DMA ISR calls a FreeRTOS API.

## Watchdog and diagnostics

The hardware watchdog has an 8 s bounded startup window for radio initialization and
a 2 s operational window. It switches to the latter only after both Core 0 perception
and the ready BTstack worker have progressed. A radio task merely polling/scheduling
a stuck worker cannot feed it. Core 0 publishes a monotonic generation, avoiding the
previous cross-core check/clear flag race.

USB's low-priority software ISR formerly ran `tud_task()`. It is compiled out.
`Diagnostics_Task` pumps the SDK's mutex-protected USB flush/service path every 10 ms,
at a lower priority than perception and actuation. Diagnostic output uses 1 ms timeout
settings and may be dropped under backpressure. Driver callbacks, formatting and
command handling are task work, not application ISR work.

## BARR-C:2018 application and verification

The audited application paths use explicit-width data, unsigned elapsed-time
comparisons, bounded loops/buffers, static storage, input validation, checked task
creation, private ISR visibility and compiler ISR annotation. The I2C multi-write
VLA was replaced by a fixed 34-byte buffer with validated length/address bounds.
The project retains its established AUTOSAR-style public `Module_Function` names;
this naming convention is a documented project deviation from strict BARR naming.
Vendor SDK/kernel entry-point names and implementations are retained, not relabeled
as wholly BARR-conformant application code.

Audited radio, watchdog, diagnostics, UART/I2C and radar paths compile with
`-Wall -Wextra -Werror -Wconversion -Wshadow -Wstrict-prototypes
-Wmissing-prototypes -Wvla`. Imported FreeRTOS headers are marked as system headers
so vendor diagnostics do not weaken the application's warning policy. This is an
engineering conformance review with explicit scope, not a certification statement.

Checks include:

- all states, context overrides, downshift hysteresis and timer wrap;
- controller-not-ready/restart behavior, full queues, latest snapshot preservation,
  duplicate/frozen heartbeats, receive timestamps and malformed advertisements;
- committed partial/wrapped UART frames, overrun, parser resynchronization, stale
  polling, DMA faults and byte-counter wrap;
- source guards against exposing/calling the ISR or adding loops/helper calls;
- ELF/disassembly checks for the SDK/RTOS vector paths, absence of the USB software
  worker IRQ, and an acyclic, leaf application DMA ISR within an instruction budget.

Run:

```sh
cmake --build build_host --parallel 8
ctest --test-dir build_host --output-on-failure
cmake --build build_firmware --target Cane_Node Glasses_Node --parallel 8
python3 tests/check_realtime_contracts.py --elf build_firmware/Cane_Node.elf --elf build_firmware/Glasses_Node.elf
```

FreeRTOS is pinned to the inspected SMP revision
`81c623212a8746c4b5ebc29891b91228591040cc`. SDK inspection used
`98a542c1a62fb549ffb5d66a3e5892b06276b670`. Key vendor paths are
`pico_cyw43_driver/cyw43_driver.c`, `pico_btstack/btstack_run_loop_async_context.c`,
`pico_stdio_usb/stdio_usb.c`, `pico_runtime_init/runtime_init.c`, and the RP2040
FreeRTOS `port.c`. Re-audit these paths when changing the SDK/kernel revision.

Hardware validation still needs radio capture for actual advertising/scan timing and
packet-loss behavior; IRQ entry-to-exit and re-arm latency under USB/radio traffic;
task jitter and UART overrun under bus faults; and power measurements. Static
instruction counts are not measured ISR execution time or a full-system WCET proof.
