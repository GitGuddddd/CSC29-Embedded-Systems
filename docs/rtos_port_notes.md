# Kernel and build: uT-Kernel 3.0 on the course template

The firmware runs on micro T-Kernel 3.0 (dual-core SMP) from the course
template (INF2004_ProjectTemplate). FreeRTOS is not used.

## Build

1. Install the Arm compiler and set PICO_SDK_PATH (template README, step 1).
2. `cmake -S . -B build/host && cmake --build build/host --target glasses_firmware`
   copies soup/utkernel to build/utkernel, exports the radar into
   `build/utkernel/app_program/subsystem5/` (flat, because the template
   compiles one folder level only), applies `DEVCNF_USE_SER 0`, and runs make.
3. Flash `build/utkernel/build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`.

For the team repo instead of the staging copy:
`python3 tools/export_to_template.py <team-repo-root>` (then the
`DEVCNF_USE_SER 0` change must be recorded in its TEAM.md).

Host tests (PC): `cmake --build build/host && ctest --test-dir build/host`.

## What the radar code uses

| Radar code | uT-Kernel |
|---|---|
| osal_queue_* (radar, audio, BLE queues) | tk_cre_mbf / tk_snd_mbf / tk_rcv_mbf |
| osal_stream_* (UART bytes, NFR7) | message buffer, written by the UART interrupt with TMO_POL |
| osal_task_create | tk_cre_tsk + tk_sta_tsk, priorities 20 and 21 (subsystem 5 band) |
| osal_delay_ms | tk_dly_tsk |
| hal_time_now_ms | tk_get_otm |
| hal_uart_set_rx_handler | tk_def_int + EnableInt on UART0 (interrupt 20) |

## UART0 and the kernel serial driver

The template's serial device driver ("sera") also uses the UART0 interrupt,
reads one byte per interrupt with the FIFO off, and has a 50-byte buffer.
At 256000 baud a byte arrives every 39 us, so the radar drives UART0 itself
with the 32-byte FIFO and the receive-timeout interrupt. That needs
`DEVCNF_USE_SER 0`. GP0 and GP1 are claimed for the radar; do not build with
`CRASH_REPORT=1` or `CONSOLE=uart`.

## Open points

- Nothing here has run on the Pico yet (builds and links only).
- Interrupt latency of the UART handler under the SMP kernel is unmeasured.
- Radar tasks have no core affinity ("any core").
