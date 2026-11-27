"""Architectural guards, not a complete BARR-C conformance or hardware timing proof."""

import argparse
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise SystemExit(message)


def source_contracts():
    source = (ROOT / "src/hal/Hal_Uart.c").read_text()
    header = (ROOT / "src/hal/Hal_Uart.h").read_text()
    clean = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
    match = re.search(r"static void HAL_UART_ISR_ATTR (\w+_isr)\(void\)\s*\{", clean)
    require(match is not None, "DMA ISR must be private, void(void), and end in _isr")
    name = match.group(1)
    require(name not in header, "Do not expose an ISR as an ordinary public API")
    require('interrupt("IRQ")' in source, "Missing explicit ARM ISR compiler annotation")
    begin = match.end()
    depth = 1
    end = begin
    while depth:
        require(end < len(clean), "Unterminated ISR body")
        depth += (clean[end] == "{") - (clean[end] == "}")
        end += 1
    body = clean[begin:end - 1]
    require(not re.search(r"\b(for|while|do|goto)\b", body), "ISR must not loop or retry")
    calls = set(re.findall(r"\b([A-Za-z_]\w*)\s*\(", body)) - {"if", "sizeof"}
    require(not calls, f"ISR work must remain direct MMIO; unexpected calls: {sorted(calls)}")

    for path in (ROOT / "src").rglob("*.c"):
        text = re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(), flags=re.S)
        require(not re.search(rf"\b{name}\s*\(\s*\)", text), f"Ordinary ISR call in {path}")
        if "gap_advertisements_set_params(" in text:
            require(path.name == "BleRadio.c", "Radio configuration must stay in the worker adapter")
        if "watchdog_hw->scratch" in text:
            require(path.name == "IncidentRetention.c",
                    "Application watchdog scratch ownership must stay in IncidentRetention")
        if "IncidentRetention_Store(" in text:
            require(path.name in ("IncidentRetention.c", "Cane_Main.c", "Glasses_Main.c"),
                    "Incident retention updates must stay with the service or Core 0 owners")
    retention = (ROOT / "src/services/IncidentRetention/IncidentRetention.c").read_text()
    require(re.search(r"#define\s+INCIDENT_RETENTION_WORD_COUNT\s+\(4U\)", retention),
            "Incident retention must stay within scratch words 0-3")
    require("scratch[4" not in retention,
            "Scratch word 4 belongs to the Pico SDK watchdog marker")
    owners = (("Cane_Main.c", "static void vTask_BleRadioServer"),
              ("Glasses_Main.c", "static void vTask_GlassesBleRadioServer"))
    for main_name, radio_task in owners:
        main_source = (ROOT / "src/app" / main_name).read_text()
        require(main_source.index("IncidentRetention_Init(") < main_source.index("Watchdog_Init("),
                f"{main_name}: inspect reset retention before enabling the startup watchdog")
        radio_boundary = main_source.index(radio_task)
        for call in re.finditer(r"IncidentRetention_(?:Restore|Store)\(", main_source):
            require(call.start() < radio_boundary,
                    f"{main_name}: incident retention runtime access must remain on Core 0")
        for call in re.finditer(r"BleComm_(?:NextFallSequence|ExcludeSession)\(", main_source):
            require(call.start() < radio_boundary,
                    f"{main_name}: compact event/epoch ownership must remain on Core 0")
    types = (ROOT / "src/rte/Rte_Type.h").read_text()
    require(not re.search(r"uint32_t\s+(?:event_id|session_id|peer_ack_event_id)\s*;", types),
            "Wearable telemetry must not restore lifetime-width counters/identities")
    require(re.search(r"rolling_fall_seq\s*:\s*4", types),
            "The transmitted local fall counter must stay four bits")
    require(re.search(r"ttc_20ms\s*:\s*12", types),
            "TTC must use the bounded wire unit, not too-small millisecond storage")
    imu = (ROOT / "src/services/ImuSupervisor/ImuSupervisor.c").read_text()
    require(not re.search(r"\bMpu9250_Init\s*\(|\bsleep_(?:ms|us)\s*\(", imu),
            "IMU startup/recovery must advance without synchronous init or delays")
    require("Mpu9250_RunInitStep(" in imu and "MPU9250_INIT_RESET_WAIT_MS" in imu,
            "Retain staged IMU actions and the required reset wait")
    cmake = (ROOT / "CMakeLists.txt").read_text()
    require("pico_cyw43_arch_lwip_sys_freertos" not in cmake,
            "BLE-only node targets must not reintroduce the unused lwIP stack")
    require("CYW43_LWIP=0" in cmake and "CYW43_NO_DEFAULT_TASK_STACK=1" in cmake,
            "Keep the explicit BLE-only/no-default-stack build contract")
    radio = (ROOT / "src/drivers/BleRadio/BleRadio.c").read_text()
    require("cfg.task_stack = s_worker_stack" in radio and
            "cyw43_arch_async_context() == &s_context.core" in radio,
            "Radio startup must retain the application's supplied Core 1 worker")
    diagnostics = (ROOT / "src/services/Diagnostics/Diagnostics.c").read_text()
    require("PICO_STDIO_USB_ENABLE_IRQ_BACKGROUND_TASK" in diagnostics,
            "Keep the compile-time guard against USB protocol work in a software IRQ")
    print("Realtime source contracts passed: private annotated ISR, radio/scratch ownership, USB guard.")


def binary_contracts(elf, nm, objdump):
    symbols = subprocess.check_output([nm, "-n", str(elf)], text=True)
    entries = {}
    for line in symbols.splitlines():
        fields = line.split()
        if len(fields) == 3:
            entries[fields[2]] = (int(fields[0], 16), fields[1])
    for name in ("__vectors", "ram_vector_table", "__unhandled_user_irq",
                  "xPortSysTickHandler", "xPortPendSVHandler", "vPortSVCHandler",
                  "cyw43_gpio_irq_handler"):
        require(name in entries, f"{elf.name}: missing expected SDK/RTOS vector path: {name}")
    for name in ("IncidentRetention_Init", "IncidentRetention_Restore",
                 "IncidentRetention_Store"):
        require(name in entries, f"{elf.name}: missing incident-retention firmware path: {name}")
    require(0x20000000 <= entries["ram_vector_table"][0] < 0x20042000,
            "Vector dispatch table is not in RP2040 SRAM")
    require("low_priority_worker_irq" not in entries,
            "TinyUSB protocol processing is still linked into a stdio software IRQ")
    for name in ("ram_heap", "memp_memory_PBUF_POOL_base", "memp_memory_FRAG_PBUF_base",
                 "cyw43_async_context_freertos_task_stack"):
        require(name not in entries, f"{elf.name}: unused networking/default stack linked: {name}")
    require("s_worker_stack" in entries and "s_context" in entries,
            f"{elf.name}: supplied FreeRTOS radio worker is missing")
    require("Mpu9250_Init" not in entries,
            f"{elf.name}: synchronous IMU initialization remains linked into perception")
    require("Mpu9250_RunInitStep" in entries and "Mpu9250_SetSleepTimed" in entries,
            f"{elf.name}: staged/timed IMU driver paths are missing")
    if "hal_uart_rx_dma_isr" in entries:
        require(entries["hal_uart_rx_dma_isr"][1] == "t", "DMA ISR must be a private text symbol")
        assembly = subprocess.check_output(
            [objdump, "-d", "--disassemble=hal_uart_rx_dma_isr", str(elf)], text=True
        )
        instructions = {}
        for line in assembly.splitlines():
            parsed = re.match(r"\s*([0-9a-f]+):\s+(?:[0-9a-f]{4}\s+)+\s*([a-z][a-z0-9.]+)\s*(.*)", line)
            if parsed:
                address, opcode, operands = parsed.groups()
                instructions[int(address, 16)] = (opcode.split(".")[0], operands)
                require(opcode.split(".")[0] not in ("bl", "blx"), f"Unexpected out-of-line ISR call: {line}")
        require(0 < len(instructions) <= 96, "ISR exceeds the reviewed instruction-count guard")
        addresses = sorted(instructions)
        following = dict(zip(addresses, addresses[1:]))
        visiting = set()
        lengths = {}

        def path_length(address):
            # A backward jump to a shared return is not a loop. Check control-flow
            # cycles instead of rejecting legitimate compiler block layout.
            require(address not in visiting, "ISR contains a control-flow cycle")
            if address in lengths:
                return lengths[address]
            require(address in instructions, "ISR branches outside its instruction body")
            visiting.add(address)
            opcode, operands = instructions[address]
            edges = []
            if opcode == "bx":
                require(operands.strip() in ("lr", "r14"), "Unexpected indirect ISR branch")
            elif opcode == "pop" and "pc" in operands:
                pass
            else:
                target = re.match(r"([0-9a-f]+)\s+<", operands)
                if opcode.startswith("b") and target:
                    edges.append(int(target.group(1), 16))
                if opcode != "b" and address in following:
                    edges.append(following[address])
            length = 1 + max((path_length(edge) for edge in edges), default=0)
            visiting.remove(address)
            lengths[address] = length
            return length

        longest = path_length(addresses[0])
        print(f"{elf.name}: DMA ISR {len(instructions)} ARM instructions, longest path {longest}; no calls/loops.")
    print(f"{elf.name}: SDK/FreeRTOS vector paths present; USB software worker IRQ absent.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", action="append", type=Path, default=[])
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--objdump", default="arm-none-eabi-objdump")
    args = parser.parse_args()
    source_contracts()
    for elf in args.elf:
        binary_contracts(elf, args.nm, args.objdump)


if __name__ == "__main__":
    main()
