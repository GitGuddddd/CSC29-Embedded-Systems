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
    diagnostics = (ROOT / "src/services/Diagnostics/Diagnostics.c").read_text()
    require("PICO_STDIO_USB_ENABLE_IRQ_BACKGROUND_TASK" in diagnostics,
            "Keep the compile-time guard against USB protocol work in a software IRQ")
    print("Realtime source contracts passed: private annotated ISR, no loops/calls, radio ownership, USB guard.")


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
    require(0x20000000 <= entries["ram_vector_table"][0] < 0x20042000,
            "Vector dispatch table is not in RP2040 SRAM")
    require("low_priority_worker_irq" not in entries,
            "TinyUSB protocol processing is still linked into a stdio software IRQ")
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
