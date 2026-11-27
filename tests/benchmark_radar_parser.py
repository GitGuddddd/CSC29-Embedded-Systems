"""Native synthetic parser CPU comparison, not RP2040 timing or safety validation."""

import argparse
import json
import os
from pathlib import Path
import platform
import shlex
import statistics
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
STREAM = "src/drivers/Ld2450/Ld2450_Stream.c"
HARNESS = r"""
#include <stdio.h>
#include <time.h>
static double cpu_seconds(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t) != 0) return -1.0;
    return (double)t.tv_sec + (double)t.tv_nsec / 1e9;
}
int main(int argc, char **argv) {
    uint8_t data[512] = {0};
    bool valid = argc > 1 && argv[1][0] == 'v';
    if (valid) for (uint32_t at = 0U; at + 30U <= sizeof(data); at += 30U) {
        data[at] = 0xAAU; data[at+1U] = 0xFFU; data[at+2U] = 3U;
        data[at+6U] = 0xDCU; data[at+7U] = 5U;
        data[at+28U] = 0x55U; data[at+29U] = 0xCCU;
    }
    uint32_t accepted = 0U;
    double start = cpu_seconds();
    if (start < 0.0) return 4;
    for (uint32_t i = 0U; i < BENCH_ITERATIONS; ++i) {
        if (!Ld2450_Init(&Ld2450_Config_Glasses)) return 2;
        if (!Hal_Uart_TestInjectRx(data, sizeof(data))) return 3;
        Radar_Target_t targets[3]; uint8_t count;
        if (Ld2450_ProcessStream(targets, &count, i * 20U)) accepted++;
    }
    double finish = cpu_seconds();
    if (finish < start) return 4;
    printf("%.9f %u\n", finish - start, accepted);
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default="26e3c45", help="Git ref containing the original parser")
    parser.add_argument("--iterations", type=int, default=20000)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--temp-dir", help="Optional external build/scratch directory")
    args = parser.parse_args()
    if not 1 <= args.iterations <= 1000000 or not 1 <= args.repeats <= 20:
        parser.error("iterations must be 1..1000000 and repeats 1..20")
    baseline = subprocess.check_output(["git", "show", f"{args.baseline}:{STREAM}"], cwd=ROOT, text=True)
    compiler = shlex.split(os.environ.get("CC", "cc"))
    results = {}
    with tempfile.TemporaryDirectory(prefix="radar-benchmark-", dir=args.temp_dir) as directory:
        executables = {}
        for name, source in (("before", baseline), ("after", (ROOT / STREAM).read_text())):
            executable = str(Path(directory) / name)
            command = compiler + ["-O2", "-std=gnu11", f"-DBENCH_ITERATIONS={args.iterations}U"]
            for include in ("config", "src", "src/rte", "src/drivers/Ld2450", "src/hal"):
                command += ["-I", include]
            command += ["-x", "c", "-", "src/drivers/Ld2450/Ld2450.c",
                        "src/drivers/Ld2450/Ld2450_Lcfg.c", "src/hal/Hal_Uart.c", "-o", executable]
            subprocess.run(command, cwd=ROOT, input=source + "\n" + HARNESS, text=True, check=True)
            executables[name] = executable
        for mode in ("noise", "valid"):
            cases = {}
            for name, executable in executables.items():
                runs = [subprocess.check_output([executable, mode], text=True).split()
                        for _ in range(args.repeats)]
                expected = args.iterations if mode == "valid" else 0
                if any(int(run[1]) != expected for run in runs):
                    raise RuntimeError(f"{name}/{mode}: unexpected accepted-frame count")
                cases[name] = {"median_cpu_s": statistics.median(float(run[0]) for run in runs),
                               "accepted_iterations": expected}
            cases["after_over_before"] = cases["after"]["median_cpu_s"] / cases["before"]["median_cpu_s"]
            results[mode] = cases
    print(json.dumps({"platform": platform.platform(), "compiler": compiler, "baseline": args.baseline,
                      "iterations": args.iterations, "repeats": args.repeats,
                      "hardware_validated": False, "results": results}, indent=2))


if __name__ == "__main__":
    main()
