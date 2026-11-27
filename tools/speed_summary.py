#!/usr/bin/env python3
"""Summarise LD2450 speed readings from serial logs (assumption A7).

Input: one or more serial log files that contain the radar's target lines,
printed when FEATURE_DEBUG_LOG_TARGETS is 1:

    [T] t=<ms> <x,y,speed> <x,y,speed> <x,y,speed>      ("-" = empty slot)

Usage: python3 tools/speed_summary.py run_toward.txt run_away.txt ...

One report per file, so use one file per walk. Sign convention: the code
treats NEGATIVE speed as approaching (threat_assessment_lcfg.c). The report
shows both signs so you can confirm that.
"""

import re
import sys

LINE = re.compile(r"\[T\]\s+t=(\d+)((?:\s+(?:-?\d+,-?\d+,-?\d+|-))+)")
SLOT = re.compile(r"-?\d+,-?\d+,-?\d+")
CLAMP = 255          # proposed clamp in docs/radar_message_layout.md
BUCKETS = [(0, 0), (1, 10), (11, 50), (51, 100), (101, 150), (151, 255),
           (256, 10 ** 9)]


def read_speeds(path):
    frames = 0
    speeds = []
    first_ms = None
    last_ms = None

    with open(path, "r", errors="replace") as handle:
        for line in handle:
            match = LINE.search(line)
            if match is None:
                continue
            frames += 1
            last_ms = int(match.group(1))
            if first_ms is None:
                first_ms = last_ms
            for slot in SLOT.findall(match.group(2)):
                speeds.append(int(slot.split(",")[2]))

    return frames, speeds, first_ms, last_ms


def report(path):
    frames, speeds, first_ms, last_ms = read_speeds(path)

    print("== " + path)
    if frames == 0:
        print("  No [T] lines found. Is FEATURE_DEBUG_LOG_TARGETS 1 and NDEBUG "
              "undefined in the build you ran?")
        return

    seconds = (last_ms - first_ms) / 1000.0
    print("  frames: %d over %.1f s, target readings: %d" %
          (frames, seconds, len(speeds)))
    if not speeds:
        print("  No targets seen (all slots empty).")
        return

    approaching = [-s for s in speeds if s < 0]
    departing = [s for s in speeds if s > 0]
    largest = max(abs(s) for s in speeds)
    print("  largest |speed|: %d cm/s" % largest)
    print("  approaching (negative): %d readings, largest %d cm/s" %
          (len(approaching), max(approaching) if approaching else 0))
    print("  departing   (positive): %d readings, largest %d cm/s" %
          (len(departing), max(departing) if departing else 0))
    print("  zero speed: %d readings" % sum(1 for s in speeds if s == 0))

    print("  |speed| histogram (cm/s):")
    for low, high in BUCKETS:
        count = sum(1 for s in speeds if low <= abs(s) <= high)
        label = ("%d" % low) if low == high else (
            ("%d+" % low) if high > 10 ** 6 else ("%d-%d" % (low, high)))
        print("    %-8s %6d  %5.1f%%" % (label, count,
                                          100.0 * count / len(speeds)))

    over = sum(1 for s in speeds if abs(s) > CLAMP)
    print("  readings above the +/-%d clamp: %d" % (CLAMP, over))
    values = sorted(set(speeds))
    shown = ", ".join(str(v) for v in values[:40])
    print("  distinct speed values (%d): %s%s" %
          (len(values), shown, " ..." if len(values) > 40 else ""))

    if largest <= CLAMP:
        print("  RESULT: largest speed fits inside +/-%d, so 9 bits is enough "
              "for this run." % CLAMP)
    else:
        print("  RESULT: speeds above %d were seen. Decide on more bits or "
              "accept clamping (only TTC for very fast targets is affected)."
              % CLAMP)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    for path in argv[1:]:
        report(path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
