# SOUP risk assessment (IEC 62304 section 7.1.3 / 8.1.2)

One entry per item in soup/inventory.md. "Anomaly list" = published bug and
errata list of that version; reviewing it is still to be done (owner: Putra).

| ID | Item | Why we use it | Failure effect on the glasses | Mitigation in our code | Verification | Anomaly list |
|----|------|---------------|-------------------------------|------------------------|--------------|--------------|
| S-1 | Pico SDK | UART, GPIO, timer, USB stdio, startup | Wrong or missing UART data, no interrupt: radar silent | Only hal_*.c calls the SDK. A silent radar is detected after 500 ms and reported as RADAR_FAULT (FR11). | Bench run, overflow counter | To review |
| S-2 | micro T-Kernel 3.0 (course template) | Tasks, message buffers, interrupt handler | Missed deadlines, lost messages | Per-frame messages are replaceable; CLEAR, FAULT and OK wait up to 10 ms; radar and threat run at Medium priority; no blocking read without a timeout. | Bench run; later stack high-water mark | To review |
| S-3 | Unity | Host unit tests only | None on the device (not linked into firmware) | Not part of the firmware build. | Tests run from source | Not needed |
| S-4 | Arm GNU toolchain | Compiles firmware | Wrong code generation | Strict warnings on host; same sources syntax-checked for the target. | Bench run | To review |
| S-5 | picotool | Creates the UF2 | Corrupt image | Flash is verified by running the bring-up. | Bench run | Not needed |
| S-6 | Host compiler, CMake, Ninja | Run host tests | A wrong test result | Tests also run with ASan and UBSan. | Re-run on a second machine | Not needed |

Not a safety claim: this is a student project, and the IEC 62304 / ISO 26262
vocabulary follows the leader's architecture proposal.
