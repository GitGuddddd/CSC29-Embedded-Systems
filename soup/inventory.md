# SOUP inventory (IEC 62304 section 8.1.2)

Every third-party item that is compiled into, or used to build or test, the
glasses firmware. SOUP is never edited in place. Barr-C checks and the
strict compiler flags are not applied to it.

| ID | Item | Version | Source | Licence | Used by | How it is obtained | Status |
|----|------|---------|--------|---------|---------|--------------------|--------|
| S-1 | Raspberry Pi Pico SDK | 2.2.0 | https://github.com/raspberrypi/pico-sdk | BSD-3-Clause | USB console and WiFi driver of the course template (not called by radar code) | installed per the template README | Tool, not in repo |
| S-2 | micro T-Kernel 3.0 (dual-core RP2040 port, derived from tron-forum/mtk3_bsp branch pico_rp2040) | as in the course template INF2004_ProjectTemplate | https://github.com/sirfonzie/INF2004_ProjectTemplate | T-License 2.2 | osal_mtk3.c, hal_uart.c, hal_time.c | copied unmodified to `soup/utkernel` | Present; pin the template commit |
| S-3 | Unity test framework | 2.7.2 (copied files, commit 2b80d1a of 2026-09-10) | https://github.com/ThrowTheSwitch/Unity | MIT | tests/ (host only) | copied into `soup/unity` | Present, unmodified |
| S-4 | Arm GNU toolchain | 15_2_Rel1 | Pico VS Code extension | GPL with runtime library exception | firmware build | installed by the extension | Tool, not in repo |
| S-5 | picotool | 2.3.1 | Pico VS Code extension | BSD-3-Clause | UF2 creation | installed by the extension | Tool, not in repo |
| S-6 | Host C compiler, CMake, Ninja | any recent | OS packages | various | host tests | package manager | Tool, not in repo |

Versions of S-1, S-2, S-4 and S-5 are copied from the team RTOS project
(CMakeLists.txt and FreeRTOS-Kernel/include/task.h). Licences are quoted from
memory of the upstream projects and must be confirmed against the LICENSE files
once the submodules are added.

Rules for this folder
- Do not edit SOUP files. Wrap it (see src/hal/) or configure it
  (config) instead.
- Changing a version needs an entry in docs/soup_risk_assessment.md first.
