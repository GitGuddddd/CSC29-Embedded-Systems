#include "Watchdog.h"
#include "Watchdog_Cfg.h"
#include <stddef.h>
#include <stdio.h>
#include <stdatomic.h>

#ifdef PICO_ON_DEVICE
#include "hardware/watchdog.h"
#endif

static uint32_t s_last_kick_ms = 0U;
static uint32_t s_timeout_ms = WATCHDOG_CFG_DEADMAN_TIMEOUT_MS;
static _Atomic(uint32_t) s_core0_generation;
static uint32_t s_observed_generation;
static bool s_operational;

void Watchdog_Init(const Watchdog_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        s_timeout_ms = p_cfg->deadman_timeout_ms;
    }
    s_last_kick_ms = 0U;
    atomic_store_explicit(&s_core0_generation, 0U, memory_order_relaxed);
    s_observed_generation = 0U;
    s_operational = false;

#ifdef PICO_ON_DEVICE
    /* Initialize RP2040 Hardware Watchdog timer */
    watchdog_enable(WATCHDOG_CFG_STARTUP_TIMEOUT_MS, true);
    printf("[WATCHDOG] Startup limit %u ms; operational limit %u ms\n",
           (unsigned int)WATCHDOG_CFG_STARTUP_TIMEOUT_MS, (unsigned int)WATCHDOG_CFG_HW_TIMEOUT_MS);
#endif
}

void Watchdog_KickBleDeadman(uint32_t now_ms)
{
    s_last_kick_ms = now_ms;
}

bool Watchdog_CheckBleLinkLoss(uint32_t now_ms)
{
    if ((now_ms - s_last_kick_ms) >= s_timeout_ms)
    {
        return true;
    }
    return false;
}

void Watchdog_SetCore0Alive(void)
{
    /* Single Core 0 writer. A monotonic generation avoids a check/clear lost-token race. */
    uint32_t generation = atomic_load_explicit(&s_core0_generation, memory_order_relaxed);
    atomic_store_explicit(&s_core0_generation, generation + 1U, memory_order_release);
}

bool Watchdog_FeedHardware(void)
{
    uint32_t generation = atomic_load_explicit(&s_core0_generation, memory_order_acquire);
    if (generation == s_observed_generation)
    {
        return false;
    }
    s_observed_generation = generation;
    /* The radio task calls this only after confirmed BTstack-worker progress. */
    if (!s_operational)
    {
#ifdef PICO_ON_DEVICE
        watchdog_enable(WATCHDOG_CFG_HW_TIMEOUT_MS, true);
#endif
        s_operational = true;
    }
    else
    {
#ifdef PICO_ON_DEVICE
        watchdog_update();
#endif
    }
    return true;
}
