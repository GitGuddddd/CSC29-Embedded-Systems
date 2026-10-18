#include "Watchdog.h"
#include "Watchdog_Cfg.h"
#include <stddef.h>

static uint32_t s_last_kick_ms = 0U;
static uint32_t s_timeout_ms = WATCHDOG_CFG_DEADMAN_TIMEOUT_MS;

void Watchdog_Init(const Watchdog_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        s_timeout_ms = p_cfg->deadman_timeout_ms;
    }
    s_last_kick_ms = 0U;
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
