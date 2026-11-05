#include "mock_BleRadio.h"
#include <string.h>

static MockBleRadio_State_t s_state;
static BleRadio_WorkCallback_t s_work;
static BleRadio_RxCallback_t s_rx;
static uint32_t s_now_ms;
static bool s_ready;
static bool s_running;
static bool s_in_worker;

void MockBleRadio_Reset(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_work = NULL;
    s_rx = NULL;
    s_now_ms = 0U;
    s_ready = false;
    s_running = true;
    s_in_worker = false;
}

void MockBleRadio_SetTime(uint32_t now_ms) { s_now_ms = now_ms; }
void MockBleRadio_SetWorkerRunning(bool running) { s_running = running; }

void MockBleRadio_SetReady(bool ready)
{
    s_ready = ready;
    if (ready && (s_work != NULL))
    {
        s_in_worker = true;
        s_work(true, s_now_ms);
        s_in_worker = false;
    }
}

void MockBleRadio_Receive(const uint8_t * const p_data, uint8_t len)
{
    if (s_rx != NULL)
    {
        s_in_worker = true;
        s_rx(p_data, len, s_now_ms);
        s_in_worker = false;
    }
}

const MockBleRadio_State_t * MockBleRadio_GetState(void) { return &s_state; }

bool BleRadio_Start(const char * const p_name, BleRadio_WorkCallback_t work,
                     BleRadio_RxCallback_t rx)
{
    (void)p_name;
    ++s_state.starts;
    s_work = work;
    s_rx = rx;
    return true;
}

void BleRadio_RequestWork(void)
{
    if (s_running && (s_work != NULL))
    {
        s_in_worker = true;
        s_work(false, s_now_ms);
        s_in_worker = false;
    }
}

bool BleRadio_IsReady(void) { return s_ready; }

void BleRadio_Configure(const BleRadio_Settings_t * const p_settings)
{
    if (!s_in_worker) { ++s_state.wrong_context_count; }
    s_state.settings = *p_settings;
    ++s_state.configure_count;
}

void BleRadio_SetAdvertisingData(const uint8_t * const p_data, uint8_t len)
{
    if (!s_in_worker) { ++s_state.wrong_context_count; }
    if (len <= sizeof(s_state.data))
    {
        memcpy(s_state.data, p_data, len);
        s_state.data_len = len;
        ++s_state.data_count;
    }
}
