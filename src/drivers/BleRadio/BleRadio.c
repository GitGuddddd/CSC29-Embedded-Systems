#include "BleRadio.h"
#include <stddef.h>
#include <stdatomic.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "pico/async_context_freertos.h"
#include "pico/stdlib.h"
#include "btstack.h"

/* Hardware ownership: the SDK async worker and its host-wake IRQ both use Core 1.
 * Public request calls come from a task; all GAP/HCI operations run in that worker.
 */
#define BLERADIO_MAX_AD_BYTES       (31U)
#define BLERADIO_WORKER_STACK_WORDS (1024U)
#define BLERADIO_CORE_ID            (1U)
#define BLERADIO_WORKER_PRIORITY    (4U)
#define BLERADIO_ADV_SCAN_IND       (0x02U) /* Scannable, non-connectable; phone name discovery. */

static async_context_freertos_t s_context;
static StackType_t s_worker_stack[BLERADIO_WORKER_STACK_WORDS];
static btstack_packet_callback_registration_t s_hci_registration;
static btstack_context_callback_registration_t s_work_registration;
static BleRadio_WorkCallback_t s_work_callback;
static BleRadio_RxCallback_t s_rx_callback;
static atomic_bool s_ready;
static atomic_bool s_work_pending;
static uint8_t s_advertising_data[BLERADIO_MAX_AD_BYTES];
static uint8_t s_scan_response[BLERADIO_MAX_AD_BYTES];
static uint8_t s_scan_response_len;
static BleRadio_Settings_t s_applied;
static bool s_configured;
static bool s_advertising;
static bool s_scanning;

static void Check_WorkerContext(void)
{
    configASSERT(__get_current_exception() == 0U);
    configASSERT(get_core_num() == BLERADIO_CORE_ID);
    configASSERT(xTaskGetCurrentTaskHandle() == s_context.task_handle);
}

static void Radio_WorkCallback(void * p_context)
{
    (void)p_context;
    Check_WorkerContext();
    s_work_callback(false, to_ms_since_boot(get_absolute_time()));
    atomic_store_explicit(&s_work_pending, false, memory_order_release);
}

/* BTstack event callback, not an ISR. The actual GPIO ISR only wakes the SDK worker. */
static void Radio_PacketCallback(uint8_t packet_type, uint16_t channel,
                                  uint8_t * p_packet, uint16_t size)
{
    (void)channel;
    Check_WorkerContext();
    if ((packet_type != HCI_EVENT_PACKET) || (p_packet == NULL) || (size < 2U) ||
        (((uint16_t)p_packet[1] + 2U) > size))
    {
        return;
    }
    uint8_t event = hci_event_packet_get_type(p_packet);
    if ((event == BTSTACK_EVENT_STATE) && (size >= 3U))
    {
        bool ready = (btstack_event_state_get_state(p_packet) == HCI_STATE_WORKING);
        atomic_store_explicit(&s_ready, ready, memory_order_release);
        s_configured = false;
        s_advertising = false;
        s_scanning = false;
        if (ready)
        {
            gap_scan_response_set_data(s_scan_response_len, s_scan_response);
            s_work_callback(true, to_ms_since_boot(get_absolute_time()));
        }
    }
    else if ((event == GAP_EVENT_ADVERTISING_REPORT) && (size >= 12U))
    {
        uint8_t len = gap_event_advertising_report_get_data_length(p_packet);
        if ((len <= BLERADIO_MAX_AD_BYTES) && ((uint16_t)len <= (size - 12U)))
        {
            s_rx_callback(gap_event_advertising_report_get_data(p_packet), len,
                          to_ms_since_boot(get_absolute_time()));
        }
    }
    else
    {
        /* Other controller events are handled by BTstack. */
    }
}

static uint32_t Radio_StartWorker(void * p_context)
{
    (void)p_context;
    Check_WorkerContext();
    l2cap_init();
    sm_init();
    s_hci_registration.callback = Radio_PacketCallback;
    hci_add_event_handler(&s_hci_registration);
    int status = hci_power_control(HCI_POWER_ON);
    return (status == 0) ? 0U : 1U;
}

bool BleRadio_Start(const char * const p_name, BleRadio_WorkCallback_t work_callback,
                     BleRadio_RxCallback_t rx_callback)
{
    configASSERT(__get_current_exception() == 0U);
    configASSERT(get_core_num() == BLERADIO_CORE_ID);
    if ((p_name == NULL) || (work_callback == NULL) || (rx_callback == NULL))
    {
        return false;
    }
    size_t name_len = strlen(p_name);
    if (name_len > (BLERADIO_MAX_AD_BYTES - 2U))
    {
        return false;
    }
    s_scan_response[0] = (uint8_t)(name_len + 1U);
    s_scan_response[1] = 0x09U;
    memcpy(&s_scan_response[2], p_name, name_len);
    s_scan_response_len = (uint8_t)(name_len + 2U);
    s_work_callback = work_callback;
    s_rx_callback = rx_callback;
    atomic_store_explicit(&s_ready, false, memory_order_relaxed);
    atomic_store_explicit(&s_work_pending, false, memory_order_relaxed);
    s_work_registration.callback = Radio_WorkCallback;
    s_work_registration.context = NULL;

    async_context_freertos_config_t cfg = async_context_freertos_default_config();
    cfg.task_priority = BLERADIO_WORKER_PRIORITY;
    cfg.task_stack_size = BLERADIO_WORKER_STACK_WORDS;
    cfg.task_stack = s_worker_stack;
    cfg.task_core_id = BLERADIO_CORE_ID;
    if (!async_context_freertos_init(&s_context, &cfg))
    {
        return false;
    }
    cyw43_arch_set_async_context(&s_context.core);
    if (cyw43_arch_init() != 0)
    {
        cyw43_arch_set_async_context(NULL);
        async_context_deinit(&s_context.core);
        return false;
    }
    return (async_context_execute_sync(&s_context.core, Radio_StartWorker, NULL) == 0U);
}

void BleRadio_RequestWork(void)
{
    configASSERT(__get_current_exception() == 0U);
    /* One requesting task. The worker releases the registration only after returning
     * from application work; never enqueue one intrusive callback record twice.
     */
    if (!atomic_load_explicit(&s_work_pending, memory_order_acquire))
    {
        atomic_store_explicit(&s_work_pending, true, memory_order_release);
        btstack_run_loop_execute_on_main_thread(&s_work_registration);
    }
}

bool BleRadio_IsReady(void)
{
    return atomic_load_explicit(&s_ready, memory_order_acquire);
}

void BleRadio_Configure(const BleRadio_Settings_t * const p_settings)
{
    Check_WorkerContext();
    if ((p_settings == NULL) || !BleRadio_IsReady() ||
        (p_settings->advertising_units < 0x00A0U) || (p_settings->advertising_units > 0x4000U) ||
        (p_settings->scan_interval_units < 4U) || (p_settings->scan_interval_units > 0x4000U) ||
        (p_settings->scan_window_units < 4U) ||
        (p_settings->scan_window_units > p_settings->scan_interval_units))
    {
        return;
    }
    bool adv_changed = !s_configured || (p_settings->advertising_units != s_applied.advertising_units);
    bool scan_changed = !s_configured || (p_settings->scan_interval_units != s_applied.scan_interval_units) ||
                        (p_settings->scan_window_units != s_applied.scan_window_units);
    if (adv_changed)
    {
        if (s_advertising)
        {
            gap_advertisements_enable(0);
            s_advertising = false;
        }
        gap_advertisements_set_params(p_settings->advertising_units, p_settings->advertising_units,
                                       BLERADIO_ADV_SCAN_IND, 0U, NULL, 0x07U, 0U);
    }
    if (scan_changed)
    {
        if (s_scanning)
        {
            gap_stop_scan();
        }
        gap_set_scan_parameters(0U, p_settings->scan_interval_units, p_settings->scan_window_units);
        gap_start_scan();
        s_scanning = true;
    }
    s_applied = *p_settings;
    s_configured = true;
}

void BleRadio_SetAdvertisingData(const uint8_t * const p_data, uint8_t len)
{
    Check_WorkerContext();
    if ((p_data == NULL) || (len == 0U) || (len > BLERADIO_MAX_AD_BYTES) ||
        !BleRadio_IsReady() || !s_configured)
    {
        return;
    }
    memcpy(s_advertising_data, p_data, len);
    gap_advertisements_set_data(len, s_advertising_data);
    if (!s_advertising)
    {
        gap_advertisements_enable(1);
        s_advertising = true;
    }
}
