#ifndef BLERADIO_H
#define BLERADIO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t advertising_units; /* BLE units: 0.625 ms */
    uint16_t scan_interval_units;
    uint16_t scan_window_units;
} BleRadio_Settings_t;

/* Both callbacks run in the BTstack/FreeRTOS worker, never in an ISR. */
typedef void (*BleRadio_WorkCallback_t)(bool restarted, uint32_t now_ms);
typedef void (*BleRadio_RxCallback_t)(const uint8_t * p_data, uint8_t len, uint32_t now_ms);

/* Start once from the Core 1 radio task, after the scheduler starts. */
bool BleRadio_Start(const char * const p_name, BleRadio_WorkCallback_t work_callback,
                     BleRadio_RxCallback_t rx_callback);
void BleRadio_RequestWork(void);
bool BleRadio_IsReady(void);

/* Worker-context-only operations. The driver owns persistent HCI data buffers. */
void BleRadio_Configure(const BleRadio_Settings_t * const p_settings);
void BleRadio_SetAdvertisingData(const uint8_t * const p_data, uint8_t len);

#endif /* BLERADIO_H */
