#ifndef MOCK_BLERADIO_H
#define MOCK_BLERADIO_H

#include "drivers/BleRadio/BleRadio.h"

typedef struct
{
    BleRadio_Settings_t settings;
    uint8_t data[31];
    uint8_t data_len;
    uint32_t configure_count;
    uint32_t data_count;
    uint32_t wrong_context_count;
    uint32_t starts;
} MockBleRadio_State_t;

void MockBleRadio_Reset(void);
void MockBleRadio_SetTime(uint32_t now_ms);
void MockBleRadio_SetReady(bool ready);
void MockBleRadio_SetWorkerRunning(bool running);
void MockBleRadio_Receive(const uint8_t * const p_data, uint8_t len);
const MockBleRadio_State_t * MockBleRadio_GetState(void);

#endif /* MOCK_BLERADIO_H */
