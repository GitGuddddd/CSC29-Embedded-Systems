#include "drivers/Ld2450/Ld2450.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
 * \file fuzz_RadarFrames.c
 * \brief Fuzz testing harness for Ld2450 radar frame parser
 */

int main(void)
{
    srand((unsigned int)time(NULL));

    uint8_t buffer[64];
    Radar_Target_t targets[3];
    uint8_t count = 0U;

    printf("[FUZZ] Pumping 500,000 corrupt/random byte streams through Ld2450 parser...\n");
    for (int i = 0; i < 500000; i++)
    {
        uint32_t len = (uint32_t)(rand() % 64);
        for (uint32_t j = 0; j < len; j++)
        {
            buffer[j] = (uint8_t)(rand() & 0xFF);
        }

        /* 1 in 100 chance: inject valid header and footer with random payload */
        if ((rand() % 100) == 0 && (len >= 30))
        {
            buffer[0] = 0xAAU;
            buffer[1] = 0xFFU;
            buffer[2] = 0x03U;
            buffer[3] = 0x00U;
            buffer[28] = 0x55U;
            buffer[29] = 0xCCU;
        }

        (void)Ld2450_ParseFrame(buffer, len, targets, &count);
    }

    printf("[FUZZ] Complete: Ld2450 parser survived 500,000 cycles with zero segmentation faults.\n");
    return 0;
}
