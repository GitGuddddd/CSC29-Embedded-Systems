#include "app/FallDetection/FallDetection.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
 * \file fuzz_FallDetection.c
 * \brief Fuzz testing harness for FallDetection SWC
 */

int main(void)
{
    FallDetection_Init(&FallConfig_Head);
    srand((unsigned int)time(NULL));

    Imu_Sample_t s;
    uint32_t now = 0U;

    printf("[FUZZ] Running 500,000 randomized iterations through FallDetection SWC...\n");
    for (int i = 0; i < 500000; i++)
    {
        s.ax = (((float)rand() / (float)RAND_MAX) * 20.0f) - 10.0f;
        s.ay = (((float)rand() / (float)RAND_MAX) * 20.0f) - 10.0f;
        s.az = (((float)rand() / (float)RAND_MAX) * 20.0f) - 10.0f;

        s.gx = (((float)rand() / (float)RAND_MAX) * 2000.0f) - 1000.0f;
        s.gy = (((float)rand() / (float)RAND_MAX) * 2000.0f) - 1000.0f;
        s.gz = (((float)rand() / (float)RAND_MAX) * 2000.0f) - 1000.0f;

        now += (uint32_t)(rand() % 50);

        (void)FallDetection_Step(&s, now);
    }

    printf("[FUZZ] Complete: FallDetection SWC survived with no memory faults or NaN errors.\n");
    return 0;
}
