#include "Diagnostics.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"

#if PICO_STDIO_USB_ENABLE_IRQ_BACKGROUND_TASK
#error USB protocol work must run in task context, not the stdio software IRQ
#endif

void Diagnostics_Task(void * p_parameters)
{
    (void)p_parameters;
    for (;;)
    {
        /* SDK flush takes its USB mutex and pumps TinyUSB even before CDC connects.
         * Using this path, rather than a concurrent raw tud_task(), preserves the
         * SDK's synchronization with printf/getchar on the two cores.
         */
        stdio_flush();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
