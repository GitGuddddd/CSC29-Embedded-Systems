#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

typedef struct
{
    /* Optional nonblocking producer queue drain, called once per USB service cycle. */
    void (*drain_log)(void);
} Diagnostics_Config_t;

/* FreeRTOS task entry. Parameters may point to a Diagnostics_Config_t. */
void Diagnostics_Task(void * p_parameters);

#endif /* DIAGNOSTICS_H */
