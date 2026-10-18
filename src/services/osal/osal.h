/** @file osal.h
 *
 * @brief Operating-system adapter: tasks, queues, byte stream, delay.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The radar code calls only these functions. They are implemented on
 * uT-Kernel 3.0 in osal.c (Week 6 design, NFR12). The adapter keeps the
 * kernel calls in one file, so the radar, threat and RTE code do not change
 * if the kernel API changes.
 */

#ifndef OSAL_H
#define OSAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Timeout value meaning "block until it happens". */
#define OSAL_WAIT_FOREVER        (0xFFFFFFFFu)

/** Largest chunk the interrupt may send in one osal_stream_send_from_isr. */
#define OSAL_STREAM_MAX_CHUNK    (64u)

/** Abstract task priority: a higher number is a higher priority. */
typedef uint8_t osal_priority_t;

/** Task entry function. */
typedef void (*osal_task_fn_t)(void * p_arg);

/** Queue of fixed-size items (copy in, copy out). */
typedef struct
{
    void *  p_handle;   /**< Backend handle (pointer style). */
    int32_t id;         /**< Backend handle (identifier style). */
    size_t  item_size;  /**< Size of one item in bytes. */
} osal_queue_t;

/** Byte stream written by one interrupt and read by one task. */
typedef struct
{
    void *  p_handle;                          /**< Backend handle. */
    int32_t id;                                /**< Backend handle. */
    uint8_t residue[OSAL_STREAM_MAX_CHUNK];    /**< Unread bytes (uT-Kernel). */
    size_t  residue_length;                    /**< Bytes left in residue. */
    size_t  residue_offset;                    /**< Next residue byte. */
} osal_stream_t;

bool osal_queue_create (osal_queue_t * p_queue, size_t item_size,
                        size_t length);
bool osal_queue_send (const osal_queue_t * p_queue, const void * p_item,
                      uint32_t timeout_ms);
bool osal_queue_receive (const osal_queue_t * p_queue, void * p_item,
                         uint32_t timeout_ms);

bool osal_stream_create (osal_stream_t * p_stream, size_t size_bytes);
size_t osal_stream_send_from_isr (osal_stream_t * p_stream,
                                  const uint8_t * p_src, size_t length);
size_t osal_stream_receive (osal_stream_t * p_stream, uint8_t * p_dst,
                            size_t max_length, uint32_t timeout_ms);

bool osal_task_create (osal_task_fn_t p_entry, uint32_t stack_words,
                       osal_priority_t priority, void * p_arg);
void osal_delay_ms (uint32_t delay_ms);
void osal_start_scheduler (void);

#endif /* OSAL_H */

/*** end of file ***/
