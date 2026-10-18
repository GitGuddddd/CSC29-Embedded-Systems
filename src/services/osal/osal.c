/** @file osal.c
 *
 * @brief Operating-system adapter on uT-Kernel 3.0 (the course template).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Built with the course template (dual-core SMP kernel on the Pico W). It
 * compiles and links in the template; it has not yet run on the Pico.
 *
 * Queues and the byte stream use message buffers (copy in, copy out).
 * tk_snd_mbf may be called from an interrupt handler only with TMO_POL,
 * which is what the stream uses.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tk_include.h"

#include "osal.h"

// uT-Kernel priority 1 is the highest. Abstract level n maps to this base
// minus n, so a higher level gives a smaller (more urgent) number. With the
// base 22 the Medium tasks run at 20 and the BLE task at 21, which is the
// priority band 20-21 of subsystem 5 in TEAM.md.
#define OSAL_BASE_PRIORITY    (22)

// Bytes per stack word (the API takes the stack size in bytes).
#define OSAL_BYTES_PER_WORD   (4u)

// The kernel accounts a length field with every message; the stream gets
// twice its payload size so many small chunks still fit.
#define OSAL_STREAM_FACTOR    (2u)

// Most tasks this adapter can start (one start-up context each).
#define OSAL_MAX_TASKS        (8u)

typedef struct
{
    osal_task_fn_t p_entry;
    void *         p_arg;
} osal_task_ctx_t;

static osal_task_ctx_t g_task_ctx[OSAL_MAX_TASKS];
static size_t g_task_count = 0u;

static TMO osal_to_tmo (uint32_t timeout_ms);
static void osal_task_trampoline (INT stacd, void * p_exinf);

/*!
 * @brief Create a queue of fixed-size items (a message buffer).
 *
 * @param[out] p_queue    Receives the queue.
 * @param[in]  item_size  Size of one item in bytes.
 * @param[in]  length     Number of items.
 *
 * @return true when the queue was created.
 */
bool
osal_queue_create (osal_queue_t * p_queue, size_t item_size, size_t length)
{
    T_CMBF cmbf = { 0 };
    ID id;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ) (length * (item_size + sizeof(INT)));
    cmbf.maxmsz = (INT) item_size;

    id = tk_cre_mbf(&cmbf);
    p_queue->p_handle = NULL;
    p_queue->id = (int32_t) id;
    p_queue->item_size = item_size;

    return (id > 0);
}

/*!
 * @brief Copy one item into a queue.
 *
 * @param[in] p_queue     The queue.
 * @param[in] p_item      Item to copy.
 * @param[in] timeout_ms  Longest wait for space, 0 = do not wait.
 *
 * @return true when the item was queued.
 */
bool
osal_queue_send (const osal_queue_t * p_queue, const void * p_item,
                 uint32_t timeout_ms)
{
    return (E_OK == tk_snd_mbf((ID) p_queue->id, p_item,
                               (INT) p_queue->item_size,
                               osal_to_tmo(timeout_ms)));
}

/*!
 * @brief Copy the oldest item out of a queue.
 *
 * @param[in]  p_queue     The queue.
 * @param[out] p_item      Receives the item.
 * @param[in]  timeout_ms  Longest wait, or OSAL_WAIT_FOREVER.
 *
 * @return true when an item was received.
 */
bool
osal_queue_receive (const osal_queue_t * p_queue, void * p_item,
                    uint32_t timeout_ms)
{
    // A positive result is the size of the received message.
    return (tk_rcv_mbf((ID) p_queue->id, p_item,
                       osal_to_tmo(timeout_ms)) > 0);
}

/*!
 * @brief Create a byte stream for one interrupt writer and one task reader.
 *
 * @param[out] p_stream    Receives the stream.
 * @param[in]  size_bytes  Capacity in bytes.
 *
 * @return true when the stream was created.
 */
bool
osal_stream_create (osal_stream_t * p_stream, size_t size_bytes)
{
    T_CMBF cmbf = { 0 };
    ID id;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ) (size_bytes * OSAL_STREAM_FACTOR);
    cmbf.maxmsz = (INT) OSAL_STREAM_MAX_CHUNK;

    id = tk_cre_mbf(&cmbf);
    p_stream->p_handle = NULL;
    p_stream->id = (int32_t) id;
    p_stream->residue_length = 0u;
    p_stream->residue_offset = 0u;

    return (id > 0);
}

/*!
 * @brief Write one chunk from an interrupt handler (never waits).
 *
 * WARNING: Runs in interrupt context, so the timeout must be TMO_POL.
 *
 * @param[in,out] p_stream  The stream.
 * @param[in]     p_src     Bytes to write.
 * @param[in]     length    Number of bytes, at most OSAL_STREAM_MAX_CHUNK.
 *
 * @return Bytes accepted: all of them, or 0 when the chunk was dropped.
 */
size_t
osal_stream_send_from_isr (osal_stream_t * p_stream, const uint8_t * p_src,
                           size_t length)
{
    size_t accepted = 0u;

    if ((length > 0u) && (length <= OSAL_STREAM_MAX_CHUNK))
    {
        if (E_OK == tk_snd_mbf((ID) p_stream->id, p_src, (INT) length,
                               TMO_POL))
        {
            accepted = length;
        }
    }

    return (accepted);
}

/*!
 * @brief Read bytes, blocking until some arrive or time runs out.
 *
 * Messages arrive whole, so unread bytes are kept in the stream and given
 * out first on the next call. Only one task may read the stream.
 *
 * @param[in,out] p_stream    The stream.
 * @param[out]    p_dst       Destination buffer.
 * @param[in]     max_length  Size of the destination buffer.
 * @param[in]     timeout_ms  Longest wait in ms, 0 = poll.
 *
 * @return Number of bytes copied (0 on timeout).
 */
size_t
osal_stream_receive (osal_stream_t * p_stream, uint8_t * p_dst,
                     size_t max_length, uint32_t timeout_ms)
{
    size_t copied = 0u;

    if (p_stream->residue_offset >= p_stream->residue_length)
    {
        INT received = tk_rcv_mbf((ID) p_stream->id, p_stream->residue,
                                  osal_to_tmo(timeout_ms));

        p_stream->residue_offset = 0u;
        p_stream->residue_length = 0u;

        if (received > 0)
        {
            // The cast is safe: received was just checked to be positive.
            p_stream->residue_length = (size_t) received;
        }
    }

    while ((copied < max_length) &&
           (p_stream->residue_offset < p_stream->residue_length))
    {
        p_dst[copied] = p_stream->residue[p_stream->residue_offset];
        copied++;
        p_stream->residue_offset++;
    }

    return (copied);
}

/*!
 * @brief Create and start a task.
 *
 * @param[in] p_entry     Task function.
 * @param[in] stack_words Stack size in 32-bit words.
 * @param[in] priority    Abstract priority, higher = more urgent.
 * @param[in] p_arg       Argument passed to the task.
 *
 * @return true when the task was created and started.
 */
bool
osal_task_create (osal_task_fn_t p_entry, uint32_t stack_words,
                  osal_priority_t priority, void * p_arg)
{
    bool b_ok = false;
    T_CTSK ctsk = { 0 };

    if (g_task_count < OSAL_MAX_TASKS)
    {
        osal_task_ctx_t * p_ctx = &g_task_ctx[g_task_count];
        ID id;

        p_ctx->p_entry = p_entry;
        p_ctx->p_arg = p_arg;

        ctsk.exinf = p_ctx;
        ctsk.tskatr = TA_HLNG | TA_RNG3;
        // The cast is needed: the kernel declares its task type as a
        // function pointer without a parameter list.
        ctsk.task = (FP) osal_task_trampoline;
        ctsk.itskpri = (PRI) (OSAL_BASE_PRIORITY - (INT) priority);
        ctsk.stksz = (SZ) (stack_words * OSAL_BYTES_PER_WORD);

        id = tk_cre_tsk(&ctsk);

        if ((id > 0) && (E_OK == tk_sta_tsk(id, 0)))
        {
            g_task_count++;
            b_ok = true;
        }
    }

    return (b_ok);
}

/*!
 * @brief Block the calling task.
 *
 * @param[in] delay_ms  Time in milliseconds (one kernel tick = 1 ms).
 */
void
osal_delay_ms (uint32_t delay_ms)
{
    (void) tk_dly_tsk((RELTIM) delay_ms);
}

/*!
 * @brief The kernel is already running when usermain() is called, so this
 *        only parks the calling task.
 */
void
osal_start_scheduler (void)
{
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
    }
}

/*!
 * @brief Convert a timeout in ms to a kernel timeout.
 *
 * @param[in] timeout_ms  Timeout in milliseconds.
 *
 * @return TMO_FEVR for OSAL_WAIT_FOREVER, otherwise the time in ms.
 */
static TMO
osal_to_tmo (uint32_t timeout_ms)
{
    return ((OSAL_WAIT_FOREVER == timeout_ms) ? TMO_FEVR : (TMO) timeout_ms);
}

/*!
 * @brief Task start-up function: call the real task with its argument.
 *
 * @param[in] stacd    Start code (unused).
 * @param[in] p_exinf  The start-up context of this task.
 */
static void
osal_task_trampoline (INT stacd, void * p_exinf)
{
    const osal_task_ctx_t * p_ctx = p_exinf;

    // The void cast only silences the unused-parameter warning.
    (void) stacd;

    p_ctx->p_entry(p_ctx->p_arg);

    // A task function never returns; end the task if it does.
    tk_ext_tsk();
}

/*** end of file ***/
