/**
 * @file    node_status.c
 * @brief   Packed status flags and counters of the glasses node.
 *
 * WARNING: Disabling interrupts protects the bytes against interrupt
 * handlers and callbacks that run on the same core. If the second core
 * ever touches this data, replace the critical section with a hardware
 * spinlock.
 *
 * NOTE: The two bytes are separate variables on purpose. A change to one
 * byte can never overwrite a bit of the other byte.
 */

#include "node_status.h"

#include "hardware/sync.h"

/* Byte 0: flags and the fall detector state. Bit-fields are used only for
 * private data, never on the wire. Every field has an unsigned base type.
 * The fields fill all 8 bits. */
typedef struct
{
    uint8_t b_is_sensor_fault : 1;
    uint8_t b_is_fall_latched : 1;
    uint8_t b_is_fall_unacked : 1;
    uint8_t fail_count        : 2;
    uint8_t fall_state        : 3;
} node_flags_t;

/* Byte 1: counter and one-time flag. Five bits used, three bits spare. */
typedef struct
{
    uint8_t fall_count    : 4;
    uint8_t b_is_cal_done : 1;
} node_counts_t;

/* Compile-time size checks: the array size is negative if a record is not
 * exactly one byte, which stops the build. */
typedef char node_flags_size_check_t[(1u == sizeof(node_flags_t)) ? 1 : -1];
typedef char node_counts_size_check_t[(1u == sizeof(node_counts_t)) ? 1 : -1];

/* Volatile because callbacks or interrupts may use them as well. */
static volatile node_flags_t  g_flags;
static volatile node_counts_t g_counts;

/**
 * @brief Clears all fields and sets the sensor fault flag.
 */
void node_status_init (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_flags.b_is_sensor_fault = 1u;
    g_flags.b_is_fall_latched = 0u;
    g_flags.b_is_fall_unacked = 0u;
    g_flags.fail_count        = 0u;
    g_flags.fall_state        = 0u;
    g_counts.fall_count       = 0u;
    g_counts.b_is_cal_done    = 0u;

    restore_interrupts(irq_state);
}

/**
 * @brief  Reads the sensor fault flag.
 * @return true while the sensor is in the fault state.
 */
bool node_status_is_sensor_fault (void)
{
    return (0u != g_flags.b_is_sensor_fault);
}

/**
 * @brief Sets or clears the sensor fault flag.
 * @param b_is_fault  true to set the flag, false to clear it.
 */
void node_status_set_sensor_fault (bool b_is_fault)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_flags.b_is_sensor_fault = b_is_fault;

    restore_interrupts(irq_state);
}

/**
 * @brief  Reads the gyro calibration flag.
 * @return true once the gyro has been calibrated or skipped.
 */
bool node_status_is_cal_done (void)
{
    return (0u != g_counts.b_is_cal_done);
}

/**
 * @brief Sets the gyro calibration flag.
 */
void node_status_set_cal_done (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_counts.b_is_cal_done = 1u;

    restore_interrupts(irq_state);
}

/**
 * @brief  Counts one failed sensor read, saturating at the maximum.
 * @return The failed-read count after the increment.
 */
uint8_t node_status_note_read_fail (void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint8_t  count;

    if (g_flags.fail_count < NODE_STATUS_FAIL_COUNT_MAX)
    {
        g_flags.fail_count++;
    }
    count = g_flags.fail_count;

    restore_interrupts(irq_state);

    return count;
}

/**
 * @brief Clears the failed-read counter.
 */
void node_status_clear_read_fails (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_flags.fail_count = 0u;

    restore_interrupts(irq_state);
}

/**
 * @brief Records a confirmed fall.
 */
void node_status_fall_detected (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_flags.b_is_fall_latched = 1u;
    g_flags.b_is_fall_unacked = 1u;
    if (g_counts.fall_count < NODE_STATUS_FALL_COUNT_MAX)
    {
        g_counts.fall_count++;
    }

    restore_interrupts(irq_state);
}

/**
 * @brief  Reads the latched flag.
 * @return true during the fast-blink window after a fall.
 */
bool node_status_is_fall_latched (void)
{
    return (0u != g_flags.b_is_fall_latched);
}

/**
 * @brief Clears the latched flag at the end of the fast-blink window.
 */
void node_status_clear_fall_latched (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_flags.b_is_fall_latched = 0u;

    restore_interrupts(irq_state);
}

/**
 * @brief  Reads the unacknowledged flag.
 * @return true until the wearer acknowledges the fall.
 */
bool node_status_is_fall_unacked (void)
{
    return (0u != g_flags.b_is_fall_unacked);
}

/**
 * @brief Acknowledges the fall: clears the unacknowledged flag.
 */
void node_status_fall_ack (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    g_flags.b_is_fall_unacked = 0u;

    restore_interrupts(irq_state);
}

/**
 * @brief  Reads the fall counter.
 * @return Number of falls since start-up, at most 15.
 */
uint8_t node_status_fall_count (void)
{
    return g_counts.fall_count;
}

/**
 * @brief  Reads the fall detector state number.
 * @return The state number, 0 to NODE_STATUS_FALL_STATE_MAX.
 */
uint8_t node_status_get_fall_state (void)
{
    return g_flags.fall_state;
}

/**
 * @brief Stores the fall detector state number.
 * @param state  New state number. A value above the maximum is ignored.
 */
void node_status_set_fall_state (uint8_t state)
{
    uint32_t irq_state;

    if (state <= NODE_STATUS_FALL_STATE_MAX)
    {
        irq_state = save_and_disable_interrupts();

        /* Cast: the value was checked above, so the mask loses nothing. */
        g_flags.fall_state = (uint8_t)(state & NODE_STATUS_FALL_STATE_MAX);

        restore_interrupts(irq_state);
    }
}

/* end of node_status.c */

