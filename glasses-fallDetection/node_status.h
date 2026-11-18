/**
 * @file    node_status.h
 * @brief   Packed status flags and counters of the glasses node.
 *
 * All flags, small counters and the fall detector state live in two
 * private bytes:
 *   - Byte 0 (8 of 8 bits used): sensor fault flag, fall latched flag, fall
 *     unacknowledged flag, failed-read counter and fall detector state.
 *   - Byte 1 (5 of 8 bits used): fall counter and gyro calibration flag.
 * Other modules read and change them only through the functions below.
 * Each function that changes a byte does so inside a critical section, so
 * a change made from an interrupt or a callback cannot be lost.
 */

#ifndef NODE_STATUS_H
#define NODE_STATUS_H

#include <stdbool.h>
#include <stdint.h>

/* The failed-read counter is 2 bits wide, so it saturates at 3. */
#define NODE_STATUS_FAIL_COUNT_MAX  3u

/* The fall counter is 4 bits wide, so it saturates at 15. */
#define NODE_STATUS_FALL_COUNT_MAX  15u

/* The fall detector state is 3 bits wide, so it holds 0 to 7. */
#define NODE_STATUS_FALL_STATE_MAX  7u

/* Clears all fields. The sensor fault flag starts set until the first
 * successful sensor start-up clears it. */
void node_status_init (void);

/* Sensor fault flag. */
bool node_status_is_sensor_fault (void);
void node_status_set_sensor_fault (bool b_is_fault);

/* Gyro calibration done flag. Set once, never cleared. */
bool node_status_is_cal_done (void);
void node_status_set_cal_done (void);

/* Counts one failed sensor read, saturating at NODE_STATUS_FAIL_COUNT_MAX.
 * Returns the count after the increment. */
uint8_t node_status_note_read_fail (void);
void    node_status_clear_read_fails (void);

/* Records a confirmed fall: sets the latched and unacknowledged flags and
 * counts the fall, saturating at NODE_STATUS_FALL_COUNT_MAX. */
void node_status_fall_detected (void);

/* Latched flag: true during the fast-blink window after a fall. */
bool node_status_is_fall_latched (void);
void node_status_clear_fall_latched (void);

/* Unacknowledged flag: true until the wearer presses the button. */
bool node_status_is_fall_unacked (void);
void node_status_fall_ack (void);

/* Number of falls since start-up, saturated at 15. */
uint8_t node_status_fall_count (void);

/* Fall detector state number. The meaning of the numbers belongs to
 * fall_detect.c. A value above NODE_STATUS_FALL_STATE_MAX is ignored. */
uint8_t node_status_get_fall_state (void);
void    node_status_set_fall_state (uint8_t state);

#endif /* NODE_STATUS_H */

/* end of node_status.h */

