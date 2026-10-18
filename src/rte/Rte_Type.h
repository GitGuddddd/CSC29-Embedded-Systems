#ifndef RTE_TYPE_H
#define RTE_TYPE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Rte_Type.h
 * \brief Standard types, PDUs, and enumerations shared across SWCs
 * Compliance: Barr-C:2018
 */

typedef enum
{
    RTE_STATUS_OK = 0U,
    RTE_STATUS_ERROR,
    RTE_STATUS_TIMEOUT,
    RTE_STATUS_BUSY
} Rte_Status_t;

typedef enum
{
    SYS_STATE_INIT = 0U,
    SYS_STATE_IDLE,
    SYS_STATE_ACTIVE,
    SYS_STATE_WATCH,
    SYS_STATE_FALL_CONFIRMED,
    SYS_STATE_FALL_MUTED,
    SYS_STATE_HARD_FAULT
} Sys_State_t;

typedef enum
{
    GRIP_STATE_RELEASED = 0U,
    GRIP_STATE_HELD,
    GRIP_STATE_UNKNOWN
} Grip_State_t;

typedef enum
{
    ALERT_NONE = 0U,
    ALERT_OBSTACLE_PROXIMITY,
    ALERT_CLIFF_WARNING,
    ALERT_CLIFF_STEP_UP,
    ALERT_FALL_ALARM,
    ALERT_FALL_MUTED,
    ALERT_SENSOR_FAULT,
    ALERT_LINK_LOSS
} Alert_Type_t;

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint32_t timestamp_ms;
} Imu_Sample_t;

typedef struct
{
    uint16_t distance_mm;
    int16_t speed_cms;
    int16_t angle_deg;
    bool is_valid;
} Radar_Target_t;

/*
 * 11-byte AdvancedThreatPacket schema per FR14 & Section 8.3:
 * [Preamble(1B), DeviceID(1B), SeqNum(2B), StateFlags(1B), TargetDist(2B), TargetSpeed(2B), TTC(2B)]
 */
typedef struct
{
    uint8_t preamble;
    uint8_t device_id;
    uint16_t seq_num;
    uint8_t state_flags;
    uint16_t nearest_dist_mm;
    int16_t closing_speed_cms;
    uint16_t ttc_ms;
} __attribute__((packed)) Ble_AdvancedThreatPacket_t;

#endif /* RTE_TYPE_H */
