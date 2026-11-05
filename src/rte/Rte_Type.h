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
    SYS_STATE_HARD_FAULT,
    SYS_STATE_STANDBY,
    SYS_STATE_UNGRIPPED_WATCH,
    SYS_STATE_FAULT_DEGRADED,
    SYS_STATE_COUNT
} Sys_State_t;

typedef enum
{
    GRIP_STATE_RELEASED = 0U,
    GRIP_STATE_HELD,
    GRIP_STATE_UNKNOWN
} Grip_State_t;

/*
 * Sam's Cane Subsystem Telemetry Bit-Field (Curveball Minimum-Bit Constraint, Barr-C:2018)
 * Compactly maps application state into 16 bits (2 bytes).
 */
typedef struct
{
    uint16_t grip_state       : 2; /* 0=RELEASED, 1=HELD, 2=UNKNOWN */
    uint16_t fall_latched     : 1; /* Incident remains latched during audio pause */
    uint16_t tof_obstructed   : 1; /* 1 = sensor blinded/blocked */
    uint16_t imu_healthy      : 1; /* 1 = IMU responding */
    uint16_t link_lost        : 1; /* 1 = >500ms watchdog tripped */
    uint16_t tx_seq           : 3; /* 3-bit rolling packet sequence (0-7) */
    uint16_t fall_seq         : 4; /* 4-bit monotonic fall counter (0-15) */
    uint16_t reserved         : 3; /* Reserved for future cliff flags */
} Cane_CompactStatus_t;

typedef enum
{
    ALERT_NONE = 0U,
    ALERT_OBSTACLE_PROXIMITY,
    ALERT_CLIFF_WARNING,
    ALERT_CLIFF_STEP_UP,
    ALERT_FALL_ALARM,
    ALERT_FALL_MUTED,
    ALERT_SENSOR_FAULT,
    ALERT_LINK_LOSS,
    ALERT_CALIBRATE_PROMPT,
    ALERT_CAL_STARTED,
    ALERT_CAL_DONE,
    ALERT_CAL_FAILED
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

#define BLE_PACKET_SYNC_NIBBLE       (0xAU)
#define BLE_DEFAULT_PAIR_ID          (0x1U)

#define BLE_FLAG_FALL_ALERT          (0x01U)
#define BLE_FLAG_SENSOR_FAULT        (0x02U)
#define BLE_FLAG_OBSTACLE_ACTIVE     (0x04U)
#define BLE_FLAG_ALARM_PAUSED        (0x08U)
#define BLE_FLAG_CANE_HELD           (0x10U)
#define BLE_FLAG_STANDBY             (0x20U)
#define BLE_FLAG_PARKED_BY_USER      (0x40U)

#define BLE_DIST_NO_TARGET           (0x0FFFU)
#define BLE_TTC_NO_THREAT            (0x0FFFU)

typedef enum
{
    BLE_NODE_TYPE_GLASSES = 0U,
    BLE_NODE_TYPE_CANE    = 1U
} Ble_NodeType_t;

/*
 * 6-byte Compact Unified Packet schema (FR14, NFR13, Barr-C compliant):
 * Byte 0: [Preamble Sync (4b: 0xA) | Pair ID (4b: 0x0-0xF)]
 * Byte 1: [Node Type (1b) | Heartbeat Seq (3b: 0-7) | Rolling Fall Seq (4b: 0-15)]
 * Byte 2: [State Flags (8b)]
 * Byte 3: [Distance Low (8b: dist[7:0])] (Units: 2mm, Range: 0-8190mm)
 * Byte 4: [Distance High (4b: dist[11:8]) | TTC Low (4b: ttc[3:0])]
 * Byte 5: [TTC High (8b: ttc[11:4])] (Units: 20ms steps, Range: 0-81.9s)
 */
typedef struct
{
    uint8_t preamble_pair_id;
    uint8_t type_seq_fall;
    uint8_t state_flags;
    uint8_t dist_low;
    uint8_t dist_high_ttc_low;
    uint8_t ttc_high;
} __attribute__((packed)) Ble_RawPacket_t;

/* Decoded packet structure for internal SWC consumption */
typedef struct
{
    uint8_t preamble;             /* 0xAA or raw preamble */
    uint8_t device_id;            /* 0x01 = Cane, 0x02 = Glasses */
    uint8_t pair_id;              /* 0 - 15 */
    Ble_NodeType_t node_type;
    uint8_t heartbeat_seq;        /* 3-bit monotonic sequence (0 - 7) */
    uint8_t rolling_fall_seq;     /* 4-bit monotonic sequence (0 - 15) */
    uint16_t seq_num;             /* Backward compatible sequence number */
    uint8_t state_flags;          /* BLE_FLAG_* */
    uint16_t nearest_dist_mm;     /* 0 - 8190 mm, or 0xFFFF if invalid / no target */
    int16_t closing_speed_cms;
    uint16_t ttc_ms;              /* 0 - 81900 ms, or 0xFFFF if no threat */
} Ble_AdvancedThreatPacket_t;

#endif /* RTE_TYPE_H */
