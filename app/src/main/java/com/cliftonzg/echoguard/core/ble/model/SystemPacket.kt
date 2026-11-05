package com.cliftonzg.echoguard.core.ble.model

enum class ThreatLevel {
    CRITICAL,
    WARNING,
    SAFE,
    UNKNOWN
}

enum class NodeType(val deviceId: Int, val label: String) {
    CANE(0x01, "Smart Cane"),
    GLASSES(0x02, "Smart Glasses"),
    LEGACY(0x00, "Legacy Node"),
}

/** Observable telemetry states, not the complete firmware Sys_State_t (which is not transmitted). */
enum class TelemetryStatus {
    FALL_ALARM,
    FALL_PAUSED,
    STANDBY,
    SENSOR_FAULT,
    GROUND_HAZARD,
    OBSTACLE_CRITICAL,
    OBSTACLE_WARNING,
    GRIP_RELEASED,
    MONITORING,
    NO_READING,
}

/**
 * Represents a packet received over BLE from the EchoGuard system.
 * Supports 5-byte basic packet schema, 10-11 byte AdvancedThreatPacket schema,
 * and compact status with the six-byte v2 receipt extension (12 bytes total).
 * The former 24-byte v1 frame is readable for telemetry only.
 */
data class SystemPacket(
    val magic: Int,
    val seqNum: Int,
    val radarDistMm: Int,
    val checksum: Int,
    val deviceId: Int = 0,
    val stateFlags: Int = 0,
    val closingSpeedCms: Int = 0,
    val ttcMs: Int = -1,
    val isAdvanced: Boolean = false,
    val compactFormat: Boolean = false,
    val rollingFallSeq: Int = 0,
    val pairId: Int = 0,
    val protocolVersion: Int = 0,
    val sessionId: Long = 0L,
    val eventId: Long = rollingFallSeq.toLong(),
    val sessionRollover: Boolean = false,
    val peerAckValid: Boolean = false,
    val peerAckSessionId: Long = 0L,
    val peerAckEventId: Long = 0L,
    // Monotonic capture time: ScanResult.timestampNanos / 1_000_000, not wall-clock time.
    val receivedAt: Long = 0L,
) {
    val isFallAlert: Boolean get() = (stateFlags and FLAG_FALL_ALERT) != 0
    val isAlarmPaused: Boolean get() = (stateFlags and FLAG_ALARM_PAUSED) != 0
    val hasLatchedFall: Boolean get() = isFallAlert || isAlarmPaused
    val isSensorFault: Boolean get() = (stateFlags and FLAG_SENSOR_FAULT) != 0
    val isRadarFault: Boolean get() = isSensorFault // Compatibility with legacy callers.
    val isObstacleActive: Boolean get() = (stateFlags and FLAG_OBSTACLE_ACTIVE) != 0
    val isStandby: Boolean get() = (stateFlags and FLAG_STANDBY) != 0
    val isParkedByUser: Boolean get() = (stateFlags and FLAG_PARKED_BY_USER) != 0
    val hasGripTelemetry: Boolean get() = compactFormat && nodeType == NodeType.CANE
    val isCaneHeld: Boolean get() = nodeType == NodeType.CANE && (stateFlags and FLAG_CANE_HELD) != 0
    val hasDistance: Boolean get() = radarDistMm >= 0
    val hasClosingSpeed: Boolean get() = isAdvanced && !compactFormat
    val nodeType: NodeType
        get() = when (deviceId) {
            0x01 -> NodeType.CANE
            0x02 -> NodeType.GLASSES
            else -> NodeType.LEGACY
        }

    val threatLevel: ThreatLevel
        get() = when {
            hasLatchedFall -> ThreatLevel.CRITICAL
            isStandby || isSensorFault -> ThreatLevel.UNKNOWN
            (isAdvanced || compactFormat) && nodeType == NodeType.CANE -> when {
                isObstacleActive -> ThreatLevel.CRITICAL
                hasGripTelemetry && !isCaneHeld -> ThreatLevel.UNKNOWN
                radarDistMm <= 0 -> ThreatLevel.UNKNOWN
                else -> ThreatLevel.SAFE // Firmware classifies against its calibrated ground plane.
            }
            (isAdvanced || compactFormat) && nodeType == NodeType.GLASSES -> when {
                isObstacleActive && radarDistMm in 1..799 -> ThreatLevel.CRITICAL
                isObstacleActive -> ThreatLevel.WARNING
                radarDistMm == 0 -> ThreatLevel.UNKNOWN
                else -> ThreatLevel.SAFE // Distance alone must not undo the firmware's threat filtering.
            }
            deviceId == 0x01 -> {
                // Legacy distance-only data has no calibrated hazard flag.
                when {
                    radarDistMm in 1..1500 -> ThreatLevel.SAFE
                    radarDistMm > 1500 || radarDistMm < 0 -> ThreatLevel.CRITICAL
                    else -> ThreatLevel.UNKNOWN
                }
            }
            else -> {
                // Glasses Radar (Forward obstacle sensor):
                // Small distances (< 800 mm) = Proximity Danger -> CRITICAL (Red)
                // Medium distances (800..1500 mm) = WARNING (Orange)
                // Far distances (> 1500 mm or -1 / no target) = SAFE (Green)
                when {
                    radarDistMm in 1..799 -> ThreatLevel.CRITICAL
                    radarDistMm in 800..1500 -> ThreatLevel.WARNING
                    radarDistMm > 1500 || radarDistMm < 0 -> ThreatLevel.SAFE
                    else -> ThreatLevel.UNKNOWN
                }
            }
        }

    fun isValid(): Boolean {
        if (compactFormat) return isCompactValid()
        if (magic != 0xAA) return false

        if (isAdvanced) {
            return (deviceId == 0x01 || deviceId == 0x02)
        }
        val distLow = radarDistMm and 0xFF
        val distHigh = (radarDistMm shr 8) and 0xFF
        val calculated = (magic xor seqNum xor distLow xor distHigh) and 0xFF
        return calculated == (checksum and 0xFF)
    }

    val status: TelemetryStatus
        get() = when {
            hasLatchedFall && isAlarmPaused -> TelemetryStatus.FALL_PAUSED
            hasLatchedFall -> TelemetryStatus.FALL_ALARM
            isStandby -> TelemetryStatus.STANDBY
            isSensorFault -> TelemetryStatus.SENSOR_FAULT
            nodeType == NodeType.CANE && threatLevel == ThreatLevel.CRITICAL -> TelemetryStatus.GROUND_HAZARD
            hasGripTelemetry && !isCaneHeld -> TelemetryStatus.GRIP_RELEASED
            threatLevel == ThreatLevel.CRITICAL -> TelemetryStatus.OBSTACLE_CRITICAL
            threatLevel == ThreatLevel.WARNING -> TelemetryStatus.OBSTACLE_WARNING
            threatLevel == ThreatLevel.SAFE -> TelemetryStatus.MONITORING
            else -> TelemetryStatus.NO_READING
        }

    private fun isCompactValid(): Boolean {
        // Compact 6-byte packed schema: deviceId derived from node type (0x01=Cane, 0x02=Glasses)
        if (magic != 0xAA || (deviceId != 0x01 && deviceId != 0x02) ||
            pairId !in 0 until 16 || seqNum !in 0 until 8 || rollingFallSeq !in 0 until 16
        ) return false
        return when (protocolVersion) {
            0 -> true
            LEGACY_PROTOCOL_VERSION -> sessionId in 1L..0xFFFFFFFFL
            PROTOCOL_VERSION -> sessionId in 1L..0xFFFFL && eventId == rollingFallSeq.toLong() &&
                if (peerAckValid) {
                    nodeType == NodeType.CANE && peerAckSessionId in 1L..0xFFFFL && peerAckEventId in 0L..15L
                } else {
                    peerAckSessionId == 0L && peerAckEventId == 0L
                }
            else -> false
        }
    }

    /**
     * Monotonic sequence number check per FR15.
     * Rejects stale or out-of-order sequence numbers (accounting for wrap-around).
     */
    fun isNewerThan(lastSeqNum: Int): Boolean {
        if (lastSeqNum < 0) return true
        if (seqNum == lastSeqNum) return false
        val maxSeq = when {
            compactFormat -> 8  // 3-bit heartbeat sequence (0-7)
            isAdvanced -> 65536 // 16-bit sequence
            else -> 256         // 8-bit sequence
        }
        val diff = ((seqNum - lastSeqNum) + maxSeq) % maxSeq
        return diff in 1..(maxSeq / 2)
    }

    /** Matches firmware short-counter rebasing, while rejecting out-of-order Android batches. */
    fun isFreshAfter(previous: SystemPacket?): Boolean {
        if (previous == null) return true
        if (receivedAt <= previous.receivedAt) return false
        if (compactFormat != previous.compactFormat || isAdvanced != previous.isAdvanced) return true
        if (!compactFormat) return isNewerThan(previous.seqNum)
        if (protocolVersion != previous.protocolVersion || sessionId != previous.sessionId) return true
        if (protocolVersion == PROTOCOL_VERSION && rollingFallSeq < previous.rollingFallSeq) return false
        val delta = (seqNum - previous.seqNum) and 0x07
        val statusChanged = stateFlags != previous.stateFlags ||
            rollingFallSeq != previous.rollingFallSeq || sessionId != previous.sessionId ||
            eventId != previous.eventId || peerAckValid != previous.peerAckValid ||
            peerAckSessionId != previous.peerAckSessionId || peerAckEventId != previous.peerAckEventId
        if (delta == 0) return statusChanged
        return delta <= 4 || receivedAt - previous.receivedAt >= SEQUENCE_REBASE_MS
    }

    companion object {
        const val FLAG_FALL_ALERT = 0x01
        const val FLAG_SENSOR_FAULT = 0x02
        const val FLAG_RADAR_FAULT = FLAG_SENSOR_FAULT
        const val FLAG_OBSTACLE_ACTIVE = 0x04
        const val FLAG_ALARM_PAUSED = 0x08
        const val FLAG_CANE_HELD = 0x10
        const val FLAG_STANDBY = 0x20
        const val FLAG_PARKED_BY_USER = 0x40
        const val LINK_TIMEOUT_MS = 500L
        const val SEQUENCE_REBASE_MS = 250L
        const val PROTOCOL_VERSION = 2
        const val SAFETY_EXTENSION_MARKER = 0xE2
        const val SAFETY_FLAG_PEER_ACK = 0x10
        const val SAFETY_FLAG_ROLLOVER = 0x20
        const val SAFETY_RESERVED_MASK = 0xC0
        const val LEGACY_PROTOCOL_VERSION = 1
        const val LEGACY_SAFETY_EXTENSION_MARKER = 0xE1

        private fun readU16Le(data: ByteArray, offset: Int): Long =
            (data[offset].toLong() and 0xFFL) or ((data[offset + 1].toLong() and 0xFFL) shl 8)

        private fun readU32Le(data: ByteArray, offset: Int): Long =
            (data[offset].toLong() and 0xFFL) or
                ((data[offset + 1].toLong() and 0xFFL) shl 8) or
                ((data[offset + 2].toLong() and 0xFFL) shl 16) or
                ((data[offset + 3].toLong() and 0xFFL) shl 24)

        fun parse(data: ByteArray, timestamp: Long = 0L): SystemPacket? {
            if (data.size > 6 && (data[0].toInt() and 0xF0) == 0xA0 &&
                (data[6].toInt() and 0xF0) == 0xE0
            ) {
                // Pair 10 makes the compact sync byte 0xAA, like the old advanced
                // preamble. A marked but truncated extension must not fall through.
                val marker = data[6].toInt() and 0xFF
                if (!((data.size == 12 && marker == SAFETY_EXTENSION_MARKER) ||
                        (data.size == 24 && marker == LEGACY_SAFETY_EXTENSION_MARKER))
                ) return null
            }
            return when {
                data.size == 10 || data.size == 11 -> {
                    // AdvancedThreatPacket Schema (10 to 11 bytes):
                    // [Preamble(1B), DeviceID(1B), SeqNum(2B), StateFlags(1B), TargetDist(2B), TargetSpeed(2B), TTC(2B)]
                    val preamble = data[0].toInt() and 0xFF
                    val devId = data[1].toInt() and 0xFF
                    val seq = (data[2].toInt() and 0xFF) or ((data[3].toInt() and 0xFF) shl 8)
                    val flags = data[4].toInt() and 0xFF
                    val rawDist = (data[5].toInt() and 0xFF) or ((data[6].toInt() and 0xFF) shl 8)
                    val dist = if (rawDist == 0xFFFF) -1 else rawDist
                    val speed = (data[7].toInt() and 0xFF) or (data[8].toInt() shl 8)
                    val rawTtc = if (data.size == 11) (data[9].toInt() and 0xFF) or ((data[10].toInt() and 0xFF) shl 8) else 0xFFFF
                    val ttc = if (rawTtc == 0xFFFF) -1 else rawTtc

                    val packet = SystemPacket(
                        magic = preamble,
                        seqNum = seq,
                        radarDistMm = dist,
                        checksum = 0,
                        deviceId = devId,
                        stateFlags = flags,
                        closingSpeedCms = speed,
                        ttcMs = ttc,
                        isAdvanced = true,
                        receivedAt = timestamp,
                    )
                    packet.takeIf { it.isValid() }
                }
                data.size == 6 || data.size == 12 || data.size == 24 -> {
                    // The four-bit count is already in the six-byte status.
                    // V2 adds only two 16-bit epochs and packed receipt/rollover flags.
                    // Byte 0: [Sync Nibble 0xA (4b) | Pair ID (4b)]
                    // Byte 1: [Node Type (1b) | Heartbeat Seq (3b) | Rolling Fall Seq (4b)]
                    // Byte 2: [State Flags (8b)]
                    // Byte 3: [Distance Low (8b)]        Units: 2mm
                    // Byte 4: [Distance High (4b) | TTC Low (4b)]
                    // Byte 5: [TTC High (8b)]            Units: 20ms
                    val preamblePairId = data[0].toInt() and 0xFF
                    val syncNibble = (preamblePairId shr 4) and 0x0F
                    val pairId = preamblePairId and 0x0F

                    // Validate sync nibble
                    if (syncNibble != 0x0A) return null

                    val typeSeqFall = data[1].toInt() and 0xFF
                    val nodeType = (typeSeqFall shr 7) and 0x01      // 0=Glasses, 1=Cane
                    val heartbeatSeq = (typeSeqFall shr 4) and 0x07   // 3-bit
                    val rollingFallSeq = typeSeqFall and 0x0F         // 4-bit

                    val stateFlags = data[2].toInt() and 0xFF

                    val distLow = data[3].toInt() and 0xFF
                    val distHighTtcLow = data[4].toInt() and 0xFF
                    val ttcHigh = data[5].toInt() and 0xFF

                    // Decode 12-bit distance (2mm steps, 0-8190mm, 0xFFF=no target)
                    val distRaw = distLow or (((distHighTtcLow shr 4) and 0x0F) shl 8)
                    val nearestDistMm = if (distRaw >= 0x0FFF) -1 else distRaw * 2

                    // Decode 12-bit TTC (20ms steps, 0-81900ms, 0xFFF=no threat)
                    val ttcRaw = (distHighTtcLow and 0x0F) or (ttcHigh shl 4)
                    val ttcMsVal = if (ttcRaw >= 0x0FFF) -1 else ttcRaw * 20

                    val deviceId = if (nodeType == 1) 0x01 else 0x02

                    val compactReceipt = data.size == 12
                    val oldExtension = data.size == 24
                    if (compactReceipt && (data[6].toInt() and 0xFF) != SAFETY_EXTENSION_MARKER) return null
                    if (oldExtension && (data[6].toInt() and 0xFF) != LEGACY_SAFETY_EXTENSION_MARKER) return null
                    val protocolVersion = when {
                        compactReceipt -> PROTOCOL_VERSION
                        oldExtension -> LEGACY_PROTOCOL_VERSION
                        else -> 0
                    }
                    val sessionId = when {
                        compactReceipt -> readU16Le(data, 7)
                        oldExtension -> readU32Le(data, 7)
                        else -> 0L
                    }
                    val eventId = if (oldExtension) readU32Le(data, 11) else rollingFallSeq.toLong()
                    val peerAckSessionId = when {
                        compactReceipt -> readU16Le(data, 9)
                        oldExtension -> readU32Le(data, 15)
                        else -> 0L
                    }
                    val safetyFlags = when {
                        compactReceipt -> data[11].toInt() and 0xFF
                        oldExtension -> data[23].toInt() and 0xFF
                        else -> 0
                    }
                    if (compactReceipt && (safetyFlags and SAFETY_RESERVED_MASK) != 0) return null
                    val peerAckEventId = when {
                        compactReceipt -> (safetyFlags and 0x0F).toLong()
                        oldExtension -> readU32Le(data, 19)
                        else -> 0L
                    }

                    val packet = SystemPacket(
                        magic = 0xAA, // Synthetic preamble for UI compatibility
                        seqNum = heartbeatSeq,
                        radarDistMm = nearestDistMm,
                        checksum = 0,
                        deviceId = deviceId,
                        stateFlags = stateFlags,
                        closingSpeedCms = 0,
                        ttcMs = ttcMsVal,
                        isAdvanced = true,
                        compactFormat = true,
                        rollingFallSeq = rollingFallSeq,
                        pairId = pairId,
                        protocolVersion = protocolVersion,
                        sessionId = sessionId,
                        eventId = eventId,
                        sessionRollover = compactReceipt && (safetyFlags and SAFETY_FLAG_ROLLOVER) != 0,
                        peerAckValid = (safetyFlags and if (oldExtension) 0x01 else SAFETY_FLAG_PEER_ACK) != 0,
                        peerAckSessionId = peerAckSessionId,
                        peerAckEventId = peerAckEventId,
                        receivedAt = timestamp,
                    )
                    packet.takeIf { it.isValid() }
                }
                data.size == 5 -> {
                    val magic = data[0].toInt() and 0xFF
                    val seq = data[1].toInt() and 0xFF
                    val rawDist = (data[2].toInt() and 0xFF) or ((data[3].toInt() and 0xFF) shl 8)
                    val dist = if (rawDist == 0xFFFF) -1 else rawDist
                    val chk = data[4].toInt() and 0xFF

                    val packet = SystemPacket(
                        magic = magic,
                        seqNum = seq,
                        radarDistMm = dist,
                        checksum = chk,
                        isAdvanced = false,
                        receivedAt = timestamp,
                    )
                    packet.takeIf { it.isValid() }
                }
                else -> null
            }
        }
    }
}
