package com.cliftonzg.echoguard.core.ble.model

import android.util.Log

enum class ThreatLevel {
    CRITICAL, // < 800 mm or Sensor Fault / Fall Alert
    WARNING,  // 800 mm .. 1500 mm
    SAFE,     // > 1500 mm
    UNKNOWN
}

/**
 * Represents a packet received over BLE from the EchoGuard system.
 * Supports 5-byte basic packet schema, 10-11 byte AdvancedThreatPacket schema,
 * and the compact 6-byte packed schema (minimum-bit curveball response).
 */
data class SystemPacket(
    val magic: Int,
    val seqNum: Int,
    val radarDistMm: Int,
    val checksum: Int,
    val deviceId: Int = 0,
    val stateFlags: Int = 0,
    val closingSpeedCms: Int = 0,
    val ttcMs: Int = 0,
    val isAdvanced: Boolean = false,
    val compactFormat: Boolean = false,
    val rollingFallSeq: Int = 0,
    val pairId: Int = 0,
    val receivedAt: Long = System.currentTimeMillis(),
) {
    val isFallAlert: Boolean get() = (stateFlags and FLAG_FALL_ALERT) != 0
    val isRadarFault: Boolean get() = (stateFlags and FLAG_RADAR_FAULT) != 0
    val isObstacleActive: Boolean get() = (stateFlags and FLAG_OBSTACLE_ACTIVE) != 0

    val threatLevel: ThreatLevel
        get() = when {
            isFallAlert || isRadarFault -> ThreatLevel.CRITICAL
            deviceId == 0x01 -> {
                // Cane ToF (Downward ground sensor):
                // Small distances (1..1500 mm) = Solid ground under cane -> SAFE (Green)
                // Large distances (> 1500 mm or -1 / 0xFFFF) = Cliff / hole / drop-off -> CRITICAL (Red)
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

    private fun isCompactValid(): Boolean {
        // Compact 6-byte packed schema: deviceId derived from node type (0x01=Cane, 0x02=Glasses)
        return (deviceId == 0x01 || deviceId == 0x02) && pairId in 0 until 16
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

    companion object {
        const val FLAG_FALL_ALERT = 0x01
        const val FLAG_RADAR_FAULT = 0x02
        const val FLAG_OBSTACLE_ACTIVE = 0x04

        fun parse(data: ByteArray, timestamp: Long = System.currentTimeMillis()): SystemPacket? {
            return when {
                data.size >= 10 -> {
                    // AdvancedThreatPacket Schema (10 to 11 bytes):
                    // [Preamble(1B), DeviceID(1B), SeqNum(2B), StateFlags(1B), TargetDist(2B), TargetSpeed(2B), TTC(2B)]
                    val preamble = data[0].toInt() and 0xFF
                    val devId = data[1].toInt() and 0xFF
                    val seq = (data[2].toInt() and 0xFF) or ((data[3].toInt() and 0xFF) shl 8)
                    val flags = data[4].toInt() and 0xFF
                    val dist = (data[5].toInt() and 0xFF) or ((data[6].toInt() and 0xFF) shl 8)
                    val speed = (data[7].toInt() and 0xFF) or (data[8].toInt() shl 8)
                    val ttc = if (data.size >= 11) (data[9].toInt() and 0xFF) or ((data[10].toInt() and 0xFF) shl 8) else 0

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
                    if (packet.isValid()) {
                        Log.i("EchoGuardBLE", "Parsed VALID Advanced Packet: devId=0x%02X, seq=$seq, dist=$dist mm, flags=0x%02X".format(devId, flags))
                        packet
                    } else {
                        null
                    }
                }
                data.size == 6 -> {
                    // Compact 6-byte Packed Schema (minimum-bit curveball response):
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
                        receivedAt = timestamp,
                    )
                    if (packet.isValid()) {
                        Log.i("EchoGuardBLE", "Parsed VALID Compact Packet: devId=0x%02X, hbSeq=$heartbeatSeq, fallSeq=$rollingFallSeq, dist=$nearestDistMm mm, ttc=$ttcMsVal ms, flags=0x%02X".format(deviceId, stateFlags))
                        packet
                    } else {
                        null
                    }
                }
                data.size >= 5 -> {
                    val magic = data[0].toInt() and 0xFF
                    val seq = data[1].toInt() and 0xFF
                    val dist = (data[2].toInt() and 0xFF) or ((data[3].toInt() and 0xFF) shl 8)
                    val chk = data[4].toInt() and 0xFF

                    val packet = SystemPacket(
                        magic = magic,
                        seqNum = seq,
                        radarDistMm = dist,
                        checksum = chk,
                        isAdvanced = false,
                        receivedAt = timestamp,
                    )
                    if (packet.isValid()) {
                        Log.i("EchoGuardBLE", "Parsed VALID 5B Packet: magic=0x%02X, seq=$seq, dist=$dist mm".format(magic))
                        packet
                    } else {
                        null
                    }
                }
                else -> null
            }
        }
    }
}
