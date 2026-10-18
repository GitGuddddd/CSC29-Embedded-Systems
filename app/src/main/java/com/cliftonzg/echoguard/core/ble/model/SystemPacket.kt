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
 * Supports 5-byte basic packet schema, 10-byte, and 11-byte AdvancedThreatPacket schema.
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
    val receivedAt: Long = System.currentTimeMillis(),
) {
    val isFallAlert: Boolean get() = (stateFlags and FLAG_FALL_ALERT) != 0
    val isRadarFault: Boolean get() = (stateFlags and FLAG_RADAR_FAULT) != 0
    val isObstacleActive: Boolean get() = (stateFlags and FLAG_OBSTACLE_ACTIVE) != 0

    val threatLevel: ThreatLevel
        get() = when {
            isFallAlert || isRadarFault -> ThreatLevel.CRITICAL
            (radarDistMm in 1..799) -> ThreatLevel.CRITICAL
            (radarDistMm in 800..1500) -> ThreatLevel.WARNING
            radarDistMm > 1500 -> ThreatLevel.SAFE
            else -> ThreatLevel.UNKNOWN
        }

    fun isValid(): Boolean {
        // Strict EchoGuard System Preamble: Must be 0xAA
        if (magic != 0xAA) return false

        if (isAdvanced) {
            return (deviceId == 0x01 || deviceId == 0x02)
        }
        val distLow = radarDistMm and 0xFF
        val distHigh = (radarDistMm shr 8) and 0xFF
        val calculated = (magic xor seqNum xor distLow xor distHigh) and 0xFF
        return calculated == (checksum and 0xFF)
    }

    /**
     * Monotonic sequence number check per FR15.
     * Rejects stale or out-of-order sequence numbers (accounting for wrap-around).
     */
    fun isNewerThan(lastSeqNum: Int): Boolean {
        if (lastSeqNum < 0) return true
        if (seqNum == lastSeqNum) return false
        val maxSeq = if (isAdvanced) 65536 else 256
        val diff = ((seqNum - lastSeqNum) + maxSeq) % maxSeq
        return diff in 1 until (maxSeq - 1)
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
