package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.SystemPacket

/** Phone-owned cumulative statistics; never used to acknowledge or clear a wearable alarm. */
data class FallHistoryRecord(
    val total: Long,
    val epoch: Int,
    val sequence: Int,
    val hasGaps: Boolean,
) {
    fun encode(): String = "$total,$epoch,$sequence,${if (hasGaps) 1 else 0}"

    companion object {
        fun decode(value: String?): FallHistoryRecord? {
            val fields = value?.split(',') ?: return null
            if (fields.size != 4) return null
            val total = fields[0].toLongOrNull()?.takeIf { it >= 0L } ?: return null
            val epoch = fields[1].toIntOrNull()?.takeIf { it in 1..0xFFFF } ?: return null
            val sequence = fields[2].toIntOrNull()?.takeIf { it in 0..15 } ?: return null
            val gaps = fields[3].toIntOrNull()?.takeIf { it in 0..1 } ?: return null
            return FallHistoryRecord(total, epoch, sequence, gaps == 1)
        }
    }
}

interface FallHistoryStore {
    fun load(key: String): FallHistoryRecord?
    fun save(key: String, record: FallHistoryRecord)
}

class MemoryFallHistoryStore : FallHistoryStore {
    private val records = mutableMapOf<String, FallHistoryRecord>()
    override fun load(key: String): FallHistoryRecord? = records[key]
    override fun save(key: String, record: FallHistoryRecord) { records[key] = record }
}

/** Access is serialized by TelemetrySession/BleScannerEngine, including persistent writes. */
class PhoneFallHistory(private val store: FallHistoryStore = MemoryFallHistoryStore()) {
    private val records = mutableMapOf<String, FallHistoryRecord>()

    fun observe(key: String, packet: SystemPacket): FallHistoryRecord? {
        if (!packet.compactFormat || packet.protocolVersion != SystemPacket.PROTOCOL_VERSION ||
            !packet.isValid()
        ) return null
        val previous = records[key] ?: store.load(key)
        val epoch = packet.sessionId.toInt()
        val sequence = packet.rollingFallSeq
        val next = if (previous == null) {
            // First contact is a baseline, not proof of all events before phone monitoring.
            FallHistoryRecord(0L, epoch, sequence,
                sequence != 0 || packet.sessionRollover || packet.hasLatchedFall)
        } else if (epoch == previous.epoch && sequence < previous.sequence) {
            // A stale first callback after process recreation must not move the saved
            // baseline backwards and cause the next genuine packet to be counted twice.
            previous.copy(hasGaps = true)
        } else {
            val nextEpoch = if (previous.epoch == 0xFFFF) 1 else previous.epoch + 1
            val delta = when {
                epoch == previous.epoch && sequence >= previous.sequence -> sequence - previous.sequence
                packet.sessionRollover && epoch == nextEpoch -> 16 - previous.sequence + sequence
                else -> null // Reboot, multiple missed epochs or an invalid backward counter.
            }
            val increment = delta?.toLong() ?: 0L
            val overflow = previous.total > Long.MAX_VALUE - increment
            FallHistoryRecord(
                total = if (overflow) Long.MAX_VALUE else previous.total + increment,
                epoch = epoch,
                sequence = sequence,
                hasGaps = previous.hasGaps || delta == null || overflow,
            )
        }
        records[key] = next
        if (next != previous) store.save(key, next) // Never write on every heartbeat.
        return next
    }
}
