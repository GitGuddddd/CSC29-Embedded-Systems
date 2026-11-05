package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.TelemetryStatus

enum class HapticCue(val priority: Int, val durationMs: Long, val reminderMs: Long) {
    STOP(0, 0L, Long.MAX_VALUE),
    WARNING(1, 120L, 3_000L),
    SENSOR_FAULT(2, 500L, 10_000L),
    LINK_LOSS(3, 650L, 10_000L),
    CRITICAL(4, 250L, 2_000L),
    FALL(5, 900L, 5_000L),
}

/** Single phone-vibrator owner. Heartbeat updates do not restart an in-progress waveform. */
class SafetyHapticPolicy {
    private data class Signal(val cue: HapticCue, val incidentSeq: Int = 0)

    private var selection: Pair<String, Int?>? = null
    private var previous = emptyMap<NodeType, Signal>()
    private var current = HapticCue.STOP
    private var lastEmittedMs = 0L

    fun update(state: MonitoringState, nowMs: Long): HapticCue? {
        val newSelection = state.selectedDevice?.let { it.address to it.pairId }
        if (newSelection != selection) {
            selection = newSelection
            previous = emptyMap()
        }
        if (state.selectedDevice == null || !state.isScanning) {
            previous = emptyMap()
            return stop()
        }
        val signals = state.nodes.associate { it.type to signal(it) }
        val winner = signals.values.maxByOrNull { it.cue.priority }?.cue ?: HapticCue.STOP
        val newIncident = signals.any { (node, signal) -> signal.cue == winner && previous[node] != signal }
        previous = signals
        if (winner == HapticCue.STOP) return stop()
        val elapsed = nowMs - lastEmittedMs
        val shouldEmit = winner != current || elapsed >= winner.reminderMs ||
            (newIncident && elapsed >= winner.durationMs)
        if (!shouldEmit) return null
        current = winner
        lastEmittedMs = nowMs
        return winner
    }

    private fun stop(): HapticCue? {
        if (current == HapticCue.STOP) return null
        current = HapticCue.STOP
        return HapticCue.STOP
    }

    private fun signal(node: NodeTelemetry): Signal {
        if (node.linkState == LinkState.LINK_LOSS) return Signal(HapticCue.LINK_LOSS)
        val packet = node.packet ?: return Signal(HapticCue.STOP)
        if (!node.isFresh) return Signal(HapticCue.STOP)
        return when (packet.status) {
            TelemetryStatus.FALL_ALARM -> Signal(HapticCue.FALL, packet.rollingFallSeq)
            TelemetryStatus.GROUND_HAZARD, TelemetryStatus.OBSTACLE_CRITICAL -> Signal(HapticCue.CRITICAL)
            TelemetryStatus.OBSTACLE_WARNING -> Signal(HapticCue.WARNING)
            TelemetryStatus.SENSOR_FAULT -> Signal(HapticCue.SENSOR_FAULT)
            TelemetryStatus.FALL_PAUSED, TelemetryStatus.STANDBY, TelemetryStatus.GRIP_RELEASED,
            TelemetryStatus.MONITORING, TelemetryStatus.NO_READING -> Signal(HapticCue.STOP)
        }
    }
}
