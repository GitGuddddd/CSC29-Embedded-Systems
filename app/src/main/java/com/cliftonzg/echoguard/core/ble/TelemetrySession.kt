package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.SystemPacket

/** Pure session/sequence logic. The scanner serializes access to this object. */
class TelemetrySession {
    private var selected: DiscoveredBleDevice? = null
    private var selectedAt = 0L
    private var scanStartedAt = 0L
    private var scanning = false
    private val entries = mutableMapOf<NodeType, NodeTelemetry>()

    fun select(device: DiscoveredBleDevice?, nowMs: Long) {
        selected = device
        selectedAt = nowMs
        entries.clear()
    }

    fun setScanning(enabled: Boolean, nowMs: Long) {
        if (enabled && !scanning) scanStartedAt = nowMs
        scanning = enabled
    }

    fun receive(address: String, packet: SystemPacket, nowMs: Long): Boolean {
        val device = selected ?: return false // Discovery does not silently start monitoring a stranger.
        if (!scanning || !packet.isValid() || packet.receivedAt < maxOf(selectedAt, scanStartedAt)) return false
        val age = nowMs - packet.receivedAt
        if (age < 0L || age >= SystemPacket.LINK_TIMEOUT_MS) return false
        if (address == device.address && device.nodeType != null && packet.nodeType != device.nodeType) return false

        if (device.pairId != null) {
            if (!packet.compactFormat || packet.pairId != device.pairId) return false
            if (address != device.address && packet.nodeType == device.nodeType) return false
        } else if (address != device.address) {
            return false // Legacy data has no pair ID: scope it to the explicitly selected address.
        }

        val previous = entries[packet.nodeType]
        if (previous?.address != null && previous.address != address) return false
        if (!packet.isFreshAfter(previous?.packet)) return false
        if (address == device.address && device.nodeType == null) {
            selected = device.copy(nodeType = packet.nodeType, pairId = packet.pairId.takeIf { packet.compactFormat })
        }
        entries[packet.nodeType] = NodeTelemetry(packet.nodeType, address, packet, LinkState.ACTIVE)
        return true
    }

    fun snapshot(nowMs: Long): MonitoringState {
        val device = selected
        if (device == null) return MonitoringState(isScanning = scanning)
        val types = if (device.pairId != null) listOf(NodeType.CANE, NodeType.GLASSES)
        else listOf(device.nodeType ?: NodeType.LEGACY)
        val nodes = types.map { type ->
            val entry = entries[type] ?: NodeTelemetry(
                type = type,
                address = device.address.takeIf { device.nodeType == type },
            )
            val link = when {
                !scanning -> LinkState.DISCONNECTED
                entry.packet == null -> LinkState.WAITING
                nowMs - entry.packet.receivedAt >= SystemPacket.LINK_TIMEOUT_MS -> LinkState.LINK_LOSS
                else -> LinkState.ACTIVE
            }
            entry.copy(linkState = link)
        }
        return MonitoringState(device, scanning, nodes, observedAt = nowMs)
    }
}
