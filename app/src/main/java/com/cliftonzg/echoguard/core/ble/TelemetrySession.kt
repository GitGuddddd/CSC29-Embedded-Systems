package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.SystemPacket

enum class ReceiveRejectReason {
    NONE,
    NO_SELECTION,
    PRE_SELECTION,
    EXPIRED_PACKET,
    WRONG_PAIR,
    UNSELECTED_ADDRESS,
    ROLE_CONFLICT,
    RETIRED_EPOCH,
    STALE_OR_DUPLICATE,
}

/** Pure session/sequence logic. The scanner serializes access to this object. */
class TelemetrySession(private val history: PhoneFallHistory = PhoneFallHistory()) {
    private var selected: DiscoveredBleDevice? = null
    private var selectedAt = 0L
    private var scanStartedAt = 0L
    private var scanning = false
    private val entries = mutableMapOf<NodeType, NodeTelemetry>()
    private val retiredEpochs = mutableMapOf<NodeType, Long>()

    fun select(device: DiscoveredBleDevice?, nowMs: Long) {
        selected = device
        selectedAt = nowMs
        entries.clear()
        retiredEpochs.clear()
    }

    fun setScanning(enabled: Boolean, nowMs: Long) {
        if (enabled && !scanning) scanStartedAt = nowMs
        scanning = enabled
    }

    fun receive(address: String, packet: SystemPacket, nowMs: Long): Boolean =
        receiveWithReason(address, packet, nowMs) == ReceiveRejectReason.NONE

    fun receiveWithReason(address: String, packet: SystemPacket, nowMs: Long): ReceiveRejectReason {
        val device = selected ?: return ReceiveRejectReason.NO_SELECTION // Discovery does not silently start monitoring a stranger.
        if (!scanning || !packet.isValid() || packet.receivedAt < maxOf(selectedAt, scanStartedAt)) {
            return ReceiveRejectReason.PRE_SELECTION
        }
        val age = nowMs - packet.receivedAt
        if (age < 0L || age >= SystemPacket.LINK_TIMEOUT_MS) return ReceiveRejectReason.EXPIRED_PACKET
        if (address == device.address && device.nodeType != null && packet.nodeType != device.nodeType) {
            return ReceiveRejectReason.ROLE_CONFLICT
        }

        if (device.pairId != null) {
            if (!packet.compactFormat || packet.pairId != device.pairId) return ReceiveRejectReason.WRONG_PAIR
            if (address != device.address && packet.nodeType == device.nodeType) return ReceiveRejectReason.ROLE_CONFLICT
        } else if (address != device.address) {
            return ReceiveRejectReason.UNSELECTED_ADDRESS // Legacy data has no pair ID: scope it to the explicitly selected address.
        }

        val previous = entries[packet.nodeType]
        if (previous?.address != null && previous.address != address) return ReceiveRejectReason.ROLE_CONFLICT
        val changedEpoch = packet.protocolVersion == SystemPacket.PROTOCOL_VERSION &&
            previous?.packet?.sessionId != packet.sessionId
        if (changedEpoch && retiredEpochs[packet.nodeType] == packet.sessionId) return ReceiveRejectReason.RETIRED_EPOCH
        if (!packet.isFreshAfter(previous?.packet)) return ReceiveRejectReason.STALE_OR_DUPLICATE
        if (address == device.address && device.nodeType == null) {
            selected = device.copy(nodeType = packet.nodeType, pairId = packet.pairId.takeIf { packet.compactFormat })
        }
        if (changedEpoch) previous?.packet?.sessionId?.let { retiredEpochs[packet.nodeType] = it }
        val scope = selected ?: device
        val key = "${scope.pairId}:${address}:${packet.nodeType}"
        val record = history.observe(key, packet)
        entries[packet.nodeType] = NodeTelemetry(
            packet.nodeType, address, packet, LinkState.ACTIVE,
            phoneFallCount = record?.total, phoneHistoryHasGaps = record?.hasGaps == true,
        )
        return ReceiveRejectReason.NONE
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
