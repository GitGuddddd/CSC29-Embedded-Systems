package com.cliftonzg.echoguard.core.ble.model

data class NodeTelemetry(
    val type: NodeType,
    val address: String? = null,
    val packet: SystemPacket? = null,
    val linkState: LinkState = LinkState.WAITING,
) {
    val isFresh: Boolean get() = linkState == LinkState.ACTIVE
}

enum class PeerReceiptState {
    NOT_APPLICABLE,
    WAITING,
    CONFIRMED,
    CANE_UNAVAILABLE,
    LEGACY_UNAVAILABLE,
}

/** One coherent snapshot is shared by the dashboard and the foreground alert service. */
data class MonitoringState(
    val selectedDevice: DiscoveredBleDevice? = null,
    val isScanning: Boolean = false,
    val nodes: List<NodeTelemetry> = emptyList(),
    val observedAt: Long = 0L,
) {
    val selectedLinkState: LinkState
        get() = when {
            selectedDevice == null || !isScanning -> LinkState.DISCONNECTED
            else -> nodes.firstOrNull { it.address == selectedDevice.address }?.linkState ?: LinkState.WAITING
        }

    val latestPacket: SystemPacket?
        get() = nodes.mapNotNull { it.packet }.maxByOrNull { it.receivedAt }

    fun node(type: NodeType): NodeTelemetry? = nodes.firstOrNull { it.type == type }

    val glassesFallReceiptState: PeerReceiptState
        get() {
            val glasses = node(NodeType.GLASSES)
            val event = glasses?.packet
            if (glasses?.isFresh != true || event?.hasLatchedFall != true) {
                return PeerReceiptState.NOT_APPLICABLE
            }
            if (event.protocolVersion != SystemPacket.PROTOCOL_VERSION || event.sessionId == 0L) {
                return PeerReceiptState.LEGACY_UNAVAILABLE
            }
            val cane = node(NodeType.CANE)
            val receipt = cane?.packet
            if (cane?.isFresh != true || receipt == null) return PeerReceiptState.CANE_UNAVAILABLE
            if (receipt.protocolVersion != SystemPacket.PROTOCOL_VERSION || receipt.sessionId == 0L) {
                return PeerReceiptState.LEGACY_UNAVAILABLE
            }
            return if (receipt.peerAckValid && receipt.peerAckSessionId == event.sessionId &&
                receipt.peerAckEventId == event.eventId
            ) PeerReceiptState.CONFIRMED else PeerReceiptState.WAITING
        }
}
