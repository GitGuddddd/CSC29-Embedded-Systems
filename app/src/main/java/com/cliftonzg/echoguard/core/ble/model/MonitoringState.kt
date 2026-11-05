package com.cliftonzg.echoguard.core.ble.model

data class NodeTelemetry(
    val type: NodeType,
    val address: String? = null,
    val packet: SystemPacket? = null,
    val linkState: LinkState = LinkState.WAITING,
) {
    val isFresh: Boolean get() = linkState == LinkState.ACTIVE
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
}
