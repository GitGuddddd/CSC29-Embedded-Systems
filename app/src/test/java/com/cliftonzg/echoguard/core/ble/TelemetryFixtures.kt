package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.SystemPacket

internal val caneDevice = DiscoveredBleDevice(
    "Test Cane", "AA:00:00:00:00:01", -50, true, nodeType = NodeType.CANE, pairId = 1,
)
internal const val GLASSES_ADDRESS = "AA:00:00:00:00:02"

internal fun packet(
    type: NodeType = NodeType.CANE,
    seq: Int = 1,
    at: Long = 0L,
    flags: Int = SystemPacket.FLAG_CANE_HELD,
    fallSeq: Int = 0,
    pair: Int = 1,
    distance: Int = 1000,
    session: Long = 0x1001L,
    event: Long = fallSeq.toLong(),
    ackSession: Long = 0L,
    ackEvent: Long = 0L,
    ackValid: Boolean = false,
) = SystemPacket(
    magic = 0xAA, seqNum = seq, radarDistMm = distance, checksum = 0,
    deviceId = type.deviceId, stateFlags = flags, isAdvanced = true,
    compactFormat = true, rollingFallSeq = fallSeq, pairId = pair,
    protocolVersion = SystemPacket.PROTOCOL_VERSION, sessionId = session, eventId = event,
    peerAckValid = ackValid, peerAckSessionId = ackSession, peerAckEventId = ackEvent,
    receivedAt = at,
)

internal fun state(vararg nodes: NodeTelemetry, scanning: Boolean = true) =
    MonitoringState(caneDevice, scanning, nodes.toList())

internal fun node(packet: SystemPacket, link: LinkState = LinkState.ACTIVE) = NodeTelemetry(
    type = packet.nodeType,
    address = if (packet.nodeType == NodeType.CANE) caneDevice.address else GLASSES_ADDRESS,
    packet = packet,
    linkState = link,
)
