package com.cliftonzg.echoguard.core.ble.model

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TelemetryPresentationTest {
    private fun report(
        type: NodeType,
        session: Long,
        event: Long,
        ackSession: Long = 0L,
        ackEvent: Long = 0L,
        ackValid: Boolean = false,
    ) = SystemPacket(
        magic = 0xAA,
        seqNum = 1,
        radarDistMm = -1,
        checksum = 0,
        deviceId = type.deviceId,
        stateFlags = if (type == NodeType.GLASSES) SystemPacket.FLAG_FALL_ALERT else 0,
        isAdvanced = true,
        compactFormat = true,
        rollingFallSeq = (event and 0x0FL).toInt(),
        pairId = 1,
        protocolVersion = SystemPacket.PROTOCOL_VERSION,
        sessionId = session,
        eventId = event,
        peerAckValid = ackValid,
        peerAckSessionId = ackSession,
        peerAckEventId = ackEvent,
    )

    @Test fun `only matching fresh cane application receipt confirms glasses event`() {
        val glasses = report(NodeType.GLASSES, 0x1234L, 1L)
        val wrong = report(NodeType.CANE, 0x2001L, 0L, 0x1234L, 0L, true)
        fun state(cane: SystemPacket) = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(
                NodeTelemetry(NodeType.CANE, "C", cane, LinkState.ACTIVE),
                NodeTelemetry(NodeType.GLASSES, "G", glasses, LinkState.ACTIVE),
            ),
        )
        assertEquals(PeerReceiptState.WAITING, state(wrong).glassesFallReceiptState)
        val confirmed = state(wrong.copy(peerAckEventId = 1L))
        assertEquals(PeerReceiptState.CONFIRMED, confirmed.glassesFallReceiptState)
        assertTrue(confirmed.presentation().detail.contains("receipt confirmed"))
    }

    @Test fun `stale cane cannot claim a current receipt`() {
        val glasses = report(NodeType.GLASSES, 0x1234L, 2L)
        val cane = report(NodeType.CANE, 0x2001L, 0L, 0x1234L, 2L, true)
        val state = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(
                NodeTelemetry(NodeType.CANE, "C", cane, LinkState.LINK_LOSS),
                NodeTelemetry(NodeType.GLASSES, "G", glasses, LinkState.ACTIVE),
            ),
        )
        assertEquals(PeerReceiptState.CANE_UNAVAILABLE, state.glassesFallReceiptState)
        assertTrue(state.presentation().detail.contains("fallback should be active"))
    }

    @Test fun `legacy cane status cannot claim application receipt support`() {
        val glasses = report(NodeType.GLASSES, 0x1234L, 2L)
        val legacyCane = report(NodeType.CANE, 0x2001L, 0L).copy(
            protocolVersion = 0,
            sessionId = 0L,
        )
        val state = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(
                NodeTelemetry(NodeType.CANE, "C", legacyCane, LinkState.ACTIVE),
                NodeTelemetry(NodeType.GLASSES, "G", glasses, LinkState.ACTIVE),
            ),
        )
        assertEquals(PeerReceiptState.LEGACY_UNAVAILABLE, state.glassesFallReceiptState)
    }
}
