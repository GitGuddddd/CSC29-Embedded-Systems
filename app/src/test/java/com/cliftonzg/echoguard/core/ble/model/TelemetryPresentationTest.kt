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

    @Test fun `healthy cane reports cane as primary audio buzzer for pair`() {
        val cane = report(NodeType.CANE, 0x2001L, 0L)
        val state = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(
                NodeTelemetry(NodeType.CANE, "C", cane, LinkState.ACTIVE),
                NodeTelemetry(NodeType.GLASSES, "G", report(NodeType.GLASSES, 0x1234L, 0L).copy(stateFlags = 0), LinkState.ACTIVE),
            ),
        )
        val routing = state.audioRoutingPresentation()
        assertEquals("Primary audio: Cane", routing.title)
        assertEquals("Cane is primary audio buzzer for the pair.", routing.detail)
        assertEquals(StatusTone.OK, routing.tone)
    }

    @Test fun `cane buzzer fault surfaces audio switched to glasses failover messaging`() {
        val cane = report(NodeType.CANE, 0x2001L, 0L).copy(stateFlags = SystemPacket.FLAG_BUZZER_FAULT)
        val caneNode = NodeTelemetry(NodeType.CANE, "C", cane, LinkState.ACTIVE)
        val glassesNode = NodeTelemetry(NodeType.GLASSES, "G", report(NodeType.GLASSES, 0x1234L, 0L).copy(stateFlags = 0), LinkState.ACTIVE)
        val state = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(caneNode, glassesNode),
        )
        val routing = state.audioRoutingPresentation()
        assertEquals("Buzzer failover", routing.title)
        assertEquals("Cane buzzer fault · Audio switched to glasses", routing.detail)
        assertEquals(StatusTone.CAUTION, routing.tone)

        val nodePres = caneNode.presentation()
        assertEquals("Buzzer fault", nodePres.title)
        assertEquals("Cane buzzer fault · Audio switched to glasses", nodePres.detail)
        assertEquals(StatusTone.CAUTION, nodePres.tone)

        val summary = state.presentation()
        assertEquals("Buzzer fault", summary.title)
        assertEquals("Cane buzzer fault · Audio switched to glasses", summary.detail)
        assertEquals(StatusTone.CAUTION, summary.tone)
    }

    @Test fun `cane buzzer fault during fall alert includes failover warning`() {
        val glasses = report(NodeType.GLASSES, 0x1234L, 1L)
        val caneWithFault = report(NodeType.CANE, 0x2001L, 0L, 0x1234L, 1L, true)
            .copy(stateFlags = SystemPacket.FLAG_BUZZER_FAULT)
        val state = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(
                NodeTelemetry(NodeType.CANE, "C", caneWithFault, LinkState.ACTIVE),
                NodeTelemetry(NodeType.GLASSES, "G", glasses, LinkState.ACTIVE),
            ),
        )
        val summary = state.presentation()
        assertEquals("Fall alarm", summary.title)
        assertTrue(summary.detail.contains("Cane buzzer fault · Audio switched to glasses"))
    }

    @Test fun `offline cane surfaces fallback audio routing`() {
        val state = MonitoringState(
            selectedDevice = DiscoveredBleDevice("Cane", "C", -50, true, nodeType = NodeType.CANE, pairId = 1),
            isScanning = true,
            nodes = listOf(
                NodeTelemetry(NodeType.GLASSES, "G", report(NodeType.GLASSES, 0x1234L, 0L), LinkState.ACTIVE),
            ),
        )
        val routing = state.audioRoutingPresentation()
        assertEquals("Audio output: Glasses", routing.title)
        assertEquals("Cane offline; glasses buzzer fallback active.", routing.detail)
        assertEquals(StatusTone.NEUTRAL, routing.tone)
    }
}
