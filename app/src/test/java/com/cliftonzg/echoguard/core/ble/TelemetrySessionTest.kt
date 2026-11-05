package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.StatusTone
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import com.cliftonzg.echoguard.core.ble.model.presentation
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class TelemetrySessionTest {
    private lateinit var session: TelemetrySession

    @Before fun setup() {
        session = TelemetrySession()
        session.setScanning(true, 0L)
        session.select(caneDevice, 0L)
    }

    @Test fun `selection waits for telemetry and discovery alone does not monitor`() {
        assertEquals(LinkState.WAITING, session.snapshot(0L).selectedLinkState)
        session.select(null, 0L)
        assertFalse(session.receive(caneDevice.address, packet(), 0L))
        assertTrue(session.snapshot(0L).nodes.isEmpty())
    }

    @Test fun `each node owns a separate heartbeat sequence and timeout`() {
        assertTrue(session.receive(caneDevice.address, packet(seq = 6), 0L))
        assertTrue(session.receive(GLASSES_ADDRESS, packet(NodeType.GLASSES, seq = 6, at = 490L), 490L))
        val state = session.snapshot(500L)
        assertEquals(LinkState.LINK_LOSS, state.node(NodeType.CANE)?.linkState)
        assertEquals(LinkState.ACTIVE, state.node(NodeType.GLASSES)?.linkState)
        assertEquals(LinkState.LINK_LOSS, state.selectedLinkState)
        assertNotEquals(StatusTone.OK, state.presentation().tone)
    }

    @Test fun `wrong pair and same-role neighbours cannot replace monitored nodes`() {
        assertFalse(session.receive(GLASSES_ADDRESS, packet(NodeType.GLASSES, pair = 2), 0L))
        assertFalse(session.receive("OTHER-CANE", packet(), 0L))
        assertFalse(session.receive(caneDevice.address, packet(pair = 2), 0L))
        assertTrue(session.receive(GLASSES_ADDRESS, packet(NodeType.GLASSES), 0L))
        assertFalse(session.receive("OTHER-GLASSES", packet(NodeType.GLASSES, seq = 2, at = 100L), 100L))
        assertEquals(GLASSES_ADDRESS, session.snapshot(100L).node(NodeType.GLASSES)?.address)
    }

    @Test fun `identical repeats never extend heartbeat lifetime even after a long gap`() {
        assertTrue(session.receive(caneDevice.address, packet(at = 0L), 0L))
        assertFalse(session.receive(caneDevice.address, packet(at = 300L), 300L))
        assertFalse(session.receive(caneDevice.address, packet(at = 800L), 800L))
        assertEquals(LinkState.ACTIVE, session.snapshot(499L).selectedLinkState)
        assertEquals(LinkState.LINK_LOSS, session.snapshot(500L).selectedLinkState)
    }

    @Test fun `compact counters rebase after gap and same-sequence state changes survive wrap`() {
        assertTrue(session.receive(caneDevice.address, packet(seq = 1), 0L))
        assertFalse(session.receive(caneDevice.address, packet(seq = 7, at = 100L), 100L))
        assertTrue(session.receive(caneDevice.address, packet(seq = 7, at = 250L), 250L))
        val pause = packet(seq = 7, at = 300L, flags = SystemPacket.FLAG_FALL_ALERT or SystemPacket.FLAG_ALARM_PAUSED)
        assertTrue(session.receive(caneDevice.address, pause, 300L))
        assertTrue(session.snapshot(300L).node(NodeType.CANE)?.packet?.isAlarmPaused == true)
    }

    @Test fun `batched stale and future results cannot claim a fresh link`() {
        assertTrue(session.receive(caneDevice.address, packet(seq = 1, at = 100L), 100L))
        assertFalse(session.receive(caneDevice.address, packet(seq = 2, at = 200L), 800L))
        assertFalse(session.receive(caneDevice.address, packet(seq = 2, at = 900L), 800L))
        assertTrue(session.receive(caneDevice.address, packet(seq = 2, at = 800L), 850L))
        assertFalse(session.receive(caneDevice.address, packet(seq = 3, at = 790L), 850L))
        assertEquals(800L, session.snapshot(850L).node(NodeType.CANE)?.packet?.receivedAt)
    }

    @Test fun `stop and restart retain duplicate detection but reject old scan batches`() {
        assertTrue(session.receive(caneDevice.address, packet(seq = 3), 0L))
        session.setScanning(false, 100L)
        assertEquals(LinkState.DISCONNECTED, session.snapshot(100L).selectedLinkState)
        session.setScanning(true, 600L)
        assertFalse(session.receive(caneDevice.address, packet(seq = 4, at = 500L), 700L))
        assertFalse(session.receive(caneDevice.address, packet(seq = 3, at = 650L), 700L))
        assertTrue(session.receive(caneDevice.address, packet(seq = 4, at = 710L), 720L))
        assertEquals(LinkState.ACTIVE, session.snapshot(720L).selectedLinkState)
    }

    @Test fun `healthy peer cannot clear a stale latched fall`() {
        assertTrue(session.receive(caneDevice.address, packet(flags = SystemPacket.FLAG_FALL_ALERT), 0L))
        assertTrue(session.receive(GLASSES_ADDRESS, packet(NodeType.GLASSES, at = 600L, flags = 0), 600L))
        val snapshot = session.snapshot(600L)
        assertTrue(snapshot.node(NodeType.CANE)?.packet?.hasLatchedFall == true)
        assertEquals(StatusTone.ALERT, snapshot.presentation().tone)
        assertTrue(snapshot.presentation().title.contains("Fall"))
    }

    @Test fun `switching selection clears old pair and legacy is address scoped`() {
        assertTrue(session.receive(caneDevice.address, packet(flags = SystemPacket.FLAG_FALL_ALERT), 0L))
        val legacy = caneDevice.copy(address = "LEGACY", pairId = null)
        session.select(legacy, 100L)
        assertNull(session.snapshot(100L).latestPacket)
        val legacyPacket = packet(at = 110L).copy(compactFormat = false)
        assertFalse(session.receive(caneDevice.address, legacyPacket, 110L))
        assertTrue(session.receive(legacy.address, legacyPacket, 110L))
        assertEquals(1, session.snapshot(110L).nodes.size)
    }

    @Test fun `compact epochs reject retired replays and preserve phone statistics on reselection`() {
        assertTrue(session.receive(caneDevice.address, packet(), 0L))
        assertTrue(session.receive(caneDevice.address, packet(seq = 2, fallSeq = 15, at = 100L), 100L))
        assertTrue(session.receive(caneDevice.address, packet(seq = 0, session = 0x1002L,
            rollover = true, at = 110L), 110L))
        assertEquals(16L, session.snapshot(110L).node(NodeType.CANE)?.phoneFallCount)
        assertFalse(session.receive(caneDevice.address, packet(seq = 3, fallSeq = 15, at = 120L), 120L))
        session.select(caneDevice, 200L)
        assertTrue(session.receive(caneDevice.address, packet(seq = 1, session = 0x1002L,
            rollover = true, fallSeq = 1, at = 210L), 210L))
        assertEquals(17L, session.snapshot(210L).node(NodeType.CANE)?.phoneFallCount)
        session.select(caneDevice.copy(address = GLASSES_ADDRESS, nodeType = NodeType.GLASSES), 220L)
        assertTrue(session.receive(caneDevice.address, packet(seq = 2, session = 0x1002L,
            rollover = true, fallSeq = 2, at = 230L), 230L))
        assertEquals(18L, session.snapshot(230L).node(NodeType.CANE)?.phoneFallCount)
    }
}
