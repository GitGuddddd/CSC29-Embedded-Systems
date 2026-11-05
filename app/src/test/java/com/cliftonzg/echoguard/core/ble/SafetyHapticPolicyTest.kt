package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.StatusTone
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import com.cliftonzg.echoguard.core.ble.model.presentation
import org.junit.Assert.*
import org.junit.Test

class SafetyHapticPolicyTest {
    @Test fun `heartbeat updates do not restart fall waveform but reminders are bounded`() {
        val policy = SafetyHapticPolicy()
        val fall = packet(flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 5)
        assertEquals(HapticCue.FALL, policy.update(state(node(fall)), 0L))
        for (time in 100L until 5000L step 100L) {
            assertNull(policy.update(state(node(fall.copy(seqNum = (time.toInt() / 100) and 7, receivedAt = time))), time))
        }
        assertEquals(HapticCue.FALL, policy.update(state(node(fall)), 5000L))
    }

    @Test fun `pause cancels phone fall signal without clearing incident and same-seq realarm works`() {
        val policy = SafetyHapticPolicy()
        val fall = packet(flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 5)
        policy.update(state(node(fall)), 0L)
        val paused = fall.copy(stateFlags = SystemPacket.FLAG_FALL_ALERT or SystemPacket.FLAG_ALARM_PAUSED)
        assertEquals(HapticCue.STOP, policy.update(state(node(paused)), 1000L))
        assertTrue(paused.hasLatchedFall)
        assertNull(policy.update(state(node(paused)), 2000L))
        assertEquals(HapticCue.FALL, policy.update(state(node(fall)), 3000L))
    }

    @Test fun `standby with held grip fault and no range is silent`() {
        val policy = SafetyHapticPolicy()
        val parked = packet(flags = SystemPacket.FLAG_STANDBY or SystemPacket.FLAG_PARKED_BY_USER or
            SystemPacket.FLAG_CANE_HELD or SystemPacket.FLAG_SENSOR_FAULT, distance = -1)
        assertNull(policy.update(state(node(parked)), 0L))
        assertEquals(StatusTone.NEUTRAL, state(node(parked)).presentation().tone)
    }

    @Test fun `one node pause or healthy heartbeat cannot suppress another node fall`() {
        val policy = SafetyHapticPolicy()
        val cane = packet(flags = SystemPacket.FLAG_FALL_ALERT or SystemPacket.FLAG_ALARM_PAUSED)
        val headFall = packet(NodeType.GLASSES, flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 3)
        val both = state(node(cane), node(headFall))
        assertEquals(HapticCue.FALL, policy.update(both, 0L))
        val caneHealthy = node(packet())
        assertEquals(StatusTone.ALERT, state(caneHealthy, node(headFall)).presentation().tone)
        assertNull(policy.update(state(caneHealthy, node(headFall)), 100L))
    }

    @Test fun `relay report does not restart a waveform already sounding`() {
        val policy = SafetyHapticPolicy()
        val headFall = packet(NodeType.GLASSES, flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 9)
        val relayFall = packet(
            flags = SystemPacket.FLAG_FALL_ALERT,
            fallSeq = 0,
            ackValid = true,
            ackSession = headFall.sessionId,
            ackEvent = headFall.eventId,
        )
        assertEquals(HapticCue.FALL, policy.update(state(node(headFall)), 0L))
        assertNull(policy.update(state(node(headFall), node(relayFall)), 1000L))
    }

    @Test fun `independent cane event remains distinct from glasses event`() {
        val policy = SafetyHapticPolicy()
        val headFall = packet(NodeType.GLASSES, flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 9)
        val caneFall = packet(flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 1)
        assertEquals(HapticCue.FALL, policy.update(state(node(headFall)), 0L))
        assertEquals(HapticCue.FALL, policy.update(state(node(headFall), node(caneFall)), 1000L))
    }

    @Test fun `same-priority non-fall alerts remain node scoped`() {
        val policy = SafetyHapticPolicy()
        val caneHazard = packet(flags = SystemPacket.FLAG_OBSTACLE_ACTIVE)
        val glassesHazard = packet(
            NodeType.GLASSES,
            flags = SystemPacket.FLAG_OBSTACLE_ACTIVE,
            distance = 500,
        )
        assertEquals(HapticCue.CRITICAL, policy.update(state(node(caneHazard)), 0L))
        assertEquals(
            HapticCue.CRITICAL,
            policy.update(state(node(caneHazard), node(glassesHazard)), 1000L),
        )
    }

    @Test fun `new incident counter alerts before normal reminder interval`() {
        val policy = SafetyHapticPolicy()
        val old = packet(flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 15)
        policy.update(state(node(old)), 0L)
        assertEquals(
            HapticCue.FALL,
            policy.update(state(node(old.copy(rollingFallSeq = 0, eventId = 16L))), 1000L),
        )
    }

    @Test fun `new boot session cannot alias an old incident counter`() {
        val policy = SafetyHapticPolicy()
        val old = packet(flags = SystemPacket.FLAG_FALL_ALERT, fallSeq = 1, session = 0x1001L)
        policy.update(state(node(old)), 0L)
        assertEquals(
            HapticCue.FALL,
            policy.update(state(node(old.copy(sessionId = 0x1002L))), 1000L),
        )
    }

    @Test fun `manual stop cancels alerts and link loss uses a distinct bounded cue`() {
        val policy = SafetyHapticPolicy()
        val active = state(node(packet(flags = SystemPacket.FLAG_FALL_ALERT)))
        policy.update(active, 0L)
        assertEquals(HapticCue.STOP, policy.update(active.copy(isScanning = false), 100L))
        val lost = state(node(packet(), LinkState.LINK_LOSS))
        assertEquals(HapticCue.LINK_LOSS, policy.update(lost, 500L))
        assertNull(policy.update(lost, 600L))
    }

    @Test fun `fall remains higher priority than faults and missing peers`() {
        val policy = SafetyHapticPolicy()
        val fall = node(packet(flags = SystemPacket.FLAG_FALL_ALERT or SystemPacket.FLAG_STANDBY))
        val lost = node(packet(NodeType.GLASSES), LinkState.LINK_LOSS)
        assertEquals(HapticCue.FALL, policy.update(state(fall, lost), 0L))
    }
}
