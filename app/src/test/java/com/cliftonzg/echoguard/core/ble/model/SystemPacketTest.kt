package com.cliftonzg.echoguard.core.ble.model

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class SystemPacketTest {

    @Test
    fun `parse valid 5-byte legacy packet calculates correct XOR checksum`() {
        val magic = 0xAA
        val seq = 0x05
        val distMm = 1200 // 0x04B0 -> low=0xB0 (176), high=0x04 (4)
        val distLow = 0xB0
        val distHigh = 0x04
        val checksum = (magic xor seq xor distLow xor distHigh) and 0xFF

        val byteArray = byteArrayOf(
            magic.toByte(),
            seq.toByte(),
            distLow.toByte(),
            distHigh.toByte(),
            checksum.toByte(),
        )

        val packet = SystemPacket.parse(byteArray)
        assertNotNull(packet)
        assertEquals(0xAA, packet?.magic)
        assertEquals(5, packet?.seqNum)
        assertEquals(distMm, packet?.radarDistMm)
        assertTrue(packet?.isValid() == true)
        assertFalse(packet?.isAdvanced == true)
    }

    @Test
    fun `parse 5-byte packet with invalid magic byte returns null`() {
        val byteArray = byteArrayOf(0xBB.toByte(), 0x01, 0x00, 0x00, 0xBB.toByte())
        val packet = SystemPacket.parse(byteArray)
        assertNull(packet)
    }

    @Test
    fun `parse 5-byte packet with corrupted checksum returns null`() {
        val byteArray = byteArrayOf(0xAA.toByte(), 0x01, 0x10, 0x00, 0xFF.toByte())
        val packet = SystemPacket.parse(byteArray)
        assertNull(packet)
    }

    @Test
    fun `parse 11-byte AdvancedThreatPacket decodes all fields correctly`() {
        // [Preamble(1B), DeviceID(1B), SeqNum(2B), StateFlags(1B), TargetDist(2B), TargetSpeed(2B), TTC(2B)]
        val data = byteArrayOf(
            0xAA.toByte(), // preamble
            0x01,          // deviceId
            0x10, 0x00,    // seqNum = 16
            0x07,          // stateFlags: Fall (1) + Radar Fault (2) + Obstacle Active (4) = 7
            0xE8.toByte(), 0x03, // dist = 1000 mm (0x03E8)
            0xF6.toByte(), 0xFF.toByte(), // closingSpeed = -10 cm/s
            0xF4.toByte(), 0x01, // TTC = 500 ms (0x01F4)
        )

        val packet = SystemPacket.parse(data)
        assertNotNull(packet)
        assertTrue(packet?.isAdvanced == true)
        assertEquals(0xAA, packet?.magic)
        assertEquals(1, packet?.deviceId)
        assertEquals(16, packet?.seqNum)
        assertEquals(7, packet?.stateFlags)
        assertTrue(packet?.isFallAlert == true)
        assertTrue(packet?.isRadarFault == true)
        assertTrue(packet?.isObstacleActive == true)
        assertEquals(1000, packet?.radarDistMm)
        assertEquals(-10, packet?.closingSpeedCms)
        assertEquals(500, packet?.ttcMs)
    }

    @Test
    fun `parse 6-byte compact packet decodes all fields correctly`() {
        // Byte 0: Sync 0xA + PairID 0x1 -> 0xA1
        // Byte 1: Node Type Glasses (0) + Heartbeat Seq 3 + Rolling Fall Seq 5 -> 0x35
        // Byte 2: Flags: Fall (1) + Radar Fault (2) = 0x03
        // Byte 3: Dist Low: 1200 mm -> 600 units (0x0258) -> distLow = 0x58
        // Byte 4: Dist High (0x2) + TTC Low (1000 ms -> 50 units = 0x032 -> low = 0x2) -> 0x22
        // Byte 5: TTC High (0x03) -> 0x03
        val data = byteArrayOf(
            0xA1.toByte(),
            0x35.toByte(),
            0x03.toByte(),
            0x58.toByte(),
            0x22.toByte(),
            0x03.toByte(),
        )

        val packet = SystemPacket.parse(data)
        assertNotNull(packet)
        assertTrue(packet?.compactFormat == true)
        assertTrue(packet?.isAdvanced == true)
        assertEquals(0xAA, packet?.magic)
        assertEquals(2, packet?.deviceId)
        assertEquals(1, packet?.pairId)
        assertEquals(3, packet?.seqNum)
        assertEquals(5, packet?.rollingFallSeq)
        assertEquals(3, packet?.stateFlags)
        assertTrue(packet?.isFallAlert == true)
        assertTrue(packet?.isRadarFault == true)
        assertEquals(1200, packet?.radarDistMm)
        assertEquals(1000, packet?.ttcMs)
    }

    @Test
    fun `parse 6-byte compact packet handles no target and no threat sentinels`() {
        // Dist raw = 0xFFF (-1 / no target), TTC raw = 0xFFF (-1 / no threat)
        val data = byteArrayOf(
            0xA1.toByte(),
            0x90.toByte(), // Cane node (1 << 7), hbSeq=1, fallSeq=0
            0x00.toByte(),
            0xFF.toByte(),
            0xFF.toByte(),
            0xFF.toByte(),
        )

        val packet = SystemPacket.parse(data)
        assertNotNull(packet)
        assertTrue(packet?.compactFormat == true)
        assertEquals(1, packet?.deviceId)
        assertEquals(-1, packet?.radarDistMm)
        assertEquals(-1, packet?.ttcMs)
    }

    @Test
    fun `threatLevel evaluates correctly based on sensor type and distance`() {
        // Glasses Radar (Forward obstacle sensor): close = CRITICAL (Red)
        val glassesCritical = SystemPacket(magic = 0xAA, deviceId = 0x02, seqNum = 1, radarDistMm = 450, checksum = 0)
        assertEquals(ThreatLevel.CRITICAL, glassesCritical.threatLevel)

        val glassesWarning = SystemPacket(magic = 0xAA, deviceId = 0x02, seqNum = 2, radarDistMm = 1200, checksum = 0)
        assertEquals(ThreatLevel.WARNING, glassesWarning.threatLevel)

        val glassesSafe = SystemPacket(magic = 0xAA, deviceId = 0x02, seqNum = 3, radarDistMm = 1800, checksum = 0)
        assertEquals(ThreatLevel.SAFE, glassesSafe.threatLevel)

        // Cane ToF (Downward ground sensor): small distance (solid ground) = SAFE (Green)
        val caneSolidGround = SystemPacket(magic = 0xAA, deviceId = 0x01, seqNum = 1, radarDistMm = 300, checksum = 0)
        assertEquals(ThreatLevel.SAFE, caneSolidGround.threatLevel)

        val caneNormalGround = SystemPacket(magic = 0xAA, deviceId = 0x01, seqNum = 2, radarDistMm = 1000, checksum = 0)
        assertEquals(ThreatLevel.SAFE, caneNormalGround.threatLevel)

        // Cane ToF: large distance or -1 (cliff / drop-off / hole) = CRITICAL (Red)
        val caneCliff = SystemPacket(magic = 0xAA, deviceId = 0x01, seqNum = 3, radarDistMm = 2000, checksum = 0)
        assertEquals(ThreatLevel.CRITICAL, caneCliff.threatLevel)

        val caneDropOff = SystemPacket(magic = 0xAA, deviceId = 0x01, seqNum = 4, radarDistMm = -1, checksum = 0)
        assertEquals(ThreatLevel.CRITICAL, caneDropOff.threatLevel)

        val unknownPacket = SystemPacket(magic = 0xAA, deviceId = 0x01, seqNum = 5, radarDistMm = 0, checksum = 0)
        assertEquals(ThreatLevel.UNKNOWN, unknownPacket.threatLevel)
    }

    @Test
    fun `isNewerThan correctly enforces monotonic sequence check`() {
        val packetNew = SystemPacket(magic = 0xAA, seqNum = 10, radarDistMm = 500, checksum = 0)
        assertTrue(packetNew.isNewerThan(5))
        assertFalse(packetNew.isNewerThan(10))
        assertFalse(packetNew.isNewerThan(12))

        // Wrap-around check for 8-bit sequence (max 256)
        val packetWrap = SystemPacket(magic = 0xAA, seqNum = 2, radarDistMm = 500, checksum = 0, isAdvanced = false)
        assertTrue(packetWrap.isNewerThan(250))

        // Wrap-around check for 16-bit sequence (max 65536)
        val packetWrap16 = SystemPacket(magic = 0xAA, seqNum = 5, radarDistMm = 500, checksum = 0, isAdvanced = true)
        assertTrue(packetWrap16.isNewerThan(65530))

        // Wrap-around check for compact 3-bit sequence (max 8)
        val packetWrap3Bit = SystemPacket(magic = 0xAA, seqNum = 0, radarDistMm = 500, checksum = 0, compactFormat = true)
        assertTrue(packetWrap3Bit.isNewerThan(7))
    }

    private fun compact(flags: Int, distanceRaw: Int = 0xFFF, cane: Boolean = true): SystemPacket {
        val bytes = byteArrayOf(
            0xA1.toByte(), (if (cane) 0xB5 else 0x35).toByte(), flags.toByte(),
            distanceRaw.toByte(), (((distanceRaw shr 8) shl 4) or 0x0F).toByte(), 0xFF.toByte(),
        )
        return requireNotNull(SystemPacket.parse(bytes, 123L))
    }

    @Test fun `firmware paused packet retains incident and acknowledgement metadata`() {
        val packet = compact(0x19) // FALL + PAUSED + HELD
        assertTrue(packet.hasLatchedFall)
        assertTrue(packet.isAlarmPaused)
        assertTrue(packet.isCaneHeld)
        assertFalse(packet.isStandby)
        assertEquals(TelemetryStatus.FALL_PAUSED, packet.status)
        assertEquals(ThreatLevel.CRITICAL, packet.threatLevel)
        assertEquals(5, packet.rollingFallSeq)
        assertEquals(123L, packet.receivedAt)
    }

    @Test fun `standby overrides held grip and absent range without hiding fault flag`() {
        val packet = compact(0x72) // SENSOR_FAULT + HELD + STANDBY + PARKED_BY_USER
        assertTrue(packet.isStandby)
        assertTrue(packet.isParkedByUser)
        assertTrue(packet.isSensorFault)
        assertEquals(TelemetryStatus.STANDBY, packet.status)
        assertEquals(ThreatLevel.UNKNOWN, packet.threatLevel)
        assertEquals(-1, packet.radarDistMm)
    }

    @Test fun `modern cane uses calibrated hazard flags instead of a hardcoded ground distance`() {
        assertEquals(ThreatLevel.SAFE, compact(0x10, 1000).threatLevel) // 2000 mm ground baseline
        assertEquals(TelemetryStatus.GROUND_HAZARD, compact(0x14, 100).status) // Explicit hazard at 200 mm
        assertEquals(TelemetryStatus.GRIP_RELEASED, compact(0x00).status)
        assertEquals(TelemetryStatus.NO_READING, compact(0x10).status)
    }

    @Test fun `glasses respect firmware threat filtering and do not invent compact speed`() {
        assertEquals(ThreatLevel.SAFE, compact(0x00, 200, cane = false).threatLevel)
        assertEquals(ThreatLevel.CRITICAL, compact(0x04, 200, cane = false).threatLevel)
        assertFalse(compact(0x00, cane = false).hasClosingSpeed)
        assertEquals(TelemetryStatus.SENSOR_FAULT, compact(0x02, cane = false).status)
    }

    @Test fun `legacy advanced sentinels and missing TTC are unavailable rather than zero`() {
        val bytes = byteArrayOf(0xAA.toByte(), 2, 1, 0, 0, 0xFF.toByte(), 0xFF.toByte(), 0, 0, 0xFF.toByte(), 0xFF.toByte())
        val full = requireNotNull(SystemPacket.parse(bytes))
        assertEquals(-1, full.radarDistMm)
        assertEquals(-1, full.ttcMs)
        assertEquals(-1, SystemPacket.parse(bytes.copyOf(10))?.ttcMs)
    }

    @Test fun `unsupported payload sizes do not fall through to legacy checksum parsing`() {
        val validBasic = byteArrayOf(0xAA.toByte(), 1, 2, 3, (0xAA xor 1 xor 2 xor 3).toByte())
        for (length in listOf(7, 8, 9, 12, 14, 20, 23, 25)) {
            assertNull(SystemPacket.parse(validBasic.copyOf(length)))
        }
    }

    @Test fun `compact v2 golden vector carries four-bit sequence and scoped receipt`() {
        // Same 12 bytes asserted by test_Fall_to_Ble_Dispatch.c.
        val extended = byteArrayOf(
            0xA5.toByte(), 0xEE.toByte(), 0x01, 0xFF.toByte(), 0xFF.toByte(), 0xFF.toByte(),
            0xE2.toByte(), 0x04, 0xA1.toByte(), 0x05, 0xB2.toByte(), 0x32,
        )
        val packet = requireNotNull(SystemPacket.parse(extended, 900L))
        assertEquals(SystemPacket.PROTOCOL_VERSION, packet.protocolVersion)
        assertEquals(0xA104L, packet.sessionId)
        assertEquals(14, packet.rollingFallSeq)
        assertEquals(14L, packet.eventId)
        assertTrue(packet.peerAckValid)
        assertTrue(packet.sessionRollover)
        assertEquals(0xB205L, packet.peerAckSessionId)
        assertEquals(2L, packet.peerAckEventId)
        for (length in 7..11) assertNull(SystemPacket.parse(extended.copyOf(length)))
        extended[6] = 0xE1.toByte()
        assertNull(SystemPacket.parse(extended))
        extended[6] = SystemPacket.SAFETY_EXTENSION_MARKER.toByte()
        extended[7] = 0
        extended[8] = 0
        assertNull(SystemPacket.parse(extended))
    }

    @Test fun `compact receipt rejects reserved flags and missing acknowledgement epoch`() {
        val bytes = byteArrayOf(
            0xA1.toByte(), 0xB5.toByte(), 1, 0xFF.toByte(), 0xFF.toByte(), 0xFF.toByte(),
            0xE2.toByte(), 1, 0, 0, 0, 0x10,
        )
        assertNull(SystemPacket.parse(bytes))
        bytes[11] = 0x80.toByte()
        assertNull(SystemPacket.parse(bytes))
    }

    @Test fun `pair ten truncated extension cannot masquerade as a legacy advanced packet`() {
        val bytes = byteArrayOf(
            0xAA.toByte(), 0x01, 0, 0, 0, 0,
            0xE2.toByte(), 1, 0, 0, 0, 0,
        )
        assertNotNull(SystemPacket.parse(bytes))
        for (length in 7..11) assertNull(SystemPacket.parse(bytes.copyOf(length)))
        bytes[6] = 0xE3.toByte()
        assertNull(SystemPacket.parse(bytes.copyOf(11)))
    }

    @Test fun `former 24-byte v1 packets remain readable but are not current receipt protocol`() {
        val extended = byteArrayOf(
            0xA1.toByte(), 0xB5.toByte(), SystemPacket.FLAG_FALL_ALERT.toByte(),
            0xFF.toByte(), 0xFF.toByte(), 0xFF.toByte(),
            SystemPacket.LEGACY_SAFETY_EXTENSION_MARKER.toByte(),
            0x78, 0x56, 0x34, 0x12,
            0xEF.toByte(), 0xCD.toByte(), 0xAB.toByte(), 0x09,
            0x21, 0x43, 0x65, 0x07,
            0x98.toByte(), 0xBA.toByte(), 0xDC.toByte(), 0x0F,
            0x01,
        )
        val packet = requireNotNull(SystemPacket.parse(extended, 900L))
        assertEquals(SystemPacket.LEGACY_PROTOCOL_VERSION, packet.protocolVersion)
        assertEquals(0x12345678L, packet.sessionId)
        assertEquals(0x09ABCDEFL, packet.eventId)
        assertTrue(packet.peerAckValid)
        assertEquals(0x07654321L, packet.peerAckSessionId)
        assertEquals(0x0FDCBA98L, packet.peerAckEventId)
        extended[6] = 0xE2.toByte()
        assertNull(SystemPacket.parse(extended))
        extended[6] = SystemPacket.LEGACY_SAFETY_EXTENSION_MARKER.toByte()
        extended[7] = 0
        extended[8] = 0
        extended[9] = 0
        extended[10] = 0
        assertNull(SystemPacket.parse(extended))
    }
}
