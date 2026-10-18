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
    fun `threatLevel evaluates correctly based on distance`() {
        val criticalPacket = SystemPacket(magic = 0xAA, seqNum = 1, radarDistMm = 450, checksum = 0)
        assertEquals(ThreatLevel.CRITICAL, criticalPacket.threatLevel)

        val warningPacket = SystemPacket(magic = 0xAA, seqNum = 2, radarDistMm = 1200, checksum = 0)
        assertEquals(ThreatLevel.WARNING, warningPacket.threatLevel)

        val safePacket = SystemPacket(magic = 0xAA, seqNum = 3, radarDistMm = 1800, checksum = 0)
        assertEquals(ThreatLevel.SAFE, safePacket.threatLevel)

        val unknownPacket = SystemPacket(magic = 0xAA, seqNum = 4, radarDistMm = 0, checksum = 0)
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
    }
}
