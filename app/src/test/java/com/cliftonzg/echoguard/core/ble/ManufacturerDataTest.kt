package com.cliftonzg.echoguard.core.ble

import org.junit.Assert.*
import org.junit.Test

class ManufacturerDataTest {
    @Test fun `manufacturer payload at exact AD boundary is extracted`() {
        val payload = byteArrayOf(0xA1.toByte(), 0xB5.toByte(), 0x70, 0xFF.toByte(), 0xFF.toByte(), 0xFF.toByte())
        val ad = byteArrayOf(2, 1, 6, 9, 0xFF.toByte(), 0x29, 0) + payload
        assertArrayEquals(payload, ManufacturerData.extract(ad))
        assertNull(ManufacturerData.extract(ad.copyOf(ad.size - 1)))
    }

    @Test fun `legacy five-byte payload and byte-swapped company ID remain supported`() {
        val payload = byteArrayOf(0xAA.toByte(), 1, 2, 3, 0)
        assertArrayEquals(payload, ManufacturerData.extract(byteArrayOf(8, 0xFF.toByte(), 0, 0x29) + payload))
        assertNull(ManufacturerData.extract(byteArrayOf(8, 0xFF.toByte(), 0x12, 0x34) + payload))
    }

    @Test fun `malformed length and truncated manufacturer ID are rejected`() {
        assertNull(ManufacturerData.extract(byteArrayOf(0xFF.toByte(), 1)))
        assertNull(ManufacturerData.extract(byteArrayOf(2, 0xFF.toByte(), 0x29)))
        assertNull(ManufacturerData.extract(byteArrayOf(0)))
    }
}
