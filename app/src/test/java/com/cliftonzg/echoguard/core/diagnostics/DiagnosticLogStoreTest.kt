package com.cliftonzg.echoguard.core.diagnostics

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class DiagnosticLogStoreTest {

    @Test
    fun `ble ring buffer caps at 200 entries and evicts oldest`() {
        val store = DiagnosticLogStore()

        repeat(250) { index ->
            store.logBle(
                address = "AA:BB:CC:DD:EE:FF",
                rssi = -55,
                rawHex = "A1 00",
                outcome = BleLogOutcome.ACCEPTED,
                detail = "entry-$index",
                timestampMs = 1000L + index,
            )
        }

        val logs = store.bleLogs.value
        assertEquals(200, logs.size)
        assertEquals("entry-50", logs.first().detail)
        assertEquals("entry-249", logs.last().detail)
        assertEquals(250L, store.bleStats.value.totalFrames)
        assertEquals(250L, store.bleStats.value.acceptedFrames)
    }

    @Test
    fun `serial ring buffer caps at 400 lines and evicts oldest`() {
        val store = DiagnosticLogStore()

        repeat(450) { index ->
            store.appendSerialLine("line-$index", isOutgoing = index % 2 == 0, timestampMs = 2000L + index)
        }

        val logs = store.serialLogs.value
        assertEquals(400, logs.size)
        assertEquals("line-50", logs.first().line)
        assertEquals("line-449", logs.last().line)
    }

    @Test
    fun `pausing ble logs freezes list while stats continue updating`() {
        val store = DiagnosticLogStore()

        store.logBle("AA:01", -40, "A1", BleLogOutcome.ACCEPTED, "first", timestampMs = 100L)
        assertFalse(store.isBlePaused.value)

        store.setBlePaused(true)
        assertTrue(store.isBlePaused.value)

        store.logBle("AA:02", -50, "A2", BleLogOutcome.DROPPED, "second-dropped", timestampMs = 200L)
        store.logBle("AA:03", -60, "FF", BleLogOutcome.PARSE_ERROR, "third-error", timestampMs = 300L)

        assertEquals(1, store.bleLogs.value.size)
        assertEquals("first", store.bleLogs.value.first().detail)
        assertEquals(3L, store.bleStats.value.totalFrames)
        assertEquals(1L, store.bleStats.value.acceptedFrames)
        assertEquals(1L, store.bleStats.value.droppedFrames)
        assertEquals(1L, store.bleStats.value.parseErrors)

        store.setBlePaused(false)
        assertFalse(store.isBlePaused.value)
        store.logBle("AA:04", -45, "A4", BleLogOutcome.ACCEPTED, "fourth", timestampMs = 400L)
        assertEquals(2, store.bleLogs.value.size)
        assertEquals("fourth", store.bleLogs.value.last().detail)
    }

    @Test
    fun `stats counters track outcomes and events do not increment frame counts`() {
        val store = DiagnosticLogStore()

        store.logBle("AA:01", -40, "A1", BleLogOutcome.ACCEPTED, "ok")
        store.logBle("AA:02", -50, "A2", BleLogOutcome.DROPPED, "drop")
        store.logBle("AA:03", -60, "FF", BleLogOutcome.PARSE_ERROR, "err")
        store.logBleEvent("Scan started")

        val stats = store.bleStats.value
        assertEquals(3L, stats.totalFrames)
        assertEquals(1L, stats.acceptedFrames)
        assertEquals(1L, stats.droppedFrames)
        assertEquals(1L, stats.parseErrors)
        assertEquals(4, store.bleLogs.value.size)
        assertEquals(BleLogOutcome.EVENT, store.bleLogs.value.last().outcome)
    }

    @Test
    fun `clear methods empty logs and reset ble stats`() {
        val store = DiagnosticLogStore()

        store.logBle("AA:01", -40, "A1", BleLogOutcome.ACCEPTED, "ok")
        store.appendSerialLine("boot ok")
        store.setUsbStatus(UsbSerialStatus.Connected("Pico"))

        store.clearBleLogs()
        assertTrue(store.bleLogs.value.isEmpty())
        assertEquals(BleDiagnosticStats(), store.bleStats.value)

        store.clearSerialLogs()
        assertTrue(store.serialLogs.value.isEmpty())
        assertEquals(UsbSerialStatus.Connected("Pico"), store.usbStatus.value)
    }
}
