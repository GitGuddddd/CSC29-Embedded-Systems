package com.cliftonzg.echoguard.ui.ble

import com.cliftonzg.echoguard.core.ble.BleScannerEngine
import com.cliftonzg.echoguard.core.diagnostics.BleDiagnosticStats
import com.cliftonzg.echoguard.core.diagnostics.BleLogOutcome
import com.cliftonzg.echoguard.core.usb.UsbSerialController
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class BleViewModelTest {

    private class FakeUsbSerialController : UsbSerialController {
        var connectCalls = 0
        var disconnectCalls = 0
        val sentCommands = mutableListOf<String>()

        override fun connectOrRequestPermission() {
            connectCalls++
        }

        override fun disconnect() {
            disconnectCalls++
        }

        override fun sendCommand(text: String) {
            sentCommands += text
        }
    }

    @Test
    fun `viewModel initial state reflects scanner engine flows`() {
        val scannerEngine = BleScannerEngine(null)
        val viewModel = BleViewModel(scannerEngine, scannerEngine.diagnostics(), FakeUsbSerialController())

        assertNull(viewModel.monitoringState.value.latestPacket)
        assertFalse(viewModel.monitoringState.value.isScanning)
        assertFalse(viewModel.isDevMenuOpen.value)
    }

    @Test
    fun `startScan returns false when bluetooth adapter is null`() {
        val scannerEngine = BleScannerEngine(null)
        val viewModel = BleViewModel(scannerEngine, scannerEngine.diagnostics(), FakeUsbSerialController())

        val started = viewModel.startScan()
        assertFalse(started)
        assertFalse(viewModel.monitoringState.value.isScanning)
    }

    @Test
    fun `dev menu opens closes and clears diagnostic logs`() {
        val scannerEngine = BleScannerEngine(null)
        val logStore = scannerEngine.diagnostics()
        val usb = FakeUsbSerialController()
        val viewModel = BleViewModel(scannerEngine, logStore, usb)

        assertFalse(viewModel.isDevMenuOpen.value)
        viewModel.openDevMenu()
        assertTrue(viewModel.isDevMenuOpen.value)
        viewModel.closeDevMenu()
        assertFalse(viewModel.isDevMenuOpen.value)

        logStore.logBle("AA:00:00:00:00:01", -50, "A1 00", BleLogOutcome.ACCEPTED, "sample")
        logStore.appendSerialLine("hello from pico")
        assertEquals(1, viewModel.bleLogs.value.size)
        assertEquals(1, viewModel.serialLogs.value.size)

        viewModel.clearBleLogs()
        assertTrue(viewModel.bleLogs.value.isEmpty())
        assertEquals(BleDiagnosticStats(), viewModel.bleStats.value)

        viewModel.clearSerialLogs()
        assertTrue(viewModel.serialLogs.value.isEmpty())

        viewModel.connectUsbSerial()
        viewModel.sendSerialCommand("c")
        viewModel.disconnectUsbSerial()
        assertEquals(1, usb.connectCalls)
        assertEquals(listOf("c"), usb.sentCommands)
        assertEquals(1, usb.disconnectCalls)
    }
}
