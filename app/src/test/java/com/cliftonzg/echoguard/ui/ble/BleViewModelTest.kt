package com.cliftonzg.echoguard.ui.ble

import com.cliftonzg.echoguard.core.ble.BleScannerEngine
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Test

class BleViewModelTest {

    @Test
    fun `viewModel initial state reflects scanner engine flows`() {
        val scannerEngine = BleScannerEngine(null)
        val viewModel = BleViewModel(scannerEngine)

        assertNull(viewModel.monitoringState.value.latestPacket)
        assertFalse(viewModel.monitoringState.value.isScanning)
    }

    @Test
    fun `startScan returns false when bluetooth adapter is null`() {
        val scannerEngine = BleScannerEngine(null)
        val viewModel = BleViewModel(scannerEngine)

        val started = viewModel.startScan()
        assertFalse(started)
        assertFalse(viewModel.monitoringState.value.isScanning)
    }
}
