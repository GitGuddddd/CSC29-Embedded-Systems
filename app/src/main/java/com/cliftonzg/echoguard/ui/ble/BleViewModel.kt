package com.cliftonzg.echoguard.ui.ble

import androidx.lifecycle.ViewModel
import com.cliftonzg.echoguard.core.ble.BleScannerEngine
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class BleViewModel @Inject constructor(
    private val scannerEngine: BleScannerEngine,
) : ViewModel() {
    val monitoringState = scannerEngine.monitoringState
    val discoveredDevices = scannerEngine.discoveredDevices
    val scanError = scannerEngine.scanError

    fun startScan(): Boolean = scannerEngine.startScanning()
    fun stopScan() {
        scannerEngine.stopScanning()
    }

    fun selectDevice(device: DiscoveredBleDevice) {
        scannerEngine.selectDevice(device)
    }

    fun disconnectSelectedDevice() {
        scannerEngine.disconnectSelectedDevice()
    }
}
