package com.cliftonzg.echoguard.ui.ble

import androidx.lifecycle.ViewModel
import com.cliftonzg.echoguard.core.ble.BleScannerEngine
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.StateFlow
import javax.inject.Inject

@HiltViewModel
class BleViewModel @Inject constructor(
    private val scannerEngine: BleScannerEngine,
) : ViewModel() {
    val latestPacket = scannerEngine.packets
    val canePacket = scannerEngine.canePacket
    val glassesPacket = scannerEngine.glassesPacket
    val isScanning = scannerEngine.isScanning
    val linkState: StateFlow<LinkState> = scannerEngine.linkState
    val discoveredDevices = scannerEngine.discoveredDevices
    val selectedDevice = scannerEngine.selectedDevice

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
