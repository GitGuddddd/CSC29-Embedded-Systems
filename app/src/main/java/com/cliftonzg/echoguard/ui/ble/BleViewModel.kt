package com.cliftonzg.echoguard.ui.ble

import androidx.lifecycle.ViewModel
import com.cliftonzg.echoguard.core.ble.BleScannerEngine
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.diagnostics.DiagnosticLogStore
import com.cliftonzg.echoguard.core.usb.UsbSerialController
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import javax.inject.Inject

@HiltViewModel
class BleViewModel @Inject constructor(
    private val scannerEngine: BleScannerEngine,
    private val logStore: DiagnosticLogStore,
    private val usbSerialController: UsbSerialController,
) : ViewModel() {
    val monitoringState = scannerEngine.monitoringState
    val discoveredDevices = scannerEngine.discoveredDevices
    val scanError = scannerEngine.scanError

    private val _isDevMenuOpen = MutableStateFlow(false)
    val isDevMenuOpen = _isDevMenuOpen.asStateFlow()

    val bleLogs = logStore.bleLogs
    val bleStats = logStore.bleStats
    val isBlePaused = logStore.isBlePaused
    val serialLogs = logStore.serialLogs
    val usbStatus = logStore.usbStatus

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

    fun openDevMenu() {
        _isDevMenuOpen.value = true
    }

    fun closeDevMenu() {
        _isDevMenuOpen.value = false
    }

    fun toggleBlePaused() {
        logStore.setBlePaused(!logStore.isBlePaused.value)
    }

    fun clearBleLogs() {
        logStore.clearBleLogs()
    }

    fun clearSerialLogs() {
        logStore.clearSerialLogs()
    }

    fun connectUsbSerial() {
        usbSerialController.connectOrRequestPermission()
    }

    fun disconnectUsbSerial() {
        usbSerialController.disconnect()
    }

    fun sendSerialCommand(cmd: String) {
        usbSerialController.sendCommand(cmd)
    }
}
