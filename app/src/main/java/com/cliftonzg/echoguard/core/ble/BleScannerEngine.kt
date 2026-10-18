package com.cliftonzg.echoguard.core.ble

import android.bluetooth.BluetoothAdapter
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanRecord
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.util.Log
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject
import javax.inject.Singleton
import kotlin.time.Duration.Companion.milliseconds

@Singleton
class BleScannerEngine @Inject constructor(
    bluetoothAdapter: BluetoothAdapter?,
) {
    private val adapter = bluetoothAdapter
    private val scanner get() = adapter?.bluetoothLeScanner

    private val engineScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    private var watchdogJob: Job? = null

    private val _packets = MutableStateFlow<SystemPacket?>(null)
    val packets = _packets.asStateFlow()

    private val _canePacket = MutableStateFlow<SystemPacket?>(null)
    val canePacket = _canePacket.asStateFlow()

    private val _glassesPacket = MutableStateFlow<SystemPacket?>(null)
    val glassesPacket = _glassesPacket.asStateFlow()

    private val _isScanning = MutableStateFlow(value = false)
    val isScanning = _isScanning.asStateFlow()

    private val _linkState = MutableStateFlow(LinkState.DISCONNECTED)
    val linkState = _linkState.asStateFlow()

    private val _discoveredDevices = MutableStateFlow<List<DiscoveredBleDevice>>(emptyList())
    val discoveredDevices = _discoveredDevices.asStateFlow()

    private val _selectedDevice = MutableStateFlow<DiscoveredBleDevice?>(null)
    val selectedDevice = _selectedDevice.asStateFlow()

    private val deviceMap = mutableMapOf<String, DiscoveredBleDevice>()

    private var lastSeqNum: Int = -1
    @Volatile private var lastPacketTimeMs: Long = 0L

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val record = result.scanRecord ?: return
            val address = result.device.address ?: return
            val rawName = try {
                result.device.name
            } catch (_: SecurityException) {
                null
            } ?: record.deviceName

            val data = extractManufacturerData(record)
            val packet = data?.let { SystemPacket.parse(it) }

            // Strict EchoGuard Verification: Must have valid packet (preamble 0xAA) or explicit EchoGuard name
            val isEchoGuard = (packet != null && packet.isValid()) || 
                              (rawName?.contains("EchoGuard", ignoreCase = true) == true)

            // Accurate Friendly Naming
            val name = when {
                packet?.deviceId == 0x01 -> "EchoGuard Smart Cane"
                packet?.deviceId == 0x02 -> "EchoGuard Glasses Node"
                !rawName.isNullOrBlank() && rawName != "null" -> rawName
                packet != null && isEchoGuard -> "EchoGuard Node"
                else -> "Unknown Device ($address)"
            }

            val now = System.currentTimeMillis()
            val device = DiscoveredBleDevice(
                name = name,
                address = address,
                rssi = result.rssi,
                isEchoGuardNode = isEchoGuard,
                lastSeenMs = now,
            )

            synchronized(deviceMap) {
                deviceMap[address] = device
                _discoveredDevices.value = deviceMap.values.sortedByDescending { it.isEchoGuardNode }
            }

            if (packet != null && packet.isValid()) {
                if (packet.deviceId == 0x01) {
                    _canePacket.value = packet
                } else if (packet.deviceId == 0x02) {
                    _glassesPacket.value = packet
                }

                // When NO device is paired/selected, allow telemetry to update if an EchoGuard node is active
                val currentSelected = _selectedDevice.value
                if (currentSelected != null && currentSelected.address != address) {
                    return
                }

                val isNewer = packet.isNewerThan(lastSeqNum)
                Log.d("EchoGuardBLE", "ScanResult from device ($address): seqNum=${packet.seqNum}, devId=0x%02X, isNewer=$isNewer".format(packet.deviceId))
                if (isNewer) {
                    lastSeqNum = packet.seqNum
                    lastPacketTimeMs = now
                    _packets.value = packet
                    if (_linkState.value != LinkState.ACTIVE) {
                        Log.i("EchoGuardBLE", "LinkState transitioned to ACTIVE for $address")
                        _linkState.value = LinkState.ACTIVE
                    }
                }
            }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e("EchoGuardBLE", "BLE Scan Failed with errorCode=$errorCode")
            _isScanning.value = false
            _linkState.value = LinkState.DISCONNECTED
            stopWatchdog()
        }
    }

    fun selectDevice(device: DiscoveredBleDevice) {
        Log.i("EchoGuardBLE", "selectDevice: address=${device.address}, name=${device.name}")
        _selectedDevice.value = device
        lastSeqNum = -1
        lastPacketTimeMs = System.currentTimeMillis()
        _linkState.value = LinkState.ACTIVE
    }

    fun disconnectSelectedDevice() {
        Log.i("EchoGuardBLE", "disconnectSelectedDevice")
        _selectedDevice.value = null
        _packets.value = null
        _linkState.value = LinkState.DISCONNECTED
    }

    private fun extractManufacturerData(record: ScanRecord): ByteArray? {
        val sparse = record.manufacturerSpecificData
        if (sparse != null && sparse.size() > 0) {
            // Strictly match EchoGuard Company ID 0x0029 or 0x2900
            val known = sparse[0x0029] ?: sparse[0x2900]
            if (known != null) {
                return known
            }
        }

        val raw = record.bytes ?: return null
        var index = 0
        while (index < (raw.size - 2)) {
            val length = raw[index].toInt() and 0xFF
            if (length == 0 || index + length >= raw.size) break
            val type = raw[index + 1].toInt() and 0xFF
            if (type == 0xFF && length >= 12) {
                val companyId = (raw[index + 2].toInt() and 0xFF) or ((raw[index + 3].toInt() and 0xFF) shl 8)
                if (companyId == 0x0029 || companyId == 0x2900) {
                    val dataLen = length - 3
                    val data = ByteArray(dataLen)
                    System.arraycopy(raw, index + 4, data, 0, dataLen)
                    return data
                }
            }
            index += length + 1
        }
        return null
    }

    fun startScanning(): Boolean {
        Log.i("EchoGuardBLE", "startScanning requested")
        if (adapter == null || !adapter.isEnabled) {
            Log.e("EchoGuardBLE", "BluetoothAdapter is disabled or null!")
            _isScanning.value = false
            _linkState.value = LinkState.DISCONNECTED
            return false
        }

        if (_isScanning.value) {
            Log.d("EchoGuardBLE", "Already scanning")
            return true
        }

        val currentScanner = scanner ?: return false

        synchronized(deviceMap) {
            deviceMap.clear()
            _discoveredDevices.value = emptyList()
        }

        return try {
            val settings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
                .build()

            try {
                currentScanner.startScan(null, settings, scanCallback)
            } catch (_: SecurityException) {
                Log.e("EchoGuardBLE", "SecurityException during startScan - Bluetooth permission missing!")
            }
            _isScanning.value = true
            _linkState.value = if (_selectedDevice.value != null) LinkState.ACTIVE else LinkState.DISCONNECTED
            Log.i("EchoGuardBLE", "BLE Low-Latency Scan started successfully")
            startWatchdog()
            true
        } catch (e: Exception) {
            Log.e("EchoGuardBLE", "Exception during startScan: ${e.message}")
            _isScanning.value = false
            _linkState.value = LinkState.DISCONNECTED
            false
        }
    }

    fun stopScanning() {
        Log.i("EchoGuardBLE", "stopScanning requested")
        val currentScanner = scanner ?: return
        try {
            currentScanner.stopScan(scanCallback)
        } catch (e: Exception) {
            Log.w("EchoGuardBLE", "Exception during stopScan: ${e.message}")
        } finally {
            _isScanning.value = false
            if (_selectedDevice.value == null) {
                _linkState.value = LinkState.DISCONNECTED
            }
            stopWatchdog()
        }
    }

    private fun startWatchdog() {
        stopWatchdog()
        lastPacketTimeMs = System.currentTimeMillis()
        watchdogJob = engineScope.launch {
            while (true) {
                delay(500.milliseconds)
                val now = System.currentTimeMillis()

                // Prune stale devices from discovery list if last seen > 3000 ms
                synchronized(deviceMap) {
                    val initialSize = deviceMap.size
                    deviceMap.entries.removeAll { (_, dev) ->
                        (now - dev.lastSeenMs) > 3000L
                    }
                    if (deviceMap.size != initialSize) {
                        _discoveredDevices.value = deviceMap.values.sortedByDescending { it.isEchoGuardNode }
                    }
                }

                val currentSelected = _selectedDevice.value
                if (currentSelected == null) {
                    if (_linkState.value != LinkState.DISCONNECTED) {
                        _linkState.value = LinkState.DISCONNECTED
                    }
                } else if (_isScanning.value) {
                    val elapsed = now - lastPacketTimeMs
                    if (elapsed >= 500 && lastPacketTimeMs > 0) {
                        if (_linkState.value != LinkState.LINK_LOSS) {
                            Log.w("EchoGuardBLE", "Watchdog: LINK_LOSS triggered for selected device ${currentSelected.address}! Elapsed=$elapsed ms")
                            _linkState.value = LinkState.LINK_LOSS
                        }
                    }
                }
            }
        }
    }

    private fun stopWatchdog() {
        watchdogJob?.cancel()
        watchdogJob = null
    }
}
