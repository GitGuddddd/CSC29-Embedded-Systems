package com.cliftonzg.echoguard.core.ble

import android.bluetooth.BluetoothAdapter
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanRecord
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.os.SystemClock
import android.util.Log
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
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

@Singleton
class BleScannerEngine @Inject constructor(
    bluetoothAdapter: BluetoothAdapter?,
    fallHistoryStore: FallHistoryStore = MemoryFallHistoryStore(),
) {
    private val adapter = bluetoothAdapter
    private val lock = Any()
    private val engineScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    private var watchdogJob: Job? = null
    private val session = TelemetrySession(PhoneFallHistory(fallHistoryStore))
    private val _monitoringState = MutableStateFlow(MonitoringState())
    val monitoringState = _monitoringState.asStateFlow()
    private val _discoveredDevices = MutableStateFlow<List<DiscoveredBleDevice>>(emptyList())
    val discoveredDevices = _discoveredDevices.asStateFlow()
    private val _scanError = MutableStateFlow<String?>(null)
    val scanError = _scanError.asStateFlow()
    private val deviceMap = mutableMapOf<String, DiscoveredBleDevice>()
    private var scanGeneration = 0L
    private var scanRequested = false
    private var activeScanner: BluetoothLeScanner? = null
    private var activeCallback: ScanCallback? = null

    private fun callbackFor(generation: Long) = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            handleResult(generation, result)
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>) {
            results.sortedBy { it.timestampNanos }.forEach { handleResult(generation, it) }
        }

        override fun onScanFailed(errorCode: Int) {
            synchronized(lock) {
                if (generation != scanGeneration) return
                activeScanner = null
                activeCallback = null
                watchdogJob?.cancel()
                session.setScanning(false, SystemClock.elapsedRealtime())
                publishLocked()
                _scanError.value = "Bluetooth scan failed (code $errorCode). Try starting the scan again."
            }
            Log.w("EchoGuardBLE", "Scan failed: $errorCode")
        }
    }

    private fun handleResult(generation: Long, result: ScanResult) {
        val now = SystemClock.elapsedRealtime()
        val capturedMs = result.timestampNanos / 1_000_000L
        if (capturedMs > now || now - capturedMs >= DISCOVERY_TIMEOUT_MS) return
        val record = result.scanRecord ?: return
        val data = extractManufacturerData(record) ?: return
        val packet = SystemPacket.parse(data, capturedMs) ?: return
        val address = try {
            result.device.address
        } catch (_: SecurityException) {
            return
        }
        val device = DiscoveredBleDevice(
            name = "EchoGuard ${packet.nodeType.label}",
            address = address,
            rssi = result.rssi,
            isEchoGuardNode = true,
            lastSeenMs = capturedMs,
            nodeType = packet.nodeType,
            pairId = packet.pairId.takeIf { packet.compactFormat },
        )
        synchronized(lock) {
            if (generation != scanGeneration || !_monitoringState.value.isScanning) return
            if (capturedMs >= (deviceMap[address]?.lastSeenMs ?: -1L)) {
                deviceMap[address] = device
                publishDiscoveryLocked()
            }
            if (session.receive(address, packet, now)) publishLocked(now)
        }
    }

    fun selectDevice(device: DiscoveredBleDevice) {
        synchronized(lock) {
            session.select(device, SystemClock.elapsedRealtime())
            publishLocked()
        }
    }

    fun disconnectSelectedDevice() {
        synchronized(lock) {
            session.select(null, SystemClock.elapsedRealtime())
            publishLocked()
        }
    }

    private fun extractManufacturerData(record: ScanRecord): ByteArray? {
        return record.getManufacturerSpecificData(ManufacturerData.COMPANY_ID)
            ?: record.getManufacturerSpecificData(ManufacturerData.LEGACY_COMPANY_ID)
            ?: record.bytes?.let(ManufacturerData::extract)
    }

    fun startScanning(): Boolean {
        synchronized(lock) {
            scanRequested = true
            if (_monitoringState.value.isScanning) return true
        }
        return try {
            if (adapter == null || !adapter.isEnabled) {
                _scanError.value = "Bluetooth is unavailable or turned off."
                return false
            }
            val currentScanner = adapter.bluetoothLeScanner ?: run {
                _scanError.value = "Bluetooth scanning is unavailable."
                return false
            }
            val settings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
                .setReportDelay(0L)
                .build()
            val filters = listOf(ManufacturerData.COMPANY_ID, ManufacturerData.LEGACY_COMPANY_ID).map { id ->
                ScanFilter.Builder().setManufacturerData(id, byteArrayOf()).build()
            }
            val callback: ScanCallback
            val generation: Long
            synchronized(lock) {
                generation = ++scanGeneration
                callback = callbackFor(generation)
                activeScanner = currentScanner
                activeCallback = callback
                session.setScanning(true, SystemClock.elapsedRealtime())
                _scanError.value = null
                deviceMap.clear()
                publishDiscoveryLocked()
                publishLocked()
            }
            currentScanner.startScan(filters, settings, callback)
            synchronized(lock) {
                if (generation == scanGeneration && _monitoringState.value.isScanning) {
                    startWatchdogLocked(generation)
                }
                _monitoringState.value.isScanning
            }
        } catch (_: SecurityException) {
            stopScanningInternal(clearRequest = false)
            _scanError.value = "Bluetooth permission is required to scan."
            false
        } catch (error: RuntimeException) {
            stopScanningInternal(clearRequest = false)
            Log.w("EchoGuardBLE", "Could not start scan", error)
            _scanError.value = "Could not start Bluetooth scanning. Try again."
            false
        }
    }

    fun stopScanning() = stopScanningInternal(clearRequest = true)

    fun onBluetoothStateChanged(enabled: Boolean) {
        if (enabled) {
            _scanError.value = null
            if (synchronized(lock) { scanRequested }) startScanning()
        } else {
            stopScanningInternal(clearRequest = false)
            _scanError.value = "Bluetooth is turned off."
        }
    }

    private fun stopScanningInternal(clearRequest: Boolean) {
        val currentScanner: BluetoothLeScanner?
        val callback: ScanCallback?
        synchronized(lock) {
            if (clearRequest) scanRequested = false
            ++scanGeneration // Ignore callbacks/batches from the previous native scan.
            currentScanner = activeScanner
            callback = activeCallback
            activeScanner = null
            activeCallback = null
            watchdogJob?.cancel()
            watchdogJob = null
            session.setScanning(false, SystemClock.elapsedRealtime())
            publishLocked()
        }
        try {
            if (callback != null) currentScanner?.stopScan(callback)
        } catch (_: SecurityException) {
            // Runtime Bluetooth permission can be revoked while a scan is active.
        } catch (error: RuntimeException) {
            Log.w("EchoGuardBLE", "Native scan already unavailable", error)
        }
    }

    private fun startWatchdogLocked(generation: Long) {
        watchdogJob?.cancel()
        watchdogJob = engineScope.launch {
            while (true) {
                delay(100L)
                synchronized(lock) {
                    if (generation != scanGeneration) return@launch
                    val now = SystemClock.elapsedRealtime()
                    if (deviceMap.entries.removeAll { now - it.value.lastSeenMs >= DISCOVERY_TIMEOUT_MS }) {
                        publishDiscoveryLocked()
                    }
                    publishLocked(now)
                }
            }
        }
    }

    private fun publishLocked(nowMs: Long = SystemClock.elapsedRealtime()) {
        _monitoringState.value = session.snapshot(nowMs)
    }

    private fun publishDiscoveryLocked() {
        _discoveredDevices.value = deviceMap.values.sortedWith(
            compareBy<DiscoveredBleDevice> { it.pairId ?: Int.MAX_VALUE }.thenBy { it.name },
        )
    }

    companion object {
        private const val DISCOVERY_TIMEOUT_MS = 3_000L
    }
}
