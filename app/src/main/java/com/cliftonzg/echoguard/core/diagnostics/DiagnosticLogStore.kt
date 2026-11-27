package com.cliftonzg.echoguard.core.diagnostics

import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import javax.inject.Inject
import javax.inject.Singleton

enum class BleLogOutcome { ACCEPTED, DROPPED, PARSE_ERROR, EVENT }

data class BleLogEntry(
    val id: Long,
    val timestampMs: Long,
    val address: String,
    val rssi: Int?,
    val rawHex: String,
    val outcome: BleLogOutcome,
    val detail: String,
)

data class BleDiagnosticStats(
    val totalFrames: Long = 0L,
    val acceptedFrames: Long = 0L,
    val droppedFrames: Long = 0L,
    val parseErrors: Long = 0L,
)

sealed interface UsbSerialStatus {
    data object Disconnected : UsbSerialStatus
    data class PermissionRequired(val deviceName: String) : UsbSerialStatus
    data class Connected(val deviceName: String) : UsbSerialStatus
    data class Error(val message: String) : UsbSerialStatus
}

data class SerialLogEntry(
    val id: Long,
    val timestampMs: Long,
    val line: String,
    val isOutgoing: Boolean = false,
)

@Singleton
class DiagnosticLogStore @Inject constructor() {
    private val lock = Any()
    private val bleBuffer = ArrayDeque<BleLogEntry>(MAX_BLE_ENTRIES)
    private val serialBuffer = ArrayDeque<SerialLogEntry>(MAX_SERIAL_ENTRIES)
    private var nextBleId = 1L
    private var nextSerialId = 1L

    private val _bleLogs = MutableStateFlow<List<BleLogEntry>>(emptyList())
    val bleLogs: StateFlow<List<BleLogEntry>> = _bleLogs.asStateFlow()

    private val _bleStats = MutableStateFlow(BleDiagnosticStats())
    val bleStats: StateFlow<BleDiagnosticStats> = _bleStats.asStateFlow()

    private val _serialLogs = MutableStateFlow<List<SerialLogEntry>>(emptyList())
    val serialLogs: StateFlow<List<SerialLogEntry>> = _serialLogs.asStateFlow()

    private val _usbStatus = MutableStateFlow<UsbSerialStatus>(UsbSerialStatus.Disconnected)
    val usbStatus: StateFlow<UsbSerialStatus> = _usbStatus.asStateFlow()

    private val _isBlePaused = MutableStateFlow(false)
    val isBlePaused: StateFlow<Boolean> = _isBlePaused.asStateFlow()

    fun setBlePaused(paused: Boolean) {
        synchronized(lock) {
            _isBlePaused.value = paused
        }
    }

    fun logBle(
        address: String,
        rssi: Int?,
        rawHex: String,
        outcome: BleLogOutcome,
        detail: String,
        timestampMs: Long = System.currentTimeMillis(),
    ) {
        synchronized(lock) {
            val currentStats = _bleStats.value
            _bleStats.value = when (outcome) {
                BleLogOutcome.ACCEPTED -> currentStats.copy(
                    totalFrames = currentStats.totalFrames + 1,
                    acceptedFrames = currentStats.acceptedFrames + 1,
                )
                BleLogOutcome.DROPPED -> currentStats.copy(
                    totalFrames = currentStats.totalFrames + 1,
                    droppedFrames = currentStats.droppedFrames + 1,
                )
                BleLogOutcome.PARSE_ERROR -> currentStats.copy(
                    totalFrames = currentStats.totalFrames + 1,
                    parseErrors = currentStats.parseErrors + 1,
                )
                BleLogOutcome.EVENT -> currentStats
            }

            if (!_isBlePaused.value) {
                if (bleBuffer.size >= MAX_BLE_ENTRIES) {
                    bleBuffer.removeFirst()
                }
                bleBuffer.addLast(
                    BleLogEntry(
                        id = nextBleId++,
                        timestampMs = timestampMs,
                        address = address,
                        rssi = rssi,
                        rawHex = rawHex,
                        outcome = outcome,
                        detail = detail,
                    ),
                )
                _bleLogs.value = bleBuffer.toList()
            }
        }
        runCatching {
            val msg = "[${outcome.name}] addr=${address.ifEmpty { "-" }} rssi=${rssi ?: "-"} $detail" +
                if (rawHex.isNotEmpty()) " raw=$rawHex" else ""
            when (outcome) {
                BleLogOutcome.ACCEPTED -> Log.d(TAG_BLE, msg)
                BleLogOutcome.EVENT -> Log.i(TAG_BLE, msg)
                BleLogOutcome.DROPPED, BleLogOutcome.PARSE_ERROR -> Log.w(TAG_BLE, msg)
            }
        }
    }

    fun logBleEvent(detail: String, timestampMs: Long = System.currentTimeMillis()) {
        logBle(
            address = "",
            rssi = null,
            rawHex = "",
            outcome = BleLogOutcome.EVENT,
            detail = detail,
            timestampMs = timestampMs,
        )
    }

    fun appendSerialLine(
        line: String,
        isOutgoing: Boolean = false,
        timestampMs: Long = System.currentTimeMillis(),
    ) {
        synchronized(lock) {
            if (serialBuffer.size >= MAX_SERIAL_ENTRIES) {
                serialBuffer.removeFirst()
            }
            serialBuffer.addLast(
                SerialLogEntry(
                    id = nextSerialId++,
                    timestampMs = timestampMs,
                    line = line,
                    isOutgoing = isOutgoing,
                ),
            )
            _serialLogs.value = serialBuffer.toList()
        }
        runCatching {
            Log.d(TAG_SERIAL, if (isOutgoing) "TX: $line" else "RX: $line")
        }
    }

    fun setUsbStatus(status: UsbSerialStatus) {
        synchronized(lock) {
            _usbStatus.value = status
        }
    }

    fun clearBleLogs() {
        synchronized(lock) {
            bleBuffer.clear()
            _bleLogs.value = emptyList()
            _bleStats.value = BleDiagnosticStats()
        }
    }

    fun clearSerialLogs() {
        synchronized(lock) {
            serialBuffer.clear()
            _serialLogs.value = emptyList()
        }
    }

    companion object {
        const val TAG_BLE = "EchoGuardBLE"
        const val TAG_SERIAL = "EchoGuardSerial"
        const val MAX_BLE_ENTRIES = 200
        const val MAX_SERIAL_ENTRIES = 400
    }
}
