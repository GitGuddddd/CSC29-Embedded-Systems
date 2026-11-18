package com.cliftonzg.echoguard.core.usb

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbInterface
import android.hardware.usb.UsbManager
import android.os.Build
import androidx.core.content.ContextCompat
import com.cliftonzg.echoguard.core.diagnostics.DiagnosticLogStore
import com.cliftonzg.echoguard.core.diagnostics.UsbSerialStatus
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import javax.inject.Inject
import javax.inject.Singleton

interface UsbSerialController {
    fun connectOrRequestPermission()
    fun disconnect()
    fun sendCommand(text: String)
}

@Singleton
class UsbSerialManager @Inject constructor(
    @ApplicationContext private val context: Context,
    private val logStore: DiagnosticLogStore,
) : UsbSerialController {

    private val usbManager: UsbManager? = context.getSystemService(Context.USB_SERVICE) as? UsbManager
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private val lock = Any()

    private var activeDevice: UsbDevice? = null
    private var activeConnection: UsbDeviceConnection? = null
    private var commInterface: UsbInterface? = null
    private var dataInterface: UsbInterface? = null
    private var inEndpoint: UsbEndpoint? = null
    private var outEndpoint: UsbEndpoint? = null
    private var readJob: Job? = null

    private val actionUsbPermission = "${context.packageName}.USB_PERMISSION"

    private val usbReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context?, intent: Intent?) {
            val action = intent?.action ?: return
            val device = extractUsbDevice(intent)
            when (action) {
                actionUsbPermission -> {
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    if (granted && device != null) {
                        openDevice(device)
                    } else {
                        logStore.setUsbStatus(UsbSerialStatus.Error("USB permission denied"))
                        logStore.appendSerialLine("[USB] Permission denied")
                    }
                }
                UsbManager.ACTION_USB_DEVICE_ATTACHED -> {
                    if (device != null && isMatchingDevice(device)) {
                        logStore.appendSerialLine("[USB] Device attached: ${deviceLabel(device)}")
                    }
                }
                UsbManager.ACTION_USB_DEVICE_DETACHED -> {
                    val current = synchronized(lock) { activeDevice }
                    if (device != null && current != null && device.deviceId == current.deviceId) {
                        logStore.appendSerialLine("[USB] Device detached: ${deviceLabel(device)}")
                        disconnect()
                    }
                }
            }
        }
    }

    init {
        val filter = IntentFilter().apply {
            addAction(actionUsbPermission)
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
        }
        ContextCompat.registerReceiver(
            context,
            usbReceiver,
            filter,
            ContextCompat.RECEIVER_NOT_EXPORTED,
        )
    }

    override fun connectOrRequestPermission() {
        val manager = usbManager ?: run {
            logStore.setUsbStatus(UsbSerialStatus.Error("USB Host service unavailable"))
            return
        }
        val device = findMatchingDevice(manager)
        if (device == null) {
            logStore.setUsbStatus(
                UsbSerialStatus.Error("No Pico USB serial device found (plug in via USB-C OTG)"),
            )
            return
        }
        if (!manager.hasPermission(device)) {
            val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_MUTABLE
            } else {
                PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
            }
            val permissionIntent = Intent(actionUsbPermission).apply {
                setPackage(context.packageName)
            }
            val pendingIntent = PendingIntent.getBroadcast(context, 0, permissionIntent, flags)
            logStore.setUsbStatus(UsbSerialStatus.PermissionRequired(deviceLabel(device)))
            manager.requestPermission(device, pendingIntent)
            return
        }
        openDevice(device)
    }

    override fun disconnect() {
        val jobToCancel: Job?
        val connToClose: UsbDeviceConnection?
        val commToRelease: UsbInterface?
        val dataToRelease: UsbInterface?
        synchronized(lock) {
            jobToCancel = readJob
            readJob = null
            connToClose = activeConnection
            activeConnection = null
            commToRelease = commInterface
            commInterface = null
            dataToRelease = dataInterface
            dataInterface = null
            inEndpoint = null
            outEndpoint = null
            activeDevice = null
        }
        jobToCancel?.cancel()
        if (connToClose != null) {
            try {
                commToRelease?.let { connToClose.releaseInterface(it) }
                dataToRelease?.let { connToClose.releaseInterface(it) }
                connToClose.close()
            } catch (_: RuntimeException) {
            }
        }
        logStore.setUsbStatus(UsbSerialStatus.Disconnected)
    }

    override fun sendCommand(text: String) {
        val (conn, ep) = synchronized(lock) { activeConnection to outEndpoint }
        if (conn == null || ep == null) {
            logStore.appendSerialLine("[USB] Not connected; cannot send '$text'")
            return
        }
        val bytes = text.toByteArray(Charsets.UTF_8)
        logStore.appendSerialLine("> $text", isOutgoing = true)
        scope.launch {
            val written = conn.bulkTransfer(ep, bytes, bytes.size, WRITE_TIMEOUT_MS)
            if (written < 0) {
                logStore.appendSerialLine("[USB] Failed to write command '$text'")
            }
        }
    }

    private fun openDevice(device: UsbDevice) {
        val manager = usbManager ?: return
        disconnect()

        val interfaces = resolveCdcInterfaces(device)
        if (interfaces == null) {
            logStore.setUsbStatus(UsbSerialStatus.Error("No CDC-ACM bulk endpoints found on ${deviceLabel(device)}"))
            return
        }
        val (commIf, dataIf, bulkIn, bulkOut) = interfaces
        val connection = manager.openDevice(device)
        if (connection == null) {
            logStore.setUsbStatus(UsbSerialStatus.Error("Unable to open ${deviceLabel(device)}"))
            return
        }
        if (commIf != null) {
            connection.claimInterface(commIf, true)
        }
        if (!connection.claimInterface(dataIf, true)) {
            commIf?.let { connection.releaseInterface(it) }
            connection.close()
            logStore.setUsbStatus(UsbSerialStatus.Error("Failed to claim CDC data interface"))
            return
        }

        val controlIfIndex = commIf?.id ?: 0
        // CDC SET_CONTROL_LINE_STATE: DTR=1 (bit 0), RTS=1 (bit 1) -> 0x03
        connection.controlTransfer(
            CDC_REQ_TYPE_HOST_TO_INTERFACE,
            CDC_SET_CONTROL_LINE_STATE,
            0x03,
            controlIfIndex,
            null,
            0,
            CONTROL_TIMEOUT_MS,
        )
        // CDC SET_LINE_CODING: 115200 bps, 1 stop bit, no parity, 8 data bits
        val lineCoding = byteArrayOf(
            0x00, 0xC2.toByte(), 0x01, 0x00,
            0x00,
            0x00,
            0x08,
        )
        connection.controlTransfer(
            CDC_REQ_TYPE_HOST_TO_INTERFACE,
            CDC_SET_LINE_CODING,
            0,
            controlIfIndex,
            lineCoding,
            lineCoding.size,
            CONTROL_TIMEOUT_MS,
        )

        val label = deviceLabel(device)
        synchronized(lock) {
            activeDevice = device
            activeConnection = connection
            commInterface = commIf
            dataInterface = dataIf
            inEndpoint = bulkIn
            outEndpoint = bulkOut
        }
        logStore.setUsbStatus(UsbSerialStatus.Connected(label))
        logStore.appendSerialLine("[USB] Connected to $label (115200 8N1)")

        val job = scope.launch {
            val buffer = ByteArray(maxOf(bulkIn.maxPacketSize, READ_BUFFER_MIN_BYTES))
            val lineBuffer = StringBuilder()
            while (isActive) {
                val stillActive = synchronized(lock) { activeConnection === connection }
                if (!stillActive) break
                val len = connection.bulkTransfer(bulkIn, buffer, buffer.size, READ_TIMEOUT_MS)
                if (len > 0) {
                    val chunk = String(buffer, 0, len, Charsets.UTF_8)
                    for (ch in chunk) {
                        if (ch == '\r' || ch == '\n') {
                            if (lineBuffer.isNotEmpty()) {
                                logStore.appendSerialLine(lineBuffer.toString())
                                lineBuffer.clear()
                            }
                        } else if (lineBuffer.length < MAX_LINE_LENGTH) {
                            lineBuffer.append(ch)
                        }
                    }
                } else if (len < 0) {
                    val present = manager.deviceList.values.any { it.deviceId == device.deviceId }
                    if (!present) {
                        logStore.appendSerialLine("[USB] Connection lost")
                        disconnect()
                        break
                    }
                }
            }
        }
        synchronized(lock) {
            if (activeConnection === connection) {
                readJob = job
            } else {
                job.cancel()
            }
        }
    }

    private fun findMatchingDevice(manager: UsbManager): UsbDevice? {
        val devices = manager.deviceList.values
        return devices.firstOrNull { it.vendorId == PICO_VID && it.productId == PICO_CDC_PID }
            ?: devices.firstOrNull { isMatchingDevice(it) }
    }

    private fun isMatchingDevice(device: UsbDevice): Boolean {
        if (device.vendorId == PICO_VID && device.productId == PICO_CDC_PID) return true
        var hasComm = false
        var hasData = false
        for (i in 0 until device.interfaceCount) {
            when (device.getInterface(i).interfaceClass) {
                UsbConstants.USB_CLASS_COMM -> hasComm = true
                UsbConstants.USB_CLASS_CDC_DATA -> hasData = true
            }
        }
        return hasComm && hasData
    }

    private data class CdcInterfaces(
        val commInterface: UsbInterface?,
        val dataInterface: UsbInterface,
        val inEndpoint: UsbEndpoint,
        val outEndpoint: UsbEndpoint,
    )

    private fun resolveCdcInterfaces(device: UsbDevice): CdcInterfaces? {
        var commIf: UsbInterface? = null
        var dataIf: UsbInterface? = null
        for (i in 0 until device.interfaceCount) {
            val iface = device.getInterface(i)
            when (iface.interfaceClass) {
                UsbConstants.USB_CLASS_COMM -> if (commIf == null) commIf = iface
                UsbConstants.USB_CLASS_CDC_DATA -> if (dataIf == null) dataIf = iface
            }
        }
        val candidates = listOfNotNull(dataIf) + (0 until device.interfaceCount).map { device.getInterface(it) }
        for (iface in candidates) {
            var bulkIn: UsbEndpoint? = null
            var bulkOut: UsbEndpoint? = null
            for (e in 0 until iface.endpointCount) {
                val ep = iface.getEndpoint(e)
                if (ep.type == UsbConstants.USB_ENDPOINT_XFER_BULK) {
                    if (ep.direction == UsbConstants.USB_DIR_IN && bulkIn == null) {
                        bulkIn = ep
                    } else if (ep.direction == UsbConstants.USB_DIR_OUT && bulkOut == null) {
                        bulkOut = ep
                    }
                }
            }
            if (bulkIn != null && bulkOut != null) {
                return CdcInterfaces(commIf, iface, bulkIn, bulkOut)
            }
        }
        return null
    }

    private fun deviceLabel(device: UsbDevice): String {
        val product = device.productName?.takeIf { it.isNotBlank() }
        return product ?: String.format("USB %04X:%04X (%s)", device.vendorId, device.productId, device.deviceName)
    }

    private fun extractUsbDevice(intent: Intent): UsbDevice? {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            intent.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
        } else {
            @Suppress("DEPRECATION")
            intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
        }
    }

    companion object {
        private const val PICO_VID = 0x2E8A
        private const val PICO_CDC_PID = 0x000A
        private const val CDC_REQ_TYPE_HOST_TO_INTERFACE = 0x21
        private const val CDC_SET_LINE_CODING = 0x20
        private const val CDC_SET_CONTROL_LINE_STATE = 0x22
        private const val CONTROL_TIMEOUT_MS = 1000
        private const val READ_TIMEOUT_MS = 250
        private const val WRITE_TIMEOUT_MS = 1000
        private const val READ_BUFFER_MIN_BYTES = 512
        private const val MAX_LINE_LENGTH = 4096
    }
}
