package com.cliftonzg.echoguard.core.ble

import android.R
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.bluetooth.BluetoothAdapter
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.VibrationEffect
import android.os.Vibrator
import androidx.core.app.NotificationCompat
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.ThreatLevel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import javax.inject.Inject

@AndroidEntryPoint
class BleSafetyService : Service() {
    @Inject lateinit var scannerEngine: BleScannerEngine
    @Inject lateinit var vibrator: Vibrator

    private val serviceScope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    private val bluetoothReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action == BluetoothAdapter.ACTION_STATE_CHANGED) {
                val state = intent.getIntExtra(
                    BluetoothAdapter.EXTRA_STATE,
                    BluetoothAdapter.ERROR,
                )
                if (state == BluetoothAdapter.STATE_ON) {
                    scannerEngine.startScanning()
                }
            }
        }
    }

    override fun onCreate() {
        super.onCreate()
        val filter = IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED)
        registerReceiver(bluetoothReceiver, filter)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        createNotificationChannel()
        val notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("EchoGuard: Head-Zone Shield")
            .setContentText("Monitoring for head-height hazards...")
            .setSmallIcon(R.drawable.ic_dialog_info)
            .setOngoing(true)
            .build()

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(NOTIFICATION_ID, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
        } else {
            startForeground(NOTIFICATION_ID, notification)
        }
        scannerEngine.startScanning()

        serviceScope.launch {
            scannerEngine.packets.collect { packet ->
                if (packet == null) return@collect
                when {
                    packet.isFallAlert -> triggerFallHaptic()
                    packet.isRadarFault -> triggerFaultHaptic()
                    packet.threatLevel == ThreatLevel.CRITICAL -> triggerHaptic(durationMs = 250)
                    packet.threatLevel == ThreatLevel.WARNING -> triggerHaptic(durationMs = 120)
                }
            }
        }

        serviceScope.launch {
            var previousState = LinkState.DISCONNECTED
            scannerEngine.linkState.collect { state ->
                if ((state == LinkState.LINK_LOSS) && (previousState == LinkState.ACTIVE)) {
                    triggerLinkLossHaptic()
                }
                previousState = state
            }
        }

        return START_STICKY
    }

    private fun triggerHaptic(durationMs: Long = 150) {
        try {
            vibrator.vibrate(VibrationEffect.createOneShot(durationMs, VibrationEffect.DEFAULT_AMPLITUDE))
        } catch (_: SecurityException) {
            // Permission missing
        }
    }

    private fun triggerFallHaptic() {
        try {
            val timings = longArrayOf(0, 200, 100, 200, 100, 300)
            val amplitudes = intArrayOf(0, 255, 0, 255, 0, 255)
            vibrator.vibrate(VibrationEffect.createWaveform(timings, amplitudes, -1))
        } catch (_: SecurityException) {
            // Permission missing
        }
    }

    private fun triggerFaultHaptic() {
        try {
            val timings = longArrayOf(0, 300, 100, 300, 100, 300)
            val amplitudes = intArrayOf(0, 255, 0, 255, 0, 255)
            vibrator.vibrate(VibrationEffect.createWaveform(timings, amplitudes, -1))
        } catch (_: SecurityException) {
            // Permission missing
        }
    }

    private fun triggerLinkLossHaptic() {
        try {
            val timings = longArrayOf(0, 200, 150, 200, 150, 200)
            val amplitudes = intArrayOf(0, 200, 0, 200, 0, 200)
            vibrator.vibrate(VibrationEffect.createWaveform(timings, amplitudes, -1))
        } catch (_: SecurityException) {
            // Permission missing
        }
    }

    private fun createNotificationChannel() {
        val channel = NotificationChannel(CHANNEL_ID, "Safety Alerts", NotificationManager.IMPORTANCE_HIGH)
        getSystemService(NotificationManager::class.java)?.createNotificationChannel(channel)
    }

    override fun onDestroy() {
        try {
            unregisterReceiver(bluetoothReceiver)
        } catch (_: IllegalArgumentException) {
            // Receiver not registered
        }
        scannerEngine.stopScanning()
        serviceScope.cancel()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?) = null

    companion object {
        private const val NOTIFICATION_ID = 202
        private const val CHANNEL_ID = "safety_link_channel"
    }
}
