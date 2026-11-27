package com.cliftonzg.echoguard.core.ble

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
import android.os.SystemClock
import android.os.VibrationEffect
import android.os.Vibrator
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import com.cliftonzg.echoguard.core.ble.model.TelemetryPresentation
import com.cliftonzg.echoguard.core.ble.model.presentation
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
    private val hapticPolicy = SafetyHapticPolicy()
    private var started = false
    private var lastPresentation: TelemetryPresentation? = null

    private val bluetoothReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action == BluetoothAdapter.ACTION_STATE_CHANGED) {
                val state = intent.getIntExtra(
                    BluetoothAdapter.EXTRA_STATE,
                    BluetoothAdapter.ERROR,
                )
                when (state) {
                    BluetoothAdapter.STATE_ON -> scannerEngine.onBluetoothStateChanged(true)
                    BluetoothAdapter.STATE_OFF, BluetoothAdapter.STATE_TURNING_OFF -> scannerEngine.onBluetoothStateChanged(false)
                }
            }
        }
    }

    override fun onCreate() {
        super.onCreate()
        val filter = IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED)
        ContextCompat.registerReceiver(this, bluetoothReceiver, filter, ContextCompat.RECEIVER_EXPORTED)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        createNotificationChannel()
        val notification = notificationFor(scannerEngine.monitoringState.value.presentation())

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(NOTIFICATION_ID, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
        } else {
            startForeground(NOTIFICATION_ID, notification)
        }
        // onResume/permission changes can start this service repeatedly. Collect only once.
        if (!started) {
            started = true
            scannerEngine.startScanning()
            serviceScope.launch {
                scannerEngine.monitoringState.collect { state ->
                    when (hapticPolicy.update(state, SystemClock.elapsedRealtime())) {
                        HapticCue.STOP -> cancelHaptic()
                        HapticCue.FALL -> triggerFallHaptic()
                        HapticCue.SENSOR_FAULT -> triggerFaultHaptic()
                        HapticCue.LINK_LOSS -> triggerLinkLossHaptic()
                        HapticCue.CRITICAL -> triggerHaptic(durationMs = 250L)
                        HapticCue.WARNING -> triggerHaptic(durationMs = 120L)
                        null -> Unit
                    }
                    val presentation = state.presentation()
                    if (presentation != lastPresentation) {
                        getSystemService(NotificationManager::class.java)?.notify(
                            NOTIFICATION_ID, notificationFor(presentation),
                        )
                        lastPresentation = presentation
                    }
                }
            }
        }

        return START_STICKY
    }

    private fun notificationFor(presentation: TelemetryPresentation) = NotificationCompat.Builder(this, CHANNEL_ID)
        .setContentTitle("EchoGuard: ${presentation.title}")
        .setContentText(presentation.detail)
        .setStyle(NotificationCompat.BigTextStyle().bigText(presentation.detail))
        .setSmallIcon(android.R.drawable.ic_dialog_info)
        .setOngoing(true)
        .setOnlyAlertOnce(true)
        .setSilent(true) // Explicit haptic policy owns alerts; notification updates must not buzz too.
        .build()

    private fun triggerHaptic(durationMs: Long = 150) {
        try {
            vibrator.vibrate(VibrationEffect.createOneShot(durationMs, VibrationEffect.DEFAULT_AMPLITUDE))
        } catch (_: SecurityException) {
            // Permission missing
        }
    }

    private fun cancelHaptic() {
        try {
            vibrator.cancel()
        } catch (_: SecurityException) {
            // Permission may have changed while the service was running.
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
            val timings = longArrayOf(0, 150, 200, 150)
            val amplitudes = intArrayOf(0, 180, 0, 180)
            vibrator.vibrate(VibrationEffect.createWaveform(timings, amplitudes, -1))
        } catch (_: SecurityException) {
            // Permission missing
        }
    }

    private fun triggerLinkLossHaptic() {
        try {
            val timings = longArrayOf(0, 250, 150, 250)
            val amplitudes = intArrayOf(0, 200, 0, 200)
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
        cancelHaptic()
        serviceScope.cancel()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?) = null

    companion object {
        private const val NOTIFICATION_ID = 202
        private const val CHANNEL_ID = "safety_link_channel"
    }
}
