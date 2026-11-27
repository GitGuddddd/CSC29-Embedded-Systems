package com.cliftonzg.echoguard

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.core.content.ContextCompat
import com.cliftonzg.echoguard.core.ble.BleSafetyService
import com.cliftonzg.echoguard.ui.ble.BleDashboardScreen
import com.cliftonzg.echoguard.ui.ble.BleViewModel
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : ComponentActivity() {

    private val viewModel: BleViewModel by viewModels()

    private val hasPermissionsState = mutableStateOf(value = false)

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { _ ->
        updatePermissionState()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        updatePermissionState()
        if (!hasPermissionsState.value) {
            requestRequiredPermissions()
        }

        setContent {
            val hasPermissions by hasPermissionsState
            BleDashboardScreen(
                viewModel = viewModel,
                hasPermissions = hasPermissions,
            ) { requestRequiredPermissions() }
        }
    }

    override fun onResume() {
        super.onResume()
        updatePermissionState()
    }

    private fun updatePermissionState() {
        val granted = hasRequiredPermissions()
        hasPermissionsState.value = granted
        if (granted) {
            startSafetyService()
        }
    }

    private fun startSafetyService() {
        val serviceIntent = Intent(this, BleSafetyService::class.java)
        startForegroundService(serviceIntent)
    }

    private fun hasRequiredPermissions(): Boolean {
        val permissions = getRequiredPermissions()
        return permissions.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun requestRequiredPermissions() {
        permissionLauncher.launch(getRequiredPermissions())
    }

    private fun getRequiredPermissions(): Array<String> {
        val list = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            list.add(Manifest.permission.BLUETOOTH_SCAN)
            list.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            list.add(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            list.add(Manifest.permission.POST_NOTIFICATIONS)
        }
        return list.toTypedArray()
    }
}
