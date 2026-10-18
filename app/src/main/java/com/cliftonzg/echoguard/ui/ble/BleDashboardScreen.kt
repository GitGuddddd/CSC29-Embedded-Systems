package com.cliftonzg.echoguard.ui.ble

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import com.cliftonzg.echoguard.core.ble.model.ThreatLevel

@Composable
fun BleDashboardScreen(
    viewModel: BleViewModel,
    hasPermissions: Boolean,
    onRequestPermissions: () -> Unit,
) {
    val packet by viewModel.latestPacket.collectAsState()
    val canePacket by viewModel.canePacket.collectAsState()
    val glassesPacket by viewModel.glassesPacket.collectAsState()
    val isScanning by viewModel.isScanning.collectAsState()
    val linkState by viewModel.linkState.collectAsState()
    val discoveredDevices by viewModel.discoveredDevices.collectAsState()
    val selectedDevice by viewModel.selectedDevice.collectAsState()

    BleDashboardContent(
        packet = packet,
        canePacket = canePacket,
        glassesPacket = glassesPacket,
        isScanning = isScanning,
        linkState = linkState,
        hasPermissions = hasPermissions,
        discoveredDevices = discoveredDevices,
        selectedDevice = selectedDevice,
        onStartScan = { viewModel.startScan() },
        onStopScan = { viewModel.stopScan() },
        onSelectDevice = { viewModel.selectDevice(it) },
        onDisconnectDevice = { viewModel.disconnectSelectedDevice() },
        onRequestPermissions = onRequestPermissions,
    )
}

@Composable
fun BleDashboardContent(
    packet: SystemPacket?,
    canePacket: SystemPacket?,
    glassesPacket: SystemPacket?,
    isScanning: Boolean,
    linkState: LinkState,
    hasPermissions: Boolean,
    discoveredDevices: List<DiscoveredBleDevice>,
    selectedDevice: DiscoveredBleDevice?,
    onStartScan: () -> Unit,
    onStopScan: () -> Unit,
    onSelectDevice: (DiscoveredBleDevice) -> Unit,
    onDisconnectDevice: () -> Unit,
    onRequestPermissions: () -> Unit,
) {
    val cardColor = when {
        packet?.isFallAlert == true -> Color(0xFFD32F2F) // RED DANGER
        packet?.isRadarFault == true -> Color(0xFFD32F2F) // RED DANGER FOR SENSOR FAULT / OBSTRUCTION!
        linkState == LinkState.LINK_LOSS -> Color(0xFFD32F2F) // RED DANGER FOR LINK LOSS!
        linkState == LinkState.DISCONNECTED -> Color(0xFF37474F)
        packet?.threatLevel == ThreatLevel.CRITICAL -> Color(0xFFC62828)
        packet?.threatLevel == ThreatLevel.WARNING -> Color(0xFFE65100)
        packet?.threatLevel == ThreatLevel.SAFE -> Color(0xFF1B5E20)
        else -> Color(0xFF37474F)
    }

    Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
        Column(modifier = Modifier.padding(24.dp)) {
            Text("EchoGuard", style = MaterialTheme.typography.headlineMedium)
            Text("Safety-Critical Mobility Shield", style = MaterialTheme.typography.bodySmall)

            Spacer(Modifier.height(16.dp))

            if (!hasPermissions) {
                Card(
                    modifier = Modifier.fillMaxWidth(),
                    colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.errorContainer),
                ) {
                    Column(modifier = Modifier.padding(16.dp)) {
                        Text(
                            text = "Bluetooth & Notification Permissions Required",
                            style = MaterialTheme.typography.titleMedium,
                            color = MaterialTheme.colorScheme.onErrorContainer,
                        )
                        Spacer(Modifier.height(8.dp))
                        Button(onClick = onRequestPermissions) {
                            Text("Grant Permissions")
                        }
                    }
                }
                Spacer(Modifier.height(16.dp))
            }

            // Selected Device Card
            if (selectedDevice != null) {
                Card(
                    modifier = Modifier.fillMaxWidth(),
                    colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer),
                ) {
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(16.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Column(modifier = Modifier.weight(1f)) {
                            Text(
                                text = "Paired Device: ${selectedDevice.name}",
                                style = MaterialTheme.typography.titleMedium,
                                color = MaterialTheme.colorScheme.onPrimaryContainer,
                            )
                            Text(
                                text = "Address: ${selectedDevice.address} | RSSI: ${selectedDevice.rssi} dBm",
                                style = MaterialTheme.typography.bodySmall,
                                color = MaterialTheme.colorScheme.onPrimaryContainer.copy(alpha = 0.8f),
                            )
                        }
                        OutlinedButton(onClick = onDisconnectDevice) {
                            Text("Unpair")
                        }
                    }
                }
                Spacer(Modifier.height(16.dp))
            } else {
                Card(
                    modifier = Modifier.fillMaxWidth(),
                    colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
                ) {
                    Column(modifier = Modifier.padding(16.dp)) {
                        Text(
                            text = "No Device Paired",
                            style = MaterialTheme.typography.titleMedium,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                        Spacer(Modifier.height(4.dp))
                        Text(
                            text = if (isScanning) "Scanning for nearby EchoGuard devices... Select your device below to pair." else "Press 'Start Scanning' below to discover available EchoGuard nodes.",
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.8f),
                        )
                    }
                }
                Spacer(Modifier.height(16.dp))
            }

            // Live Threat Dashboard
            Card(
                modifier = Modifier.fillMaxWidth(),
                colors = CardDefaults.cardColors(containerColor = cardColor),
            ) {
                Column(modifier = Modifier.padding(20.dp)) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(
                            text = when {
                                packet?.isFallAlert == true -> "🚨 FALL ALERT DETECTED"
                                packet?.isRadarFault == true -> "🚨 DANGER: SENSOR FAULT / OBSTRUCTED"
                                linkState == LinkState.LINK_LOSS -> "🚨 DANGER: LINK LOSS (>500ms)"
                                linkState == LinkState.DISCONNECTED -> "LINK DISCONNECTED"
                                packet?.threatLevel == ThreatLevel.CRITICAL -> "🚨 CRITICAL PROXIMITY"
                                packet?.threatLevel == ThreatLevel.WARNING -> "⚠️ OBSTACLE WARNING"
                                else -> "🛡️ SHIELD ACTIVE"
                            },
                            style = MaterialTheme.typography.titleMedium,
                            color = Color.White,
                            fontWeight = FontWeight.Bold
                        )
                        Text(
                            text = if (isScanning) "SCANNING" else "STOPPED",
                            style = MaterialTheme.typography.labelMedium,
                            color = Color.White.copy(alpha = 0.8f),
                        )
                    }

                    packet?.let { p ->
                        val activeGlasses = if (p.deviceId == 0x02) p else glassesPacket

                        Spacer(Modifier.height(8.dp))
                        Text(
                            text = when (p.deviceId) {
                                0x01 -> "🦯 Smart Cane Node (Ground ToF)"
                                0x02 -> "🕶️ Smart Glasses Node (mmWave Radar)"
                                else -> "EchoGuard Node (0x${p.deviceId.toString(16)})"
                            },
                            style = MaterialTheme.typography.labelLarge,
                            color = Color.White.copy(alpha = 0.9f),
                            fontWeight = FontWeight.SemiBold
                        )

                        Spacer(Modifier.height(8.dp))
                        Text(
                            text = if (p.isRadarFault && p.radarDistMm == 0) "0 mm (OBSTRUCTED)" else "${p.radarDistMm} mm",
                            style = MaterialTheme.typography.displayMedium,
                            color = Color.White,
                        )

                        val progress = (p.radarDistMm / 2000f).coerceIn(0f, 1f)
                        Spacer(Modifier.height(6.dp))
                        LinearProgressIndicator(
                            progress = progress,
                            modifier = Modifier.fillMaxWidth().height(6.dp),
                            color = Color.White,
                            trackColor = Color.White.copy(alpha = 0.3f),
                        )

                        Spacer(Modifier.height(10.dp))
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                        ) {
                            Text(
                                text = "Seq: ${p.seqNum}",
                                color = Color.White.copy(alpha = 0.8f),
                                style = MaterialTheme.typography.bodyMedium,
                            )
                            if (activeGlasses != null) {
                                Text(
                                    text = "Speed: ${activeGlasses.closingSpeedCms} cm/s",
                                    color = Color.White.copy(alpha = 0.8f),
                                    style = MaterialTheme.typography.bodyMedium,
                                )
                                Text(
                                    text = "TTC: ${activeGlasses.ttcMs} ms",
                                    color = Color.White.copy(alpha = 0.8f),
                                    style = MaterialTheme.typography.bodyMedium,
                                )
                            } else if (p.deviceId == 0x01) {
                                Text(
                                    text = "Ground ToF Sensor",
                                    color = Color.White.copy(alpha = 0.8f),
                                    style = MaterialTheme.typography.bodyMedium,
                                )
                            }
                        }

                        if (p.deviceId == 0x01 && activeGlasses == null) {
                            Spacer(Modifier.height(8.dp))
                            Text(
                                text = "💡 Note: Smart Cane monitors ground drop-offs. Pair/select Smart Glasses Node (devId=0x02) for radar Speed & Time-To-Collision (TTC).",
                                style = MaterialTheme.typography.bodySmall,
                                color = Color.White.copy(alpha = 0.85f),
                            )
                        }

                        if (p.isRadarFault) {
                            Spacer(Modifier.height(12.dp))
                            Box(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .background(Color.Black.copy(alpha = 0.3f), RoundedCornerShape(8.dp))
                                    .padding(12.dp)
                            ) {
                                Column {
                                    Text(
                                        text = "🚨 DANGER: SENSOR OBSTRUCTED / FAULT",
                                        style = MaterialTheme.typography.titleSmall,
                                        color = Color.White,
                                        fontWeight = FontWeight.Bold
                                    )
                                    Spacer(Modifier.height(4.dp))
                                    Text(
                                        text = "The optical lens or radar aperture is covered or offline. Inspect and clean the sensor lens immediately!",
                                        style = MaterialTheme.typography.bodySmall,
                                        color = Color.White.copy(alpha = 0.9f)
                                    )
                                }
                            }
                        }
                    }
                }
            }

            Spacer(Modifier.height(16.dp))

            // Scan / Stop Buttons
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                Button(
                    onClick = onStartScan,
                    modifier = Modifier.weight(1f),
                    enabled = !isScanning,
                ) {
                    Text("Start Scanning")
                }
                OutlinedButton(
                    onClick = onStopScan,
                    modifier = Modifier.weight(1f),
                    enabled = isScanning,
                ) {
                    Text("Stop Scanning")
                }
            }

            Spacer(Modifier.height(16.dp))

            // Discovered Devices List
            Text("Discovered Devices (${discoveredDevices.size}):", style = MaterialTheme.typography.titleSmall)
            Spacer(Modifier.height(8.dp))

            LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                items(discoveredDevices) { device ->
                    Card(
                        modifier = Modifier.fillMaxWidth(),
                        colors = CardDefaults.cardColors(
                            containerColor = if (device.isEchoGuardNode) MaterialTheme.colorScheme.secondaryContainer else MaterialTheme.colorScheme.surfaceVariant
                        )
                    ) {
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(12.dp),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Column(modifier = Modifier.weight(1f)) {
                                Text(
                                    text = if (device.isEchoGuardNode) "🛡️ ${device.name}" else device.name,
                                    style = MaterialTheme.typography.titleSmall,
                                    fontWeight = if (device.isEchoGuardNode) FontWeight.Bold else FontWeight.Normal
                                )
                                Text(
                                    text = "MAC: ${device.address} | RSSI: ${device.rssi} dBm",
                                    style = MaterialTheme.typography.bodySmall,
                                    color = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.7f)
                                )
                            }
                            Button(
                                onClick = { onSelectDevice(device) },
                                enabled = selectedDevice?.address != device.address
                            ) {
                                Text(if (selectedDevice?.address == device.address) "Paired" else "Select")
                            }
                        }
                    }
                }
            }
        }
    }
}
