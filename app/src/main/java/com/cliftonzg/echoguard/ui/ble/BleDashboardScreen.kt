package com.cliftonzg.echoguard.ui.ble

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.StatusTone
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import com.cliftonzg.echoguard.core.ble.model.presentation

@Composable
fun BleDashboardScreen(
    viewModel: BleViewModel,
    hasPermissions: Boolean,
    onRequestPermissions: () -> Unit,
) {
    val state by viewModel.monitoringState.collectAsStateWithLifecycle()
    val devices by viewModel.discoveredDevices.collectAsStateWithLifecycle()
    val error by viewModel.scanError.collectAsStateWithLifecycle()
    BleDashboardContent(
        state = state,
        hasPermissions = hasPermissions,
        discoveredDevices = devices,
        scanError = error,
        onStartScan = { viewModel.startScan() },
        onStopScan = viewModel::stopScan,
        onSelectDevice = viewModel::selectDevice,
        onDisconnectDevice = viewModel::disconnectSelectedDevice,
        onRequestPermissions = onRequestPermissions,
    )
}

private fun toneColor(tone: StatusTone): Color = when (tone) {
    StatusTone.ALERT -> Color(0xFFC62828)
    StatusTone.CAUTION -> Color(0xFF805000)
    StatusTone.OK -> Color(0xFF1B5E20)
    StatusTone.NEUTRAL -> Color(0xFF37474F)
}

@Composable
fun BleDashboardContent(
    state: MonitoringState,
    hasPermissions: Boolean,
    discoveredDevices: List<DiscoveredBleDevice>,
    scanError: String?,
    onStartScan: () -> Unit,
    onStopScan: () -> Unit,
    onSelectDevice: (DiscoveredBleDevice) -> Unit,
    onDisconnectDevice: () -> Unit,
    onRequestPermissions: () -> Unit,
) {
    val summary = state.presentation()
    Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
        LazyColumn(
            modifier = Modifier.fillMaxSize().padding(horizontal = 20.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            item {
                Column(Modifier.padding(top = 24.dp, bottom = 4.dp)) {
                    Text("EchoGuard", style = MaterialTheme.typography.headlineMedium)
                    Text("Cane & glasses telemetry", style = MaterialTheme.typography.bodyMedium)
                }
            }
            if (!hasPermissions) {
                item {
                    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.errorContainer)) {
                        Column(Modifier.fillMaxWidth().padding(16.dp)) {
                            Text("Bluetooth and notification permissions required")
                            Spacer(Modifier.height(8.dp))
                            Button(onClick = onRequestPermissions) { Text("Grant permissions") }
                        }
                    }
                }
            }
            item {
                Card(colors = CardDefaults.cardColors(containerColor = toneColor(summary.tone))) {
                    Column(Modifier.fillMaxWidth().padding(20.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                        Text(
                            summary.title,
                            modifier = Modifier.testTag("summary_title"),
                            style = MaterialTheme.typography.headlineSmall,
                            color = Color.White,
                            fontWeight = FontWeight.Bold,
                        )
                        Text(summary.detail, color = Color.White, style = MaterialTheme.typography.bodyMedium)
                        Text(
                            if (state.isScanning) "SCANNING · fresh-heartbeat deadline 500 ms" else "SCANNING STOPPED",
                            color = Color.White.copy(alpha = 0.85f),
                            style = MaterialTheme.typography.labelMedium,
                        )
                    }
                }
            }
            item {
                state.selectedDevice?.let { device ->
                    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer)) {
                        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                            Text(
                                device.pairId?.let { "Monitoring pair #$it" } ?: "Monitoring selected node",
                                fontWeight = FontWeight.SemiBold,
                            )
                            Text("Selected: ${device.name}", style = MaterialTheme.typography.bodyMedium)
                            Text(device.address, style = MaterialTheme.typography.bodySmall)
                            OutlinedButton(onClick = onDisconnectDevice) { Text("Clear selection") }
                        }
                    }
                } ?: Text(
                    "Select a node below. Compact telemetry links the cane and glasses by pair ID; no Bluetooth bonding is required.",
                    style = MaterialTheme.typography.bodyMedium,
                )
            }
            items(state.nodes, key = { it.type }) { node -> NodeCard(node) }
            if (scanError != null) {
                item { Text(scanError, color = MaterialTheme.colorScheme.error) }
            }
            item {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                    Button(
                        onClick = onStartScan,
                        modifier = Modifier.weight(1f),
                        enabled = hasPermissions && !state.isScanning,
                    ) { Text("Start scanning") }
                    OutlinedButton(
                        onClick = onStopScan,
                        modifier = Modifier.weight(1f),
                        enabled = state.isScanning,
                    ) { Text("Stop scanning") }
                }
            }
            item {
                Text("Discovered nodes (${discoveredDevices.size})", style = MaterialTheme.typography.titleMedium)
            }
            items(discoveredDevices, key = { it.address }) { device ->
                Card {
                    Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text(device.name, style = MaterialTheme.typography.titleMedium)
                        Text(
                            "${device.pairId?.let { "Pair #$it · " } ?: "Legacy · "}${device.rssi} dBm",
                            style = MaterialTheme.typography.bodySmall,
                        )
                        Text(device.address, style = MaterialTheme.typography.bodySmall)
                        val selected = state.selectedDevice?.let {
                            it.address == device.address && it.pairId == device.pairId && it.nodeType == device.nodeType
                        } == true
                        Button(onClick = { onSelectDevice(device) }, enabled = !selected) {
                            Text(if (selected) "Selected" else "Select")
                        }
                    }
                }
            }
            item { Spacer(Modifier.height(16.dp)) }
        }
    }
}

@Composable
private fun NodeCard(node: NodeTelemetry) {
    val presentation = node.presentation()
    Card(colors = CardDefaults.cardColors(containerColor = toneColor(presentation.tone))) {
        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(node.type.label, color = Color.White, style = MaterialTheme.typography.titleLarge)
            Text(
                when (node.linkState) {
                    LinkState.ACTIVE -> "FRESH HEARTBEAT"
                    LinkState.LINK_LOSS -> "STALE · last report only"
                    LinkState.WAITING -> "NOT YET SEEN"
                    LinkState.DISCONNECTED -> "STOPPED · last report only"
                },
                color = Color.White.copy(alpha = 0.8f),
                style = MaterialTheme.typography.labelSmall,
            )
            Text(
                presentation.title,
                modifier = Modifier.testTag("node_${node.type}_status"),
                color = Color.White,
                fontWeight = FontWeight.Bold,
            )
            Text(presentation.detail, color = Color.White, style = MaterialTheme.typography.bodyMedium)
            node.packet?.let { packet ->
                val range = when {
                    !node.isFresh -> "Live range unavailable"
                    packet.isStandby -> "Range inactive · sensor sleeping"
                    packet.isSensorFault -> "Range unavailable · sensing fault"
                    packet.hasGripTelemetry && !packet.isCaneHeld -> "Ground sensing inactive · grip released"
                    packet.radarDistMm < 0 && node.type == NodeType.CANE -> "No valid ground range"
                    packet.radarDistMm < 0 -> "No target"
                    else -> "${packet.radarDistMm} mm"
                }
                Text(range, color = Color.White, style = MaterialTheme.typography.titleMedium)
                if (node.type == NodeType.GLASSES && node.isFresh && !packet.isStandby && !packet.isSensorFault) {
                    Text(
                        if (packet.ttcMs < 0) "TTC: not reported" else "TTC: ${packet.ttcMs} ms",
                        color = Color.White.copy(alpha = 0.9f),
                        style = MaterialTheme.typography.bodySmall,
                    )
                    if (packet.hasClosingSpeed) {
                        Text("Speed: ${packet.closingSpeedCms} cm/s", color = Color.White, style = MaterialTheme.typography.bodySmall)
                    }
                }
                if (packet.isSensorFault && packet.status != com.cliftonzg.echoguard.core.ble.model.TelemetryStatus.SENSOR_FAULT) {
                    Text("Sensor/data-path fault also reported", color = Color.White, style = MaterialTheme.typography.bodySmall)
                }
                Text(
                    "Heartbeat #${packet.seqNum}" + if (packet.compactFormat) " · Fall seq #${packet.rollingFallSeq}" else "",
                    color = Color.White.copy(alpha = 0.8f),
                    style = MaterialTheme.typography.labelSmall,
                )
            }
        }
    }
}

@Preview(name = "Paused incident", widthDp = 360, heightDp = 900, showBackground = true)
@Composable
private fun PausedPreview() = DashboardPreview(SystemPacket.FLAG_FALL_ALERT or SystemPacket.FLAG_ALARM_PAUSED or SystemPacket.FLAG_CANE_HELD)

@Preview(name = "Deliberate standby", widthDp = 360, heightDp = 900, showBackground = true)
@Composable
private fun StandbyPreview() = DashboardPreview(SystemPacket.FLAG_STANDBY or SystemPacket.FLAG_PARKED_BY_USER or SystemPacket.FLAG_CANE_HELD)

@Composable
private fun DashboardPreview(caneFlags: Int) {
    val device = DiscoveredBleDevice("EchoGuard Smart Cane", "DE:MO:00:00:00:01", -48, true, nodeType = NodeType.CANE, pairId = 1)
    fun packet(type: NodeType, flags: Int) = SystemPacket(
        magic = 0xAA, seqNum = 3, radarDistMm = -1, checksum = 0, deviceId = type.deviceId,
        stateFlags = flags, compactFormat = true, isAdvanced = true, pairId = 1, rollingFallSeq = 5,
    )
    val state = MonitoringState(device, true, listOf(
        NodeTelemetry(NodeType.CANE, device.address, packet(NodeType.CANE, caneFlags), LinkState.ACTIVE),
        NodeTelemetry(NodeType.GLASSES, "DE:MO:00:00:00:02", packet(NodeType.GLASSES, 0), LinkState.ACTIVE),
    ))
    MaterialTheme {
        BleDashboardContent(state, true, emptyList(), null, {}, {}, {}, {}, {})
    }
}
