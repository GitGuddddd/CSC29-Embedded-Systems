package com.cliftonzg.echoguard.ui.ble

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Divider
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Surface
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.PeerReceiptState
import com.cliftonzg.echoguard.core.ble.model.StatusTone
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import com.cliftonzg.echoguard.core.ble.model.audioRoutingPresentation
import com.cliftonzg.echoguard.core.ble.model.presentation
import com.cliftonzg.echoguard.core.diagnostics.BleDiagnosticStats
import com.cliftonzg.echoguard.core.diagnostics.BleLogEntry
import com.cliftonzg.echoguard.core.diagnostics.BleLogOutcome
import com.cliftonzg.echoguard.core.diagnostics.SerialLogEntry
import com.cliftonzg.echoguard.core.diagnostics.UsbSerialStatus
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter

@Composable
fun BleDashboardScreen(
    viewModel: BleViewModel,
    hasPermissions: Boolean,
    onRequestPermissions: () -> Unit,
) {
    val state by viewModel.monitoringState.collectAsStateWithLifecycle()
    val devices by viewModel.discoveredDevices.collectAsStateWithLifecycle()
    val error by viewModel.scanError.collectAsStateWithLifecycle()
    val isDevMenuOpen by viewModel.isDevMenuOpen.collectAsStateWithLifecycle()
    val bleLogs by viewModel.bleLogs.collectAsStateWithLifecycle()
    val bleStats by viewModel.bleStats.collectAsStateWithLifecycle()
    val isBlePaused by viewModel.isBlePaused.collectAsStateWithLifecycle()
    val serialLogs by viewModel.serialLogs.collectAsStateWithLifecycle()
    val usbStatus by viewModel.usbStatus.collectAsStateWithLifecycle()
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
        isDevMenuOpen = isDevMenuOpen,
        bleLogs = bleLogs,
        bleStats = bleStats,
        isBlePaused = isBlePaused,
        serialLogs = serialLogs,
        usbStatus = usbStatus,
        onOpenDevMenu = viewModel::openDevMenu,
        onCloseDevMenu = viewModel::closeDevMenu,
        onToggleBlePaused = viewModel::toggleBlePaused,
        onClearBleLogs = viewModel::clearBleLogs,
        onClearSerialLogs = viewModel::clearSerialLogs,
        onConnectUsb = viewModel::connectUsbSerial,
        onDisconnectUsb = viewModel::disconnectUsbSerial,
        onSendSerialCommand = viewModel::sendSerialCommand,
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
    isDevMenuOpen: Boolean = false,
    bleLogs: List<BleLogEntry> = emptyList(),
    bleStats: BleDiagnosticStats = BleDiagnosticStats(),
    isBlePaused: Boolean = false,
    serialLogs: List<SerialLogEntry> = emptyList(),
    usbStatus: UsbSerialStatus = UsbSerialStatus.Disconnected,
    onOpenDevMenu: () -> Unit = {},
    onCloseDevMenu: () -> Unit = {},
    onToggleBlePaused: () -> Unit = {},
    onClearBleLogs: () -> Unit = {},
    onClearSerialLogs: () -> Unit = {},
    onConnectUsb: () -> Unit = {},
    onDisconnectUsb: () -> Unit = {},
    onSendSerialCommand: (String) -> Unit = {},
) {
    val summary = state.presentation()
    var headerTapCount by remember { mutableIntStateOf(0) }
    var headerFirstTapMs by remember { mutableLongStateOf(0L) }

    Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
        LazyColumn(
            modifier = Modifier.fillMaxSize().padding(horizontal = 20.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            item {
                Column(
                    Modifier
                        .padding(top = 24.dp, bottom = 4.dp)
                        .clickable(
                            interactionSource = remember { MutableInteractionSource() },
                            indication = null,
                        ) {
                            val now = System.currentTimeMillis()
                            if (headerTapCount == 0 || now - headerFirstTapMs > 1_500L) {
                                headerTapCount = 1
                                headerFirstTapMs = now
                            } else {
                                headerTapCount++
                                if (headerTapCount >= 5) {
                                    headerTapCount = 0
                                    headerFirstTapMs = 0L
                                    onOpenDevMenu()
                                }
                            }
                        },
                ) {
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
                    val routing = state.audioRoutingPresentation()
                    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer)) {
                        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                            Text(
                                device.pairId?.let { "Monitoring pair #$it" } ?: "Monitoring selected node",
                                fontWeight = FontWeight.SemiBold,
                            )
                            Text("Selected: ${device.name}", style = MaterialTheme.typography.bodyMedium)
                            Text(device.address, style = MaterialTheme.typography.bodySmall)
                            Text(
                                routing.detail,
                                modifier = Modifier.testTag("audio_routing_status"),
                                style = MaterialTheme.typography.bodySmall,
                                fontWeight = FontWeight.Medium,
                            )
                            OutlinedButton(onClick = onDisconnectDevice) { Text("Clear selection") }
                        }
                    }
                } ?: Text(
                    "Select a node below. Compact telemetry links the cane and glasses by pair ID; no Bluetooth bonding is required.",
                    style = MaterialTheme.typography.bodyMedium,
                )
            }
            val caneNode = state.node(NodeType.CANE)
            val isFailover = caneNode?.isFresh == true && caneNode.packet?.isBuzzerFault == true
            if (isFailover) {
                item {
                    Card(colors = CardDefaults.cardColors(containerColor = toneColor(StatusTone.CAUTION))) {
                        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                            Text(
                                "Buzzer Failover Active",
                                modifier = Modifier.testTag("buzzer_failover_banner_title"),
                                color = Color.White,
                                style = MaterialTheme.typography.titleMedium,
                                fontWeight = FontWeight.Bold,
                            )
                            Text(
                                "Cane buzzer fault · Audio switched to glasses",
                                modifier = Modifier.testTag("buzzer_failover_banner_detail"),
                                color = Color.White,
                                style = MaterialTheme.typography.bodyMedium,
                            )
                        }
                    }
                }
            }
            items(state.nodes, key = { it.type }) { node -> NodeCard(node, state) }
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

        if (isDevMenuOpen) {
            DeveloperDiagnosticsDialog(
                bleLogs = bleLogs,
                bleStats = bleStats,
                isBlePaused = isBlePaused,
                serialLogs = serialLogs,
                usbStatus = usbStatus,
                onDismiss = onCloseDevMenu,
                onToggleBlePaused = onToggleBlePaused,
                onClearBleLogs = onClearBleLogs,
                onClearSerialLogs = onClearSerialLogs,
                onConnectUsb = onConnectUsb,
                onDisconnectUsb = onDisconnectUsb,
                onSendSerialCommand = onSendSerialCommand,
            )
        }
    }
}

private enum class BleLogFilter(val label: String) {
    ALL("All"),
    ACCEPTED("Accepted"),
    DROPPED_OR_ERR("Dropped/Err"),
}

private val logTimeFormatter: DateTimeFormatter =
    DateTimeFormatter.ofPattern("HH:mm:ss.SSS").withZone(ZoneId.systemDefault())

private fun formatLogTimestamp(timestampMs: Long): String =
    logTimeFormatter.format(Instant.ofEpochMilli(timestampMs))

@Composable
private fun DeveloperDiagnosticsDialog(
    bleLogs: List<BleLogEntry>,
    bleStats: BleDiagnosticStats,
    isBlePaused: Boolean,
    serialLogs: List<SerialLogEntry>,
    usbStatus: UsbSerialStatus,
    onDismiss: () -> Unit,
    onToggleBlePaused: () -> Unit,
    onClearBleLogs: () -> Unit,
    onClearSerialLogs: () -> Unit,
    onConnectUsb: () -> Unit,
    onDisconnectUsb: () -> Unit,
    onSendSerialCommand: (String) -> Unit,
) {
    var selectedTab by remember { mutableIntStateOf(0) }
    val tabs = listOf("Bluetooth Logs", "USB Serial")

    Dialog(
        onDismissRequest = onDismiss,
        properties = DialogProperties(usePlatformDefaultWidth = false),
    ) {
        Surface(
            modifier = Modifier
                .fillMaxWidth(0.96f)
                .fillMaxHeight(0.92f)
                .testTag("developer_diagnostics_dialog"),
            shape = RoundedCornerShape(16.dp),
            color = MaterialTheme.colorScheme.surface,
            tonalElevation = 6.dp,
        ) {
            Column(modifier = Modifier.fillMaxSize().padding(14.dp)) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Text(
                        "Developer Diagnostics",
                        style = MaterialTheme.typography.titleLarge,
                        fontWeight = FontWeight.Bold,
                    )
                    TextButton(onClick = onDismiss) { Text("Close") }
                }

                TabRow(selectedTabIndex = selectedTab) {
                    tabs.forEachIndexed { index, title ->
                        Tab(
                            selected = selectedTab == index,
                            onClick = { selectedTab = index },
                            text = { Text(title) },
                        )
                    }
                }

                Spacer(Modifier.height(10.dp))

                when (selectedTab) {
                    0 -> BluetoothLogsTab(
                        bleLogs = bleLogs,
                        bleStats = bleStats,
                        isBlePaused = isBlePaused,
                        onTogglePaused = onToggleBlePaused,
                        onClear = onClearBleLogs,
                    )
                    1 -> UsbSerialTab(
                        serialLogs = serialLogs,
                        usbStatus = usbStatus,
                        onConnect = onConnectUsb,
                        onDisconnect = onDisconnectUsb,
                        onClear = onClearSerialLogs,
                        onSendCommand = onSendSerialCommand,
                    )
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun BluetoothLogsTab(
    bleLogs: List<BleLogEntry>,
    bleStats: BleDiagnosticStats,
    isBlePaused: Boolean,
    onTogglePaused: () -> Unit,
    onClear: () -> Unit,
) {
    var filter by remember { mutableStateOf(BleLogFilter.ALL) }
    val filteredLogs = remember(bleLogs, filter) {
        when (filter) {
            BleLogFilter.ALL -> bleLogs
            BleLogFilter.ACCEPTED -> bleLogs.filter { it.outcome == BleLogOutcome.ACCEPTED }
            BleLogFilter.DROPPED_OR_ERR -> bleLogs.filter {
                it.outcome == BleLogOutcome.DROPPED || it.outcome == BleLogOutcome.PARSE_ERROR
            }
        }.asReversed()
    }

    Column(modifier = Modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(
            modifier = Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            StatPill("Total", bleStats.totalFrames.toString(), Color(0xFF37474F))
            StatPill("OK", bleStats.acceptedFrames.toString(), Color(0xFF1B5E20))
            StatPill("Drop", bleStats.droppedFrames.toString(), Color(0xFF805000))
            StatPill("Err", bleStats.parseErrors.toString(), Color(0xFFC62828))
        }

        Row(
            modifier = Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            BleLogFilter.entries.forEach { option ->
                FilterChip(
                    selected = filter == option,
                    onClick = { filter = option },
                    label = { Text(option.label) },
                )
            }
            OutlinedButton(
                onClick = onTogglePaused,
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
            ) {
                Text(if (isBlePaused) "Resume" else "Pause")
            }
            OutlinedButton(
                onClick = onClear,
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
            ) {
                Text("Clear")
            }
        }

        Box(
            modifier = Modifier
                .weight(1f)
                .fillMaxWidth()
                .background(Color(0xFF111418), RoundedCornerShape(8.dp))
                .padding(8.dp),
        ) {
            if (filteredLogs.isEmpty()) {
                Text(
                    "No matching BLE diagnostic entries.",
                    color = Color(0xFF9AA4B2),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 12.sp,
                )
            } else {
                LazyColumn(
                    modifier = Modifier.fillMaxSize(),
                    verticalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    items(filteredLogs, key = { it.id }) { entry ->
                        val outcomeColor = when (entry.outcome) {
                            BleLogOutcome.ACCEPTED -> Color(0xFF66BB6A)
                            BleLogOutcome.DROPPED -> Color(0xFFFFB74D)
                            BleLogOutcome.PARSE_ERROR -> Color(0xFFEF5350)
                            BleLogOutcome.EVENT -> Color(0xFF64B5F6)
                        }
                        Column(modifier = Modifier.fillMaxWidth()) {
                            val header = buildString {
                                append("[")
                                append(formatLogTimestamp(entry.timestampMs))
                                append("] ")
                                append(entry.outcome.name)
                                entry.rssi?.let { append(" ${it}dBm") }
                                if (entry.address.isNotBlank()) append(" · ${entry.address}")
                            }
                            Text(
                                header,
                                color = outcomeColor,
                                fontFamily = FontFamily.Monospace,
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Bold,
                            )
                            Text(
                                entry.detail,
                                color = Color(0xFFE6EDF3),
                                fontFamily = FontFamily.Monospace,
                                fontSize = 11.sp,
                            )
                            if (entry.rawHex.isNotBlank()) {
                                Text(
                                    "RAW: ${entry.rawHex}",
                                    color = Color(0xFF9AA4B2),
                                    fontFamily = FontFamily.Monospace,
                                    fontSize = 11.sp,
                                )
                            }
                            Spacer(Modifier.height(4.dp))
                            Divider(color = Color(0xFF232A33))
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun StatPill(label: String, value: String, color: Color) {
    Surface(
        shape = RoundedCornerShape(999.dp),
        color = color,
    ) {
        Text(
            "$label: $value",
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 4.dp),
            color = Color.White,
            style = MaterialTheme.typography.labelMedium,
            fontWeight = FontWeight.SemiBold,
        )
    }
}

@Composable
private fun UsbSerialTab(
    serialLogs: List<SerialLogEntry>,
    usbStatus: UsbSerialStatus,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    onClear: () -> Unit,
    onSendCommand: (String) -> Unit,
) {
    var commandInput by remember { mutableStateOf("") }
    val listState = rememberLazyListState()

    LaunchedEffect(serialLogs.size) {
        if (serialLogs.isNotEmpty()) {
            listState.animateScrollToItem(serialLogs.lastIndex)
        }
    }

    val statusText = when (usbStatus) {
        UsbSerialStatus.Disconnected -> "Status: Disconnected"
        is UsbSerialStatus.PermissionRequired -> "Status: Permission required (${usbStatus.deviceName})"
        is UsbSerialStatus.Connected -> "Status: Connected to ${usbStatus.deviceName}"
        is UsbSerialStatus.Error -> "Status: ${usbStatus.message}"
    }
    val statusColor = when (usbStatus) {
        is UsbSerialStatus.Connected -> Color(0xFF2E7D32)
        is UsbSerialStatus.Error -> MaterialTheme.colorScheme.error
        is UsbSerialStatus.PermissionRequired -> Color(0xFFEF6C00)
        UsbSerialStatus.Disconnected -> MaterialTheme.colorScheme.onSurfaceVariant
    }

    Column(modifier = Modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text(
            statusText,
            color = statusColor,
            style = MaterialTheme.typography.bodyMedium,
            fontWeight = FontWeight.SemiBold,
        )

        Row(
            modifier = Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            Button(
                onClick = onConnect,
                enabled = usbStatus !is UsbSerialStatus.Connected,
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
            ) { Text("Connect USB") }
            OutlinedButton(
                onClick = onDisconnect,
                enabled = usbStatus is UsbSerialStatus.Connected,
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
            ) { Text("Disconnect") }
            OutlinedButton(
                onClick = onClear,
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
            ) { Text("Clear") }
        }

        Row(
            modifier = Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            OutlinedButton(
                onClick = { onSendCommand("c") },
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
            ) { Text("Cal ('c')") }
            OutlinedButton(
                onClick = { onSendCommand("f") },
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
            ) { Text("Fall ('f')") }
            OutlinedButton(
                onClick = { onSendCommand("b") },
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
            ) { Text("Buzzer Fault ('b')") }
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            OutlinedTextField(
                value = commandInput,
                onValueChange = { commandInput = it },
                modifier = Modifier.weight(1f),
                singleLine = true,
                placeholder = { Text("Serial command") },
            )
            Button(
                onClick = {
                    if (commandInput.isNotEmpty()) {
                        onSendCommand(commandInput)
                        commandInput = ""
                    }
                },
                enabled = commandInput.isNotEmpty(),
            ) { Text("Send") }
        }

        Box(
            modifier = Modifier
                .weight(1f)
                .fillMaxWidth()
                .background(Color(0xFF111418), RoundedCornerShape(8.dp))
                .padding(8.dp),
        ) {
            if (serialLogs.isEmpty()) {
                Text(
                    "No USB serial output yet.",
                    color = Color(0xFF9AA4B2),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 12.sp,
                )
            } else {
                LazyColumn(
                    state = listState,
                    modifier = Modifier.fillMaxSize(),
                    verticalArrangement = Arrangement.spacedBy(3.dp),
                ) {
                    items(serialLogs, key = { it.id }) { entry ->
                        Text(
                            "[${formatLogTimestamp(entry.timestampMs)}] ${entry.line}",
                            color = if (entry.isOutgoing) Color(0xFF4FC3F7) else Color(0xFFE6EDF3),
                            fontFamily = FontFamily.Monospace,
                            fontSize = 11.sp,
                        )
                    }
                }
            }
        }
    }
}

@Composable
private fun NodeCard(node: NodeTelemetry, monitoring: MonitoringState) {
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
                if (packet.isBuzzerFault && packet.status != com.cliftonzg.echoguard.core.ble.model.TelemetryStatus.SENSOR_FAULT) {
                    Text(
                        if (node.type == NodeType.CANE) "Buzzer fault also reported · Audio switched to glasses"
                        else "Buzzer fault also reported",
                        modifier = Modifier.testTag("node_${node.type}_buzzer_fault_secondary"),
                        color = Color.White,
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
                if (node.type == NodeType.CANE && packet.isBuzzerFault) {
                    Text(
                        "Audio failover: alerts routed to glasses",
                        modifier = Modifier.testTag("cane_buzzer_failover"),
                        color = Color.White.copy(alpha = 0.9f),
                        style = MaterialTheme.typography.bodySmall,
                        fontWeight = FontWeight.Medium,
                    )
                }
                if (node.type == NodeType.GLASSES && monitoring.node(NodeType.CANE)?.packet?.isBuzzerFault == true &&
                    monitoring.node(NodeType.CANE)?.isFresh == true
                ) {
                    Text(
                        "Active buzzer failover destination · playing audio alerts for pair",
                        modifier = Modifier.testTag("glasses_failover_active"),
                        color = Color.White.copy(alpha = 0.9f),
                        style = MaterialTheme.typography.bodySmall,
                        fontWeight = FontWeight.Medium,
                    )
                }
                if (node.type == NodeType.GLASSES && packet.hasLatchedFall) {
                    val receipt = when (monitoring.glassesFallReceiptState) {
                        PeerReceiptState.CONFIRMED -> "Cane receipt: confirmed"
                        PeerReceiptState.WAITING -> "Cane receipt: waiting"
                        PeerReceiptState.CANE_UNAVAILABLE -> "Cane receipt: unavailable"
                        PeerReceiptState.LEGACY_UNAVAILABLE -> "Cane receipt: unsupported by legacy packet"
                        PeerReceiptState.NOT_APPLICABLE -> null
                    }
                    if (receipt != null) {
                        Text(
                            receipt,
                            modifier = Modifier.testTag("glasses_receipt"),
                            color = Color.White,
                            style = MaterialTheme.typography.bodySmall,
                        )
                    }
                }
                Text(
                    "Heartbeat #${packet.seqNum}" + if (packet.compactFormat) {
                        " · Fall sequence ${packet.rollingFallSeq} (0–15)"
                    } else "",
                    modifier = Modifier.testTag("node_${node.type}_sequence"),
                    color = Color.White.copy(alpha = 0.8f),
                    style = MaterialTheme.typography.labelSmall,
                )
                node.phoneFallCount?.let { count ->
                    Text(
                        "Phone-recorded falls: $count" + if (node.phoneHistoryHasGaps) " · incomplete history" else "",
                        modifier = Modifier.testTag("node_${node.type}_fall_history"),
                        color = Color.White,
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
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
        protocolVersion = SystemPacket.PROTOCOL_VERSION, sessionId = 0x1001L, eventId = 5L,
    )
    val state = MonitoringState(device, true, listOf(
        NodeTelemetry(NodeType.CANE, device.address, packet(NodeType.CANE, caneFlags), LinkState.ACTIVE),
        NodeTelemetry(NodeType.GLASSES, "DE:MO:00:00:00:02", packet(NodeType.GLASSES, 0), LinkState.ACTIVE),
    ))
    MaterialTheme {
        BleDashboardContent(state, true, emptyList(), null, {}, {}, {}, {}, {})
    }
}
