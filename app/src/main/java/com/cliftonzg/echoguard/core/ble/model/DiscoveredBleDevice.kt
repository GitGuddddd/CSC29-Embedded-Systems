package com.cliftonzg.echoguard.core.ble.model

data class DiscoveredBleDevice(
    val name: String,
    val address: String,
    val rssi: Int,
    val isEchoGuardNode: Boolean,
    val lastSeenMs: Long = 0L, // Android elapsed-realtime capture timestamp.
    val nodeType: NodeType? = null,
    val pairId: Int? = null,
)
