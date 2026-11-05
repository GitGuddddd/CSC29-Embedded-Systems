package com.cliftonzg.echoguard.core.ble.model

enum class StatusTone { NEUTRAL, OK, CAUTION, ALERT }

data class TelemetryPresentation(val title: String, val detail: String, val tone: StatusTone)

fun NodeTelemetry.presentation(): TelemetryPresentation {
    val report = packet
    if (report == null) return TelemetryPresentation(
        "Waiting for telemetry", "No fresh heartbeat received for this node yet.", StatusTone.NEUTRAL,
    )
    if (!isFresh) {
        val state = if (linkState == LinkState.DISCONNECTED) "Scanning stopped" else "Link lost"
        return when {
            report.hasLatchedFall -> TelemetryPresentation(
                if (report.isAlarmPaused) "Last report: fall paused" else "Last report: fall alarm",
                "$state. The incident has not been confirmed cleared.", StatusTone.ALERT,
            )
            else -> TelemetryPresentation(state, "Last readings are historical, not live telemetry.", StatusTone.CAUTION)
        }
    }
    return when (report.status) {
        TelemetryStatus.FALL_ALARM -> TelemetryPresentation(
            "Fall alarm", "Incident remains latched until the device reports recovery or reset.", StatusTone.ALERT,
        )
        TelemetryStatus.FALL_PAUSED -> TelemetryPresentation(
            "Fall audio paused", "Incident still latched. A pause is not a recovery confirmation.", StatusTone.CAUTION,
        )
        TelemetryStatus.STANDBY -> TelemetryPresentation(
            if (report.isParkedByUser) "Parked by user" else "Standby",
            when {
                type == NodeType.CANE && report.isCaneHeld -> "IMU asleep. Release the grip, then hold again to wake."
                type == NodeType.CANE -> "IMU asleep. Hold the cane to wake."
                else -> "IMU asleep. Wake using the cane or glasses button."
            },
            StatusTone.NEUTRAL,
        )
        TelemetryStatus.SENSOR_FAULT -> TelemetryPresentation(
            "Sensing degraded", "Sensor or data-path fault reported. The affected component is not identified in this packet.",
            StatusTone.CAUTION,
        )
        TelemetryStatus.GROUND_HAZARD -> TelemetryPresentation(
            "Ground hazard", "The cane's calibrated detector reports a ground-height hazard.", StatusTone.ALERT,
        )
        TelemetryStatus.OBSTACLE_CRITICAL -> TelemetryPresentation(
            "Close obstacle", "Forward obstacle alert reported by the glasses.", StatusTone.ALERT,
        )
        TelemetryStatus.OBSTACLE_WARNING -> TelemetryPresentation(
            "Obstacle warning", "Forward obstacle alert reported by the glasses.", StatusTone.CAUTION,
        )
        TelemetryStatus.GRIP_RELEASED -> TelemetryPresentation(
            "Grip released", "Hold the cane to use ground alerts.", StatusTone.NEUTRAL,
        )
        TelemetryStatus.MONITORING -> TelemetryPresentation(
            "Monitoring", "Fresh telemetry; no hazard reported.", StatusTone.OK,
        )
        TelemetryStatus.NO_READING -> TelemetryPresentation(
            "No valid range", "Fresh heartbeat, but no usable range reading is available.", StatusTone.NEUTRAL,
        )
    }
}

fun MonitoringState.presentation(): TelemetryPresentation {
    if (selectedDevice == null) return TelemetryPresentation(
        "No node selected", "Select an EchoGuard node to monitor its pair.", StatusTone.NEUTRAL,
    )
    if (!isScanning) return TelemetryPresentation(
        "Monitoring stopped", "Start scanning to receive fresh telemetry.", StatusTone.NEUTRAL,
    )
    val fresh = nodes.filter { it.isFresh }
    val activeFalls = fresh.filter { it.packet?.status == TelemetryStatus.FALL_ALARM }
    if (activeFalls.isNotEmpty()) return TelemetryPresentation(
        "Fall alarm", "Reported by ${activeFalls.labels()}. Incident remains latched.", StatusTone.ALERT,
    )
    val staleFalls = nodes.filter { !it.isFresh && it.packet?.hasLatchedFall == true }
    if (staleFalls.isNotEmpty()) return TelemetryPresentation(
        "Fall report · link lost", "${staleFalls.labels()}: last incident is unresolved; current status is unavailable.",
        StatusTone.ALERT,
    )
    val paused = fresh.filter { it.packet?.status == TelemetryStatus.FALL_PAUSED }
    if (paused.isNotEmpty()) return TelemetryPresentation(
        "Fall audio paused", "${paused.labels()}: acknowledged pause, incident still latched.", StatusTone.CAUTION,
    )
    val hazards = fresh.filter { it.packet?.status in setOf(
        TelemetryStatus.GROUND_HAZARD, TelemetryStatus.OBSTACLE_CRITICAL, TelemetryStatus.OBSTACLE_WARNING,
    ) }
    hazards.firstOrNull { it.packet?.threatLevel == ThreatLevel.CRITICAL }?.let { return it.presentation() }
    val lost = nodes.filter { it.linkState == LinkState.LINK_LOSS }
    if (lost.isNotEmpty()) return TelemetryPresentation(
        "Telemetry link lost", "${lost.labels()}: no fresh heartbeat for at least 500 ms.", StatusTone.CAUTION,
    )
    val faults = fresh.filter { it.packet?.status == TelemetryStatus.SENSOR_FAULT }
    if (faults.isNotEmpty()) return TelemetryPresentation(
        "Sensing degraded", "${faults.labels()}: sensor or data-path fault reported.", StatusTone.CAUTION,
    )
    hazards.firstOrNull()?.let { return it.presentation() }
    if (selectedLinkState == LinkState.WAITING) return TelemetryPresentation(
        "Waiting for selected node", "A radio selection is not a fresh heartbeat.", StatusTone.NEUTRAL,
    )
    if (fresh.isNotEmpty() && fresh.all { it.packet?.isStandby == true }) return TelemetryPresentation(
        "Standby", "Connected nodes report IMU sleep; Bluetooth telemetry remains available.", StatusTone.NEUTRAL,
    )
    fresh.firstOrNull { it.packet?.status == TelemetryStatus.MONITORING }?.let {
        return TelemetryPresentation("Monitoring", "${it.type.label}: fresh telemetry, no hazard reported.", StatusTone.OK)
    }
    return fresh.firstOrNull()?.presentation() ?: TelemetryPresentation(
        "Waiting for telemetry", "No fresh node data is available.", StatusTone.NEUTRAL,
    )
}

private fun List<NodeTelemetry>.labels(): String = joinToString { it.type.label }
