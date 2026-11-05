package com.cliftonzg.echoguard.ui.ble

import android.graphics.Bitmap
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertTextEquals
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.unit.Density
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.cliftonzg.echoguard.core.ble.model.DiscoveredBleDevice
import com.cliftonzg.echoguard.core.ble.model.LinkState
import com.cliftonzg.echoguard.core.ble.model.MonitoringState
import com.cliftonzg.echoguard.core.ble.model.NodeTelemetry
import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import java.io.File
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class BleDashboardTest {
    @get:Rule val compose = createComposeRule()

    private fun render(caneFlags: Int, glassesFlags: Int = 0, fontScale: Float = 1f) {
        val device = DiscoveredBleDevice("EchoGuard Smart Cane", "AA:00:00:00:00:01", -48, true,
            nodeType = NodeType.CANE, pairId = 1)
        fun decode(cane: Boolean, flags: Int) = requireNotNull(SystemPacket.parse(byteArrayOf(
            0xA1.toByte(), (if (cane) 0xB5 else 0x35).toByte(), flags.toByte(),
            0xFF.toByte(), 0xFF.toByte(), 0xFF.toByte(),
        )))
        val state = MonitoringState(device, true, listOf(
            NodeTelemetry(NodeType.CANE, device.address, decode(true, caneFlags), LinkState.ACTIVE),
            NodeTelemetry(NodeType.GLASSES, "AA:00:00:00:00:02", decode(false, glassesFlags), LinkState.ACTIVE),
        ))
        compose.setContent {
            val density = LocalDensity.current
            CompositionLocalProvider(LocalDensity provides Density(density.density, fontScale)) {
                MaterialTheme {
                    BleDashboardContent(state, true, emptyList(), null, {}, {}, {}, {}, {})
                }
            }
        }
    }

    private fun capture(name: String) {
        compose.waitForIdle()
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val bitmap = requireNotNull(instrumentation.uiAutomation.takeScreenshot())
        File(instrumentation.targetContext.filesDir, "$name.png").outputStream().use {
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, it)
        }
        bitmap.recycle()
    }

    @Test fun pausedFallStaysLatchedAndVisible() {
        render(0x19) // FALL + PAUSED + HELD, with unavailable range.
        compose.onNodeWithTag("summary_title").assertTextEquals("Fall audio paused")
        capture("android-paused")
        compose.onNodeWithTag("node_CANE_status").performScrollTo().assertTextEquals("Fall audio paused")
    }

    @Test fun parkedCaneWithHeldTouchDoesNotBecomeACliffAlarm() {
        render(0x70, 0x20) // HELD + STANDBY + PARKED_BY_USER; glasses standby too.
        compose.onNodeWithTag("summary_title").assertTextEquals("Standby")
        capture("android-standby")
        compose.onNodeWithTag("node_CANE_status").performScrollTo().assertTextEquals("Parked by user")
        compose.onNodeWithText("IMU asleep. Release the grip, then hold again to wake.").assertIsDisplayed()
    }

    @Test fun independentGlassesFallWinsOverCanePauseAndLargeTextCanScroll() {
        render(0x19, 0x01, fontScale = 1.4f)
        compose.onNodeWithTag("summary_title").assertTextEquals("Fall alarm")
        compose.onNodeWithTag("node_GLASSES_status").performScrollTo().assertTextEquals("Fall alarm")
        compose.onNodeWithText("Stop scanning").performScrollTo().assertIsDisplayed()
    }

    @Test fun genericImuOrDataFaultDoesNotClaimAnObstructedLens() {
        render(0x12) // SENSOR_FAULT + HELD
        compose.onNodeWithTag("summary_title").assertTextEquals("Sensing degraded")
        capture("android-fault")
        compose.onNodeWithTag("node_CANE_status").performScrollTo().assertTextEquals("Sensing degraded")
    }
}
