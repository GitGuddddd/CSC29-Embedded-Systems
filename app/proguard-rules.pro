# Keep Hilt / Dagger generated components
-keep class dagger.hilt.** { *; }
-keepclassmembers class * {
    @dagger.hilt.** *;
}

# Keep SystemPacket data class fields for BLE deserialization
-keepclasseswithmembers class com.cliftonzg.echoguard.core.ble.model.SystemPacket { *; }
