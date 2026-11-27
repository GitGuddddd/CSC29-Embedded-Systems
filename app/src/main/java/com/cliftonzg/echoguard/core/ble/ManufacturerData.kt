package com.cliftonzg.echoguard.core.ble

/** Bounds-checked AD structure fallback for devices without a populated ScanRecord sparse array. */
object ManufacturerData {
    const val COMPANY_ID = 0x0029
    const val LEGACY_COMPANY_ID = 0x2900

    fun extract(advertisement: ByteArray): ByteArray? {
        var offset = 0
        while (offset < advertisement.size) {
            val length = advertisement[offset].toInt() and 0xFF
            if (length == 0) break
            val end = offset + length + 1
            if (end > advertisement.size || length < 1) return null
            val type = advertisement[offset + 1].toInt() and 0xFF
            if (type == 0xFF && length >= 8) {
                val company = (advertisement[offset + 2].toInt() and 0xFF) or
                    ((advertisement[offset + 3].toInt() and 0xFF) shl 8)
                if (company == COMPANY_ID || company == LEGACY_COMPANY_ID) {
                    return advertisement.copyOfRange(offset + 4, end)
                }
            }
            offset = end
        }
        return null
    }
}
