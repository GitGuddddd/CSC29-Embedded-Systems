package com.cliftonzg.echoguard.core.ble

import android.content.Context

/** One atomically replaced preference value per selected pair/address/role. */
class PreferencesFallHistoryStore(context: Context) : FallHistoryStore {
    private val preferences = context.getSharedPreferences("phone_fall_history_v1", Context.MODE_PRIVATE)

    override fun load(key: String): FallHistoryRecord? = FallHistoryRecord.decode(
        runCatching { preferences.getString(key, null) }.getOrNull(),
    )

    override fun save(key: String, record: FallHistoryRecord) {
        preferences.edit().putString(key, record.encode()).apply()
    }
}
