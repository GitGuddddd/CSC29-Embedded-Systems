package com.cliftonzg.echoguard.core.ble

import com.cliftonzg.echoguard.core.ble.model.NodeType
import com.cliftonzg.echoguard.core.ble.model.SystemPacket
import org.junit.Assert.*
import org.junit.Test

class PhoneFallHistoryTest {
    @Test fun `phone accumulates totals above fifteen across a compact counter wrap`() {
        val history = PhoneFallHistory()
        history.observe("glasses", packet(NodeType.GLASSES))
        for (sequence in 1..15) {
            assertEquals(sequence.toLong(), history.observe("glasses", packet(NodeType.GLASSES, fallSeq = sequence))?.total)
        }
        assertEquals(16L, history.observe("glasses", packet(NodeType.GLASSES, session = 0x1002L, rollover = true))?.total)
        val record = history.observe("glasses", packet(NodeType.GLASSES, session = 0x1002L, rollover = true, fallSeq = 4))!!
        assertEquals(20L, record.total)
        assertFalse(record.hasGaps)
    }

    @Test fun `heartbeat pause and cane relay do not count as additional local falls`() {
        val history = PhoneFallHistory()
        history.observe("cane", packet())
        assertEquals(0L, history.observe("cane", packet(flags = SystemPacket.FLAG_FALL_ALERT,
            ackValid = true, ackSession = 0x2001L, ackEvent = 3L))?.total)
        history.observe("cane", packet(fallSeq = 1, flags = SystemPacket.FLAG_FALL_ALERT))
        assertEquals(1L, history.observe("cane", packet(fallSeq = 1, seq = 2,
            flags = SystemPacket.FLAG_FALL_ALERT or SystemPacket.FLAG_ALARM_PAUSED))?.total)
    }

    @Test fun `reboot preserves phone total but marks unavailable history instead of double counting`() {
        val history = PhoneFallHistory()
        history.observe("glasses", packet(NodeType.GLASSES))
        history.observe("glasses", packet(NodeType.GLASSES, fallSeq = 3))
        val reboot = history.observe("glasses", packet(NodeType.GLASSES, session = 0x2001L, fallSeq = 3,
            flags = SystemPacket.FLAG_FALL_ALERT))!!
        assertEquals(3L, reboot.total)
        assertTrue(reboot.hasGaps)
        assertEquals(4L, history.observe("glasses", packet(NodeType.GLASSES, session = 0x2001L, fallSeq = 4))?.total)
    }

    @Test fun `multiple missed epochs cannot silently become an exact lifetime total`() {
        val history = PhoneFallHistory()
        history.observe("node", packet())
        history.observe("node", packet(fallSeq = 15))
        val gap = history.observe("node", packet(session = 0x1003L, fallSeq = 2, rollover = true))!!
        assertEquals(15L, gap.total)
        assertTrue(gap.hasGaps)
        assertEquals(16L, history.observe("node", packet(session = 0x1003L, fallSeq = 3, rollover = true))?.total)
    }

    @Test fun `process recreation resumes the persisted baseline without recounting`() {
        val store = MemoryFallHistoryStore()
        val first = PhoneFallHistory(store)
        first.observe("node", packet())
        first.observe("node", packet(fallSeq = 4))
        val recreated = PhoneFallHistory(store)
        assertEquals(4L, recreated.observe("node", packet(fallSeq = 4))?.total)
        assertEquals(5L, recreated.observe("node", packet(fallSeq = 5))?.total)
        assertEquals(0L, recreated.observe("other-pair", packet())?.total)
        val unknown = PhoneFallHistory().observe("first-alarm", packet(flags = SystemPacket.FLAG_FALL_ALERT))!!
        assertEquals(0L, unknown.total)
        assertTrue(unknown.hasGaps)
    }

    @Test fun `epoch maximum wraps to one without expanding the wearable counter`() {
        val history = PhoneFallHistory()
        history.observe("node", packet(session = 0xFFFFL))
        history.observe("node", packet(session = 0xFFFFL, fallSeq = 15))
        assertEquals(16L, history.observe("node", packet(session = 1L, rollover = true))?.total)
    }

    @Test fun `stale persisted baseline cannot double count and phone totals saturate safely`() {
        val store = MemoryFallHistoryStore()
        store.save("node", FallHistoryRecord(4L, 0x1001, 4, false))
        val history = PhoneFallHistory(store)
        assertEquals(4L, history.observe("node", packet(fallSeq = 3))?.total)
        assertEquals(4L, history.observe("node", packet(fallSeq = 4))?.total)
        assertEquals(5L, history.observe("node", packet(fallSeq = 5))?.total)
        store.save("maximum", FallHistoryRecord(Long.MAX_VALUE, 0x1001, 0, false))
        val maximum = history.observe("maximum", packet(fallSeq = 1))!!
        assertEquals(Long.MAX_VALUE, maximum.total)
        assertTrue(maximum.hasGaps)
    }

    @Test fun `persistent representation validates ranges and round trips wide phone totals`() {
        val record = FallHistoryRecord(100_000L, 0x1234, 15, true)
        assertEquals(record, FallHistoryRecord.decode(record.encode()))
        for (bad in listOf(null, "", "-1,1,0,0", "1,0,0,0", "1,1,16,0", "1,1,0,2", "1,1,0,0,9")) {
            assertNull(FallHistoryRecord.decode(bad))
        }
    }

    @Test fun `only counter transitions write history and old protocols do not extend totals`() {
        val backing = MemoryFallHistoryStore()
        var writes = 0
        val store = object : FallHistoryStore {
            override fun load(key: String) = backing.load(key)
            override fun save(key: String, record: FallHistoryRecord) {
                writes++
                backing.save(key, record)
            }
        }
        val history = PhoneFallHistory(store)
        history.observe("node", packet())
        history.observe("node", packet(seq = 2))
        history.observe("node", packet(seq = 3, flags = SystemPacket.FLAG_STANDBY))
        assertEquals(1, writes)
        history.observe("node", packet(fallSeq = 1))
        assertEquals(2, writes)
        assertNull(history.observe("node", packet().copy(protocolVersion = 1)))
    }
}
