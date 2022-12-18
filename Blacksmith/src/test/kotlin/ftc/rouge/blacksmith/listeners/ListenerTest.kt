package ftc.rouge.blacksmith.listeners

import ftc.rouge.blacksmith.Scheduler
import org.junit.jupiter.api.Assertions.assertAll
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
internal class ListenerTest {
    private val hookedListeners = Scheduler::class.java.getDeclaredField("listeners").let {
        it.isAccessible = true
        it.get(Scheduler) as MutableSet<Listener>
    }

    @BeforeEach
    fun setUp() {
        Scheduler.reset()
    }

    @Test
    fun `listener correctly works with high & low`() {
        var shouldIncrement = true
        val listener = Listener { shouldIncrement }

        var test = 0
        listener.whileHigh { test++ }
        listener.whileLow { test-- }

        repeat(10) {
            listener.tick()
        }

        shouldIncrement = false
        listener.tick()

        assertEquals(9, test)
    }

    @Test
    fun `listener correctly works with rising & falling`() {
        var addA = false
        val listener = Listener { addA }

        listener.tick() // Initial tick so the rest are edge ticks (as first is never an edge tick)

        var test = ""
        listener.onRise { test += 'a' }
        listener.onFall { test += 'b' }

        repeat(10) {
            addA = !addA
            listener.tick()
        }

        assertEquals("ababababab", test)
    }

    @Test
    fun `subscribing to listener hooks it on the first time`() {
        val listener = Listener { true }

        val initialSize = hookedListeners.size

        listener.whileHigh { }
        val firstSize = hookedListeners.size

        listener.whileHigh { }
        val secondSize = hookedListeners.size

        assertAll(
            { assertEquals(0, initialSize) },
            { assertEquals(1, firstSize) },
            { assertEquals(1, secondSize) }
        )
    }

    @Test
    fun `destroying listener clears its added actions`() {
        val listener = Listener { true }
        listener.onRise {}

        val actions = listener::class.java.getDeclaredField("actions").let { field ->
            field.isAccessible = true
            field.get(listener) as MutableMap<*, *>
        }

        val initialSize = actions.size
        listener.destroy()
        val clearedSize = actions.size

        assertAll(
            { assertEquals(1, initialSize) },
            { assertEquals(0, clearedSize) }
        )
    }

    @Test
    fun `destroying listener removes it from the scheduler`() {
        val listener = Listener { true }

        listener.onRise {}
        val initialSize = hookedListeners.size

        listener.destroy()
        val clearedSize = hookedListeners.size

        assertAll(
            { assertEquals(1, initialSize) },
            { assertEquals(0, clearedSize) }
        )
    }
}
