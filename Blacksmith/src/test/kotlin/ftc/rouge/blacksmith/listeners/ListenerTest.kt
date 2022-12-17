package ftc.rouge.blacksmith.listeners

import ftc.rouge.blacksmith.Scheduler
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

internal class ListenerTest {
    private val hookedListeners = Scheduler::class.java.getDeclaredField("listeners").let {
        it.isAccessible = true
        it.get(Scheduler) as MutableSet<Listener>
    }

    @BeforeEach
    fun setUp() {
        hookedListeners.clear()
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

        assertEquals(0, hookedListeners.size)

        listener.whileHigh { }
        assertEquals(1, hookedListeners.size)

        listener.whileHigh { }
        assertEquals(1, hookedListeners.size)
    }

    @Test
    fun `destroying listener clears it's added actions`() {
        val listener = Listener { true }
        listener.onRise {}

        val actions = listener::class.java.getDeclaredField("actions").let { field ->
            field.isAccessible = true
            field.get(listener) as MutableMap<*, *>
        }

        assertEquals(1, actions.size)
        listener.destroy()
        assertEquals(0, actions.size)
    }

    @Test
    fun `destroying listener removes it from the scheduler`() {
        val listener = Listener { true }

        listener.onRise {}
        assertEquals(1, hookedListeners.size)

        listener.destroy()
        assertEquals(0, hookedListeners.size)
    }
}
