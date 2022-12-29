package ftc.rogue.blacksmith.internal

import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.test.assertTrue

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
internal class SignalEdgeDetectorTest {
    private var boolean = false
    private val detector = SignalEdgeDetector { boolean }

    @BeforeEach
    fun setUp() {
        boolean = false
        detector.update()
        detector.update() // Double update to clear rising edge state
    }

    @Test
    fun `detector correctly detects rising edge`() {
        boolean = true
        detector.update()

        assertTrue(detector.risingEdge())
    }

    @Test
    fun `detector correctly detects falling edge`() {
        boolean = true
        detector.update()
        boolean = false
        detector.update()

        assertTrue(detector.fallingEdge())
    }

    @Test
    fun `detector correctly detects high`() {
        boolean = true
        detector.update()

        assertTrue(detector.isHigh())
    }

    @Test
    fun `detector correctly detects low`() {
        assertTrue(detector.isLow())
    }
}