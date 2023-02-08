package ftc.rogue.blacksmith

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rogue.blacksmith.listeners.Listener
import io.mockk.clearMocks
import io.mockk.every
import io.mockk.mockk
import io.mockk.verify
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.junit.jupiter.api.assertDoesNotThrow

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
internal class SchedulerTest {
    private val linearOpMode = mockk<LinearOpMode>()
    private val telemetry = mockk<Telemetry>(relaxed = true)
    private var isStopped = false

    init {
        every { linearOpMode.opModeIsActive() } answers { !isStopped }
        every { linearOpMode.isStopRequested } answers { isStopped }
    }

    @BeforeEach
    fun setUp() {
        Scheduler.reset()
        clearMocks(telemetry)
        isStopped = false
    }

    @Test
    fun `scheduler properly runs everything and in the right order`() {
        val listeners = Array(3) { mockk<Listener>() }
        var output = ""

        listeners.forEachIndexed { index, listener ->
            every { listener.tick() } answers { output += index + 1 }
        }

        listeners.forEach(Scheduler::hookListener)

        Scheduler.beforeEach {
            output += "0"
        }

        Scheduler.launch(linearOpMode) {
            output += "4"
            isStopped = true
        }

        assertEquals("01234", output)
    }

    @Test
    fun `scheduler properly times each loop`() {
        var iterations = 0

        Scheduler.time(linearOpMode) {
            Thread.sleep(500)

            telemetry.addData("Loop time (ms)", this)

            iterations++
            if (iterations == 4) {
                isStopped = true
            }
        }

        verify(atLeast = 3) { telemetry.addData("Loop time (ms)", range(450.0, 600.0)) }
        verify(exactly = 3) { telemetry.update() }
    }

    @Test
    fun `scheduler handles adding and deleting listeners from listener`() {
        val listeners = Array(4) { mockk<Listener>(relaxed = true) }

        for (i in 0..2) {
            Scheduler.hookListener(listeners[i])
        }

        assertDoesNotThrow {
            Scheduler.launch(linearOpMode) {
                Scheduler.unhookListener(listeners[1])
                Scheduler.hookListener(listeners[3])
                isStopped = true
            }
        }
    }

    @Test
    fun `scheduler messaging works`() {
        val msg = 234109324923L

        Scheduler.on(msg) {
            telemetry.addLine("msg")
        }

        Scheduler.on(msg) {
            telemetry.addLine("msg")
        }

        Scheduler.on(0) {
            telemetry.addLine("!msg")
        }

        Scheduler.emit(msg)

        verify(exactly = 2) { telemetry.addLine("msg") }
        verify(exactly = 0) { telemetry.addLine("!msg") }
    }
}
