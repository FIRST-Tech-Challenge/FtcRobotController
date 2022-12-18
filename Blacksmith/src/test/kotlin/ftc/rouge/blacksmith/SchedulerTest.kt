package ftc.rouge.blacksmith

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rouge.blacksmith.listeners.Listener
import io.mockk.every
import io.mockk.mockk
import io.mockk.verify
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
internal class SchedulerTest {
    private val linearOpMode = mockk<LinearOpMode>()
    private var isStopped = false

    init {
        every { linearOpMode.opModeIsActive() } answers { !isStopped }
        every { linearOpMode.isStopRequested } answers { isStopped }
    }

    @BeforeEach
    fun setUp() {
        Scheduler.reset()
        isStopped = false
    }

    @Test
    fun `scheduler properly runs everything and in the right order`() {
        val listeners = Array(3) { mockk<Listener>()}
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
        val telemetry = mockk<Telemetry>(relaxed = true)

        var iterations = 0
        Scheduler.time(linearOpMode, telemetry) {
            Thread.sleep(500)

            iterations++
            if (iterations == 3) {
                isStopped = true
            }
        }

        verify(exactly = 3) { telemetry.addData("Loop time (ms)", range<Long>(500, 600)) }
        verify(exactly = 3) { telemetry.update() }
    }
}
