package computer.living.gamepadyn

import computer.living.gamepadyn.SimulatedInputUnitTest.TestAction.DEBUG_ACTION
import org.junit.Assert.assertEquals
import org.junit.Test

class SimulatedInputUnitTest {

    enum class TestAction {
        DEBUG_ACTION
    }

    private val actions: Map<TestAction, GAD> = mapOf(
        DEBUG_ACTION to GAD(InputType.DIGITAL)
    )

    @Test
    fun main() {

        val sysTest = InputSystemTesting()
        InputSystemTesting.manipulableState = false

        val gamepadyn = Gamepadyn(sysTest, strict = true, useInputThread = false, actions)

        var stateChangeCount = 0

        val p0 = gamepadyn.players[0]

        gamepadyn.update()

        gamepadyn.players[0].configuration = Configuration(
            ActionBind(RawInput.FACE_A, DEBUG_ACTION)
        )

        gamepadyn.update()

        p0.getEventDigital(DEBUG_ACTION)!!.addListener {
            println("Debug action ran!")
            stateChangeCount++
        }

        gamepadyn.update()

        // one state change
        InputSystemTesting.manipulableState = true

        gamepadyn.update()
        assertEquals(true, p0.getStateDigital(DEBUG_ACTION)?.digitalData)
        gamepadyn.update()
        assertEquals(true, p0.getStateDigital(DEBUG_ACTION)?.digitalData)

        // two state changes
        InputSystemTesting.manipulableState = false

        gamepadyn.update()
        assertEquals(false, p0.getStateDigital(DEBUG_ACTION)?.digitalData)
        gamepadyn.update()
        assertEquals(false, p0.getStateDigital(DEBUG_ACTION)?.digitalData)

        // three state changes
        InputSystemTesting.manipulableState = true

        gamepadyn.update()
        assertEquals(true, p0.getStateDigital(DEBUG_ACTION)?.digitalData)

        assertEquals(3, stateChangeCount)
//        assert(stateChangeCount == 2, "BAD!!!")

    }

}