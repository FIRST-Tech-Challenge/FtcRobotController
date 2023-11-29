package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import computer.living.gamepadyn.ActionBind
import computer.living.gamepadyn.InputDataDigital
import computer.living.gamepadyn.Configuration
import computer.living.gamepadyn.GAD
import computer.living.gamepadyn.InputType.*
import computer.living.gamepadyn.Gamepadyn
import computer.living.gamepadyn.InputData
import computer.living.gamepadyn.InputType
import computer.living.gamepadyn.RawInput
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.MOVEMENT
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.ROTATION
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.CLAW
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.TOGGLE_RELATIVITY
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.DEBUG_ACTION

@TeleOp
class KtGamepadynImpl : OpMode() {

    enum class TestAction {
        MOVEMENT, ROTATION, CLAW, TOGGLE_RELATIVITY, DEBUG_ACTION
    }

    private val testActionDescription: Map<TestAction, GAD> = mapOf(
        MOVEMENT            to GAD(ANALOG, 2),
        ROTATION            to GAD(ANALOG, 1),
        CLAW                to GAD(DIGITAL),
        TOGGLE_RELATIVITY   to GAD(DIGITAL),
        DEBUG_ACTION        to GAD(DIGITAL)
    )

    private lateinit var gamepadyn: Gamepadyn<TestAction>

    private var useBotRelativity: Boolean = false

    override fun init() {
        gamepadyn = Gamepadyn(this, false, testActionDescription)

        gamepadyn.p0.configuration = Configuration(
            ActionBind(RawInput.FACE_A, DEBUG_ACTION)
        )
    }

    override fun start() {
        super.start()

        gamepadyn.p0.getEventDigital(DEBUG_ACTION)!!.addListener {
            if (it.digitalData) useBotRelativity = !useBotRelativity
        }
    }

    override fun loop() {}
}
