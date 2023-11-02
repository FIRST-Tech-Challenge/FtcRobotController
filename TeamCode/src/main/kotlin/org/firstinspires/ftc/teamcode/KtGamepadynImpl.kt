package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import computer.living.gamepadyn.ActionBind
import computer.living.gamepadyn.ActionBindDigital
import computer.living.gamepadyn.ActionDataAnalog
import computer.living.gamepadyn.ActionDataDigital
import computer.living.gamepadyn.Configuration
import computer.living.gamepadyn.GAD
import computer.living.gamepadyn.InputType.*
import computer.living.gamepadyn.Gamepadyn
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.CLAW
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.MOVEMENT
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.ROTATION
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.TOGGLE_INTAKE_LIFT
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.TOGGLE_RELATIVITY

@TeleOp
class KtGamepadynImpl : OpMode() {

    enum class TestAction {
        MOVEMENT, ROTATION, CLAW, TOGGLE_RELATIVITY, TOGGLE_INTAKE_LIFT
    }

    private val testActionDescription: Map<TestAction, GAD> = mapOf(
        MOVEMENT            to GAD(ANALOG, 2),
        ROTATION            to GAD(ANALOG, 1),
        CLAW                to GAD(DIGITAL),
        TOGGLE_RELATIVITY   to GAD(DIGITAL),
        TOGGLE_INTAKE_LIFT  to GAD(DIGITAL)
    )

    private lateinit var gamepadyn: Gamepadyn<TestAction>

    private var useBotRelativity: Boolean = false

    class TestActionBind<T: Enum<T>> : ActionBindDigital<T>() {
        override fun transform(): Array<ActionDataAnalog?> {
            TODO("Not yet implemented")
        }

    }

    override fun init() {
        gamepadyn = Gamepadyn(this, false, testActionDescription)

        gamepadyn.p0.configuration = Configuration(
            TestActionBind(),
            TestActionBind(),
            TestActionBind()
        )
    }

    override fun start() {
        super.start()

        gamepadyn.p0.getEventDigital(TOGGLE_RELATIVITY)!!.addListener {
            if (it.digitalData) {
                useBotRelativity = !useBotRelativity
            }
        }
    }

    override fun loop() {}
}
