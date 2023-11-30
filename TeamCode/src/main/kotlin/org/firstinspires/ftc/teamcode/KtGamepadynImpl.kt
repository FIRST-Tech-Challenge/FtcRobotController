package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import computer.living.gamepadyn.ActionBind
import computer.living.gamepadyn.Configuration
import computer.living.gamepadyn.GAD
import computer.living.gamepadyn.InputType.*
import computer.living.gamepadyn.Gamepadyn
import computer.living.gamepadyn.RawInput
import computer.living.gamepadyn.ftc.InputSystemFtc
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.MOVEMENT
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.ROTATION
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.CLAW
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.DEBUG_ACTION

@TeleOp
class KtGamepadynImpl : OpMode() {

    enum class TestAction {
        MOVEMENT, ROTATION, CLAW, DEBUG_ACTION
    }

    private lateinit var gamepadyn: Gamepadyn<TestAction>


    override fun init() {
        gamepadyn = Gamepadyn(
            InputSystemFtc(this),
            true,
            false,
            MOVEMENT            to GAD(ANALOG, 2),
            ROTATION            to GAD(ANALOG, 1),
            CLAW                to GAD(DIGITAL),
            DEBUG_ACTION        to GAD(DIGITAL)
        )

        gamepadyn.players[0].configuration = Configuration(
            ActionBind(RawInput.FACE_A, DEBUG_ACTION)
        )
    }

    override fun start() {
        super.start()

        gamepadyn.players[0].getEventDigital(DEBUG_ACTION)!!.addListener {
            telemetry.addLine("Button ${if (it.digitalData) "pressed"; else "released"}!")
        }
    }

    override fun loop() {}
}
