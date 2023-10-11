package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import computer.living.gamepadyn.ActionDescriptor
import computer.living.gamepadyn.GAD
import computer.living.gamepadyn.ActionType.*
import computer.living.gamepadyn.Gamepadyn
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.CLAW
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.MOVEMENT
import org.firstinspires.ftc.teamcode.KtGamepadynImpl.TestAction.ROTATION

@TeleOp
class KtGamepadynImpl : OpMode() {

    enum class TestAction {
        MOVEMENT, ROTATION, CLAW
    }
    private var gamepadyn: Gamepadyn<TestAction>? = null

    override fun init() {
        val map: Map<TestAction, GAD> = mapOf(
            MOVEMENT to GAD(ANALOG, 2),
            ROTATION to GAD(ANALOG, 1),
            CLAW     to GAD(DIGITAL),
        )
        gamepadyn = Gamepadyn(this, false, map);
    }
    override fun loop() {}
}
