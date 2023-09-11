package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import computer.living.gamepadyn.EActionType
import computer.living.gamepadyn.Gamepadyn
import computer.living.gamepadyn.IUserAction

@TeleOp
class BasicKotlinGamepadyn : OpMode() {

    enum class UserAction(override val axes: Int = 0): IUserAction {
        ACTION_DEBUG_1, ACTION_DEBUG_2(2);
    }

    var gamepadyn = Gamepadyn<UserAction>(this, UserAction.values())

    override fun init() {}
    override fun loop() {

    }
}