package computer.living.gamepadyn.ftc

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import computer.living.gamepadyn.InputSystem

class InputSystemFtc(opMode: OpMode) : InputSystem {

    private val gamepads: Array<RawGamepadFtc> = arrayOf(RawGamepadFtc(opMode.gamepad1), RawGamepadFtc(opMode.gamepad2))

    override fun getGamepads(): Array<out InputSystem.RawGamepad> {
        return gamepads.copyOf()
    }

}

