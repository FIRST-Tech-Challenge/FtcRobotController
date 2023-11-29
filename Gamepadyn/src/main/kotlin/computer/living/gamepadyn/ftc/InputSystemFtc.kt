package computer.living.gamepadyn.ftc

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import computer.living.gamepadyn.InputData
import computer.living.gamepadyn.InputDataAnalog
import computer.living.gamepadyn.InputDataDigital
import computer.living.gamepadyn.InputSystem
import computer.living.gamepadyn.RawInput

class InputSystemFtc(opMode: OpMode) : InputSystem {

    private val gamepads: Array<RawGamepadFtc> = arrayOf(RawGamepadFtc(opMode.gamepad1), RawGamepadFtc(opMode.gamepad2))

    override fun getGamepads(): Array<out InputSystem.RawGamepad> {
        return gamepads.copyOf()
    }

}

