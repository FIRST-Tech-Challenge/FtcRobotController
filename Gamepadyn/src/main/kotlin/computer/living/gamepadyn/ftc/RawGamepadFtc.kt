package computer.living.gamepadyn.ftc

import com.qualcomm.robotcore.hardware.Gamepad
import computer.living.gamepadyn.InputData
import computer.living.gamepadyn.InputDataAnalog
import computer.living.gamepadyn.InputDataDigital
import computer.living.gamepadyn.InputSystem
import computer.living.gamepadyn.RawInput

class RawGamepadFtc(private val gamepad: Gamepad) : InputSystem.RawGamepad {
    override fun getState(input: RawInput): InputData = when (input) {
        RawInput.FACE_DOWN          -> InputDataDigital(gamepad.a)
        RawInput.FACE_A             -> InputDataDigital(gamepad.a)
        RawInput.FACE_B             -> InputDataDigital(gamepad.b)
        RawInput.FACE_RIGHT         -> InputDataDigital(gamepad.b)
        RawInput.FACE_LEFT          -> InputDataDigital(gamepad.x)
        RawInput.FACE_X             -> InputDataDigital(gamepad.x)
        RawInput.FACE_UP            -> InputDataDigital(gamepad.y)
        RawInput.FACE_Y             -> InputDataDigital(gamepad.y)
        RawInput.FACE_CROSS         -> InputDataDigital(gamepad.cross)
        RawInput.FACE_CIRCLE        -> InputDataDigital(gamepad.circle)
        RawInput.FACE_SQUARE        -> InputDataDigital(gamepad.square)
        RawInput.FACE_TRIANGLE      -> InputDataDigital(gamepad.triangle)
        RawInput.BUMPER_LEFT        -> InputDataDigital(gamepad.left_bumper)
        RawInput.BUMPER_RIGHT       -> InputDataDigital(gamepad.right_bumper)
        RawInput.DPAD_UP            -> InputDataDigital(gamepad.dpad_up)
        RawInput.DPAD_DOWN          -> InputDataDigital(gamepad.dpad_down)
        RawInput.DPAD_LEFT          -> InputDataDigital(gamepad.dpad_left)
        RawInput.DPAD_RIGHT         -> InputDataDigital(gamepad.dpad_right)
        RawInput.STICK_LEFT_BUTTON  -> InputDataDigital(gamepad.left_stick_button)
        RawInput.STICK_RIGHT_BUTTON -> InputDataDigital(gamepad.right_stick_button)
        RawInput.STICK_LEFT         -> InputDataAnalog(-gamepad.left_stick_x, -gamepad.left_stick_y)
        RawInput.STICK_RIGHT        -> InputDataAnalog(-gamepad.right_stick_x, -gamepad.right_stick_y)
        RawInput.TRIGGER_LEFT       -> InputDataAnalog(gamepad.left_trigger)
        RawInput.TRIGGER_RIGHT      -> InputDataAnalog(gamepad.right_trigger)
    }
}