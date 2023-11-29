package computer.living.gamepadyn

class InputSystemTesting : InputSystem {

    companion object {
        var manipulableState: Boolean = false
    }

    class RawGamepadTesting: InputSystem.RawGamepad {
        override fun getState(input: RawInput): InputData = when (input) {
            RawInput.FACE_DOWN          -> InputDataDigital(manipulableState)
            RawInput.FACE_A             -> InputDataDigital(manipulableState)
            RawInput.FACE_B             -> InputDataDigital(manipulableState)
            RawInput.FACE_RIGHT         -> InputDataDigital(manipulableState)
            RawInput.FACE_LEFT          -> InputDataDigital(manipulableState)
            RawInput.FACE_X             -> InputDataDigital(manipulableState)
            RawInput.FACE_UP            -> InputDataDigital(manipulableState)
            RawInput.FACE_Y             -> InputDataDigital(manipulableState)
            RawInput.FACE_CROSS         -> InputDataDigital(manipulableState)
            RawInput.FACE_CIRCLE        -> InputDataDigital(manipulableState)
            RawInput.FACE_SQUARE        -> InputDataDigital(manipulableState)
            RawInput.FACE_TRIANGLE      -> InputDataDigital(manipulableState)
            RawInput.BUMPER_LEFT        -> InputDataDigital(manipulableState)
            RawInput.BUMPER_RIGHT       -> InputDataDigital(manipulableState)
            RawInput.DPAD_UP            -> InputDataDigital(manipulableState)
            RawInput.DPAD_DOWN          -> InputDataDigital(manipulableState)
            RawInput.DPAD_LEFT          -> InputDataDigital(manipulableState)
            RawInput.DPAD_RIGHT         -> InputDataDigital(manipulableState)
            RawInput.STICK_LEFT_BUTTON  -> InputDataDigital(manipulableState)
            RawInput.STICK_RIGHT_BUTTON -> InputDataDigital(manipulableState)
            RawInput.STICK_LEFT         -> InputDataAnalog(0f, 0f)
            RawInput.STICK_RIGHT        -> InputDataAnalog(0f, 0f)
            RawInput.TRIGGER_LEFT       -> InputDataAnalog(0f)
            RawInput.TRIGGER_RIGHT      -> InputDataAnalog(0f)
        }
    }

    private val gamepads: Array<RawGamepadTesting> = arrayOf(RawGamepadTesting())

    override fun getGamepads(): Array<out InputSystem.RawGamepad> {
        return gamepads.copyOf()
    }

}

