package computer.living.gamepadyn

interface InputSystem {
    interface RawGamepad {
        fun getState(input: RawInput): InputData
    }

    fun getGamepads(): Array<out RawGamepad>
}