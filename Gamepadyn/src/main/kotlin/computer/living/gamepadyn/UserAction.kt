package computer.living.gamepadyn

enum class ETester(
    val testProp: Int = 0
) {
    VALUE_A,
    VALUE_B(42);
}

interface UserAction {
    val axes: Int
}