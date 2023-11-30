package computer.living.gamepadyn

@Suppress("unused")
class BadInputTypeException(val shouldBe: InputType, val actuallyIs: InputType) : Exception()