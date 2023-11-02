package computer.living.gamepadyn

class BadInputTypeException(val shouldBe: InputType, val actuallyIs: InputType) : Exception()