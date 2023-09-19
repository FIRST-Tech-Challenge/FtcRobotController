package computer.living.gamepadyn

import java.util.EnumMap
import kotlin.reflect.*
import kotlin.reflect.full.staticProperties
import kotlin.reflect.jvm.internal.impl.load.kotlin.JvmType

enum class ETester(
    val testProp: Int = 0
) {
    VALUE_A,
    VALUE_B(42);
}

interface IUserAction {
    val axes: Int
}