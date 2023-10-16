package computer.living.gamepadyn
import java.util.stream.Collectors.toMap

/**
 * Thunk Action
 *
 * TODO: try and make this some sort of "map builder" (like `ImmutableMap` in Guava?)
 */
class Tak<T : Enum<T>> private constructor(val action: T, val type: ActionType, val axis: Int) {
    companion object {
        /**
         * Creates an analog Tak
         */
        @JvmStatic fun <T : Enum<T>> a(action: T, axis: Int): Tak<T> {
            return Tak(action, ActionType.ANALOG, axis)
        }
        /**
         * Creates a digital Tak
         */
        @JvmStatic fun <T : Enum<T>> d(action: T): Tak<T> {
            return Tak(action, ActionType.DIGITAL, 0)
        }

        @JvmStatic fun <T : Enum<T>> makeActionMap(taks: List<Tak<T>>): Map<T, ActionDescriptor> {
            return taks.associate { it.action to ActionDescriptor(it.type, it.axis) }
        }

//        @JvmStatic /* @SafeVarargs */ fun <T : Enum<T>> makeActionMap(vararg taks: Tak<T>): Map<T, ActionDescriptor> {
//            return taks.associate { it.action to ActionDescriptor(it.type, it.axis) }
//        }
    }
}