@file:Suppress("UNCHECKED_CAST")

package ftc.rogue.blacksmith.util.kt

import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import kotlin.reflect.KProperty

inline fun <reified T> createOnGo(vararg args: () -> Any) =
    CreateOnStartR(T::class.java, *args)

@JvmSynthetic
fun <T> createOnGo(constructor: () -> T) =
    CreateOnStartL(constructor)

class CreateOnStartR<T> @PublishedApi internal constructor(
    clazz: Class<T>,
    vararg args: () -> Any,
) {
    private var value: Any? = Uninitialized

    init {
        Scheduler.on(BlackOp.STARTING_MSG) {
            val invokedArgs = args.map { it() }.toTypedArray()
            value = clazz.constructors.first().newInstance(*invokedArgs)
        }
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>): T {
        if (value is Uninitialized) {
            throw IllegalStateException("Value is uninitialized (Not started yet)")
        }
        return value as T
    }

    companion object {
        private object Uninitialized
    }
}

class CreateOnStartL<T> internal constructor(
    constructor: () -> T,
) {
    private var value: Any? = Uninitialized

    init {
        Scheduler.on(BlackOp.STARTING_MSG) {
            value = constructor()
        }
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>): T {
        if (value is Uninitialized) {
            throw IllegalStateException("Value is uninitialized (Not started yet)")
        }
        return value as T
    }

    companion object {
        private object Uninitialized
    }
}
