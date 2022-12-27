@file:Suppress("UNCHECKED_CAST")

package ftc.rouge.blacksmith.util.kt

import ftc.rouge.blacksmith.BlackOp
import ftc.rouge.blacksmith.messenger.Messenger
import kotlin.reflect.KProperty

inline fun <reified T> createOnStart(vararg args: () -> Any) =
    CreateOnStartR(T::class.java, *args)

@JvmSynthetic
fun <T> createOnStart(constructor: () -> T) =
    CreateOnStartL(constructor)

class CreateOnStartR<T> @PublishedApi internal constructor(
    clazz: Class<T>,
    vararg args: () -> Any,
) {
    private var value: Any? = Uninitialized

    init {
        Messenger.on(BlackOp.STARTING_MSG) {
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
        Messenger.on(BlackOp.STARTING_MSG) {
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
