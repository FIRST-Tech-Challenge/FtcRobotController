@file:Suppress("FunctionName")

package ftc.rogue.blacksmith.internal.blackop

import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.annotations.CreateOnGo
import ftc.rogue.blacksmith.internal.util.getFieldsAnnotatedWith
import kotlin.reflect.KProperty

@PublishedApi
internal class CreationException(message: String) : RuntimeException(message)

@PublishedApi
internal inline fun <reified T> CreateOnGoInternal(vararg args: () -> Any) =
    CreateOnGoInternal {
        val clazz = T::class.java

        val invokedArgs = args
            .map { it() }
            .toTypedArray()

        val argTypes = invokedArgs
            .map { it::class.java }
            .toTypedArray()

        val constructor = clazz.constructors
            .find { constructor ->
                constructor.parameterTypes contentEquals argTypes
            }

        constructor?.newInstance(*invokedArgs) as? T
            ?: throw CreationException("No constructor found for $clazz with args $argTypes")
    }

class CreateOnGoInternal<T : Any>
    @PublishedApi
    internal constructor(
        constructor: () -> T
    ) {

    private lateinit var value: T

    init {
        Scheduler.on(BlackOp.STARTING_MSG) { value = constructor() }
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>): T {
        if (!::value.isInitialized) {
            throw IllegalStateException("createOnGo value is uninitialized (OpMode not started yet)")
        }
        return value
    }
}

fun BlackOp.injectCreateOnGoFields() = this::class.java
    .getFieldsAnnotatedWith(CreateOnGo::class.java)
    .forEach { field ->
        val clazz = field.type

        val containsNoArgConstructor = clazz.constructors
            .none { it.parameterTypes.isEmpty() }

        if (!containsNoArgConstructor) {
            throw CreationException("Class '${clazz.simpleName}' has no no-arg constructor")
        }

        field.isAccessible = true

        if (field.getAnnotation(CreateOnGo::class.java)?.passHwMap == true) {
            field.set(this, clazz.getConstructor().newInstance(BlackOp.hwMap))
        } else {
            field.set(this, clazz.getConstructor().newInstance())
        }
    }
