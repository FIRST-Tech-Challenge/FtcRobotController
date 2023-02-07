package ftc.rogue.blacksmith.internal

import kotlin.reflect.KProperty

class NotNull<T : Any>(private val errorMsg: String) {
    private var value: T? = null

    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return value
            ?: throw IllegalStateException(
                errorMsg
            )
    }

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        this.value = value
    }
}
