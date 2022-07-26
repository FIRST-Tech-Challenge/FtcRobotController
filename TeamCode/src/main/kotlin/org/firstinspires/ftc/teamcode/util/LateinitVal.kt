@file:Suppress("UNCHECKED_CAST")

package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KProperty

inline fun <reified T> initializableOnce(): LateInitVal<T> = LateInitVal()

class LateInitVal<T> {
    private var value: Any? = Uninitialized

    operator fun getValue(thisRef: Any, property: KProperty<*>): T {
        if (value is Uninitialized) {
            throw IllegalStateException("Value is uninitialized")
        }
        return value as T
    }

    operator fun setValue(thisRef: Any, property: KProperty<*>, _value: T) {
        if (value !is Uninitialized) {
            throw IllegalStateException("Value is initialized")
        }
        value = _value
    }
}

private object Uninitialized