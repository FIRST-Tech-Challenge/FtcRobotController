@file:Suppress("UNCHECKED_CAST")

package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KProperty

/**
 * A delegate class used to simulate a lateinit value; any variable that delegates to this may
 * only be initialized once, but it may be initialized whenever it is desired.
 *
 * __NOTE:__ _This only works for class members,_ and is __Kotlin only__.
 *
 * Usage example:
 * ```
 * class IntegerThatCanOnlyBeSetOnce {
 *   var int: Int by LateInitVal()
 *   // or
 *   var int by LateInitVal<Int>()
 * }
 *
 * //...
 *
 * fun test1() {
 *   val myInt = IntegerThatCanOnlyBeSetOnce()
 *   myInt.int = 1 // this is fine
 *   myInt.int = 2 // throws an exception; int has already been initialized
 * }
 *
 * fun test2() {
 *   val myInt = IntegerThatCanOnlyBeSetOnce()
 *   println(myInt.int) // throws an exception; int is uninitialized
 * }
 * ```
 *
 * @param T The type of the value that is being delegated to.
 * @author KG
 */
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
            throw IllegalStateException("Value is already initialized")
        }
        value = _value
    }
}

/**
 * Object used to represent an uninitialized value. This is used to check if a [LateInitVal] is
 * initialized or not.
 */
private object Uninitialized