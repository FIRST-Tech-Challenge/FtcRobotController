package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL

/**
 * TODO: because compile-time checking is not feasible (rip), convert these classes to enum constants or something we can use to get the type to cast to
 * TODO 2: add flexibility and freedom via a generic binding interface?
 */

@Target(AnnotationTarget.VALUE_PARAMETER)
annotation class MustBeAnalog
@Target(AnnotationTarget.VALUE_PARAMETER)
annotation class MustBeDigital

/*
 * a bind could maybe be a lambda? it should transform raw input and produce action output.
 * how do we do individual axes? this is gonna be hard to decide upon.
 *
 * how about returning information about how to transform an action? like an analog action could return an array of `ActionDataAnalog?`s
 */

sealed class ActionBind<T: Enum<T>> {
    abstract val dataType: InputType
    var targetAction: T? = null
}
abstract class ActionBindAnalog<T: Enum<T>> : ActionBind<T>() {
    final override val dataType = ANALOG
    abstract fun transform(): ActionDataDigital
}
abstract class ActionBindDigital<T: Enum<T>> : ActionBind<T>() {
    final override val dataType = DIGITAL
    abstract fun transform(): Array<ActionDataAnalog?>
}
