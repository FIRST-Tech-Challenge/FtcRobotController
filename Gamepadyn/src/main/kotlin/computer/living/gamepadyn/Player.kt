@file:Suppress("MemberVisibilityCanBePrivate")

package computer.living.gamepadyn

sealed class Player<T: Enum<T>> private constructor(
    internal val parent: Gamepadyn<T>
) {

    internal var data: MutableMap<T, ActionData> = mutableMapOf()

    /**
     * Returns the current state of the provided action (if valid) and `null` if the state doesn't exist or hasn't been updated.
     *
     * NOTE: This is problematic. Without the ability to verify that TUA is an enum,
     * we have no way of enumerating all of its values,
     * and therefore cannot easily fill the map.
     * This has no easy solution. I've tried my best to make it better, but both Java and Kotlin have no easy way of reflecting or extending in the way I want.
     */
    fun getState(action: T): ActionData? = data[action]

//    internal fun updateState() {
//        parent.opMode.gamepad1
//    }
//    val onUpdate: Event<>

}