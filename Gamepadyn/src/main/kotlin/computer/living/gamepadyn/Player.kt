@file:Suppress("MemberVisibilityCanBePrivate")

package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL

class Player<T : Enum<T>> internal constructor(
    internal val parent: Gamepadyn<T>,
    internal var rawGamepad: InputSystem.RawGamepad
) {

    /**
     * The current state of every action tracked by the Player.
     */
    internal var state: MutableMap<T, InputData> = parent.actions.entries.associate {
        // these statements proves why Kotlin is a top-tier language. or maybe it just proves that my code is bad? idk
        return@associate when (it.value!!.type) {
            ANALOG -> (it.key to (if (it.value!!.axes == 1) InputDataAnalog(0f) else InputDataAnalog(
                0f,
                *FloatArray(it.value!!.axes - 1).toTypedArray()
            )))

            DIGITAL -> (it.key to InputDataDigital())
        }
    }.toMutableMap()

    /**
     * The state of the Player at the end of the last update.
     */
    internal var statePrevious: Map<T, InputData> = state

        /**
     * These two maps have to be separate due to Kotlin's rules on generics.
     * This doesn't really affect the user, but here it means that you need to put a bit more work into type checking.
     * In short, generics cannot be checked at runtime unless reified, but you can't reify class generic parameters.
     * If we had 1 map with both analog and digital events, we would need to create a mechanism for type-checking at runtime.
     * Previously, this came in the form of a "dataType" parameter on the ActionEvent. It no longer exists.
     *
     *
     * As much as I'd like for us to have everything work perfectly at runtime and compile-time, we have to make compromises.
     */

    internal val eventsDigital: Map<T, ActionEvent<InputDataDigital>> =
        parent.actions.entries.filter { it.value!!.type == DIGITAL }
            .associate { it.key to ActionEvent() }

    internal val eventsAnalog: Map<T, ActionEvent<InputDataAnalog>> =
        parent.actions.entries.filter { it.value!!.type == ANALOG }
            .associate { it.key to ActionEvent() }

    /**
     * The player's configuration.
     */
    var configuration: Configuration<T>? = null


    fun getEvent(action: T): ActionEvent<*>? {
        val descriptor = parent.actions[action]
        return if (descriptor == null) null; else when (descriptor.type) {
            ANALOG ->   eventsAnalog[action]
            DIGITAL ->  eventsDigital[action]
        }
    }

    fun getEventAnalog(action: T): ActionEvent<InputDataAnalog>? = eventsAnalog[action]
    fun getEventDigital(action: T): ActionEvent<InputDataDigital>? = eventsDigital[action]

    /**
     * Returns the current state of the provided action (if valid) and `null` if the state doesn't exist or hasn't been updated.
     */
    fun getState(action: T): InputData? = state[action]
    fun getStateAnalog(action: T): InputDataAnalog? {
        val s = state[action]
        return if (s != null && s is InputDataAnalog) s; else null
    }
    fun getStateDigital(action: T): InputDataDigital? {
        val s = state[action]
        return if (s != null && s is InputDataDigital) s; else null
    }

}