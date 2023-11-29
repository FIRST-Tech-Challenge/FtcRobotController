@file:Suppress("MemberVisibilityCanBePrivate")

package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL

class Player<T: Enum<T>> internal constructor(
    internal val parent: Gamepadyn<T>,
    internal var rawGamepad: InputSystem.RawGamepad
) {

    /**
     * The state of the Player at the end of the last update.
     */
    internal var statePrevious: Map<T, InputData> = mapOf()

    /**
     * The current state of every action tracked by the Player.
     */
    internal var state: MutableMap<T, InputData> = parent.actions.entries.associate {
        // these statements proves why Kotlin is a top-tier language. or maybe it just proves that my code is bad? idk
        when (it.value!!.type) {
            ANALOG ->    (it.key to InputDataAnalog())
            DIGITAL ->   (it.key to InputDataDigital())
        }
    }.toMutableMap()

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

    internal val eventsDigital: Map<T, ActionEvent<InputDataDigital>> = parent.actions.entries.filter { it.value!!.type == DIGITAL }
        .associate { it.key to ActionEvent() }

    internal val eventsAnalog: Map<T, ActionEvent<InputDataAnalog>> = parent.actions.entries.filter { it.value!!.type == ANALOG }
        .associate { it.key to ActionEvent() }

    /**
     * The player's configuration.
     */
    var configuration: Configuration<T>? = null
        set(cfg) {
            field = cfg;
            onConfigChanged();
        }


    fun getEvent(action: T): ActionEvent<*>? {
        val descriptor = parent.actions[action]
        return if (descriptor == null) null; else when (descriptor.type) {
                ANALOG -> eventsAnalog[action]
                DIGITAL -> eventsDigital[action]
        }
    }

    // look up "elvis operator kotlin"
    fun getEventAnalog(action: T): ActionEvent<InputDataAnalog>? {
        return eventsAnalog[action]
    }
    fun getEventDigital(action: T): ActionEvent<InputDataDigital>? {
        return eventsDigital[action]
    }

    /**
     * Returns the current state of the provided action (if valid) and `null` if the state doesn't exist or hasn't been updated.
     */
    fun getState(action: T): InputData? = state[action]
    fun getStateAnalog(action: T): InputDataAnalog? {
        val s = state[action]
        return if (s is InputDataAnalog) s; else null
    }
    fun getStateDigital(action: T): InputDataDigital? {
        val s = state[action]
        return if (s is InputDataDigital) s; else null
    }

    internal fun onConfigChanged() {
        // clear state if config is null
        if (configuration == null) {
            state.clear()
        } /*else { }*/ // TODO: nullify elements if their mappings were removed
    }

//    internal fun updateState() {
//        parent.opMode.gamepad1
//    }
//    val onUpdate: Event<>

}