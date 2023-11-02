@file:Suppress("MemberVisibilityCanBePrivate")

package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL

sealed class Player<T: Enum<T>> private constructor(
    internal val parent: Gamepadyn<T>
) {

    /**
     * The last state of the Player
     */
    internal var statePrevious: MutableMap<T, ActionData> = mutableMapOf()
    /**
     * The current state of the Player
     */
    internal var state: MutableMap<T, ActionData> = parent.actions.entries.associate {
        // these statements proves why Kotlin is a top-tier language. or maybe it just proves that my code is bad? idk
        when (it.value!!.type) {
            ANALOG ->    (it.key to ActionDataAnalog())
            DIGITAL ->   (it.key to ActionDataDigital())
        }
    }.toMutableMap()

    private val events: Map<T, ActionEvent<out ActionData>> = parent.actions.entries.associate {
        when (it.value!!.type) {
            ANALOG ->    (it.key to ActionEvent<ActionDataAnalog>(ANALOG))
            DIGITAL ->   (it.key to ActionEvent<ActionDataDigital>(DIGITAL))
        }
    }

    // look up "elvis operator kotlin"
    fun getEventAnalog(action: T): ActionEvent<ActionDataAnalog>? {
        val ev = events[action] ?: return null
        val shouldBe = ev.type
        val actuallyIs = parent.actions[action]!!.type
        if (shouldBe != actuallyIs) throw BadInputTypeException(shouldBe, actuallyIs)
        @Suppress("UNCHECKED_CAST")
        return ev as ActionEvent<ActionDataAnalog>
    }
    fun getEventDigital(action: T): ActionEvent<ActionDataDigital>? {
        val ev = events[action] ?: return null
        val shouldBe = ev.type
        val actuallyIs = parent.actions[action]!!.type
        if (shouldBe != actuallyIs) throw BadInputTypeException(shouldBe, actuallyIs)
        @Suppress("UNCHECKED_CAST")
        return ev as ActionEvent<ActionDataDigital>
    }

    /**
     * The player's configuration.
     */
    var configuration: Configuration<T>? = null
        set(cfg) {
            field = cfg;
            onConfigChanged();
        }

    internal fun onConfigChanged() {
        // clear state if config is null
        if (configuration == null) {
            state.clear()
        } /*else { }*/ // TODO: nullify elements if their mappings were removed
    }

    /**
     * Returns the current state of the provided action (if valid) and `null` if the state doesn't exist or hasn't been updated.
     */
    fun getState(action: T): ActionData? = state[action]

//    internal fun updateState() {
//        parent.opMode.gamepad1
//    }
//    val onUpdate: Event<>

}