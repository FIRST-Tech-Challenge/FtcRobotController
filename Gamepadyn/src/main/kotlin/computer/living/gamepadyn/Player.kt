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
    internal var statePrevious: Map<T, InputData> = mapOf()
    /**
     * The current state of the Player
     */
    internal var state: MutableMap<T, InputData> = parent.actions.entries.associate {
        // these statements proves why Kotlin is a top-tier language. or maybe it just proves that my code is bad? idk
        when (it.value!!.type) {
            ANALOG ->    (it.key to InputDataAnalog())
            DIGITAL ->   (it.key to InputDataDigital())
        }
    }.toMutableMap()

    // for some reason, association inherently applies site-varied type projections to ActionEvent's parameter. I don't know how to stop this.
    internal val events: Map<T, ActionEvent<InputData>> = parent.actions.entries.associate {
        when (it.value!!.type) {
            ANALOG ->    (it.key to ActionEvent<InputDataAnalog>(ANALOG))
            DIGITAL ->   (it.key to ActionEvent<InputDataDigital>(DIGITAL))
        }
    } as Map<T, ActionEvent<InputData>>

    /**
     * The player's configuration.
     */
    var configuration: Configuration<T>? = null
        set(cfg) {
            field = cfg;
            onConfigChanged();
        }

    // look up "elvis operator kotlin"
    fun getEventAnalog(action: T): ActionEvent<out InputDataAnalog>? {
        val ev: ActionEvent<*> = events[action] ?: return null
        val shouldBe = ev.type
        val actuallyIs = parent.actions[action]!!.type
        if (shouldBe != actuallyIs) throw BadInputTypeException(shouldBe, actuallyIs)

        @Suppress("UNCHECKED_CAST")
        return ev as? ActionEvent<out InputDataAnalog>
    }
    fun getEventDigital(action: T): ActionEvent<out InputDataDigital>? {
        val ev: ActionEvent<*> = events[action] ?: return null
        val shouldBe = ev.type
        val actuallyIs = parent.actions[action]!!.type
        if (shouldBe != actuallyIs) throw BadInputTypeException(shouldBe, actuallyIs)

        @Suppress("UNCHECKED_CAST")
        return ev as? ActionEvent<out InputDataDigital>
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