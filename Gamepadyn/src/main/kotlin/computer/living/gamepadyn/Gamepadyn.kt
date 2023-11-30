package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL
import computer.living.gamepadyn.ftc.InputSystemFtc
import kotlin.collections.toMap
import javax.validation.constraints.PositiveOrZero

/**
 * Description of an action. This typename is long; it is recommended to use the alias if you are using Kotlin, importable via `gamepadyn.GAD`.
 */
data class ActionDescriptor @JvmOverloads constructor(
    val type: InputType = DIGITAL,
    val axis: Int = 0
)

typealias GAD = ActionDescriptor

/**
 * A Gamepadyn instance.
 *
 * @param inputSystem The backend input source.
 * @param strict If enabled, failures will be loud and catastrophic. Whether that's better than "silent but deadly" is up to you.
 * @param useInputThread WIP: Whether to initialize with multithreading.
 * @param actions A map of actions that this Gamepadyn instance should be aware of
 */
@Suppress("MemberVisibilityCanBePrivate")
class Gamepadyn<T : Enum<T>> @JvmOverloads constructor(
    internal val inputSystem: InputSystem,
    var strict: Boolean = true,
    @JvmField val useInputThread: Boolean = false,
    internal val actions: Map<T, ActionDescriptor?>
) {

    /**
     * Creates a Gamepadyn instance.
     * @param inputSystem The backend input source.
     * @param strict If enabled, failures will be loud and catastrophic. Whether that's better than "silent but deadly" is up to you.
     * @param useInputThread WIP: Whether to initialize with multithreading.
     * @param actions A map of actions that this Gamepadyn instance should be aware of
     */
    @JvmOverloads
    constructor(
        inputSystem: InputSystem,
        strict: Boolean = true,
        useInputThread: Boolean = false,
        vararg actions: Pair<T, ActionDescriptor?>
    ) : this(inputSystem, strict, useInputThread, actions.toMap())

    // for calculating delta time
    // TODO: implement timing
    internal var lastUpdateTime: Double = 0.0

    /**
     * A list of active Players.
     *
     * TODO: This can never update, needs to be fixed.
     */
    val players: ArrayList<Player<T>> = ArrayList(inputSystem.getGamepads().map { Player(this, it) })
//        get() = ArrayList(inputSystem.getGamepads().map { Player(this, it) })

    fun update() {
        val playersIt = players.withIndex()
        // update each player
        for ((i, player) in playersIt) {
            // make the configuration local and constant
            val config = player.configuration

            if (config != null) for (bind in config.binds) {
                val descriptor = actions[bind.targetAction]
                // loud / silent null check
                if (strict) descriptor!!; else if (descriptor == null) break

                val rawState: InputData = inputSystem.getGamepads()[i].getState(bind.input)

                val newData: InputData = bind.transform(rawState, descriptor)

                if (newData.type != descriptor.type)
                    if (strict) throw Exception("Mismatched transformation result (expected ${descriptor.type.name.lowercase()}, got ${newData.type.name.lowercase()})")
                    else break

                player.state[bind.targetAction] = newData

                val previous = player.statePrevious[bind.targetAction]

                // changes in state trigger events
                if (previous != newData /* && previous != null */) when (descriptor.type) {
                    DIGITAL -> player.eventsDigital[bind.targetAction]?.trigger(newData as InputDataDigital)
                    ANALOG -> player.eventsAnalog[bind.targetAction]?.trigger(newData as InputDataAnalog)
                }
            }

            // update state
            player.statePrevious = player.state.toMap()
        }
    }

    init {
        // TODO: multithread input (remove this line)
        if (useInputThread) throw Exception("Gamepadyn has no multithreading implementation yet!")
    }

}