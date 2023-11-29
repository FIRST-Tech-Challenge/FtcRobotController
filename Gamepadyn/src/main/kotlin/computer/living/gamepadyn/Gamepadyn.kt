package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL
import kotlin.collections.toMap
import javax.validation.constraints.PositiveOrZero

/**
 * Description of an action. This typename is long; it is recommended to use the alias if you are using Kotlin, importable via `gamepadyn.GAD`.
 */
data class ActionDescriptor @JvmOverloads constructor(val type: InputType = DIGITAL, val axis: Int = 0) {
//    init {
//        // NOTE: runtime errors SUCK! ALL GAMEPADYN CODE WILL BE WRITTEN SO THAT AXIS IS IGNORED UNLESS THE ACTION TYPE IS ANALOG!
//        //       HOWEVER, IF AXIS IS NOT 0 FOR A DIGITAL ACTION, THAT IS STILL MALFORMED! This is just for the sake of debugging!
//
//        // assert(type != ActionType.DIGITAL || axis == 0)
//    }
}

typealias GAD = ActionDescriptor

/**
 * A Gamepadyn instance. Construct during the "init" phase of your OpMode.
 *
 * @constructor Creates a Gamepadyn instance.
 * @param opMode A reference to the OpMode where this Gamepadyn instance is running (should always be `this`)
 * @param actions A map of actions that this Gamepadyn instance should be aware of
 * @param useInputThread WIP: Whether to initialize with multithreading.
 */
@Suppress("MemberVisibilityCanBePrivate")
class Gamepadyn<T: Enum<T>> @JvmOverloads constructor(
    internal val inputSystem: InputSystem,
    @JvmField val useInputThread: Boolean = false,
    internal val actions: Map<T, ActionDescriptor?>
) {

    @JvmOverloads constructor(
        inputSystem: InputSystem,
        useInputThread: Boolean = false,
        vararg actions: Pair<T, ActionDescriptor?>
    ) : this(inputSystem, useInputThread, actions.toMap())

    // for calculating delta time
    val lastUpdateTime: Double = 0.0

    var players: ArrayList<Player<T>> = ArrayList(inputSystem.getGamepads().map { Player(this, it) })

    internal fun update() {
        for (i in players.indices) {
            val player = players[i]
            val config = player.configuration
            if (config != null) for (bind in config.binds) {
                val descriptor = actions[bind.targetAction]!!

                val llstate: InputData = inputSystem.getGamepads()[i].getState(bind.input)

                val newData: InputData = bind.transform(llstate, descriptor)

                if (newData.type != descriptor.type) throw Exception("Mismatched transformation result (expected ${descriptor.type.name.lowercase()}, got ${newData.type.name.lowercase()})")
                player.state[bind.targetAction] = newData
                // changes in state trigger events
                if (player.statePrevious[bind.targetAction] != newData) {
                    when (descriptor.type) {
                        DIGITAL -> player.eventsDigital[bind.targetAction]?.trigger(newData as InputDataDigital)
                        ANALOG -> player.eventsAnalog[bind.targetAction]?.trigger(newData as InputDataAnalog)
                    }
                }
            }
            player.statePrevious = player.state.toMap()
        }
    }

//    class MapBuilder() {
//        internal var binds: ArrayList<> =
//        fun analog() {
//
//        }
//
//        fun d() {
//
//        }
//
//        fun build() {
//            return
//        }
//    }
//
//    fun mapBuilder(): MapBuilder {
//        return MapBuilder()
//    }

    // If more than 2 players are ever allowed in the FTC:
    //   1: pigs will fly
    //   2: this code will need to be rewritten

    // TODO: multithread input (remove this line)
    init { if (useInputThread) throw Exception("Gamepadyn has no multithreading implementation yet!") }

    /**
     * Returns a reference to the player (virtual device controlled by one person) at the specified index, returning null if
     * @param i The player's index (0 is player 1, 1 is player 2, etc.)
     */
    @Suppress("unused")
    fun getPlayer(@PositiveOrZero i: Int): Player<T>? {
        assert (i >= 0)
        return players.getOrNull(i)
    }
}