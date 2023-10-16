package computer.living.gamepadyn

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import kotlin.collections.toMap
import kotlin.collections.toMutableMap
import java.util.EnumMap
import javax.validation.constraints.PositiveOrZero

/**
 * Description of an action. This typename is long; it is recommended to use the alias if you are using Kotlin, importable via `gamepadyn.GAD`.
 */
data class ActionDescriptor @JvmOverloads constructor(val type: ActionType = ActionType.DIGITAL, val axis: Int = 0) {
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
    internal val opMode: OpMode,
    @JvmField val useInputThread: Boolean = false,
    internal val actions: Map<T, ActionDescriptor?>
) {
    @JvmOverloads constructor(
        opMode: OpMode,
        useInputThread: Boolean = false,
        vararg actions: Pair<T, ActionDescriptor?>
    ) : this(opMode, useInputThread, actions.toMap())

    // for calculating delta time
    val lastUpdateTime: Double = 0.0

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

    @Suppress("MemberVisibilityCanBePrivate")
    internal lateinit var player1: Player<T>
    @Suppress("MemberVisibilityCanBePrivate")
    internal lateinit var player2: Player<T>

    // TODO: multithread input (remove this line)
    init { if (useInputThread) throw Error("Gamepadyn has no multithreading implementation yet!") }

    /**
     * Returns a reference to the player (virtual device controlled by one person) at the specified index, returning null if
     * @param i The player's index (0 is player 1, 1 is player 2, etc.)
     */
    @Suppress("unused")
    fun getPlayer(@PositiveOrZero i: Int): Player<T>? {
        assert (i >= 0)
        return when (i) {
            0 -> player1
            1 -> player2
            else -> null
        }
    }
}