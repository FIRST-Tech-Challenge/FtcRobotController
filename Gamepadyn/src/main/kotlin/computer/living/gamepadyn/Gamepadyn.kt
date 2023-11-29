package computer.living.gamepadyn

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    internal fun update() {
        val players = arrayOf(player0, player1)
        for (i in players.indices) {
            val player = players[i]

            val llgp = when (i) {
                0 -> opMode.gamepad1
                1 -> opMode.gamepad2
                else -> { TODO("there are more than 2 gamepads somehow") }
            }

            for (bind in player.configuration?.binds!!) {
                val descriptor = actions[bind.targetAction]!!

                val llstate: InputData = when (bind.input) {
                    RawInput.FACE_DOWN          -> InputDataDigital(llgp.a)
                    RawInput.FACE_A             -> InputDataDigital(llgp.a)
                    RawInput.FACE_B             -> InputDataDigital(llgp.b)
                    RawInput.FACE_RIGHT         -> InputDataDigital(llgp.b)
                    RawInput.FACE_LEFT          -> InputDataDigital(llgp.x)
                    RawInput.FACE_X             -> InputDataDigital(llgp.x)
                    RawInput.FACE_UP            -> InputDataDigital(llgp.y)
                    RawInput.FACE_Y             -> InputDataDigital(llgp.y)
                    RawInput.FACE_CROSS         -> InputDataDigital(llgp.cross)
                    RawInput.FACE_CIRCLE        -> InputDataDigital(llgp.circle)
                    RawInput.FACE_SQUARE        -> InputDataDigital(llgp.square)
                    RawInput.FACE_TRIANGLE      -> InputDataDigital(llgp.triangle)
                    RawInput.BUMPER_LEFT        -> InputDataDigital(llgp.left_bumper)
                    RawInput.BUMPER_RIGHT       -> InputDataDigital(llgp.right_bumper)
                    RawInput.DPAD_UP            -> InputDataDigital(llgp.dpad_up)
                    RawInput.DPAD_DOWN          -> InputDataDigital(llgp.dpad_down)
                    RawInput.DPAD_LEFT          -> InputDataDigital(llgp.dpad_left)
                    RawInput.DPAD_RIGHT         -> InputDataDigital(llgp.dpad_right)
                    RawInput.STICK_LEFT_BUTTON  -> InputDataDigital(llgp.left_stick_button)
                    RawInput.STICK_RIGHT_BUTTON -> InputDataDigital(llgp.right_stick_button)
                    RawInput.STICK_LEFT         -> InputDataAnalog(-llgp.left_stick_x, -llgp.left_stick_y)
                    RawInput.STICK_RIGHT        -> InputDataAnalog(-llgp.right_stick_x, -llgp.right_stick_y)
                    RawInput.TRIGGER_LEFT       -> InputDataAnalog(llgp.left_trigger)
                    RawInput.TRIGGER_RIGHT      -> InputDataAnalog(llgp.right_trigger)
                }
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

    @Suppress("MemberVisibilityCanBePrivate")
    internal lateinit var player0: Player<T>
    @Suppress("MemberVisibilityCanBePrivate")
    internal lateinit var player1: Player<T>

    // TODO: multithread input (remove this line)
    init { if (useInputThread) throw Exception("Gamepadyn has no multithreading implementation yet!") }

    /**
     * Alias for Player 0
     */
    val p0: Player<T> = player0 // getPlayer(0)!!

    /**
     * Alias for Player 1
     */
    val p1: Player<T> = player1// getPlayer(1)!!

    /**
     * Returns a reference to the player (virtual device controlled by one person) at the specified index, returning null if
     * @param i The player's index (0 is player 1, 1 is player 2, etc.)
     */
    @Suppress("unused")
    fun getPlayer(@PositiveOrZero i: Int): Player<T>? {
        assert (i >= 0)
        return when (i) {
            0 -> player0
            1 -> player1
            else -> null
        }
    }
}