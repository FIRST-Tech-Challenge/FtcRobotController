package computer.living.gamepadyn

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import kotlin.reflect.KClass

// Generic name somewhat breaks style guidelines, but the use of the generic "T" would be incredibly confusing
/**
 * A Gamepadyn instance. Construct during the "init" phase of your OpMode.
 *
 * @constructor Creates a Gamepadyn instance.
 * @param opMode A reference to the OpMode where this Gamepadyn instance is running (should always be `this`)
 * @param actionList A list of actions that this Gamepadyn instance should be aware of (should always be `someEnum.values()`)
 * @param useInputThread WIP: Whether to initialize with multithreading.
 */
@Suppress("MemberVisibilityCanBePrivate")
class Gamepadyn<TUA: IUserAction> @JvmOverloads constructor(
    internal var opMode: OpMode,
    internal val actionList: Array<TUA>,
    val useInputThread: Boolean = false
) {

    // If more than 2 players are ever allowed in the FTC:
    //   1: pigs will fly
    //   2: this code will need to be rewritten

    @Suppress("MemberVisibilityCanBePrivate")
    internal lateinit var player1: Player<TUA>
    @Suppress("MemberVisibilityCanBePrivate")
    internal lateinit var player2: Player<TUA>

    // TODO: multithread input (remove this line)
    init { if (useInputThread) throw Error("Gamepadyn has no multithreading implementation yet!") }

    /**
     * Returns a reference to the player (virtual device controlled by one person) at the specified index, returning null if
     * @param i The player's index (0 is player 1, 1 is player 2, etc.)
     */
    @Suppress("unused")
    fun getPlayer(i: Int): Player<TUA>? {
        assert (i >= 0)
        return when (i) {
            0 -> player1
            1 -> player2
            else -> null
        }
    }
}