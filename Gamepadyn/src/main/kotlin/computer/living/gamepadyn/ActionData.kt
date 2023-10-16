@file:Suppress("MemberVisibilityCanBePrivate")

package computer.living.gamepadyn

/**
 * The state of any action is representable by an implementation of this interface.
 */
sealed interface ActionData { val type: ActionType }

/**
 * Represents the value of a digital action.
 * This is effectively a Boolean that implements ActionData.
 */
data class ActionDataDigital(var digitalData: Boolean = false): ActionData { override val type = ActionType.DIGITAL  }

/**
 * Represents the value of an analog action.
 * @property analogData The action data. The size of the array is equal to the amount of axes the action has.
 */
class ActionDataAnalog(
    vararg val analogData: Float
): ActionData {
    override val type = ActionType.ANALOG

    val axes: Int get() { return analogData.size; }

    init { assert(analogData.isNotEmpty()) }
}