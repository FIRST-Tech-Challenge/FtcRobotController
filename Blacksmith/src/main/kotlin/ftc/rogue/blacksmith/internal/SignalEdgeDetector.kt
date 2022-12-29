@file:Suppress("MemberVisibilityCanBePrivate")

package ftc.rogue.blacksmith.internal

/**
 * Evaluates a condition, and checks if the state of the condition is on the rising edge, falling
 * edge, is high, and/or is low.
 *
 * @param condition The condition to evaluate.
 *
 * @author KG
 *
 * @see Condition
 * @see SignalTrigger
 */
class SignalEdgeDetector(private val condition: () -> Boolean) {
    /**
     * The current state of the condition
     */
    var currState = condition()
        private set

    /**
     * The previous state of the condition
     */
    private var lastState = currState

    /**
     * Evaluates the condition and updates the state.
     */
    fun update() {
        lastState = currState
        currState = condition()
    }

    /**
     * Checks if the condition is on the rising edge, i.e. it goes from `false` to `true`.
     * @return True if the condition is on the rising edge, false otherwise.
     */
    fun risingEdge() = !lastState && currState

    /**
     * Checks if the condition is on the falling edge, i.e. it goes from `true` to `false`.
     * @return True if the condition is on the falling edge, false otherwise.
     */
    fun fallingEdge() = lastState && !currState

    /**
     * Checks if the condition is high.
     * @return True if the condition is true, false otherwise.
     */
    fun isHigh() = currState

    /**
     * Checks if the condition is low.
     * @return True if the condition is false, false otherwise.
     */
    fun isLow() = !currState
}
