@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.scheduler

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
class SignalEdgeDetector(private val condition: Condition) {
    /**
     * The current state of the condition
     */
    var currState = false
        private set

    /**
     * The previous state of the condition
     */
    private var lastState = false

    /**
     * Evaluates the condition and updates the state.
     */
    fun update() {
        lastState = currState
        currState = condition.evaluate()
    }

    /**
     * Checks if the condition is on the rising edge.
     * @return True if the condition is on the rising edge, false otherwise.
     */
    fun risingEdge() = currState && !lastState

    /**
     * Checks if the condition is on the falling edge.
     * @return True if the condition is on the falling edge, false otherwise.
     */
    fun fallingEdge() = !currState && lastState

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
