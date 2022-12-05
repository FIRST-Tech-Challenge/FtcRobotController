@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package org.firstinspires.ftc.teamcodekt.components.scheduler.listeners

import org.firstinspires.ftc.teamcodekt.scheduler.Scheduler
import org.firstinspires.ftc.teamcodekt.scheduler.Condition
import org.firstinspires.ftc.teamcodekt.scheduler.SignalEdgeDetector
import org.firstinspires.ftc.teamcodekt.util.runOnce

/**
 * A component that  performs the set of subscribed [actions][Runnable] when the state of the
 * [condition][Condition] matches the signal state that the action was
 * subscribed on (e.g. `on rising edge` or `while low`).
 *
 * The listener is updated on every tick of the [Scheduler] when hooked.
 *
 * _The listener is not hooked by default; it is only hooked once it has actions subscribed to it._
 * This means that it is safe to create excess unused [Listeners][Listener] without worrying about
 * performance burdens.
 *
 * Java raw usage example:
 * ```java
 * @Override
 * public void runOpMode() throws InterruptedException {
 *     Listener listener = new Listener(() -> someCondition == true);
 *
 *     // Runs when the condition changes from false to true
 *     listener.onRise(this::doSomething);
 *
 *     // Runs while the condition is true
 *     listener.whileHigh(this::doSomethingElse);
 *
 *     // Runs when the condition changes from true to false
 *     listener.onFall(this::doEomethingSlse);
 *
 *     // Runs while the condition is false
 *     listener.whileLow(this::doElseSomething);
 *
 *     // Must be called to start the main loop.
 *     Scheduler.start(this);
 * }
 * ```
 *
 * @param condition The condition that must be met for the action to be performed.
 *
 * @author KG
 *
 * @see Scheduler
 * @see GamepadEx2
 * @see Timer
 */
class Listener(val condition: Condition) {
    /**
     * The subscribed set of [actions][Runnable] that are performed when the given
     * condition's state matches the given [SignalTrigger][SignalTrigger].
     */
    private val actions = mutableMapOf<Runnable, Condition>()

    /**
     * A listener that evaluates the [condition][Condition] and checks whether the state of the
     * signal is [high][SignalTrigger.IS_HIGH], [low][SignalTrigger.IS_LOW],
     * [rising][SignalTrigger.RISING_EDGE], and/or [falling][SignalTrigger.FALLING_EDGE].
     */
    private var conditionSED = SignalEdgeDetector(condition)

    /**
     * Schedules the given action to run when the trigger [condition][Condition] changes from false to true.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun onRise(action: Runnable) = this.also {
        hookIfNotHooked()
        actions[action] = conditionSED::risingEdge as Condition
    }

    /**
     * Schedules the given action to run while the trigger [condition][Condition] changes from true to false.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun onFall(action: Runnable) = this.also {
        hookIfNotHooked()
        actions[action] = conditionSED::risingEdge as Condition
    }

    /**
     * Schedules the given action to run when the trigger [condition][Condition] is true.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun whileHigh(action: Runnable) = this.also {
        hookIfNotHooked()
        actions[action] = conditionSED::risingEdge as Condition
    }

    /**
     * Schedules the given action to run while the trigger [condition][Condition] is false.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun whileLow(action: Runnable) = this.also {
        hookIfNotHooked()
        actions[action] = conditionSED::risingEdge as Condition
    }

    /**
     * Hooks this listener to the [Scheduler] if it has not already been hooked.
     */
    private val hookIfNotHooked = runOnce {
        Scheduler.hookListener(this)
    }

    /**
     * Runs every tick to update the [conditionSED] and perform the subscribed actions.
     */
    fun tick() {
        conditionSED.update()
        doActiveActions()
    }

    /**
     * Performs the actions who's conditions evaluate to true.
     */
    fun doActiveActions(): Unit =
        actions.forEach { (action, condition) ->
            if (condition.evaluate()) action.run()
        }

    /**
     * Evaluates and returns the current state of the listener's [Condition].
     *
     * @return `true` if the condition is true, `false` otherwise.
     */
    fun evaluateCondition(): Boolean = condition.evaluate()

    // ---------------------------------------------------------------
    // Listener builders
    // ---------------------------------------------------------------

    fun and(other: Condition) = Listener { condition.evaluate() && other.evaluate() }

    fun or(other: Condition) = Listener { condition.evaluate() || other.evaluate() }

    fun xor(other: Condition) = Listener { condition.evaluate() xor other.evaluate() }

    fun nand(other: Condition) = Listener { !(condition.evaluate() && other.evaluate()) }

    fun nor(other: Condition) = Listener { !(condition.evaluate() || other.evaluate()) }

    fun xnor(other: Condition) = Listener { condition.evaluate() == other.evaluate() }


    fun and(other: Listener) = Listener { condition.evaluate() && other.evaluateCondition() }

    fun or(other: Listener) = Listener { condition.evaluate() || other.evaluateCondition() }

    fun xor(other: Listener) = Listener { condition.evaluate() xor other.evaluateCondition() }

    fun nand(other: Listener) = Listener { !(condition.evaluate() && other.evaluateCondition()) }

    fun nor(other: Listener) = Listener { !(condition.evaluate() || other.evaluateCondition()) }

    fun xnor(other: Listener) = Listener { condition.evaluate() == other.evaluateCondition() }


    fun not() = Listener { !condition.evaluate() }
}
