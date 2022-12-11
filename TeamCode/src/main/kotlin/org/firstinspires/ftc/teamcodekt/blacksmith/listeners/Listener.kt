@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package org.firstinspires.ftc.teamcodekt.blacksmith.listeners

import org.firstinspires.ftc.teamcodekt.blacksmith.Scheduler
import org.firstinspires.ftc.teamcodekt.blacksmith.internal.Condition
import org.firstinspires.ftc.teamcodekt.blacksmith.internal.SignalEdgeDetector
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
 * @see ReforgedGamepad
 * @see Timer
 */
open class Listener(val condition: Condition) {
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
        actions[action] = conditionSED::fallingEdge as Condition
    }

    /**
     * Schedules the given action to run when the trigger [condition][Condition] is true.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun whileHigh(action: Runnable) = this.also {
        hookIfNotHooked()
        actions[action] = conditionSED::isHigh as Condition
    }

    /**
     * Schedules the given action to run while the trigger [condition][Condition] is false.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun whileLow(action: Runnable) = this.also {
        hookIfNotHooked()
        actions[action] = conditionSED::isLow as Condition
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

    companion object {
        private val alwaysActiveListener = Listener { true }

        /**
         * Subscribes the given action to run every tick.
         * @param action The action to run.
         * @return This [Listener] instance.
         */
        @JvmStatic
        fun always(action: Runnable) = alwaysActiveListener.whileHigh(action)
    }

    // ---------------------------------------------------------------
    // Listener builders
    // ---------------------------------------------------------------

    fun and(otherCondition: Condition) = Listener { condition() && otherCondition() }

    fun or(otherCondition: Condition) = Listener { condition() || otherCondition() }

    fun xor(otherCondition: Condition) = Listener { condition() xor otherCondition() }

    fun nand(otherCondition: Condition) = Listener { !(condition() && otherCondition()) }

    fun nor(otherCondition: Condition) = Listener { !(condition() || otherCondition()) }

    fun xnor(otherCondition: Condition) = Listener { condition() == otherCondition() }

    operator fun plus(otherCondition: Condition) = and(otherCondition)

    operator fun div(otherCondition: Condition) = or(otherCondition)


    fun and(other: Listener) = Listener { condition() && other.condition() }

    fun or(other: Listener) = Listener { condition() || other.condition() }

    fun xor(other: Listener) = Listener { condition() xor other.condition() }

    fun nand(other: Listener) = Listener { !(condition() && other.condition()) }

    fun nor(other: Listener) = Listener { !(condition() || other.condition()) }

    fun xnor(other: Listener) = Listener { condition() == other.condition() }

    operator fun plus(other: Listener) = and(other)

    operator fun div(other: Listener) = or(other)


    operator fun not() = Listener { !condition.evaluate() }
}
