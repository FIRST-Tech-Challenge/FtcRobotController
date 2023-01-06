@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package ftc.rogue.blacksmith.listeners

import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.internal.SignalEdgeDetector
import ftc.rogue.blacksmith.util.runOnce

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
 *     Scheduler.launch(this);
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
open class Listener(val condition: () -> Boolean) {
    /**
     * The subscribed set of [actions][Runnable] that are performed when the given
     * condition's state matches the given [SignalTrigger][SignalTrigger].
     */
    private val actions = mutableMapOf<Runnable, () -> Boolean>()

    /**
     * A listener that evaluates the [condition][Condition] and checks whether the state of the
     * signal is [high][SignalTrigger.IS_HIGH], [low][SignalTrigger.IS_LOW],
     * [rising][SignalTrigger.RISING_EDGE], and/or [falling][SignalTrigger.FALLING_EDGE].
     */
    private val conditionSED = SignalEdgeDetector(condition)

    /**
     * Schedules the given action to run when the trigger [condition][Condition] changes from false to true.
     * @param callback The action to run.
     * @return This [Listener] instance.
     */
    fun onRise(callback: Runnable) = this.also {
        hookIfNotHooked()
        actions[callback] = conditionSED::risingEdge
    }

    /**
     * Schedules the given action to run while the trigger [condition][Condition] changes from true to false.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun onFall(callback: Runnable) = this.also {
        hookIfNotHooked()
        actions[callback] = conditionSED::fallingEdge
    }

    /**
     * Schedules the given action to run when the trigger [condition][Condition] is true.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun whileHigh(callback: Runnable) = this.also {
        hookIfNotHooked()
        actions[callback] = conditionSED::isHigh
    }

    /**
     * Schedules the given action to run while the trigger [condition][Condition] is false.
     * @param action The action to run.
     * @return This [Listener] instance.
     */
    fun whileLow(callback: Runnable) = this.also {
        hookIfNotHooked()
        actions[callback] = conditionSED::isLow
    }

    fun onJustTrue(callback: Runnable) = onRise(callback)
    fun onJustFalse(callback: Runnable) = onFall(callback)
    fun whileTrue(callback: Runnable) = whileHigh(callback)
    fun whileFalse(callback: Runnable) = whileLow(callback)

    fun hook() = hookIfNotHooked()

    /**
     * Hooks this listener to the [Scheduler] if it has not already been hooked.
     */
    private val hookIfNotHooked = runOnce {
        Scheduler.hookListener(this)
    }

    /**
     * Runs every tick to update the [conditionSED] and perform the subscribed actions.
     */
    internal fun tick() {
        conditionSED.update()

        actions.forEach { (action, condition) ->
            if (condition()) action.run()
        }
    }

    fun destroy() {
        Scheduler.unhookListener(this)
        actions.clear()
    }

    companion object {
        private val alwaysActiveListener = Listener { true }

        /**
         * Subscribes the given action to run every tick.
         * @param action The action to run.
         * @return This [Listener] instance.
         */
        @JvmStatic
        fun always(action: Runnable) {
            alwaysActiveListener.whileHigh(action)
        }
    }

    // ---------------------------------------------------------------
    // Listener builders
    // ---------------------------------------------------------------

    infix fun and(otherCondition: () -> Boolean) = Listener { condition() && otherCondition() }

    infix fun or(otherCondition: () -> Boolean) = Listener { condition() || otherCondition() }

    infix fun xor(otherCondition: () -> Boolean) = Listener { condition() xor otherCondition() }

    infix fun nand(otherCondition: () -> Boolean) = Listener { !(condition() && otherCondition()) }

    infix fun nor(otherCondition: () -> Boolean) = Listener { !(condition() || otherCondition()) }

    infix fun xnor(otherCondition: () -> Boolean) = Listener { condition() == otherCondition() }

    operator fun plus(otherCondition: () -> Boolean) = and(otherCondition)

    operator fun div(otherCondition: () -> Boolean) = or(otherCondition)


    infix fun and(other: Listener) = Listener { condition() && other.condition() }

    infix fun or(other: Listener) = Listener { condition() || other.condition() }

    infix fun xor(other: Listener) = Listener { condition() xor other.condition() }

    infix fun nand(other: Listener) = Listener { !(condition() && other.condition()) }

    infix fun nor(other: Listener) = Listener { !(condition() || other.condition()) }

    infix fun xnor(other: Listener) = Listener { condition() == other.condition() }

    operator fun plus(other: Listener) = and(other)

    operator fun div(other: Listener) = or(other)


    operator fun not() = Listener { !condition() }
}
