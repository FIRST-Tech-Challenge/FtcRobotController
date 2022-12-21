@file:Suppress("ClassName")

package ftc.rouge.blacksmith.listeners

import ftc.rouge.blacksmith.units.TimeUnit
import kotlin.properties.Delegates

/**
 * A timer that can be used to schedule actions to be performed at a specific time.
 *
 * Java usage example:
 * ```java
 * @Override
 * public void runOpMode() throws InterruptedException {
 *     Timer timer = new Timer(5, TimeUnit.SECONDS);
 *
 *     // Runs the given Runnable while waiting for the timer to finish
 *     timer.whileWaiting(this::doSomething);
 *
 *     // Runs the given Runnable once after 5 seconds
 *     timer.onDone(this::doSomethingElse);
 *
 *     // Runs the given Runnable every tick after 5 seconds, until the timer is reset
 *     timer.whileDone(this::doYetAnotherThing);
 *
 *     waitForStart();
 *
 *     // Timer should be reset before usage. Prevents the timer from running immediately.
 *     // This will primarily be used in combination with a gamepad trigger.
 *     gamepadx1.a.onHigh(timer::reset);
 *
 *     // Starts the timer
 *     Scheduler.launch(this);
 * }
 * ```
 *
 * @author KG
 *
 * @see Scheduler
 * @see Listener
 * @see ReforgedGamepad
 */
class Timer @JvmOverloads constructor(length: Long, unit: TimeUnit = TimeUnit.MILLISECONDS) {

    /**
     * The internal representation of the timer
     */
    private val listener = Listener { !isPending && System.currentTimeMillis() - startTime >= unit.toMs(length) }

    /**
     * The length of the timer in milliseconds.
     */
    private val length = unit.toMs(length)

    /**
     * The time at which the timer started.
     */
    private var startTime by Delegates.notNull<Long>()

    /**
     * Determines whether the timer is running or simply waiting to start.
     */
    private var isPending = false

    /**
     * Schedules the given action to run while the timer is running and not yet finsihed.
     * @param action The action to run.
     * @return The timer instance.
     */
    fun whileWaiting(action: Runnable) = this.also { listener.whileLow(action) }

    /**
     * Schedules the given action to run once when the timer is finished.
     * @param action The action to run.
     * @return The timer instance.
     */
    fun onDone(action: Runnable) = this.also { listener.onRise(action) }

    fun start() {
        isPending = false
        startTime = System.currentTimeMillis()
    }

    fun reset() {
        startTime = System.currentTimeMillis()
    }

    fun finishPrematurely() {
        startTime = System.currentTimeMillis() + length
    }

    fun setPending() {
        isPending = true
    }

    fun destroy() {
        listener.destroy()
    }

    fun isDone() = listener.condition()
}

class after(val time: Long) {
    fun milliseconds(callback: Runnable) = unit(TimeUnit.MILLISECONDS, callback)

    fun seconds(callback: Runnable) = unit(TimeUnit.SECONDS, callback)

    fun unit(unit: TimeUnit, callback: Runnable) = Timer(time, unit)
        .run {
            onDone(callback)
            onDone(::destroy)
            start()
        }
}
