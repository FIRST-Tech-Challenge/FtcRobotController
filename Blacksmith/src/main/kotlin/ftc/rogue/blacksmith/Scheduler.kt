package ftc.rogue.blacksmith

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rogue.blacksmith.internal.util.DoubleConsumer
import ftc.rogue.blacksmith.internal.scheduler.SchedulerInternal
import ftc.rogue.blacksmith.listeners.*

/**
 * [**LINK TO OFFICIAL DOCS (click on me) (please read) (I like cars)**](https://blacksmithftc.vercel.app/scheduler-api/overview)
 *
 * A component that simplifies the process of scheduling actions to be performed at a
 * specific time or condition, for use in a [LinearOpMode]. It seeks to transform teleop code
 * into a declarative haven, where the robot's actions are defined in a set of [actions][Runnable]
 * bound to a [condition][Condition] on which to perform it.
 *
 * Usage of this component groups all mutation of state into once central location, making it
 * infinitely easier to debug and maintain.
 *
 * For example:
 * ```java
 * while (opmode.opModeIsActive() && !opmode.isStopRequested) {
 *     openClaw();  // ?? When is this used and how do I use it ??
 * }
 *
 * // 300 billion lines later...
 *
 * private boolean gamepad1aWasFalseBefore = //...;
 *
 * private void checkIfShouldDoSomething() {
 *    if (gamepad1.a && gamepad1aWasFalseBefore) {
 *        claw.open();
 *    }
 *    gamepad1aWasFalseBefore = //...
 * } // ugh imperative hell
 *
 * // --- VS: ---
 *
 * gamepadx1.a.onRise(claw::open); // So clean and cool wow the person who made
 * Scheduler.launch(this);  // this was probably so smart and a genius I'm so jealous!!
 * ```
 *
 * The performance impact of this component is minimal, with [Listeners][Listener] being lazily
 * hooked only when required, and each tick barely encumbering the stack.
 *
 * All tasks are guaranteed to run in the order that they are scheduled. The code blocks run in
 * the following order: `beforeEach` -> `scheduled tasks` -> `block of code provided in start`
 *
 * Java usage example:
 * ```java
 * @Override
 * public void runOpMode() throws InterruptedException {
 *     // Runs this block of code while the OpMode is active,
 *     // and before each tick
 *     Scheduler.beforeEach(() -> {
 *         something = 0;
 *         doSomethingBefore();
 *     });
 *
 *     // Usage of scheduler through a more convenient method
 *     ReforgedGamepad gamepadx1 = new ReforgedGamepad(gamepad1);
 *     gamepadx1.a.onHigh(this::doSomething);
 *
 *     // Creating a listener through the raw API
 *     new Listener(() -> someConditionAbc123)
 *         .onRise(this::doSumpthinElse)
 *         .onFall(this::doYetAnotherThing);
 *
 *     // Runs the code when OpMode starts & while the OpMode is active.
 *     Scheduler.launchOnStart(this, () -> {
 *         // Optional block of code to be run after the above listeners
 *         // This parameter may be omitted if unnecessary.
 *         updateSomething();
 *         doSomethingElse();
 *         blahBlahBlah();
 *     });
 * }
 * ```
 *
 * @author KG
 *
 * @see Listener
 * @see ReforgedGamepad
 * @see Timer
 */
object Scheduler {
    const val STARTING_MSG = 2350948905823L

    @JvmStatic
    fun beforeEach(block: Runnable) {
        internal.beforeEach = block
    }

    /**
     * Starts the [Scheduler], and runs the program in the given [afterEach] until the [LinearOpMode]
     * is no longer active.
     * Java usage example:
     * ```java
     * @Override
     * public void runOpMode() throws InterruptedException {
     *    // Instantiate listeners...
     *
     *    waitForStart();
     *
     *    Scheduler.launch(this);
     *
     *    //or
     *
     *    Scheduler.launch(this, () -> {
     *        updateSomething();
     *        updateTelemetry(telemetry);
     *    });
     * }
     * ```
     * @param opmode The [LinearOpMode] to run the [Scheduler] in.
     * @param afterEach An optional block of code to run every tick, after the listeners have ran.
     */
    @JvmStatic
    @JvmOverloads
    fun launch(opmode: LinearOpMode, afterEach: Runnable = Runnable {}) {
        internal.launch(opmode, afterEach)
    }

    @JvmStatic
    @JvmOverloads
    fun launchOnStart(opmode: LinearOpMode, afterEach: Runnable = Runnable {}) {
        internal.launchOnStart(opmode, afterEach)
    }

    @JvmStatic
    inline fun launchManually(condition: () -> Boolean, afterEach: Runnable = Runnable {}) {
        internal.launchManually(condition, afterEach)
    }

    @JvmStatic
    fun time(opmode: LinearOpMode, afterEach: DoubleConsumer) {
        internal.time(opmode, afterEach)
    }

    @JvmStatic
    fun on(message: Any, callback: Runnable) {
        internal.on(message, callback)
    }

    @JvmStatic
    fun emit(message: Any) {
        internal.emit(message)
    }

    @JvmStatic
    fun nuke() {
        internal.nuke()
    }

    // -- INTERNAL --

    @JvmSynthetic
    @PublishedApi
    internal val internal = SchedulerInternal()

    @JvmSynthetic
    internal fun hookListener(listener: Listener) {
        internal.hookListener(listener)
    }

    @JvmSynthetic
    internal fun unhookListener(listener: Listener) {
        internal.unhookListener(listener)
    }
}
