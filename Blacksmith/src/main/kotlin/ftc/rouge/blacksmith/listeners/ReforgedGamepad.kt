@file:Suppress("PropertyName", "ObjectPropertyName", "FunctionName", "MemberVisibilityCanBePrivate", "unused")

package ftc.rouge.blacksmith.listeners

import com.qualcomm.robotcore.hardware.Gamepad
import ftc.rouge.blacksmith.Scheduler
import kotlin.math.abs

/**
 * A wrapper around the base [Gamepad] class that can create [Listeners][Listener] for each button.
 * Listeners are only hooked when it's usage is required.
 *
 * Java usage example:
 * ```java
 * @Override
 * public void runOpMode() throws InterruptedException {
 *     ReforgedGamepad gamepadx1 = new ReforgedGamepad(gamepad1);
 *
 *     // Use onRise and onFall to run the action once when the
 *     // condition is first true or first false.
 *     // (In this case, when the button is first pressed or released)
 *     gamepadx1.a.onRise(this::openClaw)
 *                .onFall(this::closeClaw);
 *
 *     // Use onHigh if the action should be run as long as
 *     // the condition is true, and onLow for the opposite.
 *     gamepadx1.left_trigger(.1).onHigh(this::driveSlow) // <- Notice method
 *                               .onLow(this::driveFast); // chaining allowed
 *
 *     // Gamepad buttons returning a float can be passed an option deadzone.
 *     // If called as a normal variable, it defaults to .5.
 *     // e.g.
 *     // `gamepadx1.left_trigger(.1)` triggers when abs(left_trigger) > .1
 *     // `gamepadx1.left_trigger` triggers when abs(left_trigger) > .5
 *
 *     Scheduler.launch(this);
 * }
 *
 * private void openClaw() {
 *     claw.setPosition(Claw.OPEN);
 * }
 *
 * //...
 * ```
 *
 * @author KG
 *
 * @see Gamepad
 * @see Scheduler
 * @see Listener
 */
class ReforgedGamepad(val gamepad: Gamepad) {
    // -- START MACHINE GENERATED CODE --

    // -- Main gamepad buttons --
    /**
     * Allows client to perform an action when the gamepad's 'a' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.a.whileLow(this::doSomething);
     */
    @JvmField
    val a = GamepadBooleanListener(gamepad::a)

    /**
     * Allows client to perform an action when the gamepad's 'b' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.b.onFall(this::doSomething);
     */
    @JvmField
    val b = GamepadBooleanListener(gamepad::b)

    /**
     * Allows client to perform an action when the gamepad's 'x' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.x.onFall(this::doSomething);
     */
    @JvmField
    val x = GamepadBooleanListener(gamepad::x)

    /**
     * Allows client to perform an action when the gamepad's 'y' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.y.onRise(this::doSomething);
     */
    @JvmField
    val y = GamepadBooleanListener(gamepad::y)

    // -- Dpad --
    /**
     * Allows client to perform an action when the gamepad's 'dpad_up' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_up.onFall(this::doSomething);
     */
    @JvmField
    val dpad_up = GamepadBooleanListener(gamepad::dpad_up)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_down' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_down.whileHigh(this::doSomething);
     */
    @JvmField
    val dpad_down = GamepadBooleanListener(gamepad::dpad_down)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_left' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_left.whileHigh(this::doSomething);
     */
    @JvmField
    val dpad_left = GamepadBooleanListener(gamepad::dpad_left)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_right' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_right.whileLow(this::doSomething);
     */
    @JvmField
    val dpad_right = GamepadBooleanListener(gamepad::dpad_right)

    // -- Bumpers --
    /**
     * Allows client to perform an action when the gamepad's 'left_bumper' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.left_bumper.whileLow(this::doSomething);
     */
    @JvmField
    val left_bumper = GamepadBooleanListener(gamepad::left_bumper)

    /**
     * Allows client to perform an action when the gamepad's 'right_bumper' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.right_bumper.onFall(this::doSomething);
     */
    @JvmField
    val right_bumper = GamepadBooleanListener(gamepad::right_bumper)

    // -- Joysticks --
    /**
     * Allows client to perform an action when the gamepad's 'left_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_x) > .5)
     * gamepad_x1.left_stick_x.onFall(this::doSomething);
     */
    @JvmField
    val left_stick_x = left_stick_x(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'left_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_x) > the_given_deadzone)
     * gamepad_x1.left_stick_x(.1).onFall(this::doSomething);
     * ```
     * @param deadzone The minimum value that the left_stick_x must be above to trigger the event.
     */
    fun left_stick_x(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::left_stick_x)
    }

    /**
     * Allows client to perform an action when the gamepad's 'left_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_y) > .5)
     * gamepad_x1.left_stick_y.whileHigh(this::doSomething);
     */
    @JvmField
    val left_stick_y = left_stick_y(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'left_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_y) > the_given_deadzone)
     * gamepad_x1.left_stick_y(.1).whileHigh(this::doSomething);
     * ```
     * @param deadzone The minimum value that the left_stick_y must be above to trigger the event.
     */
    fun left_stick_y(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::left_stick_y)
    }

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_x) > .5)
     * gamepad_x1.right_stick_x.whileHigh(this::doSomething);
     */
    @JvmField
    val right_stick_x = right_stick_x(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_x) > the_given_deadzone)
     * gamepad_x1.right_stick_x(.1).whileHigh(this::doSomething);
     * ```
     * @param deadzone The minimum value that the right_stick_x must be above to trigger the event.
     */
    fun right_stick_x(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::right_stick_x)
    }

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_y) > .5)
     * gamepad_x1.right_stick_y.whileLow(this::doSomething);
     */
    @JvmField
    val right_stick_y = right_stick_y(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_y) > the_given_deadzone)
     * gamepad_x1.right_stick_y(.1).whileLow(this::doSomething);
     * ```
     * @param deadzone The minimum value that the right_stick_y must be above to trigger the event.
     */
    fun right_stick_y(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::right_stick_y)
    }

    // -- Triggers --
    /**
     * Allows client to perform an action when the gamepad's 'left_trigger' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_trigger) > .5)
     * gamepad_x1.left_trigger.whileHigh(this::doSomething);
     */
    @JvmField
    val left_trigger = left_trigger(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'left_trigger' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_trigger) > the_given_deadzone)
     * gamepad_x1.left_trigger(.1).whileHigh(this::doSomething);
     * ```
     * @param deadzone The minimum value that the left_trigger must be above to trigger the event.
     */
    fun left_trigger(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::left_trigger)
    }

    /**
     * Allows client to perform an action when the gamepad's 'right_trigger' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_trigger) > .5)
     * gamepad_x1.right_trigger.onFall(this::doSomething);
     */
    @JvmField
    val right_trigger = right_trigger(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'right_trigger' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_trigger) > the_given_deadzone)
     * gamepad_x1.right_trigger(.1).onFall(this::doSomething);
     * ```
     * @param deadzone The minimum value that the right_trigger must be above to trigger the event.
     */
    fun right_trigger(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::right_trigger)
    }

    // -- Joystick buttons --
    /**
     * Allows client to perform an action when the gamepad's 'left_stick_button' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.left_stick_button.onFall(this::doSomething);
     */
    @JvmField
    val left_stick_button = GamepadBooleanListener(gamepad::left_stick_button)

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_button' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.right_stick_button.onRise(this::doSomething);
     */
    @JvmField
    val right_stick_button = GamepadBooleanListener(gamepad::right_stick_button)

    // -- PS4 controller buttons --
    /**
     * Allows client to perform an action when the gamepad's 'circle' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.circle.whileLow(this::doSomething);
     */
    @JvmField
    val circle = GamepadBooleanListener(gamepad::circle)

    /**
     * Allows client to perform an action when the gamepad's 'cross' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.cross.onRise(this::doSomething);
     */
    @JvmField
    val cross = GamepadBooleanListener(gamepad::cross)

    /**
     * Allows client to perform an action when the gamepad's 'triangle' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.triangle.whileLow(this::doSomething);
     */
    @JvmField
    val triangle = GamepadBooleanListener(gamepad::triangle)

    /**
     * Allows client to perform an action when the gamepad's 'square' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.square.onRise(this::doSomething);
     */
    @JvmField
    val square = GamepadBooleanListener(gamepad::square)

    // -- Random buttons --
    /**
     * Allows client to perform an action when the gamepad's 'share' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.share.onFall(this::doSomething);
     */
    @JvmField
    val share = GamepadBooleanListener(gamepad::share)

    /**
     * Allows client to perform an action when the gamepad's 'options' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.options.onRise(this::doSomething);
     */
    @JvmField
    val options = GamepadBooleanListener(gamepad::options)

    /**
     * Allows client to perform an action when the gamepad's 'guide' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.guide.onFall(this::doSomething);
     */
    @JvmField
    val guide = GamepadBooleanListener(gamepad::guide)

    /**
     * Allows client to perform an action when the gamepad's 'start' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.start.onFall(this::doSomething);
     */
    @JvmField
    val start = GamepadBooleanListener(gamepad::start)

    /**
     * Allows client to perform an action when the gamepad's 'back' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.back.onRise(this::doSomething);
     */
    @JvmField
    val back = GamepadBooleanListener(gamepad::back)

    // -- Touchpad buttons & triggers --
    /**
     * Allows client to perform an action when the gamepad's 'touchpad' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.touchpad.onRise(this::doSomething);
     */
    @JvmField
    val touchpad = GamepadBooleanListener(gamepad::touchpad)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.touchpad_finger_1.whileLow(this::doSomething);
     */
    @JvmField
    val touchpad_finger_1 = GamepadBooleanListener(gamepad::touchpad_finger_1)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.touchpad_finger_2.onRise(this::doSomething);
     */
    @JvmField
    val touchpad_finger_2 = GamepadBooleanListener(gamepad::touchpad_finger_2)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_1_x) > .5)
     * gamepad_x1.touchpad_finger_1_x.onRise(this::doSomething);
     */
    @JvmField
    val touchpad_finger_1_x = touchpad_finger_1_x(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_1_x) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_1_x(.1).onRise(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_1_x must be above to trigger the event.
     */
    fun touchpad_finger_1_x(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::touchpad_finger_1_x)
    }

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_1_y) > .5)
     * gamepad_x1.touchpad_finger_1_y.onFall(this::doSomething);
     */
    @JvmField
    val touchpad_finger_1_y = touchpad_finger_1_y(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_1_y) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_1_y(.1).onFall(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_1_y must be above to trigger the event.
     */
    fun touchpad_finger_1_y(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::touchpad_finger_1_y)
    }

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_x) > .5)
     * gamepad_x1.touchpad_finger_2_x.whileHigh(this::doSomething);
     */
    @JvmField
    val touchpad_finger_2_x = touchpad_finger_2_x(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_x) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_2_x(.1).whileHigh(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_2_x must be above to trigger the event.
     */
    fun touchpad_finger_2_x(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::touchpad_finger_2_x)
    }

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_y) > .5)
     * gamepad_x1.touchpad_finger_2_y.whileHigh(this::doSomething);
     */
    @JvmField
    val touchpad_finger_2_y = touchpad_finger_2_y(deadzone = .5)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_y) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_2_y(.1).whileHigh(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_2_y must be above to trigger the event.
     */
    fun touchpad_finger_2_y(deadzone: Double): GamepadAnalogueListener {
        return GamepadAnalogueListener(deadzone, gamepad::touchpad_finger_2_y)
    }

    // -- Whatever this is --
    /**
     * Allows client to perform an action when the gamepad's 'ps' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.ps.onRise(this::doSomething);
     */
    @JvmField
    val ps = GamepadBooleanListener(gamepad::ps)

    // -- END MACHINE GENERATED CODE --

    class GamepadBooleanListener internal constructor(val input: () -> Boolean) : Listener(input) {
        fun get() = input()

        @JvmSynthetic
        operator fun invoke() = input()
    }

    class GamepadAnalogueListener internal constructor(deadzone: Double, val input: () -> Float) : Listener({ abs(input()) > deadzone }) {
        fun get() = input()

        @JvmSynthetic
        operator fun invoke() = input()
    }
}
