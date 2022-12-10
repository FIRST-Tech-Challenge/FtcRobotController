@file:Suppress("PropertyName", "ObjectPropertyName", "FunctionName", "MemberVisibilityCanBePrivate", "unused")

package org.firstinspires.ftc.teamcodekt.blacksmith.listeners

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcodekt.blacksmith.Scheduler
import kotlin.math.abs

/**
 * A wrapper around the base [Gamepad] class that can create [Listeners][Listener] for each button.
 * Listeners are only hooked when it's usage is required.
 *
 * Java usage example:
 * ```java
 * @Override
 * public void runOpMode() throws InterruptedException {
 *     GamepadEx2 gamepadx1 = new GamepadEx2(gamepad1);
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
 *     Scheduler.start(this);
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
class GamepadEx2(val gamepad: Gamepad) {
    // -- START MACHINE GENERATED CODE --

    /**
     * Allows client to perform an action when the gamepad's 'a' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.a.onRise(this::doSomething);
     */
    @JvmField
    val a = Listener(gamepad::a)

    /**
     * Allows client to perform an action when the gamepad's 'b' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.b.whileLow(this::doSomething);
     */
    @JvmField
    val b = Listener(gamepad::b)

    /**
     * Allows client to perform an action when the gamepad's 'x' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.x.onFall(this::doSomething);
     */
    @JvmField
    val x = Listener(gamepad::x)

    /**
     * Allows client to perform an action when the gamepad's 'y' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.y.whileLow(this::doSomething);
     */
    @JvmField
    val y = Listener(gamepad::y)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_up' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_up.whileLow(this::doSomething);
     */
    @JvmField
    val dpad_up = Listener(gamepad::dpad_up)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_down' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_down.whileHigh(this::doSomething);
     */
    @JvmField
    val dpad_down = Listener(gamepad::dpad_down)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_left' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_left.onFall(this::doSomething);
     */
    @JvmField
    val dpad_left = Listener(gamepad::dpad_left)

    /**
     * Allows client to perform an action when the gamepad's 'dpad_right' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.dpad_right.whileLow(this::doSomething);
     */
    @JvmField
    val dpad_right = Listener(gamepad::dpad_right)

    /**
     * Allows client to perform an action when the gamepad's 'left_bumper' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.left_bumper.whileLow(this::doSomething);
     */
    @JvmField
    val left_bumper = Listener(gamepad::left_bumper)

    /**
     * Allows client to perform an action when the gamepad's 'right_bumper' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.right_bumper.whileHigh(this::doSomething);
     */
    @JvmField
    val right_bumper = Listener(gamepad::right_bumper)

    /**
     * Allows client to perform an action when the gamepad's 'left_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_x) > .5)
     * gamepad_x1.left_stick_x.whileLow(this::doSomething);
     */
    @JvmField
    val left_stick_x = left_stick_x(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'left_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_x) > the_given_deadzone)
     * gamepad_x1.left_stick_x(.1).whileLow(this::doSomething);
     * ```
     * @param deadzone The minimum value that the left_stick_x must be above to trigger the event.
     */
    fun left_stick_x(deadzone: Double): Listener {
        return Listener { abs(gamepad.left_stick_x) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'left_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_y) > .5)
     * gamepad_x1.left_stick_y.whileLow(this::doSomething);
     */
    @JvmField
    val left_stick_y = left_stick_y(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'left_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(left_stick_y) > the_given_deadzone)
     * gamepad_x1.left_stick_y(.1).whileLow(this::doSomething);
     * ```
     * @param deadzone The minimum value that the left_stick_y must be above to trigger the event.
     */
    fun left_stick_y(deadzone: Double): Listener {
        return Listener { abs(gamepad.left_stick_y) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_x) > .5)
     * gamepad_x1.right_stick_x.whileLow(this::doSomething);
     */
    @JvmField
    val right_stick_x = right_stick_x(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'right_stick_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_x) > the_given_deadzone)
     * gamepad_x1.right_stick_x(.1).whileLow(this::doSomething);
     * ```
     * @param deadzone The minimum value that the right_stick_x must be above to trigger the event.
     */
    fun right_stick_x(deadzone: Double): Listener {
        return Listener { abs(gamepad.right_stick_x) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_y) > .5)
     * gamepad_x1.right_stick_y.onRise(this::doSomething);
     */
    @JvmField
    val right_stick_y = right_stick_y(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'right_stick_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_stick_y) > the_given_deadzone)
     * gamepad_x1.right_stick_y(.1).onRise(this::doSomething);
     * ```
     * @param deadzone The minimum value that the right_stick_y must be above to trigger the event.
     */
    fun right_stick_y(deadzone: Double): Listener {
        return Listener { abs(gamepad.right_stick_y) > deadzone }
    }

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
    fun left_trigger(deadzone: Double): Listener {
        return Listener { abs(gamepad.left_trigger) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'right_trigger' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_trigger) > .5)
     * gamepad_x1.right_trigger.whileLow(this::doSomething);
     */
    @JvmField
    val right_trigger = right_trigger(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'right_trigger' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(right_trigger) > the_given_deadzone)
     * gamepad_x1.right_trigger(.1).whileLow(this::doSomething);
     * ```
     * @param deadzone The minimum value that the right_trigger must be above to trigger the event.
     */
    fun right_trigger(deadzone: Double): Listener {
        return Listener { abs(gamepad.right_trigger) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'left_stick_button' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.left_stick_button.whileLow(this::doSomething);
     */
    @JvmField
    val left_stick_button = Listener(gamepad::left_stick_button)

    /**
     * Allows client to perform an action when the gamepad's 'right_stick_button' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.right_stick_button.onFall(this::doSomething);
     */
    @JvmField
    val right_stick_button = Listener(gamepad::right_stick_button)

    /**
     * Allows client to perform an action when the gamepad's 'circle' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.circle.whileLow(this::doSomething);
     */
    @JvmField
    val circle = Listener(gamepad::circle)

    /**
     * Allows client to perform an action when the gamepad's 'cross' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.cross.whileLow(this::doSomething);
     */
    @JvmField
    val cross = Listener(gamepad::cross)

    /**
     * Allows client to perform an action when the gamepad's 'triangle' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.triangle.whileHigh(this::doSomething);
     */
    @JvmField
    val triangle = Listener(gamepad::triangle)

    /**
     * Allows client to perform an action when the gamepad's 'square' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.square.onFall(this::doSomething);
     */
    @JvmField
    val square = Listener(gamepad::square)

    /**
     * Allows client to perform an action when the gamepad's 'share' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.share.onFall(this::doSomething);
     */
    @JvmField
    val share = Listener(gamepad::share)

    /**
     * Allows client to perform an action when the gamepad's 'options' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.options.onRise(this::doSomething);
     */
    @JvmField
    val options = Listener(gamepad::options)

    /**
     * Allows client to perform an action when the gamepad's 'guide' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.guide.onFall(this::doSomething);
     */
    @JvmField
    val guide = Listener(gamepad::guide)

    /**
     * Allows client to perform an action when the gamepad's 'start' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.start.onRise(this::doSomething);
     */
    @JvmField
    val start = Listener(gamepad::start)

    /**
     * Allows client to perform an action when the gamepad's 'back' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.back.onFall(this::doSomething);
     */
    @JvmField
    val back = Listener(gamepad::back)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.touchpad.onFall(this::doSomething);
     */
    @JvmField
    val touchpad = Listener(gamepad::touchpad)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.touchpad_finger_1.whileLow(this::doSomething);
     */
    @JvmField
    val touchpad_finger_1 = Listener(gamepad::touchpad_finger_1)

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.touchpad_finger_2.onRise(this::doSomething);
     */
    @JvmField
    val touchpad_finger_2 = Listener(gamepad::touchpad_finger_2)

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
    fun touchpad_finger_1_x(deadzone: Double): Listener {
        return Listener { abs(gamepad.touchpad_finger_1_x) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_1_y) > .5)
     * gamepad_x1.touchpad_finger_1_y.whileLow(this::doSomething);
     */
    @JvmField
    val touchpad_finger_1_y = touchpad_finger_1_y(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_1_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_1_y) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_1_y(.1).whileLow(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_1_y must be above to trigger the event.
     */
    fun touchpad_finger_1_y(deadzone: Double): Listener {
        return Listener { abs(gamepad.touchpad_finger_1_y) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_x) > .5)
     * gamepad_x1.touchpad_finger_2_x.onRise(this::doSomething);
     */
    @JvmField
    val touchpad_finger_2_x = touchpad_finger_2_x(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_x' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_x) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_2_x(.1).onRise(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_2_x must be above to trigger the event.
     */
    fun touchpad_finger_2_x(deadzone: Double): Listener {
        return Listener { abs(gamepad.touchpad_finger_2_x) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_y) > .5)
     * gamepad_x1.touchpad_finger_2_y.onRise(this::doSomething);
     */
    @JvmField
    val touchpad_finger_2_y = touchpad_finger_2_y(deadzone = .5)
    
    /**
     * Allows client to perform an action when the gamepad's 'touchpad_finger_2_y' button's state is mutated.
     * ```java
     * //e.g: (Triggers when abs(touchpad_finger_2_y) > the_given_deadzone)
     * gamepad_x1.touchpad_finger_2_y(.1).onRise(this::doSomething);
     * ```
     * @param deadzone The minimum value that the touchpad_finger_2_y must be above to trigger the event.
     */
    fun touchpad_finger_2_y(deadzone: Double): Listener {
        return Listener { abs(gamepad.touchpad_finger_2_y) > deadzone }
    }

    /**
     * Allows client to perform an action when the gamepad's 'ps' button's state is mutated.
     * ```java
     * //e.g:
     * gamepad_x1.ps.onFall(this::doSomething);
     */
    @JvmField
    val ps = Listener(gamepad::ps)

    // -- END MACHINE GENERATED CODE --
}
