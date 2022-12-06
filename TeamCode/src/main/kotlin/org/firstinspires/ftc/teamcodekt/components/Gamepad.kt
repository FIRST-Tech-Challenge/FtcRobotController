package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.abs

/**
 * Utility function simply to determine of either of the Game-pad's joysticks are triggered.
 * Triggered in this context is defined as if any joystick has moved greater than the given
 * threshold value.
 *
 * Kotlin usage examples:
 * ```
 * override fun loop() {
 *   gamepad1.isJoystickTriggered()
 *   gamepad1.isJoystickTriggered(.01)
 *   gamepad1.isJoystickTriggered(threshold = .3)
 * }
 * ```
 *
 * Java usage examples:
 * ```java
 * @Override
 * public void loop() {
 *   _GamepadKt.isJoystickTriggered(gamepad1);
 *   _GamepadKt.isJoystickTriggered(gamepad1, .3);
 * }
 * ```
 *
 * @param deadzone Minimum joystick offset to be considered "triggered"; defaults to `0.05`
 *
 * @return `true` if any of the joysticks are triggered
 */
@JvmOverloads
fun Gamepad.isAnyJoystickTriggered(deadzone: Double = .05) =
    listOf(left_stick_y, left_stick_x, right_stick_x, right_stick_y)
        .any { abs(it) > deadzone }

/**
 * __Note: This is intended only for Kotlin use__
 *
 * Returns a list of the three used joysticks to destructure for easy initialization.
 *
 * Kotlin usage examples:
 * ```
 * override fun loop() {
 *   val (speed, strafe, turn) = gamepad1.getDriveSticks()
 * }
 * ```
 *
 * @return a list of (-ly, lx, and rx)
 */
fun Gamepad.getDriveSticks() = listOf(left_stick_x, -left_stick_y, right_stick_x)