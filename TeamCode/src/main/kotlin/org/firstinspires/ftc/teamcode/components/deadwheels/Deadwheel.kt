@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode.components.deadwheels

import com.qualcomm.robotcore.hardware.DcMotor

/**
 * Wrapper class around a Motor to provide dead-wheel functionality.
 *
 * Kotlin usage examples:
 * ```
 * fun main() {
 *   val motor = initializedMotor("frontRight", hardwareMap)
 *   val rightDeadwheel = Deadwheel(motor)
 *   print(rightDeadwheel.ticks)
 * }
 * ```
 *
 * Java usage examples:
 * ```
 * public static void main(String... args) {
 *   DcMotor motor = initializedMotor("frontRight", hardwareMap);
 *   Deadwheel rightDeadwheel = new Deadwheel(motor);
 *   System.out.print(rightDeadwheel.getTicks());
 * }
 * ```
 *
 * _Note: Both language examples produce the exact same objects_
 *
 * @property correspondingMotor The underlying motor that the deadwheel is connected to.
 *
 * @author KG
 */
class Deadwheel(
    private val correspondingMotor: DcMotor,
) {
    val ticks
        get() = correspondingMotor.currentPosition

    var prevTicks = 0
        private set

    val offset
        get() = ticks - prevTicks

    /**
     * Snapshots the current value of `ticks`, should be called at the end of every `OpMode` loop.
     */
    fun snapshotTicks() {
        prevTicks = correspondingMotor.currentPosition
    }
}
