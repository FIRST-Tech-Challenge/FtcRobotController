package org.firstinspires.ftc.teamcode.components.deadwheels

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.components.motors.DriveMotors
import org.firstinspires.ftc.teamcode.util.DataSupplier
import org.firstinspires.ftc.teamcode.util.LateInitVal

/**
 * The bot's drive motors logically grouped together.
 *
 * Kotlin usage example:
 * ```
 * fun main() {
 *   val motors = initializedDriveMotors(hardwareMap)
 *   val deadwheels = initializedDeadwheels(motors)
 *   deadwheels.logData(telemetry) { it.ticks }
 *   deadwheels.snapshotTicks()
 * }
 * ```
 *
 * Java usage example:
 * ```
 * public static void main(String... args) {
 *   DriveMotors motors = initializedDriveMotors(hardwareMap);
 *   Deadwheels deadwheels = DeadwheelsKt.initializedDeadwheels(motors);
 *   deadwheels.logData(telemetry, Deadwheel::getTicks);
 *   deadwheels.snapshotTicks();
 * }
 * ```
 * _Note: Both language examples produce the exact same outputs_
 *
 * @property left The left deadwheel.
 * @property right The right deadwheel.
 * @property back The back deadwheel.
 *
 * @see [initializedDeadwheels]
 * @see [DriveMotors]
 *
 * @author KG
 */
class Deadwheels {
    var left: Deadwheel by LateInitVal()
    var right: Deadwheel by LateInitVal()
    var back: Deadwheel by LateInitVal()

    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<Deadwheel>) {
        telemetry.addData("Left wheel:", dataSupplier(left))
        telemetry.addData("Right wheel:", dataSupplier(right))
        telemetry.addData("Back wheel:", dataSupplier(back))
    }

    /**
     * Snapshots the current ticks of the deadwheels. Should be called at the end of every
     * OpMode loop so that movement change can be tracked.
     */
    private object IgnoreMeIExistSoKDocsWillActuallyRenderTheDoc
    fun snapshotTicks() = this.run {
        left.snapshotTicks()
        right.snapshotTicks()
        back.snapshotTicks()
    }
}

/**
 * Initializes a [Deadwheels] object with the default configurations.
 *
 * Kotlin usage example:
 * ```
 * val Deadwheels = initializedDeadwheels(hardwareMap)
 * ```
 *
 * Java usage example:
 * ```
 * Deadwheels deadwheels = DeadwheelsKt.initializedDeadwheels(hardwareMap)
 * ```
 *
 * @param motors The [DriveMotors] object containing the corresponding motors
 * @return A [Deadwheels] object with the motors initialized initialized.`
 *
 * @author KG
 */
private object IgnoreMeIExistSoKDocsWillActuallyRenderTheDoc
fun initializedDeadwheels(motors: DriveMotors) = Deadwheels().apply {
    left = Deadwheel(motors.backRight)
    right = Deadwheel(motors.frontLeft)
    back = Deadwheel(motors.frontRight)
}