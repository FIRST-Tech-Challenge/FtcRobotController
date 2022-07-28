package org.firstinspires.ftc.teamcode.util

/**
 * An alias (a.k.a nickname) for a lamda (nameless function) that passes a Component as a parameter.
 * Intended for telemetry functions where Component is the part (e.g. a
 * [DcMotorEx][com.qualcomm.robotcore.hardware.DcMotorEx]) given to the lambda, and the output is
 * the desired data to be logged to telemetry (e.g.
 * [DcMotorEx.power][com.qualcomm.robotcore.hardware.DcMotorEx.getPower]).
 *
 * Essentially a macro; `DataSupplier<Component>` expands to `(Component) -> Any` in the actual code
 *
 * Example of usage; probably will not be used in your own code
 * ```
 * fun DriveMotors.logDriveMotorData(dataSupplier: DataSupplier<DcMotorEx>) {
 *   telemetry.addData("Front-left motor:", dataSupplier(frontLeft))
 *   telemetry.addData("Front-right motor:", dataSupplier(frontRight))
 *   //...etc
 * }
 *
 * fun main() {
 *   val driveMotors = //...
 *   logDriveMotorData(driveMotors) { it.power } // logs the drive motors' powers.
 *   logDriveMotorData(driveMotors) { motor -> motor.velocity } // logs velocities.
 *   // ^ either way works, though 'it' is more idiomatic for single param lambdas
 * }
 * ```
 *
 * __Honestly, just don't worry about this. I just added it to make some of the code cleaner and
 * less repetitive__
 *
 * @param Component The part being passed into the data supplier function
 */
typealias DataSupplier<Component> = (Component) -> Any