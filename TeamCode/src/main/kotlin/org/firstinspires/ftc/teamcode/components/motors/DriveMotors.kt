package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.DataSupplier
import org.firstinspires.ftc.teamcode.util.LateInitVal

/**
 * The bot's drive motors logically grouped together.
 *
 * Kotlin usage example:
 * ```
 * fun main() {
 *   val motors = initializedDriveMotors(hardwareMap)
 *   motors.frontLeft.power = 0.5
 *   motors.logData(telemetry) { it.power }
 * }
 * ```
 *
 * Java usage example:
 * ```
 * public static void main(String... args) {;
 *   DriveMotors motors = DriveMotorsKt.initializedDriveMotors(hardwareMap);
 *   motors.getFrontLeft().setPower(0.5);
 *   motors.logData(telemetry, DcMotorEx::getPower);
 * }
 * ```
 * _Note: Both language examples produce the exact same outputs_
 *
 * @property frontLeft The front left motor.
 * @property frontRight The front right motor.
 * @property backLeft The back left motor.
 * @property backRight The back right motor.
 *
 * @see [initializedDriveMotors]
 *
 * @author KG
 */
class DriveMotors {
    var frontLeft: DcMotorEx by LateInitVal()
    var frontRight: DcMotorEx by LateInitVal()
    var backLeft: DcMotorEx by LateInitVal()
    var backRight: DcMotorEx by LateInitVal()

    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<DcMotorEx>) {
        telemetry.addData("Front-left motor:", dataSupplier(frontLeft))
        telemetry.addData("Front-right motor:", dataSupplier(frontRight))
        telemetry.addData("Back-left motor:", dataSupplier(backLeft))
        telemetry.addData("Back-right motor:", dataSupplier(backRight))
    }
}

/**
 * Initializes a [DriveMotors] object with the default configurations.
 *
 * Kotlin usage example:
 * ```
 * val motors = initializedDriveMotors(hardwareMap)
 * ```
 *
 * Java usage example:
 * ```
 * DriveMotors motors = DriveMotorsKt.initializedDriveMotors(hardwareMap)
 * ```
 *
 * @param hwMap The [HardwareMap]
 * @return A [DriveMotors] object with the motors initialized initialized.`
 *
 * @author KG
 */
private object IgnoreMeIExistSoKDocsWillActuallyRenderTheDoc
fun initializedDriveMotors(hwMap: HardwareMap) = DriveMotors().apply {
    frontLeft = initializedMotor("FL", hwMap)
    frontRight = initializedMotor("FR", hwMap)
    backLeft = initializedMotor("BL", hwMap)
    backRight = initializedMotor("BR", hwMap)
}