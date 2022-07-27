package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util._get

typealias ZPB = DcMotor.ZeroPowerBehavior
typealias DCDirection = DcMotorSimple.Direction
typealias DCMode = DcMotor.RunMode

@JvmOverloads
fun initializedMotor(
    name: String,
    hwMap: HardwareMap,
    zpb: ZPB = ZPB.BRAKE,
    reversed: Boolean = false,
    runMode: DCMode = DCMode.RUN_WITHOUT_ENCODER
): DcMotorEx {
    return hwMap
        ._get<DcMotorEx>(name)
        .apply {
            mode = runMode
            zeroPowerBehavior = zpb
            direction = if (reversed) DCDirection.REVERSE else DCDirection.FORWARD
        }
}

fun initializedMotors(hwMap: HardwareMap) = Motors().apply {
    frontLeft = initializedMotor("FL", hwMap)
    frontRight = initializedMotor("FR", hwMap)
    backLeft = initializedMotor("BL", hwMap)
    backRight = initializedMotor("BR", hwMap)
    shooter = initializedMotor("SH", hwMap, zpb = ZPB.FLOAT, runMode = DCMode.RUN_USING_ENCODER)
}

fun Motors.logDriveMotorData(telemetry: Telemetry, dataSupplier: (DcMotorEx) -> Any) {
    telemetry.addData("Front-left motor:", dataSupplier(frontLeft))
    telemetry.addData("Front-right motor:", dataSupplier(frontRight))
    telemetry.addData("Back-left motor:", dataSupplier(backLeft))
    telemetry.addData("Back-right motor:", dataSupplier(backRight))
}

fun Motors.logShooterData(telemetry: Telemetry, dataSupplier: (DcMotorEx) -> Any) {
    telemetry.addData("Shooter motor:", dataSupplier(shooter))
}