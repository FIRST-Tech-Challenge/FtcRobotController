package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util._get

typealias ZPB = DcMotor.ZeroPowerBehavior
typealias DCDirection = DcMotorSimple.Direction

@JvmOverloads
fun initializedMotor(name: String, hwMap: HardwareMap, zpb: ZPB = ZPB.BRAKE, reversed: Boolean = false): DcMotorEx {
    return hwMap
        ._get<DcMotorEx>(name)
        .apply {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = zpb
            direction = if (reversed) DCDirection.REVERSE else DCDirection.FORWARD
        }
}

fun initializedDriveMotors(hwMap: HardwareMap) = Motors().apply {
    frontLeft = initializedMotor("FL", hwMap)
    frontRight = initializedMotor("FR", hwMap)
    backLeft = initializedMotor("BL", hwMap)
    backRight = initializedMotor("BR", hwMap)
}

fun Motors.logDriveMotorData(telemetry: Telemetry, dataSupplier: (DcMotorEx) -> Any) {
    telemetry.addData("Front-left power:", dataSupplier(frontLeft))
    telemetry.addData("Front-right power:", dataSupplier(frontRight))
    telemetry.addData("Back-left power:", dataSupplier(backLeft))
    telemetry.addData("Back-right power:", dataSupplier(backRight))
}