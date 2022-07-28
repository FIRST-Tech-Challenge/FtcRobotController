package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.initializableOnce

class DriveMotors {
    var frontLeft: DcMotorEx by initializableOnce()
    var frontRight: DcMotorEx by initializableOnce()
    var backLeft: DcMotorEx by initializableOnce()
    var backRight: DcMotorEx by initializableOnce()

    fun logData(telemetry: Telemetry, dataSupplier: (DcMotorEx) -> Any) {
        telemetry.addData("Front-left motor:", dataSupplier(frontLeft))
        telemetry.addData("Front-right motor:", dataSupplier(frontRight))
        telemetry.addData("Back-left motor:", dataSupplier(backLeft))
        telemetry.addData("Back-right motor:", dataSupplier(backRight))
    }
}

fun initializedDriveMotors(hwMap: HardwareMap) = DriveMotors().apply {
    frontLeft = initializedMotor("FL", hwMap)
    frontRight = initializedMotor("FR", hwMap)
    backLeft = initializedMotor("BL", hwMap)
    backRight = initializedMotor("BR", hwMap)
}