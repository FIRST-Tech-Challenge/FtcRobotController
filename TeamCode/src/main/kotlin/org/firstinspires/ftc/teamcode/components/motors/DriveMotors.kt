package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.DataSupplier
import org.firstinspires.ftc.teamcode.util.LateInitVal

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

fun initializedDriveMotors(hwMap: HardwareMap) = DriveMotors().apply {
    frontLeft = initializedMotor("FL", hwMap)
    frontRight = initializedMotor("FR", hwMap)
    backLeft = initializedMotor("BL", hwMap)
    backRight = initializedMotor("BR", hwMap)
}